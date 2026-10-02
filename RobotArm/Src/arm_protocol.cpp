//
// arm_protocol.cpp — UART1 上位机命令通道实现
//
// 接收路径：UART1 中断(ReceiveToIdle_IT) -> HAL_UARTEx_RxEventCallback
//   -> UartProtocol 解析 -> onFrame（中断上下文，仅入队+刷新链路计时）
// 执行路径：ArmProtocol_Tick（任务上下文）出队 -> 调用 ArmController
// 发送路径：HAL_UART_Transmit_IT + 忙标志；遥测按轮转每次至多一帧。
//

#include "../Inc/arm_protocol.h"

#include <cstring>

#include "../Inc/arm_config.h"
#include "../Inc/arm_controller.h"
#include "usart.h"
#include "usart_protocol.h"

// ---- 调试变量（Live Watch） ----
volatile uint32_t arm_proto_rx_frames = 0;    // 累计收到有效帧
volatile uint32_t arm_proto_tx_frames = 0;    // 累计发出帧
volatile uint32_t arm_proto_tx_dropped = 0;   // 因发送忙丢弃的帧
volatile uint32_t arm_proto_link_timeouts = 0;  // 链路超时失能次数
volatile uint8_t  arm_proto_last_cmd = 0;     // 最近执行的命令类型
volatile uint32_t arm_proto_rx_rearms = 0;    // 接收异常后重新武装次数
volatile uint32_t arm_proto_last_rx_age = 0;  // 最近一帧距今 ms（链路健康度）

namespace {

// ---- 帧类型（与 arm_protocol.h 注释一致） ----
constexpr uint8_t kRxSetEnable   = 0x01;
constexpr uint8_t kRxMoveJ       = 0x02;
constexpr uint8_t kRxMoveJQueue  = 0x03;
constexpr uint8_t kRxMoveCart    = 0x04;
constexpr uint8_t kRxJog         = 0x05;
constexpr uint8_t kRxGripper     = 0x06;
constexpr uint8_t kRxHeartbeat   = 0x07;
constexpr uint8_t kRxQuery       = 0x08;

constexpr uint8_t kTxAck    = 0x80;
constexpr uint8_t kTxStatus = 0x81;
constexpr uint8_t kTxJoints = 0x82;
constexpr uint8_t kTxPose   = 0x83;

// ACK 附加状态码（0~5 复用 arm::RequestStatus）
constexpr uint8_t kAckEnableFailed = 6;

// ---- 命令队列（中断生产 / 任务消费，单生产单消费无锁） ----
struct Cmd {
    uint8_t type;
    uint8_t len;
    uint8_t payload[26];  // 帧载荷上限 = FRAME_MAX_LEN-5
};

constexpr std::size_t kCmdQueueDepth = 8;
Cmd cmd_queue_[kCmdQueueDepth];
volatile uint8_t q_head_ = 0;
volatile uint8_t q_tail_ = 0;

volatile uint32_t last_rx_ms_ = 0;

uint8_t rx_buf_[32];        // ReceiveToIdle 接收缓冲
uint8_t tx_buf_[40];        // 发送帧缓冲
volatile bool tx_busy_ = false;

UartProtocol* proto_ = nullptr;

uint32_t last_telemetry_ms_ = 0;
uint8_t telemetry_phase_ = 0;  // 轮转: 0=STATUS 1=JOINTS 2=POSE

float readF32(const uint8_t* p) {
    float f;
    std::memcpy(&f, p, 4);
    return f;
}

void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    if (tx_busy_) {
        ++arm_proto_tx_dropped;  // 下一轮遥测会重发，命令回执偶尔丢可由 QUERY 补
        return;
    }
    const uint8_t n = proto_->buildFrame(tx_buf_, type, payload, len);
    if (n == 0) return;
    tx_busy_ = true;
    if (HAL_UART_Transmit_IT(&huart1, tx_buf_, n) != HAL_OK) {
        tx_busy_ = false;
        ++arm_proto_tx_dropped;
        return;
    }
    ++arm_proto_tx_frames;
}

void sendAck(uint8_t cmd, uint8_t status) {
    const uint8_t p[2] = {cmd, status};
    sendFrame(kTxAck, p, 2);
}

void sendStatus() {
    // [0]state [1]request [2:4]fault [4:8]track [8:12]prog
    // [12]motor_err_mask [13]fb_missing_mask
    uint8_t p[14];
    p[0] = static_cast<uint8_t>(arm::armController.state());
    p[1] = arm::arm_fw_request;
    const uint16_t fault = static_cast<uint16_t>(arm::armController.fault());
    p[2] = static_cast<uint8_t>(fault & 0xFF);
    p[3] = static_cast<uint8_t>(fault >> 8);
    const float te = arm::armController.trackingError();
    const float pr = arm::armController.trajProgress();
    std::memcpy(p + 4, &te, 4);
    std::memcpy(p + 8, &pr, 4);
    p[12] = arm::arm_fw_motor_err_mask;
    p[13] = arm::arm_fw_fb_missing_mask;
    sendFrame(kTxStatus, p, sizeof(p));
}

void sendJoints() {
    float q[arm::config::kJointCount];
    arm::armController.getCurrentJoints(q);
    uint8_t p[24];
    std::memcpy(p, q, sizeof(p));
    sendFrame(kTxJoints, p, sizeof(p));
}

void sendPose() {
    arm::Pose pose;
    if (!arm::armController.getCurrentPose(pose)) return;  // 未标定不发
    const float f[6] = {pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw};
    uint8_t p[24];
    std::memcpy(p, f, sizeof(p));
    sendFrame(kTxPose, p, sizeof(p));
}

// UartProtocol 回调（UART 中断上下文）：只入队，绝不直接调控制器
void onFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    last_rx_ms_ = HAL_GetTick();
    ++arm_proto_rx_frames;
    if (type == kRxHeartbeat) return;  // 心跳不入队

    const uint8_t next = static_cast<uint8_t>((q_head_ + 1) % kCmdQueueDepth);
    if (next == q_tail_) return;  // 队满丢弃（ACK 不会发，上位机会重试）
    Cmd& c = cmd_queue_[q_head_];
    c.type = type;
    c.len = len < sizeof(c.payload) ? len : sizeof(c.payload);
    std::memcpy(c.payload, payload, c.len);
    q_head_ = next;
}

void execCmd(const Cmd& c) {
    using arm::armController;
    using arm::RequestStatus;
    const uint32_t now = HAL_GetTick();
    arm_proto_last_cmd = c.type;

    switch (c.type) {
        case kRxSetEnable: {
            if (c.len < 1) break;
            if (c.payload[0] != 0U) {
                sendAck(c.type, armController.enable(now) ? static_cast<uint8_t>(RequestStatus::Ok)
                                                          : kAckEnableFailed);
            } else {
                armController.disable(now);
                sendAck(c.type, static_cast<uint8_t>(RequestStatus::Ok));
            }
            return;
        }

        case kRxMoveJ:
        case kRxMoveJQueue: {
            if (c.len < 24) break;
            float q[arm::config::kJointCount];
            for (std::size_t i = 0; i < arm::config::kJointCount; ++i) {
                q[i] = readF32(c.payload + i * 4);
            }
            const RequestStatus st = (c.type == kRxMoveJ)
                                         ? armController.moveJ(q, now)
                                         : armController.moveJQueue(q, now);
            sendAck(c.type, static_cast<uint8_t>(st));
            return;
        }

        case kRxMoveCart: {
            if (c.len < 24) break;
            const arm::Pose pose = {
                readF32(c.payload + 0), readF32(c.payload + 4), readF32(c.payload + 8),
                readF32(c.payload + 12), readF32(c.payload + 16), readF32(c.payload + 20)};
            const RequestStatus st = armController.moveCartesian(pose, now);
            sendAck(c.type, static_cast<uint8_t>(st));
            return;
        }

        case kRxJog: {
            if (c.len < 5) break;
            const RequestStatus st = armController.jog(c.payload[0],
                                                       readF32(c.payload + 1), now);
            sendAck(c.type, static_cast<uint8_t>(st));
            return;
        }

        case kRxGripper: {
            if (c.len < 1) break;
            armController.setGripper(c.payload[0] != 0U);
            sendAck(c.type, static_cast<uint8_t>(RequestStatus::Ok));
            return;
        }

        case kRxQuery: {
            // QUERY 优先发遥测；ACK 放最后（受发送忙限制可能丢，可接受）
            sendStatus();
            sendJoints();
            sendPose();
            sendAck(c.type, static_cast<uint8_t>(RequestStatus::Ok));
            return;
        }

        default:
            sendAck(c.type, static_cast<uint8_t>(RequestStatus::InvalidTarget));
            return;
    }

    // 载荷长度不足
    sendAck(c.type, static_cast<uint8_t>(RequestStatus::InvalidTarget));
}

}  // namespace

// HAL 接收回调（弱符号覆盖；本工程仅 UART1 使用 HAL UART 接收）
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if (huart->Instance == USART1 && proto_ != nullptr) {
        for (uint16_t i = 0; i < size; ++i) {
            proto_->input(rx_buf_[i]);
        }
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf_, sizeof(rx_buf_));
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        tx_busy_ = false;
    }
}

// UART 出错（溢出/帧错误等）时重新武装接收，保证链路不断
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1 && proto_ != nullptr) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf_, sizeof(rx_buf_)) == HAL_OK) {
            ++arm_proto_rx_rearms;
        }
    }
}

extern "C" void ArmProtocol_Init(void) {
    static UartProtocol proto(onFrame);
    proto_ = &proto;

    q_head_ = 0;
    q_tail_ = 0;
    last_rx_ms_ = HAL_GetTick();
    tx_busy_ = false;
    telemetry_phase_ = 0;
    last_telemetry_ms_ = HAL_GetTick();

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf_, sizeof(rx_buf_));
}

extern "C" void ArmProtocol_Tick(uint32_t now) {
    // 0. 接收看门狗：若接收不在忙态（出错后 HAL 中止且重武装失败会静默死亡），
    //    在任务上下文兜底重新武装
    if (proto_ != nullptr && huart1.RxState != HAL_UART_STATE_BUSY_RX) {
        if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf_, sizeof(rx_buf_)) == HAL_OK) {
            ++arm_proto_rx_rearms;
        }
    }
    arm_proto_last_rx_age = now - last_rx_ms_;

    // 1. 执行命令队列（任务上下文）
    while (q_tail_ != q_head_) {
        execCmd(cmd_queue_[q_tail_]);
        q_tail_ = static_cast<uint8_t>((q_tail_ + 1) % kCmdQueueDepth);
    }
    // （链路超时失能保护已按用户要求移除，2026-10-02）

    // 3. 遥测：轮转每周期至多发一帧（STATUS / JOINTS / POSE）
    if ((now - last_telemetry_ms_) >= arm::config::kUartTelemetryMs) {
        last_telemetry_ms_ = now;
        switch (telemetry_phase_) {
            case 0:
                sendStatus();
                break;
            case 1:
                sendJoints();
                break;
            default:
                sendPose();
                break;
        }
        telemetry_phase_ = static_cast<uint8_t>((telemetry_phase_ + 1) % 3);
    }
}
