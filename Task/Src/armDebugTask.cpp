//
// armDebugTask.cpp — 机械臂标定/调试模式：持续失能并读取电机位置
//
// 用途：零位标定（Doc/机械臂控制框架说明.md 第 4.1/4.2 节）时持续观察
// 六台达妙电机的位置。标定流程：把机械臂摆到 CAD 零位姿态 -> 手拖各关节
// （电机全程失能）-> 读 arm_dbg_motor_pos 填 kJoints[].zero_offset ->
// 填好后 arm_dbg_kin_q 应显示 ~0。
//
// 行为：
//  - 上电等 2s（达妙初始化），先对六台电机各发一次失能命令；
//  - 之后每 100ms 逐台发送 0x7FF/0xCC 状态刷新请求（不改变使能状态，
//    电机保持失能可自由拖动；旧固件无刷新应答时回退发失能命令取反馈，
//    同样保持失能）；
//  - 数据通过两条通道输出：
//      1) volatile 调试变量（调试器 Live Watch，工程惯用方式）
//      2) USB 虚拟串口文本，500ms 一行（任意串口助手打开对应 COM 口即可）
//
// !!! 与 MotorTask / ArmTask 驱动同一组电机，三者互斥：
//     在 freertos.c 里切换任务启动，标定完记得切回 !!!
//
// 观察变量（Live Watch）：
//   arm_dbg_motor_pos[6]  电机反馈位置 rad（电机原始坐标系；填 zero_offset 看这个）
//   arm_dbg_kin_q[6]      运动学关节角（已套用 arm_config 零位/方向/差速换算）
//   arm_dbg_motor_err[6]  达妙状态码（0失能 1使能 8~E故障）
//   arm_dbg_rx_count[6]   各电机累计应答帧数（哪台不回话看这个）
//   arm_dbg_stale_mask    bit i = 第 i 台本轮无应答
//   arm_dbg_cycles        已完成的轮询轮数
//

#include "../Inc/armDebugTask.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "arm_config.h"
#include "arm_hal.h"
#include "bsp_can.h"
#include "main.h"
#include "usbd_cdc_if.h"

// 定义在 bsp_can.cpp
extern DM_motor_info DM_motor_J0;
extern DM_motor_info DM_motor_J1;
extern DM_motor_info DM_motor_J2;
extern DM_motor_info DM_motor_J3;
extern DM_motor_info DM_motor_J4;
extern DM_motor_info DM_motor_J5;

// ---- 调试变量（volatile，调试器直接观察） ----
volatile float    arm_dbg_motor_pos[arm::config::kJointCount] = {0};
volatile float    arm_dbg_kin_q[arm::config::kJointCount] = {0};
volatile uint8_t  arm_dbg_motor_err[arm::config::kJointCount] = {0};
volatile uint32_t arm_dbg_rx_count[arm::config::kJointCount] = {0};
volatile uint32_t arm_dbg_stale_mask = 0;
volatile uint32_t arm_dbg_cycles = 0;

// ---- 标量镜像（Live Watch 直接逐个添加监视，无需展开数组） ----
volatile float arm_dbg_q0 = 0, arm_dbg_q1 = 0, arm_dbg_q2 = 0;
volatile float arm_dbg_q3 = 0, arm_dbg_q4 = 0, arm_dbg_q5 = 0;
volatile float arm_dbg_dm4 = 0;          // 0x06 电机相对腕零位 Δm4
volatile float arm_dbg_dm5 = 0;          // 0x07 电机相对腕零位 Δm5
volatile float arm_dbg_wrist_common = 0; // Δm4+Δm5 共模（roll 方向）
volatile float arm_dbg_wrist_diff = 0;   // Δm4-Δm5 差模（pitch 方向）

namespace {

DM_motor_info* const kMotors[arm::config::kJointCount] = {
    &DM_motor_J0, &DM_motor_J1, &DM_motor_J2,
    &DM_motor_J3, &DM_motor_J4, &DM_motor_J5
};

constexpr uint32_t kPollPeriodMs   = 100;  // 全关节轮询周期
constexpr uint32_t kReplyWaitMs    = 10;   // 单台应答等待
constexpr uint32_t kUsbPeriodMs    = 500;  // 串口输出周期
constexpr uint32_t kFrameGapMs     = 2;    // 相邻 CAN 帧间隔（沿用现有经验值）
constexpr bool     kUsbTextOutput  = true; // false = 关闭串口文本输出

constexpr uint16_t kMotorCanId(std::size_t i) {
    return arm::config::kJoints[i].can_id;
}

bsp_can& canBus() {
    static bsp_can instance;
    return instance;
}

bool tickReached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

uint32_t snapshotRx(std::size_t i) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t rx = kMotors[i]->rx_count;
    if (primask == 0U) __enable_irq();
    return rx;
}

// 等待该电机收到新反馈帧（rx_count 增加）
bool awaitNewFrame(std::size_t i, uint32_t rx_before) {
    const uint32_t deadline = HAL_GetTick() + kReplyWaitMs;
    while (!tickReached(HAL_GetTick(), deadline)) {
        if (snapshotRx(i) != rx_before) return true;
        osDelay(1);
    }
    return snapshotRx(i) != rx_before;
}

// 手写 +X.XXX 定点格式化，避免依赖 newlib-nano 的浮点 printf
int fmtFloat(char* dst, std::size_t cap, float v) {
    if (!std::isfinite(v)) {
        return snprintf(dst, cap, " ????");
    }
    const float a = v < 0.0f ? -v : v;
    uint32_t ip = static_cast<uint32_t>(a);
    uint32_t fr = static_cast<uint32_t>((a - static_cast<float>(ip)) * 1000.0f + 0.5f);
    if (fr >= 1000U) {
        ++ip;
        fr -= 1000U;
    }
    return snprintf(dst, cap, " %c%u.%03u", v < 0.0f ? '-' : '+',
                    static_cast<unsigned>(ip), static_cast<unsigned>(fr));
}

}  // namespace

void ArmDebugTask::run() {
    osDelay(arm::config::kMotorStartupDelayMs);

    // 确保全部失能（达妙失能命令同时会触发一帧反馈）
    for (std::size_t i = 0; i < arm::config::kJointCount; ++i) {
        canBus().BSP_CAN1_DMMotorDisableCmd(kMotorCanId(i), DM_POS_MODE);
        osDelay(kFrameGapMs);
    }

    uint32_t next_poll = HAL_GetTick();
    uint32_t next_usb = HAL_GetTick() + kUsbPeriodMs;

    for (;;) {
        const uint32_t now = HAL_GetTick();
        if (!tickReached(now, next_poll)) {
            osDelay(1);
            continue;
        }
        next_poll += kPollPeriodMs;
        if (static_cast<int32_t>(now - next_poll) >= static_cast<int32_t>(kPollPeriodMs)) {
            next_poll = now + kPollPeriodMs;  // 掉帧过多则重新对齐
        }

        // ---- 逐台请求反馈（刷新请求优先，旧固件回退失能命令；均保持失能） ----
        uint32_t stale = 0;
        for (std::size_t i = 0; i < arm::config::kJointCount; ++i) {
            const uint32_t rx_before = snapshotRx(i);
            bool ok = false;
            if (canBus().BSP_CAN1_DMMotorRefreshStatusCmd(kMotorCanId(i)) == HAL_OK) {
                ok = awaitNewFrame(i, rx_before);
            }
            if (!ok) {
                const uint32_t rx_retry = snapshotRx(i);
                if (canBus().BSP_CAN1_DMMotorDisableCmd(kMotorCanId(i), DM_POS_MODE) == HAL_OK) {
                    ok = awaitNewFrame(i, rx_retry);
                }
            }
            if (!ok) stale |= (1UL << i);
        }
        arm_dbg_stale_mask = stale;

        // ---- 关中断快照并换算运动学角 ----
        float pos[arm::config::kJointCount];
        uint8_t err[arm::config::kJointCount];
        {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            __DMB();
            for (std::size_t i = 0; i < arm::config::kJointCount; ++i) {
                pos[i] = kMotors[i]->pos;
                err[i] = kMotors[i]->err;
                arm_dbg_rx_count[i] = kMotors[i]->rx_count;
            }
            __DMB();
            if (primask == 0U) __enable_irq();
        }
        float kin[arm::config::kJointCount];
        arm::hal::motorToKin(pos, kin);
        for (std::size_t i = 0; i < arm::config::kJointCount; ++i) {
            arm_dbg_motor_pos[i] = pos[i];
            arm_dbg_motor_err[i] = err[i];
            arm_dbg_kin_q[i] = kin[i];
        }
        arm_dbg_q0 = kin[0];
        arm_dbg_q1 = kin[1];
        arm_dbg_q2 = kin[2];
        arm_dbg_q3 = kin[3];
        arm_dbg_q4 = kin[4];
        arm_dbg_q5 = kin[5];
        arm_dbg_dm4 = pos[4] - arm::config::kWristM4Zero;
        arm_dbg_dm5 = pos[5] - arm::config::kWristM5Zero;
        arm_dbg_wrist_common = arm_dbg_dm4 + arm_dbg_dm5;
        arm_dbg_wrist_diff = arm_dbg_dm4 - arm_dbg_dm5;
        ++arm_dbg_cycles;

        // ---- USB 虚拟串口文本输出 ----
        if (kUsbTextOutput && tickReached(now, next_usb)) {
            next_usb = now + kUsbPeriodMs;

            char line[192];
            int n = snprintf(line, sizeof(line), "[dbg] m:");
            for (std::size_t i = 0; i < arm::config::kJointCount && n > 0 && n < (int)sizeof(line); ++i) {
                n += fmtFloat(line + n, sizeof(line) - n, pos[i]);
            }
            n += snprintf(line + n, sizeof(line) - n, " q:");
            for (std::size_t i = 0; i < arm::config::kJointCount && n > 0 && n < (int)sizeof(line); ++i) {
                n += fmtFloat(line + n, sizeof(line) - n, kin[i]);
            }
            n += snprintf(line + n, sizeof(line) - n, " stale:%lu cyc:%lu\r\n",
                          static_cast<unsigned long>(stale),
                          static_cast<unsigned long>(arm_dbg_cycles));
            if (n > 0) {
                const uint16_t len = n < static_cast<int>(sizeof(line))
                                         ? static_cast<uint16_t>(n)
                                         : static_cast<uint16_t>(sizeof(line) - 1);
                CDC_Transmit_FS(reinterpret_cast<uint8_t*>(line), len);
            }
        }
    }
}

extern "C" {

static ArmDebugTask armDebugTask;

void ArmDebugTask_Init() {
    armDebugTask.start((char*)"ArmDbg", 1536, osPriorityNormal);
}

}
