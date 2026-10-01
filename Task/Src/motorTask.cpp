//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "dbus.h"
#include "hc05_gamepad.h"
#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal_flash_ex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

// 达妙(DM)六关节电机反馈变量（定义在 bsp_can.cpp）
extern DM_motor_info DM_motor_J0;
extern DM_motor_info DM_motor_J1;
extern DM_motor_info DM_motor_J2;
extern DM_motor_info DM_motor_J3;
extern DM_motor_info DM_motor_J4;
extern DM_motor_info DM_motor_J5;

// 便于在调试器中直接观察录制/回放状态。
volatile uint8_t  arm_control_state = 0;
volatile uint32_t arm_record_sample_count = 0;
volatile uint32_t arm_playback_sample_index = 0;
volatile uint32_t arm_fault_code = 0;
volatile uint32_t arm_missing_feedback_mask = 0;
volatile uint32_t arm_motor_error_mask = 0;
volatile int32_t  arm_can_tx_failed_joint = -1;
volatile uint8_t  arm_remote_raw_mode = 0;
volatile uint8_t  arm_requested_mode = 0;
volatile uint32_t arm_pa0_press_count = 0;
volatile uint8_t  arm_pa0_mode = 0;

namespace {
constexpr std::size_t JOINT_COUNT = 6;

constexpr uint16_t DM_CAN_IDS[JOINT_COUNT] = {
    0x02,  // J0 底部 yaw
    0x03,  // J1 底部 pitch
    0x04,  // J2 roll
    0x05,  // J3 末端 yaw
    0x06,  // J4 差速腕部电机 1
    0x07   // J5 差速腕部电机 2
};

DM_motor_info* const DM_MOTORS[JOINT_COUNT] = {
    &DM_motor_J0, &DM_motor_J1, &DM_motor_J2,
    &DM_motor_J3, &DM_motor_J4, &DM_motor_J5
};

// ===== 机械限位（rad） =====
constexpr float J0_PERIOD   = 6.283185307179586f;
constexpr float J0_LIMIT_LO = 1.72713089f;
constexpr float J0_LIMIT_HI = 5.405096f;
constexpr float J0_ANCHOR   = (J0_LIMIT_LO + J0_LIMIT_HI) * 0.5f;
constexpr float J1_MIN      = 1.05001163f;
constexpr float J1_MAX      = 2.27759933f;
constexpr float J2_MIN      = 0.00324249268f;
constexpr float J2_MAX      = 1.84920311f;

// J4/J5 差速限位，差值的等价周期为 DM 反馈位置范围 25 rad。
constexpr float DM_POS_PERIOD = 25.0f;
constexpr float J45_LOW       = -2.08094978f;
constexpr float J45_HIGH      = 2.83817816f;
constexpr float J45_CENTER    = (J45_LOW + J45_HIGH) * 0.5f;

// ===== 手动控制参数 =====
constexpr float CTRL_INC    = 0.00015f;
constexpr float CTRL_DEAD   = 20.0f;
constexpr float POS_MAX_VEL = 2.0f;

// ===== 录制与回放参数 =====
constexpr uint32_t RECORD_FLASH_ADDRESS = 0x080E0000U;  // STM32F407 sector 11
constexpr uint32_t RECORD_FLASH_SIZE    = 128U * 1024U;
constexpr uint32_t RECORD_MAGIC         = 0x524D5241U;  // "ARMR" little-endian
constexpr uint32_t RECORD_VERSION       = 2U;
constexpr uint32_t RECORD_PERIOD_MS     = 10U;          // 100 Hz

constexpr uint32_t DBUS_TIMEOUT_MS      = 100U;
constexpr uint32_t FEEDBACK_TIMEOUT_MS  = 100U;
constexpr uint32_t FEEDBACK_REQUEST_TIMEOUT_MS = 20U;
constexpr uint32_t FEEDBACK_REQUEST_RETRIES = 3U;
constexpr uint32_t MOTOR_STARTUP_DELAY_MS = 2000U;
constexpr uint32_t STARTUP_ENABLE_TEST_MS  = 1000U;
constexpr uint32_t STARTUP_TEST_PERIOD_MS  = 10U;
constexpr uint32_t SWITCH_SETTLE_MS       = 50U;
constexpr uint32_t PLAYBACK_SWITCH_HOLD_MS = 50U;
constexpr bool PA0_BUTTON_TEST_MODE       = true;
constexpr uint32_t PA0_DEBOUNCE_MS        = 30U;
// Temporary diagnostic switch: disable the arm-specific software travel limits
// and recorded-step guard. Keep finite-value/DM protocol-range validation active.
constexpr bool SOFTWARE_POSITION_LIMITS_ENABLED = false;
constexpr float RETURN_SPEED            = 0.30f;        // rad/s
constexpr float PLAYBACK_MAX_SPEED      = 2.0f;         // rad/s
constexpr float RETURN_TOLERANCE        = 0.04f;        // rad
constexpr float TRACKING_PAUSE_ERROR    = 0.35f;        // rad
constexpr uint32_t POSITION_STABLE_MS   = 300U;
constexpr uint32_t TRACKING_TIMEOUT_MS  = 3000U;
constexpr float LIMIT_TOLERANCE         = 0.05f;
constexpr float MAX_RECORDED_STEP       = 0.50f;        // 过滤坏帧/反馈跳变

struct RecordHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t sample_period_ms;
    uint32_t sample_count;
    uint32_t data_bytes;
    uint32_t data_crc32;
    uint32_t reserved0;
    uint32_t reserved1;
};

struct RecordSample {
    uint16_t pos_raw[JOINT_COUNT];
    uint8_t gripper_state;
    uint8_t reserved[3];
};

static_assert(sizeof(RecordHeader) == 32U, "Unexpected record header size");
static_assert(sizeof(RecordSample) == 16U, "Unexpected record sample size");

constexpr uint32_t RECORD_DATA_ADDRESS = RECORD_FLASH_ADDRESS + sizeof(RecordHeader);
constexpr uint32_t MAX_RECORD_SAMPLES =
    (RECORD_FLASH_SIZE - sizeof(RecordHeader)) / sizeof(RecordSample);

enum class ControlState : uint8_t {
    Safe = 0,
    Manual = 1,
    Recording = 2,
    RecordStopped = 3,
    ReturnToStart = 4,
    Playback = 5,
    PlaybackComplete = 6,
    Fault = 7
};

enum FaultCode : uint32_t {
    FAULT_NONE = 0,
    FAULT_DBUS_TIMEOUT = 1,
    FAULT_FEEDBACK_TIMEOUT = 2,
    FAULT_MOTOR_ERROR = 3,
    FAULT_FLASH_ERASE = 4,
    FAULT_FLASH_PROGRAM = 5,
    FAULT_RECORD_INVALID = 6,
    FAULT_CAN_TX = 7,
    FAULT_RETURN_TIMEOUT = 8,
    FAULT_TRACKING_TIMEOUT = 9,
    FAULT_UNSAFE_SAMPLE = 10
};

enum class FeedbackPollResult : uint8_t {
    Ok = 0,
    TxError = 1,
    Timeout = 2
};

struct JointSnapshot {
    uint16_t pos_raw[JOINT_COUNT];
    float pos[JOINT_COUNT];
    uint8_t err[JOINT_COUNT];
    uint32_t last_update_tick[JOINT_COUNT];
    uint32_t rx_count[JOINT_COUNT];
};

struct ManualTargets {
    float j0;
    float j1;
    float j2;
    float j3;
    float wrist_pitch;
};

struct PlaybackContext {
    RecordHeader header{};
    float start[JOINT_COUNT]{};
    float return_command[JOINT_COUNT]{};
    float last_command[JOINT_COUNT]{};
    uint32_t sample_index = 0;
    uint32_t next_tick = 0;
    uint32_t return_deadline = 0;
    uint32_t stable_since = 0;
    uint32_t tracking_error_since = 0;
};

struct SwitchFilter {
    uint8_t candidate = 0U;
    uint8_t active = 0U;
    uint32_t candidate_since = 0U;
};

struct ButtonFilter {
    bool candidate_pressed = false;
    bool stable_pressed = false;
    uint32_t candidate_since = 0U;
};

bool pa0_button_press_event(ButtonFilter& filter, uint32_t now) {
    const bool pressed = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;
    if (pressed != filter.candidate_pressed) {
        filter.candidate_pressed = pressed;
        filter.candidate_since = now;
    }
    if (filter.stable_pressed != filter.candidate_pressed &&
        (now - filter.candidate_since) >= PA0_DEBOUNCE_MS) {
        filter.stable_pressed = filter.candidate_pressed;
        return filter.stable_pressed;
    }
    return false;
}

float dbus_deadzone(int16_t value) {
    if (value > CTRL_DEAD) return static_cast<float>(value - CTRL_DEAD);
    if (value < -CTRL_DEAD) return static_cast<float>(value + CTRL_DEAD);
    return 0.0f;
}

float wrap_j0(float pos) {
    float best = pos;
    float best_distance = std::fabs(pos - J0_ANCHOR);
    for (int k = -1; k <= 1; ++k) {
        const float candidate = pos + static_cast<float>(k) * J0_PERIOD;
        const float distance = std::fabs(candidate - J0_ANCHOR);
        if (distance < best_distance) {
            best = candidate;
            best_distance = distance;
        }
    }
    return best;
}

// 将差速值映射到最靠近限位区间中心的等价值。
float wrap_wrist_delta(float delta) {
    return delta + std::round((J45_CENTER - delta) / DM_POS_PERIOD) * DM_POS_PERIOD;
}

float raw_to_position(uint16_t raw) {
    return static_cast<float>(raw) * (DM_P_MAX - DM_P_MIN) / 65535.0f + DM_P_MIN;
}

bool tick_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void schedule_next_tick(uint32_t now, uint32_t& deadline, uint32_t period) {
    deadline += period;
    if (static_cast<int32_t>(now - deadline) >= static_cast<int32_t>(period)) {
        deadline = now + period;
    }
}

void take_joint_snapshot(JointSnapshot& snapshot) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        snapshot.pos_raw[i] = DM_MOTORS[i]->pos_raw;
        snapshot.pos[i] = DM_MOTORS[i]->pos;
        snapshot.err[i] = DM_MOTORS[i]->err;
        snapshot.last_update_tick[i] = DM_MOTORS[i]->last_update_tick;
        snapshot.rx_count[i] = DM_MOTORS[i]->rx_count;
    }
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

bool feedback_is_fresh(const JointSnapshot& snapshot, uint32_t now) {
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        if (snapshot.rx_count[i] == 0U ||
            (now - snapshot.last_update_tick[i]) > FEEDBACK_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

bool feedback_has_error(const JointSnapshot& snapshot) {
    uint32_t error_mask = 0U;
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        // 达妙反馈高四位的 0/1 分别是失能/使能状态，8~E 才是故障。
        if (snapshot.err[i] >= 8U) error_mask |= (1UL << i);
    }
    arm_motor_error_mask = error_mask;
    return error_mask != 0U;
}

void update_joint_fault_blink(uint32_t now, uint32_t joint_mask,
                              uint8_t fault_indicator) {
    if (joint_mask == 0U) {
        HC05_Gamepad_SetRuntimeIndicator(fault_indicator);
        return;
    }

    uint8_t joint = 0U;
    while (joint + 1U < JOINT_COUNT &&
           (joint_mask & (1UL << joint)) == 0U) {
        ++joint;
    }

    // 每 3 秒重复一组：粉色闪 1~6 次分别表示 J0~J5，单次亮灭各 150 ms。
    const uint32_t phase = now % 3000U;
    const uint32_t pulse_window = static_cast<uint32_t>(joint + 1U) * 300U;
    const bool led_on = phase < pulse_window && (phase % 300U) < 150U;
    HC05_Gamepad_SetRuntimeIndicator(led_on ? fault_indicator
                                            : HC05_INDICATOR_OFF);
}

void update_fault_blink(uint32_t now, FaultCode fault) {
    if (fault == FAULT_MOTOR_ERROR) {
        update_joint_fault_blink(now, arm_motor_error_mask,
                                 HC05_INDICATOR_FAULT_MOTOR);
        return;
    }

    if (fault == FAULT_FEEDBACK_TIMEOUT) {
        update_joint_fault_blink(now, arm_missing_feedback_mask,
                                 HC05_INDICATOR_FAULT_FEEDBACK);
        return;
    }

    if (fault == FAULT_CAN_TX && arm_can_tx_failed_joint >= 0 &&
        arm_can_tx_failed_joint < static_cast<int32_t>(JOINT_COUNT)) {
        update_joint_fault_blink(now,
                                 1UL << static_cast<uint32_t>(arm_can_tx_failed_joint),
                                 HC05_INDICATOR_FAULT_FEEDBACK);
    }
}

bool remote_is_fresh(uint32_t now) {
    return dbus.frame_count != 0U && (now - dbus.last_update_tick) <= DBUS_TIMEOUT_MS;
}

bool disable_all_motors(bsp_can& can) {
    bool ok = true;
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        ok = (can.BSP_CAN1_DMMotorDisableCmd(DM_CAN_IDS[i], DM_POS_MODE) == HAL_OK) && ok;
        osDelay(2);
    }
    return ok;
}

// 达妙为一发一收模式。录制时电机保持失能，逐个发送状态刷新请求取得新位置反馈。
FeedbackPollResult poll_disabled_feedback(bsp_can& can, JointSnapshot& snapshot) {
    arm_missing_feedback_mask = 0U;
    arm_can_tx_failed_joint = -1;

    // 达妙为一发一收模式。逐台请求并等待对应反馈，避免连续六帧后统一等待
    // 导致慢回复或偶发丢帧把整个机械臂误判为离线。
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        bool updated = false;
        bool request_sent = false;
        for (uint32_t retry = 0U; retry < FEEDBACK_REQUEST_RETRIES; ++retry) {
            JointSnapshot before{};
            take_joint_snapshot(before);

            if (can.BSP_CAN1_DMMotorRefreshStatusCmd(DM_CAN_IDS[i]) == HAL_OK) {
                request_sent = true;
                const uint32_t deadline = HAL_GetTick() + FEEDBACK_REQUEST_TIMEOUT_MS;
                do {
                    take_joint_snapshot(snapshot);
                    if (snapshot.rx_count[i] != before.rx_count[i]) {
                        updated = true;
                        break;
                    }
                    osDelay(1);
                } while (!tick_reached(HAL_GetTick(), deadline));
            }
            if (updated) break;

            // 兼容不支持 0x7FF/0xCC 状态刷新的旧固件：回退到原失能命令取反馈。
            take_joint_snapshot(before);
            if (can.BSP_CAN1_DMMotorDisableCmd(DM_CAN_IDS[i], DM_POS_MODE) == HAL_OK) {
                request_sent = true;
                const uint32_t deadline = HAL_GetTick() + FEEDBACK_REQUEST_TIMEOUT_MS;
                do {
                    take_joint_snapshot(snapshot);
                    if (snapshot.rx_count[i] != before.rx_count[i]) {
                        updated = true;
                        break;
                    }
                    osDelay(1);
                } while (!tick_reached(HAL_GetTick(), deadline));
            }
            if (updated) break;

            if (!request_sent && retry + 1U == FEEDBACK_REQUEST_RETRIES) {
                arm_can_tx_failed_joint = static_cast<int32_t>(i);
                return FeedbackPollResult::TxError;
            }
        }

        if (!updated) {
            arm_missing_feedback_mask |= (1UL << i);
        }
    }

    take_joint_snapshot(snapshot);
    return arm_missing_feedback_mask == 0U
               ? FeedbackPollResult::Ok
               : FeedbackPollResult::Timeout;
}

bool handle_feedback_poll_result(FeedbackPollResult result, FaultCode& fault) {
    switch (result) {
        case FeedbackPollResult::Ok:
            return true;
        case FeedbackPollResult::TxError:
            fault = FAULT_CAN_TX;
            return false;
        case FeedbackPollResult::Timeout:
        default:
            fault = FAULT_FEEDBACK_TIMEOUT;
            return false;
    }
}

void record_enable_failure(std::size_t joint) {
    arm_can_tx_failed_joint = static_cast<int32_t>(joint);
}

void clear_enable_diagnostics() {
    arm_can_tx_failed_joint = -1;
    arm_motor_error_mask = 0U;
}

bool wait_for_mode_feedback(bsp_can& can, JointSnapshot& snapshot,
                            FaultCode& fault) {
    const FeedbackPollResult result = poll_disabled_feedback(can, snapshot);
    return handle_feedback_poll_result(result, fault);
}

bool enable_all_at_current_position(bsp_can& can, const JointSnapshot& snapshot) {
    clear_enable_diagnostics();
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        if (can.BSP_CAN1_DMMotorEnableCmd(DM_CAN_IDS[i], DM_POS_MODE) != HAL_OK) {
            record_enable_failure(i);
            disable_all_motors(can);
            return false;
        }
        osDelay(2);
        if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[i], snapshot.pos[i],
                                            RETURN_SPEED) != HAL_OK) {
            record_enable_failure(i);
            disable_all_motors(can);
            return false;
        }
        osDelay(2);
    }
    return true;
}

uint8_t update_switch_filter(SwitchFilter& filter, uint8_t raw, uint32_t now) {
    if (raw < 1U || raw > 3U) {
        filter.candidate = 0U;
        filter.active = 0U;
        filter.candidate_since = now;
        return 0U;
    }

    if (raw != filter.candidate) {
        filter.candidate = raw;
        filter.candidate_since = now;
    }

    const uint32_t required_hold =
        (filter.candidate == 3U) ? PLAYBACK_SWITCH_HOLD_MS : SWITCH_SETTLE_MS;
    if (filter.active != filter.candidate &&
        (now - filter.candidate_since) >= required_hold) {
        filter.active = filter.candidate;
    }
    return filter.active;
}

bool send_joint_positions(bsp_can& can, const float target[JOINT_COUNT],
                          float max_velocity, uint32_t inter_frame_delay_ms) {
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        if (!std::isfinite(target[i]) ||
            can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[i], target[i], max_velocity) != HAL_OK) {
            return false;
        }
        if (inter_frame_delay_ms != 0U) osDelay(inter_frame_delay_ms);
    }
    return true;
}


uint32_t crc32_update(uint32_t crc, const uint8_t* data, std::size_t length) {
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc;
}

bool erase_record_sector() {
    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = FLASH_SECTOR_11;
    erase.NbSectors = 1;

    uint32_t sector_error = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    return status == HAL_OK && sector_error == 0xFFFFFFFFU;
}

bool program_flash_words(uint32_t address, const void* source, std::size_t length) {
    if ((address & 3U) != 0U || (length & 3U) != 0U) return false;

    const auto* bytes = static_cast<const uint8_t*>(source);
    HAL_FLASH_Unlock();
    for (std::size_t offset = 0; offset < length; offset += sizeof(uint32_t)) {
        uint32_t word = 0;
        std::memcpy(&word, bytes + offset, sizeof(word));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        if (*reinterpret_cast<const volatile uint32_t*>(address + offset) != word) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

void read_record_sample(uint32_t index, RecordSample& sample) {
    const uint32_t address = RECORD_DATA_ADDRESS + index * sizeof(RecordSample);
    std::memcpy(&sample, reinterpret_cast<const void*>(address), sizeof(sample));
}

void sample_to_positions(const RecordSample& sample, float target[JOINT_COUNT]) {
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        target[i] = raw_to_position(sample.pos_raw[i]);
    }
}

bool positions_are_safe(const float target[JOINT_COUNT]) {
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        if (!std::isfinite(target[i]) || target[i] < DM_P_MIN || target[i] > DM_P_MAX) {
            return false;
        }
    }

    if (!SOFTWARE_POSITION_LIMITS_ENABLED) return true;

    if (target[0] < J0_LIMIT_LO - LIMIT_TOLERANCE ||
        target[0] > J0_LIMIT_HI + LIMIT_TOLERANCE ||
        target[1] < J1_MIN - LIMIT_TOLERANCE ||
        target[1] > J1_MAX + LIMIT_TOLERANCE ||
        target[2] < J2_MIN - LIMIT_TOLERANCE ||
        target[2] > J2_MAX + LIMIT_TOLERANCE) {
        return false;
    }

    const float wrist_delta = wrap_wrist_delta(target[4] - target[5]);
    return wrist_delta >= J45_LOW - LIMIT_TOLERANCE &&
           wrist_delta <= J45_HIGH + LIMIT_TOLERANCE;
}

bool gripper_state_is_valid(uint8_t state) {
    return state == 1U || state == 3U;
}

bool validate_record(RecordHeader& header) {
    std::memcpy(&header, reinterpret_cast<const void*>(RECORD_FLASH_ADDRESS), sizeof(header));
    if (header.magic != RECORD_MAGIC ||
        header.version != RECORD_VERSION ||
        header.sample_period_ms != RECORD_PERIOD_MS ||
        header.sample_count == 0U ||
        header.sample_count > MAX_RECORD_SAMPLES ||
        header.data_bytes != header.sample_count * sizeof(RecordSample)) {
        return false;
    }

    uint32_t crc = 0xFFFFFFFFU;
    float previous[JOINT_COUNT]{};
    for (uint32_t index = 0; index < header.sample_count; ++index) {
        RecordSample sample{};
        float target[JOINT_COUNT]{};
        read_record_sample(index, sample);
        crc = crc32_update(crc, reinterpret_cast<const uint8_t*>(&sample), sizeof(sample));
        sample_to_positions(sample, target);
        if (!positions_are_safe(target) || !gripper_state_is_valid(sample.gripper_state))
            return false;

        if (SOFTWARE_POSITION_LIMITS_ENABLED && index != 0U) {
            for (std::size_t joint = 0; joint < JOINT_COUNT; ++joint) {
                if (std::fabs(target[joint] - previous[joint]) > MAX_RECORDED_STEP) {
                    return false;
                }
            }
        }
        std::memcpy(previous, target, sizeof(previous));
    }
    return (~crc) == header.data_crc32;
}

bool finalize_record(uint32_t sample_count, uint32_t running_crc) {
    if (sample_count == 0U) return false;

    RecordHeader header{};
    header.magic = RECORD_MAGIC;
    header.version = RECORD_VERSION;
    header.sample_period_ms = RECORD_PERIOD_MS;
    header.sample_count = sample_count;
    header.data_bytes = sample_count * sizeof(RecordSample);
    header.data_crc32 = ~running_crc;

    // magic 最后写入：掉电或写入失败时，该记录不会被误认为有效。
    const auto* header_bytes = reinterpret_cast<const uint8_t*>(&header);
    if (!program_flash_words(RECORD_FLASH_ADDRESS + sizeof(uint32_t),
                             header_bytes + sizeof(uint32_t),
                             sizeof(RecordHeader) - sizeof(uint32_t))) {
        return false;
    }
    return program_flash_words(RECORD_FLASH_ADDRESS, &header.magic, sizeof(header.magic));
}

bool append_record_sample(const JointSnapshot& snapshot, uint8_t gripper_state,
                          uint32_t index, uint32_t& running_crc) {
    if (index >= MAX_RECORD_SAMPLES) return false;
    if (!gripper_state_is_valid(gripper_state)) return false;

    RecordSample sample{};
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        sample.pos_raw[i] = snapshot.pos_raw[i];
    }
    sample.gripper_state = gripper_state;
    const uint32_t address = RECORD_DATA_ADDRESS + index * sizeof(RecordSample);
    if (!program_flash_words(address, &sample, sizeof(sample))) return false;
    running_crc = crc32_update(running_crc,
                               reinterpret_cast<const uint8_t*>(&sample), sizeof(sample));
    return true;
}

float move_towards(float current, float target, float max_step) {
    if (current < target) return std::min(current + max_step, target);
    return std::max(current - max_step, target);
}

float maximum_error(const float target[JOINT_COUNT], const JointSnapshot& snapshot) {
    float error = 0.0f;
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        error = std::max(error, std::fabs(target[i] - snapshot.pos[i]));
    }
    return error;
}

void sync_manual_targets(ManualTargets& target, const JointSnapshot& snapshot) {
    target.j0 = SOFTWARE_POSITION_LIMITS_ENABLED
                    ? std::clamp(wrap_j0(snapshot.pos[0]), J0_LIMIT_LO, J0_LIMIT_HI)
                    : snapshot.pos[0];
    target.j1 = SOFTWARE_POSITION_LIMITS_ENABLED
                    ? std::clamp(snapshot.pos[1], J1_MIN, J1_MAX)
                    : snapshot.pos[1];
    target.j2 = SOFTWARE_POSITION_LIMITS_ENABLED
                    ? std::clamp(snapshot.pos[2], J2_MIN, J2_MAX)
                    : snapshot.pos[2];
    target.j3 = snapshot.pos[3];
    const float wrist_delta = snapshot.pos[4] - snapshot.pos[5];
    target.wrist_pitch = SOFTWARE_POSITION_LIMITS_ENABLED
                             ? std::clamp(wrap_wrist_delta(wrist_delta),
                                          J45_LOW, J45_HIGH)
                             : wrist_delta;
}

bool run_manual_control(bsp_can& can, ManualTargets& target,
                        const JointSnapshot& snapshot) {
    const float c0 = dbus_deadzone(dbus.ch[0]);
    const float c1 = dbus_deadzone(dbus.ch[1]);
    const float c2 = dbus_deadzone(dbus.ch[2]);
    const float c3 = dbus_deadzone(dbus.ch[3]);
    const float c4 = dbus_deadzone(dbus.ch[4]);

    target.j0 += c0 * CTRL_INC;
    if (SOFTWARE_POSITION_LIMITS_ENABLED)
        target.j0 = std::clamp(target.j0, J0_LIMIT_LO, J0_LIMIT_HI);
    if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[0], target.j0, POS_MAX_VEL) != HAL_OK)
        return false;
    osDelay(2);

    target.j1 += c1 * CTRL_INC;
    if (SOFTWARE_POSITION_LIMITS_ENABLED)
        target.j1 = std::clamp(target.j1, J1_MIN, J1_MAX);
    if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[1], target.j1, POS_MAX_VEL) != HAL_OK)
        return false;
    osDelay(2);

    target.j2 += c2 * CTRL_INC;
    if (SOFTWARE_POSITION_LIMITS_ENABLED)
        target.j2 = std::clamp(target.j2, J2_MIN, J2_MAX);
    if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[2], target.j2, POS_MAX_VEL) != HAL_OK)
        return false;
    osDelay(2);

    target.j3 += c3 * CTRL_INC;
    if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[3], target.j3, 1.0f) != HAL_OK)
        return false;
    osDelay(2);

    target.wrist_pitch += c4 * CTRL_INC;
    if (SOFTWARE_POSITION_LIMITS_ENABLED)
        target.wrist_pitch = std::clamp(target.wrist_pitch, J45_LOW, J45_HIGH);
    const float raw_current_pitch = snapshot.pos[4] - snapshot.pos[5];
    const float current_pitch = SOFTWARE_POSITION_LIMITS_ENABLED
                                    ? wrap_wrist_delta(raw_current_pitch)
                                    : raw_current_pitch;
    const float pitch_error = target.wrist_pitch - current_pitch;
    const float wrist_target[2] = {
        snapshot.pos[4] + pitch_error * 0.5f,
        snapshot.pos[5] - pitch_error * 0.5f
    };
    if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[4], wrist_target[0], POS_MAX_VEL) != HAL_OK)
        return false;
    osDelay(2);
    if (can.BSP_CAN1_DMMotorPositionCmd(DM_CAN_IDS[5], wrist_target[1], POS_MAX_VEL) != HAL_OK)
        return false;
    osDelay(2);
    return true;
}

void apply_gripper_state(uint8_t state) {
    if (state == 1U) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1700);
    } else if (state == 3U) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2500);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1200);
    }
}

void update_gripper_from_remote(uint8_t& current_state) {
    if (dbus.s1 == 1U || dbus.s1 == 3U) {
        current_state = dbus.s1;
        apply_gripper_state(current_state);
    }
}

}  // namespace

bsp_can ABC;

void MotorTask::run() {
    ControlState state = ControlState::Safe;
    ManualTargets manual_target{};
    PlaybackContext playback{};
    bool motors_enabled = false;
    uint32_t record_crc = 0xFFFFFFFFU;
    uint32_t record_count = 0U;
    uint32_t record_next_tick = 0U;
    uint32_t manual_tx_failures = 0U;
    uint32_t handled_start_press_count = 0U;
    uint32_t handled_mode_press_count = 0U;
    SwitchFilter switch_filter{};
    ButtonFilter pa0_button_filter{};
    uint8_t pa0_mode = 0U;
    // main.c 上电时将夹爪初始化为打开状态。
    uint8_t current_gripper_state = 3U;

    auto set_state = [&](ControlState next) {
        state = next;
        arm_control_state = static_cast<uint8_t>(next);
        if (next == ControlState::Recording) {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_RECORDING);
        } else if (next == ControlState::ReturnToStart ||
                   next == ControlState::Playback ||
                   next == ControlState::PlaybackComplete) {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_PLAYBACK);
        } else {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_STOP);
        }
    };

    auto enter_fault = [&](FaultCode fault) {
        if (motors_enabled) {
            disable_all_motors(ABC);
            motors_enabled = false;
        }
        arm_fault_code = fault;
        set_state(ControlState::Fault);
        if (fault == FAULT_MOTOR_ERROR) {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_FAULT_MOTOR);
        } else if (fault == FAULT_FLASH_ERASE || fault == FAULT_FLASH_PROGRAM) {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_FAULT_FLASH);
        } else if (fault == FAULT_RECORD_INVALID || fault == FAULT_UNSAFE_SAMPLE) {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_FAULT_DATA);
        } else {
            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_FAULT_FEEDBACK);
        }
    };

    // 达妙上电初始化约 1 秒；留足 2 秒后再发送第一帧 CAN 命令。
    osDelay(MOTOR_STARTUP_DELAY_MS);

    // PA0 测试模式：上电默认停止并失能，等待板载按键的第一次按下。
    // PA0 test mode: after the two-second driver boot delay, briefly enable every
    // joint at its measured position. This exercises the real enable path without
    // commanding a move, then returns to the disabled recording-ready state.
    if (PA0_BUTTON_TEST_MODE) {
        JointSnapshot startup_snapshot{};
        FaultCode startup_fault = FAULT_NONE;

        if (!wait_for_mode_feedback(ABC, startup_snapshot, startup_fault) ||
            !feedback_is_fresh(startup_snapshot, HAL_GetTick())) {
            if (startup_fault == FAULT_NONE) startup_fault = FAULT_FEEDBACK_TIMEOUT;
            enter_fault(startup_fault);
        } else {
            float hold_position[JOINT_COUNT]{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                hold_position[i] = startup_snapshot.pos[i];
            }

            HC05_Gamepad_SetRuntimeIndicator(HC05_INDICATOR_RECORDING);
            if (!enable_all_at_current_position(ABC, startup_snapshot)) {
                enter_fault(FAULT_CAN_TX);
            } else {
                motors_enabled = true;
                uint32_t detected_error_mask = 0U;
                for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                    if (startup_snapshot.err[i] >= 8U) {
                        detected_error_mask |= (1UL << i);
                    }
                }

                const uint32_t test_deadline = HAL_GetTick() + STARTUP_ENABLE_TEST_MS;
                uint32_t next_test_tick = HAL_GetTick();
                while (!tick_reached(HAL_GetTick(), test_deadline)) {
                    const uint32_t test_now = HAL_GetTick();
                    if (!tick_reached(test_now, next_test_tick)) {
                        osDelay(1);
                        continue;
                    }

                    JointSnapshot live_snapshot{};
                    take_joint_snapshot(live_snapshot);
                    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                        if (live_snapshot.err[i] >= 8U) {
                            detected_error_mask |= (1UL << i);
                        }
                    }

                    if (!feedback_is_fresh(live_snapshot, test_now)) {
                        startup_fault = FAULT_FEEDBACK_TIMEOUT;
                        break;
                    }
                    if (!send_joint_positions(ABC, hold_position, RETURN_SPEED, 1U)) {
                        startup_fault = FAULT_CAN_TX;
                        break;
                    }
                    schedule_next_tick(test_now, next_test_tick,
                                       STARTUP_TEST_PERIOD_MS);
                }

                disable_all_motors(ABC);
                motors_enabled = false;
                arm_motor_error_mask = detected_error_mask;
                if (detected_error_mask != 0U) {
                    enter_fault(FAULT_MOTOR_ERROR);
                } else if (startup_fault != FAULT_NONE) {
                    enter_fault(startup_fault);
                } else {
                    arm_fault_code = FAULT_NONE;
                    set_state(ControlState::Safe);
                }
            }
        }
    } else {
        disable_all_motors(ABC);
        set_state(ControlState::Safe);
    }

    // Keep a startup diagnostic fault latched. With mode 0 the normal stop branch
    // intentionally clears faults, so use the record request value until PA0 is
    // pressed; the Fault branch still prevents recording or enabling any motor.
    if (PA0_BUTTON_TEST_MODE && state == ControlState::Fault) {
        pa0_mode = 1U;
        arm_pa0_mode = pa0_mode;
    }

    for (;;) {
        const uint32_t now = HAL_GetTick();

        if (!PA0_BUTTON_TEST_MODE && !remote_is_fresh(now)) {
            if (state == ControlState::Recording && record_count != 0U) {
                if (!finalize_record(record_count, record_crc)) {
                    arm_fault_code = FAULT_FLASH_PROGRAM;
                }
            }
            if (motors_enabled) {
                disable_all_motors(ABC);
                motors_enabled = false;
            }
            arm_fault_code = FAULT_DBUS_TIMEOUT;
            set_state(ControlState::Safe);
            osDelay(5);
            continue;
        }

        if (PA0_BUTTON_TEST_MODE && pa0_button_press_event(pa0_button_filter, now)) {
            ++arm_pa0_press_count;
            switch (state) {
                case ControlState::Recording:
                case ControlState::RecordStopped:
                    pa0_mode = 3U;  // 录制结束并回放。
                    break;
                case ControlState::ReturnToStart:
                case ControlState::Playback:
                    pa0_mode = 0U;  // 回放过程中强制失能。
                    break;
                case ControlState::PlaybackComplete:
                case ControlState::Safe:
                case ControlState::Manual:
                    pa0_mode = 1U;  // 新一轮录制。
                    break;
                case ControlState::Fault:
                default:
                    arm_fault_code = FAULT_NONE;
                    set_state(ControlState::Safe);
                    pa0_mode = 1U;  // 故障后按一次按键直接重新录制。
                    break;
            }
            arm_pa0_mode = pa0_mode;
        }

        if (!PA0_BUTTON_TEST_MODE) {
            // START 使用独立的按下计数，因此在已经停止时再次单击也能触发一次清除。
            const uint32_t start_press_count = hc05_gamepad.start_press_count;
            if (start_press_count != handled_start_press_count) {
                handled_start_press_count = start_press_count;
                if (motors_enabled) {
                    disable_all_motors(ABC);
                    motors_enabled = false;
                } else {
                    disable_all_motors(ABC);
                }

                set_state(ControlState::Safe);
                if (!erase_record_sector()) {
                    enter_fault(FAULT_FLASH_ERASE);
                    osDelay(5);
                    continue;
                }

                record_crc = 0xFFFFFFFFU;
                record_count = 0U;
                arm_record_sample_count = 0U;
                arm_playback_sample_index = 0U;
                arm_fault_code = FAULT_NONE;
                switch_filter = {};
                arm_remote_raw_mode = 0U;
                arm_requested_mode = 0U;
                set_state(ControlState::Safe);
                osDelay(5);
                continue;
            }

            const uint32_t mode_press_count = hc05_gamepad.mode_press_count;
            const bool new_mode_request = mode_press_count != handled_mode_press_count;
            if (new_mode_request) {
                handled_mode_press_count = mode_press_count;
                if (state == ControlState::Fault) {
                    arm_fault_code = FAULT_NONE;
                    switch_filter = {};
                    set_state(ControlState::Safe);
                }
            }
        }

        const uint8_t requested_mode = PA0_BUTTON_TEST_MODE
                                           ? pa0_mode
                                           : update_switch_filter(switch_filter, dbus.s2, now);
        arm_remote_raw_mode = PA0_BUTTON_TEST_MODE ? pa0_mode : dbus.s2;
        arm_requested_mode = requested_mode;

        // 回放挡内不接受实时 S1，避免覆盖 Flash 中记录的夹爪动作。
        // 返回起点期间保持进入回放前的夹爪状态；正式回放后由采样点驱动。
        if (!PA0_BUTTON_TEST_MODE && requested_mode != 3U) {
            update_gripper_from_remote(current_gripper_state);
        }

        // LB 从录制切到回放时先提交记录；START 已在上面的独立分支中停止并清空记录。
        if (state == ControlState::Recording && requested_mode != 1U) {
            if (!finalize_record(record_count, record_crc)) {
                if (requested_mode == 0U) {
                    // 非按键事件造成的停止仍优先保证电机安全失能。
                    set_state(ControlState::Safe);
                } else {
                    enter_fault(FAULT_FLASH_PROGRAM);
                    osDelay(5);
                    continue;
                }
            } else {
                set_state(ControlState::RecordStopped);
            }
        }

        if (requested_mode < 1U || requested_mode > 3U) {
            if (motors_enabled) {
                disable_all_motors(ABC);
                motors_enabled = false;
            }
            // START 或通信超时后的停止状态同时解除故障锁存。
            arm_fault_code = FAULT_NONE;
            set_state(ControlState::Safe);
            osDelay(5);
            continue;
        }

        // 同一请求发生故障后保持锁存，直到收到下一次 RB/LB 单击或 START。
        if (state == ControlState::Fault) {
            update_fault_blink(now, static_cast<FaultCode>(arm_fault_code));
            osDelay(5);
            continue;
        }

        if (requested_mode == 1U) {
            if (state != ControlState::Recording && state != ControlState::RecordStopped) {
                if (motors_enabled) {
                    disable_all_motors(ABC);
                    motors_enabled = false;
                } else {
                    disable_all_motors(ABC);
                }

                // 扇区擦除可能耗时较长，但此时六个电机已经失能。
                if (!erase_record_sector()) {
                    enter_fault(FAULT_FLASH_ERASE);
                    continue;
                }
                record_crc = 0xFFFFFFFFU;
                record_count = 0U;
                arm_record_sample_count = 0U;
                record_next_tick = HAL_GetTick();
                set_state(ControlState::Recording);
            }

            if (state == ControlState::Recording && tick_reached(now, record_next_tick)) {
                JointSnapshot snapshot{};
                FaultCode poll_fault = FAULT_NONE;
                if (!wait_for_mode_feedback(ABC, snapshot, poll_fault)) {
                    enter_fault(poll_fault);
                    continue;
                }
                const uint32_t sample_now = HAL_GetTick();
                if (!feedback_is_fresh(snapshot, sample_now)) {
                    enter_fault(FAULT_FEEDBACK_TIMEOUT);
                    continue;
                }
                // 录制时电机保持失能，记录报码但不阻止位置采样。
                // 回放使能前仍会严格检查并拒绝带故障电机。
                feedback_has_error(snapshot);
                if (!positions_are_safe(snapshot.pos)) {
                    enter_fault(FAULT_UNSAFE_SAMPLE);
                    continue;
                }

                if (!append_record_sample(snapshot, current_gripper_state,
                                          record_count, record_crc)) {
                    enter_fault(FAULT_FLASH_PROGRAM);
                    continue;
                }
                ++record_count;
                arm_record_sample_count = record_count;
                schedule_next_tick(sample_now, record_next_tick, RECORD_PERIOD_MS);

                if (record_count >= MAX_RECORD_SAMPLES) {
                    if (!finalize_record(record_count, record_crc)) {
                        enter_fault(FAULT_FLASH_PROGRAM);
                    } else {
                        set_state(ControlState::RecordStopped);
                    }
                }
            }
            osDelay(1);
            continue;
        }

        if (requested_mode == 2U) {
            if (state != ControlState::Manual) {
                if (motors_enabled) {
                    disable_all_motors(ABC);
                    motors_enabled = false;
                }

                JointSnapshot snapshot{};
                // 失能状态下没有主动上报，进入手动前主动请求一次全关节反馈。
                FaultCode poll_fault = FAULT_NONE;
                if (!wait_for_mode_feedback(ABC, snapshot, poll_fault)) {
                    enter_fault(poll_fault);
                    osDelay(5);
                    continue;
                }
                if (!feedback_is_fresh(snapshot, HAL_GetTick())) {
                    enter_fault(FAULT_FEEDBACK_TIMEOUT);
                    osDelay(5);
                    continue;
                }
                if (feedback_has_error(snapshot)) {
                    enter_fault(FAULT_MOTOR_ERROR);
                    osDelay(5);
                    continue;
                }
                sync_manual_targets(manual_target, snapshot);
                if (!enable_all_at_current_position(ABC, snapshot)) {
                    enter_fault(FAULT_CAN_TX);
                    continue;
                }
                motors_enabled = true;
                manual_tx_failures = 0U;
                arm_fault_code = FAULT_NONE;
                set_state(ControlState::Manual);
            }

            JointSnapshot snapshot{};
            take_joint_snapshot(snapshot);
            if (!feedback_is_fresh(snapshot, HAL_GetTick())) {
                enter_fault(FAULT_FEEDBACK_TIMEOUT);
                continue;
            }
            if (feedback_has_error(snapshot)) {
                enter_fault(FAULT_MOTOR_ERROR);
                continue;
            }
            if (!run_manual_control(ABC, manual_target, snapshot)) {
                if (++manual_tx_failures >= 3U) enter_fault(FAULT_CAN_TX);
            } else {
                manual_tx_failures = 0U;
            }
            continue;
        }

        // requested_mode == 3：低速回到录制起点，再按记录时间轴回放。
        if (state != ControlState::ReturnToStart &&
            state != ControlState::Playback &&
            state != ControlState::PlaybackComplete) {
            if (motors_enabled) {
                disable_all_motors(ABC);
                motors_enabled = false;
            } else {
                disable_all_motors(ABC);
            }

            if (!validate_record(playback.header)) {
                enter_fault(FAULT_RECORD_INVALID);
                continue;
            }

            RecordSample first_sample{};
            read_record_sample(0U, first_sample);
            sample_to_positions(first_sample, playback.start);

            JointSnapshot snapshot{};
            FaultCode poll_fault = FAULT_NONE;
            if (!wait_for_mode_feedback(ABC, snapshot, poll_fault)) {
                enter_fault(poll_fault);
                continue;
            }
            if (!feedback_is_fresh(snapshot, HAL_GetTick())) {
                enter_fault(FAULT_FEEDBACK_TIMEOUT);
                continue;
            }
            if (feedback_has_error(snapshot)) {
                enter_fault(FAULT_MOTOR_ERROR);
                continue;
            }
            if (!enable_all_at_current_position(ABC, snapshot)) {
                enter_fault(FAULT_CAN_TX);
                continue;
            }
            motors_enabled = true;

            float max_distance = 0.0f;
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                playback.return_command[i] = snapshot.pos[i];
                playback.last_command[i] = snapshot.pos[i];
                max_distance = std::max(max_distance,
                                        std::fabs(playback.start[i] - snapshot.pos[i]));
            }
            playback.sample_index = 0U;
            playback.next_tick = HAL_GetTick();
            playback.stable_since = 0U;
            playback.tracking_error_since = 0U;
            const uint32_t expected_return_ms =
                static_cast<uint32_t>(max_distance / RETURN_SPEED * 1000.0f);
            playback.return_deadline = HAL_GetTick() + expected_return_ms + 5000U;
            arm_playback_sample_index = 0U;
            set_state(ControlState::ReturnToStart);
        }

        if (!tick_reached(HAL_GetTick(), playback.next_tick)) {
            osDelay(1);
            continue;
        }

        JointSnapshot snapshot{};
        const uint32_t playback_now = HAL_GetTick();
        take_joint_snapshot(snapshot);
        if (!feedback_is_fresh(snapshot, playback_now)) {
            enter_fault(FAULT_FEEDBACK_TIMEOUT);
            continue;
        }
        if (feedback_has_error(snapshot)) {
            enter_fault(FAULT_MOTOR_ERROR);
            continue;
        }

        if (state == ControlState::ReturnToStart) {
            if (tick_reached(playback_now, playback.return_deadline)) {
                enter_fault(FAULT_RETURN_TIMEOUT);
                continue;
            }

            // 电机明显落后时暂停软件斜坡，避免目标继续跑远。
            if (maximum_error(playback.return_command, snapshot) <= TRACKING_PAUSE_ERROR) {
                const float step = RETURN_SPEED *
                    (static_cast<float>(RECORD_PERIOD_MS) / 1000.0f);
                for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                    playback.return_command[i] =
                        move_towards(playback.return_command[i], playback.start[i], step);
                }
            }

            if (!send_joint_positions(ABC, playback.return_command, RETURN_SPEED, 1U)) {
                enter_fault(FAULT_CAN_TX);
                continue;
            }

            if (maximum_error(playback.start, snapshot) <= RETURN_TOLERANCE) {
                if (playback.stable_since == 0U) playback.stable_since = playback_now;
                if ((playback_now - playback.stable_since) >= POSITION_STABLE_MS) {
                    std::memcpy(playback.last_command, playback.start,
                                sizeof(playback.last_command));
                    playback.sample_index = 0U;
                    playback.tracking_error_since = 0U;
                    playback.stable_since = 0U;
                    set_state(ControlState::Playback);
                }
            } else {
                playback.stable_since = 0U;
            }
            schedule_next_tick(playback_now, playback.next_tick, RECORD_PERIOD_MS);
            continue;
        }

        if (state == ControlState::Playback) {
            const float tracking_error = maximum_error(playback.last_command, snapshot);
            if (tracking_error > TRACKING_PAUSE_ERROR) {
                if (playback.tracking_error_since == 0U)
                    playback.tracking_error_since = playback_now;
                if ((playback_now - playback.tracking_error_since) > TRACKING_TIMEOUT_MS) {
                    enter_fault(FAULT_TRACKING_TIMEOUT);
                    continue;
                }
                if (!send_joint_positions(ABC, playback.last_command,
                                          PLAYBACK_MAX_SPEED, 1U)) {
                    enter_fault(FAULT_CAN_TX);
                }
                schedule_next_tick(playback_now, playback.next_tick, RECORD_PERIOD_MS);
                continue;
            }
            playback.tracking_error_since = 0U;

            if (playback.sample_index < playback.header.sample_count) {
                RecordSample sample{};
                float target[JOINT_COUNT]{};
                read_record_sample(playback.sample_index, sample);
                sample_to_positions(sample, target);
                if (!positions_are_safe(target)) {
                    enter_fault(FAULT_UNSAFE_SAMPLE);
                    continue;
                }
                if (!send_joint_positions(ABC, target, PLAYBACK_MAX_SPEED, 1U)) {
                    enter_fault(FAULT_CAN_TX);
                    continue;
                }
                current_gripper_state = sample.gripper_state;
                apply_gripper_state(current_gripper_state);
                std::memcpy(playback.last_command, target, sizeof(playback.last_command));
                ++playback.sample_index;
                arm_playback_sample_index = playback.sample_index;
            } else if (tracking_error <= RETURN_TOLERANCE) {
                if (playback.stable_since == 0U) playback.stable_since = playback_now;
                if ((playback_now - playback.stable_since) >= POSITION_STABLE_MS) {
                    set_state(ControlState::PlaybackComplete);
                }
            } else {
                playback.stable_since = 0U;
            }
            schedule_next_tick(playback_now, playback.next_tick,
                               playback.header.sample_period_ms);
            continue;
        }

        // 回放结束后保持最终位置；按 START 后由前面的停止分支立即失能。
        if (state == ControlState::PlaybackComplete) {
            if (!send_joint_positions(ABC, playback.last_command, RETURN_SPEED, 1U)) {
                enter_fault(FAULT_CAN_TX);
                continue;
            }
            schedule_next_tick(playback_now, playback.next_tick, 20U);
        }
    }
}

extern "C" {

static MotorTask motorTask;

void MotorTask_Init() {
    motorTask.start((char*)"MotorTask", 1536, osPriorityNormal);
}

}
