//
// arm_hal_dm.cpp — 执行器抽象层的达妙(DM)电机实现
//
// 对接现有 Bsp 层：bsp_can 的 DM 命令接口、DM_motor_J0~J5 反馈结构体、
// TIM1 夹爪 PWM。反馈轮询逻辑移植自 motorTask.cpp 的 poll_disabled_feedback
// （达妙为一发一收协议，失能态需逐台发 0x7FF/0xCC 刷新请求，旧固件回退
// 用失能命令取反馈）。
//

#include "../Inc/arm_hal.h"

#include <cmath>

#include "bsp_can.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

// 定义在 bsp_can.cpp
extern DM_motor_info DM_motor_J0;
extern DM_motor_info DM_motor_J1;
extern DM_motor_info DM_motor_J2;
extern DM_motor_info DM_motor_J3;
extern DM_motor_info DM_motor_J4;
extern DM_motor_info DM_motor_J5;

namespace arm {
namespace hal {
namespace {

DM_motor_info* const kMotors[config::kJointCount] = {
    &DM_motor_J0, &DM_motor_J1, &DM_motor_J2,
    &DM_motor_J3, &DM_motor_J4, &DM_motor_J5
};

// 六台电机的发送 CAN ID（J0~J3 来自 kJoints，腕部两台为 0x06/0x07）
constexpr uint16_t kMotorCanId(std::size_t i) {
    return config::kJoints[i].can_id;
}

bsp_can& canBus() {
    static bsp_can instance;
    return instance;
}

constexpr uint32_t kPollTimeoutMs = 20;   // 单台电机反馈等待
constexpr uint32_t kPollRetries = 2;      // 单台电机重试次数
constexpr uint32_t kFrameGapMs = 2;       // 相邻 CAN 命令帧间隔（沿用现有经验值）

void snapshotLocked(MotorSnapshot& snap) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        snap.pos[i] = kMotors[i]->pos;
        snap.err[i] = kMotors[i]->err;
        snap.last_tick[i] = kMotors[i]->last_update_tick;
        snap.rx_count[i] = kMotors[i]->rx_count;
    }
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

bool waitJointFeedback(std::size_t joint, uint32_t rx_before) {
    const uint32_t deadline = HAL_GetTick() + kPollTimeoutMs;
    do {
        MotorSnapshot snap;
        snapshotLocked(snap);
        if (snap.rx_count[joint] != rx_before) return true;
        osDelay(1);
    } while (static_cast<int32_t>(HAL_GetTick() - deadline) < 0);
    return false;
}

}  // namespace

void init() {
    // CAN 初始化由 main.c 的 BSP_CAN_Init() 完成，无需重复。
}

bool pollFeedback(MotorSnapshot& snap) {
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        bool updated = false;
        for (uint32_t retry = 0U; retry < kPollRetries && !updated; ++retry) {
            MotorSnapshot before;
            snapshotLocked(before);

            if (canBus().BSP_CAN1_DMMotorRefreshStatusCmd(kMotorCanId(i)) == HAL_OK) {
                updated = waitJointFeedback(i, before.rx_count[i]);
            }
            if (!updated) {
                // 兼容不支持 0x7FF/0xCC 的旧固件：失能命令同样会触发一帧反馈
                snapshotLocked(before);
                if (canBus().BSP_CAN1_DMMotorDisableCmd(kMotorCanId(i), DM_POS_MODE) == HAL_OK) {
                    updated = waitJointFeedback(i, before.rx_count[i]);
                }
            }
        }
        if (!updated) return false;
    }
    snapshotLocked(snap);
    return true;
}

bool enableAllAt(const float motor_pos[config::kJointCount], float vel_limit) {
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (canBus().BSP_CAN1_DMMotorEnableCmd(kMotorCanId(i), DM_POS_MODE) != HAL_OK) {
            return false;
        }
        osDelay(kFrameGapMs);
        if (canBus().BSP_CAN1_DMMotorPositionCmd(kMotorCanId(i), motor_pos[i],
                                                 vel_limit) != HAL_OK) {
            return false;
        }
        osDelay(kFrameGapMs);
    }
    return true;
}

bool disableAll() {
    bool ok = true;
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        ok = (canBus().BSP_CAN1_DMMotorDisableCmd(kMotorCanId(i), DM_POS_MODE) == HAL_OK) && ok;
        osDelay(kFrameGapMs);
    }
    return ok;
}

bool sendJointTargets(const float q[config::kJointCount], float vel_limit) {
    float motor[config::kJointCount];
    kinToMotor(q, motor);
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (!std::isfinite(motor[i]) ||
            motor[i] < DM_P_MIN || motor[i] > DM_P_MAX) {
            return false;
        }
        if (canBus().BSP_CAN1_DMMotorPositionCmd(kMotorCanId(i), motor[i],
                                                 vel_limit) != HAL_OK) {
            return false;
        }
        osDelay(kFrameGapMs);
    }
    return true;
}

void setGripper(bool closed) {
    if (closed) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, config::kGripperCloseCh1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, config::kGripperCloseCh3);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, config::kGripperOpenCh1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, config::kGripperOpenCh3);
    }
}

void readMotors(MotorSnapshot& snap) {
    snapshotLocked(snap);
}

void motorToKin(const float motor_pos[config::kJointCount],
                float q[config::kJointCount]) {
    for (std::size_t i = 0; i < 4; ++i) {
        q[i] = config::kJoints[i].direction * (motor_pos[i] - config::kJoints[i].zero_offset);
    }
    const float dm4 = motor_pos[4] - config::kWristM4Zero;
    const float dm5 = motor_pos[5] - config::kWristM5Zero;
    q[4] = config::kWristRollGain * (dm4 + dm5);
    q[5] = config::kWristPitchGain * (dm4 - dm5);
}

void kinToMotor(const float q[config::kJointCount],
                float motor_out[config::kJointCount]) {
    for (std::size_t i = 0; i < 4; ++i) {
        motor_out[i] = config::kJoints[i].zero_offset +
                       q[i] / config::kJoints[i].direction;
    }
    const float roll = q[4] / config::kWristRollGain;
    const float pitch = q[5] / config::kWristPitchGain;
    motor_out[4] = config::kWristM4Zero + 0.5f * (roll + pitch);
    motor_out[5] = config::kWristM5Zero + 0.5f * (roll - pitch);
}

}  // namespace hal
}  // namespace arm
