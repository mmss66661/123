//
// arm_hal.h — 执行器抽象层接口
//
// 控制器只认识"运动学关节角 q[6]"，电机的使能/命令/反馈、差速腕耦合、
// 夹爪 PWM 全部封装在这一层。换电机（如换瓴控/大疆）只需重写实现文件，
// 控制器与运动学代码不动。
//
// 默认实现 arm_hal_dm.cpp：对接现有 bsp_can 的达妙(DM)电机接口。
// 上位机单元测试可用桩实现替换（见 test/ 目录）。
//

#ifndef ROBOT_ARM_ARM_HAL_H
#define ROBOT_ARM_ARM_HAL_H

#pragma once

#include "arm_config.h"
#include "arm_types.h"

namespace arm {
namespace hal {

// 六台电机的原始反馈快照（关中断拷贝，电机坐标系）
struct MotorSnapshot {
    float pos[config::kJointCount];      // rad，电机反馈位置
    uint8_t err[config::kJointCount];    // 达妙状态码：0失能 1使能 8~E故障
    uint32_t last_tick[config::kJointCount];
    uint32_t rx_count[config::kJointCount];
};

// 初始化（CAN 已在 main.c 的 BSP_CAN_Init() 完成，此处预留）
void init();

// 逐台主动请求反馈（达妙为一发一收，失能态不会主动上报）
// 返回 false = 有电机超时未响应
bool pollFeedback(MotorSnapshot& snap);

// 使能全部电机并在当前位置保持
bool enableAllAt(const float motor_pos[config::kJointCount], float vel_limit);

// 全部失能（急停/故障/正常下电共用）
bool disableAll();

// 下发六个运动学关节角（内部完成差速腕耦合与零位换算）
bool sendJointTargets(const float q[config::kJointCount], float vel_limit);

// 夹爪：closed=true 闭合
void setGripper(bool closed);

// 关中断读取反馈快照
void readMotors(MotorSnapshot& snap);

// 电机反馈坐标 -> 运动学关节坐标
void motorToKin(const float motor_pos[config::kJointCount],
                float q[config::kJointCount]);

// 运动学关节坐标 -> 电机命令坐标
void kinToMotor(const float q[config::kJointCount],
                float motor_out[config::kJointCount]);

}  // namespace hal
}  // namespace arm

#endif  // ROBOT_ARM_ARM_HAL_H
