//
// Created by 20852 on 2025/12/29.
//

#include "../Inc/chassis_spin.h"
#include "cmath"
#include "bsp_can.h"

// 构造函数
chassis_spin::chassis_spin()
    : yaw_pid_(1.5f, 0.08f, 0.0f, 600.0f, 50.0f) // PID 参数，可调
{
}

// 设置遥控器输入
void chassis_spin::setRCInput(float vx, float vy, float wz)
{
    rc_vx_ = vx;
    rc_vy_ = vy;
    rc_wz_ = wz;
}

// 设置当前云台 yaw 编码值
void chassis_spin::setGimbalEncoder(uint16_t enc)
{
    gimbal_enc_ = enc;
}

// 设置云台目标 yaw
void chassis_spin::setGimbalTarget(uint16_t target)
{
    gimbal_target_ = target;
}

// 更新底盘命令
void chassis_spin::update(ChassisCmd& cmd)
{
    // 1. XY 平面速度直接映射
    cmd.vx = rc_vx_;
    cmd.vy = rc_vy_;

    // 2. 底盘旋转速度 WZ 直接映射遥控器输入
    cmd.wz = rc_wz_;

    // 3. 云台 yaw 保持策略
    int32_t diff = static_cast<int32_t>(gimbal_target_) - gimbal_enc_;

    // 处理环绕 [-32768, 32767]
    if (diff > 32768) diff -= 65536;
    else if (diff < -32768) diff += 65536;

    const int32_t deadzone = 1200; // 编码死区

    if (abs(diff) < deadzone) {
        yaw_pid_.Clear();
        yaw_cmd_ = 0;
    } else {
        float dt = 0.01f; // 任务周期
        // 前馈补偿底盘旋转
        yaw_cmd_ = yaw_pid_.Calculate(0.0f, -diff, dt) - cmd.wz;
    }
}
