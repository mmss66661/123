//
// Created by 20852 on 2025/12/29.
//

#ifndef HERO_GIMBAL_CHASSIS_SPIN_H
#define HERO_GIMBAL_CHASSIS_SPIN_H

#pragma once
#include "chassis_strategy.h"
#include "speed_pid.h"
#include <cstdint>

class chassis_spin final : public ChassisStrategy
{
public:
    chassis_spin();

    // 设置遥控器输入
    void setRCInput(float vx, float vy, float wz);

    // 设置当前云台 yaw 编码值
    void setGimbalEncoder(uint16_t enc);

    // 设置云台目标 yaw
    void setGimbalTarget(uint16_t target);

    // 更新底盘命令
    void update(ChassisCmd& cmd) override;

    // 获取 yaw 电机命令
    float getYawCmd() const { return yaw_cmd_; }

private:
    float rc_vx_ = 0.0f;
    float rc_vy_ = 0.0f;
    float rc_wz_ = 0.0f;

    uint16_t gimbal_enc_ = 0;
    uint16_t gimbal_target_ = 0;

    float yaw_cmd_ = 0.0f;

    SpeedPID yaw_pid_;
};

#endif //HERO_GIMBAL_CHASSIS_SPIN_H
