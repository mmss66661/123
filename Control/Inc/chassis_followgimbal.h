//
// Created by 20852 on 2025/12/29.
//

#ifndef HERO_GIMBAL_CHASSIS_FOLLOWGIMBAL_H
#define HERO_GIMBAL_CHASSIS_FOLLOWGIMBAL_H

#pragma once

#include "chassis_strategy.h"
#include "speed_pid.h"
#include "dbus.h"

class chassis_followgimbal final : public ChassisStrategy
{
public:
    chassis_followgimbal();

    void setRCInput(float vx, float vy) { rc_vx_ = vx; rc_vy_ = vy; }
    void setChassisEncoder(uint16_t enc) { chassis_enc_ = enc; }
    void setGimbalTarget(uint16_t target) { gimbal_target_ = target; }

    void update(ChassisCmd& cmd) override;

private:
    float rc_vx_ = 0.0f;
    float rc_vy_ = 0.0f;

    uint16_t chassis_enc_ = 0;   // 当前底盘编码
    uint16_t gimbal_target_ = 41737; // 云台绝对角度基准

    int32_t chassis_round_ = 0;   // 多圈计数

    SpeedPID yaw_pid_;
};

#endif //HERO_GIMBAL_CHASSIS_FOLLOWGIMBAL_H