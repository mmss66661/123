//
// Created by 20852 on 2025/12/29.
//

#ifndef HERO_GIMBAL_CHASSIS_CTRL_H
#define HERO_GIMBAL_CHASSIS_CTRL_H

#pragma once
#include <cstdint>
#include "chassis_cmd.h"

enum class ChassisMode : uint8_t
{
    Stop = 0,
    FollowGimbal,
    Spin,
};

class ChassisStrategy;

class chassis_ctrl
{
public:
    void init();
    void setMode(ChassisMode mode);
    ChassisMode getMode() const;

    /* 每周期调用 */
    void update();

private:
    ChassisMode mode_{ChassisMode::Stop};
    ChassisCmd  cmd_{};

    ChassisStrategy* strategy_{nullptr};
};

#endif //HERO_GIMBAL_CHASSIS_CTRL_H