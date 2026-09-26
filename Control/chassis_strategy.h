//
// Created by 20852 on 2025/12/29.
//

#ifndef HERO_GIMBAL_CHASSIS_STRATEGY_H
#define HERO_GIMBAL_CHASSIS_STRATEGY_H

#pragma once
#include "chassis_cmd.h"

class ChassisStrategy
{
public:
    virtual ~ChassisStrategy() = default;

    //周期生成底盘运动指令
    virtual void update(ChassisCmd& cmd) = 0;
};

#endif //HERO_GIMBAL_CHASSIS_STRATEGY_H