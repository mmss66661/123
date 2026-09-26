//
// Created by 20852 on 2025/12/29.
//

#ifndef HERO_GIMBAL_CHASSIS_STOP_H
#define HERO_GIMBAL_CHASSIS_STOP_H

#pragma once
#include "chassis_strategy.h"

class chassis_stop final : public ChassisStrategy
{
public:
    void update(ChassisCmd& cmd) override;
};


#endif //HERO_GIMBAL_CHASSIS_STOP_H