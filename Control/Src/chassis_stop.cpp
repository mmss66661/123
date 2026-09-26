//
// Created by 20852 on 2025/12/29.
//
#include "../Inc/chassis_stop.h"

void chassis_stop::update(ChassisCmd& cmd)
{
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.wz = 0.0f;
}