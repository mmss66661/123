//
// Created by 20852 on 2025/12/29.
//

#ifndef HERO_GIMBAL_CHASSIS_CMD_H
#define HERO_GIMBAL_CHASSIS_CMD_H

#pragma once

struct ChassisCmd
{
    float vx;   // 前进速度
    float vy;   // 平移速度
    float wz;   // 角速度
};

#endif //HERO_GIMBAL_CHASSIS_CMD_H