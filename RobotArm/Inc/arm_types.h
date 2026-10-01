//
// arm_types.h — 机械臂框架公共类型
//
// 关节角约定：q[0..5] 一律使用"运动学坐标系"（零位 + 正方向），
// 与电机原始反馈坐标的换算统一放在 ArmHal 层（见 arm_config.h）。
//

#ifndef ROBOT_ARM_ARM_TYPES_H
#define ROBOT_ARM_ARM_TYPES_H

#pragma once

#include <cstddef>
#include <cstdint>

namespace arm {

constexpr std::size_t kJointCount = 6;

// 4x4 齐次变换矩阵（行优先，基座坐标系 -> 末端坐标系）
struct Mat4 {
    float m[4][4];
};

// 3x3 旋转矩阵
struct Mat3 {
    float m[3][3];
};

// 末端位姿：基座坐标系下的位置 + RPY 欧拉角
// 旋转按 R = Rz(yaw) * Ry(pitch) * Rx(roll) 组合（ZYX 顺序）
struct Pose {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

// 关节角数组（rad，运动学坐标系）
using JointVec = float[kJointCount];

}  // namespace arm

#endif  // ROBOT_ARM_ARM_TYPES_H
