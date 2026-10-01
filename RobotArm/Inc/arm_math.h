//
// arm_math.h — 4x4 齐次矩阵 / 旋转矩阵 / 位姿换算
//
// 面向 MCU 的极简实现，无动态内存分配。
//

#ifndef ROBOT_ARM_ARM_MATH_H
#define ROBOT_ARM_ARM_MATH_H

#pragma once

#include "arm_types.h"

namespace arm {

Mat4 mat4Identity();

// c = a * b
Mat4 mat4Multiply(const Mat4& a, const Mat4& b);

// 从齐次矩阵提取位置 + RPY（ZYX 顺序）
void mat4ToPose(const Mat4& T, Pose& pose);

// 位姿 -> 齐次矩阵（R = Rz(yaw)*Ry(pitch)*Rx(roll)）
Mat4 poseToMat4(const Pose& pose);

Mat3 mat3Transpose(const Mat3& r);

// c = a * b
Mat3 mat3Multiply(const Mat3& a, const Mat3& b);

}  // namespace arm

#endif  // ROBOT_ARM_ARM_MATH_H
