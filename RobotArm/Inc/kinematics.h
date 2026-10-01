//
// kinematics.h — 正运动学 / 逆运动学
//
// 正运动学：标准 DH 链式相乘，精确解。
// 逆运动学：阻尼最小二乘（DLS）迭代数值解 —— 任意构型通用，
//          不依赖球腕等特殊几何假设，参数实测后无需重推闭式解。
//          解的选择策略与 Dummy-Robot 一致：从当前关节角出发迭代，
//          自然收敛到"距当前位形最近"的解（最小关节变化量）。
//
// 说明：Dummy-Robot 用的是球腕闭式解（8 组解取最小转角），
// 在 DH 参数确定且构型标准时可以替换为闭式实现，接口不变。
//

#ifndef ROBOT_ARM_KINEMATICS_H
#define ROBOT_ARM_KINEMATICS_H

#pragma once

#include "arm_config.h"
#include "arm_types.h"

namespace arm {

enum class IkStatus : uint8_t {
    Ok = 0,           // 收敛到容差内
    MaxIterations = 1, // 未收敛（目标不可达或构型受限），q_out 为最优逼近
    InvalidInput = 2,  // 输入包含非有限值
};

// 正运动学：q(rad, 运动学坐标系) -> 末端齐次矩阵 T（基座坐标系）
void solveFk(const float q[config::kJointCount], Mat4& T);

// 正运动学并输出位姿
void solveFkPose(const float q[config::kJointCount], Pose& pose);

// 几何雅可比：J[0..2][i] = z_i x (p_e - o_i)，J[3..5][i] = z_i
// 列索引 i 对应关节 i（i = 0..5）
void calcJacobian(const float q[config::kJointCount], float J[6][config::kJointCount]);

// 逆运动学（DLS 数值迭代）
//   target : 期望末端位姿（基座坐标系）
//   q_ref  : 迭代初值，一般传当前关节角
//   q_out  : 输出解（满足关节限位）
//   pos_error / rot_error / iterations : 可选输出，调试用
IkStatus solveIk(const Pose& target,
                 const float q_ref[config::kJointCount],
                 float q_out[config::kJointCount],
                 float* pos_error = nullptr,
                 float* rot_error = nullptr,
                 int* iterations = nullptr);

// 关节限位夹紧（has_limits=false 的关节不动）
void clampToLimits(float q[config::kJointCount]);

// 检查关节角是否在软件限位内（含容差）
bool withinLimits(const float q[config::kJointCount],
                  float tolerance = config::kLimitTolerance);

}  // namespace arm

#endif  // ROBOT_ARM_KINEMATICS_H
