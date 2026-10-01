//
// trajectory.h — 关节空间同步梯形轨迹（MoveJ）
//
// 策略与 Dummy-Robot 的 MoveJ 相同：
//   以行程最大的关节为主导关节，按其 v/a 限制计算总时长，
//   其余关节使用同一个归一化时间标尺 s(t) ∈ [0,1] 等比缩放，
//   保证六个关节同时起步、同时到位。
//

#ifndef ROBOT_ARM_TRAJECTORY_H
#define ROBOT_ARM_TRAJECTORY_H

#pragma once

#include "arm_config.h"

namespace arm {

struct JointTraj {
    float q0[config::kJointCount];  // 起点（运动学坐标）
    float q1[config::kJointCount];  // 终点
    float t_total;                  // 总时长 s
    float t_acc;                    // 加（减）速段时间 s
    float s_vel;                    // 归一化速度峰值 1/s
    bool valid;                     // false = 未初始化/零行程
};

// 生成从 q0 到 q1 的同步梯形轨迹
//   vel / acc : 主导关节的速度/加速度限制 rad/s, rad/s²
void trajMakeMoveJ(JointTraj& traj,
                   const float q0[config::kJointCount],
                   const float q1[config::kJointCount],
                   float vel = config::kTrajDefaultVel,
                   float acc = config::kTrajDefaultAcc);

// 按时刻 t 采样（t 会被自动钳位到 [0, t_total]）
void trajSample(const JointTraj& traj, float t, float q_out[config::kJointCount]);

// 轨迹总时长 s
float trajDuration(const JointTraj& traj);

}  // namespace arm

#endif  // ROBOT_ARM_TRAJECTORY_H
