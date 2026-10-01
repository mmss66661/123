//
// trajectory.cpp — 同步梯形轨迹实现
//

#include "../Inc/trajectory.h"

#include <cmath>

namespace arm {

void trajMakeMoveJ(JointTraj& traj,
                   const float q0[config::kJointCount],
                   const float q1[config::kJointCount],
                   float vel,
                   float acc) {
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        traj.q0[i] = q0[i];
        traj.q1[i] = q1[i];
    }
    traj.valid = false;
    traj.t_total = 0.0f;
    traj.t_acc = 0.0f;
    traj.s_vel = 0.0f;

    if (vel <= 0.0f || acc <= 0.0f) return;

    // 主导关节 = 行程最大的关节
    float dominant = 0.0f;
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        const float d = std::fabs(q1[i] - q0[i]);
        if (d > dominant) dominant = d;
    }
    if (dominant < 1e-6f) return;  // 已在目标位置

    // 梯形：行程足够则带匀速段，否则退化为三角形
    const float d_ramp = vel * vel / acc;  // 达到 vel 所需行程
    if (dominant >= d_ramp) {
        traj.t_acc = vel / acc;
        traj.t_total = traj.t_acc + dominant / vel;
        traj.s_vel = vel / dominant;  // 1 / (T - t_acc)
    } else {
        traj.t_acc = sqrtf(dominant / acc);
        traj.t_total = 2.0f * traj.t_acc;
        traj.s_vel = 1.0f / traj.t_acc;
    }
    traj.valid = true;
}

void trajSample(const JointTraj& traj, float t, float q_out[config::kJointCount]) {
    float s = 0.0f;
    if (traj.valid) {
        if (t < 0.0f) t = 0.0f;
        if (t > traj.t_total) t = traj.t_total;

        const float a_s = traj.s_vel / traj.t_acc;  // 归一化加速度 1/s²
        const float t_flat = traj.t_total - traj.t_acc;
        if (t < traj.t_acc) {
            s = 0.5f * a_s * t * t;                                  // 加速段
        } else if (t < t_flat) {
            s = 0.5f * a_s * traj.t_acc * traj.t_acc +
                traj.s_vel * (t - traj.t_acc);                       // 匀速段
        } else {
            const float t_end = traj.t_total - t;                    // 减速段
            s = 1.0f - 0.5f * a_s * t_end * t_end;
        }
    } else {
        s = 1.0f;  // 零行程：直接输出终点
    }

    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        q_out[i] = traj.q0[i] + (traj.q1[i] - traj.q0[i]) * s;
    }
}

float trajDuration(const JointTraj& traj) {
    return traj.valid ? traj.t_total : 0.0f;
}

}  // namespace arm
