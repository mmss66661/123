//
// Created by 20852 on 2025/12/29.
//

#include "../Inc/chassis_followgimbal.h"
#include "cmath"

chassis_followgimbal::chassis_followgimbal()
    : yaw_pid_(1.0f, 0.0000f, 0.00000f, 1500.0f, 50.0f)
{
}

void chassis_followgimbal::update(ChassisCmd& cmd)
{
    cmd.vx = rc_vx_;
    cmd.vy = rc_vy_;

    /* ---------- 多圈处理 ---------- */
    static uint16_t prev_chassis_enc = chassis_enc_;
    if (chassis_enc_ < 10000 && prev_chassis_enc > 55000) chassis_round_++;
    else if (chassis_enc_ > 55000 && prev_chassis_enc < 10000) chassis_round_--;
    prev_chassis_enc = chassis_enc_;

    int32_t chassis_total =
        static_cast<int32_t>(chassis_enc_) + chassis_round_ * 65536;

    /* ---------- 位置误差 ---------- */
    int32_t diff = static_cast<int32_t>(gimbal_target_) - chassis_total;
    if (diff > 32768) diff -= 65536;
    else if (diff < -32768) diff += 65536;

    /* ---------- 区域参数（修正） ---------- */
    const int32_t deadzone  = 700;
    const int32_t slow_zone = 4000;   // 必须 > deadzone

    /* ---------- 动态目标点 ---------- */
    int32_t target_enc = gimbal_target_;
    if (abs(diff) < slow_zone) {
        if (diff > 0)
            target_enc = gimbal_target_ - deadzone;
        else
            target_enc = gimbal_target_ + deadzone;
    }

    int32_t new_diff = target_enc - chassis_total;
    if (new_diff > 32768) new_diff -= 65536;
    else if (new_diff < -32768) new_diff += 65536;

    /* ---------- 云台角速度前馈（关键） ---------- */
    static int32_t last_gimbal_target = gimbal_target_;
    int32_t delta_gimbal = gimbal_target_ - last_gimbal_target;
    if (delta_gimbal > 32768) delta_gimbal -= 65536;
    else if (delta_gimbal < -32768) delta_gimbal += 65536;
    last_gimbal_target = gimbal_target_;

    float gimbal_wz_ff = delta_gimbal * 0.001f;   // 前馈比例，可调 0.005~0.02


    /* ---------- 输出 ---------- */
    if (abs(new_diff) < deadzone) {
        yaw_pid_.Clear();
        cmd.wz = gimbal_wz_ff;    // 仍然跟随云台转速
    } else {
        float wz_pid = yaw_pid_.Calculate(0.0f, -new_diff, 0.01f);
        cmd.wz = wz_pid + gimbal_wz_ff;
    }
}
