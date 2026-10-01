//
// arm_config.h — 机械臂全部可调参数（唯一的标定入口）
//
// 所有标注 [未测量] 的量都是占位值，实测后只改本文件即可，
// 其余模块一律从这里取参数。标定流程见 Doc/机械臂控制框架说明.md。
//
// 已实测的数据（关节 CAN ID、J0~J2/J5 限位、差速腕定义、夹爪 PWM）
// 沿用 motorTask.cpp / 工作日志中的现值。
//

#ifndef ROBOT_ARM_ARM_CONFIG_H
#define ROBOT_ARM_ARM_CONFIG_H

#pragma once

#include <cstddef>
#include <cstdint>

namespace arm {
namespace config {

constexpr std::size_t kJointCount = 6;

// =========================================================================
// 1. 标定状态总开关
// =========================================================================
// DH 参数与零位尚未实测时保持 false：
//   - moveJ / jog / 录制回放原有功能不受影响（关节空间控制不依赖几何参数）
//   - moveCartesian（末端位姿控制）会直接返回 NotCalibrated
// 测量填好第 3 节 DH 表和第 2 节零位后改为 true。
constexpr bool kDhParamsMeasured = false;

// =========================================================================
// 2. 关节-电机映射与软件限位
// =========================================================================
// 运动学关节 0..5 与 CAN1 达妙电机的对应关系（ID 为实测值）。
//
// J0~J3 单电机关节：  q = direction * (motor_pos - zero_offset)
// J4/J5 差速腕（0x06/0x07 两电机耦合，两轴平行、方向相反）：
//   roll  = kWristRollGain  * ((m4 - kWristM4Zero) + (m5 - kWristM5Zero))
//   pitch = kWristPitchGain * ((m4 - kWristM4Zero) - (m5 - kWristM5Zero))
//   （与现有手动控制中 pitch = m4 - m5 的定义一致）
struct JointConfig {
    const char* name;     // 关节名（调试/日志用）
    uint16_t can_id;      // 达妙电机 CAN ID（J0~J3 单电机；J4/J5 = 差速腕电机 0x06/0x07）
    float direction;      // +1/-1：运动学正方向相对电机正方向（J4/J5 不使用）
    float zero_offset;    // [未测量] 零位时的电机反馈位置 rad（J4/J5 不使用）
    float limit_lo;       // 运动学坐标系下的软件限位 rad
    float limit_hi;
    bool has_limits;      // false = 无机械限位（仅受电机行程约束）
    float max_vel;        // 单关节最大速度 rad/s
};

constexpr JointConfig kJoints[kJointCount] = {
    //  name             can_id  dir   zero_offset  limit_lo     limit_hi     limits  max_vel
    { "J0_base_yaw",    0x02,  +1.0f, 0.0f, 1.72713089f, 5.405096f,    true,  2.0f },
    { "J1_shoulder",    0x03,  +1.0f, 0.0f, 1.05001163f, 2.27759933f,  true,  2.0f },
    { "J2_elbow",       0x04,  +1.0f, 0.0f, 0.00324249f, 1.84920311f,  true,  2.0f },
    { "J3_forearm_yaw", 0x05,  +1.0f, 0.0f, 0.0f,        0.0f,         false, 2.0f },
    { "J4_wrist_roll",  0x06,  +1.0f, 0.0f, 0.0f,        0.0f,         false, 2.0f },
    { "J5_wrist_pitch", 0x07,  +1.0f, 0.0f, -2.08094978f, 2.83817816f, true,  2.0f },
};

// 差速腕参数（J4/J5 与 0x06/0x07 两电机的合成关系）
constexpr float kWristRollGain  = 0.5f;  // [未测量] 共模(和) -> roll 传动比例
constexpr float kWristPitchGain = 1.0f;  // 差模(差) -> pitch，现有手动控制按 m4-m5 定义
constexpr float kWristM4Zero    = 0.0f;  // [未测量] 腕零位时 0x06 电机反馈位置 rad
constexpr float kWristM5Zero    = 0.0f;  // [未测量] 腕零位时 0x07 电机反馈位置 rad

// =========================================================================
// 3. DH 参数（标准 DH 约定，全部 [未测量] 占位值）
// =========================================================================
// A_i = Rz(theta_i + theta_offset) * Tz(d) * Tx(a) * Rx(alpha)
// 基座坐标系原点在 J0 轴线上，q 全为零时为标定零位。
//
// 下表是典型"垂直基座 + 球腕"六轴构型的占位值，仅用于框架联调。
// 实测方法：量出各连杆的轴向长度(沿关节轴)和偏置(垂直关节轴)后逐项替换，
// 并把各关节的零位偏差写入 theta_offset。
struct DhRow {
    float d;            // 连杆偏移 m（沿 zi 轴）
    float a;            // 连杆长度 m（沿 xi 轴）
    float alpha;        // 连杆扭角 rad（绕 xi 轴）
    float theta_offset; // 关节零位偏置 rad
};

// 来源：2026-10-01 由 Fusion 零位轴线 + TCP 提取（tools/fusion_export_axes.py
// → tools/dh_from_axes.py），往返校验 5e-16。
// 注意：theta_offset 对应"CAD 截取时的姿态"。实物零位标定时必须把机械臂
// 摆成与 CAD 截图相同的姿态，再记录电机位置作为 zero_offset。
// TCP 相对腕轴横向偏距 3.5mm、轴向 -118.2mm（J5 正方向指向臂身一侧）。
// FK(0) 末端位置（基座系, m）：(0.278352, -0.333913, 0.390722)，供测试核对
constexpr DhRow kDh[kJointCount] = {
    /* J0 底部yaw   */ { 0.037620f, 0.000000f,  1.57079633f,  1.57079633f },
    /* J1 大臂pitch */ { 0.000000f, 0.210999f,  1.57079633f,  2.35619449f },
    /* J2 肘        */ { 0.016000f, 0.000000f,  1.57079633f,  2.35619464f },
    /* J3 前臂      */ { 0.272900f, 0.000000f,  1.57079662f, -3.13697835f },
    /* J4 腕roll    */ { 0.000000f, 0.000000f,  1.57079604f,  0.00000000f },
    /* J5 法兰/TCP  */ { -0.118233f, 0.003499f, 0.00000000f, -2.36874442f },
};

// 使能后的安全姿态（默认取各限位中点；零位标定后可改为标定零位）
constexpr float kHome[kJointCount] = { 3.566f, 1.664f, 0.926f, 0.0f, 0.0f, 0.379f };

// =========================================================================
// 4. 轨迹规划与 IK 参数
// =========================================================================
constexpr float kTrajDefaultVel  = 1.0f;  // rad/s  主导关节最大速度
constexpr float kTrajDefaultAcc  = 2.5f;  // rad/s² 主导关节加速度
constexpr float kSendVelLimit    = 2.0f;  // rad/s  下发电机速度限幅
constexpr float kHoldVelLimit    = 0.3f;  // rad/s  保持/到位段速度限幅
constexpr float kArriveTolerance = 0.04f; // rad    到位判定阈值
constexpr uint32_t kArriveStableMs = 300; // 到位需持续的时间
constexpr uint32_t kHoldResendMs    = 50; // Ready 态保持命令重发周期
constexpr uint32_t kCanTxFailLimit  = 3;  // 连续发送失败次数上限

// IK：LM 阻尼最小二乘迭代（数值解，任意构型通用；零位标定后同样适用）
constexpr int   kIkMaxIter  = 150;
constexpr float kIkTolPos   = 5e-4f;  // m
constexpr float kIkTolRot   = 5e-3f;  // rad
constexpr float kIkLam0     = 0.10f;  // LM 初始阻尼 λ0（迭代中自适应升降）
constexpr float kIkMaxDq    = 0.30f;  // 单次迭代关节步长限幅 rad

// =========================================================================
// 5. 安全参数（沿用 motorTask 实测经验值）
// =========================================================================
constexpr uint32_t kFeedbackTimeoutMs   = 100;   // 反馈超时 -> 失能
constexpr uint32_t kMotorStartupDelayMs = 2000;  // 达妙上电后再发首帧命令
constexpr float    kTrackingPauseError  = 0.35f; // 跟踪误差过大 -> 暂停时间轴
constexpr uint32_t kTrackingTimeoutMs   = 3000;  // 暂停持续超时 -> 失能
constexpr float    kLimitTolerance      = 0.05f; // 软件限位容差

// 顺序模式指令队列深度
constexpr std::size_t kQueueDepth = 8;

// 夹爪 PWM 比较值（TIM1，沿用 motorTask 实测值）
constexpr uint32_t kGripperCloseCh1 = 2000;
constexpr uint32_t kGripperCloseCh3 = 1700;
constexpr uint32_t kGripperOpenCh1  = 2500;
constexpr uint32_t kGripperOpenCh3  = 1200;

}  // namespace config
}  // namespace arm

#endif  // ROBOT_ARM_ARM_CONFIG_H
