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
// 2026-10-02 标定完成并逐项验证：
//   DH（CAD 轴线提取，往返校验 ~1e-16）、六个关节零位（实测）、软件限位
//   （已换算运动学坐标系）、J0~J3 方向（+1）、差速腕映射（q4=pitch 差模
//   +0.5，q5=roll 共模 -0.2577，q5 正方向为用户约定）、J3/J5 共线（ZYZ 腕）。
// moveJ/jog/moveCartesian 全部可用。
constexpr bool kDhParamsMeasured = true;

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
    //  name             can_id  dir   zero_offset   limit_lo     limit_hi    limits  max_vel
    //  零位实测 2026-10-02（CAD 零位姿态，ArmDebugTask 读取）；
    //  limit 已由电机坐标换算到运动学坐标（原值 - zero_offset）
    //  J0/J1/J2 限位为 2026-10-02 机械限位实测（运动学坐标，手扳到限位读 q）
    //  J0/J1/J2 限位 2026-10-02 实测（手扳到限位读 DM_motor_Jx.pos = 电机坐标），
    //  换算到运动学坐标（原值 - zero_offset）：J0 电机0~3.649→-1.8305~1.8185；
    //  J1 电机1.05~2.23→-0.3086~0.8714；J2 电机0~1.84→-0.6044~1.2356
    //  （曾误存为电机坐标，导致零位姿态超"限位"、回零被拒、IK 被错误钳制）
    { "J0_base_yaw",    0x02,  +1.0f, 1.83051f, -1.83051f,  1.81849f,  true,  2.0f },
    { "J1_shoulder",    0x03,  +1.0f, 1.35863f, -0.30863f,  0.87137f,  true,  2.0f },
    { "J2_elbow",       0x04,  +1.0f, 0.60445f, -0.60445f,  1.23555f,  true,  2.0f },
    { "J3_forearm_yaw", 0x05,  +1.0f, 1.98768f,  0.0f,      0.0f,      false,  2.0f },
    //  2026-10-02 实测修正：腕部语义与最初假设相反——弯曲(pitch,差模)绕 ∥J2
    //  的轴（存储 J4 线），自旋(roll,共模)绕过 TCP 的工具轴（存储 J5 线）。
    //  DH 链几何不变，仅交换两关节的控制映射/增益/限位。
    //  J4 限位：原 J45 软件限位(-2.08095~2.83818 定义在 m4-m5 上) × 0.5(物理
    //  pitch = 0.5*(m4-m5)) + 零位差值补偿 0.216485 = -0.82399 ~ 1.63557
    { "J4_wrist_pitch", 0x06,  +1.0f, 0.0f,     -0.82399f,  1.63557f,  true,  2.0f },
    { "J5_wrist_roll",  0x07,  +1.0f, 0.0f,      0.0f,      0.0f,      false, 2.0f },
};

// 差速腕参数（2026-10-02 实测：弯曲轴=存储J4线，自旋轴=存储J5线/TCP工具轴）
//   q4(pitch,弯曲) = kWristPitchGain * (Δm4 - Δm5)   差模，带机械限位
//   q5(roll,自旋)  = kWristRollGain  * (Δm4 + Δm5)   共模，无限位
// 纯 pitch 时末端角 = 单电机变化量；纯 roll 90° 时单电机变化 ~3.048 rad
// （理论常见值 0.25 对应 π，可拧 180° 复测：单电机 ~6.28 → 0.25，~5.86 → 0.2577）
constexpr float kWristPitchGain = -0.5f;    // pitch = kP * (Δm4 - Δm5)；负号经
                                             // 实机-渲染对照验证（2026-10-02 曾随
                                             // 整体回退被误退回 +0.5，现恢复）
constexpr float kWristRollGain  = -0.2577f; // roll = kR * (Δm4 + Δm5)；q5正 = 共模减小（用户规定 2026-10-02）
constexpr float kWristM4Zero    = 4.97044f;  // 腕零位时 0x06 电机反馈位置 rad
constexpr float kWristM5Zero    = 5.40341f;  // 腕零位时 0x07 电机反馈位置 rad

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
    /* J4 腕pitch   */ { 0.000000f, 0.000000f,  1.57079604f,  0.00000000f },
    /* J5 腕roll/TCP*/ { -0.118233f, 0.003499f, 0.00000000f, -2.36874442f },
};

// 使能后的安全姿态：标定零位（= CAD 零位姿态）
constexpr float kHome[kJointCount] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

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

// =========================================================================
// 6. UART1 上位机命令通道（115200 8N1，见 RobotArm/Src/arm_protocol.cpp）
// =========================================================================
constexpr uint32_t kUartTelemetryMs   = 50;    // 遥测调度周期（轮转发一帧：
                                               // STATUS/JOINTS/POSE 各 ~150ms 一次）
// 链路超时失能保护已于 2026-10-02 按用户要求移除（排查"使能1秒后失能"期间
// 用户明确要求删除）。剩余保护：反馈超时/电机报码/CAN失败/跟踪超时。
// 若需恢复：在 ArmProtocol_Tick 中重建"超时无帧 -> disable"逻辑。

}  // namespace config
}  // namespace arm

#endif  // ROBOT_ARM_ARM_CONFIG_H
