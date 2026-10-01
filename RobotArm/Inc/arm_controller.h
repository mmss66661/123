//
// arm_controller.h — 机械臂顶层控制器
//
// 分层（参考 Dummy-Robot 的 Robot 核心库结构）：
//   armTask（FreeRTOS 任务/HMI）
//     -> ArmController（状态机 / 指令队列 / 安全监控）   ← 本文件
//       -> trajectory（同步梯形轨迹 MoveJ）
//       -> kinematics（DH 正解 + DLS 数值逆解）
//       -> ArmHal（电机使能/命令/反馈、差速腕耦合、夹爪）
//
// 使用模式（对齐 Dummy-Robot 的三种命令模式）：
//   moveJ(...)        INT 立即模式：打断当前轨迹，从当前指令角出发
//   moveJQueue(...)   SEQ 顺序模式：进入队列，按序执行
//   moveCartesian(...) 末端位姿目标，内部先解 IK 再走 MoveJ
//
// tick(now) 由任务循环周期调用（周期不要求严格，轨迹按真实时间采样）。
//

#ifndef ROBOT_ARM_ARM_CONTROLLER_H
#define ROBOT_ARM_ARM_CONTROLLER_H

#pragma once

#include "arm_config.h"
#include "arm_types.h"
#include "kinematics.h"
#include "trajectory.h"

namespace arm {

enum class CtrlState : uint8_t {
    Disabled = 0,  // 失能（上电默认）
    Ready = 1,     // 使能，保持当前位置，可接收运动指令
    Moving = 2,    // 轨迹执行中
    Fault = 3,     // 故障锁存（电机已失能），需 disable() 后重新 enable()
};

enum class RequestStatus : uint8_t {
    Ok = 0,
    NotEnabled = 1,       // 未使能就请求运动
    NotCalibrated = 2,    // DH 参数未标定，笛卡尔控制不可用
    IkFailed = 3,         // 逆解未收敛（目标不可达）
    InvalidTarget = 4,    // 目标超限或含非法数值
    QueueFull = 5,
};

enum class CtrlFault : uint32_t {
    None = 0,
    FeedbackTimeout = 1,  // 任一关节反馈超时
    MotorError = 2,       // 电机报码（达妙状态 8~E）
    CanTx = 3,            // CAN 连续发送失败
    TrackingTimeout = 4,  // 跟踪误差暂停超时
    LimitViolation = 5,   // 轨迹采样点超出软件限位
};

class ArmController {
public:
    // 上电初始化：清空队列与轨迹，处于 Disabled
    void reset();

    // 使能：轮询反馈 -> 在当前位置使能并保持。失败返回 false 且进入 Fault。
    bool enable(uint32_t now);

    // 主动失能（急停/正常下电），同时清除故障锁存
    void disable(uint32_t now);

    // ---- 运动指令（均非阻塞，立即返回） ----
    RequestStatus moveJ(const float q[config::kJointCount], uint32_t now);
    RequestStatus moveJQueue(const float q[config::kJointCount], uint32_t now);
    RequestStatus moveCartesian(const Pose& target, uint32_t now);
    RequestStatus jog(uint8_t joint, float delta, uint32_t now);

    void setGripper(bool closed);

    // 主循环心跳
    void tick(uint32_t now);

    // ---- 观测接口 ----
    CtrlState state() const { return state_; }
    CtrlFault fault() const { return fault_; }
    float trajProgress() const;                  // 0..1
    void getCurrentJoints(float q[config::kJointCount]) const;   // 反馈角（运动学）
    void getCommandedJoints(float q[config::kJointCount]) const; // 指令角
    bool getCurrentPose(Pose& out);              // FK(反馈角)
    float trackingError() const { return tracking_error_; }

private:
    RequestStatus validateTarget(const float q[config::kJointCount]) const;
    void enterFault(CtrlFault fault, uint32_t now);
    void startTraj(const float q_target[config::kJointCount], uint32_t now);
    bool checkFeedback(uint32_t now);

    CtrlState state_ = CtrlState::Disabled;
    CtrlFault fault_ = CtrlFault::None;

    JointTraj traj_;
    uint32_t traj_start_ms_ = 0;
    uint32_t traj_pause_ms_ = 0;   // 跟踪暂停累计的时间轴补偿
    uint32_t last_tick_ms_ = 0;
    bool paused_ = false;
    uint32_t pause_since_ = 0;

    float q_cmd_[config::kJointCount]{};       // 最后下发的指令角
    float q_measured_[config::kJointCount]{};  // 最新反馈角
    float tracking_error_ = 0.0f;

    float queue_[config::kQueueDepth][config::kJointCount];
    std::size_t queue_head_ = 0;
    std::size_t queue_count_ = 0;

    uint32_t last_hold_ms_ = 0;
    uint32_t arrive_since_ = 0;
    uint32_t tx_fail_count_ = 0;
    RequestStatus last_request_ = RequestStatus::Ok;
};

// 全局单例（armTask 使用；上位机测试可自行实例化）
extern ArmController armController;

// ---- 调试变量（volatile，调试器直接观察，沿用工程现有风格） ----
extern volatile uint8_t  arm_fw_state;        // CtrlState
extern volatile uint32_t arm_fw_fault;        // CtrlFault
extern volatile uint8_t  arm_fw_request;      // 最近一次请求的 RequestStatus
extern volatile int32_t  arm_fw_ik_iters;     // 最近一次 IK 迭代次数
extern volatile float    arm_fw_ik_err_pos;   // m
extern volatile float    arm_fw_ik_err_rot;   // rad
extern volatile float    arm_fw_traj_progress;
extern volatile float    arm_fw_tracking_err; // rad

}  // namespace arm

#endif  // ROBOT_ARM_ARM_CONTROLLER_H
