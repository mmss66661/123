//
// arm_controller.cpp — 顶层控制器实现
//

#include "../Inc/arm_controller.h"

#include "../Inc/arm_hal.h"

#include <cmath>
#include <cstring>

namespace arm {

ArmController armController;

volatile uint8_t  arm_fw_state = 0;
volatile uint32_t arm_fw_fault = 0;
volatile uint8_t  arm_fw_request = 0;
volatile int32_t  arm_fw_ik_iters = 0;
volatile float    arm_fw_ik_err_pos = 0.0f;
volatile float    arm_fw_ik_err_rot = 0.0f;
volatile float    arm_fw_traj_progress = 0.0f;
volatile float    arm_fw_tracking_err = 0.0f;

void ArmController::reset() {
    state_ = CtrlState::Disabled;
    fault_ = CtrlFault::None;
    traj_ = JointTraj{};
    traj_start_ms_ = 0;
    traj_pause_ms_ = 0;
    last_tick_ms_ = 0;
    paused_ = false;
    pause_since_ = 0;
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        q_cmd_[i] = 0.0f;
        q_measured_[i] = 0.0f;
    }
    queue_head_ = 0;
    queue_count_ = 0;
    last_hold_ms_ = 0;
    arrive_since_ = 0;
    tx_fail_count_ = 0;
    tracking_error_ = 0.0f;
    last_request_ = RequestStatus::Ok;

    arm_fw_state = static_cast<uint8_t>(state_);
    arm_fw_fault = static_cast<uint32_t>(fault_);
    arm_fw_request = static_cast<uint8_t>(last_request_);
    arm_fw_traj_progress = 0.0f;
}

bool ArmController::enable(uint32_t now) {
    (void)now;
    if (state_ == CtrlState::Ready || state_ == CtrlState::Moving) return true;

    // 失能态电机不会主动上报，先逐台轮询反馈
    hal::MotorSnapshot snap;
    if (!hal::pollFeedback(snap)) {
        enterFault(CtrlFault::FeedbackTimeout, now);
        return false;
    }
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (snap.err[i] >= 8U) {
            enterFault(CtrlFault::MotorError, now);
            return false;
        }
    }

    hal::motorToKin(snap.pos, q_cmd_);
    std::memcpy(q_measured_, q_cmd_, sizeof(q_measured_));  // NOLINT

    if (!hal::enableAllAt(snap.pos, config::kHoldVelLimit)) {
        enterFault(CtrlFault::CanTx, now);
        return false;
    }

    fault_ = CtrlFault::None;
    arm_fw_fault = 0;
    state_ = CtrlState::Ready;
    last_hold_ms_ = now;
    tx_fail_count_ = 0;
    arm_fw_state = static_cast<uint8_t>(state_);
    return true;
}

void ArmController::disable(uint32_t now) {
    (void)now;
    hal::disableAll();
    state_ = CtrlState::Disabled;
    fault_ = CtrlFault::None;
    queue_count_ = 0;
    arrive_since_ = 0;
    traj_ = JointTraj{};
    arm_fw_state = static_cast<uint8_t>(state_);
    arm_fw_fault = 0;
}

RequestStatus ArmController::validateTarget(const float q[config::kJointCount]) const {
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (!std::isfinite(q[i])) return RequestStatus::InvalidTarget;
    }
    if (!withinLimits(q)) return RequestStatus::InvalidTarget;
    return RequestStatus::Ok;
}

void ArmController::enterFault(CtrlFault fault, uint32_t now) {
    (void)now;
    hal::disableAll();
    state_ = CtrlState::Fault;
    fault_ = fault;
    queue_count_ = 0;
    traj_ = JointTraj{};
    arm_fw_state = static_cast<uint8_t>(state_);
    arm_fw_fault = static_cast<uint32_t>(fault_);
}

void ArmController::startTraj(const float q_target[config::kJointCount], uint32_t now) {
    trajMakeMoveJ(traj_, q_cmd_, q_target);
    traj_start_ms_ = now;
    traj_pause_ms_ = 0;
    last_tick_ms_ = now;
    paused_ = false;
    pause_since_ = 0;
    arrive_since_ = 0;
    state_ = CtrlState::Moving;
    arm_fw_state = static_cast<uint8_t>(state_);
}

RequestStatus ArmController::moveJ(const float q[config::kJointCount], uint32_t now) {
    last_request_ = validateTarget(q);
    arm_fw_request = static_cast<uint8_t>(last_request_);
    if (last_request_ != RequestStatus::Ok) return last_request_;
    if (state_ != CtrlState::Ready && state_ != CtrlState::Moving) {
        last_request_ = RequestStatus::NotEnabled;
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }
    queue_count_ = 0;  // 立即模式清空排队目标
    startTraj(q, now);
    return last_request_;
}

RequestStatus ArmController::moveJQueue(const float q[config::kJointCount], uint32_t now) {
    last_request_ = validateTarget(q);
    if (last_request_ != RequestStatus::Ok) {
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }
    if (state_ != CtrlState::Ready && state_ != CtrlState::Moving) {
        last_request_ = RequestStatus::NotEnabled;
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }

    if (state_ == CtrlState::Ready && queue_count_ == 0) {
        startTraj(q, now);  // 空闲时直接启动
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }

    if (queue_count_ >= config::kQueueDepth) {
        last_request_ = RequestStatus::QueueFull;
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }
    const std::size_t slot = (queue_head_ + queue_count_) % config::kQueueDepth;
    std::memcpy(queue_[slot], q, sizeof(queue_[slot]));  // NOLINT
    ++queue_count_;
    arm_fw_request = static_cast<uint8_t>(last_request_);
    return last_request_;
}

RequestStatus ArmController::moveCartesian(const Pose& target, uint32_t now) {
    if (!config::kDhParamsMeasured) {
        last_request_ = RequestStatus::NotCalibrated;
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }

    // 以当前指令角为初值，收敛到"距当前位形最近"的解
    float q_ik[config::kJointCount];
    float pos_err = 0.0f;
    float rot_err = 0.0f;
    int iters = 0;
    const IkStatus ik = solveIk(target, q_cmd_, q_ik, &pos_err, &rot_err, &iters);

    arm_fw_ik_iters = iters;
    arm_fw_ik_err_pos = pos_err;
    arm_fw_ik_err_rot = rot_err;

    if (ik != IkStatus::Ok) {
        last_request_ = RequestStatus::IkFailed;
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }

    last_request_ = moveJ(q_ik, now);
    return last_request_;
}

RequestStatus ArmController::jog(uint8_t joint, float delta, uint32_t now) {
    if (joint >= config::kJointCount || !std::isfinite(delta)) {
        last_request_ = RequestStatus::InvalidTarget;
        arm_fw_request = static_cast<uint8_t>(last_request_);
        return last_request_;
    }
    float q[config::kJointCount];
    std::memcpy(q, q_cmd_, sizeof(q));  // NOLINT
    q[joint] += delta;
    return moveJ(q, now);
}

void ArmController::setGripper(bool closed) {
    hal::setGripper(closed);
}

bool ArmController::checkFeedback(uint32_t now) {
    hal::MotorSnapshot snap;
    hal::readMotors(snap);

    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (snap.rx_count[i] == 0U ||
            (now - snap.last_tick[i]) > config::kFeedbackTimeoutMs) {
            enterFault(CtrlFault::FeedbackTimeout, now);
            return false;
        }
    }
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        // 达妙状态 0(失能)/1(使能) 正常，8~E 为故障
        if (snap.err[i] >= 8U) {
            enterFault(CtrlFault::MotorError, now);
            return false;
        }
    }
    hal::motorToKin(snap.pos, q_measured_);
    return true;
}

void ArmController::tick(uint32_t now) {
    arm_fw_state = static_cast<uint8_t>(state_);
    if (state_ == CtrlState::Disabled || state_ == CtrlState::Fault) return;

    if (!checkFeedback(now)) return;

    // 跟踪误差（最大关节偏差）
    tracking_error_ = 0.0f;
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        const float e = std::fabs(q_cmd_[i] - q_measured_[i]);
        if (e > tracking_error_) tracking_error_ = e;
    }
    arm_fw_tracking_err = tracking_error_;

    if (state_ == CtrlState::Moving) {
        // 暂停期间冻结轨迹时间轴（沿用回放逻辑：误差过大先暂停，超时才失能）
        if (paused_) {
            traj_pause_ms_ += (now - last_tick_ms_);
            if (tracking_error_ <= config::kTrackingPauseError) {
                paused_ = false;
            } else if ((now - pause_since_) > config::kTrackingTimeoutMs) {
                enterFault(CtrlFault::TrackingTimeout, now);
                return;
            }
        } else if (tracking_error_ > config::kTrackingPauseError) {
            paused_ = true;
            pause_since_ = now;
        }
        last_tick_ms_ = now;

        const float t_s = static_cast<float>(now - traj_start_ms_ - traj_pause_ms_) / 1000.0f;
        trajSample(traj_, t_s, q_cmd_);
        arm_fw_traj_progress = trajProgress();

        if (!withinLimits(q_cmd_, config::kLimitTolerance)) {
            enterFault(CtrlFault::LimitViolation, now);
            return;
        }

        if (!hal::sendJointTargets(q_cmd_, config::kSendVelLimit)) {
            if (++tx_fail_count_ >= config::kCanTxFailLimit) {
                enterFault(CtrlFault::CanTx, now);
                return;
            }
        } else {
            tx_fail_count_ = 0;
        }

        // 完成判定：时间轴走完 + 到位且稳定
        if (t_s >= traj_.t_total) {
            if (tracking_error_ <= config::kArriveTolerance) {
                if (arrive_since_ == 0) arrive_since_ = now;
                if ((now - arrive_since_) >= config::kArriveStableMs) {
                    if (queue_count_ > 0) {
                        float next[config::kJointCount];
                        std::memcpy(next, queue_[queue_head_], sizeof(next));  // NOLINT
                        queue_head_ = (queue_head_ + 1) % config::kQueueDepth;
                        --queue_count_;
                        startTraj(next, now);
                    } else {
                        state_ = CtrlState::Ready;
                        arm_fw_state = static_cast<uint8_t>(state_);
                        last_hold_ms_ = now;
                    }
                }
            } else {
                arrive_since_ = 0;
            }
        }
        return;
    }

    // Ready 态：队列非空则启动下一段；否则周期性重发保持命令
    if (queue_count_ > 0) {
        float next[config::kJointCount];
        std::memcpy(next, queue_[queue_head_], sizeof(next));  // NOLINT
        queue_head_ = (queue_head_ + 1) % config::kQueueDepth;
        --queue_count_;
        startTraj(next, now);
        return;
    }

    if ((now - last_hold_ms_) >= config::kHoldResendMs) {
        if (!hal::sendJointTargets(q_cmd_, config::kHoldVelLimit)) {
            if (++tx_fail_count_ >= config::kCanTxFailLimit) {
                enterFault(CtrlFault::CanTx, now);
                return;
            }
        } else {
            tx_fail_count_ = 0;
        }
        last_hold_ms_ = now;
    }
}

float ArmController::trajProgress() const {
    if (state_ != CtrlState::Moving || !traj_.valid) return 0.0f;
    const uint32_t now = last_tick_ms_;
    const float t_s = static_cast<float>(now - traj_start_ms_ - traj_pause_ms_) / 1000.0f;
    const float p = t_s / traj_.t_total;
    return p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
}

void ArmController::getCurrentJoints(float q[config::kJointCount]) const {
    std::memcpy(q, q_measured_, sizeof(q_measured_));  // NOLINT
}

void ArmController::getCommandedJoints(float q[config::kJointCount]) const {
    std::memcpy(q, q_cmd_, sizeof(q_cmd_));  // NOLINT
}

bool ArmController::getCurrentPose(Pose& out) {
    if (!config::kDhParamsMeasured) return false;
    solveFkPose(q_measured_, out);
    return true;
}

}  // namespace arm
