//
// kinematics.cpp — DH 正运动学 / 几何雅可比 / DLS 数值逆运动学实现
//

#include "../Inc/kinematics.h"

#include "../Inc/arm_math.h"

#include <cmath>
#include <cstring>

namespace arm {
namespace {

// 标准 DH 单关节变换：A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
void dhTransform(const config::DhRow& row, float q, Mat4& A) {
    const float th = q + row.theta_offset;
    const float ct = cosf(th), st = sinf(th);
    const float ca = cosf(row.alpha), sa = sinf(row.alpha);

    A.m[0][0] = ct;
    A.m[0][1] = -st * ca;
    A.m[0][2] = st * sa;
    A.m[0][3] = row.a * ct;
    A.m[1][0] = st;
    A.m[1][1] = ct * ca;
    A.m[1][2] = -ct * sa;
    A.m[1][3] = row.a * st;
    A.m[2][0] = 0.0f;
    A.m[2][1] = sa;
    A.m[2][2] = ca;
    A.m[2][3] = row.d;
    A.m[3][0] = 0.0f;
    A.m[3][1] = 0.0f;
    A.m[3][2] = 0.0f;
    A.m[3][3] = 1.0f;
}

// FK 并保留各级中间坐标系 T[0..6]（雅可比需要）
void fkChain(const float q[config::kJointCount], Mat4 T[config::kJointCount + 1]) {
    T[0] = mat4Identity();
    Mat4 A;
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        dhTransform(config::kDh[i], static_cast<float>(q[i]), A);
        T[i + 1] = mat4Multiply(T[i], A);
    }
}

// 6x6 Cholesky 求解 M*y = e（M 对称正定，DLS 中 M = JJ^T + λ²I 必然满足）
bool choleskySolve6(const float M[6][6], const float e[6], float y[6]) {
    float L[6][6]{};

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = M[i][j];
            for (int k = 0; k < j; ++k) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                if (sum <= 1e-12f) return false;  // 数值上不正定
                L[i][j] = sqrtf(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }

    // 前代 L z = e
    float z[6]{};
    for (int i = 0; i < 6; ++i) {
        float sum = e[i];
        for (int k = 0; k < i; ++k) sum -= L[i][k] * z[k];
        z[i] = sum / L[i][i];
    }
    // 回代 L^T y = z
    for (int i = 5; i >= 0; --i) {
        float sum = z[i];
        for (int k = i + 1; k < 6; ++k) sum -= L[k][i] * y[k];
        y[i] = sum / L[i][i];
    }
    return true;
}

// 姿态误差 = 旋转矢量 log(Re)。
// 不能用 0.5*(Re - Re^T) 反对称提取：其幅值为 sin(theta)，误差角接近 pi 时
// 趋于 0，迭代会被"翻转 180 度"的伪极小值吸引而虚假收敛。
// log 映射的幅值等于真实误差角，全角度范围（除 theta 恰为 pi 的轴奇异性）梯度有效。
void rotationLogError(const Mat3& Re, float eo[3]) {
    constexpr float kPi = 3.14159265358979f;

    const float tr = Re.m[0][0] + Re.m[1][1] + Re.m[2][2];
    float c = 0.5f * (tr - 1.0f);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    const float theta = acosf(c);

    if (theta < 1e-5f) {
        // 小角度：反对称部分即近似旋转矢量
        eo[0] = 0.5f * (Re.m[2][1] - Re.m[1][2]);
        eo[1] = 0.5f * (Re.m[0][2] - Re.m[2][0]);
        eo[2] = 0.5f * (Re.m[1][0] - Re.m[0][1]);
        return;
    }

    if (kPi - theta < 1e-3f) {
        // 接近 pi：反对称部分退化，转轴由 (R+I)/2 = a*a^T 的对角元恢复，
        // 符号由对称化的非对角元 R_ij + R_ji = 2*a_i*a_j 确定
        float a[3];
        a[0] = sqrtf(Re.m[0][0] > -1.0f ? (Re.m[0][0] + 1.0f) * 0.5f : 0.0f);
        a[1] = sqrtf(Re.m[1][1] > -1.0f ? (Re.m[1][1] + 1.0f) * 0.5f : 0.0f);
        a[2] = sqrtf(Re.m[2][2] > -1.0f ? (Re.m[2][2] + 1.0f) * 0.5f : 0.0f);

        int anchor = 0;
        if (a[1] > a[anchor]) anchor = 1;
        if (a[2] > a[anchor]) anchor = 2;
        for (int j = 0; j < 3; ++j) {
            if (j == anchor) continue;
            if ((Re.m[anchor][j] + Re.m[j][anchor]) < 0.0f) a[j] = -a[j];
        }
        eo[0] = a[0] * theta;
        eo[1] = a[1] * theta;
        eo[2] = a[2] * theta;
        return;
    }

    // vee(Re) = sin(theta)*axis，乘 theta/sin(theta) 得到 theta*axis
    const float k = theta / sinf(theta);
    eo[0] = k * 0.5f * (Re.m[2][1] - Re.m[1][2]);
    eo[1] = k * 0.5f * (Re.m[0][2] - Re.m[2][0]);
    eo[2] = k * 0.5f * (Re.m[1][0] - Re.m[0][1]);
}

}  // namespace

void solveFk(const float q[config::kJointCount], Mat4& T) {
    Mat4 chain[config::kJointCount + 1];
    fkChain(q, chain);
    T = chain[config::kJointCount];
}

void solveFkPose(const float q[config::kJointCount], Pose& pose) {
    Mat4 T;
    solveFk(q, T);
    mat4ToPose(T, pose);
}

void calcJacobian(const float q[config::kJointCount], float J[6][config::kJointCount]) {
    Mat4 T[config::kJointCount + 1];
    fkChain(q, T);

    const float pe[3] = {T[6].m[0][3], T[6].m[1][3], T[6].m[2][3]};

    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        // 关节 i 绕 T[i] 的 z 轴旋转，轴过 T[i] 的原点
        const float z[3] = {T[i].m[0][2], T[i].m[1][2], T[i].m[2][2]};
        const float o[3] = {T[i].m[0][3], T[i].m[1][3], T[i].m[2][3]};

        const float r[3] = {pe[0] - o[0], pe[1] - o[1], pe[2] - o[2]};
        J[0][i] = z[1] * r[2] - z[2] * r[1];
        J[1][i] = z[2] * r[0] - z[0] * r[2];
        J[2][i] = z[0] * r[1] - z[1] * r[0];
        J[3][i] = z[0];
        J[4][i] = z[1];
        J[5][i] = z[2];
    }
}

void clampToLimits(float q[config::kJointCount]) {
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (config::kJoints[i].has_limits) {
            if (q[i] < config::kJoints[i].limit_lo) q[i] = config::kJoints[i].limit_lo;
            if (q[i] > config::kJoints[i].limit_hi) q[i] = config::kJoints[i].limit_hi;
        }
    }
}

bool withinLimits(const float q[config::kJointCount], float tolerance) {
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (!std::isfinite(q[i])) return false;
        if (config::kJoints[i].has_limits &&
            (q[i] < config::kJoints[i].limit_lo - tolerance ||
             q[i] > config::kJoints[i].limit_hi + tolerance)) {
            return false;
        }
    }
    return true;
}

IkStatus solveIk(const Pose& target,
                 const float q_ref[config::kJointCount],
                 float q_out[config::kJointCount],
                 float* pos_error,
                 float* rot_error,
                 int* iterations) {
    // ===== DLS + Levenberg-Marquardt 自适应阻尼 + 限位冻结 + 随机重启 =====
    // 策略与 test/verify_kinematics.py 完全一致（上位机已数值验证）：
    //  - LM：步后误差下降则接受且 λ*=0.6，否则拒绝该步且 λ*=3；
    //  - 限位冻结：顶在限位且步长仍向外的关节，本代雅可比列置零后重解，
    //    避免把步长浪费在撞限位方向上（边界爬行）；
    //  - 停滞：连续 10 代无绝对改善 → 限位内随机重启，跳出错误吸引域。
    if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(target.z) || !std::isfinite(target.roll) ||
        !std::isfinite(target.pitch) || !std::isfinite(target.yaw)) {
        return IkStatus::InvalidInput;
    }
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        if (!std::isfinite(q_ref[i])) return IkStatus::InvalidInput;
    }

    constexpr float kLamMin = 1e-4f;
    constexpr float kLamMax = 1.0f;
    constexpr float kAcceptScale = 0.6f;
    constexpr float kRejectScale = 3.0f;
    constexpr int kMaxRestarts = 4;
    constexpr int kStallLimit = 10;
    constexpr float kLimitEps = 1e-3f;

    const Mat4 Td = poseToMat4(target);

    float q[config::kJointCount];
    for (std::size_t i = 0; i < config::kJointCount; ++i) {
        q[i] = q_ref[i];
    }
    clampToLimits(q);

    // 记录迭代过程中的最优解（不可达时返回最接近的位形）
    float best_q[config::kJointCount];
    float best_p = 1e30f;
    float best_r = 0.0f;
    std::memcpy(best_q, q, sizeof(q));  // NOLINT

    // 重启用的确定性伪随机序列（xorshift32）
    uint32_t rng = 0x20260930u;
    auto uniform = [&rng](float lo, float hi) {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return lo + (hi - lo) * (static_cast<float>(rng >> 8) / 16777216.0f);
    };

    // 计算位姿误差向量 e = [位置差; log(Rd*Rc^T)] 与其范数
    auto errorOf = [&](const float qq[config::kJointCount], float e[6],
                       float& err_p, float& err_r) {
        Mat4 Tc;
        solveFk(qq, Tc);
        for (int r = 0; r < 3; ++r) {
            e[r] = Td.m[r][3] - Tc.m[r][3];
        }
        Mat3 Re{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                float sum = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    sum += Td.m[r][k] * Tc.m[c][k];  // Rd * Rc^T
                }
                Re.m[r][c] = sum;
            }
        }
        float eo[3];
        rotationLogError(Re, eo);
        e[3] = eo[0];
        e[4] = eo[1];
        e[5] = eo[2];

        err_p = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        err_r = sqrtf(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
    };

    // 求解一个 DLS 步（frozen 关节的雅可比列置零）
    auto solveStep = [&](const float qq[config::kJointCount], const float e[6],
                         float lam, const bool frozen[config::kJointCount],
                         float dq[config::kJointCount]) {
        float J[6][config::kJointCount];
        calcJacobian(qq, J);

        const float lam2 = lam * lam;
        float M[6][6]{};
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                float sum = (r == c) ? lam2 : 0.0f;
                for (std::size_t k = 0; k < config::kJointCount; ++k) {
                    if (frozen[k]) continue;
                    sum += J[r][k] * J[c][k];
                }
                M[r][c] = sum;
            }
        }

        float y[6]{};
        if (!choleskySolve6(M, e, y)) return false;

        for (std::size_t i = 0; i < config::kJointCount; ++i) {
            if (frozen[i]) {
                dq[i] = 0.0f;
                continue;
            }
            float step = 0.0f;
            for (int r = 0; r < 6; ++r) {
                step += J[r][i] * y[r];
            }
            if (step > config::kIkMaxDq) step = config::kIkMaxDq;
            if (step < -config::kIkMaxDq) step = -config::kIkMaxDq;
            dq[i] = step;
        }
        return true;
    };

    auto atLimitOutside = [](std::size_t i, float qi, float dqi) {
        const auto& jc = config::kJoints[i];
        if (!jc.has_limits) return false;
        return (qi <= jc.limit_lo + kLimitEps && dqi < 0.0f) ||
               (qi >= jc.limit_hi - kLimitEps && dqi > 0.0f);
    };

    float lam = config::kIkLam0;
    int iters = 0;
    int restarts = 0;
    int stall = 0;
    float prev_comb = 1e30f;

    while (iters < config::kIkMaxIter) {
        float e[6];
        float err_p = 0.0f;
        float err_r = 0.0f;
        errorOf(q, e, err_p, err_r);
        const float comb = err_p + 0.1f * err_r;
        if (comb < best_p + 0.1f * best_r) {
            best_p = err_p;
            best_r = err_r;
            std::memcpy(best_q, q, sizeof(q));  // NOLINT
        }

        if (err_p < config::kIkTolPos && err_r < config::kIkTolRot) {
            std::memcpy(q_out, q, sizeof(q));  // NOLINT
            if (pos_error) *pos_error = err_p;
            if (rot_error) *rot_error = err_r;
            if (iterations) *iterations = iters;
            return IkStatus::Ok;
        }

        // 第一遍全雅可比步，用于判断哪些关节"顶限位且仍向外"
        bool frozen[config::kJointCount] = {false, false, false, false, false, false};
        float dq[config::kJointCount];
        if (!solveStep(q, e, lam, frozen, dq)) break;
        ++iters;

        bool any_frozen = false;
        for (std::size_t i = 0; i < config::kJointCount; ++i) {
            frozen[i] = atLimitOutside(i, q[i], dq[i]);
            any_frozen = any_frozen || frozen[i];
        }
        if (any_frozen) {
            float dq2[config::kJointCount];
            if (solveStep(q, e, lam, frozen, dq2)) {
                std::memcpy(dq, dq2, sizeof(dq2));  // NOLINT
            }
        }

        float q_try[config::kJointCount];
        for (std::size_t i = 0; i < config::kJointCount; ++i) {
            q_try[i] = q[i] + dq[i];
        }
        clampToLimits(q_try);

        float e_try[6];
        float p_try = 0.0f;
        float r_try = 0.0f;
        errorOf(q_try, e_try, p_try, r_try);
        const float comb_try = p_try + 0.1f * r_try;
        if (comb_try < comb) {
            std::memcpy(q, q_try, sizeof(q));  // NOLINT
            lam = (lam * kAcceptScale > kLamMin) ? lam * kAcceptScale : kLamMin;
        } else {
            lam = (lam * kRejectScale < kLamMax) ? lam * kRejectScale : kLamMax;
        }

        // 停滞检测：连续 kStallLimit 代无绝对改善 → 随机重启
        const bool improved = comb_try < prev_comb - 1e-5f;
        if (prev_comb > comb_try) prev_comb = comb_try;
        stall = improved ? 0 : stall + 1;
        if (stall >= kStallLimit && restarts < kMaxRestarts &&
            comb > config::kIkTolPos + 0.1f * config::kIkTolRot) {
            ++restarts;
            stall = 0;
            lam = config::kIkLam0;
            for (std::size_t i = 0; i < config::kJointCount; ++i) {
                const auto& jc = config::kJoints[i];
                if (jc.has_limits) {
                    const float margin = (jc.limit_hi - jc.limit_lo) * 0.1f;
                    q[i] = uniform(jc.limit_lo + margin, jc.limit_hi - margin);
                } else {
                    q[i] = uniform(-1.5f, 1.5f);
                }
            }
        }
    }

    // 未收敛：返回历史最优逼近（调用方可根据 pos/rot error 决定是否接受）
    std::memcpy(q_out, best_q, sizeof(best_q));  // NOLINT
    if (pos_error) *pos_error = best_p;
    if (rot_error) *rot_error = best_r;
    if (iterations) *iterations = iters;
    return IkStatus::MaxIterations;
}

}  // namespace arm
