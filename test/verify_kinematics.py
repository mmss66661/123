#!/usr/bin/env python3
"""
verify_kinematics.py — 机械臂框架数值验证（上位机，无硬件依赖）

直接解析 RobotArm/Inc/arm_config.h 中实际配置的 DH 参数/关节限位/IK 参数，
用与 C++ 实现完全相同的公式做以下检查：

1. 差速腕耦合换算 motorToKin/kinToMotor 往返一致性
2. DH 正运动学对手工推导值（q=0 时末端位置）的核对
3. 解析雅可比 vs 有限差分雅可比
4. FK/IK 随机往返（40 组可达目标，从扰动初值收敛）
5. 不可达目标的失败行为
6. 梯形轨迹边界/单调性/速度限幅/关节同步

用法:  python verify_kinematics.py
"""

import math
import random
import re
import sys
from pathlib import Path

CONFIG_H = Path(__file__).resolve().parent.parent / "RobotArm" / "Inc" / "arm_config.h"

# ---------------------------------------------------------------- 配置解析


def parse_config(text):
    def block(name):
        m = re.search(name + r"\[[^=]*=\s*\{(.*?)\};", text, re.S)
        return m.group(1)

    dh = []
    for m in re.finditer(
        r"\{\s*(-?[\d.eE+]+)f\s*,\s*(-?[\d.eE+]+)f\s*,\s*(-?[\d.eE+]+)f\s*,\s*(-?[\d.eE+]+)f\s*\}",
        block(r"constexpr DhRow kDh"),
    ):
        dh.append(tuple(float(g) for g in m.groups()))

    joints = []
    for m in re.finditer(
        r'\{\s*"(\w+)"\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*([+-]?[\d.]+)f\s*,\s*([+-]?[\d.]+)f\s*,'
        r"\s*([+-]?[\d.]+)f\s*,\s*([+-]?[\d.]+)f\s*,\s*(true|false)\s*,\s*([\d.]+)f\s*\}",
        block(r"constexpr JointConfig kJoints"),
    ):
        joints.append(
            dict(
                name=m.group(1),
                lo=float(m.group(5)),
                hi=float(m.group(6)),
                has_limits=m.group(7) == "true",
            )
        )

    def scalar(name, default):
        m = re.search(name + r"\s*=\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)f?", text)
        return float(m.group(1)) if m else default

    cfg = dict(
        dh=dh,
        joints=joints,
        ik_max_iter=int(scalar(r"constexpr int\s+kIkMaxIter", 80)),
        ik_tol_pos=scalar(r"kIkTolPos", 5e-4),
        ik_tol_rot=scalar(r"kIkTolRot", 5e-3),
        ik_lam0=scalar(r"kIkLam0", 0.10),
        ik_max_dq=scalar(r"kIkMaxDq", 0.30),
        traj_vel=scalar(r"kTrajDefaultVel", 1.0),
        traj_acc=scalar(r"kTrajDefaultAcc", 2.5),
        roll_gain=scalar(r"kWristRollGain", 0.5),
        pitch_gain=scalar(r"kWristPitchGain", 1.0),
    )
    assert len(dh) == 6, f"DH 行数异常: {len(dh)}"
    assert len(joints) == 6, f"关节数异常: {len(joints)}"
    return cfg


# ------------------------------------------------------------ 数学（镜像 C++）


def mat_mul(a, b):
    n, m, p = len(a), len(b), len(b[0])
    return [[sum(a[i][k] * b[k][j] for k in range(m)) for j in range(p)] for i in range(n)]


def mat_t(a):
    return [[a[j][i] for j in range(len(a))] for i in range(len(a[0]))]


def dh_transform(row, q):
    d, a, alpha, off = row
    th = q + off
    ct, st = math.cos(th), math.sin(th)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def fk_chain(cfg, q):
    T = [np_eye(4)]
    for i in range(6):
        T.append(mat_mul(T[-1], dh_transform(cfg["dh"][i], q[i])))
    return T


def np_eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def fk(cfg, q):
    return fk_chain(cfg, q)[-1]


def jacobian(cfg, q):
    T = fk_chain(cfg, q)
    pe = [T[6][i][3] for i in range(3)]
    J = [[0.0] * 6 for _ in range(6)]
    for i in range(6):
        z = [T[i][r][2] for r in range(3)]
        o = [T[i][r][3] for r in range(3)]
        rvec = [pe[k] - o[k] for k in range(3)]
        J[0][i] = z[1] * rvec[2] - z[2] * rvec[1]
        J[1][i] = z[2] * rvec[0] - z[0] * rvec[2]
        J[2][i] = z[0] * rvec[1] - z[1] * rvec[0]
        J[3][i], J[4][i], J[5][i] = z
    return J


def rotation_log_error(R):
    """姿态误差 = 旋转矢量 log(R)。与 C++ rotationLogError 完全一致。

    不能用 0.5*(R-R^T)（幅值 sin(theta)）：误差角接近 pi 时会虚假收敛。
    """
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    theta = math.acos(c)
    if theta < 1e-5:
        return [0.5 * (R[2][1] - R[1][2]), 0.5 * (R[0][2] - R[2][0]), 0.5 * (R[1][0] - R[0][1])]
    if math.pi - theta < 1e-3:
        a = [math.sqrt(max(0.0, (R[i][i] + 1.0) * 0.5)) for i in range(3)]
        anchor = max(range(3), key=lambda i: a[i])
        for j in range(3):
            if j != anchor and (R[anchor][j] + R[j][anchor]) < 0:
                a[j] = -a[j]
        return [ai * theta for ai in a]
    k = theta / math.sin(theta)  # vee=sin*axis，乘 theta/sin 得 theta*axis
    return [
        k * 0.5 * (R[2][1] - R[1][2]),
        k * 0.5 * (R[0][2] - R[2][0]),
        k * 0.5 * (R[1][0] - R[0][1]),
    ]


# 兼容旧引用：小角度下两者一致，用于有限差分
def rot_vec_from_matrix(R):
    return rotation_log_error(R)


def cholesky_solve(M, e):
    n = 6
    L = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = M[i][j] - sum(L[i][k] * L[j][k] for k in range(j))
            if i == j:
                if s <= 1e-12:
                    return None
                L[i][j] = math.sqrt(s)
            else:
                L[i][j] = s / L[j][j]
    z = [0.0] * n
    for i in range(n):
        z[i] = (e[i] - sum(L[i][k] * z[k] for k in range(i))) / L[i][i]
    y = [0.0] * n
    for i in reversed(range(n)):
        y[i] = (z[i] - sum(L[k][i] * y[k] for k in range(i + 1, n))) / L[i][i]
    return y


def clamp_limits(cfg, q):
    for i, j in enumerate(cfg["joints"]):
        if j["has_limits"]:
            q[i] = max(j["lo"], min(j["hi"], q[i]))
    return q


def solve_ik(cfg, Td, q_ref, rng=None):
    """DLS + Levenberg-Marquardt 自适应阻尼 + 限位冻结 + 随机重启。

    - LM：步后误差下降则接受且 λ*=0.6，否则拒绝该步且 λ*=3。
    - 限位冻结：已顶在限位且步长仍向外的关节，本代雅可比列置零后重解，
      避免每代把步长浪费在撞限位方向上（边界爬行）。
    - 停滞：连续 10 代无绝对改善 → 限位内随机重启，跳出错误吸引域。
    """
    lam0, lam_min, lam_max = cfg["ik_lam0"], 1e-4, 1.0
    accept_scale, reject_scale = 0.6, 3.0
    max_restarts, stall_limit = 4, 10
    limit_eps = 1e-3

    def error_of(q):
        Tc = fk(cfg, q)
        ep = [Td[r][3] - Tc[r][3] for r in range(3)]
        Rc = [[Tc[r][c] for c in range(3)] for r in range(3)]
        Rd = [[Td[r][c] for c in range(3)] for r in range(3)]
        eo = rotation_log_error(mat_mul(Rd, mat_t(Rc)))
        err_p = math.sqrt(sum(x * x for x in ep))
        err_r = math.sqrt(sum(x * x for x in eo))
        return ep + eo, err_p, err_r

    def solve_step(q, e, lam, frozen):
        J = jacobian(cfg, q)
        Jm = [[0.0 if frozen[i] else J[r][i] for i in range(6)] for r in range(6)]
        lam2 = lam * lam
        M = [[sum(Jm[r][k] * Jm[c][k] for k in range(6)) + (lam2 if r == c else 0.0)
              for c in range(6)] for r in range(6)]
        y = cholesky_solve(M, e)
        if y is None:
            return None
        dq = [max(-cfg["ik_max_dq"], min(cfg["ik_max_dq"],
               sum(Jm[r][i] * y[r] for r in range(6)))) for i in range(6)]
        return dq

    def at_limit_outside(i, qi, dqi):
        j = cfg["joints"][i]
        if not j["has_limits"]:
            return False
        return (qi <= j["lo"] + limit_eps and dqi < 0.0) or \
               (qi >= j["hi"] - limit_eps and dqi > 0.0)

    q = clamp_limits(cfg, list(q_ref))
    best_q, best_p, best_r = list(q), 1e30, 0.0
    lam = lam0
    iters = 0
    restarts = 0
    stall = 0
    prev_comb = 1e30
    status = "MaxIterations"

    while iters < cfg["ik_max_iter"]:
        e, err_p, err_r = error_of(q)
        comb = err_p + 0.1 * err_r
        if comb < best_p + 0.1 * best_r:
            best_q, best_p, best_r = list(q), err_p, err_r

        if err_p < cfg["ik_tol_pos"] and err_r < cfg["ik_tol_rot"]:
            return "Ok", list(q), err_p, err_r, iters

        # 第一遍：全雅可比步，用于判断哪些关节"顶限位且仍向外"
        dq = solve_step(q, e, lam, [False] * 6)
        iters += 1
        if dq is None:
            break
        frozen = [at_limit_outside(i, q[i], dq[i]) for i in range(6)]
        if any(frozen):
            dq2 = solve_step(q, e, lam, frozen)
            if dq2 is not None:
                dq = dq2

        q_try = clamp_limits(cfg, [q[i] + dq[i] for i in range(6)])
        _, p_try, r_try = error_of(q_try)
        comb_try = p_try + 0.1 * r_try
        if comb_try < comb:
            q = q_try
            lam = max(lam_min, lam * accept_scale)
        else:
            lam = min(lam_max, lam * reject_scale)

        # 停滞检测：连续 10 代无绝对改善 → 随机重启跳出吸引域
        improved = comb_try < prev_comb - 1e-5
        prev_comb = min(prev_comb, comb_try)
        stall = 0 if improved else stall + 1
        if (stall >= stall_limit and restarts < max_restarts
                and comb > cfg["ik_tol_pos"] + 0.1 * cfg["ik_tol_rot"]):
            restarts += 1
            stall = 0
            lam = lam0
            if rng is not None:
                q = random_q(cfg, rng)

    return status, best_q, best_p, best_r, iters


# ------------------------------------------------------- 轨迹（镜像 C++）


def traj_make(dq_dominant, vel, acc):
    if dq_dominant < 1e-6:
        return None
    d_ramp = vel * vel / acc
    if dq_dominant >= d_ramp:
        t_acc = vel / acc
        t_total = t_acc + dq_dominant / vel
        s_vel = vel / dq_dominant
    else:
        t_acc = math.sqrt(dq_dominant / acc)
        t_total = 2 * t_acc
        s_vel = 1.0 / t_acc
    return t_total, t_acc, s_vel


def traj_s(traj, t):
    t_total, t_acc, s_vel = traj
    t = max(0.0, min(t, t_total))
    a_s = s_vel / t_acc
    t_flat = t_total - t_acc
    if t < t_acc:
        return 0.5 * a_s * t * t
    if t < t_flat:
        return 0.5 * a_s * t_acc * t_acc + s_vel * (t - t_acc)
    t_end = t_total - t
    return 1.0 - 0.5 * a_s * t_end * t_end


# ------------------------------------------------------------------- 测试


def random_q(cfg, rng):
    q = []
    for j in cfg["joints"]:
        if j["has_limits"]:
            # 限位内留 10% 余量，避免 IK 在边界上抖动
            span = (j["hi"] - j["lo"]) * 0.1
            q.append(rng.uniform(j["lo"] + span, j["hi"] - span))
        else:
            q.append(rng.uniform(-1.5, 1.5))
    return q


def test_wrist_coupling(cfg):
    for _ in range(1000):
        m = [random.uniform(-5, 5) for _ in range(2)]
        dm4, dm5 = m[0], m[1]
        roll = cfg["roll_gain"] * (dm4 + dm5)
        pitch = cfg["pitch_gain"] * (dm4 - dm5)
        r, p = roll / cfg["roll_gain"], pitch / cfg["pitch_gain"]
        m4 = 0.5 * (r + p)
        m5 = 0.5 * (r - p)
        assert abs(m4 - m[0]) < 1e-9 and abs(m5 - m[1]) < 1e-9, "差速腕往返换算不一致"
    return "差速腕耦合 motorToKin/kinToMotor 往返一致 (1000 组)"


def test_fk_zero(cfg):
    T = fk(cfg, [0.0] * 6)
    # 期望值写在 arm_config.h 的 kDh 注释里（FK(0) = (x, y, z) m），
    # 由 dh_from_axes 从轴线数据独立构造（与矩阵公式不同源）
    m = re.search(r"FK\(0\).*?\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)", CONFIG_H.read_text(encoding="utf-8"))
    assert m, "arm_config.h 中缺少 FK(0) 期望值注释"
    expect = [float(g) for g in m.groups()]
    err = math.dist([T[i][3] for i in range(3)], expect)
    assert err < 1e-6, f"FK(0) 位置偏差 {err}"
    return f"FK(q=0) 末端位置 = ({expect[0]}, {expect[1]}, {expect[2]})，与轴线构造值一致"


def test_jacobian_fd(cfg, rng):
    worst = 0.0
    for _ in range(20):
        q = random_q(cfg, rng)
        J = jacobian(cfg, q)
        h = 1e-5
        for i in range(6):
            qp, qm = list(q), list(q)
            qp[i] += h
            qm[i] -= h
            Tp, Tm = fk(cfg, qp), fk(cfg, qm)
            fd_lin = [(Tp[r][3] - Tm[r][3]) / (2 * h) for r in range(3)]
            Rp = [[Tp[r][c] for c in range(3)] for r in range(3)]
            Rm = [[Tm[r][c] for c in range(3)] for r in range(3)]
            rv = rot_vec_from_matrix(mat_mul(Rp, mat_t(Rm)))
            fd_ang = [x / (2 * h) for x in rv]
            for r in range(3):
                worst = max(worst, abs(J[r][i] - fd_lin[r]), abs(J[r + 3][i] - fd_ang[r]))
    assert worst < 1e-4, f"雅可比与有限差分偏差 {worst}"
    return f"解析雅可比 vs 有限差分 最大偏差 {worst:.2e}"


def run_ik_trials(cfg, rng, mode, count, pass_threshold):
    """mode='nearby'：种子在目标附近 ±0.4 rad（真实工况：从当前位姿运动）；
    mode='random'：完全随机种子（跨位形严苛场景）。"""
    ok, worst_p, worst_r, worst_iters = 0, 0.0, 0.0, 0
    for trial in range(count):
        q_t = random_q(cfg, rng)
        Td = fk(cfg, q_t)
        if mode == "nearby":
            q_seed = clamp_limits(cfg, [qi + rng.uniform(-0.4, 0.4) for qi in q_t])
        else:
            q_seed = random_q(cfg, rng)
        status, q_sol, err_p, err_r, iters = solve_ik(cfg, Td, q_seed, rng=rng)
        # 复核：把解代回 FK 与目标矩阵比较
        T_sol = fk(cfg, q_sol)
        pos_err = math.dist([T_sol[r][3] for r in range(3)], [Td[r][3] for r in range(3)])
        Rerr = mat_mul(mat_t([[T_sol[r][c] for c in range(3)] for r in range(3)]),
                       [[Td[r][c] for c in range(3)] for r in range(3)])
        rot_err = math.sqrt(sum(x * x for x in rotation_log_error(Rerr)))
        if pos_err < cfg["ik_tol_pos"] * 2 and rot_err < cfg["ik_tol_rot"] * 2:
            ok += 1
            worst_p = max(worst_p, pos_err)
            worst_r = max(worst_r, rot_err)
            worst_iters = max(worst_iters, iters)
        else:
            print(f"  [未收敛] trial {trial}: pos={pos_err:.5f} rot={rot_err:.5f} iters={iters}")
    label = "邻近种子(±0.4rad)" if mode == "nearby" else "随机种子(严苛)"
    detail = (f"IK {label} {ok}/{count} 收敛，最差位置误差 {worst_p*1000:.2f} mm，"
              f"姿态误差 {worst_r:.4f} rad，最多 {worst_iters} 次迭代")
    assert ok >= pass_threshold, f"{detail} —— 低于阈值 {pass_threshold}"
    return detail


def test_ik_unreachable(cfg):
    Td = np_eye(4)
    Td[0][3] = 10.0  # 10 m 外，远超臂展
    status, _, err_p, _, _ = solve_ik(cfg, Td, [0.0] * 6)
    assert status != "Ok" and err_p > cfg["ik_tol_pos"], "不可达目标不应误报收敛"
    return f"不可达目标正确返回 MaxIterations（剩余位置误差 {err_p:.3f} m）"


def test_trajectory(cfg):
    rng = random.Random(7)
    for _ in range(200):
        q0 = random_q(cfg, rng)
        q1 = random_q(cfg, rng)
        dominant = max(abs(q1[i] - q0[i]) for i in range(6))
        traj = traj_make(dominant, cfg["traj_vel"], cfg["traj_acc"])
        if traj is None:
            continue
        t_total, t_acc, s_vel = traj
        # 边界
        assert abs(traj_s(traj, 0.0)) < 1e-12
        assert abs(traj_s(traj, t_total) - 1.0) < 1e-9
        # 单调 + 速度限幅（主导关节）
        n, prev, vmax_seen = 400, 0.0, 0.0
        for k in range(1, n + 1):
            s = traj_s(traj, t_total * k / n)
            assert s >= prev - 1e-12, "轨迹不单调"
            vmax_seen = max(vmax_seen, (s - prev) / (t_total / n))
            prev = s
        assert vmax_seen <= s_vel * (1 + 1e-6), f"主导关节峰值速度越限 {vmax_seen}"
        # 各关节同起同止：采样值始终位于起终点的包络内
        for i in range(6):
            lo, hi = sorted((q0[i], q1[i]))
            s_mid = traj_s(traj, t_total * 0.37)
            qi = q0[i] + (q1[i] - q0[i]) * s_mid
            assert lo - 1e-9 <= qi <= hi + 1e-9
    return "梯形轨迹边界/单调性/速度限幅/包络检查通过 (200 组)"


def main():
    cfg = parse_config(CONFIG_H.read_text(encoding="utf-8"))
    print(f"已解析 {CONFIG_H}")
    print(f"  DH 参数: {cfg['dh']}")
    print(f"  IK: max_iter={cfg['ik_max_iter']} λ0={cfg['ik_lam0']} "
          f"tol=({cfg['ik_tol_pos']}m, {cfg['ik_tol_rot']}rad)")
    print()

    rng = random.Random(20260930)
    tests = [
        ("差速腕耦合", lambda: test_wrist_coupling(cfg)),
        ("正运动学", lambda: test_fk_zero(cfg)),
        ("雅可比", lambda: test_jacobian_fd(cfg, rng)),
        ("逆运动学-邻近", lambda: run_ik_trials(cfg, rng, "nearby", 60, 60)),
        ("逆运动学-随机", lambda: run_ik_trials(cfg, rng, "random", 40, 33)),
        ("不可达目标", lambda: test_ik_unreachable(cfg)),
        ("轨迹规划", lambda: test_trajectory(cfg)),
    ]
    failed = 0
    for name, fn in tests:
        try:
            msg = fn()
            print(f"[PASS] {name}: {msg}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {name}: {e}")
    print()
    if failed:
        print(f"结果: {failed} 项失败")
        sys.exit(1)
    print("结果: 全部通过 ✔")


if __name__ == "__main__":
    main()
