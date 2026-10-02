#!/usr/bin/env python3
"""dh_from_axes.py — 由"零位姿态下的六条关节轴线"计算标准 DH 参数。

数学模型的本质就是六条关节轴线（每条 = 一点 + 一方向）。轴线数据可以来自：
  - Fusion 360 脚本 tools/fusion_export_axes.py（推荐，见该文件说明）
  - 任何 CAD（FreeCAD 打开 STEP 等）里手工测量
  - 实物测量（角尺 + 卷尺，精度低，仅应急）

约定（与 RobotArm 框架完全一致）：
  A_i = Rz(theta_i + theta_offset) * Tz(d) * Tx(a) * Rx(alpha)
  - 基座坐标系 = 输入轴线所在的坐标系（建议 Z 轴沿 J0 轴线、方向=J0 正方向）
  - 各轴线方向 = 该关节的运动学正方向（方向选反了 DH 也能算，但关节正方向会反）

用法：
  1) 编辑下方 AXES_MM（零位姿态、基座坐标系、单位 mm）后运行：
        python dh_from_axes.py
  2) 或命令行传入（6 组 "px,py,pz,dx,dy,dz"）：
        python dh_from_axes.py --axes 0,0,0,0,0,1  ... --tool 210,0,150
  3) 自检（用 arm_config.h 当前占位 DH 生成的轴线做往返验证）：
        python dh_from_axes.py --selftest
"""

import math
import sys
from pathlib import Path

# ============ 在此填入零位姿态下实测/导出的轴线（mm） ============
# 每行: (轴线上任意一点 x,y,z,  轴线方向 dx,dy,dz)，方向无需归一化
# 下面是"占位 DH 对应的轴线"示例（与 arm_config.h 当前 kDh 等价），
# 供演示数据格式；实际使用时替换为 CAD 导出的真实轴线。
AXES_MM = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),     # J0 底部 yaw
    (0.0, 0.0, 160.0, 0.0, -1.0, 0.0),  # J1 底部 pitch
    (140.0, 0.0, 160.0, 0.0, -1.0, 0.0),  # J2
    (160.0, 0.0, 160.0, 0.0, 0.0, -1.0),  # J3 末端 yaw
    (160.0, 0.0, 0.0, 0.0, -1.0, 0.0),  # J4 腕 roll
    (160.0, -60.0, 0.0, 0.0, 0.0, -1.0),  # J5 腕 pitch
]
# 工具中心点 TCP（夹爪两指中点等，mm）。省略时取 J5 轴线上一点。
# 此示例 = 占位 DH 的末端点
TOOL_POINT_MM = (160.0, -60.0, -80.0)
# ==============================================================

EPS_PARALLEL = 1e-6   # sin(夹角) 小于此值视为平行
EPS_COINCIDENT = 1e-9  # 公垂线距离小于此值(m)视为共线


# ---------------------------- 三维小工具 ----------------------------
def v_add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def v_scale(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def v_dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def v_norm(a): return math.sqrt(v_dot(a, a))
def v_unit(a):
    n = v_norm(a)
    if n < 1e-15:
        raise ValueError("零向量无法归一化")
    return v_scale(a, 1.0 / n)
def v_dist(a, b): return v_norm(v_sub(a, b))


def line_closest_feet(p1, d1, p2, d2):
    """两条直线的公垂线垂足（方向向量需已归一化）。平行时返回任一组最近点。"""
    r = v_sub(p2, p1)
    b = v_dot(d1, d2)
    denom = 1.0 - b * b
    if denom < 1e-12:  # 平行：line2 上离 p1 最近的点
        t = -v_dot(r, d2)
        return p1, v_add(p2, v_scale(d2, t))
    s = (v_dot(r, d1) - b * v_dot(r, d2)) / denom
    t = s * b - v_dot(r, d2)
    return v_add(p1, v_scale(d1, s)), v_add(p2, v_scale(d2, t))


def point_line_distance(p, line_p, line_d):
    return v_norm(v_cross(v_sub(p, line_p), line_d))


def signed_angle(u, v, axis):
    """绕 axis（单位向量）从 u 到 v 的有符号角。"""
    return math.atan2(v_dot(v_cross(u, v), axis), v_dot(u, v))


def orthonormal_frame(z, x_hint):
    """由 z 轴与期望的 x 方向构造右手正交基 (xb, yb, zb)。"""
    zb = z
    x = v_sub(x_hint, v_scale(zb, v_dot(x_hint, zb)))
    if v_norm(x) < 1e-12:
        ref = (1.0, 0.0, 0.0) if abs(zb[0]) < 0.9 else (0.0, 1.0, 0.0)
        x = v_cross(v_cross(zb, ref), zb)
    xb = v_unit(x)
    yb = v_cross(zb, xb)
    return xb, yb, zb


def express_in_frame(B, p):
    """把世界坐标点 p 表示到基 B=(xb,yb,zb) 下。"""
    xb, yb, zb = B
    return (v_dot(xb, p), v_dot(yb, p), v_dot(zb, p))


# ---------------------------- DH 推导 ----------------------------
def dh_from_axes(axes, tool_point=None):
    """axes: [(point, unit_dir)] * 6（米，世界/CAD 坐标系）。

    返回 (rows, base_frame)。注意：DH 的基座标架约定为 Z 轴沿 J0 轴线、
    x 轴沿 J0/J1 公垂线在基座平面内的方向 —— 与输入的 CAD 世界系可能差一个
    固定旋转 base_frame = (xb, yb, zb)（世界系 -> 基座系）。若 J0 恰沿世界 Z
    则 base_frame 为恒等。返回的 rows 在该基座系下成立。
    """
    dirs = [v_unit(a[1]) for a in axes]
    pts = [a[0] for a in axes]

    # ---- 基座标架：z0 = J0 轴线；x0 取指向 J1 轴线的垂线方向 ----
    p0, q0 = line_closest_feet(pts[0], dirs[0], pts[1], dirs[1])
    w = v_sub(q0, p0)
    x_hint = v_sub(w, v_scale(dirs[0], v_dot(w, dirs[0])))
    if v_norm(x_hint) < 1e-9:
        # J0/J1 轴线相交（w 退化）：改用后续轴线在基座平面内的投影方向。
        # 不能用世界坐标轴兜底——那不是旋转等变的，会导致基座 X 的选择
        # 随模型朝向漂移（曾由自检"任意基座"用例暴露）。
        for k in range(1, 6):
            cand = v_sub(dirs[k], v_scale(dirs[0], v_dot(dirs[k], dirs[0])))
            if v_norm(cand) > 1e-9:
                x_hint = cand
                break
    base = orthonormal_frame(dirs[0], x_hint)

    # 所有轴线变换到规范基座系（此时 J0 方向 = +Z，与 DH 重建链一致）
    pts = [express_in_frame(base, p) for p in pts]
    dirs = [express_in_frame(base, d) for d in dirs]
    tool_point = express_in_frame(base, tool_point) if tool_point is not None else None

    x = (1.0, 0.0, 0.0)
    # 基座标架 = 世界原点（约定：世界原点应位于 J0 轴线上），row0 的 d 由此算起
    o = (0.0, 0.0, 0.0)

    rows = []
    # ---- 相邻轴线 i -> i+1：经典 DH 公垂线构造 ----
    for i in range(5):
        z_prev, z_next = dirs[i], dirs[i + 1]
        n = v_cross(z_prev, z_next)
        nn = v_norm(n)
        p, q = line_closest_feet(pts[i], z_prev, pts[i + 1], z_next)
        dist = v_dist(q, p)

        if nn > EPS_PARALLEL:
            x_next = v_scale(n, 1.0 / nn)
            alpha = math.atan2(nn, v_dot(z_prev, z_next))
            a = v_dot(v_sub(q, p), x_next)
            d = v_dot(v_sub(p, o), z_prev)
            theta = signed_angle(x, x_next, z_prev)
            o_next = q
        elif dist > EPS_COINCIDENT:
            # 平行：x_{i+1} 取两轴线连线方向（DH 的经典处理）
            x_next = v_unit(v_sub(q, p))
            alpha = 0.0
            a = dist if v_dot(v_sub(q, p), x_next) >= 0 else -dist
            d = v_dot(v_sub(p, o), z_prev)
            theta = signed_angle(x, x_next, z_prev)
            o_next = q
        else:
            # 共线：无几何约束，保持 x 不变
            x_next = x
            alpha = a = d = 0.0
            theta = 0.0
            o_next = p

        rows.append((d, a, alpha, theta))
        x, o = x_next, o_next

    # ---- 末行（工具标架）：原点精确放到 TCP，z 沿 J5 轴线 ----
    # TCP 通常不在 J5 轴线上：a = 横向偏距(沿 x6)，d = 轴向距离，
    # theta = x5 -> x6（指向横向偏距方向）的转角，alpha = 0。
    t = tool_point if tool_point is not None else pts[5]
    r = v_sub(t, o)
    d5 = v_dot(r, dirs[5])
    lateral = v_sub(r, v_scale(dirs[5], d5))
    lateral_norm = v_norm(lateral)
    if lateral_norm > 1e-9:
        x6 = v_scale(lateral, 1.0 / lateral_norm)
        a5 = lateral_norm
        theta5 = signed_angle(x, x6, dirs[5])
    else:
        a5 = 0.0
        theta5 = 0.0
    rows.append((d5, a5, 0.0, theta5))
    return rows, base


# ---------------------------- DH 链重建（自校验用） ----------------------------
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


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def dh_chain(rows, q):
    T = [[[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]]
    for i, row in enumerate(rows):
        T.append(mat_mul(T[-1], dh_transform(row, q[i])))
    return T


def validate(rows, axes, tool_point=None):
    """从 DH 重建各级轴线与 TCP，与输入比对（方向一致 + 原点落在轴线上）。"""
    T = dh_chain(rows, [0.0] * 6)
    worst_dir, worst_pos = 0.0, 0.0
    for i in range(6):
        z = (T[i][0][2], T[i][1][2], T[i][2][2])
        o = (T[i][0][3], T[i][1][3], T[i][2][3])
        worst_dir = max(worst_dir, v_dist(z, axes[i][1]))
        worst_pos = max(worst_pos, point_line_distance(o, axes[i][0], axes[i][1]))
    if tool_point is not None:
        tcp = (T[6][0][3], T[6][1][3], T[6][2][3])
        worst_pos = max(worst_pos, v_dist(tcp, tool_point))
    return worst_dir, worst_pos


# ---------------------------- 输出 ----------------------------
def format_cpp(rows):
    lines = [
        "// 由 tools/dh_from_axes.py 生成，粘贴到 RobotArm/Inc/arm_config.h 的 kDh[]",
        "// (d, a, alpha, theta_offset)，单位 m / rad；填好后置 kDhParamsMeasured = 1",
        "constexpr DhRow kDh[kJointCount] = {",
    ]
    names = ["J0", "J1", "J2", "J3", "J4", "J5"]
    for i, (d, a, alpha, off) in enumerate(rows):
        lines.append(f"    /* {names[i]} */ {{ {d:.6f}f, {a:.6f}f, {alpha:+.8f}f, {off:+.8f}f }},")
    lines.append("};")
    return "\n".join(lines)


def parse_axes_arg(argv):
    axes, tool, mode = [], None, "run"
    i = 0
    while i < len(argv):
        if argv[i] == "--selftest":
            mode = "selftest"
        elif argv[i] == "--axes":
            vals = [float(x) for x in argv[i + 1].split(",")]
            if len(vals) != 6:
                raise SystemExit("--axes 需要 6 个数: px,py,pz,dx,dy,dz")
            axes.append(vals)
            i += 1
        elif argv[i] == "--tool":
            tool = [float(x) for x in argv[i + 1].split(",")]
            i += 1
        i += 1
    return mode, axes, tool


def selftest():
    """用 arm_config.h 当前 kDh 生成轴线做往返：算法重建的轴线必须与输入重合。
    覆盖两种情况：J0 沿世界 Z（顺）与 J0 任意方向（需基座旋转）。"""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "test"))
    from verify_kinematics import parse_config, CONFIG_H, fk_chain

    def rot(axis, ang):
        ax = v_unit(axis)
        c, s = math.cos(ang), math.sin(ang)
        x, y, z = ax
        t = 1 - c
        return ((t*x*x + c,    t*x*y - s*z, t*x*z + s*y),
                (t*x*y + s*z,  t*y*y + c,   t*y*z - s*x),
                (t*x*z - s*y,  t*y*z + s*x, t*z*z + c))

    def apply(R, v):
        return (v_dot(R[0], v), v_dot(R[1], v), v_dot(R[2], v))

    cfg = parse_config(CONFIG_H.read_text(encoding="utf-8"))
    T = fk_chain(cfg, [0.0] * 6)
    axes0 = [((T[i][0][3], T[i][1][3], T[i][2][3]),
              (T[i][0][2], T[i][1][2], T[i][2][2])) for i in range(6)]
    tcp0 = (T[6][0][3], T[6][1][3], T[6][2][3])

    # 情形1：J0 恰沿世界 Z（恒等基座）
    rows, base = dh_from_axes(axes0, tcp0)
    axes_b = [(express_in_frame(base, p), express_in_frame(base, d)) for p, d in axes0]
    d1, p1 = validate(rows, axes_b, express_in_frame(base, tcp0))

    # 情形2：整条臂随机旋转（J0 指向任意方向）——曾暴露基座系 bug 的场景
    R = rot((0.3, -0.7, 0.6), 1.234)
    axes_r = [(apply(R, p), apply(R, d)) for p, d in axes0]
    tcp_r = apply(R, tcp0)
    rows_r, base_r = dh_from_axes(axes_r, tcp_r)
    axes_rb = [(express_in_frame(base_r, p), express_in_frame(base_r, d)) for p, d in axes_r]
    d2, p2 = validate(rows_r, axes_rb, express_in_frame(base_r, tcp_r))
    # 旋转后求出的 rows 应与情形1完全一致（同一构型，仅基座表达不同）
    d3 = max(abs(a - b) for r1, r2 in zip(rows, rows_r) for a, b in zip(r1, r2))

    assert max(d1, p1, d2, p2, d3) < 1e-9, f"往返校验失败: {d1},{p1},{d2},{p2},{d3}"
    print(f"[SELFTEST PASS] 恒等基座(方向差{d1:.1e}/位置差{p1:.1e}), "
          f"任意基座(方向差{d2:.1e}/位置差{p2:.1e}), 两情形 DH 一致(差{d3:.1e})")
    print()
    print(format_cpp(rows))


def main():
    mode, axes_arg, tool_arg = parse_axes_arg(sys.argv[1:])
    if mode == "selftest":
        selftest()
        return

    axes_in = axes_arg if axes_arg else AXES_MM
    if len(axes_in) != 6:
        raise SystemExit(f"需要 6 条轴线，当前 {len(axes_in)} 条")
    axes = [((a[0] / 1000.0, a[1] / 1000.0, a[2] / 1000.0),
             v_unit((a[3], a[4], a[5]))) for a in axes_in]
    tool = None
    if tool_arg if tool_arg is not None else TOOL_POINT_MM:
        t = tool_arg if tool_arg is not None else TOOL_POINT_MM
        tool = (t[0] / 1000.0, t[1] / 1000.0, t[2] / 1000.0)

    rows, base = dh_from_axes(axes, tool)
    axes_base = [(express_in_frame(base, p), express_in_frame(base, d)) for p, d in axes]
    tool_base = express_in_frame(base, tool) if tool is not None else None
    worst_dir, worst_pos = validate(rows, axes_base, tool_base)

    print("输入轴线（零位、CAD 世界坐标系）:")
    for i, (p, d) in enumerate(axes):
        print(f"  J{i}: 过点 ({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f}) mm,"
              f" 方向 ({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f})")
    print()
    xb, yb, zb = base
    print(f"基座标架（在 CAD 世界系下）: X=({xb[0]:+.4f},{xb[1]:+.4f},{xb[2]:+.4f})"
      f" Y=({yb[0]:+.4f},{yb[1]:+.4f},{yb[2]:+.4f})"
      f" Z=({zb[0]:+.4f},{zb[1]:+.4f},{zb[2]:+.4f})  ← Z 即 J0 轴线")
    print("（DH/FK 的坐标都在这个基座系下；若 J0 原本就沿世界 Z，基座系=世界系）")
    print()
    print(format_cpp(rows))
    print()
    status = "OK" if (worst_dir < 1e-9 and worst_pos < 1e-9) else "异常！请检查输入"
    print(f"[校验] 重建轴线 vs 输入轴线: 方向偏差 {worst_dir:.2e}, 位置偏差 {worst_pos:.2e} m -> {status}")
    print()
    print("提醒:")
    print("  1) theta_offset 对应'CAD 零位'；电机零位仍需按框架文档 4.1 节实测标定。")
    print("  2) 差速腕的 kWristRollGain/kWristPitchGain 需查 CAD 齿轮比或实测。")
    print("  3) 填入 arm_config.h 后运行: python test/verify_kinematics.py")


if __name__ == "__main__":
    main()
