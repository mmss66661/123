# fusion_check_axis.py — Fusion 360 脚本：核对关节轴线顺序（预选模式，自诊断版）
#
# 用法：
#   1. Run 一次：创建命名构造轴 J0+~J5+（失败会跳过）+ 显示说明；
#   2. 用选择工具(M)点选 1~6 条轴线实体（模型圆边/圆柱面/直边/构造轴，
#      Ctrl 连选，选择顺序 = 报告顺序）；
#   3. 再 Run 一次：报告每个实体的匹配结果 + 原始读数 + 到六条轴的距离。
#
# 自检技巧：先只选浏览器里的 J3+ 构造轴 Run 一次 ——
# 报告必须是 "J3, 重合 <0.001mm"。如果这步就不对，说明 axes_output.txt
# 或坐标系有问题；这步对了再点模型上的边，若模型边全不匹配则是
# 装配坐标问题（本版已自动处理装配变换链）。
#
# 正方向锚点：J0+ = CAD +X；J1+ = +Y；J2+ = (0.707,0,0.707)；
#             J3+ = (0.5,0.707,-0.5)；J4+ ≈ 与 J2 同手性；J5+ 与 J3 反手性。

import math
import re
import traceback
from pathlib import Path

import adsk.core
import adsk.fusion

CM_TO_MM = 10.0
MM_TO_CM = 0.1
AXES_FILE = r"E:\stm32_project\arm\123\tools\axes_output.txt"
ANCHORS = [
    "J0+: CAD 世界 +X（J0 轴即竖直方向）",
    "J1+: CAD 世界 +Y",
    "J2+: (0.707, 0, 0.707)，与 J4 轴同向平行",
    "J3+: (0.5, 0.707, -0.5)，与 J5 共线反向（同一条线）",
    "J4+: (0.709, -0.003, 0.705)，与 J2 平行（腕弯曲轴）",
    "J5+: (-0.5, -0.707, 0.5)，与 J3 共线（工具自旋轴，过 TCP）",
]


def _unit(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-12:
        raise RuntimeError("方向向量长度为 0")
    return (v[0] / n, v[1] / n, v[2] / n)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dist(a, b):
    return math.sqrt(sum(x * x for x in _sub(a, b)))


def line_distance_and_dot(p1, d1, p2, d2):
    """两条直线间距离 + 方向单位向量点积。"""
    c, d = _unit(d1), _unit(d2)
    b = _dot(c, d)
    r = _sub(p2, p1)
    denom = 1.0 - b * b
    if denom < 1e-12:
        t = -_dot(r, d)
        q2 = (p2[0] + d[0] * t, p2[1] + d[1] * t, p2[2] + d[2] * t)
        return _dist(q2, p1), b
    s = (_dot(r, c) - b * _dot(r, d)) / denom
    q1 = (p1[0] + c[0] * s, p1[1] + c[1] * s, p1[2] + c[2] * s)
    t = s * b - _dot(r, d)
    q2 = (p2[0] + d[0] * t, p2[1] + d[1] * t, p2[2] + d[2] * t)
    return _dist(q1, q2), b


def load_axes_mm():
    text = Path(AXES_FILE).read_text(encoding="utf-8", errors="ignore")
    axes = []
    for m in re.finditer(
        r"\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\)",
        text,
    ):
        axes.append(tuple(float(g) for g in m.groups()))
    if len(axes) < 6:
        raise RuntimeError(f"{AXES_FILE} 只解析到 {len(axes)} 条轴线，请先运行 fusion_export_axes 导出")
    return axes[:6]


def world_matrices_of(ent):
    """实体所在组件 -> 世界 的变换矩阵列表（从最内层到最外层）。

    该版本 API 的 Matrix3D 没有 setToProduct，因此不做矩阵合成，
    调用方对点/向量按本列表顺序逐级 transformBy 即可。"""
    mats = []
    occ = getattr(ent, "assemblyContext", None)
    guard = 0
    while occ is not None and guard < 16:
        t = getattr(occ, "transform2", None) or getattr(occ, "transform", None)
        if t is None:
            break
        mats.append(t)
        occ = getattr(occ, "assemblyContext", None)
        guard += 1
    return mats


def axis_from_selection(sel):
    """提取选中实体的世界坐标轴线（mm），自动处理装配变换链。"""
    ent = sel.entity
    geo = getattr(ent, "worldGeometry", None)
    if geo is None:
        geo = getattr(ent, "geometry", None)
    if geo is None:
        raise RuntimeError("没有几何")

    try:
        if "ConstructionAxis" in ent.classType():
            data = geo.getData()                  # (ok, origin, direction)
            p, d = data[1], data[2]
        else:
            gct = geo.classType()
            data = geo.getData()
            if "Line3D" in gct:
                sp, ep = data[1], data[2]
                p = sp
                d = (ep.x - sp.x, ep.y - sp.y, ep.z - sp.z)
            else:                                 # Circle3D / Cylinder / InfiniteLine3D
                p, d = data[1], data[2]
    except RuntimeError:
        raise
    except Exception as e:
        raise RuntimeError(f"读取几何失败: {e}")

    # BRep 实体的 geometry 是组件坐标：装配上下文存在时逐级变换到世界
    if not isinstance(d, tuple):
        d = (d.x, d.y, d.z)
    mats = world_matrices_of(ent)
    if mats and "ConstructionAxis" not in ent.classType():
        pt = adsk.core.Point3D.create(p.x, p.y, p.z)
        vec = adsk.core.Vector3D.create(d[0], d[1], d[2])
        for m in mats:  # 从最内层到最外层逐级变换
            pt.transformBy(m)
            vec.transformBy(m)
        return ((pt.x * CM_TO_MM, pt.y * CM_TO_MM, pt.z * CM_TO_MM),
                _unit((vec.x, vec.y, vec.z)))

    return ((p.x * CM_TO_MM, p.y * CM_TO_MM, p.z * CM_TO_MM), _unit(d))


def create_named_axes(ui, design, axes_mm):
    comp = design.rootComponent
    existing = set()
    for i in range(comp.constructionAxes.count):
        existing.add(comp.constructionAxes.item(i).name)
    made = 0
    for i, (px, py, pz, dx, dy, dz) in enumerate(axes_mm):
        name = f"J{i}+"
        if name in existing:
            continue
        d = _unit((dx, dy, dz))
        origin = adsk.core.Point3D.create(px * MM_TO_CM, py * MM_TO_CM, pz * MM_TO_CM)
        direction = adsk.core.Vector3D.create(d[0], d[1], d[2])
        try:
            inp = comp.constructionAxes.createInput()
            if not inp.setByLine(adsk.core.InfiniteLine3D.create(origin, direction)):
                continue
            ax = comp.constructionAxes.add(inp)
            if ax:
                ax.name = name
                made += 1
        except RuntimeError:
            continue
    return made


def run(context):
    ui = None
    print("=== fusion_check_axis 已运行 ===")
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("请先切换到 DESIGN 工作区")
            return

        axes_mm = load_axes_mm()

        # 数据文件自检：报告共线对（J3/J5 共线 = ZYZ 腕的正常设计，不算错误，
        # 但若是"同一个孔点了两遍"造成的就需要重新导出）
        def coaxial(i, j):
            a, b = axes_mm[i], axes_mm[j]
            dist, dot = line_distance_and_dot(
                (a[0], a[1], a[2]), (a[3], a[4], a[5]),
                (b[0], b[1], b[2]), (b[3], b[4], b[5]))
            return dist < 0.01 and abs(dot) > 0.9999
        dup = [(i, j) for i in range(6) for j in range(i + 1, 6) if coaxial(i, j)]
        warn = "" if not dup else (
            "\nℹ 存在共线轴对 %s —— 若设计上本就共线（如前臂yaw与工具roll共轴的"
            "ZYZ腕）属正常；若是导出时同一个孔点了两遍，请重新导出。"
            % ", ".join(f"J{i}/J{j}" for i, j in dup))

        count = ui.activeSelections.count
        if count == 0:
            try:
                made = create_named_axes(ui, design, axes_mm)
            except Exception:
                made = 0
            ui.messageBox(
                ("构造轴 J0+~J5+ 已就绪（新增 %d 根）。\n\n" % made if made
                 else "构造轴创建被环境拒绝，已跳过（不影响核对）。\n\n") +
                "自检：先只选 J3+ 构造轴 Run 一次，报告应为 J3/重合<0.001mm。\n\n"
                "然后按你认定的关节顺序（底座→肩→肘→末端→腕roll→腕pitch）\n"
                "Ctrl 连选六条模型轴线，再 Run 一次看报告。\n\n"
                "正方向锚点：\n" + "\n".join("  " + a for a in ANCHORS) + warn)
            return

        lines = [f"共选中 {count} 个实体：", ""]
        for k in range(count):
            try:
                p, d = axis_from_selection(ui.activeSelections[k])
            except RuntimeError as e:
                lines.append(f"#{k+1}: 无法读取（{e}）")
                continue

            scored = []
            for i, a in enumerate(axes_mm):
                dist, dot = line_distance_and_dot(
                    p, d, (a[0], a[1], a[2]), (a[3], a[4], a[5]))
                scored.append((dist, abs(dot), dot, i))
            # 腕部多条轴线互相相交（距离=0），必须用"方向平行"做判别；
            # 共线设计的轴（如 J3/J5）会同时命中，全部列出
            scored.sort(key=lambda s: (0 if s[1] > 0.9999 else 1, s[0]))
            dist0, adot0, dot0, i0 = scored[0]
            hits = [s for s in scored
                    if s[1] > 0.9999 and s[0] < 0.5] if adot0 > 0.9999 else []

            if hits:
                names = " 和 ".join(f"J{s[3]}" for s in hits)
                verdict = f"匹配 {names}（重合 {hits[0][0]:.3f} mm" + (
                    "，共线设计，两个标签指同一条轴）" if len(hits) > 1 else "）")
            elif adot0 > 0.9999 and dist0 < 20.0:
                verdict = f"平行但不重合：距 J{i0} {dist0:.2f} mm（选错孔了？）"
            else:
                nearest = min(s[0] for s in scored)
                verdict = f"无匹配：没有方向平行的轴线（最近的一条也差 {nearest:.1f} mm）"
            lines.append(
                f"#{k+1}: {verdict}，方向{'同' if dot0 > 0 else '反'}")
            lines.append(
                f"     读数: 过点({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})mm "
                f"方向({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f})")
            lines.append(
                "     距离: " + "  ".join(
                    (f"J{i}={'∥' if ad > 0.9999 else ''}{dst:.1f}"
                     for dst, ad, _, i in sorted(scored, key=lambda s: s[3]))))
        report = "\n".join(lines)
        print(report)
        ui.messageBox(report + warn)
    except:
        print("失败:\n" + traceback.format_exc())
        if ui:
            ui.messageBox("失败:\n" + traceback.format_exc())
