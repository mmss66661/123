# fusion_export_axes.py — Fusion 360 脚本：导出零位姿态下的关节轴线
#
# 有两种用法（运行后脚本会先弹窗说明）：
#
# 【用法 A：先选后运行（推荐，最直观）】
#   1. 打开模型，把各关节摆到零位姿态；
#   2. 用普通的选择工具（快捷键 M）按 J0→J5 的顺序依次点选 6 条关节轴线
#      （轴承孔的圆形边最方便，也可以是圆柱面/直边/构造轴）；
#   3. （可选）第 7 个选 TCP 点（顶点/草图点）；
#   4. 运行本脚本，它直接读取当前选中的内容并输出。
#
# 【用法 B：运行后逐个选】
#   运行后按右下角出现的选择提示逐个点选（提示条在画布右下角，容易错过）。
#
# 输出：在"文本命令"窗口（菜单 查看 → 显示文本命令）打印 AXES_MM 六行，
#       粘贴到 tools/dh_from_axes.py 即可算出 DH 参数。
# 单位：Fusion 内部单位 cm，已换算为 mm。
#
# 兼容性说明（按本机 Fusion 的 adsk 包实测）：
#   - 构造轴类名是 ConstructionAxis，没有 adsk.fusion.WorkAxis；
#   - Circle3D/Cylinder 没有 .center/.axis 属性，统一用 getData() 元组取值；
#   - 实体类型判断一律用 classType() 字符串，不用 isinstance。

import math
import traceback

import adsk.core
import adsk.fusion

CM_TO_MM = 10.0
JOINT_NAMES = ["J0(底部yaw)", "J1(底部pitch)", "J2", "J3(末端yaw)", "J4(腕roll)", "J5(腕pitch)"]
SELECTION_FILTERS = "CircularEdges,CylindricalFaces,LinearEdges,ConstructionLines"


def _unit(v):
    n = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    if n < 1e-12:
        raise RuntimeError("方向向量长度为 0")
    return adsk.core.Vector3D.create(v.x / n, v.y / n, v.z / n)


def _class_type(obj):
    try:
        return obj.classType()
    except Exception:
        return ""


def _axis_from_selection(sel):
    """从选中的实体提取 (point_cm, direction_unit)。优先世界坐标。"""
    ent = sel.entity
    ct = _class_type(ent)

    # 构造轴：geometry -> InfiniteLine3D
    if "ConstructionAxis" in ct:
        geo = getattr(ent, "geometry", None)
        if geo is None:
            raise RuntimeError("构造轴读取不到几何")
        data = geo.getData()          # (ok, origin, direction)
        return data[1], _unit(data[2])

    # BRep 边/面：优先世界坐标（装配环境下 .geometry 是组件坐标）
    geo = getattr(ent, "worldGeometry", None)
    if geo is None:
        geo = getattr(ent, "geometry", None)
    if geo is None:
        raise RuntimeError(f"选中的实体({ct})没有几何，请选圆形边/圆柱面/直边/构造轴")

    gct = _class_type(geo)

    if "Circle3D" in gct:            # 圆形边: getData -> (ok, center, normal, radius)
        data = geo.getData()
        return data[1], _unit(data[2])

    if "Cylinder" in gct:            # 圆柱面: getData -> (ok, origin, axis, radius)
        data = geo.getData()
        return data[1], _unit(data[2])

    if "Line3D" in gct:              # 直边: getData -> (ok, startPoint, endPoint)
        data = geo.getData()
        sp, ep = data[1], data[2]
        d = adsk.core.Vector3D.create(ep.x - sp.x, ep.y - sp.y, ep.z - sp.z)
        return sp, _unit(d)

    if "InfiniteLine3D" in gct:      # getData -> (ok, origin, direction)
        data = geo.getData()
        return data[1], _unit(data[2])

    raise RuntimeError(f"不支持的几何类型 {gct}，请选圆形边/圆柱面/直边/构造轴")


def _point_from_selection(sel):
    geo = getattr(sel.entity, "geometry", None)
    if geo is None or not hasattr(geo, "x"):
        raise RuntimeError("TCP 必须选顶点或草图点")
    return (geo.x, geo.y, geo.z)


def _select_axis(ui, name):
    """交互式选择一条关节轴线；全部失败视为用户取消，返回 None。"""
    for f in (SELECTION_FILTERS, "CircularEdges", "Edges"):
        try:
            return ui.selectEntity(
                f"选择 {name} 的关节轴线（圆形边/圆柱面/直边/构造轴）", f)
        except RuntimeError:
            continue
    return None


def _emit(ui, results, tool):
    lines = ["AXES_MM = ["]
    for i, (p, d) in enumerate(results):
        lines.append(f"    ({p.x * CM_TO_MM:.3f}, {p.y * CM_TO_MM:.3f}, {p.z * CM_TO_MM:.3f},"
                     f" {d.x:.6f}, {d.y:.6f}, {d.z:.6f}),  # {JOINT_NAMES[i]}")
    lines.append("]")
    if tool is not None:
        lines.append(f"TOOL_POINT_MM = ({tool[0] * CM_TO_MM:.3f}, {tool[1] * CM_TO_MM:.3f},"
                     f" {tool[2] * CM_TO_MM:.3f})")
    else:
        lines.append("TOOL_POINT_MM = None")
    text = "\n".join(lines)
    print(text)

    # 自动落盘，免去在文本命令窗口里翻找
    import os
    saved = None
    for path in (r"E:\stm32_project\arm\123\tools\axes_output.txt",
                 os.path.join(os.path.dirname(os.path.realpath(__file__)),
                              "axes_output.txt")):
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text + "\n")
            saved = path
            break
        except OSError:
            continue

    summary = ["完成！共读取 %d 条轴线。" % len(results)]
    if saved:
        summary.append(f"结果已保存到:\n{saved}\n(可直接交给 dh_from_axes.py 使用)")
    else:
        summary.append("请在 文本命令 窗口(查看→显示文本命令)复制 AXES_MM")
    ui.messageBox("\n".join(summary))


def run(context):
    ui = None
    print("=== fusion_export_axes 已运行 ===")
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # 先弹说明框：只要脚本真的在跑，这个框一定会出现
        res = ui.messageBox(
            "本脚本导出零位姿态下的 6 条关节轴线。\n\n"
            "用法A(推荐)：先在画布里按 J0→J5 顺序点选 6 条轴线\n"
            "（可再选第 7 个 TCP 顶点），选亮后点确定；\n"
            "用法B：点确定后按右下角的提示逐个选（注意提示条\n"
            "出现在画布右下角，容易错过）。\n\n"
            "当前已选中的实体个数：%d" % ui.activeSelections.count,
            "fusion_export_axes")
        if res != adsk.core.DialogResults.DialogOK:
            return

        # ---------- 用法 A：预选模式 ----------
        count = ui.activeSelections.count
        if count >= 6:
            results = []
            for i in range(6):
                p, d = _axis_from_selection(ui.activeSelections[i])
                results.append((p, d))
                print(f"[预选] {JOINT_NAMES[i]} 已读取")
            tool = None
            if count >= 7:
                try:
                    tool = _point_from_selection(ui.activeSelections[6])
                    print("[预选] TCP 已读取")
                except Exception:
                    print("[预选] 第 7 个实体不是点，忽略 TCP")
            ui.activeSelections.clear()
            _emit(ui, results, tool)
            return

        # ---------- 用法 B：交互模式 ----------
        results = []
        for i, name in enumerate(JOINT_NAMES):
            while True:
                sel = _select_axis(ui, name)
                if sel is None:
                    ui.messageBox("已取消（没有读到 %s，已读取 %d 条）" % (name, len(results)))
                    return
                try:
                    p, d = _axis_from_selection(sel)
                    print(f"[交互] {name} 已读取")
                    break
                except RuntimeError as e:
                    ui.messageBox(f"{e}\n请重新选择 {name}")
            results.append((p, d))

        tool = None
        try:
            sel = ui.selectEntity("选择工具中心点 TCP（顶点/草图点，Esc 跳过）",
                                  "Vertices,SketchPoints")
        except RuntimeError:
            sel = None
        if sel is not None:
            tool = _point_from_selection(sel)

        _emit(ui, results, tool)
    except:
        print("失败:\n" + traceback.format_exc())
        if ui:
            ui.messageBox("失败:\n" + traceback.format_exc())
