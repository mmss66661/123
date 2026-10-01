#!/usr/bin/env python3
"""step_list_axes.py — 从 STEP 文件中解出所有圆柱面轴线（找关节轴线用）。

STEP(AP203/AP214/AP242) 是纯文本，圆柱面会以 CYLINDRICAL_SURFACE 实体存储，
其轴线由 AXIS2_PLACEMENT_3D + CARTESIAN_POINT + DIRECTION 完整描述。
机械臂的关节轴线通常就是电机座/轴承孔/轴肩的圆柱面轴线——从本工具
列出的清单里挑出 6 条，填入 dh_from_axes.py 的 AXES_MM 即可算 DH。

用法：
    python step_list_axes.py <模型.step>
    python step_list_axes.py --selftest      # 内置样例自检

输出：按"被引用次数、半径"排序的圆柱轴线清单（已按单位换算为 mm，
同轴同径的多个圆柱面已合并）。
"""

import math
import re
import sys
from pathlib import Path

import dh_from_axes as D  # 复用三点/直线几何工具


RE_POINT = re.compile(
    r"#(\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*([^)]*?)\s*\)\s*\)",
    re.S)
RE_DIR = re.compile(
    r"#(\d+)\s*=\s*DIRECTION\s*\(\s*'[^']*'\s*,\s*\(\s*([^)]*?)\s*\)\s*\)",
    re.S)
RE_PLACE = re.compile(
    r"#(\d+)\s*=\s*AXIS2_PLACEMENT_3D\s*\(\s*'[^']*'\s*,\s*(#\d+)\s*"
    r"(?:,\s*(#[\d]+|\$))?\s*(?:,\s*(#[\d]+|\$))?\s*\)",
    re.S)
RE_CYL = re.compile(
    r"#(\d+)\s*=\s*CYLINDRICAL_SURFACE\s*\(\s*'[^']*'\s*,\s*(#\d+)\s*,\s*"
    r"([+\-0-9.eE]+)\s*\)",
    re.S)


def _vec(s):
    vals = [float(x) for x in s.split(",")]
    if len(vals) != 3:
        raise ValueError(f"向量分量数异常: {s!r}")
    return tuple(vals)


def detect_length_unit_mm(text):
    """返回 STEP 长度单位到 mm 的换算系数。"""
    if re.search(r"\.INCH\.", text) or re.search(r"inch\s+length", text, re.I):
        return 25.4
    if re.search(r"SI_UNIT\s*\(\s*\$?\s*,\s*\.METRE\.?\s*\)", text):
        # 没有 MILLI 前缀的 METRE 才是米
        m = re.search(r"SI_UNIT\s*\(\s*(\.[A-Z]+\.?)\s*,\s*\.METRE\.?\s*\)", text)
        if m and m.group(1) in (".MILLI.", "$"):
            return 1.0 if m.group(1) == ".MILLI." else 1000.0
        return 1.0  # 默认按 mm（绝大多数导出为毫米）
    return 1.0


def parse_step_axes(path):
    text = Path(path).read_text(encoding="utf-8", errors="ignore")
    scale = detect_length_unit_mm(text)

    points = {int(m.group(1)): _vec(m.group(2)) for m in RE_POINT.finditer(text)}
    dirs = {int(m.group(1)): _vec(m.group(2)) for m in RE_DIR.finditer(text)}

    placements = {}
    for m in RE_PLACE.finditer(text):
        pid = int(m.group(1))
        p_ref = int(m.group(2)[1:])
        d_ref = m.group(3)
        if d_ref and d_ref.startswith("#"):
            d = dirs.get(int(d_ref[1:]), (0.0, 0.0, 1.0))
        else:
            d = (0.0, 0.0, 1.0)  # AXIS2_PLACEMENT_3D 缺省方向
        placements[pid] = (points.get(p_ref, (0.0, 0.0, 0.0)), d)

    cylinders = []  # (radius_raw, point, dir, surface_id)
    for m in RE_CYL.finditer(text):
        place_id = int(m.group(2)[1:])
        radius = float(m.group(3))
        p, d = placements.get(place_id, ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
        cylinders.append((radius * scale,
                          D.v_scale(p, scale),
                          D.v_unit(d),
                          int(m.group(1))))

    # 合并同轴同径圆柱面（一个轴孔常被多个面引用）
    merged = []
    for radius, p, d, sid in cylinders:
        found = False
        for item in merged:
            same_radius = abs(item["r"] - radius) < max(1e-6, radius * 1e-4)
            parallel = abs(abs(D.v_dot(item["d"], d)) - 1.0) < 1e-9
            coaxial = D.point_line_distance(p, item["p"], item["d"]) < 1e-6
            if same_radius and parallel and coaxial:
                item["count"] += 1
                found = True
                break
        if not found:
            merged.append({"r": radius, "p": p, "d": d, "count": 1})
    merged.sort(key=lambda c: (-c["count"], -c["r"]))
    return merged, scale, len(cylinders)


SELFTEST_STEP = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('test.step','2026-10-01',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=AXIS2_PLACEMENT_3D('',#1,#2,$);
#4=CYLINDRICAL_SURFACE('',#3,4.);
#5=CARTESIAN_POINT('',(0.,0.,160.));
#6=DIRECTION('',(0.,-1.,0.));
#7=AXIS2_PLACEMENT_3D('',#5,#6,$);
#8=CYLINDRICAL_SURFACE('',#7,2.5);
#9=CYLINDRICAL_SURFACE('',#7,2.5);
#10=CARTESIAN_POINT('',(140.,0.,160.));
#11=AXIS2_PLACEMENT_3D('',#10,#6,$);
#12=CYLINDRICAL_SURFACE('',#11,3.25);
ENDSEC;
END-ISO-10303-21;
"""


def selftest():
    tmp = Path(__file__).parent / "_selftest.step"
    tmp.write_text(SELFTEST_STEP, encoding="utf-8")
    try:
        merged, scale, total = parse_step_axes(tmp)
        assert scale == 1.0, f"单位系数异常: {scale}"
        assert total == 4, f"圆柱面数量异常: {total}"
        assert len(merged) == 3, f"合并后数量异常: {len(merged)}"
        # r=2.5 的两片圆柱面应合并为 1 条且 count=2
        item25 = [c for c in merged if abs(c["r"] - 2.5) < 1e-9]
        assert item25 and item25[0]["count"] == 2, "同轴同径合并失败"
        assert D.v_dist(item25[0]["p"], (0.0, 0.0, 160.0)) < 1e-9
        assert D.v_dist(item25[0]["d"], (0.0, -1.0, 0.0)) < 1e-9
        print("[SELFTEST PASS] STEP 解析: 4 个圆柱面 -> 3 条独立轴线, 同轴合并正确")
        for c in merged:
            print(f"  r={c['r']:.2f}mm 过点{tuple(round(x,1) for x in c['p'])} "
                  f"方向{tuple(round(x,2) for x in c['d'])} count={c['count']}")
    finally:
        tmp.unlink(missing_ok=True)


def main():
    if len(sys.argv) < 2 or sys.argv[1] == "--selftest":
        selftest()
        return
    path = sys.argv[1]
    merged, scale, total = parse_step_axes(path)
    print(f"文件: {path}")
    print(f"圆柱面总数 {total}，独立轴线 {len(merged)} 条（单位换算系数 ×{scale} → mm）")
    print()
    for k, c in enumerate(merged):
        p = D.v_scale(c["p"], 1.0)
        print(f"[{k}] r={c['r']:.3f}mm  过点({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})  "
              f"方向({c['d'][0]:+.3f}, {c['d'][1]:+.3f}, {c['d'][2]:+.3f})  "
              f"引用 {c['count']} 次")
    print()
    print("下一步: 挑出 J0~J5 对应的 6 条（通常是电机座/轴承孔的轴线），")
    print("按 dh_from_axes.py 需要的格式填入 AXES_MM（方向取关节正方向，反了加负号）。")


if __name__ == "__main__":
    main()
