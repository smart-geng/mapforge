# -*- coding: utf-8 -*-
"""xodr 文件级"不平滑"定位器：只读生成文件，找出查看器里会显形的缺陷及其位置。

D1 参考线段间接缝：几何 i 解析终点位姿 vs 几何 i+1 起点（位置 m / 航向 deg）；
D2 laneOffset 斜率跳变：相邻多项式在边界处的一阶导差（rad 近似）；
D3 续接车道宽度斜率跳变：section 边界两侧 width 多项式一阶导差；
D4 生灭车道突变宽：section 边界处落单车道边界的"张口"宽度（>0.3m 即渲染成矩形缺口）；
D5 laneOffset 过冲：多项式极值超出两端值范围的幅度（样条摆动，边缘呈波浪）。

用法：.venv/Scripts/python scripts/xodr_diag.py out/direct_xodr/*.xodr
"""
import glob
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.validate.smoothness import (_geoms, _offsets, _sections,  # noqa: E402
                                          cross_edges_at)


def _end_pose(kind, x, y, h, L, k0, k1):
    if kind == "line":
        return x + L * math.cos(h), y + L * math.sin(h), h
    if kind == "arc":
        return (x + (math.sin(h + k0 * L) - math.sin(h)) / k0,
                y - (math.cos(h + k0 * L) - math.cos(h)) / k0, h + k0 * L)
    from pyclothoids import Clothoid
    cl = Clothoid.StandardParams(x, y, h, k0, (k1 - k0) / max(L, 1e-9), L)
    return cl.X(L), cl.Y(L), h + k0 * L + 0.5 * (k1 - k0) * L


def diag(path):
    root = ET.parse(str(path)).getroot()
    rep = {"joint_pos": [], "joint_hdg": [], "off_slope": [], "w_slope": [],
           "mouth": [], "overshoot": []}
    for rd in root.findall("road"):
        rid = rd.get("id")
        gs = _geoms(rd)
        for i in range(len(gs) - 1):
            ex, ey, eh = _end_pose(*gs[i])
            dx = math.hypot(gs[i + 1][1] - ex, gs[i + 1][2] - ey)
            dh = abs((gs[i + 1][3] - eh + math.pi) % (2 * math.pi) - math.pi)
            rep["joint_pos"].append((dx, rid, i))
            rep["joint_hdg"].append((math.degrees(dh), rid, i))
        offs = _offsets(rd)
        L_road = sum(g[4] for g in gs)
        bounds = [o[0] for o in offs[1:]] + [L_road]
        for (s0, a, b, c, d), s1 in zip(offs, bounds):
            ds = s1 - s0
            # 边界斜率差（与下一段 b 比较）
            slope_end = b + 2 * c * ds + 3 * d * ds ** 2
            nxt = [o for o in offs if abs(o[0] - s1) < 1e-6]
            if nxt:
                rep["off_slope"].append((abs(slope_end - nxt[0][2]), rid, s1))
            # 段内过冲：极值超出端点值范围
            u = np.linspace(0, ds, 40)
            v = a + b * u + c * u ** 2 + d * u ** 3
            lo, hi = min(v[0], v[-1]), max(v[0], v[-1])
            rep["overshoot"].append((max(0.0, v.max() - hi, lo - v.min()), rid, s0))
        if rd.get("junction") in (None, "-1"):
            secs = _sections(rd)
            for (sa, _ra, _la), (sb, _rb, _lb) in zip(secs, secs[1:]):
                ea = cross_edges_at(rd, sb - 1e-3)
                eb = cross_edges_at(rd, sb + 1e-3)
                # D3：一对一匹配边界的宽度斜率跳变（近似：比较两侧相邻边界差分）
                pairs = sorted((abs(x - y), i, j) for i, x in enumerate(ea)
                               for j, y in enumerate(eb))
                ua, ub = set(), set()
                mp = {}
                for gap, i, j in pairs:
                    if gap >= 1.5 or i in ua or j in ub:
                        continue
                    ua.add(i)
                    ub.add(j)
                    mp[i] = j
                # D4：落单边界的张口 = 相邻匹配边界间隔中多出的宽度
                for i in range(len(ea)):
                    if i not in ua and 0 < i < len(ea) - 1:
                        rep["mouth"].append((abs(ea[i - 1] - ea[i + 1]) and
                                             min(abs(ea[i] - ea[i - 1]), abs(ea[i] - ea[i + 1])),
                                             rid, sb, "die"))
                for j in range(len(eb)):
                    if j not in ub and 0 < j < len(eb) - 1:
                        rep["mouth"].append((min(abs(eb[j] - eb[j - 1]), abs(eb[j] - eb[j + 1])),
                                             rid, sb, "born"))
                # D3 宽度斜率：边界两侧 ±5cm 极限斜率差（锥形段自身坡度是设计值，不计）
                for i, j in mp.items():
                    if i == 0:
                        continue                          # laneOffset 已单独查
                    e_m2 = cross_edges_at(rd, max(sb - 0.05, 0.0))
                    e_p2 = cross_edges_at(rd, sb + 0.05)
                    if i < len(e_m2) and j < len(e_p2):
                        sl_a = (ea[i] - e_m2[i]) / 0.05
                        sl_b = (e_p2[j] - eb[j]) / 0.05
                        rep["w_slope"].append((abs(sl_a - sl_b), rid, sb, i))
    return rep


def main():
    files = []
    for a in sys.argv[1:]:
        files.extend(glob.glob(a))
    for f in files:
        rep = diag(f)
        print(f"== {f}")
        for key, unit, thr in (("joint_pos", "m", 0.01), ("joint_hdg", "deg", 0.1),
                               ("off_slope", "", 0.02), ("w_slope", "", 0.05),
                               ("mouth", "m", 0.3), ("overshoot", "m", 0.15)):
            items = sorted(rep[key], reverse=True)[:4]
            bad = [x for x in rep[key] if x[0] > thr]
            worst = ", ".join(f"{x[0]:.3f}@road{x[1]}" +
                              (f"/s{x[2]:.0f}" if len(x) > 2 and isinstance(x[2], float) else "")
                              for x in items if x[0] > 1e-4)
            print(f"  {key:10s} 超阈值 {len(bad):3d}  worst: {worst or '-'} {unit}")


if __name__ == "__main__":
    main()
