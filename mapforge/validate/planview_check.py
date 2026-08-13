# -*- coding: utf-8 -*-
"""planView 连续性检查器（方案 5.7 门禁）：相邻几何段端点位置/航向连续性。

line/arc/spiral 统一数值前向积分（Simpson 级密度），阈值：位置 <1mm、航向 <0.001 rad。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

POS_TOL = 1e-3       # 1 mm
HDG_TOL = 1e-3       # 0.001 rad


def _end_pose(x, y, h, length, k0, k1):
    """数值积分几何段末端位姿（曲率沿弧长线性 k0→k1，覆盖 line/arc/spiral）。"""
    n = max(64, int(length * 8))
    s = np.linspace(0.0, length, n + 1)
    c = k0 + (k1 - k0) * s / max(length, 1e-12)
    theta = h + np.concatenate([[0.0], np.cumsum(0.5 * (c[1:] + c[:-1]) * np.diff(s))])
    dx = np.cos(theta)
    dy = np.sin(theta)
    xe = x + np.trapezoid(dx, s)
    ye = y + np.trapezoid(dy, s)
    return xe, ye, float(theta[-1])


def check_road_planview(road_el):
    """返回 [(seg_idx, pos_gap_m, hdg_gap_rad)]（相邻段衔接差）。"""
    gaps = []
    prev_end = None
    for i, g in enumerate(road_el.findall("planView/geometry")):
        x, y, h = float(g.get("x")), float(g.get("y")), float(g.get("hdg"))
        L = float(g.get("length"))
        child = g[0]
        if child.tag == "line":
            k0 = k1 = 0.0
        elif child.tag == "arc":
            k0 = k1 = float(child.get("curvature"))
        elif child.tag == "spiral":
            k0, k1 = float(child.get("curvStart")), float(child.get("curvEnd"))
        else:                       # poly3/paramPoly3 暂不积分
            prev_end = None
            continue
        if prev_end is not None:
            dp = math.hypot(x - prev_end[0], y - prev_end[1])
            dh = abs((h - prev_end[2] + math.pi) % (2 * math.pi) - math.pi)
            gaps.append((i, dp, dh))
        prev_end = _end_pose(x, y, h, L, k0, k1)
    return gaps


def check_file(path: str):
    """整文件检查。返回 {road_id: [(seg_idx, pos_gap, hdg_gap)...]} 仅含超阈值项，以及统计。"""
    root = ET.parse(path).getroot()
    bad, n_pairs, worst_p, worst_h = {}, 0, 0.0, 0.0
    for r in root.findall("road"):
        gaps = check_road_planview(r)
        n_pairs += len(gaps)
        for _, dp, dh in gaps:
            worst_p, worst_h = max(worst_p, dp), max(worst_h, dh)
        viol = [g for g in gaps if g[1] > POS_TOL or g[2] > HDG_TOL]
        if viol:
            bad[r.get("id")] = viol
    return {"violations": bad, "pairs_checked": n_pairs,
            "worst_pos_gap_m": worst_p, "worst_hdg_gap_rad": worst_h}
