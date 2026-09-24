# -*- coding: utf-8 -*-
"""Spike-C：IBD 曲率验收通道——拟合 κ(s) vs 逐形点实测 CURVATURE。

选两条样本车道（一条含明显弯道、一条近直线对照），验证"符合实际"的量化验收报表：
- 拟合曲线曲率函数与 IBD_LANE_POSITION 实测曲率序列的偏差统计；
- 直线段实测曲率均值（应≈0，验证"直就是直"）。
注：IBD 坐标为经纬度（声称 WGS84 未核验，spike 只做形状级验证不做绝对定位）；
    实测 CURVATURE 符号约定未知，对比用绝对值并另报符号一致率。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import shapefile  # pyshp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.ops.refline_fit import fit_polyline, eval_planview, lateral_deviation, arclength

SHP_DIR = ROOT / "shp_0222-0326"
OUT = ROOT / "out"
R_EARTH = 6378137.0


def scan_lane_position():
    """扫 IBD_LANE_POSITION（PointZ 密集采样点层，~1m 间距）：每 PID → [(seq, κ, lon, lat)]。

    注：该层是独立采样点几何（非 LANE_LINK 形点属性），列名 LINK_PID 值实为 LANE_PID（资料盘点 2.3）。"""
    r = shapefile.Reader(str(SHP_DIR / "IBD_LANE_POSITION"))
    data = {}
    for sr in r.iterShapeRecords(fields=["LINK_PID", "SEQ_NUM", "CURVATURE"]):
        pid, seq, cur = sr.record
        pt = sr.shape.points[0] if sr.shape.points else None
        if pt is None:
            continue
        try:
            data.setdefault(str(pid).strip(), []).append(
                (int(str(seq).strip()), float(cur), float(pt[0]), float(pt[1])))
        except (ValueError, TypeError):
            continue
    return data


def project_local(deg_pts: np.ndarray):
    lat0 = deg_pts[:, 1].mean()
    lon0 = deg_pts[:, 0].mean()
    x = np.radians(deg_pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(deg_pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def kappa_of_planview(pv, s_query: np.ndarray):
    """planView 的 κ(s)（分段常数）。"""
    bounds, ks = [], []
    s0 = 0.0
    for seg in pv.segs:
        bounds.append((s0, s0 + seg.length))
        ks.append(seg.curvature if seg.kind == "arc" else 0.0)
        s0 += seg.length
    out = np.zeros_like(s_query)
    for (a, b), k in zip(bounds, ks):
        out[(s_query >= a) & (s_query <= b)] = k
    return out


def analyse(pid: str, rows: list[tuple[int, float, float, float]], label: str, lines: list[str]):
    rows = sorted(rows)
    k_meas = np.array([k for _, k, _, _ in rows])
    deg = np.array([(lon, lat) for _, _, lon, lat in rows])
    pts = project_local(deg)
    keep = np.concatenate([[True], np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-3])
    pts, k_meas = pts[keep], k_meas[keep]

    pv, _ = fit_polyline(pts, kappa_th=1 / 800, min_seg_len=10.0, smooth_win=7, refine=True)
    s = arclength(pts)
    k_fit = kappa_of_planview(pv, s)
    recon = eval_planview(pv, step=1.0)
    dmax, dmean = lateral_deviation(recon, pts)

    abs_err = np.abs(np.abs(k_fit) - np.abs(k_meas))
    line_mask = k_fit == 0.0
    arc_mask = ~line_mask
    sign_agree = None
    if arc_mask.any():
        nz = arc_mask & (np.abs(k_meas) > 1e-6)
        if nz.any():
            sign_agree = float(np.mean(np.sign(k_fit[nz]) == np.sign(k_meas[nz])))
    segstr = " ".join(("A" if sg.kind == "arc" else "L") + f"{sg.length:.0f}m"
                      + (f"(R={1/abs(sg.curvature):.0f})" if sg.kind == "arc" and sg.curvature else "")
                      for sg in pv.segs)
    lines.append(
        f"- {label} PID={pid}：{pts.shape[0]} 点 / {s[-1]:.0f} m，实测 |κ| max {np.abs(k_meas).max():.5f}"
        f"（R_min≈{1/max(np.abs(k_meas).max(),1e-9):.0f} m）")
    lines.append(f"  拟合 [{segstr}]  横向偏差 max {dmax*100:.1f} / mean {dmean*100:.1f} cm")
    lines.append(
        f"  |κ| 偏差：RMS {np.sqrt((abs_err**2).mean()):.6f}  max {abs_err.max():.6f} (1/m)；"
        f"直线段实测 |κ| 均值 {np.abs(k_meas[line_mask]).mean() if line_mask.any() else float('nan'):.6f}"
        f"（越小越证明'直就是直'）"
        + (f"；弧段符号一致率 {sign_agree*100:.0f}%" if sign_agree is not None else ""))


def main():
    OUT.mkdir(exist_ok=True)
    lines = ["# Spike-C 报告：IBD 曲率验收通道", ""]
    data = scan_lane_position()
    stats = []
    for pid, ks in data.items():
        if len(ks) < 100:
            continue
        arr = np.abs(np.array([k for _, k, _, _ in ks]))
        stats.append((pid, len(ks), arr.max(), np.median(arr)))
    curvy = max((x for x in stats if x[2] < 1 / 15), key=lambda x: x[3], default=None)  # 按中位|κ|选整体弯道
    straight = min(stats, key=lambda x: (x[3], x[2]), default=None)
    lines.append(f"扫描 IBD_LANE_POSITION：{len(data)} 条车道有逐点曲率记录，候选统计 {len(stats)} 条（≥100 点）")
    lines.append("")
    if curvy:
        analyse(curvy[0], data[curvy[0]], "弯道样本", lines)
    if straight:
        analyse(straight[0], data[straight[0]], "直线对照", lines)
    report = "\n".join(lines)
    (OUT / "spike_c_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
