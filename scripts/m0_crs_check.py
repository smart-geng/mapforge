# -*- coding: utf-8 -*-
"""M0 作业④：SHP↔MAP 坐标真伪核验——同路口叠合 + GCJ02 假设检验。

三个假设分别计算 MAP 点到 IBD 车道中心线的最近距离分布：
  H0 同坐标系（直接叠合）
  H1 SHP=WGS84、MAP=GCJ02（把 SHP 加偏后叠合）
  H2 SHP=GCJ02、MAP=WGS84（把 MAP 加偏后叠合）
车道级数据若同源同系，H0 距离中位应 <2 m（MAP 中线点落在某条车道附近）。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import shapefile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.mapir.crs_probe import wgs2gcj

SRC = ROOT / "v2x_map_xml"
SHP = ROOT / "shp_0222-0326"
OUT = ROOT / "out"
R_EARTH = 6378137.0
FILES = ["map凤阁路-金剑路路口node16.xml", "map凤苑路-金玥路node4.xml", "map含金路-金剑路路口node17.xml"]


def load_map_points(path):
    n = parse_map_xml(str(path))
    pts = []
    for lk in n.links:
        pts += lk.points
        for ln in lk.lanes:
            pts += ln.points
    return n, np.asarray(pts)                      # (lon, lat) 度


def load_shp_segments(bbox):
    """IBD_LANE_LINK 中与 bbox 相交的车道线段集合 [(p0, p1), ...]（度）。"""
    r = shapefile.Reader(str(SHP / "IBD_LANE_LINK"))
    segs = []
    for sh in r.iterShapes():
        b = sh.bbox
        if b[2] < bbox[0] or b[0] > bbox[2] or b[3] < bbox[1] or b[1] > bbox[3]:
            continue
        pts = np.asarray(sh.points)
        for i in range(len(pts) - 1):
            segs.append((pts[i], pts[i + 1]))
    return segs


def project(pts, lat0, lon0):
    pts = np.asarray(pts, dtype=float)
    x = np.radians(pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def nearest_dists(query_xy, seg_p0, seg_p1):
    """query 点到线段集的最近距离（向量化粗筛：线段中点 KDTree 取近邻 32 条精算）。"""
    from scipy.spatial import cKDTree
    mid = 0.5 * (seg_p0 + seg_p1)
    tree = cKDTree(mid)
    k = min(32, mid.shape[0])
    _, idx = tree.query(query_xy, k=k)
    if k == 1:
        idx = idx[:, None]
    out = np.empty(query_xy.shape[0])
    for i, (p, ids) in enumerate(zip(query_xy, idx)):
        a = seg_p0[ids]
        b = seg_p1[ids]
        ab = b - a
        L2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
        t = np.clip(((p - a) * ab).sum(axis=1) / L2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        out[i] = np.sqrt(((proj - p) ** 2).sum(axis=1).min())
    return out


def offset_vector(query_xy, seg_p0, seg_p1):
    """粗略系统偏移：query 点到最近线段投影点的向量均值。"""
    from scipy.spatial import cKDTree
    mid = 0.5 * (seg_p0 + seg_p1)
    tree = cKDTree(mid)
    _, idx = tree.query(query_xy)
    a = seg_p0[idx]
    b = seg_p1[idx]
    ab = b - a
    L2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
    t = np.clip(((query_xy - a) * ab).sum(axis=1) / L2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d = query_xy - proj
    return d.mean(axis=0), np.linalg.norm(d, axis=1)


def main():
    OUT.mkdir(exist_ok=True)
    rep = ["# CRS 核验报告：SHP(IBD) ↔ MAP XML 坐标真伪", "",
           "IBD 声称 WGS84（.prj 自定义 WKT 无 EPSG）；MAP XML 未声明。三假设叠合检验：", ""]
    verdicts = []
    for fname in FILES:
        node, map_deg = load_map_points(SRC / fname)
        lat0, lon0 = node.ref_lat, node.ref_lon
        pad = 0.004
        bbox = (map_deg[:, 0].min() - pad, map_deg[:, 1].min() - pad,
                map_deg[:, 0].max() + pad, map_deg[:, 1].max() + pad)
        segs = load_shp_segments(bbox)
        if not segs:
            rep.append(f"- {fname}: bbox 内无 SHP 车道线，跳过")
            continue
        p0_deg = np.asarray([s[0] for s in segs])
        p1_deg = np.asarray([s[1] for s in segs])

        cases = {
            "H0 同系直接叠合": (map_deg, p0_deg, p1_deg),
            "H1 SHP→GCJ02 后叠合": (map_deg,
                                    np.asarray([wgs2gcj(x, y) for x, y in p0_deg]),
                                    np.asarray([wgs2gcj(x, y) for x, y in p1_deg])),
            "H2 MAP→GCJ02 后叠合": (np.asarray([wgs2gcj(x, y) for x, y in map_deg]), p0_deg, p1_deg),
        }
        rep.append(f"## {fname}（{map_deg.shape[0]} MAP 点，SHP 线段 {len(segs)}）")
        stats = {}
        for name, (q, a, b) in cases.items():
            qxy = project(q, lat0, lon0)
            axy = project(a, lat0, lon0)
            bxy = project(b, lat0, lon0)
            d = nearest_dists(qxy, axy, bxy)
            stats[name] = (np.median(d), np.percentile(d, 90), d.max())
            rep.append(f"- {name}: 距离中位 {np.median(d):.2f} m / p90 {np.percentile(d,90):.2f} m / max {d.max():.2f} m")
        vec, _ = offset_vector(project(map_deg, lat0, lon0),
                               project(p0_deg, lat0, lon0), project(p1_deg, lat0, lon0))
        rep.append(f"- H0 残余系统偏移向量（MAP−SHP）：dx={vec[0]:+.2f} m, dy={vec[1]:+.2f} m")
        best = min(stats, key=lambda k: stats[k][0])
        verdicts.append((fname, best, stats[best][0], stats["H0 同系直接叠合"][0]))
        rep.append("")

    rep += ["## 结论", ""]
    all_h0 = all(b.startswith("H0") for _, b, _, _ in verdicts)
    for fname, best, dmed, h0med in verdicts:
        rep.append(f"- {fname}: 最优假设 {best}（中位 {dmed:.2f} m）")
    if all_h0 and all(d < 3.0 for _, _, d, _ in verdicts):
        rep += ["", "**判定：SHP 与 MAP XML 坐标同源同系（H0 全胜，车道级距离量级）。**",
                "两者相对一致 ≠ 绝对为真 WGS84：与真实地心坐标的绝对校验仍需实测控制点/权威影像（遗留），",
                "但生产链路内 SHP↔MAP 互转不存在坐标系错配风险；GCJ02 假设（H1/H2）均被数据否定（偏移应≈数百米）。"]
    else:
        rep += ["", "**判定：存在坐标系错配嫌疑，逐路口见上；禁止进入生产转换（crs_integrity=suspect）。**"]
    report = "\n".join(rep)
    (OUT / "crs_check_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
