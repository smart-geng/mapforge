# -*- coding: utf-8 -*-
"""把 Profile SHP 真实车道边界与生成 OpenDRIVE 车道边缘叠画并量化。

示例：
  python scripts/shp_xodr_overlay.py shp_0222-0326 out/node4.xodr out/overlay.png
"""
from __future__ import annotations

import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate.shp_boundary_fidelity import _point_polyline_distance
from mapforge.validate.smoothness import _sections, lane_edges_at, sample_road_ref
from scripts.xodr_topdown import road_patches

R_EARTH = 6378137.0


def _origin(root):
    text = root.findtext("header/geoReference") or ""
    lat = re.search(r"\+lat_0=([-+0-9.eE]+)", text)
    lon = re.search(r"\+lon_0=([-+0-9.eE]+)", text)
    if not lat or not lon:
        raise ValueError("xodr geoReference 缺 +lat_0/+lon_0")
    return float(lat.group(1)), float(lon.group(1))


def _project(points, lat0, lon0):
    g = np.asarray(points, float)
    x = np.radians(g[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(g[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def _target_edges(road, ds=0.5):
    pts, ss, hh = sample_road_ref(road, ds)
    nrm = np.column_stack([-np.sin(hh), np.cos(hh)])
    secs = _sections(road)
    all_edges, outer = [], []
    sec_els = road.findall("lanes/laneSection")
    for si, (s0, right, left) in enumerate(secs):
        s1 = secs[si + 1][0] if si + 1 < len(secs) else float(road.get("length"))
        mask = (ss >= s0 - 1e-9) & (ss <= s1 + 1e-9)
        if mask.sum() < 2:
            continue
        for side, lanes in (("right", right), ("left", left)):
            if not lanes:
                continue
            offsets = np.array([lane_edges_at(road, s, side=side) for s in ss[mask]])
            for k in range(offsets.shape[1]):
                line = pts[mask] + offsets[:, k, None] * nrm[mask]
                all_edges.append(line)
            line = pts[mask] + offsets[:, -1, None] * nrm[mask]
            path = "right/lane" if side == "right" else "left/lane"
            key = (lambda lane: -int(lane.get("id"))) if side == "right" else (
                lambda lane: int(lane.get("id")))
            lane_els = sorted(sec_els[si].findall(path), key=key)
            source_id = None
            support_s = None
            provenance = {}
            if lane_els:
                ud = lane_els[-1].find("userData[@code='mapforge.source_lane']")
                source_id = ud.get("value") if ud is not None else None
                meta_ud = lane_els[-1].find("userData[@code='mapforge.provenance/v1']")
                if meta_ud is not None and meta_ud.get("value"):
                    try:
                        provenance = json.loads(meta_ud.get("value"))
                        support_s = provenance.get("support_s")
                    except json.JSONDecodeError:
                        support_s = None
            if support_s and len(support_s) == 2:
                q = ss[mask]
                keep = (q >= float(support_s[0]) - 1e-6) & (q <= float(support_s[1]) + 1e-6)
                if keep.sum() >= 2:
                    line = line[keep]
                else:
                    line = np.zeros((0, 2))
            if len(line) >= 2:
                outer.append({
                    "source_lane_id": source_id, "points": line,
                    "eligibility": provenance.get("eligibility"),
                    "exclusion_code": provenance.get("exclusion_code"),
                    "boundary_evidence_trusted": provenance.get(
                        "boundary_evidence_trusted"),
                })
    return all_edges, outer


def _stats(values):
    a = np.asarray(values, float)
    return {"count": int(len(a)), "median_m": float(np.median(a)),
            "p95_m": float(np.percentile(a, 95)), "max_m": float(np.max(a))}


def _densify(line, ds=0.5):
    g = np.asarray(line, float)
    if len(g) < 2:
        return g
    seg = np.linalg.norm(np.diff(g, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= ds:
        return g
    q = np.arange(0.0, s[-1], ds)
    if s[-1] - q[-1] > 1e-9:
        q = np.append(q, s[-1])
    return np.column_stack([np.interp(q, s, g[:, 0]), np.interp(q, s, g[:, 1])])


def render(shp_dir, xodr, output, profile="ibd-smarteditor-v1"):
    root = ET.parse(str(xodr)).getroot()
    lat0, lon0 = _origin(root)
    src = ProfileSource(str(shp_dir), profile)
    src._load_boundaries()
    boundary_geom, boundary_rel = src._bnd

    source_by_road = defaultdict(set)
    roads = []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        roads.append(road)
        for lane in road.findall("lanes/laneSection/right/lane") + road.findall(
                "lanes/laneSection/left/lane"):
            ud = lane.find("userData[@code='mapforge.source_lane']")
            if ud is not None and ud.get("value"):
                source_by_road[road.get("id")].add(ud.get("value"))

    all_source = []
    seen_all = set()
    for road_id, lane_ids in source_by_road.items():
        refs = Counter()
        for lane_id in lane_ids:
            for ids in boundary_rel.get(lane_id, {}).values():
                refs.update(ids)
        for bid, count in refs.items():
            for geom in boundary_geom.get(bid, []):
                line = _project(geom, lat0, lon0)
                key = (bid, len(line), tuple(np.round(line[0], 3)), tuple(np.round(line[-1], 3)))
                if key not in seen_all:
                    seen_all.add(key)
                    all_source.append(line)

    target_all, target_outer_by_road = [], defaultdict(list)
    for road in roads:
        edges, outer = _target_edges(road)
        target_all.extend(edges)
        target_outer_by_road[road.get("id")].extend(outer)

    s2t, t2s = [], []
    paired_source, paired_target = [], []
    per_road = {}
    for road_id in sorted(target_outer_by_road, key=int):
        grouped = defaultdict(list)
        for record in target_outer_by_road[road_id]:
            if record.get("boundary_evidence_trusted") is False:
                continue
            if (record.get("exclusion_code") in {
                    "source-extension", "source-support-unavailable",
                    "source-support-too-short"}
                    and record.get("boundary_evidence_trusted") is not True):
                continue
            if record["source_lane_id"]:
                grouped[record["source_lane_id"]].append(record["points"])
        road_s2t, road_t2s = [], []
        for source_id, parts in grouped.items():
            tp = np.vstack(parts)
            candidates = [_densify(_project(g, lat0, lon0))
                          for g in src.lane_boundary_geometries(source_id)]
            if not candidates:
                continue
            # 对应 lane 的两条边界中，离目标最外缘更近者就是该侧物理外边界。
            sp = min(candidates, key=lambda g: float(np.median(
                _point_polyline_distance(tp, g)[0])))
            a, source_inside = _point_polyline_distance(sp, tp)
            b, target_inside = _point_polyline_distance(tp, sp)
            # 共同支持域：最近点落在对方内部，或端点闭合误差不超过 1.5m。
            # 这样道路数据本身长短不一的尾线不会污染“同一区域形状误差”。
            sm = source_inside | (a <= 1.5)
            tm = target_inside | (b <= 1.5)
            road_s2t.extend(a[sm].tolist())
            road_t2s.extend(b[tm].tolist())
            paired_source.append({"points": sp, "common": sm})
            paired_target.append({"points": tp, "common": tm})
        if road_s2t and road_t2s:
            s2t.extend(road_s2t)
            t2s.extend(road_t2s)
            per_road[road_id] = {"source_to_target": _stats(road_s2t),
                                 "target_to_source": _stats(road_t2s)}

    fig, axes = plt.subplots(2, 3, figsize=(21, 13))
    panels = (
        (axes[0, 0], None, "Full junction overlay"),
        (axes[0, 1], (-45, 45, -45, 45), "Junction center (+/-45 m)"),
        (axes[0, 2], (-220, -80, -22, 22), "West leg detail"),
        (axes[1, 0], (-22, 22, 20, 105), "North leg detail"),
        (axes[1, 1], (20, 165, -22, 22), "East leg detail"),
        (axes[1, 2], (-22, 22, -210, -55), "South leg detail"),
    )
    for ax, crop, title in panels:
        for road in root.findall("road"):
            for poly, _is_junction in road_patches(road):
                ax.add_patch(Polygon(poly, closed=True, facecolor="#77776f",
                                     edgecolor="none", alpha=0.45))
        # 所有参与这些 road 的原始 SHP 边界都画出来。旧图只显示“已配对的
        # 最外缘”，会把中央分隔带/双幅路之间的真实空带隐藏掉，无法判断
        # 蓝色露底究竟来自源数据还是转换错误。
        for line in all_source:
            ax.plot(line[:, 0], line[:, 1], color="#d95f02", lw=0.65,
                    alpha=0.28, linestyle="--")
        # 内部目标车道边缘仅作浅色结构背景；对拍主线只画已按 source id 配对的
        # 物理最外缘，避免把内部边界/短碎线误看成道路外轮廓。
        for line in target_all:
            ax.plot(line[:, 0], line[:, 1], color="#2762d7", lw=0.5, alpha=0.18)
        for record in paired_source:
            line = record["points"]
            ax.plot(line[:, 0], line[:, 1], color="#f08a24", lw=1.3, alpha=0.9)
        for record in paired_target:
            line = record["points"]
            ax.plot(line[:, 0], line[:, 1], color="#2762d7", lw=1.1, alpha=0.9)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.18)
        ax.set_xlabel("local x (m)")
        ax.set_ylabel("local y (m)")
        ax.set_title(title)
        if crop:
            ax.set_xlim(crop[0], crop[1])
            ax.set_ylim(crop[2], crop[3])
        else:
            ax.autoscale()
    axes[0, 0].plot([], [], color="#d95f02", lw=0.8, linestyle="--",
                    label="SHP all participating boundaries")
    axes[0, 0].plot([], [], color="#f08a24", label="SHP paired outer boundaries")
    axes[0, 0].plot([], [], color="#2762d7", label="OpenDRIVE paired outer edges")
    fig.legend(loc="lower center", ncol=2)
    summary = {"schema": "mapforge/shp-xodr-outer-edge-overlay/v2",
               "artifact": str(xodr), "road_outer_edges": {
                   "source_to_target": _stats(s2t), "target_to_source": _stats(t2s)},
               "per_road": per_road}
    fig.suptitle(
        f"{Path(xodr).name} SHP vs OpenDRIVE | road outer edges "
        f"S->T median {summary['road_outer_edges']['source_to_target']['median_m']:.3f} m / "
        f"P95 {summary['road_outer_edges']['source_to_target']['p95_m']:.3f} m | "
        f"T->S median {summary['road_outer_edges']['target_to_source']['median_m']:.3f} m / "
        f"P95 {summary['road_outer_edges']['target_to_source']['p95_m']:.3f} m")
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    output = Path(output)
    fig.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(fig)
    report = output.with_suffix(".json")
    report.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(output)
    print(report)
    print(json.dumps(summary["road_outer_edges"], ensure_ascii=False))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("shp_dir")
    ap.add_argument("xodr")
    ap.add_argument("output")
    ap.add_argument("--profile", default="ibd-smarteditor-v1")
    ns = ap.parse_args()
    render(ns.shp_dir, ns.xodr, ns.output, ns.profile)
