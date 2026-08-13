# -*- coding: utf-8 -*-
"""MAP(MapNode) → OpenDRIVE：Link 参考线拟合 + 固定几何写出（方向 6，spike-B 流程的模块化）。

每条 Link 一条 road（多段参数几何，G1 由拟合器保证）；可选 G2 连接路（pyclothoids SolveG2）。
写出后自动清除 scenariogeneration 的空 elevationProfile/lateralProfile（1.5M XSD 兼容）。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scenariogeneration import xodr

from mapforge.adapters.v2xmap.xml_reader import MapNode
from mapforge.ops.refline_fit import fit_polyline, PlanView

R_EARTH = 6378137.0


def _project(pts_deg, lat0, lon0):
    pts = np.asarray(pts_deg, dtype=float)
    x = np.radians(pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def _planview_geoms(pv: PlanView):
    geoms = []
    x, y, h = pv.x0, pv.y0, pv.hdg
    for seg in pv.segs:
        if seg.kind == "line" or abs(seg.curvature) < 1e-12:
            geoms.append((xodr.Line(seg.length), x, y, h))
            x += seg.length * math.cos(h)
            y += seg.length * math.sin(h)
        else:
            k = seg.curvature
            geoms.append((xodr.Arc(k, length=seg.length), x, y, h))
            x = x + (math.sin(h + k * seg.length) - math.sin(h)) / k
            y = y - (math.cos(h + k * seg.length) - math.cos(h)) / k
            h += k * seg.length
    return geoms, (x, y, h)


def _road(road_id, pv, n_right, lane_w, road_type=-1):
    plan = xodr.PlanView()
    geoms, _ = _planview_geoms(pv)
    for geom, x, y, h in geoms:
        plan.add_fixed_geometry(geom, x, y, h)
    ls = xodr.LaneSection(0, xodr.Lane(a=0))
    for _ in range(max(1, n_right)):
        ls.add_right_lane(xodr.Lane(lane_type=xodr.LaneType.driving, a=lane_w))
    lanes = xodr.Lanes()
    lanes.add_lanesection(ls)
    return xodr.Road(road_id, plan, lanes, road_type=road_type)


def _strip_empty_profiles(path: Path):
    tree = ET.parse(str(path))
    for road in tree.getroot().findall("road"):
        for tag in ("elevationProfile", "lateralProfile"):
            el = road.find(tag)
            if el is not None and len(el) == 0:
                road.remove(el)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def build_xodr(node: MapNode, out_path: str | Path,
               connect_pairs: list[tuple[str, str]] | None = None) -> dict:
    """MapNode → .xodr。connect_pairs: [(link_name_a, link_name_b)] 生成 G2 连接路（可选）。"""
    lat0, lon0 = node.ref_lat, node.ref_lon
    odr = xodr.OpenDrive(f"node{node.node_id}")
    stats = {"links": 0, "conn_roads": 0, "segs": 0}
    end_pose = {}
    for i, lk in enumerate(node.links):
        if len(lk.points) < 2:
            continue
        pts = _project(lk.points, lat0, lon0)
        pv, _ = fit_polyline(pts, kappa_th=1 / 600, min_seg_len=6.0, smooth_win=3, refine=True)
        _, ep = _planview_geoms(pv)
        end_pose[lk.name] = ep
        lane_ws = [ln.width_cm for ln in lk.lanes if ln.width_cm]
        lane_w = (sum(lane_ws) / len(lane_ws) / 100.0) if lane_ws else 3.5
        odr.add_road(_road(10 + i, pv, n_right=max(1, len(lk.lanes)), lane_w=lane_w))
        stats["links"] += 1
        stats["segs"] += len(pv.segs)
    rid = 50
    for a, b in (connect_pairs or []):
        if a not in end_pose or b not in end_pose:
            continue
        from pyclothoids import SolveG2
        xa, ya, ha = end_pose[a]
        xb, yb, hb = end_pose[b]
        clothoids = SolveG2(xa, ya, ha, 0.0, xb, yb, hb + math.pi, 0.0)
        plan = xodr.PlanView()
        for c in clothoids:
            plan.add_fixed_geometry(xodr.Spiral(c.KappaStart, c.KappaEnd, length=c.length),
                                    c.XStart, c.YStart, c.ThetaStart)
        ls = xodr.LaneSection(0, xodr.Lane(a=0))
        ls.add_right_lane(xodr.Lane(lane_type=xodr.LaneType.driving, a=3.5))
        lanes = xodr.Lanes()
        lanes.add_lanesection(ls)
        odr.add_road(xodr.Road(rid, plan, lanes))
        rid += 1
        stats["conn_roads"] += 1
    out_path = Path(out_path)
    odr.write_xml(str(out_path), prettyprint=True)
    _strip_empty_profiles(out_path)
    return stats
