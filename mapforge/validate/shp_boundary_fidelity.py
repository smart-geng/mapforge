# -*- coding: utf-8 -*-
"""SHP 物理车道边界与 OpenDRIVE 最终道路外缘的同身份对拍。

车道中心线对拍无法证明道路带形状正确：宽度/offset 写错时，中心仍可能贴合，
外缘却会鼓包或收缩。本模块只按 ``mapforge.source_lane`` 身份配对，不允许用
“全图最近边缘”掩盖绑错对象；它是 SHP→OpenDRIVE 的世界边界保真门禁。
"""
from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

from mapforge.validate.smoothness import _sections, lane_edges_at, sample_road_ref

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


def _densify(line, ds=0.5):
    g = np.asarray(line, float)
    if len(g) < 2:
        return g
    seg = np.linalg.norm(np.diff(g, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= ds:
        return g
    q = np.arange(0.0, s[-1], ds)
    if not len(q) or s[-1] - q[-1] > 1e-9:
        q = np.append(q, s[-1])
    return np.column_stack([np.interp(q, s, g[:, 0]),
                            np.interp(q, s, g[:, 1])])


def _target_outer_edges(road, ds=0.5):
    pts, ss, hh = sample_road_ref(road, ds)
    nrm = np.column_stack([-np.sin(hh), np.cos(hh)])
    secs = _sections(road)
    sec_els = road.findall("lanes/laneSection")
    outer = []
    for si, (s0, right, left) in enumerate(secs):
        s1 = secs[si + 1][0] if si + 1 < len(secs) else float(road.get("length"))
        mask = (ss >= s0 - 1e-9) & (ss <= s1 + 1e-9)
        if mask.sum() < 2:
            continue
        for side, lanes in (("right", right), ("left", left)):
            if not lanes:
                continue
            offsets = np.asarray([lane_edges_at(road, float(s), side=side)[-1]
                                  for s in ss[mask]], float)
            line = pts[mask] + offsets[:, None] * nrm[mask]
            path = "right/lane" if side == "right" else "left/lane"
            key = (lambda lane: -int(lane.get("id"))) if side == "right" else (
                lambda lane: int(lane.get("id")))
            lane_els = sorted(sec_els[si].findall(path), key=key)
            if not lane_els:
                continue
            lane = lane_els[-1]
            source = lane.find("userData[@code='mapforge.source_lane']")
            source_id = source.get("value") if source is not None else None
            support_s = None
            boundary_evidence_trusted = None
            eligibility = None
            exclusion_code = None
            support_kind = None
            meta = lane.find("userData[@code='mapforge.provenance/v1']")
            if meta is not None and meta.get("value"):
                try:
                    provenance = json.loads(meta.get("value"))
                    support_s = provenance.get("support_s")
                    boundary_evidence_trusted = provenance.get(
                        "boundary_evidence_trusted")
                    eligibility = provenance.get("eligibility")
                    exclusion_code = provenance.get("exclusion_code")
                    support_kind = provenance.get("support_kind")
                except json.JSONDecodeError:
                    pass
            if support_s and len(support_s) == 2:
                q = ss[mask]
                keep = ((q >= float(support_s[0]) - 1e-6)
                        & (q <= float(support_s[1]) + 1e-6))
                line = line[keep]
            if source_id and len(line) >= 2:
                outer.append({"source_lane_id": source_id, "points": line,
                              "boundary_evidence_trusted":
                                  boundary_evidence_trusted,
                              "eligibility": eligibility,
                              "exclusion_code": exclusion_code,
                              "support_kind": support_kind})
    return outer


def _stats(values):
    a = np.asarray(values, float)
    if not len(a):
        return {"count": 0, "median_m": float("inf"),
                "p95_m": float("inf"), "max_m": float("inf")}
    return {"count": int(len(a)), "median_m": float(np.median(a)),
            "p95_m": float(np.percentile(a, 95)), "max_m": float(np.max(a))}


def _point_polyline_distance(points, line, chunk=1024):
    """精确点到折线段距离，并标记投影是否位于首末端点之外。"""
    p = np.asarray(points, float)
    q = np.asarray(line, float)
    if len(p) == 0 or len(q) < 2:
        return np.full(len(p), float("inf")), np.zeros(len(p), bool)
    a = q[:-1]
    v = q[1:] - q[:-1]
    vv = np.einsum("ij,ij->i", v, v)
    valid = vv > 1e-16
    distances, interior = [], []
    for begin in range(0, len(p), chunk):
        block = p[begin:begin + chunk]
        d = block[:, None, :] - a[None, :, :]
        t = np.zeros((len(block), len(a)), float)
        t[:, valid] = (np.einsum("bsi,si->bs", d[:, valid], v[valid])
                       / vv[valid][None, :])
        tc = np.clip(t, 0.0, 1.0)
        delta = d - tc[:, :, None] * v[None, :, :]
        d2 = np.einsum("bsi,bsi->bs", delta, delta)
        idx = np.argmin(d2, axis=1)
        row = np.arange(len(block))
        best_t = tc[row, idx]
        best_d = np.sqrt(d2[row, idx])
        at_first = (idx == 0) & (best_t <= 1e-9)
        at_last = (idx == len(a) - 1) & (best_t >= 1.0 - 1e-9)
        distances.append(best_d)
        interior.append(~(at_first | at_last))
    return np.concatenate(distances), np.concatenate(interior)


def _clip_source_to_target_support(source, target):
    """把完整 SHP 边界裁到当前导出 road 的共同纵向支持域。

    图商可能让同一边界对象继续绕入下一路口或掉头口，而单路口 XODR leg 在
    当前路口走廊末端结束。仅靠“投影是否落在线段内部”不能识别这种 U 形续段：
    续段仍可能投影到目标直路内部并制造十几米伪误差。目标首末点在源折线上的
    最近位置给出同身份公共子链；完整性仍由 G8 coverage/endpoint 单独负责。
    """
    src = np.asarray(source, float)
    tgt = np.asarray(target, float)
    if len(src) < 3 or len(tgt) < 2:
        return src
    i0 = int(np.argmin(np.linalg.norm(src - tgt[0], axis=1)))
    i1 = int(np.argmin(np.linalg.norm(src - tgt[-1], axis=1)))
    lo, hi = sorted((i0, i1))
    # 留一个 0.5m 采样邻点，使端点距离仍能暴露真实错位，而不吞掉续段。
    lo, hi = max(0, lo - 1), min(len(src) - 1, hi + 1)
    clipped = src[lo:hi + 1]
    target_len = float(np.sum(np.linalg.norm(np.diff(tgt, axis=0), axis=1)))
    clipped_len = float(np.sum(np.linalg.norm(np.diff(clipped, axis=0), axis=1)))
    # 错配或自交导致子链明显不足时不裁，避免门禁被错误缩短。
    return clipped if len(clipped) >= 2 and clipped_len >= 0.75 * target_len else src


def evaluate_shp_outer_edges(shp_dir, xodr, profile="ibd-smarteditor-v1", src=None):
    """返回 SHP 物理外边界与目标 road 外缘的双向同身份距离。"""
    root = ET.parse(str(xodr)).getroot()
    lat0, lon0 = _origin(root)
    if src is None:
        from mapforge.adapters.shp.profile_source import ProfileSource
        src = ProfileSource(str(shp_dir), profile)

    target_by_road = defaultdict(list)
    excluded_source_conflicts = set()
    excluded_unsupported = defaultdict(set)
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        for record in _target_outer_edges(road):
            if record.get("boundary_evidence_trusted") is False:
                excluded_source_conflicts.add(record["source_lane_id"])
                continue
            # 延伸段、车道渐生/渐消带和无源支撑段是 OpenDRIVE 拓扑修复产物，
            # 不能冒充 SHP 实测外边界参与“源边界保真”统计。physical-edge-fill
            # 虽然不是可行驶车道，却直接由物理外边界生成，仍应接受本门禁检验。
            unsupported_code = record.get("exclusion_code") in {
                "source-extension", "source-support-unavailable",
                "source-support-too-short",
            }
            if (unsupported_code
                    and record.get("boundary_evidence_trusted") is not True):
                code = (record.get("exclusion_code")
                        or record.get("support_kind") or "excluded-unsupported")
                excluded_unsupported[str(code)].add(record["source_lane_id"])
                continue
            target_by_road[road.get("id")].append(record)

    s2t, t2s, per_road = [], [], {}
    paired_tracks = 0
    for road_id in sorted(target_by_road, key=int):
        grouped = defaultdict(list)
        for record in target_by_road[road_id]:
            grouped[record["source_lane_id"]].append(record["points"])
        road_s2t, road_t2s = [], []
        for source_id, parts in grouped.items():
            target = np.vstack(parts)
            candidates = [_densify(_project(g, lat0, lon0))
                          for g in src.lane_boundary_geometries(source_id)]
            candidates = [g for g in candidates if len(g) >= 2]
            if not candidates:
                continue
            source = min(candidates, key=lambda g: float(np.median(
                _point_polyline_distance(target, g)[0])))
            source = _clip_source_to_target_support(source, target)
            a, source_inside = _point_polyline_distance(source, target)
            b, target_inside = _point_polyline_distance(target, source)
            # 只比较共同支持域；源/目标尾长差在 G8 endpoint/coverage 中另行负责。
            sm = source_inside | (a <= 1.5)
            tm = target_inside | (b <= 1.5)
            if sm.any() and tm.any():
                paired_tracks += 1
                road_s2t.extend(a[sm].tolist())
                road_t2s.extend(b[tm].tolist())
        if road_s2t and road_t2s:
            s2t.extend(road_s2t)
            t2s.extend(road_t2s)
            per_road[road_id] = {"source_to_target": _stats(road_s2t),
                                 "target_to_source": _stats(road_t2s)}

    return {
        "schema": "mapforge/shp-boundary-fidelity/v1",
        "artifact": str(Path(xodr)),
        "paired_tracks": paired_tracks,
        "paired_roads": len(per_road),
        "excluded_source_conflict_ids": sorted(excluded_source_conflicts),
        "excluded_source_conflicts": len(excluded_source_conflicts),
        "excluded_unsupported_ids": sorted({source_id
                                             for ids in excluded_unsupported.values()
                                             for source_id in ids}),
        "excluded_unsupported": sum(len(ids)
                                    for ids in excluded_unsupported.values()),
        "excluded_unsupported_by_code": {
            code: sorted(ids) for code, ids in sorted(excluded_unsupported.items())
        },
        "source_to_target": _stats(s2t),
        "target_to_source": _stats(t2s),
        "per_road": per_road,
    }
