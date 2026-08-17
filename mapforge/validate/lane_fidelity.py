# -*- coding: utf-8 -*-
"""车道来源保真（G8）：来源 manifest 与 OpenDRIVE 目标车道的双向比较。"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from mapforge.validate.g8_model import (GATE_SCHEMA, json_safe, load_policy,
                                         validate_manifest, validate_policy)
from mapforge.validate.smoothness import (_lane_widths, _offsets, _poly_eval,
                                          _sections, lane_edges_at,
                                          sample_road_ref)


def lane_centers(road, ds: float = 1.0):
    """一条 road 的各车道中心线世界坐标 {lane_id: (n,2)}（兼容旧诊断）。"""
    pts, ss, hh = sample_road_ref(road, ds)
    nrm = np.column_stack([-np.sin(hh), np.cos(hh)])
    secs = _sections(road)
    out = {}
    for si, (s0, right, left) in enumerate(secs):
        s1 = secs[si + 1][0] if si + 1 < len(secs) else ss[-1]
        m = (ss >= s0 - 1e-9) & (ss <= s1 + 1e-9)
        if m.sum() < 2:
            continue
        for side, lanes in (("right", right), ("left", left)):
            edges = np.array([lane_edges_at(road, s, side=side) for s in ss[m]])
            for k, (lid, _w) in enumerate(lanes):
                t = (edges[:, k] + edges[:, k + 1]) / 2
                seg = pts[m] + t[:, None] * nrm[m]
                out.setdefault(lid, []).append(seg)
    return {lid: np.vstack(v) for lid, v in out.items()}


def surface_points(root, ds: float = 1.0, skip_junction: bool = True) -> np.ndarray:
    """整文件所有车道中心线采样点（旧全局最近距诊断）。"""
    acc = []
    for rd in root.findall("road"):
        if skip_junction and rd.get("junction") not in (None, "-1"):
            continue
        if rd.get("name") == "junction_paving":
            continue
        for seg in lane_centers(rd, ds).values():
            acc.append(seg)
    return np.vstack(acc) if acc else np.zeros((0, 2))


def source_lane_centers(root, ds: float = 1.0) -> dict[str, np.ndarray]:
    """兼容旧诊断：按 ``mapforge.source_lane`` 汇总目标中心线。

    正式 G8 不调用本函数，因为同 source id 的不连续片段不能无条件 vstack。
    """
    out: dict[str, list[np.ndarray]] = {}
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        pts, ss, hh = sample_road_ref(road, ds)
        if not len(ss):
            continue
        nrm = np.column_stack([-np.sin(hh), np.cos(hh)])
        sec_els = road.findall("lanes/laneSection")
        for si, sec in enumerate(sec_els):
            s0 = float(sec.get("s"))
            s1 = float(sec_els[si + 1].get("s")) if si + 1 < len(sec_els) else ss[-1]
            m = (ss >= s0 - 1e-9) & (ss <= s1 + 1e-9)
            if m.sum() < 2:
                continue
            for side, path, key in (("right", "right/lane", lambda x: -int(x.get("id"))),
                                    ("left", "left/lane", lambda x: int(x.get("id")))):
                lanes = sorted(sec.findall(path), key=key)
                if not lanes:
                    continue
                edges = np.array([lane_edges_at(road, s, side=side) for s in ss[m]])
                for k, lane in enumerate(lanes):
                    ud = lane.find("userData[@code='mapforge.source_lane']")
                    if ud is None or not ud.get("value"):
                        continue
                    t = (edges[:, k] + edges[:, k + 1]) / 2.0
                    seg = pts[m] + t[:, None] * nrm[m]
                    out.setdefault(ud.get("value"), []).append(seg)
    return {sid: np.vstack(parts) for sid, parts in out.items()}


def _resample(points, ds: float = 1.0) -> np.ndarray:
    g = np.asarray(points, float)
    if g.ndim != 2 or g.shape[0] == 0:
        return np.zeros((0, 2))
    if g.shape[0] == 1:
        return g
    keep = np.concatenate([[True], np.linalg.norm(np.diff(g, axis=0), axis=1) > 1e-6])
    g = g[keep]
    if g.shape[0] < 2:
        return g
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))])
    u = np.arange(0.0, s[-1], ds)
    if not len(u) or s[-1] - u[-1] > 1e-9:
        u = np.append(u, s[-1])
    return np.column_stack([np.interp(u, s, g[:, 0]), np.interp(u, s, g[:, 1])])


def _stats(values) -> dict:
    a = np.asarray(values, float)
    if not a.size:
        return {"max": float("nan"), "p95": float("nan"),
                "median": float("nan"), "n": 0}
    return {"max": float(a.max()), "p95": float(np.percentile(a, 95)),
            "median": float(np.median(a)), "n": int(a.size)}


def _safe_stats(values) -> dict:
    a = np.asarray(values, float)
    if not a.size:
        return {"max_m": None, "p95_m": None, "median_m": None, "count": 0}
    return {"max_m": float(a.max()), "p95_m": float(np.percentile(a, 95)),
            "median_m": float(np.median(a)), "count": int(a.size)}


def paired_deviation(root, src_by_id: dict[str, np.ndarray], ds: float = 1.0) -> dict:
    """旧 API：来源 lane → 对应目标 lane 的双向统计，不作正式 G8 判定。"""
    from scipy.spatial import cKDTree

    targets = source_lane_centers(root, ds)
    s2t_all, t2s_all, per_lane = [], [], {}
    missing = []
    for sid, raw in src_by_id.items():
        if sid not in targets:
            missing.append(sid)
            continue
        src = _resample(raw, ds)
        tgt = _resample(targets[sid], ds)
        if not len(src) or not len(tgt):
            missing.append(sid)
            continue
        s2t = cKDTree(tgt).query(src)[0]
        t2s = cKDTree(src).query(tgt)[0]
        s2t_all.extend(s2t.tolist())
        t2s_all.extend(t2s.tolist())
        per_lane[sid] = {"source_to_target": _stats(s2t),
                         "target_to_source": _stats(t2s)}
    return {"matched": len(per_lane), "missing": sorted(missing),
            "orphan_targets": sorted(set(targets) - set(src_by_id)),
            "source_to_target": _stats(s2t_all),
            "target_to_source": _stats(t2s_all), "per_lane": per_lane}


def _as_root(target):
    if hasattr(target, "getroot"):
        return target.getroot()
    if hasattr(target, "tag") and target.tag == "OpenDRIVE":
        return target
    return ET.parse(str(target)).getroot()


def _lane_provenance(lane, road, side):
    src = lane.find("userData[@code='mapforge.source_lane']")
    source_id = src.get("value") if src is not None else None
    ud = lane.find("userData[@code='mapforge.provenance/v1']")
    meta, error = None, None
    if ud is not None and ud.get("value"):
        try:
            meta = json.loads(ud.get("value"))
        except (TypeError, json.JSONDecodeError):
            error = "invalid_provenance_json"
    if meta is None:
        lane_type = lane.get("type", "driving")
        if lane_type == "median":
            meta = {"eligibility": "excluded", "status": "TRANSFORMED",
                    "role": "median", "support_kind": "median",
                    "travel_direction": "against_s" if side == "left" else "with_s",
                    "exclusion_code": "median-non-driving"}
        elif road.get("name") == "junction_paving" or lane_type == "restricted":
            meta = {"eligibility": "excluded", "status": "INFERRED",
                    "role": "paving", "support_kind": "paving",
                    "travel_direction": "with_s",
                    "exclusion_code": "paving-non-driving"}
        elif source_id:
            meta = {"eligibility": "comparable", "status": "TRANSFORMED",
                    "role": "legacy", "support_kind": "legacy-source-lane",
                    "policy_class": "legacy", "travel_direction":
                    "against_s" if side == "left" else "with_s"}
        else:
            meta = {"eligibility": "unclassified", "status": None,
                    "role": "unknown", "support_kind": "unknown",
                    "travel_direction": "against_s" if side == "left" else "with_s"}
    return source_id, meta, error


def _split_runs(mask):
    idx = np.flatnonzero(mask)
    if not len(idx):
        return []
    cut = np.flatnonzero(np.diff(idx) > 1) + 1
    return [x for x in np.split(idx, cut) if len(x) >= 2]


def _section_samples(s0, s1, ds, include_end):
    if s1 <= s0 + 1e-9:
        return np.zeros(0)
    q = np.arange(s0, s1, ds, dtype=float)
    end = s1 if include_end else np.nextafter(s1, s0)
    if not len(q):
        q = np.array([s0], float)
    if end - q[-1] > min(ds * 0.25, 0.05):
        q = np.append(q, end)
    elif include_end:
        q[-1] = end
    if len(q) == 1:
        q = np.append(q, end)
    return q


def extract_target_components(root, *, ds: float = 1.0,
                              zero_width_epsilon_m: float = 0.05) -> dict:
    """枚举目标 occurrence，并仅合并可证明连续的正宽 component。"""
    root = _as_root(root)
    raw, exclusions, unprovenanced, metadata_errors = [], [], [], []
    occurrence_ids, duplicate_occurrences = set(), []

    for road in root.findall("road"):
        pts0, ss0, hh0 = sample_road_ref(road, min(ds / 4.0, 0.25))
        if not len(ss0):
            continue
        ss0, ui = np.unique(ss0, return_index=True)
        pts0, hh0 = pts0[ui], np.unwrap(hh0[ui])
        sec_els = road.findall("lanes/laneSection")
        offsets = _offsets(road)
        road_len = float(road.get("length", ss0[-1]))
        for si, sec in enumerate(sec_els):
            s0 = float(sec.get("s"))
            s1 = float(sec_els[si + 1].get("s")) if si + 1 < len(sec_els) else road_len
            qs = _section_samples(s0, s1, ds, si == len(sec_els) - 1)
            if len(qs) < 2:
                continue
            xy = np.column_stack([np.interp(qs, ss0, pts0[:, 0]),
                                  np.interp(qs, ss0, pts0[:, 1])])
            hd = np.interp(qs, ss0, hh0)
            nrm = np.column_stack([-np.sin(hd), np.cos(hd)])
            for side, path, sorter, sign in (
                    ("right", "right/lane", lambda x: -int(x.get("id")), -1.0),
                    ("left", "left/lane", lambda x: int(x.get("id")), +1.0)):
                lanes = sorted(sec.findall(path), key=sorter)
                edge = np.array([_poly_eval(offsets, s) for s in qs], float)
                for k, lane in enumerate(lanes):
                    lid = int(lane.get("id"))
                    oid = (road.get("id"), si, side, lid)
                    if oid in occurrence_ids:
                        duplicate_occurrences.append(oid)
                    occurrence_ids.add(oid)
                    wp = _lane_widths(lane)
                    width = np.array([max(0.0, _poly_eval(wp, s - s0)) for s in qs]) \
                        if wp else np.zeros(len(qs))
                    next_edge = edge + sign * width
                    center = (edge + next_edge) / 2.0
                    source_id, meta, error = _lane_provenance(lane, road, side)
                    if error:
                        metadata_errors.append({"target": oid, "error": error})
                    valid = width > zero_width_epsilon_m
                    support_s = meta.get("support_s")
                    lane_qs, lane_xy, lane_nrm = qs, xy, nrm
                    lane_width, lane_center = width, center
                    if (isinstance(support_s, list) and len(support_s) == 2
                            and meta.get("eligibility") == "comparable"):
                        bounds = [float(support_s[0]), float(support_s[1])]
                        upper = s1 if si == len(sec_els) - 1 else s1 - 1e-6
                        extra = [min(max(value, s0), upper) for value in bounds
                                 if s0 - 1e-9 <= value <= s1 + 1e-9]
                        if extra:
                            base_qs = qs if si == len(sec_els) - 1 else np.minimum(qs, upper)
                            lane_qs = np.unique(np.concatenate([base_qs, np.asarray(extra, float)]))
                            lane_xy = np.column_stack([
                                np.interp(lane_qs, ss0, pts0[:, 0]),
                                np.interp(lane_qs, ss0, pts0[:, 1]),
                            ])
                            lane_hd = np.interp(lane_qs, ss0, hh0)
                            lane_nrm = np.column_stack([-np.sin(lane_hd), np.cos(lane_hd)])
                            lane_edges = np.asarray([
                                lane_edges_at(road, value, side=side) for value in lane_qs])
                            lane_width = np.abs(lane_edges[:, k + 1] - lane_edges[:, k])
                            lane_center = (lane_edges[:, k] + lane_edges[:, k + 1]) / 2.0
                    valid = lane_width > zero_width_epsilon_m
                    if (isinstance(support_s, list) and len(support_s) == 2
                            and meta.get("eligibility") == "comparable"):
                        inside = ((lane_qs >= float(support_s[0]) - 1e-6)
                                  & (lane_qs <= float(support_s[1]) + 1e-6))
                        if np.any(valid & ~inside):
                            exclusions.append({"target": oid,
                                               "code": meta.get("support_exclusion_code",
                                                                "source-support-outside"),
                                               "status": "APPROXIMATED",
                                               "source_lane_id": source_id,
                                               "s_range": [float(support_s[0]), float(support_s[1])]})
                        valid &= inside
                    runs = _split_runs(valid)
                    if not runs:
                        code = "zero-width" if not np.any(
                            lane_width > zero_width_epsilon_m) else "source-support-unavailable"
                        exclusions.append({"target": oid, "code": code,
                                           "status": meta.get("status"), "source_lane_id": source_id})
                        edge = next_edge
                        continue
                    if meta.get("eligibility") == "excluded":
                        exclusions.append({"target": oid,
                                           "code": meta.get("exclusion_code", "unspecified"),
                                           "status": meta.get("status"),
                                           "role": meta.get("role"),
                                           "support_kind": meta.get("support_kind"),
                                           "source_lane_id": source_id})
                        edge = next_edge
                        continue
                    if not source_id:
                        unprovenanced.append(oid)
                        edge = next_edge
                        continue
                    for run_i, idx in enumerate(runs):
                        seg = lane_xy[idx] + lane_center[idx, None] * lane_nrm[idx]
                        direction = meta.get("travel_direction") or (
                            "against_s" if side == "left" else "with_s")
                        if direction == "against_s":
                            seg = seg[::-1]
                        raw.append({
                            "source_lane_id": source_id,
                            "road_id": str(road.get("id")),
                            "section_index": si,
                            "section_first": si,
                            "section_last": si,
                            "side": side,
                            "lane_id": lid,
                            "run_index": run_i,
                            "s_min": float(lane_qs[idx[0]]),
                            "s_max": float(lane_qs[idx[-1]]),
                            "points": seg,
                            "role": meta.get("role"),
                            "status": meta.get("status"),
                            "support_kind": meta.get("support_kind"),
                            "policy_class": meta.get("policy_class"),
                            "travel_direction": direction,
                        })
                    edge = next_edge

    grouped: dict[tuple, list[dict]] = {}
    for comp in raw:
        key = (comp["source_lane_id"], comp["road_id"], comp["side"], comp["lane_id"],
               comp["role"], comp["status"], comp["support_kind"], comp["policy_class"],
               comp["travel_direction"])
        grouped.setdefault(key, []).append(comp)
    merged = []
    for parts in grouped.values():
        reverse = parts[0]["travel_direction"] == "against_s"
        parts.sort(key=lambda x: (x["section_index"], x["run_index"]), reverse=reverse)
        cur = None
        for part in parts:
            if cur is None:
                cur = dict(part)
                cur["_tail_section"] = part["section_index"]
                continue
            adjacent = abs(part["section_index"] - cur["_tail_section"]) == 1
            gap = float(np.linalg.norm(cur["points"][-1] - part["points"][0]))
            if adjacent and gap <= max(ds * 2.5, 0.25):
                cur["points"] = np.vstack([cur["points"], part["points"]])
                cur["section_first"] = min(cur["section_first"], part["section_first"])
                cur["section_last"] = max(cur["section_last"], part["section_last"])
                cur["s_min"] = min(cur["s_min"], part["s_min"])
                cur["s_max"] = max(cur["s_max"], part["s_max"])
                cur["_tail_section"] = part["section_index"]
            else:
                merged.append(cur)
                cur = dict(part)
                cur["_tail_section"] = part["section_index"]
        if cur is not None:
            merged.append(cur)

    return {
        "components": merged,
        "exclusions": exclusions,
        "unprovenanced": [list(x) for x in sorted(set(unprovenanced))],
        "duplicate_occurrences": [list(x) for x in duplicate_occurrences],
        "metadata_errors": metadata_errors,
    }


def _line_length(points) -> float:
    g = np.asarray(points, float)
    return float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()) if len(g) >= 2 else 0.0


def _point_to_polyline(point, line) -> float | None:
    p = np.asarray(point, float)
    g = np.asarray(line, float)
    if g.ndim != 2 or not len(g):
        return None
    if len(g) == 1:
        return float(np.linalg.norm(p - g[0]))
    a, b = g[:-1], g[1:]
    v = b - a
    den = (v * v).sum(axis=1)
    t = np.divide(((p - a) * v).sum(axis=1), den,
                  out=np.zeros_like(den), where=den > 1e-12)
    t = np.clip(t, 0.0, 1.0)
    q = a + t[:, None] * v
    return float(np.linalg.norm(q - p, axis=1).min())


def _metric_limit_fail(prefix, stats, limits, reasons, sid):
    names = (("median_m", "median_max_m"), ("p95_m", "p95_max_m"),
             ("max_m", "max_max_m"))
    for metric, limit in names:
        lim = limits.get(limit)
        val = stats.get(metric)
        if lim is not None and val is not None and val > float(lim) + 1e-9:
            reasons.append(f"{sid}:{prefix}.{metric}>{lim}")


def evaluate_g8(target_xodr, source_lane_manifest: dict | None,
                policy_ref: str | Path | dict | None) -> dict:
    """执行正式 G8，返回严格 JSON ``mapforge/gate-result/v1``。"""
    from scipy.spatial import cKDTree

    policy = load_policy(policy_ref)
    result = {
        "schema": GATE_SCHEMA,
        "gate_id": "G8",
        "name": "lane-correspondence-fidelity",
        "status": "UNAVAILABLE",
        "policy": None if policy is None else {
            "id": policy.get("policy_id"), "version": policy.get("version"),
            "lifecycle": policy.get("lifecycle"), "sha256": policy.get("policy_sha256")},
        "scope": {}, "metrics": {}, "per_lane": [], "issues": {},
        "exclusions": [], "failure_reasons": [],
    }
    manifest_errors = validate_manifest(source_lane_manifest)
    policy_errors = validate_policy(policy)
    fatal_policy = [x for x in policy_errors if x != "policy_not_active"]
    if manifest_errors or fatal_policy:
        result["failure_reasons"] = manifest_errors + fatal_policy
        return json_safe(result)
    app = policy.get("applicability") or {}
    if app.get("target_format") != "opendrive":
        result["status"] = "NOT_RUN"
        result["failure_reasons"] = ["target_format_not_applicable"]
        return json_safe(result)
    src_format = source_lane_manifest.get("source_format")
    if app.get("source_formats") and src_format not in app["source_formats"]:
        result["status"] = "NOT_RUN"
        result["failure_reasons"] = ["source_format_not_applicable"]
        return json_safe(result)

    try:
        root = _as_root(target_xodr)
        extracted = extract_target_components(
            root, ds=float((policy.get("sampling") or {}).get("step_m", 1.0)),
            zero_width_epsilon_m=float((policy.get("sampling") or {})
                                        .get("zero_width_epsilon_m", 0.05)))
    except Exception as exc:
        result["failure_reasons"] = [f"target_extract_failed:{type(exc).__name__}:{exc}"]
        return json_safe(result)

    lanes = source_lane_manifest.get("lanes", [])
    sources = {x["source_lane_id"]: x for x in lanes
               if x.get("comparison", {}).get("eligible")}
    components_by_source: dict[str, list[dict]] = {}
    for comp in extracted["components"]:
        components_by_source.setdefault(comp["source_lane_id"], []).append(comp)
    missing = sorted(set(sources) - set(components_by_source))
    orphan = sorted(set(components_by_source) - set(sources))
    one_many = sorted(sid for sid, comps in components_by_source.items() if len(comps) > 1)
    issues = {
        "missing_source_ids": missing,
        "orphan_target_ids": orphan,
        "one_source_many_targets": one_many,
        "unprovenanced_targets": extracted["unprovenanced"],
        "duplicate_target_occurrences": extracted["duplicate_occurrences"],
        "metadata_errors": extracted["metadata_errors"],
        "unmeasurable_source_ids": [],
        "unknown_policy_classes": [],
    }
    result["issues"] = issues
    result["exclusions"] = extracted["exclusions"]

    sampling = policy.get("sampling") or {}
    ds = float(sampling.get("step_m", 1.0))
    outlier_limit = int(sampling.get("outlier_limit_per_direction", 20))
    classes = policy.get("classes") or {}
    s2t_all, t2s_all, start_all, end_all = [], [], [], []
    per_lane, threshold_failures = [], []
    unavailable = []

    for sid in sorted(set(sources) & set(components_by_source)):
        if len(components_by_source[sid]) != 1:
            continue
        src_rec = sources[sid]
        comp = components_by_source[sid][0]
        class_id = src_rec.get("policy_class") or comp.get("policy_class")
        cfg = classes.get(class_id)
        if cfg is None:
            issues["unknown_policy_classes"].append({"source_lane_id": sid,
                                                       "policy_class": class_id})
            unavailable.append(sid)
            continue
        src = _resample((src_rec.get("geometry") or {}).get("coordinates", []), ds)
        tgt = _resample(comp["points"], ds)
        if len(src) < 2 or len(tgt) < 2:
            issues["unmeasurable_source_ids"].append(sid)
            unavailable.append(sid)
            continue
        kt, ks = cKDTree(tgt), cKDTree(src)
        s2t, sidx = kt.query(src)
        t2s, tidx = ks.query(tgt)
        s2t_stat, t2s_stat = _safe_stats(s2t), _safe_stats(t2s)
        radius = float(cfg.get("coverage_radius_m", 1.5))
        src_cov = float(np.mean(s2t <= radius))
        tgt_cov = float(np.mean(t2s <= radius))
        src_len, tgt_len = _line_length(src), _line_length(tgt)
        ratio = tgt_len / src_len if src_len > 1e-9 else None
        start_err = float(np.linalg.norm(src[0] - tgt[0]))
        end_err = float(np.linalg.norm(src[-1] - tgt[-1]))
        stop = src_rec.get("stop_line") or {}
        stop_metric = {"applicable": False, "source_distance_m": None,
                       "target_distance_m": None, "delta_m": None}
        if stop.get("availability") in ("available", "anchor"):
            geom = (stop.get("geometry") or {}).get("coordinates") or stop.get("coordinates") or []
            sd, td = _point_to_polyline(src[-1], geom), _point_to_polyline(tgt[-1], geom)
            stop_metric = {"applicable": True, "source_distance_m": sd,
                           "target_distance_m": td,
                           "delta_m": abs(td - sd) if sd is not None and td is not None else None}
        outliers = []
        for direction, dist, points, near_points, near_idx in (
                ("source_to_target", s2t, src, tgt, sidx),
                ("target_to_source", t2s, tgt, src, tidx)):
            worst = np.argsort(dist)[::-1][:outlier_limit]
            for i in worst:
                if dist[i] <= radius:
                    continue
                outliers.append({"direction": direction, "distance_m": float(dist[i]),
                                 "point": points[i].tolist(),
                                 "nearest": near_points[int(near_idx[i])].tolist()})
        rec = {
            "source_lane_id": sid, "policy_class": class_id,
            "target": {k: comp[k] for k in ("road_id", "side", "lane_id",
                                               "section_first", "section_last")},
            "source_to_target": s2t_stat,
            "target_to_source": t2s_stat,
            "endpoint": {"travel_start_m": start_err, "travel_end_m": end_err},
            "stopline": stop_metric,
            "coverage": {"radius_m": radius, "source": src_cov, "target": tgt_cov,
                         "source_length_m": src_len, "target_length_m": tgt_len,
                         "target_source_length_ratio": ratio},
            "outliers": outliers,
        }
        _metric_limit_fail("source_to_target", s2t_stat,
                           cfg.get("source_to_target", {}), threshold_failures, sid)
        _metric_limit_fail("target_to_source", t2s_stat,
                           cfg.get("target_to_source", {}), threshold_failures, sid)
        if start_err > float(cfg.get("endpoint_max_m", float("inf"))) + 1e-9:
            threshold_failures.append(f"{sid}:endpoint.start>{cfg['endpoint_max_m']}")
        if end_err > float(cfg.get("endpoint_max_m", float("inf"))) + 1e-9:
            threshold_failures.append(f"{sid}:endpoint.end>{cfg['endpoint_max_m']}")
        if src_cov + 1e-9 < float(cfg.get("source_coverage_min", 0.0)):
            threshold_failures.append(f"{sid}:coverage.source<{cfg['source_coverage_min']}")
        if tgt_cov + 1e-9 < float(cfg.get("target_coverage_min", 0.0)):
            threshold_failures.append(f"{sid}:coverage.target<{cfg['target_coverage_min']}")
        lr = cfg.get("length_ratio") or {}
        if ratio is not None and ratio < float(lr.get("min", -float("inf"))) - 1e-9:
            threshold_failures.append(f"{sid}:length_ratio<{lr['min']}")
        if ratio is not None and ratio > float(lr.get("max", float("inf"))) + 1e-9:
            threshold_failures.append(f"{sid}:length_ratio>{lr['max']}")
        if stop_metric["applicable"]:
            dmax = cfg.get("stopline_delta_max_m")
            if dmax is not None and stop_metric["delta_m"] > float(dmax) + 1e-9:
                threshold_failures.append(f"{sid}:stopline.delta>{dmax}")
        elif cfg.get("require_stopline", False):
            unavailable.append(sid)
            issues["unmeasurable_source_ids"].append(f"{sid}:stopline")
        per_lane.append(rec)
        s2t_all.extend(s2t.tolist())
        t2s_all.extend(t2s.tolist())
        start_all.append(start_err)
        end_all.append(end_err)

    structural = []
    for key in ("missing_source_ids", "orphan_target_ids", "one_source_many_targets",
                "unprovenanced_targets", "duplicate_target_occurrences", "metadata_errors"):
        if issues[key]:
            structural.append(key)
    allowed = set(policy.get("allowed_exclusions") or [])
    unknown_exclusions = sorted({x.get("code") for x in result["exclusions"]
                                 if x.get("code") not in allowed})
    if unknown_exclusions:
        structural.append("unknown_exclusions")
        issues["unknown_exclusions"] = unknown_exclusions
    result["scope"] = {
        "source_format": src_format,
        "eligible_source_lanes": len(sources),
        "matched_source_lanes": len(per_lane),
        "target_components": len(extracted["components"]),
        "excluded_occurrences": len(result["exclusions"]),
    }
    lane_medians = [x["source_to_target"]["median_m"] for x in per_lane]
    result["metrics"] = {
        "source_to_target": _safe_stats(s2t_all),
        "target_to_source": _safe_stats(t2s_all),
        "endpoint_start": _safe_stats(start_all),
        "endpoint_end": _safe_stats(end_all),
        "lane_weighted_source_median": _safe_stats(lane_medians),
    }
    result["per_lane"] = per_lane
    reasons = structural + threshold_failures
    if not sources:
        unavailable.append("no_comparable_source_lanes")
    if policy_errors or unavailable:
        result["status"] = "UNAVAILABLE"
        reasons.extend(policy_errors + [f"unavailable:{x}" for x in unavailable])
    elif reasons:
        result["status"] = "FAIL"
    else:
        result["status"] = "PASS"
    result["failure_reasons"] = sorted(set(reasons))
    return json_safe(result)


def deviation(root, src_lane_pts, ds: float = 1.0):
    """旧 API：源车道点列到全路面的最近距统计。"""
    tgt = surface_points(root, ds)
    if tgt.size == 0:
        return {"max": float("nan"), "p95": float("nan"), "median": float("nan"), "n": 0}
    d = []
    for g in src_lane_pts:
        g = np.asarray(g, float)
        if g.shape[0] < 1:
            continue
        for p in g:
            d.append(float(np.min(np.linalg.norm(tgt - p[None, :], axis=1))))
    return _stats(d)
