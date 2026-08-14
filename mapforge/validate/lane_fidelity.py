# -*- coding: utf-8 -*-
"""车道保真度：写出的车道中心线 vs 源车道点列的横向偏差。

为什么需要它：参考线**不是交付物**——车道位置是按实测轮廓相对参考线的横距写成
laneOffset/width 的，所以把参考线做平滑并不必然移动车道（差量由横距吸收）。
但"不必然"要有证据，否则平滑参考线就是在悄悄挪路。本模块就是那份证据：
直接比对文件里算出来的车道中心与源点列，超差即判定平滑过度。
"""
from __future__ import annotations

import math

import numpy as np

from mapforge.validate.smoothness import _sections, lane_edges_at, sample_road_ref


def lane_centers(road, ds: float = 1.0):
    """一条 road 的各车道中心线世界坐标 {lane_id: (n,2)}（含左右侧）。"""
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
    """整文件所有车道中心线采样点（默认只取非 junction road——源点列也是路段级）。"""
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
    """按 writer 落盘的 ``mapforge.source_lane`` provenance 还原目标车道中心。

    同一个 OpenDRIVE lane id 可在 laneSection 边界换绑来源车道，故不能只按
    ``(road,id)`` 合并；这里逐 section 读取 userData，再按来源键汇总。
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


def paired_deviation(root, src_by_id: dict[str, np.ndarray], ds: float = 1.0) -> dict:
    """来源 lane → 对应目标 lane 的双向保真统计。

    与旧 ``deviation`` 的“到全路面最近距离”不同，本函数只允许相同 provenance
    键互相比较，因此相邻车道、重复车道或 lane 绑错不会掩盖偏差；同时计算
    source→target 和 target→source，分别捕获漏画与多画。
    """
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


def deviation(root, src_lane_pts, ds: float = 1.0):
    """源车道点列 → 写出车道中心的最近距统计（m）。

    src_lane_pts: [(n,2)] 世界坐标（与 xodr 同一投影）。
    返回 {max, p95, median, n}——max 是"最坏的一个源点离写出路面有多远"。"""
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
    if not d:
        return {"max": float("nan"), "p95": float("nan"), "median": float("nan"), "n": 0}
    a = np.array(d)
    return {"max": float(a.max()), "p95": float(np.percentile(a, 95)),
            "median": float(np.median(a)), "n": int(a.size)}
