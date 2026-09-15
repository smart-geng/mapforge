# -*- coding: utf-8 -*-
"""SHP(IBD) → OpenDRIVE 直转（方向 3；不过 MAP 窄门，junction 一等公民）。

道路模型 = OpenDRIVE 规范做法：**一条街一条 road，参考线沿进口幅中心，双侧展开**——
进口车道挂右侧（-1..-n），对向出口车道按实测横距挂左侧（+1..+n），
两幅间实测间隙写 median 车道（宽可为 0），中线双黄。进/出口按端点位姿+方向自动配对；
配不上的单向链退化为单侧 road（物理分隔/数据缺失时的合法形态）。

写出走自研规范级 writer（adapters/opendrive/writer.py，弃 scenariogeneration）：
车道 <speed>/<roadMark> 原生落盘、geoReference 记录 PROJ 管线、无后处理补丁。

断面：**多 laneSection**——进口链每源 Link 一段、对向链边界投影到参考线后与进口边界
合并（1.5m 内吸附）；每车道 S_WIDTH→E_WIDTH smoothstep 变宽、laneOffset C1 单调样条、
生/灭车道 20m 锥形收放、断面重划分处起宽衔接上段末宽（Σ宽连续）。
路口：<junction> + 连接路（路口内车道实测几何 + 两端 G2 桥）+ Connection/laneLink；
出口连接路接同一条 leg road 的 END 接触点（左侧车道，行车方向沿 s 递减）。

已知简化（正式化清单）：高程平面输出（SLOPE/BANKING 字段未核验，资料盘点）；
标线为通行画法默认（MARK_TYPE 权威枚举待消费）；lane_type 除 median 外全 driving。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from mapforge.adapters.opendrive import writer as W
from mapforge.adapters.opendrive.writer import std_mark
from mapforge.adapters.shp.ibd_reader import IbdSource, JunctionRec
from mapforge.ops.refline_fit import (ReflineFitError, eval_planview, fc_clamp,
                                      fit_connector_minimal, fit_leg_refline,
                                      fit_polyline_auto,
                                      g2ify_planview, planview_prims, seg_kappa,
                                      simplify_planview, weld_g2)
from mapforge.validate.g8_model import geometry_sha256, make_manifest, source_lane

R_EARTH = 6378137.0
_DEG_EPS = 1e-5          # 端点衔接容差（度，≈1m）
_TAPER = 20.0            # 生/灭车道锥形收放长度（m，APPROXIMATED：数据以整 Link 粒度生灭）
_SNAP = 1.5              # 对向链边界向进口边界吸附距离（m）
_SOURCE_TRANSITION_TOL = 0.5  # 连续化后仍可归属来源中心线的最大横向位移（m）
_SOURCE_TRANSITION_MARGIN = 0.5  # 支持域裁剪的纵向安全余量（m）
_BOUNDARY_SECTION_STEP = 10.0  # 有真实 LANE_BOUNDARY 时的横断面采样间隔（m）


class CandidateSurfaceError(RuntimeError):
    """保留失败候选的诊断供复查；调用方不得把部分 XODR 当成交付成功。"""
    def __init__(self, stats):
        self.stats = stats
        super().__init__(stats.get("source_surface_error", "candidate source surface unresolved"))


def _source_endpoint_widths(lane):
    """Millimetres, preserving explicit zero independently of missing fields."""
    return tuple(getattr(lane, name) if getattr(lane, known, False) else
                 (getattr(lane, name) or lane.width_mm or 3500)
                 for name, known in (("s_width_mm", "s_width_known"),
                                     ("e_width_mm", "e_width_known")))


def _match_side_sections(specs, src, stats):
    """Choose physical continuations within declared TOPO, not nearest forks.

    A zero-width branch can have the nearest *path* center to its parent.
    Prefer the nonzero, width-continuous successor/predecessor as the section
    backbone. Unselected declared forks stay births/deaths; do not invent a
    different link through the geometric fallback. No source TOPO is changed.
    """
    topo_out = getattr(src, "topo_out", {}) or {}
    topo_in = getattr(src, "topo_in", {}) or {}
    result = []
    for sa, sb in zip(specs, specs[1:]):
        if sa is None or sb is None:
            result.append({}); continue
        if sa.get("span_key", (id(sa["span"]),)) == sb.get("span_key", (id(sb["span"]),)):
            result.append({i:i for i in range(len(sa["lanes"]))}); continue
        pairs, declared_a, declared_b = [], set(), set()
        for i,x in enumerate(sa["lanes"]):
            xid = x["l"].lane_pid
            linked = set(topo_out.get(xid, ())) | set(topo_in.get(xid, ()))
            for j,y in enumerate(sb["lanes"]):
                yid = y["l"].lane_pid
                reverse = set(topo_out.get(yid, ())) | set(topo_in.get(yid, ()))
                if yid in linked or xid in reverse:
                    # Widths already follow increasing road s (including
                    # reversed outgoing geometry) and precede reconciliation.
                    wa, wb = max(0.,x["w1"]), max(0.,y["w0"])
                    score = (int(min(wa,wb) < .05), abs(wa-wb), abs(x["v1"]-y["v0"]))
                    pairs.append((score,i,j)); declared_a.add(i); declared_b.add(j)
        ua, ub, mapping = set(), set(), {}
        for score,i,j in sorted(pairs):
            if i in ua or j in ub: continue
            ua.add(i); ub.add(j); mapping[i] = j
            stats["topology_lane_matches"] = stats.get("topology_lane_matches",0)+1
            alternatives = [(q,ii,jj) for q,ii,jj in pairs if ii==i or jj==j]
            if len(alternatives)>1:
                stats.setdefault("topology_continuation_choices",[]).append({
                    "from":sa["lanes"][i]["l"].lane_pid, "to":sb["lanes"][j]["l"].lane_pid,
                    "reason":"nonzero-width-continuation-before-center-distance", "score":list(score),
                    "alternatives":[[sa["lanes"][ii]["l"].lane_pid,sb["lanes"][jj]["l"].lane_pid] for _,ii,jj in alternatives]})
        fallback = sorted((abs(x["v1"]-y["v0"]),i,j)
                          for i,x in enumerate(sa["lanes"]) for j,y in enumerate(sb["lanes"]))
        for gap,i,j in fallback:
            if gap>=2. or i in ua or j in ub or i in declared_a or j in declared_b: continue
            ua.add(i); ub.add(j); mapping[i]=j
            stats["inferred_section_lane_matches"] = stats.get("inferred_section_lane_matches",0)+1
        result.append(mapping)
    return result


def _smooth_w(w0, w1, L):
    """smoothstep 三次系数（两端零斜率）：宽度过渡 C1，边缘无折角。"""
    L = max(L, 1e-3)
    return (w0, 0.0, 3 * (w1 - w0) / L ** 2, -2 * (w1 - w0) / L ** 3)


def _c2_station_slopes(stations, values, *, zero_start=False, zero_end=False):
    """给一串横断面站值求全局 C2 三次样条的一阶导数。

    OpenDRIVE 的 laneOffset/width 单条记录只有三次项，但多记录共享样条结点
    导数后仍可表达完整 C2 曲线。生/灭车道在零宽端使用零斜率，普通端使用
    natural（二阶导数为零）边界条件。
    """
    from scipy.interpolate import CubicSpline

    x = np.asarray(stations, float)
    y = np.asarray(values, float)
    if len(x) <= 1:
        return np.zeros_like(y)
    if len(x) == 2 and not zero_start and not zero_end:
        slope = (y[1] - y[0]) / max(x[1] - x[0], 1e-9)
        return np.asarray([slope, slope])
    bc0 = (1, 0.0) if zero_start else (2, 0.0)
    bc1 = (1, 0.0) if zero_end else (2, 0.0)
    spline = CubicSpline(x, y, bc_type=(bc0, bc1))
    return np.asarray(spline(x, 1), float)


def _regularize_micro_stations(stations, values, min_span=5.0):
    """移除仅由采样/Link 边界形成的微小几何控制站。

    laneSection 仍原样保留拓扑语义；这里只令落在宽邻域平滑曲线上的站值不再
    迫使 laneOffset/median 在 1--3m 内完成横移。首末站和没有双侧邻站的真实
    边界不动。返回新数组和改写站数。
    """
    x = np.asarray(stations, float)
    y = np.asarray(values, float).copy()
    changed = set()
    if len(x) < 4:
        return y, 0
    # 相邻微区间可能成串，迭代两次即可把局部值投回两侧宽跨度弦线。
    for _ in range(2):
        for q in np.flatnonzero(np.diff(x) < float(min_span)):
            lo, hi = q - 1, q + 2
            if lo < 0 or hi >= len(x):
                continue
            span = float(x[hi] - x[lo])
            if span <= 1e-9:
                continue
            for j in (q, q + 1):
                u = float((x[j] - x[lo]) / span)
                y[j] = (1.0 - u) * y[lo] + u * y[hi]
                changed.add(j)
    return y, len(changed)


def _width_pieces(sw, ew, L, born, dying, sw_join=None):
    """车道宽度分段多项式 [(sOffset, a, b, c, d)]。
    生车道从 0 张开、灭车道收拢到 0、sw_join 覆盖起宽与上段末宽衔接——
    数据按 Link 粒度瞬现/瞬失/横断面重划分，直写会在边缘留矩形缺口（查看器显形）；
    锥形/衔接段 ≤_TAPER 后回归数据值，为记录在案的修复。所有过渡 smoothstep（两端零斜率）。"""
    if born and dying:                                   # 仅存在于本 section 的短车道
        T = L / 2
        wm = max(sw, ew)
        return [(0.0, *_smooth_w(0.0, wm, T)), (T, *_smooth_w(wm, 0.0, L - T))]
    w0 = 0.0 if born else sw
    if not born and sw_join is not None and abs(sw_join - sw) > 0.01:
        w0 = sw_join
    if dying:
        T = min(_TAPER, L)
        if T < L - 1e-6:
            w_t = sw + (ew - sw) * (L - T) / L
            return [(0.0, *_smooth_w(w0, w_t, L - T)), (L - T, *_smooth_w(w_t, 0.0, T))]
        return [(0.0, *_smooth_w(w0, 0.0, L))]
    T = min(_TAPER, L)
    if born:                                             # 生车道锥形：必须张满到 ew——
        w_t = sw + (ew - sw) * T / L                     # 坡度上限只属于衔接分支
        out = [(0.0, *_smooth_w(0.0, w_t, T))]           # （误入会钳住张开量，下段跳台阶）
        if T < L - 1e-6:
            out.append((T, *_smooth_w(w_t, ew, L - T)))
        return out
    if abs(w0 - sw) > 1e-9:                              # 起宽被覆盖（衔接上段末宽）
        w_end = ew
        max_dw = 0.10 * L                                # 坡度上限：车道中心横摆可驾驶
        if abs(ew - w0) > max_dw:                        # 短 section 消化不完 → 残量传下段
            w_end = w0 + math.copysign(max_dw, ew - w0)
        if abs(w_end - ew) < 1e-9 and T < L - 1e-6:
            w_t = sw + (ew - sw) * T / L
            return [(0.0, *_smooth_w(w0, w_t, T)), (T, *_smooth_w(w_t, ew, L - T))]
        return [(0.0, *_smooth_w(w0, w_end, L))]
    return [(0.0, *_smooth_w(w0, ew, L))]


_fc = fc_clamp                                           # Hermite 斜率单调限幅（共享）


def _pieces_end(pieces, L):
    """分段宽度多项式在 section 末端的实际值（written 末宽——衔接链的真值）。"""
    so, a, b, c, d = pieces[-1]
    t = max(L - so, 0.0)
    return a + b * t + c * t ** 2 + d * t ** 3


def _lane_signature(lane):
    """laneSection 合并判据中的车道拓扑签名。

    几何控制点可以产生新的 width 记录，但不能因此产生新的 laneSection。
    support_s 是同一来源车道在各几何小段上的局部支持域，合并时取并集，故不参与
    拓扑签名；其余 provenance 变化仍会阻止合并，避免掩盖生灭/推导状态变化。
    """
    provenance = dict(lane.provenance or {})
    provenance.pop("support_s", None)
    return (lane.lane_id, lane.lane_type, lane.source_id, lane.mark, lane.speed_ms,
            tuple(sorted((key, repr(value)) for key, value in provenance.items())))


def _merge_lane_geometry(dst, src, shift):
    dst.widths.extend((float(s) + shift, a, b, c, d)
                      for s, a, b, c, d in src.widths)
    dst.borders.extend((float(s) + shift, a, b, c, d)
                       for s, a, b, c, d in src.borders)
    dst.succ = src.succ
    if dst.provenance is not None and src.provenance is not None:
        a = dst.provenance.get("support_s")
        b = src.provenance.get("support_s")
        if a and b:
            dst.provenance["support_s"] = [min(float(a[0]), float(b[0])),
                                             max(float(a[1]), float(b[1]))]


def _compact_lane_sections(road):
    """把仅由几何采样产生的相邻 laneSection 合并。

    OpenDRIVE 的 laneSection 是拓扑/功能断面，不是横断面采样容器。合并后，原来
    每个小段的 width 多项式以新的 sOffset 保留，因此道路几何不变；laneOffset
    仍是 road 级多记录。返回 (合并前数量, 合并后数量)。
    """
    if not road.sections:
        return 0, 0
    before = len(road.sections)
    compacted = [road.sections[0]]
    for sec in road.sections[1:]:
        prev = compacted[-1]
        same = (prev.center_mark == sec.center_mark
                and tuple(_lane_signature(x) for x in prev.left)
                == tuple(_lane_signature(x) for x in sec.left)
                and tuple(_lane_signature(x) for x in prev.right)
                == tuple(_lane_signature(x) for x in sec.right))
        if not same:
            compacted.append(sec)
            continue
        shift = float(sec.s) - float(prev.s)
        for a, b in zip(prev.left, sec.left):
            _merge_lane_geometry(a, b, shift)
        for a, b in zip(prev.right, sec.right):
            _merge_lane_geometry(a, b, shift)
    road.sections = compacted
    return before, len(compacted)


def _smooth_lane_width_tracks(sections, specs, matches, objects, start_widths,
                              end_widths, desired_totals, road_length, stats, side_name,
                              base_offsets, side_sign, allow_total_relax=True):
    """沿真实车道连续关系整体优化 width，而不是逐小段各自收放。

    旧实现只在“车道消失前最后一个几何小段”收至零；若最后一段只有 1--3m，
    就会出现 odrviewer 截图中的尖脖子。这里先按 laneLink 一对一映射构造完整
    车道轨迹，再用平滑样条抑制宽度噪声；birth/death 的零宽约束在整条轨迹上
    传播，最后以 PCHIP/Hermite 写回每个几何小段。拓扑不变，只改变横断面形态。
    """
    from scipy.interpolate import CubicSpline, PchipInterpolator, UnivariateSpline

    if not any(objects):
        return

    original_start_widths = [list(map(float, values)) for values in start_widths]
    original_end_widths = [list(map(float, values)) for values in end_widths]

    nodes = [(si, k) for si, lanes in enumerate(objects) for k in range(len(lanes))]
    parent = {node: node for node in nodes}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for si, mapping in enumerate(matches):
        for a, b in mapping.items():
            if (si, a) in parent and (si + 1, b) in parent:
                union((si, a), (si + 1, b))

    groups = {}
    for node in nodes:
        groups.setdefault(find(node), []).append(node)

    adjusted_max = 0.0
    transition_tracks = 0
    smoothed_tracks = 0
    tracks = []
    for group in groups.values():
        group.sort()
        # laneLink 图是一对一链；出现同 section 多节点说明上游匹配异常，诚实跳过。
        if len({si for si, _ in group}) != len(group):
            continue
        first_si, first_k = group[0]
        last_si, last_k = group[-1]
        xs = np.asarray([sections[first_si][0]]
                        + [sections[si][1] for si, _ in group], float)
        raw = np.asarray([start_widths[first_si][first_k]]
                         + [end_widths[si][k] for si, k in group], float)
        if len(xs) < 2 or np.any(np.diff(xs) <= 1e-8):
            continue
        # source Link/ID 断开不等于物理车道生灭。只有横断面车道数确实增加/减少
        # 才建立零宽锥形；同车道数下的 ID/拓扑断链保持原宽，避免无依据挪动车道。
        born = (xs[0] > 1e-3 and first_si > 0
                and len(objects[first_si]) > len(objects[first_si - 1]))
        dying = (xs[-1] < road_length - 1e-3
                 and last_si + 1 < len(objects)
                 and len(objects[last_si]) > len(objects[last_si + 1]))
        transition = born or dying
        # 端点/非过渡普通车道权重大；过渡内部允许在 0.20m 量级调整，以消除
        # 原始 Link 粒度瞬变。数据不够时退回 PCHIP，不凭空制造额外自由度。
        weights = np.ones(len(xs), float)
        weights[[0, -1]] = 6.0
        if born:
            raw[0] = 0.0
            weights[0] = 80.0
        if dying:
            raw[-1] = 0.0
            weights[-1] = 80.0
        if len(xs) >= 4:
            budget = 0.15 if transition else 0.07
            spline = UnivariateSpline(xs, raw, w=weights,
                                      k=min(3, len(xs) - 1),
                                      s=len(xs) * budget ** 2)
            fitted = np.asarray(spline(xs), float)
            limit = 0.35 if transition else 0.12
            fitted = np.clip(fitted, raw - limit, raw + limit)
        else:
            fitted = raw.copy()
        fitted = np.maximum(fitted, 0.0)
        if born:
            fitted[0] = 0.0
            mask = xs <= xs[0] + _TAPER + 1e-9
            fitted[mask] = np.maximum.accumulate(fitted[mask])
        if dying:
            fitted[-1] = 0.0
            mask = xs >= xs[-1] - _TAPER - 1e-9
            idx = np.flatnonzero(mask)
            if len(idx):
                fitted[idx] = np.minimum.accumulate(fitted[idx])
        smoothed_tracks += 1
        transition_tracks += int(transition)
        tracks.append({"group": group, "xs": xs, "raw": raw, "values": fitted,
                       "born": born, "dying": dying, "transition": transition})

    # 车道生灭不是“最后一个小 section 内突然归零”，而是一条有明确长度的
    # 物理汇入/分出边界。由原始宽度首次离开/抵达平台的位置确定实际 horizon
    # （最多 60m），使用 quintic smoothstep 的站值/导数；相比 20m cubic，它在
    # 两端同时令一、二阶导数归零，避免 lane center 在 taper 端点产生曲率尖峰。
    for track in tracks:
        xs = track["xs"]
        values = track["values"]
        fixed = np.zeros(len(xs), dtype=bool)
        fixed_slopes = np.full(len(xs), np.nan, dtype=float)
        if track["born"] and len(xs) >= 2:
            window = np.flatnonzero(xs <= xs[0] + 60.0 + 1e-9)
            peak = float(np.max(values[window])) if len(window) else float(values[-1])
            reached = np.flatnonzero(values >= 0.90 * peak)
            end_idx = int(reached[0]) if len(reached) else len(xs) - 1
            end_idx = min(max(end_idx, 1), len(xs) - 1)
            length = max(float(xs[end_idx] - xs[0]), 1e-3)
            target = max(float(values[end_idx]), 0.0)
            q = np.clip((xs[:end_idx + 1] - xs[0]) / length, 0.0, 1.0)
            h = 10.0 * q ** 3 - 15.0 * q ** 4 + 6.0 * q ** 5
            dh = 30.0 * q ** 2 - 60.0 * q ** 3 + 30.0 * q ** 4
            values[:end_idx + 1] = target * h
            fixed_slopes[:end_idx + 1] = target * dh / length
            fixed[:end_idx + 1] = True
        if track["dying"] and len(xs) >= 2:
            window = np.flatnonzero(xs >= xs[-1] - 60.0 - 1e-9)
            peak = float(np.max(values[window])) if len(window) else float(values[0])
            plateau = np.flatnonzero((values >= 0.90 * peak)
                                     & (xs >= xs[-1] - 60.0 - 1e-9))
            start_idx = int(plateau[0]) if len(plateau) else max(0, len(xs) - 2)
            start_idx = min(max(start_idx, 0), len(xs) - 2)
            length = max(float(xs[-1] - xs[start_idx]), 1e-3)
            anchor = max(float(values[start_idx]), 0.0)
            q = np.clip((xs[start_idx:] - xs[start_idx]) / length, 0.0, 1.0)
            h = 10.0 * q ** 3 - 15.0 * q ** 4 + 6.0 * q ** 5
            dh = 30.0 * q ** 2 - 60.0 * q ** 3 + 30.0 * q ** 4
            values[start_idx:] = anchor * (1.0 - h)
            fixed_slopes[start_idx:] = -anchor * dh / length
            fixed[start_idx:] = True
        track["transition_fixed"] = fixed
        track["transition_slopes"] = fixed_slopes

    # 小于 5m 的 section 多为 source Link/采样站边界，不是道路设计控制点。
    # 保留 laneSection 做拓扑/来源分段，但几何站值改为跨窗插值，禁止在 1--3m
    # 内完成一次横移。生灭车道的解析 taper 已有物理含义，不参与此消噪。
    for track in tracks:
        xs, values = track["xs"], track["values"]
        fixed = track["transition_fixed"]
        for q in np.flatnonzero(np.diff(xs) < 5.0):
            lo, hi = q - 1, q + 2
            if lo < 0 or hi >= len(xs) or np.any(fixed[q:q + 2]):
                continue
            span = max(float(xs[hi] - xs[lo]), 1e-9)
            for j in (q, q + 1):
                u = float((xs[j] - xs[lo]) / span)
                values[j] = (1.0 - u) * values[lo] + u * values[hi]
            stats["micro_section_geometry_knots_removed"] = stats.get(
                "micro_section_geometry_knots_removed", 0) + 2

    # 同一站的所有车道宽度之和必须回到共享包络总宽。否则逐车道平滑会把
    # 本来平顺的道路外缘再次推成鼓包。以“track × station”统一变量归一化，
    # laneSection 两侧自然取到同一个值，不再靠事后平均缝接。
    desired_by_station = {}
    desired_slope_by_station = {}
    for si, (u0, u1) in enumerate(sections):
        desired_by_station.setdefault(round(float(u0), 4), []).append(
            float(desired_totals[si][0]))
        desired_by_station.setdefault(round(float(u1), 4), []).append(
            float(desired_totals[si][2]))
        desired_slope_by_station.setdefault(round(float(u0), 4), []).append(
            float(desired_totals[si][1]))
        desired_slope_by_station.setdefault(round(float(u1), 4), []).append(
            float(desired_totals[si][3]))
    desired_by_station = {key: float(np.median(values))
                          for key, values in desired_by_station.items()}
    desired_slope_by_station = {key: float(np.median(values))
                                for key, values in desired_slope_by_station.items()}
    base_by_station = {}
    for si, (u0, u1) in enumerate(sections):
        b0, _m0, b1, _m1 = base_offsets[si]
        base_by_station.setdefault(round(float(u0), 4), []).append(float(b0))
        base_by_station.setdefault(round(float(u1), 4), []).append(float(b1))
    base_by_station = {key: float(np.median(values))
                       for key, values in base_by_station.items()}
    envelope_keys = sorted(desired_by_station)
    for q, (a, b) in enumerate(zip(envelope_keys, envelope_keys[1:])):
        if b - a >= 5.0 or q == 0 or q + 2 >= len(envelope_keys):
            continue
        lo, hi = envelope_keys[q - 1], envelope_keys[q + 2]
        span = max(hi - lo, 1e-9)
        slope = (desired_by_station[hi] - desired_by_station[lo]) / span
        for key in (a, b):
            desired_by_station[key] = (desired_by_station[lo]
                                       + slope * (key - lo))
            desired_slope_by_station[key] = slope

    at_station = {}
    for ti, track in enumerate(tracks):
        for q, value in enumerate(track["xs"]):
            at_station.setdefault(round(float(value), 4), []).append((ti, q))

    # 生灭区内不仅零宽边界要平滑，剩余车道之间的宽度占比也必须在同一完整
    # horizon 内迁移；否则会在最后一个 2--3m 小段突然把 0.8m 从一条车道倒给
    # 另一条，外包络虽平滑，内部标线仍呈蛇形。
    windows = set()
    for track in tracks:
        idx = np.flatnonzero(track["transition_fixed"])
        if len(idx) >= 2:
            windows.add((round(float(track["xs"][idx[0]]), 4),
                         round(float(track["xs"][idx[-1]]), 4)))
    for wa, wb in sorted(windows):
        keys = sorted(key for key in at_station if wa <= key <= wb)
        if len(keys) < 2 or wb - wa < 1e-6:
            continue
        start_map = {ti: q for ti, q in at_station[keys[0]]}
        end_map = {ti: q for ti, q in at_station[keys[-1]]}
        stable = sorted(
            ti for ti in set(start_map) & set(end_map)
            if not tracks[ti]["transition_fixed"][start_map[ti]]
            and not tracks[ti]["transition_fixed"][end_map[ti]])
        if not stable:
            continue
        sv = np.asarray([tracks[ti]["values"][start_map[ti]] for ti in stable], float)
        ev = np.asarray([tracks[ti]["values"][end_map[ti]] for ti in stable], float)
        if sv.sum() <= 1e-9 or ev.sum() <= 1e-9:
            continue
        sshare, eshare = sv / sv.sum(), ev / ev.sum()
        for key in keys:
            refs = {ti: q for ti, q in at_station[key]}
            if not all(ti in refs for ti in stable):
                continue
            fixed_total = sum(
                float(tracks[ti]["values"][q]) for ti, q in at_station[key]
                if tracks[ti]["transition_fixed"][q])
            free_total = max(desired_by_station.get(key, fixed_total) - fixed_total, 0.0)
            u = np.clip((key - wa) / (wb - wa), 0.0, 1.0)
            h = 3.0 * u ** 2 - 2.0 * u ** 3
            share = (1.0 - h) * sshare + h * eshare
            share /= max(float(share.sum()), 1e-12)
            for ti, fraction in zip(stable, share):
                tracks[ti]["values"][refs[ti]] = free_total * float(fraction)

    transition_station_keys = {
        key for key, refs in at_station.items()
        if any(tracks[ti]["transition_fixed"][q] for ti, q in refs)
    }

    for _ in range(4):
        for key, refs in at_station.items():
            desired = desired_by_station.get(key)
            if desired is None:
                continue
            fixed_refs = [(ti, q) for ti, q in refs
                          if tracks[ti]["transition_fixed"][q]]
            free_refs = [(ti, q) for ti, q in refs
                         if not tracks[ti]["transition_fixed"][q]]
            fixed_total = sum(float(tracks[ti]["values"][q])
                              for ti, q in fixed_refs)
            current = sum(float(tracks[ti]["values"][q]) for ti, q in free_refs)
            target = max(desired - fixed_total, 0.0)
            if current <= 1e-9:
                continue
            scale = target / current
            for ti, q in free_refs:
                tracks[ti]["values"][q] = max(
                    0.0, float(tracks[ti]["values"][q]) * scale)
        for track in tracks:
            fitted = track["values"]
            xs = track["xs"]
            if track["born"]:
                fitted[0] = 0.0
                mask = xs <= xs[0] + _TAPER + 1e-9
                fitted[mask] = np.maximum.accumulate(fitted[mask])
            if track["dying"]:
                fitted[-1] = 0.0
                mask = xs >= xs[-1] - _TAPER - 1e-9
                idx = np.flatnonzero(mask)
                if len(idx):
                    fitted[idx] = np.minimum.accumulate(fitted[idx])

    # 最后一遍严格归一化；出生/消失站的零宽值乘比例后仍为零。
    for key, refs in at_station.items():
        desired = desired_by_station.get(key)
        if desired is None:
            continue
        fixed_refs = [(ti, q) for ti, q in refs
                      if tracks[ti]["transition_fixed"][q]]
        free_refs = [(ti, q) for ti, q in refs
                     if not tracks[ti]["transition_fixed"][q]]
        fixed_total = sum(float(tracks[ti]["values"][q])
                          for ti, q in fixed_refs)
        current = sum(float(tracks[ti]["values"][q]) for ti, q in free_refs)
        target = max(desired - fixed_total, 0.0)
        if current > 1e-9:
            scale = target / current
            for ti, q in free_refs:
                tracks[ti]["values"][q] *= scale

    # 归一化是横断面的最后一次站值改写，因此微小 laneSection 的去噪也必须放在
    # 归一化之后。旧顺序先消除 1--3m 几何节点、随后又按原始总宽逐站缩放，等于
    # 把同一个坏节点重新写回；road11 的 1.0777m section 即因此在可比车道中心上
    # 产生了很大的曲率变化率。这里只移动几何站值，不删除拓扑 laneSection；
    # birth/death 的解析 taper 继续由 transition_fixed 保护。
    for track in tracks:
        xs, values = track["xs"], track["values"]
        fixed = track["transition_fixed"]
        for q in np.flatnonzero(np.diff(xs) < 5.0):
            lo, hi = q - 1, q + 2
            if lo < 0 or hi >= len(xs) or np.any(fixed[q:q + 2]):
                continue
            span = max(float(xs[hi] - xs[lo]), 1e-9)
            for j in (q, q + 1):
                u = float((xs[j] - xs[lo]) / span)
                values[j] = (1.0 - u) * values[lo] + u * values[hi]
            stats["micro_section_post_normalization_repairs"] = stats.get(
                "micro_section_post_normalization_repairs", 0) + 2

    # ------------------------------------------------------------------
    # 车道中心是自动驾驶真正跟踪的轨迹。仅分别平滑 laneOffset 与 width，二者
    # 仍可能叠加成蛇形；因此先在最终绝对横距域对每条连续 lane track 求最平滑
    # 可行中心，再以“中心误差 + 总宽等式 + 最小宽度改动”的凸二次问题反解每站
    # width。这里不增加 laneSection，也不增加 width 记录，只改已有公共站值。
    node_ref = {}
    for ti, track in enumerate(tracks):
        for q, node in enumerate(track["group"]):
            node_ref[node] = (ti, q)

    def _section_width_values(si, at_end):
        values = []
        for k in range(len(objects[si])):
            ref = node_ref.get((si, k))
            if ref is None:
                values.append(float(end_widths[si][k] if at_end
                                    else start_widths[si][k]))
            else:
                ti, q = ref
                values.append(float(tracks[ti]["values"][q + int(at_end)]))
        return np.asarray(values, float)

    current_starts = [_section_width_values(si, False)
                      for si in range(len(objects))]
    current_ends = [_section_width_values(si, True)
                    for si in range(len(objects))]
    center_targets = {}
    center_tracks_smoothed = 0
    center_adjust_max = 0.0
    for ti, track in enumerate(tracks):
        xs = track["xs"]
        group = track["group"]
        if len(xs) < 4 or xs[-1] - xs[0] < 20.0:
            continue
        first_si, first_k = group[0]
        raw = [float(base_offsets[first_si][0]) + float(side_sign) * (
            float(np.sum(current_starts[first_si][:first_k]))
            + float(current_starts[first_si][first_k]) / 2.0)]
        source_mask = []
        first_prov = objects[first_si][first_k].provenance or {}
        source_mask.append(first_prov.get("eligibility") == "comparable")
        for si, k in group:
            raw.append(float(base_offsets[si][2]) + float(side_sign) * (
                float(np.sum(current_ends[si][:k]))
                + float(current_ends[si][k]) / 2.0))
            provenance = objects[si][k].provenance or {}
            source_mask.append(provenance.get("eligibility") == "comparable")
        raw = np.asarray(raw, float)
        source_mask = np.asarray(source_mask, bool)
        weights = np.where(source_mask, 1.0, 0.05)
        if source_mask[0]:
            weights[0] = 8.0
        if source_mask[-1]:
            weights[-1] = 8.0
        candidates = []
        for budget in (0.08, 0.12, 0.18, 0.25, 0.35, 0.50, 0.75, 1.0,
                       1.5, 2.0):
            try:
                spline = UnivariateSpline(xs, raw, w=weights, k=3,
                                          s=len(xs) * budget ** 2)
                fitted = np.asarray(spline(xs), float)
            except Exception:
                continue
            source_dev = (float(np.max(np.abs(fitted[source_mask] - raw[source_mask])))
                          if np.any(source_mask) else 0.0)
            inferred_dev = (float(np.max(np.abs(fitted[~source_mask] - raw[~source_mask])))
                            if np.any(~source_mask) else 0.0)
            if source_dev > 0.35 + 1e-9 or inferred_dev > 1.50 + 1e-9:
                continue
            curve = CubicSpline(xs, fitted, bc_type="natural")
            dense = np.linspace(xs[0], xs[-1],
                                max(101, int(xs[-1] - xs[0]) * 2))
            score = (float(np.max(np.abs(curve(dense, 3)))),
                     float(np.max(np.abs(curve(dense, 2)))), source_dev)
            candidates.append((score, fitted))
        if not candidates:
            continue
        _score, fitted = min(candidates, key=lambda item: item[0])
        # 无源前缀/后缀按最近真实中心轨迹作线性延拓。它不是来源拟合区，允许在
        # 1.5m 内移动，但必须保持动力学连续，不能用逐站伪观测画出 S 形摆动。
        supported_idx = np.flatnonzero(source_mask)
        if len(supported_idx):
            first_supported = int(supported_idx[0])
            last_supported = int(supported_idx[-1])
            if first_supported >= 1 and first_supported + 1 < len(xs):
                slope = ((fitted[first_supported + 1] - fitted[first_supported])
                         / max(xs[first_supported + 1] - xs[first_supported], 1e-9))
                proposal = (fitted[first_supported]
                            + slope * (xs[:first_supported] - xs[first_supported]))
                lo = raw[:first_supported] - 1.50
                hi = raw[:first_supported] + 1.50
                fitted[:first_supported] = np.clip(proposal, lo, hi)
            if last_supported + 1 < len(xs) and last_supported >= 1:
                slope = ((fitted[last_supported] - fitted[last_supported - 1])
                         / max(xs[last_supported] - xs[last_supported - 1], 1e-9))
                proposal = (fitted[last_supported]
                            + slope * (xs[last_supported + 1:] - xs[last_supported]))
                lo = raw[last_supported + 1:] - 1.50
                hi = raw[last_supported + 1:] + 1.50
                fitted[last_supported + 1:] = np.clip(proposal, lo, hi)
        # 首末微 section 常是停止线/Link 圆整产生的 1--3m 站，不是新线形控制点。
        # 端点沿相邻宽段斜率外推，真实点仍受 0.35m 位移上限。
        if len(xs) >= 3 and xs[-1] - xs[-2] < 5.0:
            slope = ((fitted[-2] - fitted[-3])
                     / max(xs[-2] - xs[-3], 1e-9))
            proposal = fitted[-2] + slope * (xs[-1] - xs[-2])
            limit = 0.35 if source_mask[-1] else 1.50
            fitted[-1] = float(np.clip(proposal, raw[-1] - limit, raw[-1] + limit))
        if len(xs) >= 3 and xs[1] - xs[0] < 5.0:
            slope = ((fitted[2] - fitted[1])
                     / max(xs[2] - xs[1], 1e-9))
            proposal = fitted[1] - slope * (xs[1] - xs[0])
            limit = 0.35 if source_mask[0] else 1.50
            fitted[0] = float(np.clip(proposal, raw[0] - limit, raw[0] + limit))
        for q, value in enumerate(fitted):
            center_targets[(ti, q)] = float(value)
        center_tracks_smoothed += 1
        center_adjust_max = max(center_adjust_max,
                                float(np.max(np.abs(fitted - raw))))

    proposals = {}
    total_proposals = {}
    if center_targets:
        for si in range(len(objects)):
            for at_end, raw_widths, base_index, station in (
                    (False, current_starts[si], 0, sections[si][0]),
                    (True, current_ends[si], 2, sections[si][1])):
                n = len(raw_widths)
                if not n:
                    continue
                key = round(float(station), 4)
                base = float(base_offsets[si][base_index])
                A = np.tril(np.ones((n, n), float), -1) + 0.5 * np.eye(n)
                raw_centers = A @ raw_widths
                targets = raw_centers.copy()
                target_weights = np.ones(n, float)
                fixed_zero = set()
                refs = []
                for k in range(n):
                    ti, q0 = node_ref[(si, k)]
                    q = q0 + int(at_end)
                    refs.append((ti, q))
                    if (ti, q) in center_targets:
                        targets[k] = float(side_sign) * (
                            center_targets[(ti, q)] - base)
                        # 车道中心是车辆实际跟踪轨迹；若这里只给弱权重，后面的
                        # 总宽正则会把已经平滑的中心重新拉回逐 Link 噪声。提高中心
                        # 约束，同时仍由总宽等式和非负约束守住道路包络。
                        target_weights[k] = 60.0
                    if (raw_widths[k] <= 1e-7
                            and tracks[ti]["transition_fixed"][q]):
                        fixed_zero.add(k)
                W = np.diag(np.sqrt(target_weights))
                M = W @ A
                rhs = W @ targets
                reg = 0.15
                H = M.T @ M + reg * np.eye(n)
                bvec = M.T @ rhs + reg * raw_widths
                desired = float(desired_by_station.get(key, np.sum(raw_widths)))
                try:
                    unconstrained = np.linalg.solve(H, bvec)
                except np.linalg.LinAlgError:
                    unconstrained = np.linalg.lstsq(H, bvec, rcond=None)[0]
                preferred_total = float(np.sum(np.maximum(unconstrained, 0.0)))
                target_total = (float(np.clip(preferred_total,
                                              max(0.5, desired - 0.35),
                                              desired + 0.35))
                                if road_length >= 120.0 and allow_total_relax else desired)
                solved = np.zeros(n, float)
                free = [k for k in range(n) if k not in fixed_zero]
                while free:
                    Hf = H[np.ix_(free, free)]
                    bf = bvec[free]
                    ones = np.ones(len(free), float)
                    kkt = np.block([[Hf, ones[:, None]],
                                    [ones[None, :], np.zeros((1, 1))]])
                    vec = np.concatenate([bf, [target_total]])
                    try:
                        answer = np.linalg.solve(kkt, vec)[:-1]
                    except np.linalg.LinAlgError:
                        answer = np.linalg.lstsq(kkt, vec, rcond=None)[0][:-1]
                    if float(np.min(answer)) >= -1e-8:
                        solved[free] = np.maximum(answer, 0.0)
                        break
                    worst = free[int(np.argmin(answer))]
                    fixed_zero.add(worst)
                    free = [k for k in free if k != worst]
                # 一次站值最多改 1m；原值已经满足总宽，线性回退仍保持等式和非负。
                delta = solved - raw_widths
                alpha = min(1.0, 1.0 / max(float(np.max(np.abs(delta))), 1.0))
                solved = raw_widths + alpha * delta
                total_proposals.setdefault(key, []).append(float(np.sum(solved)))
                for ref, value in zip(refs, solved):
                    proposals.setdefault(ref, []).append(float(value))
        for (ti, q), values in proposals.items():
            tracks[ti]["values"][q] = max(0.0, float(np.median(values)))
        # 相邻 section 对共享站提出的解取中位后，恢复精确总宽等式。
        for key, refs in at_station.items():
            desired = (float(np.median(total_proposals[key]))
                       if key in total_proposals else desired_by_station.get(key))
            current = sum(float(tracks[ti]["values"][q]) for ti, q in refs)
            if desired is not None and current > 1e-9:
                scale = float(desired) / current
                for ti, q in refs:
                    tracks[ti]["values"][q] *= scale
        # 量化“平滑车道中心目标”在共享包络/非负宽度联合求解后被牺牲了多少。
        # 该残差是判断需要调联合权重、还是改用分向 carriageway 的直接证据；
        # 只写统计，不改变几何。
        target_residual_max = 0.0
        target_residual_worst = None
        for ti, track in enumerate(tracks):
            for q, (si, k) in enumerate(track["group"]):
                for at_end, tq, base_index in ((False, q, 0), (True, q + 1, 2)):
                    target = center_targets.get((ti, tq))
                    if target is None:
                        continue
                    widths = _section_width_values(si, at_end)
                    actual = float(base_offsets[si][base_index]) + float(side_sign) * (
                        float(np.sum(widths[:k])) + float(widths[k]) / 2.0)
                    residual = abs(actual - float(target))
                    if residual > target_residual_max:
                        target_residual_max = residual
                        target_residual_worst = {
                            "source_lane": objects[si][k].source_id,
                            "section": si, "station_m": float(sections[si][int(at_end)]),
                            "target_m": float(target), "actual_m": actual,
                        }
        stats[f"{side_name}_center_target_residual_max_m"] = target_residual_max
        if target_residual_worst is not None:
            stats[f"{side_name}_center_target_residual_worst"] = target_residual_worst
        stats[f"{side_name}_joint_envelope_adjust_max_m"] = max(
            (abs(float(np.median(values)) - desired_by_station[key])
             for key, values in total_proposals.items()), default=0.0)
    stats[f"{side_name}_center_tracks_smoothed"] = center_tracks_smoothed
    stats[f"{side_name}_center_track_adjust_max_m"] = center_adjust_max

    # PCHIP 给出每条车道的公共站导数；随后把“各车道导数之和”约束到共享
    # 包络导数。这样 Σwidth 的 Hermite 多项式在整段（不仅端点）都等于包络，
    # 无需再在某个 1.8m 小段把全部闭合残差硬塞进单条车道。
    for track in tracks:
        curve = PchipInterpolator(track["xs"], track["values"])
        track["slopes"] = np.asarray(curve.derivative()(track["xs"]), float)
        mask = track["transition_fixed"]
        track["slopes"][mask] = track["transition_slopes"][mask]
        if track["born"]:
            track["slopes"][0] = 0.0
        if track["dying"]:
            track["slopes"][-1] = 0.0

    # 宽度导数必须从“累计共享边界”求差，不能逐 lane 独立取 PCHIP。后者在
    # lane 数变化的 1m 小段会给相邻宽度分配互相抵消的大导数，车道中心蛇形，
    # 即使总宽看起来闭合。累计边界用全局自然三次样条，只求导数、不改站值。
    node_ref = {}
    for ti, track in enumerate(tracks):
        for q, node in enumerate(track["group"]):
            node_ref[node] = (ti, q)

    starts, ends = [], []
    for si, lanes in enumerate(objects):
        sv, ev = [], []
        for k in range(len(lanes)):
            ref = node_ref.get((si, k))
            if ref is None:
                sv.append(float(start_widths[si][k]))
                ev.append(float(end_widths[si][k]))
            else:
                ti, q = ref
                sv.append(float(tracks[ti]["values"][q]))
                ev.append(float(tracks[ti]["values"][q + 1]))
        starts.append(np.asarray(sv, float))
        ends.append(np.asarray(ev, float))

    boundary_nodes = [(si, edge) for si, lanes in enumerate(objects)
                      for edge in range(len(lanes) + 1)]
    bparent = {node: node for node in boundary_nodes}

    def bfind(node):
        while bparent[node] != node:
            bparent[node] = bparent[bparent[node]]
            node = bparent[node]
        return node

    def bunion(a, b):
        ra, rb = bfind(a), bfind(b)
        if ra != rb:
            bparent[rb] = ra

    # lane i->j 同时证明它的内、外边界 i->j 与 i+1->j+1 连续；只有
    # 双向唯一的边界对应才合并，分裂/汇合冲突处保持两条独立轨迹。
    for si, mapping in enumerate(matches):
        a_to, b_to = {}, {}
        for i, j in mapping.items():
            for ea, eb in ((i, j), (i + 1, j + 1)):
                if ((si, ea) in bparent and (si + 1, eb) in bparent):
                    a_to.setdefault(ea, set()).add(eb)
                    b_to.setdefault(eb, set()).add(ea)
        for ea, targets in a_to.items():
            if len(targets) != 1:
                continue
            eb = next(iter(targets))
            if len(b_to.get(eb, ())) == 1:
                bunion((si, ea), (si + 1, eb))

    bgroups = {}
    for node in boundary_nodes:
        bgroups.setdefault(bfind(node), []).append(node)

    # ------------------------------------------------------------------
    # 稳定车道的内部共享边界不能直接穿过每个 5--10m 采样站。
    # 即使站值误差只有 0.1--0.3m，天然三次样条也会在短跨度内
    # 生成很大的二/三阶导数，最终让 lane center 在 60km/h 下超过
    # G11-D。这里以“物理边界轨迹”为对象，在不改内/外包络、不跨
    # birth/death 的前提下做全局容差平滑。后续 width 从这些共享边界
    # 作差得到，不再让各 lane 独立平滑后互相抵消。
    bstarts = [np.concatenate([[0.0], np.cumsum(values)]) for values in starts]
    bends = [np.concatenate([[0.0], np.cumsum(values)]) for values in ends]
    proposed_starts = [values.copy() for values in bstarts]
    proposed_ends = [values.copy() for values in bends]
    boundary_adjust_max = 0.0
    boundary_tracks_smoothed = 0

    def _edge_touches_transition(si, edge):
        for lane_index in (edge - 1, edge):
            ref = node_ref.get((si, lane_index))
            if ref is None:
                continue
            ti, q = ref
            fixed = tracks[ti]["transition_fixed"]
            # 一条车道可能只在几十米外生/灭；不能因此冻结它整条边界轨迹。
            # 只保护当前 section 两端确属解析 taper 的站点。
            if bool(fixed[q]) or bool(fixed[q + 1]):
                return True
        return False

    def _edge_has_source_support(si, edge):
        adjacent = []
        for lane_index in (edge - 1, edge):
            if not (0 <= lane_index < len(objects[si])):
                continue
            provenance = objects[si][lane_index].provenance or {}
            adjacent.append(provenance.get("support_kind") != "source-extension")
        return any(adjacent)

    def _safe_boundary_runs(group):
        """把物理边界按局部 taper 窗口切开，而不是因远处拓扑事件整条跳过。"""
        runs, current = [], []
        for node in sorted(group):
            si, edge = node
            safe = not _edge_touches_transition(si, edge)
            contiguous = not current or si == current[-1][0] + 1
            if not safe or not contiguous:
                if current:
                    runs.append(current)
                    current = []
                if not safe:
                    continue
            current.append(node)
        if current:
            runs.append(current)
        return runs

    # 站值已经由车道中心联合二次问题确定；此处只保留后面的共享边界导数求解，
    # 不再用另一套独立 smoothing spline 二次改写中心。
    for group in ():
        # edge=0 / edge=n 是已由来源边界锁定的道路包络；本步只修内部标线。
        if any(edge <= 0 or edge >= len(starts[si]) for si, edge in group):
            continue
        for run in _safe_boundary_runs(group):
            observations = {}
            support = {}
            for si, edge in run:
                for station, value in (
                        (float(sections[si][0]), float(bstarts[si][edge])),
                        (float(sections[si][1]), float(bends[si][edge]))):
                    observations.setdefault(station, []).append(value)
                    support.setdefault(station, []).append(
                        _edge_has_source_support(si, edge))
            x = np.asarray(sorted(observations), float)
            y = np.asarray([np.median(observations[value]) for value in x], float)
            source_mask = np.asarray([any(support[value]) for value in x], bool)
            if len(x) < 4 or x[-1] - x[0] < 20.0 or not np.any(source_mask):
                continue
            # 无源延伸只是待求的平滑延拓，不应和真实 SHP 观测同权。真实支持点
            # 保持逐站 0.35m 硬上限；延伸点只受边界顺序和全局平滑约束。
            weights = np.where(source_mask, 1.0, 0.05)
            if source_mask[0]:
                weights[0] = 8.0
            if source_mask[-1]:
                weights[-1] = 8.0
            candidates = []
            for budget in (0.08, 0.12, 0.18, 0.25, 0.35, 0.50, 0.75, 1.0,
                           1.5, 2.0):
                try:
                    spl = UnivariateSpline(x, y, w=weights, k=3,
                                           s=len(x) * budget ** 2)
                    fitted = np.asarray(spl(x), float)
                except Exception:
                    continue
                source_dev = float(np.max(np.abs(fitted[source_mask] - y[source_mask])))
                if source_dev > 0.35 + 1e-9:
                    continue
                curve = CubicSpline(x, fitted, bc_type="natural")
                dense = np.linspace(x[0], x[-1],
                                    max(101, int(x[-1] - x[0]) * 2))
                roughness = (float(np.max(np.abs(curve(dense, 3)))),
                             float(np.max(np.abs(curve(dense, 2)))), source_dev)
                candidates.append((roughness, fitted))
            if not candidates:
                continue
            _roughness, fitted = min(candidates, key=lambda item: item[0])
            by_station = {float(station): float(value)
                          for station, value in zip(x, fitted)}
            for si, edge in run:
                proposed_starts[si][edge] = by_station[float(sections[si][0])]
                proposed_ends[si][edge] = by_station[float(sections[si][1])]
            boundary_adjust_max = max(
                boundary_adjust_max, float(np.max(np.abs(fitted - y))))
            boundary_tracks_smoothed += 1

    # 各内部边界独立拟合后做统一线搜：若局部宽度会变负，同比收缩
    # 所有修正，保持边界顺序和道路总宽。
    alpha = 1.0
    while alpha > 1e-5:
        safe = True
        for original, proposed in zip(bstarts + bends,
                                      proposed_starts + proposed_ends):
            trial = original + alpha * (proposed - original)
            if float(np.min(np.diff(trial))) < -1e-6:
                safe = False
                break
        if safe:
            break
        alpha *= 0.5
    if alpha <= 1e-5:
        alpha = 0.0
    for si in range(len(objects)):
        bs = bstarts[si] + alpha * (proposed_starts[si] - bstarts[si])
        be = bends[si] + alpha * (proposed_ends[si] - bends[si])
        starts[si] = np.diff(bs)
        ends[si] = np.diff(be)
    # 将边界轨迹反解回每条连续 lane width 的公共站值。
    for track in tracks:
        for q, (si, k) in enumerate(track["group"]):
            if q == 0:
                track["values"][0] = float(starts[si][k])
            track["values"][q + 1] = float(ends[si][k])
    stats[f"{side_name}_internal_boundary_tracks_smoothed"] = boundary_tracks_smoothed
    stats[f"{side_name}_internal_boundary_adjust_max_m"] = boundary_adjust_max * alpha
    stats[f"{side_name}_internal_boundary_adjust_scale"] = alpha

    start_edge_slopes = [np.full(len(lanes) + 1, np.nan, float)
                         for lanes in objects]
    end_edge_slopes = [np.full(len(lanes) + 1, np.nan, float)
                       for lanes in objects]
    for group in bgroups.values():
        observations = {}
        for si, edge in group:
            observations.setdefault(float(sections[si][0]), []).append(
                float(np.sum(starts[si][:edge])))
            observations.setdefault(float(sections[si][1]), []).append(
                float(np.sum(ends[si][:edge])))
        x = np.asarray(sorted(observations), float)
        y = np.asarray([np.median(observations[value]) for value in x], float)
        if len(x) >= 3:
            curve = CubicSpline(x, y, bc_type="natural")
            derivative = {float(value): float(curve(value, 1)) for value in x}
        elif len(x) == 2:
            slope = float((y[1] - y[0]) / max(x[1] - x[0], 1e-9))
            derivative = {float(value): slope for value in x}
        else:
            derivative = {float(x[0]): 0.0}
        for si, edge in group:
            start_edge_slopes[si][edge] = derivative[float(sections[si][0])]
            end_edge_slopes[si][edge] = derivative[float(sections[si][1])]

    for si in range(len(objects)):
        start_edge_slopes[si] = np.nan_to_num(start_edge_slopes[si])
        end_edge_slopes[si] = np.nan_to_num(end_edge_slopes[si])
        for k, width in enumerate(starts[si]):
            if width < 1e-5:
                start_edge_slopes[si][k + 1] = start_edge_slopes[si][k]
        for k, width in enumerate(ends[si]):
            if width < 1e-5:
                end_edge_slopes[si][k + 1] = end_edge_slopes[si][k]

    # 不再强迫 Σ(width') 追随一条独立拟合的包络导数。累计边界公共站点的
    # 全局导数已经保证所有堆叠边缘 C1。

    for track in tracks:
        group = track["group"]
        xs = track["xs"]
        fitted = track["values"]
        raw = track["raw"]
        slopes = track["slopes"]
        adjusted_max = max(adjusted_max, float(np.max(np.abs(fitted - raw))))

        for q, (si, k) in enumerate(group):
            length = max(float(sections[si][1] - sections[si][0]), 1e-3)
            w0, w1 = float(fitted[q]), float(fitted[q + 1])
            m0 = float(start_edge_slopes[si][k + 1] - start_edge_slopes[si][k])
            m1 = float(end_edge_slopes[si][k + 1] - end_edge_slopes[si][k])
            c = (3 * (w1 - w0) - (2 * m0 + m1) * length) / length ** 2
            d = (-2 * (w1 - w0) + (m0 + m1) * length) / length ** 3
            sample = np.linspace(0.0, length, 31)
            width_sample = w0 + m0 * sample + c * sample ** 2 + d * sample ** 3
            if float(width_sample.min()) < -1e-5:
                # 极端病态站才退回单调限幅；记录后由形态门禁拦截包络闭合误差。
                m0, m1 = _fc(w0, m0, w1, m1, length)
                c = (3 * (w1 - w0) - (2 * m0 + m1) * length) / length ** 2
                d = (-2 * (w1 - w0) + (m0 + m1) * length) / length ** 3
                stats["width_nonnegative_fallback"] = stats.get(
                    "width_nonnegative_fallback", 0) + 1
            objects[si][k].widths = [(0.0, w0, m0, c, d)]
            start_widths[si][k] = w0
            end_widths[si][k] = w1
            in_transition = (
                round(float(sections[si][0]), 4) in transition_station_keys
                or round(float(sections[si][1]), 4) in transition_station_keys
                or max(abs(float(fitted[q]) - float(raw[q])),
                       abs(float(fitted[q + 1]) - float(raw[q + 1]))) > 0.05)
            if k < len(original_start_widths[si]):
                old_c0 = (sum(original_start_widths[si][:k])
                          + original_start_widths[si][k] / 2.0)
                old_c1 = (sum(original_end_widths[si][:k])
                          + original_end_widths[si][k] / 2.0)
                new_c0 = sum(start_widths[si][:k]) + start_widths[si][k] / 2.0
                new_c1 = sum(end_widths[si][:k]) + end_widths[si][k] / 2.0
                in_transition = in_transition or max(
                    abs(new_c0 - old_c0), abs(new_c1 - old_c1)) > 0.05
            if in_transition:
                provenance = objects[si][k].provenance
                if provenance is not None and provenance.get("eligibility") == "comparable":
                    provenance.update({
                        "eligibility": "excluded", "status": "APPROXIMATED",
                        "support_kind": "lane-transition-ribbon",
                        "exclusion_code": "lane-transition-taper",
                        "geometry_adjustment": "lane-transition-ribbon",
                    })
                    provenance.pop("support_s", None)

    stats[f"{side_name}_width_tracks_smoothed"] = smoothed_tracks
    stats[f"{side_name}_transition_tracks"] = transition_tracks
    stats["width_track_max_adjust_m"] = max(
        stats.get("width_track_max_adjust_m", 0.0), adjusted_max)


def _weld_linked_lane_width_derivatives(sections, matches, objects, stats, side_name):
    """构造性焊接相邻 laneSection 中同一物理车道的 width 一阶导数。"""
    def endpoint(record, length):
        _so, a, b, c, d = record
        return (float(a + b * length + c * length ** 2 + d * length ** 3),
                float(b + 2.0 * c * length + 3.0 * d * length ** 2))

    def hermite(w0, m0, w1, m1, length):
        c = (3.0 * (w1 - w0) - (2.0 * m0 + m1) * length) / length ** 2
        d = (-2.0 * (w1 - w0) + (m0 + m1) * length) / length ** 3
        return (0.0, float(w0), float(m0), float(c), float(d))

    def nonnegative(record, length):
        _so, a, b, c, d = record
        q = np.linspace(0.0, length, 61)
        return float(np.min(a + b * q + c * q ** 2 + d * q ** 3)) >= -1e-6

    welded = 0
    max_before = 0.0
    for si, mapping in enumerate(matches):
        if si + 1 >= len(objects):
            break
        lp = max(float(sections[si][1] - sections[si][0]), 1e-3)
        ln = max(float(sections[si + 1][1] - sections[si + 1][0]), 1e-3)
        for ia, ib in mapping.items():
            if ia >= len(objects[si]) or ib >= len(objects[si + 1]):
                continue
            prev, nxt = objects[si][ia], objects[si + 1][ib]
            if len(prev.widths) != 1 or len(nxt.widths) != 1:
                continue
            pa, pb = prev.widths[0], nxt.widths[0]
            pw0, pm0 = float(pa[1]), float(pa[2])
            pw1, pm1 = endpoint(pa, lp)
            nw0, nm0 = float(pb[1]), float(pb[2])
            nw1, nm1 = endpoint(pb, ln)
            join_value = 0.5 * (pw1 + nw0)
            candidate = (pm1 * lp + nm0 * ln) / (lp + ln)
            max_before = max(max_before, abs(pm1 - nm0))
            scale = 1.0
            while scale > 1e-4:
                common = candidate * scale
                left = hermite(pw0, pm0, join_value, common, lp)
                right = hermite(join_value, common, nw1, nm1, ln)
                if nonnegative(left, lp) and nonnegative(right, ln):
                    prev.widths = [left]
                    nxt.widths = [right]
                    welded += 1
                    break
                scale *= 0.5
    stats[f"{side_name}_lane_width_derivative_welds"] = welded
    stats["lane_width_derivative_jump_before_max"] = max(
        stats.get("lane_width_derivative_jump_before_max", 0.0), max_before)


def _smooth_lane_center_tracks(sections, specs, matches, stats, side_name,
                               max_adjust_m=0.25):
    """沿 laneLink 连续关系整体拟合来源车道中心横距。

    参考线与 width 各自平滑并不能保证最终 lane center 平滑；共享参考线略弯时，
    一条真实直车道必须由缓慢变化的横距抵消该弯曲。这里把每条连续物理车道的
    v(s) 作为直接对象，在来源容差内选择三阶导数最小的全局样条，再由后续
    `_bounds_at` 反解共享边界和宽度。拓扑 section 不删除，也不增加几何记录。
    """
    from scipy.interpolate import UnivariateSpline

    nodes = [(si, k) for si, spec in enumerate(specs) if spec is not None
             for k in range(len(spec["lanes"]))]
    parent = {node: node for node in nodes}

    def find(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    def union(a, b):
        if a not in parent or b not in parent:
            return
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for si, mapping in enumerate(matches):
        for ia, ib in mapping.items():
            union((si, ia), (si + 1, ib))
    groups = {}
    for node in nodes:
        groups.setdefault(find(node), []).append(node)

    changed_tracks = changed_stations = 0
    max_adjust = 0.0
    for group in groups.values():
        group.sort()
        if len({si for si, _k in group}) != len(group):
            continue
        first_si, first_k = group[0]
        xs = np.asarray([sections[first_si][0]]
                        + [sections[si][1] for si, _k in group], float)
        raw = np.asarray([specs[first_si]["lanes"][first_k]["v0"]]
                         + [specs[si]["lanes"][k]["v1"] for si, k in group], float)
        if len(xs) < 4 or xs[-1] - xs[0] < 20.0 or np.any(np.diff(xs) <= 1e-8):
            continue
        weights = np.ones(len(xs), float)
        # road 端点和车道轨迹端点具有口部/停止线意义，权重高但不强制逐点插值。
        weights[[0, -1]] = 12.0
        candidates = []
        n = len(xs)
        for factor in (0.01, 0.04, 0.09, 0.16, 0.25, 0.49, 1.0, 2.0, 4.0):
            try:
                curve = UnivariateSpline(xs, raw, w=weights, k=3,
                                         s=n * factor ** 2)
            except Exception:
                continue
            fitted = np.asarray(curve(xs), float)
            dev = float(np.max(np.abs(fitted - raw)))
            if dev > max_adjust_m + 1e-9:
                continue
            dense = np.linspace(xs[0], xs[-1], max(101, int(xs[-1] - xs[0]) * 2))
            d3 = np.asarray(curve.derivative(3)(dense), float)
            d2 = np.asarray(curve.derivative(2)(dense), float)
            rough = (float(np.max(np.abs(d3))), float(np.max(np.abs(d2))), -factor)
            candidates.append((rough, curve, fitted, dev))
        if not candidates:
            continue
        _score, curve, fitted, dev = min(candidates, key=lambda item: item[0])
        slopes = np.asarray(curve.derivative()(xs), float)
        # 只有实质改善才改写，避免给本已线性的轨迹增加浮点噪声。
        raw_curve = UnivariateSpline(xs, raw, k=3, s=0.0)
        dense = np.linspace(xs[0], xs[-1], max(101, int(xs[-1] - xs[0]) * 2))
        if (float(np.max(np.abs(curve.derivative(3)(dense)))) >=
                float(np.max(np.abs(raw_curve.derivative(3)(dense)))) - 1e-10):
            continue
        for q, (si, k) in enumerate(group):
            lane = specs[si]["lanes"][k]
            lane["v0"], lane["m0"] = float(fitted[q]), float(slopes[q])
            lane["v1"], lane["m1"] = float(fitted[q + 1]), float(slopes[q + 1])
        changed_tracks += 1
        changed_stations += len(xs)
        max_adjust = max(max_adjust, dev)
    stats[f"{side_name}_lane_center_tracks_smoothed"] = changed_tracks
    stats[f"{side_name}_lane_center_stations_smoothed"] = changed_stations
    stats["lane_center_track_max_adjust_m"] = max(
        stats.get("lane_center_track_max_adjust_m", 0.0), max_adjust)


def _enforce_shared_envelope(sections, bounds, objects, start_widths, end_widths,
                             sign, stats, side_name):
    """令 Σlane.width(s) 与共享物理包络三次多项式严格相等。

    单独平滑每条 lane 即使端点宽度和正确，端点导数之和仍可能不正确，造成道路
    外缘在 laneSection 处有肉眼可见的折角。把极小的闭合残差吸收到本断面最宽的
    非零车道，既不改变道路包络，也不增加虚构 shoulder。
    """
    correction_max = 0.0
    for si, ((u0, u1), pair, lanes) in enumerate(zip(sections, bounds, objects)):
        if pair is None or not lanes:
            continue
        (b0, m0), (b1, m1) = pair
        length = max(float(u1 - u0), 1e-3)
        w0 = sign * (float(b0[-1]) - float(b0[0]))
        w1 = sign * (float(b1[-1]) - float(b1[0]))
        mw0 = sign * (float(m0[-1]) - float(m0[0]))
        mw1 = sign * (float(m1[-1]) - float(m1[0]))
        total = np.asarray([
            w0, mw0,
            (3 * (w1 - w0) - (2 * mw0 + mw1) * length) / length ** 2,
            (-2 * (w1 - w0) + (mw0 + mw1) * length) / length ** 3,
        ], float)
        coeffs = []
        for lane in lanes:
            if len(lane.widths) != 1 or abs(float(lane.widths[0][0])) > 1e-9:
                coeffs = []
                break
            _so, a, b, c, d = lane.widths[0]
            coeffs.append(np.asarray([a, b, c, d], float))
        if not coeffs:
            continue
        current = np.sum(coeffs, axis=0)
        delta = total - current
        # 优先选择两端都非零、且最宽的车道吸收闭合残差；避免改变生灭车道的
        # 零宽边界条件。没有稳定车道时才选平均宽度最大者。
        candidates = [k for k in range(len(lanes))
                      if min(start_widths[si][k], end_widths[si][k]) > 0.8]
        if not candidates:
            candidates = list(range(len(lanes)))
        chosen = max(candidates,
                     key=lambda k: start_widths[si][k] + end_widths[si][k])
        new = coeffs[chosen] + delta
        q = np.linspace(0.0, length, 41)
        values = new[0] + new[1] * q + new[2] * q ** 2 + new[3] * q ** 3
        if float(values.min()) < -1e-5:
            stats[f"{side_name}_envelope_closure_rejected"] = stats.get(
                f"{side_name}_envelope_closure_rejected", 0) + 1
            continue
        lanes[chosen].widths = [(0.0, *new.tolist())]
        start_widths[si][chosen] = float(new[0])
        end_widths[si][chosen] = float(new[0] + new[1] * length
                                       + new[2] * length ** 2 + new[3] * length ** 3)
        dq = delta[0] + delta[1] * q + delta[2] * q ** 2 + delta[3] * q ** 3
        correction_max = max(correction_max, float(np.max(np.abs(dq))))
        if float(np.max(np.abs(dq))) > 0.05 and lanes[chosen].provenance is not None:
            provenance = lanes[chosen].provenance
            if provenance.get("eligibility") == "comparable":
                provenance["geometry_adjustment"] = "shared-envelope-closure"
    stats[f"{side_name}_envelope_closure_max_m"] = correction_max


def _proj(pts_deg, lat0, lon0):
    pts = np.asarray(pts_deg, dtype=float)
    x = np.radians(pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def _georef(lat0, lon0):
    """本地平面投影的 PROJ 管线（球面 eqc，与 _proj 数学一致）——CRS 硬约束 3。"""
    return (f"+proj=eqc +lat_ts={lat0:.8f} +lat_0={lat0:.8f} +lon_0={lon0:.8f} "
            f"+R={R_EARTH:.0f} +units=m +no_defs")


def _polylen(pts_xy):
    return float(np.linalg.norm(np.diff(pts_xy, axis=0), axis=1).sum()) if len(pts_xy) >= 2 else 0.0


def _densify_polyline(pts_xy, step=0.5):
    """沿原折线分段线性加密；不平滑、不外推，端点与总长保持不变。"""
    pts = np.asarray(pts_xy, float)
    if len(pts) < 2:
        return pts.copy()
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    ss = np.concatenate([[0.0], np.cumsum(seg)])
    if ss[-1] <= 1e-9:
        return pts[:1].copy()
    q = np.arange(0.0, ss[-1], max(float(step), 1e-3))
    q = np.append(q, ss[-1]) if not len(q) or ss[-1] - q[-1] > 1e-9 else q
    return np.column_stack([np.interp(q, ss, pts[:, 0]),
                            np.interp(q, ss, pts[:, 1])])


def _clip_connector_source_to_mouths(points, p0, p1, step=0.25):
    """把图商内部连接线裁到进口/出口车道口部的共同支持域。

    IBD 的路口内 LaneLink 有时向两端延伸进普通 Link，也有时离口部留有空白。
    完整 connecting road 的两端由 ``p0/p1`` 决定；来源折线只应在两口部最近
    投影之间约束形状。返回裁后折线及可审计裁剪量，不做平滑、不改变中段。
    """
    raw = np.asarray(points, float)
    dense = _densify_polyline(raw, step)
    if len(dense) < 3:
        length = _polylen(raw)
        return raw, {"raw_length_m": length, "used_length_m": length,
                     "cropped_start_m": 0.0, "cropped_end_m": 0.0}

    def stations(polyline):
        return np.concatenate([[0.0], np.cumsum(np.linalg.norm(
            np.diff(polyline, axis=0), axis=1))])

    ss = stations(dense)
    i0 = int(np.argmin(np.linalg.norm(dense - np.asarray(p0[:2]), axis=1)))
    i1 = int(np.argmin(np.linalg.norm(dense - np.asarray(p1[:2]), axis=1)))
    if i1 < i0:
        dense = dense[::-1]
        ss = stations(dense)
        i0 = int(np.argmin(np.linalg.norm(dense - np.asarray(p0[:2]), axis=1)))
        i1 = int(np.argmin(np.linalg.norm(dense - np.asarray(p1[:2]), axis=1)))
    if i1 <= i0 + 1:
        length = _polylen(raw)
        return raw, {"raw_length_m": length, "used_length_m": length,
                     "cropped_start_m": 0.0, "cropped_end_m": 0.0,
                     "decision": "not-cropped-projection-order"}
    used = dense[i0:i1 + 1]
    return used, {
        "raw_length_m": float(ss[-1]),
        "used_length_m": float(ss[i1] - ss[i0]),
        "cropped_start_m": float(ss[i0]),
        "cropped_end_m": float(ss[-1] - ss[i1]),
        "start_projection_gap_m": float(np.linalg.norm(
            used[0] - np.asarray(p0[:2]))),
        "end_projection_gap_m": float(np.linalg.norm(
            used[-1] - np.asarray(p1[:2]))),
        "decision": "nearest-mouth-support-domain",
    }


def _source_envelope_axis(points, mouth_points, *, margin_m=3.0, min_keep_m=6.0):
    """候选：共同 junction 入口放在最早真实车道端点之前，不压缩提前转弯。

    仅沿原轴截断/末端切向延伸，不重新拟合、不添加密集输出 geometry。
    caller 须将被划入 junction 的普通车道尾段纳入连接路来源与审计。
    """
    g = np.asarray(points, float)
    ends = np.asarray(mouth_points, float)
    ss = np.r_[0., np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))]
    dv = g[-1] - g[-2]
    dv /= max(float(np.linalg.norm(dv)), 1e-12)
    delta = float(np.min((ends - g[-1]) @ dv)) - float(margin_m)
    station = float(ss[-1]) + delta
    if station < min_keep_m:
        raise ValueError("source-envelope mouth would remove the seed road: "
                         f"station={station:.3f}, min_keep={min_keep_m:.3f}, "
                         f"axis_length={ss[-1]:.3f}, delta={delta:.3f}")
    if delta > 0:
        result = np.vstack([g, g[-1] + delta * dv])
    else:
        tail = np.array([np.interp(station, ss, g[:, k]) for k in range(2)])
        result = np.vstack([g[ss < station - 1e-8], tail])
    return result, {"axis_shift_m": delta, "margin_m": float(margin_m),
                    "decision": "earliest-source-mouth-envelope"}


def _topological_connector_source(via, incoming, outgoing, *, join_tol_m=0.5):
    """构造 incoming→via→outgoing 的真实来源链；不虚构端点间直线。

    via 保留行车方向；前后 lane 依据真实拓扑传入，仅允许几何端点闭合。
    不闭合时拒绝组合，由调用侧保留原 via 并报告。
    """
    mid = np.asarray(via, float)
    before = np.asarray(incoming, float)
    after = np.asarray(outgoing, float)
    if np.linalg.norm(before[0]-mid[0]) < np.linalg.norm(before[-1]-mid[0]):
        before = before[::-1]
    if np.linalg.norm(after[-1]-mid[-1]) < np.linalg.norm(after[0]-mid[-1]):
        after = after[::-1]
    gaps = [float(np.linalg.norm(before[-1]-mid[0])),
            float(np.linalg.norm(mid[-1]-after[0]))]
    if max(gaps) > join_tol_m:
        return None, {"raw_join_gaps_m": gaps, "decision": "raw-topology-gap"}
    full = np.vstack([before, mid, after])
    keep = np.r_[True, np.linalg.norm(np.diff(full, axis=0), axis=1) > 1e-8]
    return full[keep], {"raw_join_gaps_m": gaps,
                        "raw_via_length_m": _polylen(mid),
                        "decision": "source-topology-composite"}


def _center_of(src: IbdSource, pid: str):
    g = src.roadcenters.get(pid)
    if g is None or g.shape[0] < 2:                      # 缺中心线：中间车道近似
        lanes = [l for l in src.lanes_of(pid) if l.geometry.shape[0] >= 2]
        if not lanes:
            return None
        g = lanes[len(lanes) // 2].geometry
    return np.asarray(g, float)


def _chained_links(src: IbdSource, seed_pid: str, is_enter: bool, proj, cj,
                   max_len: float = 160.0, hops: int = 5):
    """同名 ROADLINK 端点拼链（**跨车道数**——车道数变化由多 laneSection 表达）。
    返回 [(link_pid, center_lonlat)]，顺序=行车方向（enter: 上游→路口；leave: 路口→下游）。"""
    g0 = _center_of(src, seed_pid)
    if g0 is None:
        return None
    if (np.linalg.norm(proj(g0[:1])[0] - cj) < np.linalg.norm(proj(g0[-1:])[0] - cj)) == is_enter:
        g0 = g0[::-1]
    chain, used = [(seed_pid, g0)], {seed_pid}
    rl0 = src.roadlinks[seed_pid]
    total = _polylen(proj(g0))
    for _ in range(hops):
        if total >= max_len:
            break
        far = chain[0][1][0] if is_enter else chain[-1][1][-1]
        best = None
        for pid, c in src.roadcenters.items():
            if pid in used:
                continue
            rl = src.roadlinks.get(pid)
            if rl is None or rl.name != rl0.name:
                continue
            if not [l for l in src.lanes_of(pid) if l.geometry.shape[0] >= 2]:
                continue                                 # 无车道数据的段建不了 section
            d0 = float(np.linalg.norm(c[0] - far))
            d1 = float(np.linalg.norm(c[-1] - far))
            if min(d0, d1) < _DEG_EPS:
                cand = (c if d1 <= d0 else c[::-1]) if is_enter else (c if d0 <= d1 else c[::-1])
                if _join_angle(proj, chain, cand, is_enter) > _CHAIN_TURN:
                    continue                             # 同名但拐了街角：leg 到此为止
                if _corridor_turn(proj(cand)) > _CHAIN_TURN:
                    continue                             # 候选 link 自身已绕街角：整段不拼，绝不裁点
                best = (pid, cand) if best is None else "AMBIG"
        if best is None or best == "AMBIG":              # 无候选或分叉即停
            break
        pid, cand = best
        used.add(pid)
        chain.insert(0, (pid, cand)) if is_enter else chain.append((pid, cand))
        total += _polylen(proj(cand))
    return chain


_CHAIN_TURN = math.radians(50.0)     # 拼链接点最大转角：超过即认定"拐了街角"


def _corridor_turn(gxy) -> float:
    """单个 ROADLINK 相对起始行车方向的最大转角（rad）。

    leg 只沿当前进出口走廊延长；若下一个完整 ROADLINK 已经绕过街角，应停在
    link 边界。这里按完整对象取舍，不删除对象内部的“坏点”，从而保持来源边界
    与 laneSection provenance 一致。
    """
    g = np.asarray(gxy, float)
    if g.shape[0] < 3:
        return 0.0
    seg = np.diff(g, axis=0)
    keep = np.linalg.norm(seg, axis=1) > 0.05            # 忽略厘米级重复点的无意义航向
    if keep.sum() < 2:
        return 0.0
    hdg = np.unwrap(np.arctan2(seg[keep, 1], seg[keep, 0]))
    return float(np.max(np.abs(hdg - hdg[0])))


def _join_angle(proj, chain, cand, is_enter) -> float:
    """拼链接点处的方向变化角（rad）。同名路可能绕过街角（金玥路辅路实测 R=3m 发夹），
    leg 是进出口走廊而非整条街——转角超限即停止拼接，从源头掐掉发夹弯。"""
    cur = proj(chain[0][1] if is_enter else chain[-1][1])
    nxt = proj(cand)
    if cur.shape[0] < 2 or nxt.shape[0] < 2:
        return 0.0

    def _endpoint_tangent(g, at_start: bool):
        """取连接端的行车方向切向，跳过端部重复点。"""
        if at_start:
            anchor = g[0]
            for p in g[1:]:
                v = p - anchor
                if np.linalg.norm(v) > 1e-6:
                    return v
        else:
            anchor = g[-1]
            for p in g[-2::-1]:
                v = anchor - p
                if np.linalg.norm(v) > 1e-6:
                    return v
        return np.zeros(2, dtype=float)

    if is_enter:
        # cand 插到链首：cand[-1] → cur[0]，比较 cand 末端与 cur 起端切向。
        v_before = _endpoint_tangent(nxt, at_start=False)
        v_after = _endpoint_tangent(cur, at_start=True)
    else:
        # cand 追加到链尾：cur[-1] → cand[0]，比较 cur 末端与 cand 起端切向。
        v_before = _endpoint_tangent(cur, at_start=False)
        v_after = _endpoint_tangent(nxt, at_start=True)
    n1, n2 = np.linalg.norm(v_before), np.linalg.norm(v_after)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = float(np.dot(v_before, v_after) / (n1 * n2))
    return math.acos(max(-1.0, min(1.0, cosv)))


def _fit_ref(pts_xy, resample_step=2.0, min_seg_len=6.0):
    return fit_polyline_auto(pts_xy, resample_step, min_seg_len)   # 升级档拟合（refline_fit 共用）


def _shift(pose, t):
    x, y, h = pose
    return (x - t * math.sin(h), y + t * math.cos(h), h)


def _lane_profile_pose(ref_pose, t, dt, d2t, ref_kappa=0.0,
                       ref_sharpness=0.0, reverse=False):
    """参考线横移轨迹 ``r(s)+t(s)N(s)`` 的车道中心 G2 状态。

    只平移参考线位姿会漏掉展宽/收口产生的 ``dt/ds``，从而让连接路在端部
    被迫快速修正 1--3 度航向。这里按平行曲线微分关系同时求位置、切向和
    有符号曲率；出口车道沿 ``s`` 反向行驶时再统一反转航向与曲率。
    """
    x, y, heading = map(float, ref_pose[:3])
    t, dt, d2t = float(t), float(dt), float(d2t)
    kappa, sharp = float(ref_kappa), float(ref_sharpness)
    a = 1.0 - t * kappa
    b = dt
    speed = math.hypot(a, b)
    if speed <= 1e-8:
        raise ValueError("lane offset creates a singular centerline tangent")
    lane_heading = heading + math.atan2(b, a)
    da = -dt * kappa - t * sharp
    db = d2t
    lane_kappa = (kappa + (a * db - b * da) / (a * a + b * b)) / speed
    if reverse:
        lane_heading += math.pi
        lane_kappa = -lane_kappa
    return (x - t * math.sin(heading), y + t * math.cos(heading),
            lane_heading, lane_kappa)


def _width_kinematics(lane, station):
    """求 writer ``Lane`` 在 laneSection 局部站的 width 及一、二阶导。"""
    records = sorted(lane.widths, key=lambda item: float(item[0]))
    if not records:
        return 0.0, 0.0, 0.0
    active = records[0]
    for item in records[1:]:
        if float(item[0]) <= station + 1e-10:
            active = item
        else:
            break
    offset, a, b, c, d = map(float, active)
    u = float(station) - offset
    return (a + b * u + c * u * u + d * u ** 3,
            b + 2.0 * c * u + 3.0 * d * u * u,
            2.0 * c + 6.0 * d * u)


def _g2_prims(p0, p1, *, with_meta=False):
    """两端位姿 → 单条 G2 回旋链（SolveG2 三段）的几何原语；病态解返回 None。"""
    from mapforge.ops.refline_fit import solve_g2_balanced
    try:
        k0 = float(p0[3]) if len(p0) > 3 else 0.0
        k1 = float(p1[3]) if len(p1) > 3 else 0.0
        cls, tuning = solve_g2_balanced(p0, p1, k0, k1)
    except Exception:
        return None
    if any(max(abs(c.KappaStart), abs(c.KappaEnd)) > 0.5 for c in cls):
        return None                                      # 猪尾巴（R<2m 回环）
    prims = [("spiral", c.XStart, c.YStart, c.ThetaStart, c.length,
              c.KappaStart, c.KappaEnd) for c in cls]
    return (prims, tuning) if with_meta else prims


def _prims_fidelity(prims, pts):
    """几何原语链对来源折线的双向与端点误差。"""
    from mapforge.ops.refline_fit import PlanSeg, PlanView
    segs = [PlanSeg("spiral" if p[0] == "spiral" else p[0], p[4], p[5],
                    p[6] if p[0] == "spiral" else None) for p in prims]
    ref = eval_planview(PlanView(prims[0][1], prims[0][2], prims[0][3], segs), 0.5)
    src = np.asarray(pts, float)
    src_d = np.linalg.norm(np.diff(src, axis=0), axis=1)
    src_s = np.concatenate([[0.0], np.cumsum(src_d)])
    sample_s = np.arange(0.0, src_s[-1], 0.5)
    if not len(sample_s) or src_s[-1] - sample_s[-1] > 1e-9:
        sample_s = np.append(sample_s, src_s[-1])
    src = np.column_stack([np.interp(sample_s, src_s, src[:, 0]),
                           np.interp(sample_s, src_s, src[:, 1])])
    s2t = max(np.min(np.linalg.norm(ref - q[None, :], axis=1)) for q in src)
    t2s = max(np.min(np.linalg.norm(src - q[None, :], axis=1)) for q in ref)
    return {"source_to_target_max_m": float(s2t),
            "target_to_source_max_m": float(t2s),
            "start_m": float(np.linalg.norm(src[0] - ref[0])),
            "end_m": float(np.linalg.norm(src[-1] - ref[-1]))}


def _bridged_geoms(vg, p0, p1, fit_fn, scale=1.0):
    """路口内车道实测折线 + 两端 G2 桥：起点精确接进口车道模型位姿 p0、
    末端精确接出口车道模型位姿 p1（换乘跳变构造性归零）。中段保留实测几何。
    守卫：桥长 >40m 或桥内 |κ|>0.5（R<2m"猪尾巴"回环，位姿近退化时 SolveG2
    的病态解）即拒绝——调用侧按 scale 放大切口重试。返回 (prims, dev) 或 None。"""
    from pyclothoids import SolveG2
    d = np.linalg.norm(np.diff(vg, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    L = float(s[-1])
    dense_s = np.arange(0.0, L, 0.5)
    if not len(dense_s) or L - dense_s[-1] > 1e-9:
        dense_s = np.append(dense_s, L)
    vg = np.column_stack([np.interp(dense_s, s, vg[:, 0]),
                          np.interp(dense_s, s, vg[:, 1])])
    s = dense_s
    # 切口下限 8m：SolveG2 桥是 3 段回旋线，切口太小会把桥压成 1–2m 的碎段
    # （消费端读到高频曲率锯齿）；8m 切口 ⇒ 桥段 ≈2.5–4m，仍是合理的缓和过渡长度
    cut_fraction = min(0.45, 1.0 / 3.0 + 0.06 * max(scale - 1.0, 0.0))
    cut0 = min(max(8.0, 2.0 * float(np.linalg.norm(vg[0] - p0[:2]))) * scale,
               L * cut_fraction)
    cut1 = min(max(8.0, 2.0 * float(np.linalg.norm(vg[-1] - p1[:2]))) * scale,
               L * cut_fraction)
    mid = vg[(s >= cut0) & (s <= L - cut1)]
    if mid.shape[0] < 4:
        return None
    pv, dev = fit_fn(mid)
    got = simplify_planview(pv, mid, dev_tol=min(0.55, max(0.4, dev * 1.5)),
                            min_seg_len=12.0)
    if got is not None:                                  # 中段曲率域精简（消碎段）
        pv, dev = got[0], got[1]
        pv = weld_g2(pv)                                 # 残差焊平（不增段）
    else:
        pv = weld_g2(g2ify_planview(pv)[0])              # 未精简：插过渡段 + 焊平兜底
    prims_mid, ep = planview_prims(pv)
    try:
        b0 = SolveG2(p0[0], p0[1], p0[2], 0.0,
                     pv.x0, pv.y0, pv.hdg, seg_kappa(pv, at_end=False))
        b1 = SolveG2(ep[0], ep[1], ep[2], seg_kappa(pv, at_end=True),
                     p1[0], p1[1], p1[2], 0.0)
    except Exception:
        return None
    bridge = list(b0) + list(b1)
    if (sum(c.length for c in b0) > 40 or sum(c.length for c in b1) > 40
            or any(max(abs(c.KappaStart), abs(c.KappaEnd)) > 0.5 for c in bridge)):
        return None                                      # 桥失控/猪尾巴：由调用侧重试
    prims0 = [("spiral", c.XStart, c.YStart, c.ThetaStart, c.length,
               c.KappaStart, c.KappaEnd) for c in b0]
    prims1 = [("spiral", c.XStart, c.YStart, c.ThetaStart, c.length,
               c.KappaStart, c.KappaEnd) for c in b1]
    support_s = (sum(p[4] for p in prims0),
                 sum(p[4] for p in prims0) + sum(p[4] for p in prims_mid))
    return prims0 + prims_mid + prims1, dev, mid, support_s


def _pair_legs(src: IbdSource, junc: JunctionRec, proj, cj):
    """进/出口链按路口侧端点+方向聚合成物理道路腿。

    一个物理方向可能被图商拆成多条并行 ROADLINK（node13 南口即为两个
    2-lane 出口组）。旧的一对一贪心会把第二组写成独立单边 road，形成道路
    断裂。这里改为“每条出口选择最相符的进口腿”，允许一条腿聚合多个出口
    Link。返回 ``([(enter_pid, [leave_pid, ...])], unpaired_leaves)``。
    """
    def seed_info(pid, is_enter):
        g = _center_of(src, pid)
        if g is None or g.shape[0] < 2:
            return None
        gx = proj(g)
        if (np.linalg.norm(gx[0] - cj) < np.linalg.norm(gx[-1] - cj)) == is_enter:
            gx = gx[::-1]                                # 统一为行车方向
        if is_enter:
            pt, dv = gx[-1], gx[-1] - gx[-2]
        else:
            pt, dv = gx[0], gx[1] - gx[0]
        n = np.linalg.norm(dv)
        return (pt, dv / n) if n > 1e-9 else None

    enters = {p: seed_info(p, True) for p in junc.enter_roads}
    leaves = {p: seed_info(p, False) for p in junc.leave_roads}
    leave_of = {e: [] for e in junc.enter_roads}
    used_l = set()
    for l, li in leaves.items():
        if li is None:
            continue
        cands = []
        for e, ei in enters.items():
            if ei is None:
                continue
            dot = float(np.dot(ei[1], li[1]))
            if dot > -0.3:                              # 同腿的对向：方向必须相反
                continue
            dist = float(np.linalg.norm(ei[0] - li[0]))
            # 距离是主判据；轻微航向误差只用于同距离消歧。
            cands.append((dist + 5.0 * (1.0 + dot), dist, e))
        if not cands:
            continue
        _score, dist, e = min(cands)
        if dist > 45.0:
            continue
        leave_of[e].append(l)
        used_l.add(l)
    pairs = [(e, leave_of.get(e, []))
             for e in junc.enter_roads if enters.get(e) is not None]
    single = [l for l in junc.leave_roads if l not in used_l]
    return pairs, single


def build_junction_xodr(src: IbdSource, junc: JunctionRec, out_path: str | Path,
                        *, max_len: float = 160.0, connect_mode: str = "data",
                        allow_uturn: bool = False,
                        mouth_policy: str = "legacy-max-endpoint",
                        mouth_margin_m: float = 3.0) -> dict:
    """IBD 路口 → 完整 OpenDRIVE（双侧 leg road + junction 连接路 + laneLink）。

    connect_mode：data=仅数据 TOPO（默认，不发明拓扑）/ default=补无出口车道 /
    full=全连接（治 TOPO 整片缺录，同时把路口铺满行车带）——见 ops/junction_fill。"""
    if mouth_policy not in {"legacy-max-endpoint", "source-envelope-candidate"}:
        raise ValueError(f"unknown mouth_policy: {mouth_policy}")
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: _proj(p, lat0, lon0)                # noqa: E731
    cj = proj(junc.polygon).mean(axis=0)
    doc = W.XodrDoc(f"ibd_{junc.pid[-8:]}", geo_reference=_georef(lat0, lon0))
    JID = 1

    stats = {"roads_enter": 0, "roads_leave": 0, "legs_two_way": 0, "sections": 0,
             "multi_section_roads": 0, "conn_via": 0, "conn_g2": 0, "connections": 0,
             "lanelinks": 0, "skipped": 0, "fit_dev_max": 0.0, "speeds": 0,
             "junction": junc.name, "junction_pid": junc.pid,
             "mouth_policy": mouth_policy}
    manifest_lanes: dict[str, dict] = {}
    pending_sources: dict[str, dict] = {}
    source_profile = getattr(src, "p", {}).get("profile", "ibd-smarteditor-v1")

    def _register_source(rec, points, *, role, direction, at_stopline=False, via=False,
                         eligible=True, support_reason="target-leg-domain"):
        points = np.asarray(points, float)
        if rec.lane_pid in manifest_lanes:
            old = manifest_lanes[rec.lane_pid]
            if (old["role"] != role or old["travel"]["target_direction"] != direction
                    or old["comparison"]["eligible"] != bool(eligible)):
                raise ValueError(f"SHP source key 冲突: {rec.lane_pid}")
            prev = np.asarray(old["geometry"]["coordinates"], float)
            parts = (prev, points) if direction == "with_s" else (points, prev)
            joined = np.vstack(parts)
            keep = np.concatenate([[True], np.linalg.norm(np.diff(joined, axis=0), axis=1) > 1e-6])
            joined = joined[keep]
            old["geometry"]["coordinates"] = joined.tolist()
            old["geometry"]["geometry_sha256"] = geometry_sha256(joined)
            old["travel"]["start"] = joined[0].tolist()
            old["travel"]["end"] = joined[-1].tolist()
            old.setdefault("support", {})["compared_length_m"] = (
                _polylen(joined) if eligible else 0.0)
            return
        geom_source = getattr(rec, "geometry_source", "field")
        derived = geom_source != "field"
        support = "boundary" if geom_source == "boundaries" else "field"
        policy_class = (f"shp.{support}-via" if via else
                        f"shp.{support}-approach" if at_stopline else
                        f"shp.{support}-leg")
        stop = {"availability": "not-applicable"}
        if at_stopline:
            candidates = [x for x in getattr(src, "stoplines_by_lane", {}).get(rec.lane_pid, [])
                          if x.geometry.shape[0] >= 2]
            if candidates:
                end = np.asarray(points)[-1]
                chosen = min(candidates,
                             key=lambda x: np.linalg.norm(proj(x.geometry).mean(axis=0) - end))
                stop = {"availability": "available", "source_id": chosen.object_pid,
                        "geometry": {"type": "LineString",
                                     "coordinates": proj(chosen.geometry).tolist()}}
            else:
                stop = {"availability": "unavailable", "reason": "source-stopline-not-linked"}
        entry = source_lane(
            rec.lane_pid, points,
            owner={"format": "shp", "junction": junc.pid, "link": rec.link_pid,
                   "lane": rec.lane_pid},
            role=role, status="APPROXIMATED" if derived else "TRANSFORMED",
            support_kind=f"shp-{support}-centerline", policy_class=policy_class,
            travel_direction=direction, eligible=eligible, stop_line=stop,
        )
        entry["support"] = {
            "full_source_length_m": _polylen(proj(rec.geometry)),
            "compared_length_m": _polylen(points) if eligible else 0.0,
            "reason": support_reason,
        }
        manifest_lanes[rec.lane_pid] = entry

    rid_of, xid_of = {}, {}                              # 均以 link_pid 为键
    exit_contact = {}                                    # leave_pid → "start"|"end"
    end_pose, lane_end, lane_start = {}, {}, {}

    # ---------------------------------------------------------------- 侧向机械
    def _span_recs(pid, ref, tang, u0, u1, seed_tail):
        """一个源 Link 的车道实测：横向偏移 d + 数据首末宽 + **全程横距轮廓**
        （lg 每点投影到参考线的 (s, d) 序列——边缘贴合实测形状的原料）。"""
        ref_s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))])
        i0 = min(int(np.searchsorted(ref_s, u0, side="left")), len(ref) - 2)
        i1 = min(int(np.searchsorted(ref_s, u1, side="right")) + 1, len(ref))
        i1 = max(i1, i0 + 2)
        rwin, twin = ref[i0:i1], tang[i0:i1]
        recs = []

        def _project_sd(points_xy):
            """点列连续投影到参考线段，避免最近采样点造成 s 量化与端点堆叠。"""
            points_xy = np.asarray(points_xy, float)
            a, vec = ref[:-1], np.diff(ref, axis=0)
            length = np.linalg.norm(vec, axis=1)
            unit = vec / (length[:, None] + 1e-12)
            rel = points_xy[:, None, :] - a[None, :, :]
            raw = np.sum(rel * vec[None, :, :], axis=2) / (
                np.sum(vec * vec, axis=1)[None, :] + 1e-12)
            clipped = np.clip(raw, 0.0, 1.0)
            foot = a[None, :, :] + clipped[:, :, None] * vec[None, :, :]
            dist2 = np.sum((points_xy[:, None, :] - foot) ** 2, axis=2)
            gi = np.argmin(dist2, axis=1)
            rows = np.arange(len(points_xy))
            tt = clipped[rows, gi]
            rr = raw[rows, gi]
            q = foot[rows, gi]
            uv = unit[gi]
            ss = ref_s[gi] + tt * length[gi]
            dd = uv[:, 0] * (points_xy[:, 1] - q[:, 1]) \
                - uv[:, 1] * (points_xy[:, 0] - q[:, 0])
            inside = ~((gi == 0) & (rr * length[gi] < -1.5))
            inside &= ~((gi == len(vec) - 1) & ((rr - 1.0) * length[gi] > 1.5))
            return ss, dd, inside

        def _boundary_profile(points_deg):
            """真实 SHP 边界折线 → 当前 road 参考线上的 (s,d) 轮廓。"""
            bg = proj(points_deg)
            sparse = len(bg) <= 5
            if sparse:
                bg = _densify_polyline(bg, 0.5)
            bs, bd, inside = _project_sd(bg)
            if inside.sum() < 2:
                return None
            order = np.argsort(bs)
            support_order = np.argsort(bs[inside])
            return {
                "ps": bs[order].astype(float), "pd": bd[order].astype(float),
                "support_ps": bs[inside][support_order].astype(float),
                "support_pd": bd[inside][support_order].astype(float),
                "profile_interpolate": sparse,
            }

        for l in [x for x in src.lanes_of(pid) if x.geometry.shape[0] >= 2]:
            lg = proj(l.geometry)
            flip = float(np.dot(lg[-1] - lg[0], rwin[-1] - rwin[0])) < 0
            if flip:
                lg = lg[::-1]
            sparse_profile = len(lg) <= 5
            if sparse_profile:
                # 2--5 点来源在世界坐标中是分段直线；相对弯曲参考线的 d(s)
                # 并不线性。先沿原折线加密再投影，才能在不平滑源几何的前提下
                # 重建这条直线，而不是用两个横距端值画出一条平行弧。
                lg = _densify_polyline(lg, 0.5)
            sw, ew = _source_endpoint_widths(l)
            if flip:
                sw, ew = ew, sw
            lg_m = lg[int(lg.shape[0] * 0.7):] if seed_tail else lg
            if lg_m.shape[0] < 2:
                lg_m = lg
            sub = lg_m[:: max(1, lg_m.shape[0] // 25)]
            idx = np.argmin(np.linalg.norm(rwin[None, :, :] - sub[:, None, :], axis=2), axis=1)
            d = float(np.median(twin[idx, 0] * (sub[:, 1] - rwin[idx, 1])
                                - twin[idx, 1] * (sub[:, 0] - rwin[idx, 0])))
            # 全程轮廓：逐点投影（用全参考线，防 span 窗截断）
            base_s, dd, inside = _project_sd(lg)
            support_s = base_s[inside]
            support_lg = lg[inside]
            support_pd = dd[inside]
            order = np.argsort(base_s)
            support_order = np.argsort(support_s)
            get_boundaries = getattr(src, "lane_boundary_geometries", None)
            boundary_profiles = []
            if get_boundaries is not None:
                for boundary in get_boundaries(l.lane_pid):
                    profile = _boundary_profile(boundary)
                    if profile is not None:
                        boundary_profiles.append(profile)
            recs.append({"d": d, "l": l, "sw": sw / 1000.0, "ew": ew / 1000.0,
                         "lg": lg[order], "ps": base_s[order], "pd": dd[order],
                         "support_lg": support_lg[support_order],
                         "support_ps": support_s[support_order].astype(float),
                         "support_pd": support_pd[support_order].astype(float),
                         "profile_interpolate": sparse_profile,
                         "boundary_profiles": boundary_profiles})
        return recs

    def _prof_eval(rec, u, win=10.0):
        """轮廓在 s=u 处的 (值, 斜率)：±win 窗**局部线性回归**在 u 点取值——
        中位数在扇形段有一阶偏差（span 两侧窗互不重叠会撕开边界），回归无此偏差。"""
        # 与 manifest/G8 使用同一观测域。端点纵向越界点会全部投影到参考线
        # 首/末 s；若继续参与轮廓回归，会在它们已从来源支持域排除后仍把目标
        # laneOffset/width 横向推偏，形成不可审计的“幽灵影响”。
        ps = rec.get("support_ps", rec["ps"])
        pd = rec.get("support_pd", rec["pd"])
        ups = np.unique(ps)
        if len(ups) < 2:
            return float(np.median(pd)), 0.0
        # 同一参考线采样站可能吸附多个 SHP 点，先取站内中位值去重，再按 s
        # 分段插值。旧实现用 +/-10m 局部回归，会把展宽、收口和斜停止线端点
        # 向邻域均值拉动数米，产生“平滑但不像源地图”的边缘。
        upd = np.asarray([np.median(pd[np.isclose(ps, value)]) for value in ups])
        value = float(np.interp(u, ups, upd))
        if u <= ups[0] or u >= ups[-1]:
            return value, 0.0
        span = float(ups[-1] - ups[0])
        h = min(2.0, max(0.25, span / 20.0))
        lo, hi = max(float(ups[0]), u - h), min(float(ups[-1]), u + h)
        slope = ((float(np.interp(hi, ups, upd)) - float(np.interp(lo, ups, upd)))
                 / max(hi - lo, 1e-9))
        return value, float(min(max(slope, -0.25), 0.25))

    def _clip_support(rec, a, b):
        """源 lane 仅保留本目标 span 实际表示的沿程部分；区外长度进入 support 记账。"""
        ps = np.asarray(rec.get("support_ps", rec["ps"]), float)
        lg = np.asarray(rec.get("support_lg", rec["lg"]), float)
        if len(ps) < 2:
            return lg, None
        ps, ui = np.unique(ps, return_index=True)
        lg = lg[ui]
        lo, hi = max(float(a), float(ps[0])), min(float(b), float(ps[-1]))
        if hi <= lo + 1e-6:
            return np.zeros((0, 2)), None
        mid = ps[(ps > lo + 1e-9) & (ps < hi - 1e-9)]
        u = np.concatenate([[lo], mid, [hi]])
        points = np.column_stack([np.interp(u, ps, lg[:, 0]), np.interp(u, ps, lg[:, 1])])
        return points, (lo, hi)

    def _side_specs(spans, sections, sign):
        """逐合并 section 取本侧活动 span 的车道，宽度按 span 内数据斜坡插值到 section 端点。"""
        groups = sorted({s.get("group", 0) for s in spans})
        out = []
        for (u0, u1) in sections:
            active = []
            for group in groups:
                candidates = [s for s in spans if s.get("group", 0) == group]
                sp = next((s for s in candidates
                           if s["s0"] - 1e-3 <= u0 < s["s1"] - 1e-3 or
                           (u0 >= s["s1"] - 1e-3 and u1 <= s["s1"] + 1e-3)), None)
                if sp is not None:
                    active.append(sp)
            if not active:
                out.append(None)
                continue
            lanes = []
            rows = [(r, sp) for sp in active for r in sp["recs"]]
            # 顺序由 OpenDRIVE 侧别决定，不由 d 的正负决定。参考线可位于车道组
            # 内部，laneOffset 也可使右侧车道的 d 全为正；用 d 符号猜侧别会把
            # 右侧栈倒置，造成十几米中隔与蛇形宽度。
            rows.sort(key=lambda item: item[0]["d"], reverse=(sign < 0))
            for r, sp in rows:
                a, b = sp["s0"], sp["s1"]
                f0 = min(max((u0 - a) / max(b - a, 1e-6), 0.0), 1.0)
                f1 = min(max((u1 - a) / max(b - a, 1e-6), 0.0), 1.0)
                v0, m0 = _prof_eval(r, u0)
                v1, m1 = _prof_eval(r, u1)
                support_lg, support_s = _clip_support(r, u0, u1)
                lanes.append({"d": r["d"], "l": r["l"], "lg": r["lg"],
                              "profile_rec": r,
                              "support_lg": support_lg, "support_s": support_s,
                              "extended": bool(sp.get("extended")),
                              "w0": r["sw"] + (r["ew"] - r["sw"]) * f0,
                              "w1": r["sw"] + (r["ew"] - r["sw"]) * f1,
                              "v0": v0, "v1": v1, "m0": m0, "m1": m1,
                              # _reconcile 会为 C0/C1 改写 v/w；保留来源原值，
                              # 之后才能精确记账被连续化占用的支持域。
                              "source_v0": v0, "source_v1": v1})
            out.append({"pid": (active[0]["pid"] if len(active) == 1
                                else tuple(sp["pid"] for sp in active)),
                        "span": active[0],
                        "span_key": tuple((sp.get("group", 0), id(sp)) for sp in active),
                        "lanes": lanes,
                        "u0": u0, "u1": u1})
        return out

    def _reconcile(specs, matches):
        """交界调和：匹配车道两侧端点的值/斜率/宽度取平均——边界机器级 C0/C1。"""
        for si, mp in enumerate(matches):
            a, b = specs[si], specs[si + 1]
            if a is None or b is None:
                continue
            for ia, ib in mp.items():
                va, vb = a["lanes"][ia], b["lanes"][ib]
                for ka, kb in (("v1", "v0"), ("m1", "m0"), ("w1", "w0")):
                    mid = (va[ka] + vb[kb]) / 2
                    va[ka] = vb[kb] = mid

    boundary_trust_cache = {}

    def _trusted_boundary_lane(lane):
        """整条 source lane 一次性裁决中心/双边界是否自洽。

        判定结果按 source id 缓存，禁止在相邻横断面逐站开关边界证据；后者会把
        本来连续的边界写成二阶导数脉冲。
        """
        sid = lane["l"].lane_pid
        if sid in boundary_trust_cache:
            return boundary_trust_cache[sid]
        rec = lane["profile_rec"]
        profiles = rec.get("boundary_profiles", [])
        if len(profiles) < 2:
            boundary_trust_cache[sid] = False
            return False
        ranges = []
        for item in [rec, *profiles[:2]]:
            ps = np.asarray(item.get("support_ps", item.get("ps", [])), float)
            if len(ps) < 2:
                boundary_trust_cache[sid] = False
                return False
            ranges.append((float(ps.min()), float(ps.max())))
        lo, hi = max(x[0] for x in ranges), min(x[1] for x in ranges)
        if hi - lo < 3.0:
            boundary_trust_cache[sid] = False
            return False
        qs = np.linspace(lo, hi, max(7, min(31, int(hi - lo) + 1)))
        center_error, width_error = [], []
        span = max(hi - lo, 1e-9)
        for q in qs:
            cv = _prof_eval(rec, float(q))[0]
            bv = sorted(_prof_eval(item, float(q))[0] for item in profiles[:2])
            center_error.append(abs(cv - (bv[0] + bv[1]) / 2.0))
            observed_width = bv[1] - bv[0]
            f = min(max((float(q) - lo) / span, 0.0), 1.0)
            expected_width = rec["sw"] + (rec["ew"] - rec["sw"]) * f
            width_error.append(abs(observed_width - expected_width))
        trusted = bool(
            np.median(center_error) <= 0.20
            and np.quantile(center_error, 0.95) <= 0.40
            and np.median(width_error) <= 0.50
            and np.quantile(width_error, 0.95) <= 1.00
        )
        boundary_trust_cache[sid] = trusted
        stats["trusted_boundary_lanes" if trusted else "rejected_boundary_lanes"] = (
            stats.get("trusted_boundary_lanes" if trusted else "rejected_boundary_lanes", 0)
            + 1)
        return trusted

    def _bounds_at(spec, e, sign):
        """一侧某端点的边界数组 (b[0..n], mb[0..n])：实测中心轮廓 → 相邻中点为界，
        外缘 = 端车道中心 ± 半数据宽。sign=-1 右侧（b 递减）、+1 左侧（b[0]=内缘）。"""
        lanes = spec["lanes"]
        v = [(ln["v0"] if e == 0 else ln["v1"]) for ln in lanes]
        mm = [(ln["m0"] if e == 0 else ln["m1"]) for ln in lanes]
        w = [(ln["w0"] if e == 0 else ln["w1"]) for ln in lanes]
        b = [v[0] - sign * w[0] / 2]
        mb = [mm[0]]
        for vk, mk in zip(v, mm):
            b.append(2 * vk - b[-1])
            mb.append(2 * mk - mb[-1])
        widths = [sign * (b[i + 1] - b[i]) for i in range(len(v))]
        if any(width < 0.4 or width > 8.0 for width in widths):
            stats["center_boundary_fallback"] = stats.get("center_boundary_fallback", 0) + 1
            b = [v[0] - sign * w[0] / 2]
            mb = [mm[0]]
            for k in range(1, len(lanes)):
                b.append((v[k - 1] + v[k]) / 2)
                mb.append((mm[k - 1] + mm[k]) / 2)
            b.append(v[-1] + sign * w[-1] / 2)
            mb.append(mm[-1])
        # ProfileSource 可提供 LANE_BOUNDARY 实测折线。优先使用它们重建整个断面，
        # left/right 字段不可信，因此在当前参考线坐标系按 d 大小判内/外侧。
        # 相邻车道共享边界取两者观测均值；若支持域或宽度异常，保留上面的中心线回退。
        u = spec["u0"] if e == 0 else spec["u1"]
        actual = []
        for lane in lanes:
            candidates = []
            for profile in lane["profile_rec"].get("boundary_profiles", []):
                ps = np.asarray(profile.get("support_ps", []), float)
                if len(ps) < 2 or u < float(ps.min()) - 2.0 or u > float(ps.max()) + 2.0:
                    continue
                candidates.append(_prof_eval(profile, u))
            if len(candidates) < 2:
                actual.append(None)
                continue
            candidates.sort(key=lambda item: item[0])
            if sign < 0:                                # 右侧：大 d 为内缘，小 d 为外缘
                inner, outer = candidates[-1], candidates[0]
            else:                                       # 左侧：小 d 为内缘，大 d 为外缘
                inner, outer = candidates[0], candidates[-1]
            width = sign * (outer[0] - inner[0])
            actual.append((inner, outer) if 0.4 <= width <= 8.0 else None)

        if sum(x is not None for x in actual) >= max(1, len(lanes) - 1):
            # 边界 Profile 下，LANE_BOUNDARY 是道路形状的直接证据。旧版只锚定
            # 最外缘，再从每条 LANE_LINK 中心递推内部边界；中心线与边界一旦有
            # 局部冲突，误差会逐车道交替传播，渲染成蛇形标线。现在建立真正的
            # 共享边界栈：每条车道的 outer 与相邻车道的 inner 是同一物理边界，
            # 两份观测取稳健中位。中心线仅参与缺测回退和质量报告，不再改写边界。
            buckets = [[] for _ in range(len(lanes) + 1)]
            for k, pair in enumerate(actual):
                if pair is None:
                    continue
                buckets[k].append(pair[0])
                buckets[k + 1].append(pair[1])
            bb, bb_m = list(b), list(mb)
            observed = 0
            # 路线 A（原生 LANE_LINK 中心线）与路线 B（纯 LANE_BOUNDARY）必须
            # 区分证据优先级。路线 A 的中心线直接约束可行驶轨迹，内部边界若逐条
            # 覆盖会把图商 Link 分段噪声重新写成蛇形标线；此时只用实测最内/最外
            # 边界锁定道路包络。路线 B 没有独立中心证据，才消费完整边界栈。
            field_centers = all(
                getattr(lane["l"], "geometry_source", "field") == "field"
                for lane in lanes)
            allowed = {0, len(buckets) - 1} if field_centers else set(range(len(buckets)))
            if field_centers:
                # 不能一刀切丢掉所有内部 LANE_BOUNDARY。若某条 field center 与
                # 自身两边界中线/宽度在本站互相一致，这两条边界就是独立的可靠
                # 证据；忽略它会把堆叠误差全部累积到最外车道（node13 为 0.73m）。
                # 冲突数据（如 node18 中心与边界中线差约 1m）仍不会进入 allowed。
                for k, pair in enumerate(actual):
                    if pair is not None and _trusted_boundary_lane(lanes[k]):
                        allowed.update((k, k + 1))
                        stats["trusted_internal_boundary_endpoints"] = stats.get(
                            "trusted_internal_boundary_endpoints", 0) + 1
            for k, bucket in enumerate(buckets):
                if k not in allowed:
                    continue
                if not bucket:
                    continue
                bb[k] = float(np.median([x[0] for x in bucket]))
                bb_m[k] = float(np.median([x[1] for x in bucket]))
                observed += 1
            widths = [sign * (bb[i + 1] - bb[i]) for i in range(len(lanes))]
            enough = observed >= (1 if field_centers else len(lanes))
            if enough and all(0.25 <= width <= 8.0 for width in widths):
                stats["source_boundary_endpoints"] = stats.get(
                    "source_boundary_endpoints", 0) + 1
                key = ("source_boundary_envelope_endpoints" if field_centers
                       else "source_boundary_stack_endpoints")
                stats[key] = stats.get(key, 0) + 1
                b, mb = bb, bb_m
                spec.setdefault("boundary_anchor", [False, False])[e] = True
        return b, mb

    def _side_match(specs):
        return _match_side_sections(specs, src, stats)

    # ---------------------------------------------------------------- leg 主构建
    def build_leg(e_pid: str, l_pids: list[str] | tuple[str, ...] | None,
                  rid: int) -> bool:
        l_pids = list(l_pids or [])
        chain = _chained_links(src, e_pid, True, proj, cj, max_len=max_len)
        if not chain:
            return False
        pts = chain[0][1]
        for _, c in chain[1:]:
            pts = np.vstack([pts, c[1:]])
        gxy = proj(pts)
        enter_raw_len = _polylen(gxy)
        # —— 路口侧延伸到车道实际端点（ROADCENTER 常铺不到停止线） ——
        seed_lanes = [l for l in src.lanes_of(e_pid) if l.geometry.shape[0] >= 2]
        opposite_seed_lanes = [l for lp in l_pids for l in src.lanes_of(lp)
                               if l.geometry.shape[0] >= 2]
        if seed_lanes:
            dvec = gxy[-1] - gxy[-2]
            dvec = dvec / (np.linalg.norm(dvec) + 1e-12)
            deltas, mouth_points = [], []
            for l in seed_lanes + opposite_seed_lanes:
                lg = proj(l.geometry)
                # 同一 OpenDRIVE road 的两侧共用一个 junction contactPoint。
                # 取离路口中心更近的真实车道端点；只看进口侧会截掉斜停止线
                # 另一侧（node18 三条 1.7m 出口尾段）。
                lp = lg[0] if np.linalg.norm(lg[0] - cj) < np.linalg.norm(lg[-1] - cj) \
                    else lg[-1]
                deltas.append(float(np.dot(lp - gxy[-1], dvec)))
                mouth_points.append(lp)
            delta = min(max(0.0, max(deltas, default=0.0)), 10.0)
            if mouth_policy == "source-envelope-candidate":
                gxy, decision = _source_envelope_axis(
                    gxy, mouth_points, margin_m=mouth_margin_m,
                    # 原始 Link 是语义分段，不等于 planView geometry；不能把
                    # 6m 曲线段下限套在 Link 剩余长度上（node4 会误拦 4.6m span）。
                    min_keep_m=max(6.0, enter_raw_len - _polylen(proj(chain[-1][1])) + 0.5))
                stats.setdefault("mouth_envelope_decisions", []).append({
                    "enter_link": e_pid, "leave_links": l_pids, "road_id":str(rid), **decision})
            elif delta > 0.2:
                gxy = np.vstack([gxy, (gxy[-1] + dvec * delta)[None, :]])
                stats["mouth_extension_max_lane_m"] = max(
                    stats.get("mouth_extension_max_lane_m", 0.0), float(delta))
        enter_with_mouth_len = _polylen(gxy)

        # 同一物理出口可能由多条并行 ROADLINK 组成。若对向真实几何比进口链长，
        # 用最长的对向链补足参考线远端；仅做刚体横移以对齐路口端，不改其曲率。
        # 缺测的进口侧随后按首个真实断面延伸，明确记为 source-extension。
        cached_leave_chains = {}
        prefix_raw_len = 0.0
        if l_pids:
            for lp in l_pids:
                ch = _chained_links(src, lp, False, proj, cj, max_len=max_len)
                if ch:
                    cached_leave_chains[lp] = ch
            if len(l_pids) > 1 and cached_leave_chains:
                candidates = []
                for lp, ch in cached_leave_chains.items():
                    lpnts = ch[0][1]
                    for _pid, c in ch[1:]:
                        lpnts = np.vstack([lpnts, c[1:]])
                    lxy = proj(lpnts)
                    if np.linalg.norm(lxy[0] - cj) > np.linalg.norm(lxy[-1] - cj):
                        lxy = lxy[::-1]                  # 出口行车：路口→远端
                    candidates.append((_polylen(lxy), lp, lxy))
                leave_len, _axis_pid, lxy = max(candidates, key=lambda item: item[0])
                if leave_len > enter_with_mouth_len + 8.0:
                    opposite = lxy[::-1].copy()          # 参考 s：远端→路口
                    opposite += gxy[-1] - opposite[-1]  # 只横移/平移，不扭曲来源轴
                    cut = int(np.argmin(np.linalg.norm(opposite - gxy[0], axis=1)))
                    prefix = opposite[:cut + 1]
                    gap = float(np.linalg.norm(prefix[-1] - gxy[0])) if len(prefix) else 1e9
                    if len(prefix) >= 2 and gap <= 8.0:
                        before = _polylen(gxy)
                        gxy = np.vstack([prefix, gxy])
                        prefix_raw_len = max(0.0, _polylen(gxy) - before)
                        stats["reference_extended_from_opposite_m"] = round(prefix_raw_len, 3)
                        stats["parallel_leave_groups"] = len(l_pids)
        chain_raw = [_polylen(proj(c)) for _, c in chain]
        seg_lens = list(chain_raw)
        # 路口侧可能按真实车道端点延伸，增量只归到种子 link；不得裁掉或按比例
        # 重分来源 span，否则 laneSection 的 provenance 会与原始 ROADLINK 错位。
        seg_lens[-1] += enter_with_mouth_len - enter_raw_len
        try:
            pv, dev, smoothed = fit_leg_refline(gxy)     # 曲率封顶（双侧模型 1−tκ 护栏）
        except ReflineFitError as exc:
            raise ReflineFitError(
                f"SHP junction={junc.pid} enter_link={e_pid} "
                f"leave_links={list(l_pids)} points={len(gxy)}: {exc}"
            ) from exc
        impulse = pv.fit_meta.get("impulse_filter", {})
        if impulse.get("removed_count"):
            stats["refline_outlier_points_removed"] = (
                stats.get("refline_outlier_points_removed", 0)
                + int(impulse["removed_count"]))
            stats["refline_outlier_max_residual_m"] = max(
                stats.get("refline_outlier_max_residual_m", 0.0),
                float(impulse.get("max_residual_m", 0.0)))
        if smoothed:
            stats["refit_smoothed"] = stats.get("refit_smoothed", 0) + 1
        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
        L_fit = sum(s.length for s in pv.segs)
        scale = L_fit / max(prefix_raw_len + sum(seg_lens), 1e-6)
        prefix_fit = prefix_raw_len * scale
        e_bounds = [round(float(prefix_fit + x * scale), 4)
                    for x in np.concatenate([[0.0], np.cumsum(seg_lens)])]
        e_bounds[-1] = round(L_fit, 4)                   # 边界一次圆整（span 与 section 同源）
        ref = eval_planview(pv, 0.5)
        ref_s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))])
        tang = np.gradient(ref, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12

        def s_of(pt):
            return float(ref_s[int(np.argmin(np.linalg.norm(ref - pt[None, :], axis=1)))])

        # —— 进口 span（每源 Link 一段，覆盖 [0, L]） ——
        spans_e = []
        for i, (pid, _c) in enumerate(chain):
            u0, u1 = e_bounds[i], e_bounds[i + 1]
            recs = _span_recs(pid, ref, tang, u0, u1, seed_tail=(i == len(chain) - 1))
            if not recs:
                return False
            recs.sort(key=lambda r: -r["d"])             # 右侧：左→右
            off = recs[0]["d"] + recs[0]["sw"] / 2
            spans_e.append({"pid": pid, "s0": u0, "s1": u1, "recs": recs,
                            "off": off, "extended": False, "group": 0})
        if prefix_fit > 1.0 and spans_e:
            # 缺测区前若紧邻一个短小且含零宽/超宽交叉锥形的 Link，它通常是
            # 图商为车道生灭切出的局部记录，不能被外推几十米。保留来源清单并
            # 明确排除，只让最近的稳定断面参与缺测段重建。
            first = spans_e[0]
            pathological = (
                len(spans_e) > 1 and first["s1"] - first["s0"] < 20.0
                and any(min(r["sw"], r["ew"]) < 0.4 or max(r["sw"], r["ew"]) > 6.0
                        for r in first["recs"])
            )
            if pathological:
                for r in first["recs"]:
                    _register_source(r["l"], r["lg"], role="approach",
                                     direction="with_s", eligible=False,
                                     support_reason="lane-transition-taper")
                stats["source_transition_links_bypassed"] = \
                    stats.get("source_transition_links_bypassed", 0) + 1
                extension_end = first["s1"]
                spans_e = spans_e[1:]
            else:
                extension_end = e_bounds[0]
            # 缺测远端不得复制第一个“车道生灭”碎 Link（node13 的 4-lane Link
            # 含 0→4.75m 与 6.7→0m 两条交叉锥形）。用路口侧稳定断面镜像延伸，
            # 到真实首段再按完整 taper 过渡。
            stable = spans_e[-1]
            spans_e.insert(0, {"pid": stable["pid"], "s0": 0.0, "s1": extension_end,
                               "recs": stable["recs"], "off": stable["off"],
                               "extended": True, "group": 0})

        # —— 对向 span：leave 链边界投影到参考线（吸附/去碎） ——
        # 预算对齐进口参考线长度：对向覆盖不足会在数据尽头留"全幅漏斗"（собранная
        # 车道收拢楔），能用真实数据补就不该用漏斗
        spans_l = []
        if l_pids:
            grouped_spans = []
            for group, l_pid in enumerate(l_pids):
                chain_l = cached_leave_chains.get(l_pid) or _chained_links(
                    src, l_pid, False, proj, cj, max_len=max(max_len, L_fit + 20.0))
                one_group = []
                for pid, c in (chain_l or []):
                    cxy = proj(c)
                    sa, sb = sorted((s_of(cxy[0]), s_of(cxy[-1])))
                    sa, sb = max(sa, 0.0), min(sb, L_fit)
                    if sb - sa < 3.0:
                        continue
                    if L_fit - sb < 8.0:                 # 路口侧放宽吸附：对向幅物理到达路口，
                        sb = L_fit                       # 差值=停止线错位/投影斜量（±数米）
                    for eb in e_bounds:                  # 吸附到进口边界/端点
                        if abs(sa - eb) < _SNAP:
                            sa = eb
                        if abs(sb - eb) < _SNAP:
                            sb = eb
                    recs = _span_recs(pid, ref, tang, sa, sb,
                                      seed_tail=(pid == l_pid))
                    recs = [r for r in recs if r["d"] > 0]  # 左侧车道必在参考线左
                    if not recs:
                        continue
                    recs.sort(key=lambda r: r["d"])      # 左侧：内→外（+1 靠中线）
                    # 内缘两端各测（median 随 s 变化：横摆沿 span 分布而非集中在接缝）；
                    # 端窗取 15%——拼链相邻 Link 端点物理同点，边界处两侧实测自然收敛
                    lg = recs[0]["lg"]
                    n40 = max(2, lg.shape[0] * 3 // 20)
                    i0 = min(int(np.searchsorted(ref_s, sa, side="left")), len(ref) - 2)
                    i1 = min(int(np.searchsorted(ref_s, sb, side="right")) + 1, len(ref))
                    i1 = max(i1, i0 + 2)
                    rwin, twin = ref[i0:i1], tang[i0:i1]

                    def _dwin(part):
                        sub = part[:: max(1, part.shape[0] // 25)]
                        ix = np.argmin(np.linalg.norm(
                            rwin[None, :, :] - sub[:, None, :], axis=2), axis=1)
                        return float(np.median(
                            twin[ix, 0] * (sub[:, 1] - rwin[ix, 1])
                            - twin[ix, 1] * (sub[:, 0] - rwin[ix, 0])))

                    one_group.append({"pid": pid, "s0": sa, "s1": sb,
                                      "recs": recs, "din0": _dwin(lg[:n40]),
                                      "din1": _dwin(lg[-n40:]), "extended": False,
                                      "group": group})
                one_group.sort(key=lambda s: s["s0"])
                for a, b in zip(one_group, one_group[1:]):
                    if abs(b["s0"] - a["s1"]) > 1e-9:
                        mid = (a["s1"] + b["s0"]) / 2
                        a["s1"] = b["s0"] = mid
                one_group = [sp for sp in one_group if sp["s1"] - sp["s0"] > 1.0]
                if one_group:                            # 每个并行组独立补齐，不把重叠组串接
                    far = one_group[0]
                    if far["s0"] > 1.0:
                        stats["left_extended_m"] = round(
                            stats.get("left_extended_m", 0.0) + far["s0"], 1)
                        one_group.insert(0, {"pid": far["pid"], "s0": 0.0,
                                             "s1": far["s0"], "recs": far["recs"],
                                             "din0": far["din0"], "din1": far["din0"],
                                             "extended": True, "group": group})
                    near = one_group[-1]
                    if near["s1"] < L_fit - 1e-6:
                        stats["left_extended_m"] = round(
                            stats.get("left_extended_m", 0.0) + (L_fit - near["s1"]), 1)
                        one_group.append({"pid": near["pid"], "s0": near["s1"],
                                          "s1": L_fit, "recs": near["recs"],
                                          "din0": near["din1"], "din1": near["din1"],
                                          "extended": True, "group": group})
                grouped_spans.extend(one_group)
            spans_l = grouped_spans

        # —— 合并 section 边界（leave 边界 6m 稀疏化：微 section 会把锥形压成陡坡） ——
        bounds = {0.0, *np.round(e_bounds, 4)}
        for sp in spans_l:
            for v in (sp["s0"], sp["s1"]):
                if all(abs(v - b) > 6.0 for b in bounds) and 1.0 < v < L_fit - 1.0:
                    bounds.add(round(float(v), 4))
        # 实际道路外缘可能在一个 20m Link/section 内继续弯曲或展宽。仅在 section
        # 两端取真实边界仍会画出“平滑但不像 SHP”的捷径曲线；有边界证据时按 5m
        # 增加横断面控制站，Hermite 仍保持 C1，但不再跨过真实轮廓细节。
        if any(r.get("boundary_profiles") for sp in spans_e + spans_l for r in sp["recs"]):
            for v in np.arange(_BOUNDARY_SECTION_STEP, L_fit, _BOUNDARY_SECTION_STEP):
                if all(abs(v - b) > 1.0 for b in bounds):
                    bounds.add(round(float(v), 4))
            stats["boundary_section_step_m"] = _BOUNDARY_SECTION_STEP
        coarse = sorted(bounds)
        B = [coarse[0]]
        for a, b in zip(coarse, coarse[1:]):
            n = max(1, int(math.ceil((b - a) / 20.0)))
            B.extend(float(x) for x in np.linspace(a, b, n + 1)[1:])
        sections = [(B[i], B[i + 1]) for i in range(len(B) - 1)
                    if B[i + 1] - B[i] > 0.5]
        rs = _side_specs(spans_e, sections, -1)
        lspec = _side_specs(spans_l, sections, +1)
        m_r = _side_match(rs)
        m_l = _side_match(lspec)
        _reconcile(rs, m_r)
        _reconcile(lspec, m_l)
        _smooth_lane_center_tracks(sections, rs, m_r, stats, "right")
        _smooth_lane_center_tracks(sections, lspec, m_l, stats, "left")
        two_way = any(x is not None for x in lspec)

        # —— 边界轮廓（实测跟踪）：每 section 端点各车道实测横距 → 相邻中点为界 ——
        # 边缘/车道线由此贴合 SHP 实测形状（此前为标量宽度堆叠的合成边缘）
        rbounds = [(_bounds_at(sp, 0, -1), _bounds_at(sp, 1, -1)) if sp else None
                   for sp in rs]
        lbounds = [(_bounds_at(sp, 0, +1), _bounds_at(sp, 1, +1)) if sp else None
                   for sp in lspec]

        def _smooth_side_envelope(bounds0, sign0, side_name):
            """把道路内缘/外缘当成贯穿整条 road 的两条物理轨迹。

            lane 数变化只会使内部边界汇入/分出，不应让道路包络在 source Link
            边界重启。对每个横断面先统一共享站值，再以稳健平滑样条消除局部
            锯齿；内部边界按原横断面归一位置映射到新包络，保持顺序和共享性。
            """
            from scipy.interpolate import CubicSpline, UnivariateSpline

            valid = [i for i, value in enumerate(bounds0) if value is not None]
            if len(valid) < 3 or valid != list(range(valid[0], valid[-1] + 1)):
                return
            i0, i1 = valid[0], valid[-1]
            xs = np.asarray([sections[i0][0]]
                            + [sections[i][1] for i in range(i0, i1 + 1)], float)

            def station_value(k, which):
                values = []
                if k > i0:
                    values.append(bounds0[k - 1][1][0][which])
                if k <= i1:
                    values.append(bounds0[k][0][0][which])
                return float(np.median(values))

            raw_inner = np.asarray([station_value(k, 0)
                                    for k in range(i0, i1 + 2)], float)
            raw_outer = np.asarray([station_value(k, -1)
                                    for k in range(i0, i1 + 2)], float)

            def robust_track(raw):
                weights = np.ones(len(xs), float)
                weights[[0, -1]] = 5.0
                fitted = raw.copy()
                # IRLS：孤立的边界测量毛刺降权；长距离真实展宽由连续多站支持，
                # 不会被当成离群点。0.30m 是形态重建预算，不是来源精度声明。
                for _ in range(4):
                    spl = UnivariateSpline(xs, raw, w=weights,
                                           k=min(3, len(xs) - 1),
                                           s=len(xs) * 0.20 ** 2)
                    fitted = np.asarray(spl(xs), float)
                    residual = np.abs(raw - fitted)
                    robust = np.minimum(1.0, 0.20 / np.maximum(residual, 1e-9))
                    robust[[0, -1]] = 1.0
                    weights = robust
                    weights[[0, -1]] = 5.0
                # RMS 平滑预算不能替代逐站位移上限；跨 Link 的样条不得改写道路。
                fitted = np.clip(fitted, raw - 0.20, raw + 0.20)
                # 物理内/外包络必须跨 laneSection 保持二阶连续。PCHIP 只保证
                # C1，会在车道生灭站产生曲率台阶；这里用自然三次样条给出公共
                # 导数。后面的 `_coupled_safe` 仍负责检查宽度非负与边界不交叉。
                curve = CubicSpline(xs, fitted, bc_type="natural")
                return fitted, np.asarray(curve.derivative()(xs), float)

            inner, inner_m = robust_track(raw_inner)
            outer, outer_m = robust_track(raw_outer)
            total = sign0 * (outer - inner)
            if float(total.min()) < 1.0:
                stats[f"{side_name}_envelope_rejected"] = "non-positive-width"
                return
            stats[f"{side_name}_envelope_max_adjust_m"] = float(max(
                np.max(np.abs(inner - raw_inner)), np.max(np.abs(outer - raw_outer))))

            for si in range(i0, i1 + 1):
                for endpoint, q in ((0, si - i0), (1, si - i0 + 1)):
                    values, slopes = bounds0[si][endpoint]
                    old_inner, old_outer = float(values[0]), float(values[-1])
                    denom = old_outer - old_inner
                    if abs(denom) < 1e-6:
                        continue
                    # 只锁定物理内/外包络；内部共享边界已经由中心线/边界观测联合
                    # 确定，不能再按总宽做仿射缩放，否则中间车道会整体偏 0.8m。
                    new_values = np.asarray(values, float).copy()
                    new_slopes = np.asarray(slopes, float).copy()
                    new_values[0], new_values[-1] = inner[q], outer[q]
                    new_slopes[0], new_slopes[-1] = inner_m[q], outer_m[q]
                    if np.min(sign0 * np.diff(new_values)) < 0.20:
                        # 极端脏断面才退回保序仿射映射；留统计，禁止静默制造负宽。
                        fractions = np.clip(
                            (np.asarray(values, float) - old_inner) / denom, 0.0, 1.0)
                        new_values = inner[q] + fractions * (outer[q] - inner[q])
                        new_slopes = inner_m[q] + fractions * (outer_m[q] - inner_m[q])
                        stats[f"{side_name}_envelope_affine_fallback"] = stats.get(
                            f"{side_name}_envelope_affine_fallback", 0) + 1
                    bounds0[si] = list(bounds0[si])
                    bounds0[si][endpoint] = (new_values.tolist(), new_slopes.tolist())
                    bounds0[si] = tuple(bounds0[si])

        def _smooth_boundary_runs(specs0, matches0, bounds0, sign0):
            """在同一来源断面拓扑内，对共享绝对边界做容差平滑。

            最近距离门禁无法发现曲线在来源两侧来回摆动。这里先用 0.15m RMS
            预算的 smoothing spline 抑制测量/投影噪声，再以 PCHIP 取公共站点
            导数，避免三次 Hermite 过冲。只在 source lane id 序列完全一致的
            连续区间工作，绝不跨 birth/death/merge/split 事件抹平拓扑。
            """
            from scipy.interpolate import PchipInterpolator, UnivariateSpline

            i = 0
            while i < len(bounds0):
                sp = specs0[i]
                if sp is None or bounds0[i] is None:
                    i += 1
                    continue
                j = i + 1
                # source Link 一换，lane_pid 往往全部重编号；真正的物理边界是否连续
                # 应由一对一拓扑映射决定，而不是由字符串 ID 相等决定。仅恒等映射且
                # 车道数不变时跨 Link 平滑，birth/death/merge/split 仍是硬边界。
                while j < len(bounds0):
                    if specs0[j] is None or bounds0[j] is None:
                        break
                    n0 = len(specs0[j - 1]["lanes"])
                    n1 = len(specs0[j]["lanes"])
                    mp = matches0[j - 1]
                    if n0 != n1 or mp != {k: k for k in range(n0)}:
                        break
                    j += 1
                # 一段至少三个 section / 四个站，才有足够自由度分离道路形态与噪声。
                if j - i >= 3:
                    xs = np.asarray([sections[i][0]] + [sections[k][1] for k in range(i, j)],
                                    float)
                    nbound = len(bounds0[i][0][0])
                    raw = np.empty((len(xs), nbound), float)
                    raw[0] = bounds0[i][0][0]
                    for k in range(i, j):
                        raw[k - i + 1] = bounds0[k][1][0]
                    fitted = np.empty_like(raw)
                    deriv = np.empty_like(raw)
                    for bidx in range(nbound):
                        y = raw[:, bidx]
                        # 0.15m 是平面测量到参数化道路的形态预算，不是插值误差目标；
                        # 端点权重提高，避免改变 source Link 的真实接缝位置。
                        weights = np.ones(len(xs), float)
                        weights[[0, -1]] = 4.0
                        spl = UnivariateSpline(xs, y, w=weights, k=min(3, len(xs) - 1),
                                               s=len(xs) * 0.20 ** 2)
                        ys = np.asarray(spl(xs), float)
                        ys = np.clip(ys, y - 0.20, y + 0.20)
                        curve = PchipInterpolator(xs, ys)
                        fitted[:, bidx] = curve(xs)
                        deriv[:, bidx] = curve.derivative()(xs)
                    widths = sign0 * np.diff(fitted, axis=1)
                    if float(widths.min()) >= 0.20:
                        for k in range(i, j):
                            q = k - i
                            bounds0[k] = ((fitted[q].tolist(), deriv[q].tolist()),
                                          (fitted[q + 1].tolist(), deriv[q + 1].tolist()))
                        stats["boundary_smoothed_sections"] = stats.get(
                            "boundary_smoothed_sections", 0) + (j - i)
                        stats["boundary_smoothing_max_adjust_m"] = max(
                            stats.get("boundary_smoothing_max_adjust_m", 0.0),
                            float(np.max(np.abs(fitted - raw))))
                i = j

        _smooth_boundary_runs(rs, m_r, rbounds, -1)
        _smooth_boundary_runs(lspec, m_l, lbounds, +1)
        # 稳定拓扑区的内部边界先降噪，最后再锁定贯穿全 road 的内/外包络；
        # 反序会让局部边界样条重新改写包络端点导数，在拓扑事件处留下折角。
        _smooth_side_envelope(rbounds, -1, "right")
        _smooth_side_envelope(lbounds, +1, "left")

        # laneOffset、median 与各 lane width 都来自同一边界栈 b[i](s)。分别限幅
        # 会破坏它们的代数和，造成车道中心累计横移；因此整条 road 共用一个导数
        # 缩放 alpha，仅在 Hermite 宽度过冲为负时整体收缩，保持 C1 与中心关系。
        boundary_coupled = any(
            sp is not None and any(ln["profile_rec"].get("boundary_profiles")
                                   for ln in sp["lanes"])
            for sp in rs + lspec)

        def _coupled_safe(si, alpha0, alpha1):
            u0, u1 = sections[si]
            length = max(u1 - u0, 1e-3)
            q = np.linspace(0.0, 1.0, 21)
            h00 = 2 * q ** 3 - 3 * q ** 2 + 1
            h10 = q ** 3 - 2 * q ** 2 + q
            h01 = -2 * q ** 3 + 3 * q ** 2
            h11 = q ** 3 - q ** 2

            def curve(v0, m0, v1, m1):
                return (h00 * v0 + h10 * length * alpha0 * m0
                        + h01 * v1 + h11 * length * alpha1 * m1)

            for bounds, sign in ((rbounds[si], -1), (lbounds[si], +1)):
                if bounds is None:
                    continue
                (b0, mb0), (b1, mb1) = bounds
                for k in range(len(b0) - 1):
                    width = sign * (
                        curve(b0[k + 1], mb0[k + 1], b1[k + 1], mb1[k + 1])
                        - curve(b0[k], mb0[k], b1[k], mb1[k]))
                    if float(width.min()) < 0.05:
                        return False
            if rbounds[si] is not None and lbounds[si] is not None:
                (r0, mr0), (r1, mr1) = rbounds[si]
                (l0, ml0), (l1, ml1) = lbounds[si]
                median = (curve(l0[0], ml0[0], l1[0], ml1[0])
                          - curve(r0[0], mr0[0], r1[0], mr1[0]))
                if float(median.min()) < -1e-4:
                    return False
            return True

        if boundary_coupled:
            # 每个横断面站一个公共系数；只有相邻不安全 section 才收缩该站导数，
            # 避免一处异常把整条道路所有导数清零（旧版北向边缘因此被拉成捷径）。
            station_scale = np.ones(len(sections) + 1, float)
            for _ in range(80):
                bad = [si for si in range(len(sections))
                       if not _coupled_safe(si, station_scale[si], station_scale[si + 1])]
                if not bad:
                    break
                for si in bad:
                    station_scale[si] *= 0.8
                    station_scale[si + 1] *= 0.8
            for si in range(len(sections)):
                for item, scale in ((rbounds[si][0] if rbounds[si] else None,
                                     station_scale[si]),
                                    (rbounds[si][1] if rbounds[si] else None,
                                     station_scale[si + 1]),
                                    (lbounds[si][0] if lbounds[si] else None,
                                     station_scale[si]),
                                    (lbounds[si][1] if lbounds[si] else None,
                                     station_scale[si + 1])):
                    if item is not None:
                        _values, slopes = item
                        for k in range(len(slopes)):
                            slopes[k] *= scale
            stats["boundary_slope_scale"] = round(float(station_scale.min()), 9)
            stats["boundary_slope_scale_median"] = round(
                float(np.median(station_scale)), 9)
        # 断面端点自检：边界栈反解后必须仍以来源车道中心为中点。
        center_residual = 0.0
        for specs0, bounds0 in ((rs, rbounds), (lspec, lbounds)):
            for spec0, pair0 in zip(specs0, bounds0):
                if spec0 is None or pair0 is None:
                    continue
                for endpoint, ((values, _slopes), key) in enumerate(
                        zip(pair0, ("v0", "v1"))):
                    for k, lane0 in enumerate(spec0["lanes"]):
                        got = (values[k] + values[k + 1]) / 2.0
                        center_residual = max(center_residual,
                                              abs(got - float(lane0[key])))
        stats["boundary_center_residual_max_m"] = center_residual
        off_vals = []                                    # laneOffset 每 section (y0,m0,y1,m1)
        for rb in rbounds:
            (b0, mb0), (b1, mb1) = rb
            off_vals.append((b0[0], mb0[0], b1[0], mb1[0]))
        for i in range(len(off_vals) - 1):               # 交界硬连续（值取上段末、斜率平均）
            y0a, m0a, y1a, m1a = off_vals[i]
            y0b, m0b, y1b, m1b = off_vals[i + 1]
            mm = (m1a + m0b) / 2
            off_vals[i] = (y0a, m0a, y1a, mm)
            off_vals[i + 1] = (y1a, mm, y1b, m1b)
        if off_vals:
            station_s = [sections[0][0]] + [x[1] for x in sections]
            station_y = [off_vals[0][0]] + [x[2] for x in off_vals]
            station_y, repaired = _regularize_micro_stations(station_s, station_y)
            stats["lane_offset_micro_stations_removed"] = stats.get(
                "lane_offset_micro_stations_removed", 0) + repaired
            station_m = _c2_station_slopes(station_s, station_y)
            off_vals = [(station_y[i], station_m[i],
                         station_y[i + 1], station_m[i + 1])
                        for i in range(len(off_vals))]

        # —— 中央分隔（median）：对向内缘实测 − laneOffset（同源 Hermite，全程 C1） ——
        med_w = []                                       # 每 section: (g0, m0, g1, m1)|None
        for i, lb in enumerate(lbounds):
            if lb is None:
                med_w.append(None)
                continue
            (bl0, mbl0), (bl1, mbl1) = lb
            y0, my0, y1, my1 = off_vals[i]
            g0, mg0 = bl0[0] - y0, mbl0[0] - my0
            g1, mg1 = bl1[0] - y1, mbl1[0] - my1
            if g0 < 0.0:
                g0, mg0 = 0.0, 0.0
            if g1 < 0.0:
                g1, mg1 = 0.0, 0.0
            med_w.append((g0, mg0, g1, mg1))
        for i in range(len(med_w) - 1):                  # 交界硬连续
            if med_w[i] is None or med_w[i + 1] is None:
                continue
            g0a, mg0a, g1a, mg1a = med_w[i]
            g0b, mg0b, g1b, mg1b = med_w[i + 1]
            mm = (mg1a + mg0b) / 2
            med_w[i] = (g0a, mg0a, g1a, mm)
            med_w[i + 1] = (g1a, mm, g1b, mg1b)
        # median 可能只覆盖若干连续 section；每个连续块单独求 C2 样条。
        i = 0
        while i < len(med_w):
            if med_w[i] is None:
                i += 1
                continue
            j = i
            while j + 1 < len(med_w) and med_w[j + 1] is not None:
                j += 1
            xs = [sections[i][0]] + [sections[k][1] for k in range(i, j + 1)]
            ys = [med_w[i][0]] + [med_w[k][2] for k in range(i, j + 1)]
            ys, repaired = _regularize_micro_stations(xs, ys)
            stats["median_micro_stations_removed"] = stats.get(
                "median_micro_stations_removed", 0) + repaired
            ms = _c2_station_slopes(xs, ys,
                                    zero_start=ys[0] <= 1e-6,
                                    zero_end=ys[-1] <= 1e-6)
            for k in range(i, j + 1):
                q = k - i
                med_w[k] = (ys[q], ms[q], ys[q + 1], ms[q + 1])
            i = j + 1
        has_median = any(g is not None and max(g[0], g[2]) > 0.05 for g in med_w)
        med_off = 1 if has_median else 0

        def _c2_width_models(side_specs, side_match, bounds, sign):
            """按车道拓扑串成宽度轨迹，并为每条轨迹统一求 C2 站间多项式。"""
            raw, track_of, tracks = {}, {}, {}
            next_track = 0
            for si, spec in enumerate(side_specs):
                if spec is None:
                    continue
                (b0a, _mb0a), (b1a, _mb1a) = bounds[si]
                for k, ln_rec in enumerate(spec["lanes"]):
                    if sign < 0:
                        w0, w1 = b0a[k] - b0a[k + 1], b1a[k] - b1a[k + 1]
                    else:
                        w0, w1 = b0a[k + 1] - b0a[k], b1a[k + 1] - b1a[k]
                    if w0 < 0.4 or w1 < 0.4:
                        w0, w1 = ln_rec["w0"], ln_rec["w1"]
                        stats["profile_fallback"] = stats.get("profile_fallback", 0) + 1
                    pred = None
                    if si > 0 and side_specs[si - 1] is not None:
                        pred = next((a for a, b in side_match[si - 1].items()
                                     if b == k), None)
                    if pred is not None and (si - 1, pred) in track_of:
                        tid = track_of[(si - 1, pred)]
                    else:
                        tid = next_track
                        next_track += 1
                        tracks[tid] = []
                    track_of[(si, k)] = tid
                    tracks[tid].append((si, k))
                    raw[(si, k)] = (float(w0), float(w1))

            models = {}
            for items in tracks.values():
                items.sort()
                first_si, first_k = items[0]
                last_si, last_k = items[-1]
                born = first_si > 0
                dying = last_si < len(side_specs) - 1
                xs = [sections[first_si][0]]
                ys = [0.0 if born else raw[(first_si, first_k)][0]]
                for si, k in items:
                    end = raw[(si, k)][1]
                    if si == last_si and dying:
                        end = 0.0
                    xs.append(sections[si][1])
                    ys.append(float(end))
                ms = _c2_station_slopes(xs, ys, zero_start=born, zero_end=dying)
                # 非负性自检；若 natural spline 过冲，缩放所有站导数，仍保持 C1。
                scale = 1.0
                for _ in range(20):
                    safe = True
                    for q, (si, k) in enumerate(items):
                        L = xs[q + 1] - xs[q]
                        u = np.linspace(0.0, L, 21)
                        c = (3 * (ys[q + 1] - ys[q])
                             - (2 * ms[q] + ms[q + 1]) * L) / L ** 2
                        d = (-2 * (ys[q + 1] - ys[q])
                             + (ms[q] + ms[q + 1]) * L) / L ** 3
                        if np.min(ys[q] + ms[q] * u + c * u ** 2 + d * u ** 3) < -1e-6:
                            safe = False
                            break
                    if safe:
                        break
                    ms *= 0.8
                    scale *= 0.8
                if scale < 1.0:
                    stats["width_c2_slope_scale_min"] = min(
                        stats.get("width_c2_slope_scale_min", 1.0), scale)
                for q, key in enumerate(items):
                    models[key] = (ys[q], ms[q], ys[q + 1], ms[q + 1])
            return models

        r_width_models = _c2_width_models(rs, m_r, rbounds, -1)
        l_width_models = _c2_width_models(lspec, m_l, lbounds, +1)

        # —— 逐 section 生成车道对象（宽度 = 实测边界差 Hermite；生灭仍锥形收放；
        #    交界硬连续：续接车道起宽/起斜率 := 上段 written 末值，Σ堆叠零台阶） ——
        def _emit(side_specs, side_match, bounds, sign, width_models):
            all_objs, all_xid, all_startw, all_endw = [], [], [], []
            prev_w, prev_m = None, None
            for si, spec in enumerate(side_specs):
                objs, xid, start_w, end_w, end_m = [], {}, [], [], []
                if spec is not None:
                    L_sec = sections[si][1] - sections[si][0]
                    lanes = spec["lanes"]
                    (b0a, mb0a), (b1a, mb1a) = bounds[si]
                    for k, ln_rec in enumerate(lanes):
                        dying = si < len(side_match) and k not in side_match[si]
                        born = si > 0 and (side_specs[si - 1] is None or
                                           k not in side_match[si - 1].values())
                        w0, mw0, w1, mw1 = width_models[(si, k)]
                        lane_id = sign * (k + 1 + (med_off if sign > 0 else 0))
                        direction = "with_s" if sign < 0 else "against_s"
                        role = "approach" if sign < 0 else "departure"
                        # 先冻结真正写出的 width 多项式，下面的来源支持域才能按文件
                        # 语义计算中心线，而不是按尚未落盘的理想边界猜测。
                        c = (3 * (w1 - w0) - (2 * mw0 + mw1) * L_sec) / L_sec ** 2
                        d = (-2 * (w1 - w0) + (mw0 + mw1) * L_sec) / L_sec ** 3
                        pieces = [(0.0, w0, mw0, c, d)]
                        written_end_m = mw1
                        if born or dying:
                            stats["tapers"] = stats.get("tapers", 0) + 1
                        written_start_w = float(pieces[0][1])
                        written_end_w = float(_pieces_end(pieces, L_sec))
                        if ln_rec["extended"]:
                            # 数据来源是 extension，不代表物理上就是完整行车道。
                            # 若该 lane 在当前断面出生/消亡或端宽不足 0.4 m，它是
                            # 零宽渐变带；动力学审计不得把它当作 60 km/h 可行驶中心线。
                            extended_taper = (born or dying or
                                              min(written_start_w,
                                                  written_end_w) < 0.4)
                            exclusion_code = ("lane-transition-taper"
                                              if extended_taper
                                              else "source-extension")
                            support_kind = ("lane-transition-ribbon"
                                            if extended_taper
                                            else "source-extension")
                            if extended_taper:
                                stats["extended_transition_tapers"] = \
                                    stats.get("extended_transition_tapers", 0) + 1
                            ln = W.Lane(lane_id, source_id=ln_rec["l"].lane_pid,
                                        provenance={
                                            "eligibility": "excluded", "role": role,
                                            "status": "APPROXIMATED",
                                            "support_kind": support_kind,
                                            "travel_direction": direction,
                                            "exclusion_code": exclusion_code,
                                        })
                        else:
                            comp_a, comp_b = sections[si]
                            support_exclusion = None
                            taper_start = born or ln_rec["w0"] < 0.4
                            taper_end = dying or ln_rec["w1"] < 0.4
                            if taper_start:
                                comp_a += min(_TAPER, L_sec)
                                support_exclusion = "lane-transition-taper"
                            if taper_end:
                                comp_b -= min(_TAPER, L_sec)
                                support_exclusion = "lane-transition-taper"
                            # laneOffset/median/相邻宽度的 C0 连续化可能把当前来源中心
                            # 推离原始 v0/v1。只裁掉确实被连续化占用的那一小段；固定
                            # 0.5 m 是几何构造容差，不读取 G8 policy，也不改变 ceiling。
                            y0, _my0, y1, _my1 = off_vals[si]
                            if sign < 0:
                                target_v0 = y0 - sum(start_w) - written_start_w / 2.0
                                target_v1 = y1 - sum(end_w) - written_end_w / 2.0
                            else:
                                g0 = med_w[si][0] if has_median and med_w[si] is not None else 0.0
                                g1 = med_w[si][2] if has_median and med_w[si] is not None else 0.0
                                target_v0 = y0 + g0 + sum(start_w) + written_start_w / 2.0
                                target_v1 = y1 + g1 + sum(end_w) + written_end_w / 2.0
                            err0 = abs(target_v0 - float(ln_rec["source_v0"]))
                            err1 = abs(target_v1 - float(ln_rec["source_v1"]))
                            worst = max(err0, err1)
                            if worst > stats.get("written_center_residual_max_m", 0.0):
                                stats["written_center_residual_max_m"] = worst
                                stats["written_center_residual_worst"] = {
                                    "road": rid, "side": "right" if sign < 0 else "left",
                                    "section": si, "source_lane": ln_rec["l"].lane_pid,
                                    "start_m": err0, "end_m": err1,
                                }
                            if worst > _SOURCE_TRANSITION_TOL:
                                stats.setdefault("written_center_residuals", []).append({
                                    "road": rid, "side": "right" if sign < 0 else "left",
                                    "section": si, "source_lane": ln_rec["l"].lane_pid,
                                    "start_m": err0, "end_m": err1,
                                })
                            prev_same = False
                            if si > 0 and side_specs[si - 1] is not None:
                                prev_idx = [i2 for i2, j2 in side_match[si - 1].items()
                                            if j2 == k]
                                if prev_idx:
                                    prev_lane = side_specs[si - 1]["lanes"][prev_idx[0]]
                                    prev_same = (not prev_lane["extended"] and
                                                 prev_lane["l"].lane_pid ==
                                                 ln_rec["l"].lane_pid)
                            next_same = False
                            next_transition = False
                            if (si < len(side_match) and k in side_match[si]
                                    and side_specs[si + 1] is not None):
                                next_k = side_match[si][k]
                                next_lane = side_specs[si + 1]["lanes"][next_k]
                                next_same = (not next_lane["extended"] and
                                             next_lane["l"].lane_pid ==
                                             ln_rec["l"].lane_pid)
                                next_transition = (next_same and si + 1 < len(side_match)
                                                   and next_k not in side_match[si + 1])
                            # 是否同一 source id 不能决定可比性：内侧车道生灭会经堆叠
                            # 推移外侧同 ID 车道。以最终写出中心的实际误差裁支持域，
                            # 超差部分诚实记为 APPROXIMATED，不得冒充来源精确几何。
                            if not prev_same and err0 > _SOURCE_TRANSITION_TOL:
                                if err1 < err0 - 1e-9:
                                    frac = ((err0 - _SOURCE_TRANSITION_TOL) /
                                            max(err0 - err1, 1e-9))
                                    cut = min(L_sec, frac * L_sec + _SOURCE_TRANSITION_MARGIN)
                                    comp_a = max(comp_a, sections[si][0] + cut)
                                else:
                                    comp_a = sections[si][1]
                                support_exclusion = "lane-transition-taper"
                            if ((not next_same or next_transition)
                                    and err1 > _SOURCE_TRANSITION_TOL):
                                if err0 < err1 - 1e-9:
                                    frac = ((err1 - _SOURCE_TRANSITION_TOL) /
                                            max(err1 - err0, 1e-9))
                                    cut = min(L_sec, frac * L_sec + _SOURCE_TRANSITION_MARGIN)
                                    comp_b = min(comp_b, sections[si][1] - cut)
                                else:
                                    comp_b = sections[si][0]
                                support_exclusion = "lane-transition-taper"
                            if ((not prev_same and err0 > _SOURCE_TRANSITION_TOL) or
                                    ((not next_same or next_transition)
                                     and err1 > _SOURCE_TRANSITION_TOL)):
                                stats["source_center_transition_crops"] = \
                                    stats.get("source_center_transition_crops", 0) + 1
                            support_pts, support_s = _clip_support(
                                ln_rec["profile_rec"], comp_a, comp_b)
                            if sign > 0:
                                support_pts = support_pts[::-1]
                            at_stopline = sign < 0 and spec["pid"] == e_pid
                            support_too_short = _polylen(support_pts) < 3.0
                            if (len(support_pts) < 2 or support_s is None
                                    or support_too_short):
                                if support_too_short and support_exclusion is None:
                                    support_exclusion = "source-support-too-short"
                                pending_sources.setdefault(ln_rec["l"].lane_pid, {
                                    "rec": ln_rec["l"],
                                    "points": ln_rec["lg"] if sign < 0 else ln_rec["lg"][::-1],
                                    "role": role, "direction": direction,
                                    "at_stopline": at_stopline,
                                    "reason": support_exclusion or "source-support-unavailable",
                                })
                                code = support_exclusion or "source-support-unavailable"
                                ln = W.Lane(lane_id, source_id=ln_rec["l"].lane_pid,
                                            provenance={
                                                "eligibility": "excluded", "role": role,
                                                "status": "APPROXIMATED",
                                                "support_kind": code,
                                                "travel_direction": direction,
                                                "exclusion_code": code,
                                            })
                            else:
                                _register_source(ln_rec["l"], support_pts, role=role,
                                                 direction=direction, at_stopline=at_stopline,
                                                 support_reason=(support_exclusion
                                                                 or "target-leg-domain"))
                                sm = manifest_lanes[ln_rec["l"].lane_pid]
                                provenance = {
                                    "eligibility": "comparable", "role": role,
                                    "status": sm["status"], "support_kind": sm["support_kind"],
                                    "policy_class": sm["policy_class"],
                                    "travel_direction": direction,
                                    "support_s": list(support_s),
                                    "boundary_evidence_trusted": bool(
                                        _trusted_boundary_lane(ln_rec)),
                                }
                                if support_exclusion:
                                    provenance["support_exclusion_code"] = support_exclusion
                                ln = W.Lane(lane_id, source_id=ln_rec["l"].lane_pid,
                                            provenance=provenance)
                        for so, a, b, c, dd in pieces:
                            ln.add_width(a, b, c, dd, s_offset=so)
                        start_w.append(written_start_w)
                        end_w.append(written_end_w)
                        end_m.append(written_end_m)
                        ln.mark = std_mark("outer" if k == len(lanes) - 1 else "inner")
                        if ln_rec["l"].max_speed_kmh:
                            ln.speed_ms = float(ln_rec["l"].max_speed_kmh) / 3.6
                            stats["speeds"] += 1
                        objs.append(ln)
                        xid[ln_rec["l"].seq] = lane_id
                all_objs.append(objs)
                all_xid.append(xid)
                all_startw.append(start_w)
                all_endw.append(end_w)
                prev_w = end_w if spec is not None else None
                prev_m = end_m if spec is not None else None
            return all_objs, all_xid, all_startw, all_endw

        r_objs, r_xid, r_startw, r_endw = _emit(
            rs, m_r, rbounds, -1, r_width_models)
        l_objs, l_xid, l_startw, l_endw = _emit(
            lspec, m_l, lbounds, +1, l_width_models)
        # 跨 section 车道衔接
        for si, mp in enumerate(m_r):
            for ia, ib in mp.items():
                if ia < len(r_objs[si]) and ib < len(r_objs[si + 1]):
                    r_objs[si][ia].succ = -(ib + 1)
                    r_objs[si + 1][ib].pred = -(ia + 1)
        for si, mp in enumerate(m_l):
            for ia, ib in mp.items():
                if ia < len(l_objs[si]) and ib < len(l_objs[si + 1]):
                    l_objs[si][ia].succ = ib + 1 + med_off
                    l_objs[si + 1][ib].pred = ia + 1 + med_off

        # width 必须沿完整车道轨迹平滑；几何采样小段不是独立的车道生灭事件。
        r_totals = [
            (b0[0] - b0[-1], m0[0] - m0[-1],
             b1[0] - b1[-1], m1[0] - m1[-1])
            for (b0, m0), (b1, m1) in rbounds
        ]
        l_totals = []
        for pair in lbounds:
            if pair is None:
                l_totals.append((0.0, 0.0, 0.0, 0.0))
            else:
                (b0, m0), (b1, m1) = pair
                l_totals.append(
                    (b0[-1] - b0[0], m0[-1] - m0[0],
                     b1[-1] - b1[0], m1[-1] - m1[0]))
        r_base_offsets = list(off_vals)
        l_base_offsets = []
        for si, (y0, my0, y1, my1) in enumerate(off_vals):
            if med_w[si] is None:
                l_base_offsets.append((y0, my0, y1, my1))
            else:
                g0, mg0, g1, mg1 = med_w[si]
                l_base_offsets.append((y0 + g0, my0 + mg0,
                                       y1 + g1, my1 + mg1))
        _smooth_lane_width_tracks(sections, rs, m_r, r_objs, r_startw, r_endw,
                                  r_totals, L_fit, stats, "right",
                                  r_base_offsets, -1.0,
                                  # LANE_BOUNDARY/中心线共同形成的物理道路外包络是
                                  # 目标格式必须复原的硬约束。此前双向 leg 为追车道
                                  # 中心允许总宽逐站放松 ±0.35m，叠加边界平滑后会把
                                  # node18 可信外缘推开 0.6--0.8m。中心平滑只能在内部
                                  # width 间重新分配，不得移动整幅道路外边缘。
                                  allow_total_relax=False)
        _smooth_lane_width_tracks(sections, lspec, m_l, l_objs, l_startw, l_endw,
                                  l_totals, L_fit, stats, "left",
                                  l_base_offsets, +1.0)
        _weld_linked_lane_width_derivatives(
            sections, m_r, r_objs, stats, "right")
        _weld_linked_lane_width_derivatives(
            sections, m_l, l_objs, stats, "left")
        # 总宽值与一阶导数已经在轨迹联合优化中同时闭合；这里不得再把残差
        # 塞进某一条车道，否则短 laneSection 会出现蛇形内边界。

        def _keep_single_source_component(side_objects):
            """每个 source lane 只保留最长连续 comparable 支持域。

            生灭修复可能把同一来源在中间切开；若两端都继续标 comparable，G8
            就会得到 one_source_many_targets。较短分量诚实降级为过渡区，不拼接
            不连续几何，也不改变实际写出形态。
            """
            by_source = {}
            for si, lanes in enumerate(side_objects):
                for k, lane in enumerate(lanes):
                    if (lane.source_id and lane.provenance is not None
                            and lane.provenance.get("eligibility") == "comparable"):
                        by_source.setdefault(lane.source_id, []).append((si, k, lane))
            for occurrences in by_source.values():
                runs, current = [], []
                for item in occurrences:
                    if current and item[0] != current[-1][0] + 1:
                        runs.append(current)
                        current = []
                    current.append(item)
                if current:
                    runs.append(current)
                if len(runs) <= 1:
                    continue
                keep = max(runs, key=lambda run: sum(
                    sections[si][1] - sections[si][0] for si, _k, _lane in run))
                keep_ids = {id(lane) for _si, _k, lane in keep}
                for run in runs:
                    for _si, _k, lane in run:
                        if id(lane) in keep_ids:
                            continue
                        lane.provenance.update({
                            "eligibility": "excluded", "status": "APPROXIMATED",
                            "support_kind": "lane-transition-ribbon",
                            "exclusion_code": "lane-transition-taper",
                            "geometry_adjustment": "single-source-component",
                        })
                        lane.provenance.pop("support_s", None)

        _keep_single_source_component(r_objs)
        _keep_single_source_component(l_objs)

        def _refresh_manifest_support(side_specs, side_objects, sign):
            """过渡后按最终 comparable occurrence 重建 G8 来源支持域。

            width-track 优化发生在 `_emit` 注册 manifest 之后；若扩大的 20m 生灭
            区只改 target provenance、不同步 source manifest，G8 会拿半条目标去对
            整条来源并报告数十米伪误差。这里以最终 support_s 重新裁来源中心线。
            """
            touched, parts = set(), {}
            for spec, lanes in zip(side_specs, side_objects):
                if spec is None:
                    continue
                for k, lane_rec in enumerate(spec["lanes"]):
                    sid = lane_rec["l"].lane_pid
                    touched.add(sid)
                    if k >= len(lanes):
                        continue
                    provenance = lanes[k].provenance or {}
                    support = provenance.get("support_s")
                    if (provenance.get("eligibility") != "comparable"
                            or not isinstance(support, list) or len(support) != 2):
                        continue
                    pts, _support = _clip_support(
                        lane_rec["profile_rec"], float(support[0]), float(support[1]))
                    if len(pts) >= 2:
                        parts.setdefault(sid, []).append((float(support[0]), pts))
            for sid in touched:
                entry = manifest_lanes.get(sid)
                if entry is None:
                    continue
                available = sorted(parts.get(sid, []), key=lambda item: item[0])
                if not available:
                    entry["comparison"]["eligible"] = False
                    entry.setdefault("support", {})["compared_length_m"] = 0.0
                    continue
                joined = np.vstack([item[1] for item in available])
                keep = np.concatenate([[True],
                                       np.linalg.norm(np.diff(joined, axis=0), axis=1) > 1e-6])
                joined = joined[keep]
                if sign > 0:
                    joined = joined[::-1]
                entry["geometry"]["coordinates"] = joined.tolist()
                entry["geometry"]["geometry_sha256"] = geometry_sha256(joined)
                entry["travel"]["start"] = joined[0].tolist()
                entry["travel"]["end"] = joined[-1].tolist()
                entry["comparison"]["eligible"] = True
                entry.setdefault("support", {})["compared_length_m"] = _polylen(joined)

        _refresh_manifest_support(rs, r_objs, -1)
        _refresh_manifest_support(lspec, l_objs, +1)

        # 生灭车道的零宽收口会把“驾驶车道堆叠外缘”拉向道路内部；但 SHP 的
        # 物理 LANE_BOUNDARY 可能继续存在。用非驾驶 shoulder 填满两者之间的
        # 正向余量，既不移动任何驾驶车道中心，也让 odrviewer 的道路面保持真实外形。
        def _edge_fill(side):
            fills = []
            for si in range(len(sections)):
                y0, _my0, y1, _my1 = off_vals[si]
                if side == "right":
                    desired0 = rbounds[si][0][0][-1]
                    desired1 = rbounds[si][1][0][-1]
                    current0 = y0 - sum(r_startw[si])
                    current1 = y1 - sum(r_endw[si])
                    c0, c1 = current0 - desired0, current1 - desired1
                else:
                    if lbounds[si] is None:
                        fills.append((0.0, 0.0))
                        continue
                    g0 = med_w[si][0] if has_median and med_w[si] is not None else 0.0
                    g1 = med_w[si][2] if has_median and med_w[si] is not None else 0.0
                    desired0 = lbounds[si][0][0][-1]
                    desired1 = lbounds[si][1][0][-1]
                    current0 = y0 + g0 + sum(l_startw[si])
                    current1 = y1 + g1 + sum(l_endw[si])
                    c0, c1 = desired0 - current0, desired1 - current1
                if min(c0, c1) < -0.05:
                    stats["physical_edge_overdraw_max_m"] = max(
                        stats.get("physical_edge_overdraw_max_m", 0.0), -min(c0, c1))
                fills.append((max(0.0, c0), max(0.0, c1)))
            for si in range(len(fills) - 1):
                shared = (fills[si][1] + fills[si + 1][0]) / 2.0
                fills[si] = (fills[si][0], shared)
                fills[si + 1] = (shared, fills[si + 1][1])
            return fills

        r_fill, l_fill = _edge_fill("right"), _edge_fill("left")
        # v1.30 裁决：不得用贯穿普通 road 的 shoulder 修补 driving-lane 堆叠。
        # 这种“补面”虽可降低外缘最近距离，却会额外生成一组车道线，并把每个
        # laneSection 的局部差量渲染成连续鼓包。保留差量统计作为诊断，写出禁用；
        # 后续由共享边界优化直接解决横断面，而不是用新车道遮盖错误。
        use_r_fill = False
        use_l_fill = False
        stats["physical_edge_fill_disabled"] = True
        stats["physical_edge_fill_max_m"] = max(
            max((max(x) for x in r_fill), default=0.0),
            max((max(x) for x in l_fill), default=0.0))

        rl = src.roadlinks.get(e_pid)
        road = W.Road(rid, name=(rl.name if rl else None))
        prims, ep = planview_prims(pv)
        for p in prims:
            road.add_geometry(*p)
        for i, (u0, u1) in enumerate(sections):
            y0, my0, y1, my1 = off_vals[i]               # Hermite：贴实测左缘
            Ls = max(u1 - u0, 1e-3)
            if not boundary_coupled:
                my0, my1 = _fc(y0, my0, y1, my1, Ls)
            c = (3 * (y1 - y0) - (2 * my0 + my1) * Ls) / Ls ** 2
            d = (-2 * (y1 - y0) + (my0 + my1) * Ls) / Ls ** 3
            road.add_offset(u0, y0, my0, c, d)
        med_end_last = 0.0
        for si, (u0, u1) in enumerate(sections):
            sec = W.LaneSection(u0, center_mark=std_mark("center2" if two_way else "center"))
            if lspec[si] is not None and has_median:     # median 车道恒 +1（宽可为 0）
                g0, m0, g1, m1 = med_w[si]
                Ls = max(u1 - u0, 1e-3)
                if not boundary_coupled:
                    m0, m1 = _fc(g0, m0, g1, m1, Ls)
                c = (3 * (g1 - g0) - (2 * m0 + m1) * Ls) / Ls ** 2
                d = (-2 * (g1 - g0) + (m0 + m1) * Ls) / Ls ** 3
                mln = W.Lane(1, "median", provenance={
                    "eligibility": "excluded", "role": "median",
                    "status": "TRANSFORMED", "support_kind": "median",
                    "travel_direction": "against_s", "exclusion_code": "median-non-driving",
                })
                mln.add_width(g0, m0, c, d)              # Hermite：值+斜率双侧衔接（全程 C1）
                if si == len(sections) - 1:
                    med_end_last = g1                    # 路口端中隔宽（堆叠用）
                if si > 0 and lspec[si - 1] is not None:
                    mln.pred = 1
                if si < len(sections) - 1 and lspec[si + 1] is not None:
                    mln.succ = 1
                sec.left.append(mln)
            sec.left.extend(l_objs[si])
            sec.right.extend(r_objs[si])
            Ls = max(u1 - u0, 1e-3)
            if use_l_fill and lspec[si] is not None:
                lane_id = len(l_objs[si]) + 1 + med_off
                source_id = lspec[si]["lanes"][-1]["l"].lane_pid
                shoulder = W.Lane(lane_id, "shoulder", source_id=source_id,
                                  provenance={
                                      "eligibility": "excluded", "role": "shoulder",
                                      "status": "TRANSFORMED",
                                      "support_kind": "physical-edge-fill",
                                      "travel_direction": "against_s",
                                      "exclusion_code": "physical-edge-fill",
                                  })
                shoulder.add_width(*_smooth_w(*l_fill[si], Ls))
                shoulder.mark = std_mark("outer")
                if si > 0 and lspec[si - 1] is not None:
                    shoulder.pred = len(l_objs[si - 1]) + 1 + med_off
                if si + 1 < len(sections) and lspec[si + 1] is not None:
                    shoulder.succ = len(l_objs[si + 1]) + 1 + med_off
                sec.left.append(shoulder)
            if use_r_fill and rs[si] is not None:
                lane_id = -(len(r_objs[si]) + 1)
                source_id = rs[si]["lanes"][-1]["l"].lane_pid
                shoulder = W.Lane(lane_id, "shoulder", source_id=source_id,
                                  provenance={
                                      "eligibility": "excluded", "role": "shoulder",
                                      "status": "TRANSFORMED",
                                      "support_kind": "physical-edge-fill",
                                      "travel_direction": "with_s",
                                      "exclusion_code": "physical-edge-fill",
                                  })
                shoulder.add_width(*_smooth_w(*r_fill[si], Ls))
                shoulder.mark = std_mark("outer")
                if si > 0 and rs[si - 1] is not None:
                    shoulder.pred = -(len(r_objs[si - 1]) + 1)
                if si + 1 < len(sections) and rs[si + 1] is not None:
                    shoulder.succ = -(len(r_objs[si + 1]) + 1)
                sec.right.append(shoulder)
            road.sections.append(sec)
            stats["sections"] += 1
        before_sections, after_sections = _compact_lane_sections(road)
        stats["geometry_intervals"] = stats.get("geometry_intervals", 0) + before_sections
        stats["sections"] += after_sections - before_sections
        stats["lane_sections_compacted"] = stats.get("lane_sections_compacted", 0) \
            + before_sections - after_sections
        if len(sections) > 1:
            stats["multi_section_roads"] += 1
        road.add_link("successor", "junction", JID)
        doc.add_road(road)

        # —— 路口端车道位姿（文件语义堆叠：laneOffset ± written 宽度累计） ——
        last = len(sections) - 1
        last_len = max(sections[last][1] - sections[last][0], 1e-6)

        def _second_end(values):
            y0, m0, y1, m1 = map(float, values)
            c = (3.0 * (y1 - y0) - (2.0 * m0 + m1) * last_len) / last_len ** 2
            d = (-2.0 * (y1 - y0) + (m0 + m1) * last_len) / last_len ** 3
            return 2.0 * c + 6.0 * d * last_len

        off_end, off_d1 = off_vals[-1][2], off_vals[-1][3]
        off_d2 = _second_end(off_vals[-1])
        ref_k = seg_kappa(pv, True)
        tail = pv.segs[-1]
        ref_sharp = ((float(tail.curvature_end) - float(tail.curvature))
                     / max(float(tail.length), 1e-9)
                     if tail.kind == "spiral" else 0.0)
        cum = dcum = d2cum = 0.0
        for k, ln_rec in enumerate(rs[last]["lanes"] if rs[last] else []):
            w, dw, d2w = _width_kinematics(r_objs[last][k], last_len)
            t = off_end - cum - w / 2
            dt = off_d1 - dcum - dw / 2
            d2t = off_d2 - d2cum - d2w / 2
            lane_end[ln_rec["l"].lane_pid] = _lane_profile_pose(
                ep, t, dt, d2t, ref_k, ref_sharp)
            stats["mouth_heading_adjust_max_deg"] = max(
                stats.get("mouth_heading_adjust_max_deg", 0.0),
                abs(math.degrees(math.atan2(dt, 1.0 - t * ref_k))))
            stats["mouth_lane_kappa_max"] = max(
                stats.get("mouth_lane_kappa_max", 0.0),
                abs(lane_end[ln_rec["l"].lane_pid][3]))
            cum += w
            dcum += dw
            d2cum += d2w
        rid_of[e_pid], xid_of[e_pid] = rid, r_xid[last]
        end_pose[e_pid] = ep
        stats["roads_enter"] += 1
        if l_pids and lspec[last] is not None:
            if has_median and med_w[last] is not None:
                med_end, med_d1 = med_w[last][2], med_w[last][3]
                med_d2 = _second_end(med_w[last])
            else:
                med_end = med_d1 = med_d2 = 0.0
            cum, dcum, d2cum = med_end, med_d1, med_d2
            leave_xids = {}
            for k, ln_rec in enumerate(lspec[last]["lanes"]):
                w, dw, d2w = _width_kinematics(l_objs[last][k], last_len)
                t = off_end + cum + w / 2
                dt = off_d1 + dcum + dw / 2
                d2t = off_d2 + d2cum + d2w / 2
                p = _lane_profile_pose(
                    ep, t, dt, d2t, ref_k, ref_sharp, reverse=True)
                # 单组腿保持旧契约：末 section 即便因拼链投影落到相邻 Link，
                # junction 的权威出口仍是 junc.leave_roads 中的 seed l_pid。
                # 多并行组则按各自 seed Link 登记，不把上/下游链段误算成第五条腿。
                lp = l_pids[0] if len(l_pids) == 1 else ln_rec["l"].link_pid
                lane_start[(lp, ln_rec["l"].seq)] = p
                stats["mouth_heading_adjust_max_deg"] = max(
                    stats.get("mouth_heading_adjust_max_deg", 0.0),
                    abs(math.degrees(math.atan2(dt, 1.0 - t * ref_k))))
                stats["mouth_lane_kappa_max"] = max(
                    stats.get("mouth_lane_kappa_max", 0.0), abs(p[3]))
                if lp in l_pids:
                    leave_xids.setdefault(lp, {})[ln_rec["l"].seq] = k + 1 + med_off
                cum += w
                dcum += dw
                d2cum += d2w
            if len(l_pids) == 1:
                leave_xids[l_pids[0]] = dict(l_xid[last])
            for lp, xids in leave_xids.items():
                rid_of[lp], xid_of[lp] = rid, xids
                exit_contact[lp] = "end"
            stats["roads_leave"] += len(leave_xids)
            if leave_xids:
                stats["legs_two_way"] += 1
        return True

    # ---------------------------------------------------------------- 单侧出口回退
    def build_single_leave(lpid: str, rid: int) -> bool:
        chain = _chained_links(src, lpid, False, proj, cj, max_len=max_len)
        if not chain:
            return False
        pts = chain[0][1]
        for _, c in chain[1:]:
            pts = np.vstack([pts, c[1:]])
        gxy = proj(pts)
        # 单独的出口 carriageway 按 OpenDRIVE 语义必须从 junction 向外增长。
        # 图商 ROADLINK 的点序并不总能保证这一点，先统一方向；随后在口部向
        # 路口内延伸 1m 形成 apron。若只让 road 与铺面边界数学相切，查看器
        # 浮点离散后会露出细缝，surface gate 也无法证明二者有实面积重叠。
        if np.linalg.norm(gxy[0] - cj) > np.linalg.norm(gxy[-1] - cj):
            gxy = gxy[::-1]
        if len(gxy) >= 2:
            outward = gxy[1] - gxy[0]
            outward /= np.linalg.norm(outward) + 1e-12
            inward = -outward
            if float(np.dot(cj - gxy[0], inward)) > 0.0:
                gxy = np.vstack([(gxy[0] + inward * 1.0)[None, :], gxy])
                stats["directional_carriageway_apron_m"] = max(
                    stats.get("directional_carriageway_apron_m", 0.0), 1.0)
        try:
            pv, dev, smoothed = fit_leg_refline(gxy)
        except ReflineFitError as exc:
            raise ReflineFitError(
                f"SHP junction={junc.pid} independent_leave_link={lpid} "
                f"points={len(gxy)}: {exc}"
            ) from exc
        impulse = pv.fit_meta.get("impulse_filter", {})
        if impulse.get("removed_count"):
            stats["refline_outlier_points_removed"] = (
                stats.get("refline_outlier_points_removed", 0)
                + int(impulse["removed_count"]))
            stats["refline_outlier_max_residual_m"] = max(
                stats.get("refline_outlier_max_residual_m", 0.0),
                float(impulse.get("max_residual_m", 0.0)))
        if smoothed:
            stats["refit_smoothed"] = stats.get("refit_smoothed", 0) + 1
        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
        L_fit = sum(s.length for s in pv.segs)
        ref = eval_planview(pv, 0.5)
        tang = np.gradient(ref, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        recs = _span_recs(lpid, ref, tang, 0.0, min(40.0, L_fit), seed_tail=False)
        if not recs:
            return False
        recs.sort(key=lambda r: -r["d"])
        off = recs[0]["d"] + recs[0]["sw"] / 2
        # 独立方向分幅没有对向车道帮助约束 reference line；字段 S/E_WIDTH 若与
        # 实测车道中心横距冲突，会把外侧车道中心推偏半个宽差、外缘推偏整个宽差。
        # 仅在此类 split road 内，用同一参考线上的实测中心 d 反算相邻宽度；
        # 差异不足 0.6m 保留图商字段，避免把正常展宽误判为错误。
        for endpoint in ("sw", "ew"):
            cumulative = 0.0
            for rec in recs:
                inferred = 2.0 * (off - cumulative - rec["d"])
                if (1.5 <= inferred <= 6.0
                        and rec.get("boundary_profiles")
                        and abs(inferred - rec[endpoint]) > 0.60):
                    stats["directional_width_reconciled"] = (
                        stats.get("directional_width_reconciled", 0) + 1)
                    stats["directional_width_reconcile_max_m"] = max(
                        stats.get("directional_width_reconcile_max_m", 0.0),
                        abs(inferred - rec[endpoint]))
                    rec[endpoint] = inferred
                cumulative += rec[endpoint]
        sec = W.LaneSection(0.0, center_mark=std_mark("center"))
        xid, cum = {}, 0.0
        for k, r in enumerate(recs):
            support_lg, support_s = _clip_support(r, 0.0, L_fit)
            if len(support_lg) >= 2 and support_s is not None:
                _register_source(r["l"], support_lg, role="departure", direction="with_s")
                sm = manifest_lanes[r["l"].lane_pid]
                provenance = {
                    "eligibility": "comparable", "role": "departure",
                    "status": sm["status"], "support_kind": sm["support_kind"],
                    "policy_class": sm["policy_class"],
                    "travel_direction": "with_s", "support_s": list(support_s),
                }
            else:
                _register_source(r["l"], r["lg"], role="departure", direction="with_s",
                                 eligible=False, support_reason="source-support-unavailable")
                provenance = {
                    "eligibility": "excluded", "role": "departure",
                    "status": "APPROXIMATED", "support_kind": "source-support-unavailable",
                    "travel_direction": "with_s",
                    "exclusion_code": "source-support-unavailable",
                }
            ln = W.Lane(-(k + 1), source_id=r["l"].lane_pid, provenance=provenance)
            ln.add_width(*_smooth_w(r["sw"], r["ew"], L_fit))
            ln.mark = std_mark("outer" if k == len(recs) - 1 else "inner")
            if r["l"].max_speed_kmh:
                ln.speed_ms = float(r["l"].max_speed_kmh) / 3.6
                stats["speeds"] += 1
            sec.right.append(ln)
            xid[r["l"].seq] = -(k + 1)
            t = off - cum - r["sw"] / 2
            p = _shift((pv.x0, pv.y0, pv.hdg), t)
            lane_start[(lpid, r["l"].seq)] = (p[0], p[1], pv.hdg)
            cum += r["sw"]
        rl = src.roadlinks.get(lpid)
        road = W.Road(rid, name=(rl.name if rl else None))
        prims, _ep = planview_prims(pv)
        for p in prims:
            road.add_geometry(*p)
        road.add_offset(0.0, off)
        road.sections.append(sec)
        road.add_link("predecessor", "junction", JID)
        doc.add_road(road)
        rid_of[lpid], xid_of[lpid] = rid, xid
        exit_contact[lpid] = "start"
        stats["roads_leave"] += 1
        stats["sections"] += 1
        return True

    pairs, single_leaves = _pair_legs(src, junc, proj, cj)

    # 同一物理道路的两个方向只有在“可用几何覆盖相近”时才适合共享一条
    # OpenDRIVE reference line。若一侧比另一侧多出几十米，强行共线会把
    # 缺测方向外推到不存在的区段，并把该误差转移到 laneOffset / width：
    # 结果虽可被 XSD 接受，却会出现波浪边缘和较大的横向 jerk。
    #
    # 对这种情形按专家评审意见改成两条方向独立的 carriageway。这里刻意
    # 限制为“一进一出且两侧均有足够长度”，避免把 node18 的 12m 短出口、
    # node13 的并行出口组误判为可自动拆分的普通道路。
    def _axis_length(pid: str, is_enter: bool) -> float:
        ch = _chained_links(src, pid, is_enter, proj, cj, max_len=max_len)
        return float(sum(_polylen(proj(c)) for _p, c in (ch or [])))

    split_pairs = []
    adjusted_pairs = []
    queued_single = set(single_leaves)
    for e_pid, l_pids in pairs:
        l_pids = list(l_pids or [])
        if len(l_pids) == 1:
            l_pid = l_pids[0]
            le = _axis_length(e_pid, True)
            ll = _axis_length(l_pid, False)
            delta = abs(le - ll)
            ratio = delta / max(le, ll, 1e-9)
            # 只有覆盖差异达到一条完整道路段时才拆分。20--30m 的差异可能来自
            # 合流/停止线 Link；把这类不规则 seed Link 单独成路会丢失后继断面。
            if min(le, ll) >= 60.0 and delta > 35.0 and ratio > 0.15:
                adjusted_pairs.append((e_pid, []))
                if l_pid not in queued_single:
                    single_leaves.append(l_pid)
                    queued_single.add(l_pid)
                split_pairs.append({
                    "enter_link": e_pid,
                    "leave_link": l_pid,
                    "enter_length_m": round(le, 3),
                    "leave_length_m": round(ll, 3),
                    "length_delta_m": round(delta, 3),
                    "length_delta_ratio": round(ratio, 6),
                    "decision": "directional-carriageways",
                    "reason": "opposite-source-coverage-mismatch",
                })
                continue
        adjusted_pairs.append((e_pid, l_pids))
    pairs = adjusted_pairs
    stats["directional_carriageway_splits"] = len(split_pairs)
    stats["carriageway_split_decisions"] = split_pairs

    for i, (e_pid, l_pids) in enumerate(pairs):
        build_leg(e_pid, l_pids, 10 + i)
        for l_pid in l_pids:                             # 腿建成但某并行组没挂上：单侧回退
            if l_pid not in rid_of:
                single_leaves.append(l_pid)
    for j, lpid in enumerate(single_leaves):
        build_single_leave(lpid, 30 + j)

    # —— junction 连接路：TOPO 两跳（进口车道→路口内车道→出口车道）——
    enter_set = [p for p in junc.enter_roads if p in rid_of]
    leave_set = {p for p in junc.leave_roads if p in rid_of}
    junction = W.Junction(JID, f"junc_{junc.pid[-8:]}")
    conn_roads, conn_objs = {}, {}
    rid_c = 100

    def connecting_road(key, prims, width_m, e_pid, x_pid, in_xid, out_xid,
                        *, source_rec=None, source_points=None, source_support_s=None,
                        source_exclusion_code=None,
                        source_diagnostics=None,
                        exclusion_code="inferred-connector-no-source-geometry"):
        nonlocal rid_c
        if key in conn_roads:
            rid0, owner = conn_roads[key]
            if owner == e_pid:                           # 同进口路复用：仅补 laneLink
                conn_objs[key].add_lanelink(in_xid, -1)
                stats["lanelinks"] += 1
            else:
                stats["skipped"] += 1
            return
        road = W.Road(rid_c, junction=JID)
        for p in prims:
            road.add_geometry(*p)
        road.add_offset(0.0, width_m / 2)
        sec = W.LaneSection(0.0)
        if source_rec is not None and source_points is not None:
            if source_exclusion_code is not None:
                _register_source(source_rec, source_points, role="junction-via",
                                 direction="with_s", via=True, eligible=False,
                                 support_reason=source_exclusion_code)
                if source_diagnostics:
                    manifest_lanes[source_rec.lane_pid]["source_conflict"] = \
                        dict(source_diagnostics)
                ln = W.Lane(-1, source_id=source_rec.lane_pid, provenance={
                    "eligibility": "excluded", "role": "junction-via",
                    "status": "APPROXIMATED", "support_kind": "source-topology-gap",
                    "travel_direction": "with_s",
                    "exclusion_code": source_exclusion_code,
                })
                if source_diagnostics:
                    ln.provenance["source_conflict"] = dict(source_diagnostics)
            else:
                _register_source(source_rec, source_points, role="junction-via",
                                 direction="with_s", via=True,
                                 support_reason=("source-topology-gap-bridge"
                                                 if source_support_s is not None
                                                 else "target-leg-domain"))
                sm = manifest_lanes[source_rec.lane_pid]
                provenance = {
                    "eligibility": "comparable", "role": "junction-via",
                    "status": sm["status"], "support_kind": sm["support_kind"],
                    "policy_class": sm["policy_class"],
                    "travel_direction": "with_s",
                }
                if source_support_s is not None:
                    provenance["support_s"] = list(source_support_s)
                    provenance["support_exclusion_code"] = "source-topology-gap-bridge"
                if source_diagnostics:
                    provenance["source_end_diagnostics"] = dict(source_diagnostics)
                ln = W.Lane(-1, source_id=source_rec.lane_pid, provenance=provenance)
        else:
            ln = W.Lane(-1, provenance={
                "eligibility": "excluded", "role": "connector",
                "status": "INFERRED", "support_kind": "synthetic-connector",
                "travel_direction": "with_s", "exclusion_code": exclusion_code,
            })
        ln.add_width(width_m)
        # 限速是来源属性，不是拟合器的可调参数。即使几何被降级为合成，
        # 也不能篡改其源车道限速；纯推断连接则不发明限速。
        source_limit = source_rec.max_speed_kmh if source_rec is not None else None
        if source_limit is not None and source_limit > 0:
            ln.speed_ms = float(source_limit) / 3.6
            stats["speeds"] += 1
        ln.provenance["speed_limit_basis"] = (
            "source-lane" if ln.speed_ms is not None else "absent-in-source")
        stats["speed_limit_policy"] = "preserve-source-no-geometry-derived-caps"
        ln.pred, ln.succ = in_xid, out_xid
        sec.right.append(ln)
        road.sections.append(sec)
        road.add_link("predecessor", "road", rid_of[e_pid], "end")
        road.add_link("successor", "road", rid_of[x_pid],
                      exit_contact.get(x_pid, "start"))
        doc.add_road(road)
        conn = W.Connection(rid_of[e_pid], rid_c, "start")
        conn.add_lanelink(in_xid, -1)
        junction.connections.append(conn)
        conn_roads[key], conn_objs[key] = (rid_c, e_pid), conn
        stats["connections"] += 1
        stats["lanelinks"] += 1
        rid_c += 1

    filled_pairs = set()                                 # 已建连接对（补全去重用）

    def _source_end_state_conflict(points, p0, p1, window_m=7.0):
        """来源连接折线端状态与最终道路口部状态的冲突量。

        端部用固定长度弦而非首个短边，避免采样噪声夸大角度。大角度冲突时若
        仍强追来源，优化器只能制造 1--2m 急调曲率段；该来源必须降级而不能
        作为精确可比真值。
        """
        g = np.asarray(points, float)
        ss = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))])

        def heading(at_start):
            if at_start:
                q = g[ss <= min(float(window_m), float(ss[-1])) + 1e-9]
            else:
                q = g[ss >= max(0.0, float(ss[-1]) - float(window_m)) - 1e-9]
            if len(q) < 2:
                q = g[:2] if at_start else g[-2:]
            d = q[-1] - q[0]
            return math.atan2(float(d[1]), float(d[0]))

        def delta_deg(a, b):
            return abs(math.degrees((a - b + math.pi) % (2 * math.pi) - math.pi))

        return {
            "start_heading_deg": delta_deg(heading(True), float(p0[2])),
            "end_heading_deg": delta_deg(heading(False), float(p1[2])),
            "start_position_m": float(np.linalg.norm(g[0] - np.asarray(p0[:2]))),
            "end_position_m": float(np.linalg.norm(g[-1] - np.asarray(p1[:2]))),
        }

    for e_pid in enter_set:
        for l in [x for x in src.lanes_of(e_pid) if x.geometry.shape[0] >= 2]:
            in_xid = xid_of[e_pid].get(l.seq)
            if in_xid is None:
                continue
            for out1 in src.topo_out.get(l.lane_pid, []):
                mid = src.lane(out1)
                if mid is None:
                    stats["skipped"] += 1
                    continue
                if mid.link_pid in leave_set:            # TOPO 直连（无路口内几何）→ G2 合成
                    targets = [(mid, None)]
                else:
                    targets = [(src.lane(o2), mid) for o2 in src.topo_out.get(out1, [])
                               if src.lane(o2) is not None
                               and src.lane(o2).link_pid in leave_set]
                for out_rec, via in targets:
                    out_xid = xid_of[out_rec.link_pid].get(out_rec.seq)
                    if out_xid is None:
                        stats["skipped"] += 1
                        continue
                    if via is not None and via.geometry.shape[0] >= 3:
                        vg = proj(via.geometry)
                        a = np.asarray(lane_end.get(l.lane_pid, end_pose[e_pid])[:2])
                        if np.linalg.norm(vg[-1] - a) < np.linalg.norm(vg[0] - a):
                            vg = vg[::-1]                # 起点靠进口侧
                        p0 = lane_end.get(l.lane_pid)
                        p1 = lane_start.get((out_rec.link_pid, out_rec.seq))
                        crop_diagnostics = None
                        composite_diagnostics = None
                        if mouth_policy == "source-envelope-candidate":
                            composite, composite_diagnostics = _topological_connector_source(
                                vg, proj(l.geometry), proj(out_rec.geometry))
                            if composite is not None:
                                vg = composite
                                composite_diagnostics["source_lane_ids"] = [
                                    l.lane_pid, via.lane_pid, out_rec.lane_pid]
                                stats["conn_source_composite"] = stats.get(
                                    "conn_source_composite", 0) + 1
                        if p0 is not None and p1 is not None:
                            vg, crop_diagnostics = _clip_connector_source_to_mouths(
                                vg, p0, p1)
                            if (crop_diagnostics["cropped_start_m"] > 0.25
                                    or crop_diagnostics["cropped_end_m"] > 0.25):
                                stats["conn_source_support_cropped"] = stats.get(
                                    "conn_source_support_cropped", 0) + 1
                        source_diagnostics = (
                            _source_end_state_conflict(vg, p0, p1)
                            if p0 is not None and p1 is not None else None)
                        if source_diagnostics is not None and crop_diagnostics is not None:
                            source_diagnostics["source_crop"] = crop_diagnostics
                        if source_diagnostics is not None and composite_diagnostics is not None:
                            source_diagnostics["source_composite"] = composite_diagnostics
                        source_state_conflict = bool(
                            source_diagnostics
                            and max(source_diagnostics["start_position_m"],
                                    source_diagnostics["end_position_m"]) <= 1.5
                            and max(source_diagnostics["start_heading_deg"],
                                    source_diagnostics["end_heading_deg"]) > 8.0)
                        # 整条连接线联合求解：端点 G2 精确闭合、来源双向偏差合格、
                        # 输出最多 5 段。禁止旧的“3 段桥 + 中段 + 3 段桥”碎片链。
                        fit = None
                        if p0 is not None and p1 is not None:
                            common_fit_args = {
                                "k0": (p0[3] if len(p0) > 3 else 0.0),
                                "k1": (p1[3] if len(p1) > 3 else 0.0),
                                "max_segments": 5,
                                "source_covers_endpoints": False,
                                # 连接路总长通常只有 25--60m；6m 下限可避免为了
                                # 贴点退化成逐点/米级碎片链。
                                "min_segment_m": 6.0,
                            }
                            # 先在严格来源保真域内按 3→4→5 的最少段数找解。
                            # 旧策略允许三段候选 P95 接近 1.45m 后立即返回，虽然
                            # 四段长回旋线常能把同一来源轨迹降到 0.2--0.4m。这里
                            # 仍不增加段数上限，也不降低最短段门限，只是不让“勉强
                            # 过宽门限的三段解”压过明显更准确的四/五段解。
                            fit = fit_connector_minimal(
                                vg, p0, p1,
                                median_tol=0.35, p95_tol=0.75,
                                max_dev_tol=1.50,
                                **common_fit_args)
                            if fit is None:
                                stats["conn_preferred_fidelity_unmet"] = (
                                    stats.get("conn_preferred_fidelity_unmet", 0) + 1)
                                fit = fit_connector_minimal(
                                    vg, p0, p1, **common_fit_args)
                        source_compare, source_support_s = vg, None
                        source_exclusion_code = None
                        if fit is not None:
                            prims = fit.primitives
                            dev = fit.metrics["source_to_target"]["max_m"]
                            source_support_s = fit.metrics.get("source_support_s")
                            stats["conn_minimal"] = stats.get("conn_minimal", 0) + 1
                            key = f"conn_minimal_{fit.metrics['n_primitives']}seg"
                            stats[key] = stats.get(key, 0) + 1
                            if source_state_conflict:
                                # 7m 弦方向不等于端点切线，半径 15m 的正常转弯也
                                # 会超过 8°。既然候选已满足 G2 端状态和来源硬容差，
                                # 此量只能作诊断；必须进入正式 G8，不能借它排除保真。
                                stats["conn_source_chord_heading_warning"] = stats.get(
                                    "conn_source_chord_heading_warning", 0) + 1
                                stats.setdefault("conn_source_conflicts", []).append({
                                    "source_lane": via.lane_pid,
                                    **source_diagnostics,
                                })
                        else:
                            # 无来源合格少段解时仍保住拓扑，但必须降级为明确排除的
                            # 三段 G2 合成，不得重新启用逐点/桥接碎片拟合。
                            fallback = (_g2_prims(p0, p1, with_meta=True)
                                        if p0 is not None and p1 is not None else None)
                            if fallback is None:
                                stats["conn_minimal_failed"] = \
                                    stats.get("conn_minimal_failed", 0) + 1
                                stats["skipped"] += 1
                                continue
                            prims, g2_tuning = fallback
                            dev = 0.0
                            source_exclusion_code = "minimal-chain-source-fidelity-unmet"
                            source_diagnostics = dict(source_diagnostics or {})
                            source_diagnostics["fallback_curve"] = g2_tuning
                            stats["conn_minimal_excluded"] = \
                                stats.get("conn_minimal_excluded", 0) + 1
                        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
                        filled_pairs.add((l.lane_pid,
                                          (out_rec.link_pid, out_rec.seq)))
                        connecting_road(("via", via.lane_pid), prims,
                                        (via.width_mm or 3500) / 1000.0,
                                        e_pid, out_rec.link_pid, in_xid, out_xid,
                                        source_rec=via, source_points=source_compare,
                                        source_support_s=source_support_s,
                                        source_exclusion_code=source_exclusion_code,
                                        source_diagnostics=source_diagnostics)
                        stats["conn_via"] += 1
                    else:                                # 直连：车道端位姿 SolveG2
                        p0 = lane_end.get(l.lane_pid)
                        p1 = lane_start.get((out_rec.link_pid, out_rec.seq))
                        if p0 is None or p1 is None:
                            stats["skipped"] += 1
                            continue
                        prims = _g2_prims(p0, p1)
                        if prims is None:
                            stats["skipped"] += 1
                            continue
                        filled_pairs.add((l.lane_pid,
                                          (out_rec.link_pid, out_rec.seq)))
                        connecting_road(("g2", l.lane_pid, out_rec.lane_pid), prims,
                                        (out_rec.width_mm or 3500) / 1000.0,
                                        e_pid, out_rec.link_pid, in_xid, out_xid)
                        stats["conn_g2"] += 1

    # —— 转向补全（connect_mode≠data）：源 TOPO 缺录时按几何补，标 INFERRED ——
    if connect_mode != "data":
        from mapforge.ops.junction_fill import plan_fill
        ent_meta, exit_meta = [], []
        for e_pid in enter_set:
            lanes = [x for x in src.lanes_of(e_pid)
                     if x.lane_pid in lane_end and xid_of[e_pid].get(x.seq)]
            lanes.sort(key=lambda l: -xid_of[e_pid][l.seq])   # -1 最左 → 断面序
            for i, l in enumerate(lanes):
                ent_meta.append({"key": l.lane_pid, "leg": e_pid, "idx": i,
                                 "n": len(lanes), "pose": lane_end[l.lane_pid],
                                 "pid": e_pid, "xid": xid_of[e_pid][l.seq],
                                 "lane": l})
        for x_pid in leave_set:
            lanes = [x for x in src.lanes_of(x_pid)
                     if (x_pid, x.seq) in lane_start and xid_of[x_pid].get(x.seq)]
            lanes.sort(key=lambda l: abs(xid_of[x_pid][l.seq]))   # |id| 小=靠中线
            for i, l in enumerate(lanes):
                exit_meta.append({"key": (x_pid, l.seq), "leg": x_pid, "idx": i,
                                  "n": len(lanes), "pose": lane_start[(x_pid, l.seq)],
                                  "pid": x_pid, "xid": xid_of[x_pid][l.seq],
                                  "lane": l})
        for e, x in plan_fill(ent_meta, exit_meta, filled_pairs,
                              mode=connect_mode, allow_uturn=allow_uturn):
            prims = _g2_prims(e["pose"], x["pose"])
            if prims is None or any(max(abs(p[5]), abs(p[6])) > 0.125 for p in prims):
                stats["conn_fill_skipped"] = stats.get("conn_fill_skipped", 0) + 1
                continue                                  # 无解或 R<8m：判为不可行转向
            connecting_road(("fill", e["key"], x["key"]), prims,
                            (e["lane"].width_mm or 3500) / 1000.0,
                            e["pid"], x["pid"], e["xid"], x["xid"],
                            exclusion_code="filled-connector-no-source-geometry")
            stats["conn_filled"] = stats.get("conn_filled", 0) + 1

    # —— junction 铺面：IBD 交叉口面实测轮廓（type=none 无标线沥青面，不入拓扑） ——
    from mapforge.adapters.opendrive.writer import add_paving_road
    from mapforge.ops.map_to_xodr import _mouth_apron_axes
    try:
        mouths = [{"pose": end_pose[p]} for p in enter_set if p in end_pose]
        axes = _mouth_apron_axes(mouths) or [None]
        paving_polygon = proj(junc.polygon)
        paved = 0
        for patch_i, axis in enumerate(axes[:2]):
            if add_paving_road(doc, paving_polygon, JID, road_id=90 + patch_i,
                    smooth_profile=True, preferred_axis=axis, overlap_m=0.5,
                    provenance={
                        "eligibility": "excluded", "role": "paving",
                        "status": "TRANSFORMED", "support_kind": "source-polygon",
                        "travel_direction": "with_s",
                        "exclusion_code": "source-polygon-paving"}):
                paved += 1
        if paved:
            stats["paving"] = "polygon"
            stats["paving_roads"] = paved
    except Exception as exc:
        stats["paving"] = "skip"
        stats["paving_error"] = f"{type(exc).__name__}: {exc}"

    doc.add_junction(junction)
    doc.write(out_path)
    # 先在绝对共享边界域中求解 SHP 中心线/边界证据，再精确物化为相邻
    # lane.width 之差：保持同一 reference line、laneSection 与 laneLink，
    # 同时兼容尚不支持 lane.border 的消费端（例如 esmini 3.6）。
    from mapforge.ops.lane_family_border import regularize_shp_lane_families
    stats.update(regularize_shp_lane_families(out_path))
    if mouth_policy == "source-envelope-candidate":
        from mapforge.ops.junction_surface import replace_source_paving
        root = ET.parse(out_path).getroot()
        try:
            stats.update(replace_source_paving(root, src, junc, proj,
                                              stats.get("mouth_envelope_decisions", [])))
            ET.indent(root)
            ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=True)
        except ValueError as exc:
            stats["source_surface_status"] = "FAIL"
            stats["source_surface_error"] = str(exc)
    for sid, pending in pending_sources.items():
        if sid not in manifest_lanes:
            _register_source(pending["rec"], pending["points"], role=pending["role"],
                             direction=pending["direction"],
                             at_stopline=pending["at_stopline"], eligible=False,
                             support_reason=pending["reason"])
    stats["source_lane_manifest"] = make_manifest(
        source_format="shp", source_profile=source_profile,
        comparison_crs={
            "id": "local-eqc", "units": "m", "axis_order": ["x", "y"],
            "origin": {"lon": lon0, "lat": lat0}, "proj_string": doc.geo_reference,
            "integrity": getattr(src, "p", {}).get("crs", {}).get(
                "verified", "internally-consistent"),
        },
        source_contexts=[{"junction": junc.pid, "name": junc.name}],
        lanes=[manifest_lanes[k] for k in sorted(manifest_lanes)],
    )
    if stats.get("source_surface_status") == "FAIL":
        raise CandidateSurfaceError(stats)
    return stats
