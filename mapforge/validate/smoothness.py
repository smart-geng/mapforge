# -*- coding: utf-8 -*-
"""xodr 平滑度审计：对生成文件（而非中间模型）做三层度量。

L1 参考线连续性：几何段边界位置/航向缝（planview_check 已覆盖，此处并入报表）；
L2 曲率连续性：段间 |Δκ|（line↔arc 交界为 G1 设计点；spiral 链/接缝应 G2）——
   换算等效半径与 60km/h 侧向加速度阶跃，量化"G1 而非 G2"的实际影响；
L3 车道级：laneSection 边界车道边界横向台阶（数据 S/E 宽不闭合会露出来）、
   junction 接缝"前驱车道中心端点 vs 连接路起点"的位置闭合。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np


def _geoms(road):
    out = []
    for g in road.findall("planView/geometry"):
        x, y, h, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
        if g.find("line") is not None:
            out.append(("line", x, y, h, L, 0.0, 0.0))
        elif g.find("arc") is not None:
            c = float(g.find("arc").get("curvature"))
            out.append(("arc", x, y, h, L, c, c))
        elif g.find("spiral") is not None:
            sp = g.find("spiral")
            out.append(("spiral", x, y, h, L,
                        float(sp.get("curvStart")), float(sp.get("curvEnd"))))
    return out


def sample_road_ref(road, ds=0.5):
    """参考线采样（line/arc/spiral），返回 (pts(n,2), s(n,), hdg(n,))。"""
    pts, ss, hh = [], [], []
    s0 = 0.0
    for kind, x, y, h, L, k0, k1 in _geoms(road):
        n = max(2, int(L / ds) + 1)
        u = np.linspace(0.0, L, n)
        if kind == "line":
            xs = x + u * math.cos(h)
            ys = y + u * math.sin(h)
            hs = np.full_like(u, h)
        elif kind == "arc":
            xs = x + (np.sin(h + k0 * u) - math.sin(h)) / k0
            ys = y - (np.cos(h + k0 * u) - math.cos(h)) / k0
            hs = h + k0 * u
        else:
            from pyclothoids import Clothoid
            cl = Clothoid.StandardParams(x, y, h, k0, (k1 - k0) / max(L, 1e-9), L)
            xs_l, ys_l = cl.SampleXY(n)
            xs, ys = np.asarray(xs_l), np.asarray(ys_l)
            hs = h + k0 * u + 0.5 * (k1 - k0) / max(L, 1e-9) * u * u
        pts.append(np.column_stack([xs, ys]))
        ss.append(s0 + u)
        hh.append(hs)
        s0 += L
    return np.vstack(pts), np.concatenate(ss), np.concatenate(hh)


def kappa_steps(road):
    """段间曲率跳变 |Δκ| 列表（G2 点为 0；line-arc 交界 = 设计 G1 点）。"""
    gs = _geoms(road)
    return [abs(gs[i + 1][5] - gs[i][6]) for i in range(len(gs) - 1)]


def _poly_eval(entries, s):
    """<laneOffset>/<width> 多项式族求值：entries=[(s0,a,b,c,d)]，取 s 所在段。"""
    e = None
    for cand in entries:
        if cand[0] <= s + 1e-9:
            e = cand
    if e is None:
        e = entries[0]
    ds = s - e[0]
    return e[1] + e[2] * ds + e[3] * ds ** 2 + e[4] * ds ** 3


def _poly_eval_derivative(entries, s, order=1):
    """求 OpenDRIVE 三次多项式的一/二阶导数。

    ``entries`` 的起点语义与 :func:`_poly_eval` 相同。这里显式计算导数，
    是为了审计最终车道边缘，而不是仅检查 laneSection 交界处的位置是否重合。
    """
    e = None
    for cand in entries:
        if cand[0] <= s + 1e-9:
            e = cand
    if e is None:
        e = entries[0]
    ds = s - e[0]
    if order == 1:
        return e[2] + 2.0 * e[3] * ds + 3.0 * e[4] * ds ** 2
    if order == 2:
        return 2.0 * e[3] + 6.0 * e[4] * ds
    raise ValueError(f"unsupported derivative order: {order}")


def _offsets(road):
    return [(float(o.get("s")), float(o.get("a")), float(o.get("b")),
             float(o.get("c")), float(o.get("d"))) for o in road.findall("lanes/laneOffset")] \
        or [(0.0, 0.0, 0.0, 0.0, 0.0)]


def _lane_widths(ln):
    return sorted((float(w.get("sOffset")), float(w.get("a")), float(w.get("b")),
                   float(w.get("c")), float(w.get("d"))) for w in ln.findall("width"))


def _lane_geometry(ln):
    """返回 (mode, polynomials)：border 为相对参考线的绝对外缘，width 为增量。"""
    borders = sorted((float(w.get("sOffset")), float(w.get("a")), float(w.get("b")),
                      float(w.get("c")), float(w.get("d"))) for w in ln.findall("border"))
    if borders:
        return "border", borders
    return "width", _lane_widths(ln)


def _sections(road):
    """[(s, right_lanes, left_lanes)]：right 按 -1..-n、left 按 +1..+n；
    每车道 [(sOffset,a,b,c,d), ...]（可多段 width，锥形收放）。"""
    out = []
    for sec in road.findall("lanes/laneSection"):
        right = sorted(((int(ln.get("id")), *_lane_geometry(ln))
                        for ln in sec.findall("right/lane")), key=lambda x: -x[0])
        left = sorted(((int(ln.get("id")), *_lane_geometry(ln))
                       for ln in sec.findall("left/lane")), key=lambda x: x[0])
        out.append((float(sec.get("s")), right, left))
    return out


def lane_edges_at(road, s, side="right"):
    """s 处一侧车道边界横向位置：均从 t0=laneOffset 起，right 向下累减、left 向上累加。"""
    offs = _offsets(road)
    secs = _sections(road)
    sec = None
    for cand in secs:
        if cand[0] <= s + 1e-9:
            sec = cand
    if sec is None:
        sec = secs[0]
    t = _poly_eval(offs, s)
    edges = [t]
    for _lid, mode, polynomials in (sec[1] if side == "right" else sec[2]):
        value = _poly_eval(polynomials, s - sec[0])       # 多段几何：按 sOffset 选段求值
        if mode == "border":
            t = value                                    # 相对参考线的绝对外边界
        else:
            t = t - value if side == "right" else t + value
        edges.append(t)
    return edges


def lane_edges_kinematics_at(road, s, side="right"):
    """返回横断面各边界的 ``(t, dt/ds, d2t/ds2)``。

    只看边界位置连续会漏掉用户截图中的“蛇形”：两个 section 的端点可以完全
    重合，但导数过大或二阶变化突变后，查看器里仍会形成鼓包、锯齿和来回摆动。
    该函数同时支持 ``width`` 累加与 ``border`` 绝对外缘两种 OpenDRIVE 表达。
    """
    offs = _offsets(road)
    secs = _sections(road)
    sec = None
    for cand in secs:
        if cand[0] <= s + 1e-9:
            sec = cand
    if sec is None:
        sec = secs[0]
    t = _poly_eval(offs, s)
    d1 = _poly_eval_derivative(offs, s, 1)
    d2 = _poly_eval_derivative(offs, s, 2)
    out = [(t, d1, d2)]
    sign = -1.0 if side == "right" else 1.0
    for _lid, mode, polynomials in (sec[1] if side == "right" else sec[2]):
        q = s - sec[0]
        value = _poly_eval(polynomials, q)
        value_d1 = _poly_eval_derivative(polynomials, q, 1)
        value_d2 = _poly_eval_derivative(polynomials, q, 2)
        if mode == "border":
            t, d1, d2 = value, value_d1, value_d2
        else:
            t += sign * value
            d1 += sign * value_d1
            d2 += sign * value_d2
        out.append((t, d1, d2))
    return out


def _ref_kappa_at(road, s):
    """返回参考线在 s 处的 (曲率, 曲率变化率)。"""
    cursor = 0.0
    geoms = _geoms(road)
    for _kind, _x, _y, _h, length, k0, k1 in geoms:
        if s <= cursor + length + 1e-9:
            u = min(max(s - cursor, 0.0), length)
            sharp = (k1 - k0) / max(length, 1e-9)
            return k0 + sharp * u, sharp
        cursor += length
    if not geoms:
        return 0.0, 0.0
    _kind, _x, _y, _h, length, k0, k1 = geoms[-1]
    return k1, (k1 - k0) / max(length, 1e-9)


def cross_edges_at(road, s):
    """s 处全断面边界（左外→右外），双侧路面连续性/渲染共用。"""
    left = lane_edges_at(road, s, side="left")
    right = lane_edges_at(road, s, side="right")
    return list(reversed(left)) + right[1:]


def section_boundary_steps(road):
    """laneSection 边界处车道边界台阶（全断面，含左侧）：一对一最近匹配（<1.5m），
    落单边界=生/灭车道（数据本义，不计台阶）。返回各边界最大匹配台阶 m。"""
    secs = _sections(road)
    steps = []

    def _dedupe(edges):
        """零宽车道产生重复边界——面判定按几何边界集合（去重）匹配。"""
        out = []
        for x in sorted(edges, reverse=True):
            if not out or abs(out[-1] - x) > 0.005:
                out.append(x)
        return out

    for (s_next, _r, _l) in secs[1:]:
        a = _dedupe(cross_edges_at(road, s_next - 1e-3))
        b = _dedupe(cross_edges_at(road, s_next + 1e-3))
        pairs = sorted((abs(x - y), i, j) for i, x in enumerate(a) for j, y in enumerate(b))
        ua, ub, matched = set(), set(), []
        for gap, i, j in pairs:
            if gap >= 1.5 or i in ua or j in ub:
                continue
            ua.add(i)
            ub.add(j)
            matched.append(gap)
        steps.append(max(matched) if matched else 0.0)
    return steps


def _edge_world_curvature(t, d1, d2, kappa, sharpness):
    """由 Frenet 边界 ``r(s)+t(s)n(s)`` 解析计算世界坐标曲率。"""
    along = 1.0 - kappa * t
    across = d1
    second_along = -sharpness * t - 2.0 * kappa * d1
    second_across = kappa * along + d2
    speed2 = along * along + across * across
    if speed2 <= 1e-12:
        return float("inf")
    return (along * second_across - across * second_along) / speed2 ** 1.5


def edge_shape_quality(road, ds=0.25):
    """审计最终世界坐标车道边缘，而非只审计参考线。

    指标覆盖三类曾在真实截图中出现、但旧门禁会漏掉的缺陷：

    * laneOffset/width 导数过大，导致道路横向蛇行；
    * 三次多项式二阶变化过大，导致 section 内鼓包或突然收放；
    * section 端点位置虽闭合但切向不闭合，导致道路边缘折角。

    返回值只描述普通物理 leg；junction connecting road 的高曲率转弯由 G3/G5/G7
    单独约束，不能套用干路外缘阈值。
    """
    length = float(road.get("length") or 0.0)
    secs = _sections(road)
    if length <= 1e-6 or not secs:
        return None

    slopes, seconds, curvatures = [], [], []
    outer_slopes, outer_seconds, outer_curvatures = [], [], []
    outer_runs = {"left": [], "right": []}

    for si, sec in enumerate(secs):
        s0 = max(0.0, float(sec[0]))
        s1 = float(secs[si + 1][0]) if si + 1 < len(secs) else length
        span = s1 - s0
        if span <= 1e-6:
            continue
        margin = min(0.05, span * 0.1)
        grid = np.arange(s0 + margin, s1 - margin + 1e-10, ds)
        if not len(grid):
            grid = np.asarray([(s0 + s1) / 2.0])
        for side in ("left", "right"):
            run = []
            for s in grid:
                kappa, sharpness = _ref_kappa_at(road, float(s))
                edges = lane_edges_kinematics_at(road, float(s), side)
                for t, d1, d2 in edges:
                    slopes.append(abs(d1))
                    seconds.append(abs(d2))
                    curvatures.append(abs(_edge_world_curvature(
                        t, d1, d2, kappa, sharpness)))
                t, d1, d2 = edges[-1]
                k_edge = _edge_world_curvature(t, d1, d2, kappa, sharpness)
                outer_slopes.append(abs(d1))
                outer_seconds.append(abs(d2))
                outer_curvatures.append(abs(k_edge))
                run.append(k_edge)
            outer_runs[side].append(run)

    # laneSection 边界导数门禁：位置使用既有一对一最近匹配，切向也按同一原则匹配。
    heading_steps = []
    outer_heading_steps = []
    for s_next, _r, _l in secs[1:]:
        eps = min(1e-4, max(float(s_next), length - float(s_next)) * 0.1)
        if eps <= 0.0:
            continue
        k0, _ = _ref_kappa_at(road, float(s_next) - eps)
        k1, _ = _ref_kappa_at(road, float(s_next) + eps)
        for side in ("left", "right"):
            before = lane_edges_kinematics_at(road, float(s_next) - eps, side)
            after = lane_edges_kinematics_at(road, float(s_next) + eps, side)
            if before and after and abs(before[-1][0] - after[-1][0]) < 1.5:
                ta, da, _ = before[-1]
                tb, db, _ = after[-1]
                ha = math.atan2(da, 1.0 - k0 * ta)
                hb = math.atan2(db, 1.0 - k1 * tb)
                dh = abs((hb - ha + math.pi) % (2.0 * math.pi) - math.pi)
                outer_heading_steps.append(math.degrees(dh))
            candidates = sorted((abs(a[0] - b[0]), i, j)
                                for i, a in enumerate(before)
                                for j, b in enumerate(after))
            used_a, used_b = set(), set()
            for gap, i, j in candidates:
                if gap >= 1.5 or i in used_a or j in used_b:
                    continue
                used_a.add(i)
                used_b.add(j)
                ta, da, _ = before[i]
                tb, db, _ = after[j]
                ha = math.atan2(da, 1.0 - k0 * ta)
                hb = math.atan2(db, 1.0 - k1 * tb)
                dh = abs((hb - ha + math.pi) % (2.0 * math.pi) - math.pi)
                heading_steps.append(math.degrees(dh))

    # 只统计“有量级的”外缘曲率换向，过滤接近零的数值抖动；每个 section 独立，
    # 端点切向由 heading_steps 负责，避免同一事件重复计数。
    flips = {"left": 0, "right": 0}
    for side, runs in outer_runs.items():
        for run in runs:
            for a, b in zip(run, run[1:]):
                if a * b < 0.0 and min(abs(a), abs(b)) > 0.003:
                    flips[side] += 1

    def _pct(values, q):
        return float(np.percentile(values, q)) if values else 0.0

    return {
        "road_id": road.get("id"),
        "length_m": length,
        "edge_lateral_slope_max": max(slopes, default=0.0),
        "edge_lateral_second_p95": _pct(seconds, 95),
        "edge_lateral_second_max": max(seconds, default=0.0),
        "edge_curvature_p95": _pct(curvatures, 95),
        "edge_curvature_max": max(curvatures, default=0.0),
        "outer_lateral_slope_max": max(outer_slopes, default=0.0),
        "outer_lateral_second_p95": _pct(outer_seconds, 95),
        "outer_lateral_second_max": max(outer_seconds, default=0.0),
        "outer_curvature_p95": _pct(outer_curvatures, 95),
        "outer_curvature_max": max(outer_curvatures, default=0.0),
        "outer_curvature_flips_per_100m_max": (
            max(flips.values(), default=0) / max(length, 1e-9) * 100.0),
        "edge_heading_step_max_deg": max(heading_steps, default=0.0),
        "outer_edge_heading_step_max_deg": max(outer_heading_steps, default=0.0),
    }


def edge_shape_audit(root):
    """汇总文件中所有普通道路的最终边缘形态，并保留最坏 road 便于定位。"""
    rows = [q for q in (
        edge_shape_quality(road) for road in root.findall("road")
        if road.get("junction") in (None, "-1")
        and road.get("name") != "junction_paving") if q]
    keys = (
        "edge_lateral_slope_max", "edge_lateral_second_p95",
        "edge_lateral_second_max", "edge_curvature_p95", "edge_curvature_max",
        "outer_lateral_slope_max", "outer_lateral_second_p95",
        "outer_lateral_second_max", "outer_curvature_p95", "outer_curvature_max",
        "outer_curvature_flips_per_100m_max", "edge_heading_step_max_deg",
        "outer_edge_heading_step_max_deg",
    )
    out = {"edge_shape_roads": len(rows), "edge_shape_per_road": rows}
    for key in keys:
        worst = max(rows, key=lambda q: q[key]) if rows else None
        out[key] = worst[key] if worst else 0.0
        out[f"{key}_road"] = worst["road_id"] if worst else None
    return out


def junction_seam_gaps(root):
    """连接路起点 vs 前驱路对应车道中心端点的位置闭合（m）。"""
    roads = {rd.get("id"): rd for rd in root.findall("road")}
    gaps = []
    for rd in root.findall("road"):
        if rd.get("junction") in (None, "-1"):
            continue
        pred = rd.find("link/predecessor")
        gs = _geoms(rd)
        if pred is None or not gs:
            continue
        start = np.array([gs[0][1], gs[0][2]])
        prd = roads[pred.get("elementId")]
        # 前驱路末端位姿
        pts, ss, hh = sample_road_ref(prd, ds=0.5)
        pe, he = pts[-1], hh[-1]
        s_end = ss[-1]
        # 连接路 laneLink from → 前驱车道中心 t
        conn_el = None
        for c in root.findall("junction/connection"):
            if c.get("connectingRoad") == rd.get("id"):
                conn_el = c
                break
        if conn_el is None:
            continue
        from_id = int(conn_el.find("laneLink").get("from"))
        edges = lane_edges_at(prd, s_end - 1e-3)
        k = -from_id                                     # 1-based
        t = (edges[k - 1] + edges[k]) / 2 if k < len(edges) else edges[-1]
        lane_pt = pe + t * np.array([-math.sin(he), math.cos(he)])
        gaps.append(float(np.linalg.norm(start - lane_pt)))
    return gaps


def lane_endpoint_state(road, lane_id, contact, *, forward=True):
    """Read exact world position/tangent/curvature from the written lane.

    Lane IDs are identities, not boundary-array indices. No endpoint epsilon,
    constant-width assumption, or reference-line-as-lane shortcut is used.
    Only the supported line/arc/spiral consumer profile is evaluated here.
    """
    if contact not in ('start', 'end') or lane_id == 0:
        raise ValueError('invalid lane endpoint')
    geoms = _geoms(road)
    if not geoms or len(geoms) != len(road.findall('planView/geometry')):
        raise ValueError('unsupported endpoint reference geometry')
    kind, x, y, h, length, k0, k1 = geoms[0 if contact == 'start' else -1]
    sharp = (k1-k0)/length
    kappa = k0
    if contact == 'end':
        from pyclothoids import Clothoid
        curve = Clothoid.StandardParams(x, y, h, k0, sharp, length)
        x, y, h, kappa = curve.XEnd, curve.YEnd, curve.ThetaEnd, curve.KappaEnd
    s = 0. if contact == 'start' else float(road.get('length'))
    sections = _sections(road)
    if not sections:
        raise ValueError('missing lane sections')
    section = sections[0 if contact == 'start' else -1]
    side = 'left' if lane_id > 0 else 'right'
    lane_ids = [l[0] for l in section[2 if lane_id > 0 else 1]]
    if lane_id not in lane_ids:
        raise ValueError(f'road {road.get("id")}: endpoint lane {lane_id} absent')
    i = lane_ids.index(lane_id)
    edges = lane_edges_kinematics_at(road, s, side)
    t, dt, ddt = [(a+b)/2 for a, b in zip(edges[i], edges[i+1])]
    denominator = 1-kappa*t
    if denominator*denominator+dt*dt <= 1e-12:
        raise ValueError('singular lane endpoint')
    curvature = _edge_world_curvature(t, dt, ddt, kappa, sharp)
    state = {'x': float(x-t*math.sin(h)), 'y': float(y+t*math.cos(h)),
             'heading': float(h+math.atan2(dt, denominator)+(0. if forward else math.pi)),
             'curvature': float(curvature if forward else -curvature)}
    if not all(math.isfinite(v) for v in state.values()):
        raise ValueError('non-finite lane endpoint')
    return state


def junction_lane_interfaces(root):
    """Every explicit junction laneLink, both entry and exit, in travel direction.

    Missing topology or unsupported geometry is an error row, never a skipped
    success. Handles either connecting-road contact and either lane side.
    """
    from functools import lru_cache
    roads = {r.get('id'): r for r in root.findall('road')}
    rows = []
    @lru_cache(maxsize=None)
    def state(rid, lid, contact, forward):
        return lane_endpoint_state(roads[rid], lid, contact, forward=forward)
    def compare(meta, a, b):
        return dict(meta, position_m=math.hypot(a['x']-b['x'], a['y']-b['y']),
                    heading_deg=math.degrees(abs((a['heading']-b['heading']+math.pi)%(2*math.pi)-math.pi)),
                    curvature_per_m=abs(a['curvature']-b['curvature']))
    for conn in root.findall('junction/connection'):
        cid, incoming = conn.get('connectingRoad'), conn.get('incomingRoad')
        contact = conn.get('contactPoint')
        for lane_link in conn.findall('laneLink'):
            meta = {'connection_id': conn.get('id'), 'connecting_road': cid,
                    'incoming_road': incoming, 'from_lane': lane_link.get('from'),
                    'connecting_lane': lane_link.get('to')}
            try:
                if contact not in ('start', 'end'):
                    raise ValueError('missing/invalid connecting contact')
                cr = roads[cid]; forward = contact == 'start'
                start_role, end_role = ('predecessor', 'successor') if forward else ('successor', 'predecessor')
                entry = cr.find('link/'+start_role)
                if entry is None or entry.get('elementType') != 'road' or entry.get('elementId') != incoming:
                    raise ValueError('connecting entry does not reference incoming road')
                pc = entry.get('contactPoint')
                source_id, conn_id = int(lane_link.get('from')), int(lane_link.get('to'))
                a = state(incoming, source_id, pc, pc == 'end')
                b = state(cid, conn_id, contact, forward)
                rows.append(compare(dict(meta, interface='entry'), a, b))
                exit_contact = 'end' if forward else 'start'
                exit_link = cr.find('link/'+end_role)
                if exit_link is None or exit_link.get('elementType') != 'road':
                    raise ValueError('missing connecting exit road')
                sec = cr.findall('lanes/laneSection')[-1 if forward else 0]
                # Different IDs across sections require following explicit links.
                sections = cr.findall('lanes/laneSection')
                track_id = conn_id
                for si in (range(len(sections)-1) if forward else range(len(sections)-1, 0, -1)):
                    ln = sections[si].find(f"{'left' if track_id > 0 else 'right'}/lane[@id='{track_id}']")
                    nxt = ln.find('link/'+end_role) if ln is not None else None
                    if nxt is None:
                        raise ValueError('connecting lane track is not linked across sections')
                    track_id = int(nxt.get('id'))
                clane = sec.find(f"{'left' if track_id > 0 else 'right'}/lane[@id='{track_id}']")
                next_lane = clane.find('link/'+end_role) if clane is not None else None
                if next_lane is None:
                    raise ValueError('missing connecting exit lane link')
                target, tc = exit_link.get('elementId'), exit_link.get('contactPoint')
                a = state(cid, track_id, exit_contact, forward)
                b = state(target, int(next_lane.get('id')), tc, tc == 'start')
                rows.append(compare(dict(meta, interface='exit', outgoing_road=target), a, b))
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                rows.append(dict(meta, interface='unresolved', error=str(exc)))
    return rows


def route_continuity(root):
    """Backward-compatible per-connector report, now evaluated at exact endpoints."""
    grouped = {}
    for row in junction_lane_interfaces(root):
        if 'error' in row:
            raise ValueError(row['error'])
        key = (row['connecting_road'], row['from_lane'], row['connecting_lane'])
        item = grouped.setdefault(key, {'conn': row['connecting_road']})
        suffix = 'in' if row['interface'] == 'entry' else 'out'
        item['gap_'+suffix] = row['position_m']
        item['dh_'+suffix+'_deg'] = row['heading_deg']
        item['dk_'+suffix+'_per_m'] = row['curvature_per_m']
    return list(grouped.values())


def road_surface_polygon(road, ds=0.1):
    """按消费端的参考线、laneOffset 与车道宽度重建整条 road 的可见二维外轮廓。

    与中心线/车道接缝门禁不同，这里检查的是**路面并集**。只取全断面最左/
    最右外缘，采样网格显式包含 laneSection 边界。
    """
    from shapely.geometry import Polygon

    pts, ss, hh = sample_road_ref(road, ds)
    if len(ss) < 2:
        return Polygon()
    L = float(ss[-1])
    grid = set(np.linspace(0.0, L, max(2, int(L / ds) + 2)).tolist())
    grid.update(float(x.get("s")) for x in road.findall("lanes/laneSection"))
    u = np.asarray(sorted(x for x in grid if 0.0 <= x <= L), float)
    px = np.interp(u, ss, pts[:, 0])
    py = np.interp(u, ss, pts[:, 1])
    ph = np.interp(u, ss, hh)
    p = np.column_stack([px, py])
    nrm = np.column_stack([-np.sin(ph), np.cos(ph)])
    outer = [cross_edges_at(road, min(float(s), L - 1e-7)) for s in u]
    left_t = np.asarray([x[0] for x in outer], float)
    right_t = np.asarray([x[-1] for x in outer], float)
    left = p + left_t[:, None] * nrm
    right = p + right_t[:, None] * nrm
    return Polygon(np.vstack([left, right[::-1]])).buffer(0)


def surface_continuity(root) -> dict:
    """junction 铺面与普通 leg 的二维表面闭合门禁。

    参考线/车道连接门禁无法发现查看器中的细缝和尖点。本门禁额外要求每个铺面
    road 自身及铺面并集都单连通无孔，并与每条外部 leg 有确定的面积重叠，
    不能只在一条浮点边界上相切。
    """
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    paves, legs = [], []
    for road in root.findall("road"):
        pg = road_surface_polygon(road)
        if pg.is_empty:
            continue
        if road.get("name") == "junction_paving":
            paves.append(pg)
        elif road.get("junction") in (None, "-1"):
            legs.append(pg)
    if not paves:
        return {"paving_roads": 0, "paving_components": 0,
                "paving_road_components_max": 0, "paving_holes_gt1cm2": 0,
                "paving_hole_area_max": 0.0, "paving_leg_overlap_min": 0.0,
                "paving_leg_overlaps": []}
    pu = unary_union(paves)
    geoms = list(getattr(pu, "geoms", [pu]))
    holes = [abs(float(Polygon(ring).area))
             for g in geoms for ring in getattr(g, "interiors", [])]
    overlaps = [float(pu.intersection(g).area) for g in legs]
    return {
        "paving_roads": len(paves),
        "paving_road_components_max": max(
            len(getattr(g, "geoms", [g])) for g in paves),
        "paving_components": len(geoms),
        "paving_holes_gt1cm2": sum(x > 0.01 for x in holes),
        "paving_hole_area_max": max(holes) if holes else 0.0,
        "paving_leg_overlap_min": min(overlaps) if overlaps else 0.0,
        "paving_leg_overlaps": overlaps,
    }


def audit_file(path) -> dict:
    root = ET.parse(str(path)).getroot()
    ks_all, steps_all = [], []
    for rd in root.findall("road"):
        ks_all.extend(kappa_steps(rd))
        # 断面台阶对**所有** road 检查（含路口铺面）——铺面也是交付路面，
        # 不该因为"不可行车"就免检；连接路只有单 section，天然无贡献
        steps_all.extend(section_boundary_steps(rd))
    gaps = junction_seam_gaps(root)
    ks = [k for k in ks_all if k > 1e-9]
    out = {
        "kappa_steps": len(ks),
        "kappa_step_max": max(ks) if ks else 0.0,
        "kappa_step_med": float(np.median(ks)) if ks else 0.0,
        "g2_joints": len(ks_all) - len(ks),
        "lane_edge_step_max": max(steps_all) if steps_all else 0.0,
        "lane_edge_steps_gt5cm": sum(1 for x in steps_all if x > 0.05),
        "section_boundaries": len(steps_all),
        "seam_gap_max": max(gaps) if gaps else 0.0,
        "seam_gap_med": float(np.median(gaps)) if gaps else 0.0,
        "seams": len(gaps),
    }
    out.update(surface_continuity(root))
    out.update(edge_shape_audit(root))
    return out


def curvature_quality(road, v_kmh: float | None = None):
    """曲率品质（"碎段拼接假平滑"的真正判据）。

    段长不是唯一判据——有设计意义的短 clothoid 可以平滑；但生成器用 0.xm
    碎段承载大 Δκ 是明确缺陷。故调用侧应把最短段硬下限与**曲率蛇行**联合：
    sharpness(dκ/ds) 高频变号 ⇒ 方向盘来回微抖，大 |dκ/ds| ⇒ 侧向 jerk 超标。

    v_kmh 缺省按路类取：junction 连接路 30km/h（路口内转弯），普通路 60km/h。
    返回 {sharp_sign_flips_per_100m, sharpness_max, jerk_max, jerk_p95,
          kappa_max, seg_median_len, seg_min_len, n_segs}。"""
    gs = _geoms(road)
    if not gs:
        return None
    if v_kmh is None:
        v_kmh = 30.0 if road.get("junction") not in (None, "-1") else 60.0
    v = v_kmh / 3.6
    sharp, lens = [], []
    for kind, _x, _y, _h, L, k0, k1 in gs:
        lens.append(L)
        sharp.append(0.0 if L <= 1e-9 else (k1 - k0) / L)
    total = sum(lens) or 1.0
    flips = sum(1 for a, b in zip(sharp, sharp[1:])
                if a * b < 0 and min(abs(a), abs(b)) > 1e-6)
    jerk = [abs(s) * v ** 3 for s in sharp]               # v³·dκ/ds = 侧向 jerk
    return {"sharp_sign_flips_per_100m": flips / total * 100.0,
            "sharpness_max": max((abs(x) for x in sharp), default=0.0),
            "jerk_max": max(jerk) if jerk else 0.0,
            "jerk_p95": float(np.percentile(jerk, 95)) if jerk else 0.0,
            "kappa_max": max(max(abs(g[5]), abs(g[6])) for g in gs),
            "seg_median_len": float(np.median(lens)),
            "seg_min_len": float(min(lens)),
            "n_segs": len(lens)}


def curvature_audit(root):
    """整文件曲率品质汇总（leg / conn 分开——两者速度与几何诉求不同）。"""
    out = {}
    for tag, pred in (("leg", lambda r: r.get("junction") in (None, "-1")),
                      ("conn", lambda r: r.get("junction") not in (None, "-1")
                       and r.get("name") != "junction_paving")):
        rows = [q for q in (curvature_quality(rd) for rd in root.findall("road")
                            if pred(rd)) if q]
        if not rows:
            continue
        lens = [q["seg_median_len"] for q in rows]
        out[tag] = {
            "flips_per_100m_max": max(q["sharp_sign_flips_per_100m"] for q in rows),
            "sharpness_max": max(q["sharpness_max"] for q in rows),
            "jerk_max": max(q["jerk_max"] for q in rows),
            "seg_median_len": float(np.median(lens)),
            "seg_min_len": min(q["seg_min_len"] for q in rows),
            "n_segs": sum(q["n_segs"] for q in rows),
        }
    return out
