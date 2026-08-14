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


def _offsets(road):
    return [(float(o.get("s")), float(o.get("a")), float(o.get("b")),
             float(o.get("c")), float(o.get("d"))) for o in road.findall("lanes/laneOffset")] \
        or [(0.0, 0.0, 0.0, 0.0, 0.0)]


def _lane_widths(ln):
    return sorted((float(w.get("sOffset")), float(w.get("a")), float(w.get("b")),
                   float(w.get("c")), float(w.get("d"))) for w in ln.findall("width"))


def _sections(road):
    """[(s, right_lanes, left_lanes)]：right 按 -1..-n、left 按 +1..+n；
    每车道 [(sOffset,a,b,c,d), ...]（可多段 width，锥形收放）。"""
    out = []
    for sec in road.findall("lanes/laneSection"):
        right = sorted(((int(ln.get("id")), _lane_widths(ln))
                        for ln in sec.findall("right/lane")), key=lambda x: -x[0])
        left = sorted(((int(ln.get("id")), _lane_widths(ln))
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
    for _lid, wpolys in (sec[1] if side == "right" else sec[2]):
        w = _poly_eval(wpolys, s - sec[0])               # 多段 width：按 sOffset 选段求值
        t = t - w if side == "right" else t + w
        edges.append(t)
    return edges


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


def route_continuity(root):
    """虚拟行车换乘检查（任何仿真器消费本文件的方式）：
    进口车道中心 → 连接路 → 出口车道中心，逐连接路给出两个换乘点的位置/航向跳变。"""
    roads = {rd.get("id"): rd for rd in root.findall("road")}
    out = []

    def _nrm(h):
        return np.array([-math.sin(h), math.cos(h)])

    for conn in root.findall("junction/connection"):
        crd = roads.get(conn.get("connectingRoad"))
        prd = roads.get(conn.get("incomingRoad"))
        if crd is None or prd is None:
            continue
        from_id = int(conn.find("laneLink").get("from"))
        # —— 换乘 A：进口车道中心末点 → 连接路起点 ——
        pts, ss, hh = sample_road_ref(prd, 0.5)
        pe, he, s_end = pts[-1], hh[-1], ss[-1]
        edges = lane_edges_at(prd, s_end - 1e-3)
        k = min(-from_id, len(edges) - 1)
        lane_pt = pe + (edges[k - 1] + edges[k]) / 2 * _nrm(he)
        gs = _geoms(crd)
        gap_in = float(np.linalg.norm(np.array([gs[0][1], gs[0][2]]) - lane_pt))
        dh_in = abs((gs[0][3] - he + math.pi) % (2 * math.pi) - math.pi)
        # —— 换乘 B：连接路末端（其参考线即车道中心） → 出口车道中心 ——
        # 出口在双向 leg road 上时：contactPoint=end、车道 id 为正（左侧，行车沿 s 递减）
        succ = crd.find("link/successor")
        gap_out = dh_out = float("nan")
        if succ is not None and succ.get("elementId") in roads:
            srd = roads[succ.get("elementId")]
            cpts, _cs, chh = sample_road_ref(crd, 0.5)
            ce, che = cpts[-1], chh[-1]
            lk2 = crd.find("lanes/laneSection/right/lane/link/successor")
            k2 = int(lk2.get("id")) if lk2 is not None else -1
            spts, sss, shh = sample_road_ref(srd, 0.5)
            at_end = succ.get("contactPoint", "start") == "end"
            pe2 = spts[-1] if at_end else spts[0]
            he2 = shh[-1] if at_end else shh[0]
            s_eval = (sss[-1] - 1e-3) if at_end else 1e-3
            if k2 > 0:
                sedges = lane_edges_at(srd, s_eval, side="left")
                kk = min(k2, len(sedges) - 1)
                travel_h = he2 + math.pi                 # 左侧车道行车方向与 s 相反
            else:
                sedges = lane_edges_at(srd, s_eval, side="right")
                kk = min(-k2, len(sedges) - 1)
                travel_h = he2
            target = pe2 + (sedges[kk - 1] + sedges[kk]) / 2 * _nrm(he2)
            gap_out = float(np.linalg.norm(ce - target))
            dh_out = abs((travel_h - che + math.pi) % (2 * math.pi) - math.pi)
        out.append({"conn": crd.get("id"), "gap_in": gap_in, "gap_out": gap_out,
                    "dh_in_deg": math.degrees(dh_in), "dh_out_deg": math.degrees(dh_out)})
    return out


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
    return {
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
