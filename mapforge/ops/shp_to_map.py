# -*- coding: utf-8 -*-
"""SHP(IBD)→MAP 重塑核心（方向 1 量产主线，方案 5.1–5.4 的 IBD 直读实现）。

- 交叉口发现：refPos → 最近 INTERSECTION_SURFACE（EXACT）；
- Link：ENTER_ROAD 直读 + 驶入方位 ASCII 命名（IA5String 约束）+ 上游回溯拼接（TOPO 反向 + 端点衔接）；
- connectsTo：TOPO 两跳直译（进口车道→路口内连接车道→出口车道，IBD 三层模型）；
- maneuver：路口内连接车道几何首末航向差；
- phase：仅消费外部 phase_table（(link方位名, maneuver位串)→phaseId），缺项保持 None（FORBIDDEN_AUTO）。
"""
from __future__ import annotations

import math

import numpy as np

from mapforge.adapters.shp.ibd_reader import IbdSource
from mapforge.adapters.v2xmap.xml_reader import MapNode, MapLink, MapLane, Connection
from mapforge.ops.simplify import simplify_appendix_d

R_EARTH = 6378137.0


def _proj(pts_deg, lat0, lon0):
    pts = np.asarray(pts_deg, dtype=float)
    x = np.radians(pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def _to_lonlat(pts_xy, lat0, lon0):
    return [(lon0 + math.degrees(x / (R_EARTH * math.cos(math.radians(lat0)))),
             lat0 + math.degrees(y / R_EARTH)) for x, y in pts_xy]


def _orient(pts, target_xy):
    if np.linalg.norm(pts[0] - target_xy) < np.linalg.norm(pts[-1] - target_xy):
        return pts[::-1]
    return pts


def _heading(pts, at_end):
    a, b = (pts[-2], pts[-1]) if at_end else (pts[0], pts[1])
    return math.atan2(b[1] - a[1], b[0] - a[0])


def approach_name(h_in: float) -> str:
    came_from = (math.degrees(h_in) + 180.0) % 360.0
    if came_from < 45 or came_from >= 315:
        return "east"
    if came_from < 135:
        return "north"
    if came_from < 225:
        return "west"
    return "south"


def maneuver_bits(dh: float) -> str:
    deg = math.degrees((dh + math.pi) % (2 * math.pi) - math.pi)
    if abs(deg) < 30:
        return "100000000000"
    if abs(deg) > 150:
        return "000100000000"
    return "010000000000" if deg > 0 else "001000000000"


def phase_table_from_xml(ref: MapNode) -> dict:
    """演示配时通道：从现网 XML 提取 (link方位名, maneuver位串) → phaseId。正式通道为配时表模板。"""
    tab = {}
    for lk in ref.links:
        for ln in lk.lanes:
            for c in ln.connects:
                if c.phase is not None and c.maneuver:
                    tab[(lk.name, c.maneuver.strip())] = c.phase
    return tab


def id_table_from_xml(ref: MapNode) -> dict:
    """台账 inherit-as-is 策略：从现网 XML 继承 ID——
    upstream: {link方位名: (region,id)}；remote: {(link方位名, maneuver位串): (region,id)}。"""
    up = {lk.name: lk.upstream for lk in ref.links}
    remote = {}
    for lk in ref.links:
        for ln in lk.lanes:
            for c in ln.connects:
                if c.maneuver and c.node is not None:
                    remote.setdefault((lk.name, c.maneuver.strip()), (c.region, c.node))
    return {"upstream": up, "remote": remote}


def rebuild_from_ibd(src: IbdSource, lon: float, lat: float, *, region: int, node_id: int,
                     phase_table: dict | None = None, id_table: dict | None = None,
                     join_gap: float = 5.0, max_link_len: float = 160.0, tol: float = 0.30):
    """IBD 交付 → MapNode。返回 (MapNode, summary dict)。"""
    junc, dist = src.find_junction(lon, lat)
    lat0, lon0 = lat, lon
    center_xy = _proj(junc.polygon, lat0, lon0).mean(axis=0)
    out_link_id = {lpid: i + 1 for i, lpid in enumerate(junc.leave_roads)}
    broken = []

    def extend_up(lane_rec):
        g = _orient(_proj(lane_rec.geometry, lat0, lon0), center_xy)
        total = float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum())
        cur, hops = lane_rec, 0
        while total < max_link_len and hops < 6:
            cands, gaps = [], []
            for up in src.topo_in.get(cur.lane_pid, []):
                r2 = src.lane(up)
                if r2 is None or r2.geometry.shape[0] < 2 or r2.link_pid == cur.link_pid:
                    continue
                g2 = _orient(_proj(r2.geometry, lat0, lon0), g[0:1].mean(axis=0))
                gap = float(np.linalg.norm(g2[-1] - g[0]))
                gaps.append(gap)
                if gap < join_gap:
                    cands.append((gap, r2, g2))
            if not cands:
                if total < max_link_len * 0.6:
                    broken.append({"lane": lane_rec.lane_pid, "len_m": round(total),
                                   "nearest_gap_m": round(min(gaps), 1) if gaps else None})
                break
            _, cur, g2 = min(cands, key=lambda c: c[0])
            g = np.vstack([g2[:-1], g])
            total = float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum())
            hops += 1
        return g

    node = MapNode(name=f"gen_node_{node_id}", region=region, node_id=node_id)
    node.ref_lon, node.ref_lat = lon0, lat0
    node.msg_cnt = 0
    lane_geo = {}
    n_conn = n_drop = 0
    for lpid in junc.enter_roads:
        rl = src.roadlinks.get(lpid)
        lanes = [l for l in src.lanes_of(lpid) if l.geometry.shape[0] >= 2]
        if rl is None or not lanes:
            continue
        link_xy = extend_up(lanes[0])                    # Link 中线：车道1 拼接线近似（正式化：车道组中线）
        idx = simplify_appendix_d(link_xy, tol)
        aname = approach_name(_heading(link_xy, at_end=True))
        upstream = (id_table or {}).get("upstream", {}).get(aname, (region, 0))
        link = MapLink(name=aname, upstream=upstream,
                       width_cm=int(sum(l.width_mm for l in lanes) / 10))
        link._src_pid = lpid
        link.points = _to_lonlat(link_xy[idx], lat0, lon0)
        for l in lanes:
            g = extend_up(l)
            lane = MapLane(lane_id=l.seq, width_cm=l.width_mm // 10, maneuvers=None)
            lane._src_pid = l.lane_pid
            lane_geo[(len(node.links), l.seq)] = g
            lane.points = _to_lonlat(g[simplify_appendix_d(g, tol)], lat0, lon0)
            reached = set()
            for out1 in src.topo_out.get(l.lane_pid, []):
                mid = src.lane(out1)
                if mid is None or mid.geometry.shape[0] < 2:
                    n_drop += 1
                    continue
                targets = []
                if mid.link_pid in out_link_id:
                    targets.append((mid, None))
                else:
                    for out2 in src.topo_out.get(out1, []):
                        rec2 = src.lane(out2)
                        if rec2 is not None and rec2.link_pid in out_link_id:
                            targets.append((rec2, mid))
                if not targets:
                    n_drop += 1
                    continue
                for out_rec, via in targets:
                    key = (out_rec.link_pid, out_rec.seq)
                    if key in reached:
                        continue
                    reached.add(key)
                    if via is not None:
                        vg = _orient(_proj(via.geometry, lat0, lon0), center_xy)[::-1]
                        man = maneuver_bits(_heading(vg, at_end=True) - _heading(vg, at_end=False))
                    else:
                        man = "100000000000"
                    r_reg, r_node = (id_table or {}).get("remote", {}).get(
                        (link.name, man), (region, out_link_id[out_rec.link_pid]))
                    lane.connects.append(Connection(region=r_reg, node=r_node,
                                                    lane=out_rec.seq, maneuver=man, phase=None))
                    lane.maneuvers = format(int(lane.maneuvers or "0" * 12, 2) | int(man, 2), "012b")
                    n_conn += 1
            link.lanes.append(lane)
        node.links.append(link)

    n_ph = n_ph_miss = 0
    if phase_table:
        for glk in node.links:
            for ln in glk.lanes:
                for c in ln.connects:
                    ph = phase_table.get((glk.name, c.maneuver))
                    if ph is not None:
                        c.phase = ph
                        n_ph += 1
                    else:
                        n_ph_miss += 1

    summary = {"junction": junc.name, "junction_pid": junc.pid, "ref_dist_m": round(dist, 1),
               "links": len(node.links), "lanes": sum(len(l.lanes) for l in node.links),
               "connects": n_conn, "topo_dropped": n_drop,
               "phase_bound": n_ph, "phase_missing": n_ph_miss, "broken_chains": broken}
    return node, summary, lane_geo
