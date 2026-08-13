# -*- coding: utf-8 -*-
"""交叉口重塑：OpenDRIVE junction → MAP Node-Link-Lane（方案 5.1–5.3 的 M0 实现）。

- Link = 每条 incoming road 的驶入侧车道组（驶入端由 laneLink.from 符号判定）；
  Link 点列 = 参考线（道路中线近似）按驶入方向排列，末点 = 与 junction 的接触端（≈停止线，附录 D 端点规则）；
- connectsTo = junction laneLink 直译（incoming lane → connecting road → outgoing road），
  maneuver 由 connecting road 首末航向差判别（EXACT 拓扑 + TRANSFORMED 几何判别）；
- remoteIntersection / upstreamNode 使用虚拟台账 ID（road id → 虚拟 node id，输出映射表——
  正式版从 ledger 消费，禁止发明 ID 进生产，此处为 M0 演示映射并显式导出）；
- 抽稀走附录 D（simplify_appendix_d），平面坐标经 geoReference 锚点逆投影为经纬度。
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from mapforge.adapters.opendrive.reader import (OdrMap, Road, sample_reference_line,
                                                offset_polyline, lane_center_t)
from mapforge.adapters.v2xmap.xml_reader import MapNode, MapLink, MapLane, Connection
from mapforge.ops.simplify import simplify_appendix_d

R_EARTH = 6378137.0


@dataclass
class RebuildReport:
    junction_id: str = ""
    virtual_nodes: dict[str, int] = field(default_factory=dict)   # road_id → 虚拟 node id
    links: list[str] = field(default_factory=list)
    n_conn: int = 0
    n_dropped_conn: int = 0
    notes: list[str] = field(default_factory=list)


def _anchor_from_georef(geo: str) -> tuple[float, float]:
    lat = re.search(r"\+lat_0=([0-9.eE+-]+)", geo)
    lon = re.search(r"\+lon_0=([0-9.eE+-]+)", geo)
    return (float(lat.group(1)) if lat else 0.0, float(lon.group(1)) if lon else 0.0)


def _to_lonlat(pts: np.ndarray, lat0: float, lon0: float):
    lon = lon0 + np.degrees(pts[:, 0] / (R_EARTH * math.cos(math.radians(lat0))))
    lat = lat0 + np.degrees(pts[:, 1] / R_EARTH)
    return list(zip(lon.tolist(), lat.tolist()))


def _maneuver_bits(dh: float) -> str:
    """connecting road 首末航向差（弧度，左正）→ 12 位 maneuvers 串。"""
    deg = math.degrees((dh + math.pi) % (2 * math.pi) - math.pi)
    if abs(deg) < 30:
        return "100000000000"          # straight
    if abs(deg) > 150:
        return "000100000000"          # uTurn
    return "010000000000" if deg > 0 else "001000000000"   # left / right


def _incoming_dirs(odr: OdrMap, junction_id: str):
    """{incoming_road_id: 驶入端('end'|'start')}——由 laneLink.from 符号判定（from<0 右车道→s 正向驶入=end）。"""
    dirs = {}
    j = odr.junctions[junction_id]
    for c in j.connections:
        for ll in c.lane_links:
            dirs.setdefault(c.incoming, "end" if ll.frm < 0 else "start")
    return dirs


def _other_end_target(odr: OdrMap, conn_road: Road, contact: str):
    """connecting road 非接触端连接的元素 → ('road'|'junction', id)。"""
    ends = {"start": conn_road.pred, "end": conn_road.succ}
    other = ends["end" if contact == "start" else "start"]
    return (other[0], other[1]) if other else (None, None)


def rebuild_junction(odr: OdrMap, junction_id: str, *, region: int, node_id: int,
                     step: float = 1.0, tol: float = 0.30,
                     max_link_len: float = 200.0) -> tuple[MapNode, RebuildReport]:
    rep = RebuildReport(junction_id=junction_id)
    j = odr.junctions[junction_id]
    dirs = _incoming_dirs(odr, junction_id)

    # 虚拟台账：为每条 incoming/outgoing road 的"远端"分配虚拟 node id（演示用，正式走 ledger）
    next_vid = 1
    def vid(road_id: str) -> int:
        nonlocal next_vid
        if road_id not in rep.virtual_nodes:
            rep.virtual_nodes[road_id] = next_vid
            next_vid += 1
        return rep.virtual_nodes[road_id]

    # refPos：各 incoming 接触端点的质心
    contact_pts = []
    links: dict[str, MapLink] = {}
    lane_dir_map = {}                    # (incoming_id) -> (pts_ref, lane_bucket, reversed)
    for inc_id, end in dirs.items():
        road = odr.roads[inc_id]
        if road.unsupported:
            rep.notes.append(f"road {inc_id} 含不支持几何，跳过")
            continue
        pts, hdg, s = sample_reference_line(road, step)
        lanes = road.right if end == "end" else road.left

        def _orient_cut(arr):
            """按驶入方向排列并从末端截取 max_link_len。"""
            a = arr if end == "end" else arr[::-1]
            seg = np.linalg.norm(np.diff(a, axis=0), axis=1)
            cum = np.concatenate([[0.0], np.cumsum(seg[::-1])])[::-1]
            keep = cum <= max_link_len
            return a[keep] if keep.sum() >= 2 else a

        ordered = _orient_cut(pts)
        contact_pts.append(ordered[-1])
        idx = simplify_appendix_d(ordered, tol)
        drive = [ld for ld in lanes if ld.lane_type == "driving"]
        link = MapLink(name=f"road{inc_id}", upstream=(region, vid(inc_id)),
                       width_cm=int(round(sum(ld.width_a for ld in drive) * 100)) or None)
        link._src_pid = f"road:{inc_id}"
        link._pts_xy = ordered[idx]                     # 平面坐标暂存，最后统一转经纬度
        for k, ld in enumerate(drive, start=1):
            lane = MapLane(lane_id=k, width_cm=int(round(ld.width_a * 100)), maneuvers=None)
            lane._src_pid = f"road:{inc_id}/lane:{ld.lane_id}"
            lane._odr_id = ld.lane_id
            lg = _orient_cut(offset_polyline(pts, hdg, lane_center_t(road, ld.lane_id)))
            lane._pts_xy = lg[simplify_appendix_d(lg, tol)]
            link.lanes.append(lane)
        links[inc_id] = link
        rep.links.append(f"road{inc_id}({end}, {len(drive)} driving lanes, {len(idx)} pts)")

    # connectsTo：laneLink 直译
    for c in j.connections:
        if c.incoming not in links:
            continue
        conn_road = odr.roads.get(c.connecting)
        if conn_road is None or conn_road.unsupported:
            rep.n_dropped_conn += len(c.lane_links)
            continue
        cpts, chdg, _ = sample_reference_line(conn_road, step)
        dh = float(chdg[-1] - chdg[0]) if c.contact == "start" else float(chdg[0] - chdg[-1])
        man = _maneuver_bits(dh)
        t_kind, t_id = _other_end_target(odr, conn_road, c.contact)
        remote = vid(t_id) if t_kind == "road" else (node_id if t_id == junction_id else vid(str(t_id)))
        for ll in c.lane_links:
            link = links[c.incoming]
            lane = next((ln for ln in link.lanes if getattr(ln, "_odr_id", None) == ll.frm), None)
            if lane is None:
                rep.n_dropped_conn += 1
                continue
            lane.connects.append(Connection(region=region, node=remote,
                                            lane=abs(ll.to), maneuver=man, phase=None))
            bits = int(lane.maneuvers or "0" * 12, 2) | int(man, 2)
            lane.maneuvers = format(bits, "012b")
            rep.n_conn += 1

    # 平面 → 经纬度（geoReference 锚点等距圆柱逆投影；M0 演示，正式走 PROJ 管线记录）
    lat0, lon0 = _anchor_from_georef(odr.geo_reference)
    if lat0 == 0.0 and lon0 == 0.0:
        lat0, lon0 = 29.5, 106.3
        rep.notes.append("geoReference 无锚点，使用演示锚点(29.5,106.3)")
    center = np.mean(np.asarray(contact_pts), axis=0) if contact_pts else np.zeros(2)
    node = MapNode(name=f"junction{junction_id}", region=region, node_id=node_id)
    ll = _to_lonlat(center[None, :], lat0, lon0)[0]
    node.ref_lon, node.ref_lat = ll
    node.msg_cnt = 0
    for inc_id, link in links.items():
        link.points = _to_lonlat(link._pts_xy, lat0, lon0)
        del link._pts_xy
        for lane in link.lanes:
            if hasattr(lane, "_odr_id"):
                del lane._odr_id
            if hasattr(lane, "_pts_xy"):
                lane.points = _to_lonlat(lane._pts_xy, lat0, lon0)
                del lane._pts_xy
        node.links.append(link)
    return node, rep
