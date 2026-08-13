# -*- coding: utf-8 -*-
"""XML reader 数据结构 → 消息层 ASN 值结构（pycrate set_val 格式）。

两种点列编码模式：
- absolute：position-LatLon 绝对分支（现网方言，兼容输出）
- offset  ：相对 refPos 的 position-LL1..LL6 偏移分支，逐点选最小档（方案 5.5 的字节优化项）
"""
from __future__ import annotations

from .xml_reader import MapNode, MapLink, MapLane, Connection

SCALE = 1e-7
# (分支名, 结构名占位, 半幅)——OffsetLL-B12/B14/B16/B18/B22/B24，LSB 与绝对坐标同为 1e-7°
_OFFSET_TIERS = [
    ("position-LL1", 2047),
    ("position-LL2", 8191),
    ("position-LL3", 32767),
    ("position-LL4", 131071),
    ("position-LL5", 2097151),
    ("position-LL6", 8388607),
]


def _i7(deg: float) -> int:
    return int(round(deg / SCALE))


def _bits(bitstr: str | None):
    if not bitstr:
        return None
    return (int(bitstr, 2), len(bitstr))


def _node_ref(region, nid):
    v = {"id": int(nid)}
    if region is not None:
        v["region"] = int(region)
    return v


def _point_val(lon_deg: float, lat_deg: float, elev, ref_lon_i: int, ref_lat_i: int, mode: str):
    lon_i, lat_i = _i7(lon_deg), _i7(lat_deg)
    if mode == "offset":
        dlon, dlat = lon_i - ref_lon_i, lat_i - ref_lat_i
        m = max(abs(dlon), abs(dlat))
        for branch, half in _OFFSET_TIERS:
            if m <= half:
                off = (branch, {"lon": dlon, "lat": dlat})
                break
        else:
            off = ("position-LatLon", {"lon": lon_i, "lat": lat_i})
    else:
        off = ("position-LatLon", {"lon": lon_i, "lat": lat_i})
    pos = {"offsetLL": off}
    if elev is not None:
        pos["offsetV"] = ("elevation", int(elev))
    return {"posOffset": pos}


def _conn_val(c: Connection):
    v = {"remoteIntersection": _node_ref(c.region, c.node)}
    if c.lane is not None or c.maneuver:
        cl = {}
        if c.lane is not None:
            cl["lane"] = int(c.lane)
        mv = _bits(c.maneuver)
        if mv:
            cl["maneuver"] = mv
        v["connectingLane"] = cl
    if c.phase is not None:
        v["phaseId"] = int(c.phase)
    return v


def _lane_val(ln: MapLane, ref_lon_i, ref_lat_i, mode):
    v = {"laneID": int(ln.lane_id)}
    if ln.width_cm is not None:
        v["laneWidth"] = int(ln.width_cm)
    mv = _bits(ln.maneuvers)
    if mv:
        v["maneuvers"] = mv
    if ln.connects:
        v["connectsTo"] = [_conn_val(c) for c in ln.connects]
    if ln.speed_limits:
        v["speedLimits"] = [{"type": t, "speed": int(s)} for t, s in ln.speed_limits]
    if ln.points:
        elevs = ln.points_elev if len(ln.points_elev) == len(ln.points) else [None] * len(ln.points)
        v["points"] = [_point_val(lon, lat, e, ref_lon_i, ref_lat_i, mode)
                       for (lon, lat), e in zip(ln.points, elevs)]
    return v


def _link_val(lk: MapLink, ref_lon_i, ref_lat_i, mode):
    v = {"upstreamNodeId": _node_ref(*lk.upstream),
         "linkWidth": int(lk.width_cm) if lk.width_cm is not None else 0,
         "lanes": [_lane_val(ln, ref_lon_i, ref_lat_i, mode) for ln in lk.lanes]}
    if lk.name:
        v["name"] = lk.name
    if lk.points:
        elevs = lk.points_elev if len(lk.points_elev) == len(lk.points) else [None] * len(lk.points)
        v["points"] = [_point_val(lon, lat, e, ref_lon_i, ref_lat_i, mode)
                       for (lon, lat), e in zip(lk.points, elevs)]
    if lk.movements:
        mvs = []
        for r, n, ph in lk.movements:
            mv = {"remoteIntersection": _node_ref(r, n)}
            if ph is not None:
                mv["phaseId"] = int(ph)
            mvs.append(mv)
        v["movements"] = mvs
    return v


def node_to_messageframe_val(node: MapNode, mode: str = "absolute"):
    """MapNode → ("mapFrame", MapData 值)。mode: absolute | offset。"""
    ref_lon_i, ref_lat_i = _i7(node.ref_lon), _i7(node.ref_lat)
    ref = {"lat": ref_lat_i, "long": ref_lon_i}
    if node.ref_elev is not None:
        ref["elevation"] = int(node.ref_elev)
    nd = {"id": _node_ref(node.region, node.node_id), "refPos": ref,
          "inLinks": [_link_val(lk, ref_lon_i, ref_lat_i, mode) for lk in node.links]}
    if node.name:
        nd["name"] = node.name
    md = {"msgCnt": int(node.msg_cnt or 0), "nodes": [nd]}
    if node.time_stamp is not None:
        md["timeStamp"] = int(node.time_stamp)
    return ("mapFrame", md)
