# -*- coding: utf-8 -*-
"""最小 MAP 消息 XML reader（金凤 XER 风格方言，spike 级）。

方言容错（资料盘点三.3）：位串值带空白需 strip；movements 层可整体缺失；
点列走 position-LatLon 绝对坐标分支（1e-7° 整数）；laneWidth 单位 1 cm。
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

SCALE_DEG = 1e-7


@dataclass
class Connection:
    region: int | None
    node: int | None
    lane: int | None
    maneuver: str | None
    phase: int | None


@dataclass
class MapLane:
    lane_id: int
    width_cm: int | None
    maneuvers: str | None
    points: list[tuple[float, float]] = field(default_factory=list)  # (lon, lat) 度
    points_elev: list[int | None] = field(default_factory=list)      # offsetV>elevation 原始值（0.1m）
    speed_limits: list[tuple[str, int]] = field(default_factory=list)  # (类型枚举名, speed 原始值)
    connects: list[Connection] = field(default_factory=list)


@dataclass
class MapLink:
    name: str
    upstream: tuple[int | None, int | None]
    width_cm: int | None
    points: list[tuple[float, float]] = field(default_factory=list)
    points_elev: list[int | None] = field(default_factory=list)
    lanes: list[MapLane] = field(default_factory=list)
    movements: list[tuple[int | None, int | None, int | None]] = field(default_factory=list)


@dataclass
class MapNode:
    name: str
    region: int | None
    node_id: int | None
    ref_lon: float = 0.0
    ref_lat: float = 0.0
    ref_elev: int | None = None          # 原始值（0.1 m）
    msg_cnt: int | None = None
    time_stamp: int | None = None
    links: list[MapLink] = field(default_factory=list)


def _int(el, tag):
    x = el.find(tag)
    if x is None or x.text is None:
        return None
    try:
        return int(x.text.strip())
    except ValueError:
        return None


def _bits(el, tag):
    x = el.find(tag)
    return x.text.strip() if x is not None and x.text else None


def _node_ref(el):
    return (_int(el, "region"), _int(el, "id")) if el is not None else (None, None)


def _points(el):
    pts, elevs = [], []
    if el is None:
        return pts, elevs
    for rp in el.findall("RoadPoint"):
        ll = rp.find("posOffset/offsetLL/position-LatLon")
        if ll is None:
            continue
        lon, lat = _int(ll, "lon"), _int(ll, "lat")
        if lon is None or lat is None:
            continue
        pts.append((lon * SCALE_DEG, lat * SCALE_DEG))
        ev = rp.find("posOffset/offsetV")
        elevs.append(_int(ev, "elevation") if ev is not None else None)
    return pts, elevs


def _speed_limits(el):
    out = []
    if el is None:
        return out
    for sl in el.findall("RegulatorySpeedLimit"):
        t = sl.find("type")
        tname = None
        if t is not None:
            for child in t:
                tname = child.tag
                break
        spd = _int(sl, "speed")
        if tname and spd is not None:
            out.append((tname, spd))
    return out


def parse_map_xml(path: str) -> MapNode:
    """首个 Node（单路口文件的既有行为）。多节点帧用 parse_map_xml_all。"""
    return parse_map_xml_all(path)[0]


def parse_map_xml_all(path: str) -> list[MapNode]:
    """帧内全部 Node——多节点 MAP（相邻路口合帧）时，邻居节点的 inLink 即本节点的真实出口路。"""
    root = ET.parse(path).getroot()
    mf = root.find("mapFrame")
    nodes = root.findall("mapFrame/nodes/Node")
    if not nodes:
        raise ValueError("no mapFrame/nodes/Node in %s" % path)
    return [_parse_node(nd, mf) for nd in nodes]


def _parse_node(nd, mf) -> MapNode:
    region, nid = _node_ref(nd.find("id"))
    out = MapNode(name=(nd.findtext("name") or "").strip(), region=region, node_id=nid)
    out.msg_cnt = _int(mf, "msgCnt")
    out.time_stamp = _int(mf, "timeStamp")
    ref = nd.find("refPos")
    out.ref_lat = (_int(ref, "lat") or 0) * SCALE_DEG
    out.ref_lon = (_int(ref, "long") or 0) * SCALE_DEG
    out.ref_elev = _int(ref, "elevation")
    for lk in nd.findall("inLinks/Link"):
        lpts, lelev = _points(lk.find("points"))
        link = MapLink(
            name=(lk.findtext("name") or "").strip(),
            upstream=_node_ref(lk.find("upstreamNodeId")),
            width_cm=_int(lk, "linkWidth"),
            points=lpts, points_elev=lelev,
        )
        for mv in lk.findall("movements/Movement"):
            r, n = _node_ref(mv.find("remoteIntersection"))
            link.movements.append((r, n, _int(mv, "phaseId")))
        for ln in lk.findall("lanes/Lane"):
            npts, nelev = _points(ln.find("points"))
            lane = MapLane(
                lane_id=_int(ln, "laneID") or 0,
                width_cm=_int(ln, "laneWidth"),
                maneuvers=_bits(ln, "maneuvers"),
                points=npts, points_elev=nelev,
                speed_limits=_speed_limits(ln.find("speedLimits")),
            )
            for cn in ln.findall("connectsTo/Connection"):
                r, n = _node_ref(cn.find("remoteIntersection"))
                cl = cn.find("connectingLane")
                lane.connects.append(Connection(
                    region=r, node=n,
                    lane=_int(cl, "lane") if cl is not None else None,
                    maneuver=_bits(cl, "maneuver") if cl is not None else None,
                    phase=_int(cn, "phaseId"),
                ))
            link.lanes.append(lane)
        out.links.append(link)
    return out
