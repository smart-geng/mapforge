# -*- coding: utf-8 -*-
"""MAP 消息 XML 写出器（XER 风格，对齐金凤现网方言）：MapNode → MessageFrame XML。

字段序按消息层 ASN 定义；坐标写绝对分支 position-LatLon（1e-7° 整数）；
maneuvers 写 12 位 01 串。写出后可被本项目 xml_reader 与现网工具消费。
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

from .xml_reader import MapNode, MapLink, MapLane

SCALE = 1e-7


def _sub(parent, tag, text=None):
    el = ET.SubElement(parent, tag)
    if text is not None:
        el.text = str(text)
    return el


def _i7(deg: float) -> int:
    return int(round(deg / SCALE))


def _node_ref(parent, tag, region, nid):
    el = _sub(parent, tag)
    if region is not None:
        _sub(el, "region", region)
    _sub(el, "id", nid if nid is not None else 0)
    return el


def _points(parent, pts, elevs):
    el = _sub(parent, "points")
    elevs = elevs if len(elevs) == len(pts) else [None] * len(pts)
    for (lon, lat), ev in zip(pts, elevs):
        rp = _sub(el, "RoadPoint")
        po = _sub(rp, "posOffset")
        ll = _sub(_sub(po, "offsetLL"), "position-LatLon")
        _sub(ll, "lon", _i7(lon))
        _sub(ll, "lat", _i7(lat))
        if ev is not None:
            _sub(_sub(po, "offsetV"), "elevation", ev)


def _speed_limits(parent, sls):
    if not sls:
        return
    el = _sub(parent, "speedLimits")
    for tname, spd in sls:
        rs = _sub(el, "RegulatorySpeedLimit")
        _sub(_sub(rs, "type"), tname)          # 枚举=空元素
        _sub(rs, "speed", spd)


def _lane(parent, ln: MapLane):
    el = _sub(parent, "Lane")
    _sub(el, "laneID", ln.lane_id)
    if ln.width_cm is not None:
        _sub(el, "laneWidth", ln.width_cm)
    if ln.maneuvers:
        _sub(el, "maneuvers", ln.maneuvers)
    if ln.connects:
        ct = _sub(el, "connectsTo")
        for c in ln.connects:
            conn = _sub(ct, "Connection")
            _node_ref(conn, "remoteIntersection", c.region, c.node)
            if c.lane is not None or c.maneuver:
                cl = _sub(conn, "connectingLane")
                if c.lane is not None:
                    _sub(cl, "lane", c.lane)
                if c.maneuver:
                    _sub(cl, "maneuver", c.maneuver)
            if c.phase is not None:
                _sub(conn, "phaseId", c.phase)
    _speed_limits(el, ln.speed_limits)
    if ln.points:
        _points(el, ln.points, ln.points_elev)


def _link(parent, lk: MapLink):
    el = _sub(parent, "Link")
    if lk.name:
        _sub(el, "name", lk.name)
    _node_ref(el, "upstreamNodeId", *lk.upstream)
    if lk.width_cm is not None:
        _sub(el, "linkWidth", lk.width_cm)
    if lk.points:
        _points(el, lk.points, lk.points_elev)
    if lk.movements:
        mv = _sub(el, "movements")
        for r, n, ph in lk.movements:
            m = _sub(mv, "Movement")
            _node_ref(m, "remoteIntersection", r, n)
            if ph is not None:
                _sub(m, "phaseId", ph)
    lanes = _sub(el, "lanes")
    for ln in lk.lanes:
        _lane(lanes, ln)


def node_to_xml(node: MapNode) -> str:
    root = ET.Element("MessageFrame")
    mf = _sub(root, "mapFrame")
    _sub(mf, "msgCnt", node.msg_cnt or 0)
    if node.time_stamp is not None:
        _sub(mf, "timeStamp", node.time_stamp)
    nodes = _sub(mf, "nodes")
    nd = _sub(nodes, "Node")
    if node.name:
        _sub(nd, "name", node.name)
    _node_ref(nd, "id", node.region, node.node_id)
    ref = _sub(nd, "refPos")
    _sub(ref, "lat", _i7(node.ref_lat))
    _sub(ref, "long", _i7(node.ref_lon))
    if node.ref_elev is not None:
        _sub(ref, "elevation", node.ref_elev)
    links = _sub(nd, "inLinks")
    for lk in node.links:
        _link(links, lk)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")
