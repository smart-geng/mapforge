# -*- coding: utf-8 -*-
"""轻量 OpenDRIVE reader（M0：ElementTree 直读，line/arc/spiral 几何求值）。

覆盖：header/geoReference、road(planView/lanes/link/elevation 略)、junction(connection/laneLink)。
poly3/paramPoly3 段暂不求值（遇到则该 road 标记 unsupported——Town03 无此类段）。
libOpenDRIVE pybind11 绑定与方言 Profile 属正式版范围。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Geometry:
    s: float
    x: float
    y: float
    hdg: float
    length: float
    kind: str                  # line/arc/spiral/unsupported
    k0: float = 0.0
    k1: float = 0.0


@dataclass
class LaneDef:
    lane_id: int               # OpenDRIVE 车道号（左正右负）
    lane_type: str
    width_a: float             # 首项宽度（Town03 99.6% 常数宽）
    pred: int | None = None
    succ: int | None = None


@dataclass
class Road:
    road_id: str
    length: float
    junction: str
    geoms: list[Geometry] = field(default_factory=list)
    left: list[LaneDef] = field(default_factory=list)    # 按 |id| 升序
    right: list[LaneDef] = field(default_factory=list)
    pred: tuple[str, str, str] | None = None             # (elementType, elementId, contactPoint)
    succ: tuple[str, str, str] | None = None
    unsupported: bool = False


@dataclass
class LaneLink:
    frm: int
    to: int


@dataclass
class JConnection:
    conn_id: str
    incoming: str
    connecting: str
    contact: str               # start/end
    lane_links: list[LaneLink] = field(default_factory=list)


@dataclass
class Junction:
    junction_id: str
    name: str
    connections: list[JConnection] = field(default_factory=list)


@dataclass
class OdrMap:
    roads: dict[str, Road] = field(default_factory=dict)
    junctions: dict[str, Junction] = field(default_factory=dict)
    geo_reference: str = ""


def parse_xodr(path: str) -> OdrMap:
    root = ET.parse(path).getroot()
    out = OdrMap()
    geo = root.find("header/geoReference")
    out.geo_reference = (geo.text or "").strip() if geo is not None else ""
    for r in root.findall("road"):
        road = Road(r.get("id"), float(r.get("length", 0)), r.get("junction", "-1"))
        for tag in ("predecessor", "successor"):
            el = r.find("link/" + tag)
            if el is not None:
                v = (el.get("elementType"), el.get("elementId"), el.get("contactPoint") or "")
                if tag == "predecessor":
                    road.pred = v
                else:
                    road.succ = v
        s_acc = 0.0
        for g in r.findall("planView/geometry"):
            L = float(g.get("length"))
            child = g[0]
            geom = Geometry(float(g.get("s")), float(g.get("x")), float(g.get("y")),
                            float(g.get("hdg")), L, child.tag)
            if child.tag == "arc":
                geom.k0 = geom.k1 = float(child.get("curvature"))
            elif child.tag == "spiral":
                geom.k0 = float(child.get("curvStart"))
                geom.k1 = float(child.get("curvEnd"))
            elif child.tag != "line":
                geom.kind = "unsupported"
                road.unsupported = True
            road.geoms.append(geom)
            s_acc += L
        ls = r.find("lanes/laneSection")            # M0：取首个 laneSection
        if ls is not None:
            for side, bucket in (("left", road.left), ("right", road.right)):
                sec = ls.find(side)
                if sec is None:
                    continue
                for ln in sec.findall("lane"):
                    w = ln.find("width")
                    ld = LaneDef(int(ln.get("id")), ln.get("type", ""),
                                 float(w.get("a")) if w is not None else 0.0)
                    p = ln.find("link/predecessor")
                    s_ = ln.find("link/successor")
                    ld.pred = int(p.get("id")) if p is not None else None
                    ld.succ = int(s_.get("id")) if s_ is not None else None
                    bucket.append(ld)
                bucket.sort(key=lambda d: abs(d.lane_id))
        out.roads[road.road_id] = road
    for j in root.findall("junction"):
        jn = Junction(j.get("id"), j.get("name", ""))
        for c in j.findall("connection"):
            conn = JConnection(c.get("id"), c.get("incomingRoad"), c.get("connectingRoad"),
                               c.get("contactPoint", "start"))
            for ll in c.findall("laneLink"):
                conn.lane_links.append(LaneLink(int(ll.get("from")), int(ll.get("to"))))
            jn.connections.append(conn)
        out.junctions[jn.junction_id] = jn
    return out


def _sample_geom(g: Geometry, ss: np.ndarray):
    """几何段内弧长 ss（0..length）→ (x, y, hdg) 数组。line/arc 解析，spiral 数值积分。"""
    if g.kind == "line":
        x = g.x + ss * math.cos(g.hdg)
        y = g.y + ss * math.sin(g.hdg)
        h = np.full_like(ss, g.hdg)
    elif g.kind == "arc":
        k = g.k0
        x = g.x + (np.sin(g.hdg + k * ss) - math.sin(g.hdg)) / k
        y = g.y - (np.cos(g.hdg + k * ss) - math.cos(g.hdg)) / k
        h = g.hdg + k * ss
    elif g.kind == "spiral":
        n = max(32, int(g.length * 8))
        sf = np.linspace(0.0, g.length, n + 1)
        c = g.k0 + (g.k1 - g.k0) * sf / max(g.length, 1e-12)
        th = g.hdg + np.concatenate([[0.0], np.cumsum(0.5 * (c[1:] + c[:-1]) * np.diff(sf))])
        xf = g.x + np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(th[1:]) + np.cos(th[:-1])) * np.diff(sf))])
        yf = g.y + np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(th[1:]) + np.sin(th[:-1])) * np.diff(sf))])
        x = np.interp(ss, sf, xf)
        y = np.interp(ss, sf, yf)
        h = np.interp(ss, sf, th)
    else:
        raise NotImplementedError(g.kind)
    return x, y, h


def sample_reference_line(road: Road, step: float = 1.0):
    """参考线等距采样 → (pts(n,2), hdg(n,), s(n,))。"""
    xs, ys, hs, sacc = [], [], [], []
    for g in road.geoms:
        n = max(2, int(g.length / step) + 1)
        ss = np.linspace(0.0, g.length, n)
        if xs:
            ss = ss[1:]
        x, y, h = _sample_geom(g, ss)
        xs.append(x); ys.append(y); hs.append(h)
        sacc.append(g.s + ss)
    return (np.column_stack([np.concatenate(xs), np.concatenate(ys)]),
            np.concatenate(hs), np.concatenate(sacc))


def offset_polyline(pts: np.ndarray, hdg: np.ndarray, t: float) -> np.ndarray:
    """参考线沿法向偏移 t（左正右负，OpenDRIVE 惯例）。"""
    nx = -np.sin(hdg)
    ny = np.cos(hdg)
    return pts + t * np.column_stack([nx, ny])


def lane_center_t(road: Road, lane_id: int) -> float:
    """车道中心相对参考线的 t 偏移（首 laneSection 常数宽近似，忽略 laneOffset）。"""
    bucket = road.left if lane_id > 0 else road.right
    acc = 0.0
    for ld in bucket:
        if abs(ld.lane_id) < abs(lane_id):
            acc += ld.width_a
        elif ld.lane_id == lane_id:
            acc += ld.width_a / 2.0
            break
    return acc if lane_id > 0 else -acc
