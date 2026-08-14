# -*- coding: utf-8 -*-
"""OpenDRIVE 1.5 规范级 writer（自研，替代 scenariogeneration）。

弃用第三方生成器的原因（实测痛点）：无车道 <speed>（曾靠写出后注入）、
凭空发空 elevationProfile/lateralProfile（曾靠后处理剥除）、车道衔接自动推断
与我们的显式 laneLink 冲突（覆写警告）。本 writer 只写我们消费的子集，
但**每个元素/属性/顺序严格对照 OpenDRIVE_1.5M.xsd**：

- header(revMajor=1, revMinor=5) + geoReference（PROJ 管线落盘，CRS 硬约束 3）；
- road[@rule=RHT]：link(pred/succ + contactPoint) / planView(line|arc|spiral)
  / lanes(laneOffset*, laneSection(left|center|right))；
- lane 子元素顺序 link → width+ → roadMark* → speed*（XSD sequence）；
  左侧车道按 id 降序、右侧按 id 升序绝对值（外→内/内→外的文档惯例）；
- junction/connection/laneLink。

所有数值显式格式化（%.10g），不写默认值噪声。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET


def _f(v: float) -> str:
    return f"{float(v):.10g}"


def std_mark(kind: str) -> tuple:
    """默认标线（APPROXIMATED：MARK_TYPE 权威枚举未消费前的通行画法）。
    center=单实黄（单侧路）/ center2=双黄（双向路对向分界）/ inner=虚白 / outer=实白。"""
    return {"center": ("solid", "yellow", 0.15),
            "center2": ("solid solid", "yellow", 0.15),
            "inner": ("broken", "white", 0.15),
            "outer": ("solid", "white", 0.15)}[kind]


# —— 车道 ——
@dataclass
class Lane:
    lane_id: int
    lane_type: str = "driving"                           # driving | median | ...
    widths: list = field(default_factory=list)           # [(sOffset, a, b, c, d)]
    mark: tuple | None = None                            # (type, color, width_m)
    speed_ms: float | None = None
    pred: int | None = None
    succ: int | None = None

    def add_width(self, a, b=0.0, c=0.0, d=0.0, s_offset=0.0):
        self.widths.append((s_offset, a, b, c, d))
        return self


@dataclass
class LaneSection:
    s: float
    center_mark: tuple | None = None
    left: list = field(default_factory=list)             # Lane（id 正，任意序，写出时降序）
    right: list = field(default_factory=list)            # Lane（id 负，写出时 -1..-n）


@dataclass
class Road:
    road_id: int
    name: str | None = None
    junction: int = -1
    geoms: list = field(default_factory=list)            # [(kind, x, y, hdg, L, k0, k1)]
    offsets: list = field(default_factory=list)          # [(s, a, b, c, d)]
    sections: list = field(default_factory=list)         # [LaneSection]
    links: list = field(default_factory=list)            # [(role, etype, eid, contact|None)]

    def add_geometry(self, kind, x, y, hdg, length, k0=0.0, k1=0.0):
        self.geoms.append((kind, x, y, hdg, length, k0, k1))
        return self

    def add_offset(self, s, a, b=0.0, c=0.0, d=0.0):
        self.offsets.append((s, a, b, c, d))
        return self

    def add_link(self, role, etype, eid, contact=None):
        """role: predecessor|successor; etype: road|junction; contact: start|end|None。"""
        self.links.append((role, etype, eid, contact))
        return self

    @property
    def length(self) -> float:
        return sum(g[4] for g in self.geoms)

    def end_pose(self):
        """末几何段解析终点位姿（line/arc/spiral 曲率线性推进）。"""
        kind, x, y, h, L, k0, k1 = self.geoms[-1]
        if kind == "line":
            return x + L * math.cos(h), y + L * math.sin(h), h
        if kind == "arc":
            return (x + (math.sin(h + k0 * L) - math.sin(h)) / k0,
                    y - (math.cos(h + k0 * L) - math.cos(h)) / k0, h + k0 * L)
        from pyclothoids import Clothoid
        cl = Clothoid.StandardParams(x, y, h, k0, (k1 - k0) / max(L, 1e-12), L)
        return cl.X(L), cl.Y(L), h + k0 * L + 0.5 * (k1 - k0) * L


@dataclass
class Connection:
    incoming: int
    connecting: int
    contact: str = "start"
    lane_links: list = field(default_factory=list)       # [(from, to)]

    def add_lanelink(self, frm, to):
        if (frm, to) not in self.lane_links:
            self.lane_links.append((frm, to))
        return self


@dataclass
class Junction:
    junction_id: int
    name: str = ""
    connections: list = field(default_factory=list)


class XodrDoc:
    """整文档：roads + junctions → 严格 1.5 XML。"""

    def __init__(self, name: str, geo_reference: str | None = None):
        self.name = name
        self.geo_reference = geo_reference
        self.roads: list[Road] = []
        self.junctions: list[Junction] = []

    def add_road(self, road: Road):
        self.roads.append(road)
        return road

    def add_junction(self, j: Junction):
        self.junctions.append(j)
        return j

    # ---------------------------------------------------------------- 写出
    def _lane_el(self, parent, ln: Lane):
        el = ET.SubElement(parent, "lane", id=str(ln.lane_id),
                           type=ln.lane_type, level="false")
        if ln.pred is not None or ln.succ is not None:   # 顺序：link 最先
            lk = ET.SubElement(el, "link")
            if ln.pred is not None:
                ET.SubElement(lk, "predecessor", id=str(ln.pred))
            if ln.succ is not None:
                ET.SubElement(lk, "successor", id=str(ln.succ))
        for (so, a, b, c, d) in sorted(ln.widths):
            ET.SubElement(el, "width", sOffset=_f(so), a=_f(a), b=_f(b), c=_f(c), d=_f(d))
        if ln.mark is not None:
            typ, color, w = ln.mark
            ET.SubElement(el, "roadMark", sOffset="0", type=typ,
                          weight="standard", color=color, width=_f(w),
                          laneChange="both")
        if ln.speed_ms is not None:
            ET.SubElement(el, "speed", sOffset="0", max=_f(ln.speed_ms))
        return el

    def _road_el(self, root, rd: Road):
        el = ET.SubElement(root, "road", name=rd.name or "", length=_f(rd.length),
                           id=str(rd.road_id), junction=str(rd.junction), rule="RHT")
        if rd.links:
            lk = ET.SubElement(el, "link")
            for role, etype, eid, contact in rd.links:
                attrs = {"elementType": etype, "elementId": str(eid)}
                if contact is not None:
                    attrs["contactPoint"] = contact
                ET.SubElement(lk, role, **attrs)
        pv = ET.SubElement(el, "planView")
        s = 0.0
        for kind, x, y, h, L, k0, k1 in rd.geoms:
            g = ET.SubElement(pv, "geometry", s=_f(s), x=_f(x), y=_f(y),
                              hdg=_f(h), length=_f(L))
            if kind == "line":
                ET.SubElement(g, "line")
            elif kind == "arc":
                ET.SubElement(g, "arc", curvature=_f(k0))
            elif kind == "spiral":
                ET.SubElement(g, "spiral", curvStart=_f(k0), curvEnd=_f(k1))
            else:
                raise ValueError(f"unknown geometry kind: {kind}")
            s += L
        lanes = ET.SubElement(el, "lanes")
        for (so, a, b, c, d) in sorted(rd.offsets):
            ET.SubElement(lanes, "laneOffset", s=_f(so), a=_f(a), b=_f(b), c=_f(c), d=_f(d))
        for sec in sorted(rd.sections, key=lambda x: x.s):
            se = ET.SubElement(lanes, "laneSection", s=_f(sec.s))
            if sec.left:
                le = ET.SubElement(se, "left")
                for ln in sorted(sec.left, key=lambda x: -x.lane_id):
                    self._lane_el(le, ln)
            ce = ET.SubElement(se, "center")
            cl = ET.SubElement(ce, "lane", id="0", type="none", level="false")
            if sec.center_mark is not None:
                typ, color, w = sec.center_mark
                ET.SubElement(cl, "roadMark", sOffset="0", type=typ,
                              weight="standard", color=color, width=_f(w),
                              laneChange="none")
            if sec.right:
                re_ = ET.SubElement(se, "right")
                for ln in sorted(sec.right, key=lambda x: -x.lane_id):
                    self._lane_el(re_, ln)
        return el

    def write(self, path: str | Path):
        root = ET.Element("OpenDRIVE")
        hdr = ET.SubElement(root, "header", revMajor="1", revMinor="5",
                            name=self.name, version="1.00", date="",
                            north="0", south="0", east="0", west="0")
        if self.geo_reference:
            ET.SubElement(hdr, "geoReference").text = self.geo_reference
        for rd in sorted(self.roads, key=lambda r: r.road_id):
            self._road_el(root, rd)
        for j in self.junctions:
            je = ET.SubElement(root, "junction", name=j.name, id=str(j.junction_id))
            for i, c in enumerate(j.connections):
                ce = ET.SubElement(je, "connection", id=str(i),
                                   incomingRoad=str(c.incoming),
                                   connectingRoad=str(c.connecting),
                                   contactPoint=c.contact)
                for frm, to in c.lane_links:
                    ET.SubElement(ce, "laneLink", **{"from": str(frm), "to": str(to)})
        pretty = minidom.parseString(ET.tostring(root, encoding="unicode"))
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(pretty.toprettyxml(indent="    "), encoding="utf-8")


def add_paving_road(doc: "XodrDoc", poly_xy, junction_id: int, road_id: int = 90):
    """junction 内部铺面 road：参考线沿多边形主轴（PCA），单条 type=none 车道的
    宽度轮廓逐站扫掠出多边形形状——查看器渲染为无标线沥青面，填补连接路带
    盖不住的路口角落；不入拓扑、无 connection 引用、不参与寻路。
    多边形边即折线 → 线性宽度逐段精确贴合。返回 True/False（退化多边形跳过）。"""
    import numpy as np
    from shapely.geometry import LineString, Polygon

    pg = Polygon(poly_xy).buffer(0)
    if pg.is_empty or pg.area < 10:
        return False
    pts = np.asarray(pg.exterior.coords)
    c = pts.mean(axis=0)
    q = pts - c
    _u, _s, vt = np.linalg.svd(q, full_matrices=False)
    ax = vt[0]
    nrm = np.array([-ax[1], ax[0]])
    s_all = q @ ax
    s0, s1 = float(s_all.min()) + 0.05, float(s_all.max()) - 0.05
    L = s1 - s0
    if L < 4.0:
        return False
    start = c + ax * s0
    road = Road(road_id, name="junction_paving", junction=junction_id)
    road.add_geometry("line", float(start[0]), float(start[1]),
                      math.atan2(float(ax[1]), float(ax[0])), L)
    K = max(3, int(L / 4) + 2)
    us = np.linspace(0.0, L, K)
    prof = []
    for u in us:
        p0 = start + ax * u
        cut = LineString([p0 - nrm * 200, p0 + nrm * 200]).intersection(pg)
        ts = []
        for g in getattr(cut, "geoms", [cut]):
            ts += [float(np.dot(np.asarray(xy) - p0, nrm)) for xy in g.coords]
        prof.append((min(ts), max(ts)) if ts else None)
    for i in range(K):                                   # 端头空切片借邻值
        if prof[i] is None:
            near = next((prof[j] for j in
                         sorted(range(K), key=lambda j: abs(j - i))
                         if prof[j] is not None), (0.0, 0.0))
            prof[i] = near
    for i in range(K - 1):
        u0 = float(us[i])
        Ls = float(us[i + 1] - us[i])
        (lo0, hi0), (lo1, hi1) = prof[i], prof[i + 1]
        road.add_offset(u0, hi0, (hi1 - hi0) / Ls)
        sec = LaneSection(u0)
        ln = Lane(-1, "none")
        w0, w1 = hi0 - lo0, hi1 - lo1
        ln.add_width(w0, (w1 - w0) / Ls)
        if i > 0:
            ln.pred = -1
        if i < K - 2:
            ln.succ = -1
        sec.right.append(ln)
        road.sections.append(sec)
    doc.add_road(road)
    return True
