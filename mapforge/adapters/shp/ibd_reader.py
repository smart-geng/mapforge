# -*- coding: utf-8 -*-
"""IBD 规格 SHP 读取器（M1：路口重塑所需图层子集，Profile=ibd-smarteditor-v1）。

- DBF LDID=0x57 实为 GBK；字段一律按名索引（pyshp 的 fields 参数按文件序返回，不可按请求序取）；
- 长度/宽度单位毫米、速度 km/h（资料盘点 2.2）；
- 实测姿态字段可信度：HEADING 可用、CURVATURE 不可用（Spike-C 核验），本 reader 不消费 CURVATURE。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import shapefile


@dataclass
class LaneRec:
    lane_pid: str
    link_pid: str
    seq: int
    width_mm: int
    lane_type: int
    max_speed_kmh: int
    geometry: np.ndarray          # (n,2) lon/lat 度


@dataclass
class RoadLinkRec:
    link_pid: str
    name: str
    lane_num: int
    s_node: str
    e_node: str
    geometry: np.ndarray


@dataclass
class JunctionRec:
    pid: str
    name: str
    enter_roads: list[str]
    leave_roads: list[str]
    polygon: np.ndarray

    @property
    def center(self) -> np.ndarray:
        return self.polygon.mean(axis=0)


def _s(v) -> str:
    return str(v).strip()


def _i(v, default=0) -> int:
    try:
        return int(str(v).strip())
    except (ValueError, TypeError):
        return default


class IbdSource:
    """按需加载 + 内存索引。"""

    def __init__(self, shp_dir: str):
        self.dir = Path(shp_dir)
        self._junctions: list[JunctionRec] | None = None
        self._roadlinks: dict[str, RoadLinkRec] | None = None
        self._lanes_by_link: dict[str, list[LaneRec]] | None = None
        self._lane_by_pid: dict[str, LaneRec] | None = None
        self._topo_out: dict[str, list[str]] | None = None

    def _reader(self, layer: str) -> shapefile.Reader:
        return shapefile.Reader(str(self.dir / layer), encoding="gbk")

    @property
    def junctions(self) -> list[JunctionRec]:
        if self._junctions is None:
            out = []
            r = self._reader("IBD_OBJECT_INTERSECTION_SURFACE")
            fields = [f[0] for f in r.fields[1:]]
            for sr in r.iterShapeRecords():
                m = dict(zip(fields, sr.record))
                out.append(JunctionRec(
                    pid=_s(m["OBJECT_PID"]), name=_s(m.get("NAME", "")),
                    enter_roads=[x for x in _s(m.get("ENTER_ROAD", "")).split(";") if x],
                    leave_roads=[x for x in _s(m.get("LEAVE_ROAD", "")).split(";") if x],
                    polygon=np.asarray(sr.shape.points)))
            self._junctions = out
        return self._junctions

    def find_junction(self, lon: float, lat: float) -> tuple[JunctionRec, float]:
        """最近路口面（返回记录与中心距，米级近似）。"""
        best, bd = None, 1e18
        for j in self.junctions:
            c = j.center
            d = math.hypot((c[0] - lon) * 111320 * math.cos(math.radians(lat)),
                           (c[1] - lat) * 110540)
            if d < bd:
                best, bd = j, d
        return best, bd

    @property
    def roadlinks(self) -> dict[str, RoadLinkRec]:
        if self._roadlinks is None:
            out = {}
            r = self._reader("IBD_ROADLINK")
            fields = [f[0] for f in r.fields[1:]]
            for sr in r.iterShapeRecords():
                m = dict(zip(fields, sr.record))
                out[_s(m["LINK_PID"])] = RoadLinkRec(
                    link_pid=_s(m["LINK_PID"]), name=_s(m.get("ROADNAME", "")),
                    lane_num=_i(m.get("LANE_NUM")), s_node=_s(m.get("S_NODE_PID", "")),
                    e_node=_s(m.get("E_NODE_PID", "")), geometry=np.asarray(sr.shape.points))
            self._roadlinks = out
        return self._roadlinks

    def _load_lanes(self):
        if self._lanes_by_link is not None:
            return
        by_link, by_pid = {}, {}
        self._merge_pids = set()
        for layer in ("IBD_LANE_LINK", "IBD_LANE_LINK_MERGE"):
            r = self._reader(layer)
            fields = [f[0] for f in r.fields[1:]]
            for sr in r.iterShapeRecords():
                m = dict(zip(fields, sr.record))
                rec = LaneRec(
                    lane_pid=_s(m["LANE_PID"]), link_pid=_s(m.get("LINK_PID", "")),
                    seq=_i(m.get("SEQ_NUM")), width_mm=_i(m.get("WIDTH")),
                    lane_type=_i(m.get("LANE_TYPE")), max_speed_kmh=_i(m.get("MAX_SPEED")),
                    geometry=np.asarray(sr.shape.points) if sr.shape.points else np.zeros((0, 2)))
                if layer == "IBD_LANE_LINK_MERGE":
                    self._merge_pids.add(rec.lane_pid)
                    by_pid.setdefault(rec.lane_pid, rec)      # MERGE 不覆盖普通层
                    continue
                by_pid[rec.lane_pid] = rec
                by_link.setdefault(rec.link_pid, []).append(rec)
        for k, v in by_link.items():
            seen, uniq = set(), []
            for rec in sorted(v, key=lambda x: x.seq):
                if rec.lane_pid not in seen:
                    seen.add(rec.lane_pid)
                    uniq.append(rec)
            by_link[k] = uniq
        self._lanes_by_link, self._lane_by_pid = by_link, by_pid

    def lanes_of(self, link_pid: str) -> list[LaneRec]:
        """普通车道层（IBD_LANE_LINK），按 SEQ 去重排序；MERGE 虚拟车道不在其中。"""
        self._load_lanes()
        return self._lanes_by_link.get(link_pid, [])

    def lane(self, lane_pid: str) -> LaneRec | None:
        self._load_lanes()
        return self._lane_by_pid.get(lane_pid)

    @property
    def topo_out(self) -> dict[str, list[str]]:
        if self._topo_out is None:
            out, inv = {}, {}
            r = self._reader("IBD_LANE_TOPO_DETAIL")
            fields = [f[0] for f in r.fields[1:]]
            for rec in r.iterRecords():
                m = dict(zip(fields, rec))
                a, b = _s(m["IN_PID"]), _s(m["OUT_PID"])
                out.setdefault(a, []).append(b)
                inv.setdefault(b, []).append(a)
            self._topo_out, self._topo_in = out, inv
        return self._topo_out

    @property
    def topo_in(self) -> dict[str, list[str]]:
        _ = self.topo_out
        return self._topo_in
