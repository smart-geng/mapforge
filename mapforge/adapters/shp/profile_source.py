# -*- coding: utf-8 -*-
"""通用 SHP Profile 引擎（方案 3.2 的 YAML 声明式映射；《接入指南》流程的落地）。

一份 YAML 描述一家图商：图层文件名、字段名、单位、编码、几何路线。车道几何两套来源：
- 路线 A（LANE_LINK 等价物）：车道中心线 + 宽度字段（geometry: field）；
- 路线 B（LANE_BOUNDARY 等价物）：边界线 + 车道↔边界左右关系
  （geometry: boundaries —— 中心线=左右边界中线合成；宽度=边界横距）。
宽度按 width_from 阶梯获取：field → boundaries → spacing（相邻车道间距）→ default；
非 field 来源属 APPROXIMATED，逐来源计数在 derivation_stats（进损失报告）。

与 IbdSource 同构（duck type）：junctions / find_junction / roadlinks / roadcenters /
lanes_of / lane / topo_out / topo_in / is_merge —— shp_to_xodr / shp_to_map 无需改动即可消费。
金凤实测依据（接入指南二）：边界横距推宽误差中位 1.4cm；相邻间距推宽中位 1.4cm。
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import shapefile
import yaml

from mapforge.adapters.shp.ibd_reader import (JunctionRec, LaneRec, RoadLinkRec,
                                              StopLineRec, _chain_segments, _i, _s, _known_width)

_LEN_TO_MM = {"mm": 1.0, "cm": 10.0, "m": 1000.0}
_ROOT = Path(__file__).resolve().parents[3]


def _f(v, default=0.0) -> float:
    try:
        return float(str(v).strip())
    except (ValueError, TypeError):
        return default


def _resample_m(pts, step):
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < 2 * step:
        return pts
    u = np.arange(0.0, s[-1], step)
    return np.column_stack([np.interp(u, s, pts[:, 0]), np.interp(u, s, pts[:, 1])])


def midline_of(left_m: np.ndarray, right_m: np.ndarray, step: float = 1.0) -> np.ndarray:
    """左右边界（米制折线）→ 中线：左线重采样，逐点取右线最近点的中点。"""
    L = _resample_m(left_m, step)
    R = _resample_m(right_m, step / 2)
    idx = np.argmin(np.linalg.norm(R[None, :, :] - L[:, None, :], axis=2), axis=1)
    return (L + R[idx]) / 2.0


def validate_profile(p: dict) -> list[str]:
    errs = []
    if (p or {}).get('lane_identity', 'reject-cross-layer-duplicates') not in (
            'reject-cross-layer-duplicates', 'primary-with-supplement-records'):
        errs.append('lane_identity 未知：必须明确跨层同ID规则')
    layers = (p or {}).get("layers", {})
    lane = layers.get("lane")
    if not lane or not lane.get("file"):
        errs.append("缺 layers.lane（车道表必填：路线A带几何；路线B至少给 id/road）")
        return errs
    fields = lane.get("fields", {})
    for k in ("id", "road"):
        if k not in fields:
            errs.append(f"缺 layers.lane.fields.{k}")
    geom = lane.get("geometry", "field")
    if geom not in ("field", "boundaries"):
        errs.append(f"layers.lane.geometry 只能 field|boundaries，得到 {geom!r}")
    need_bnd = geom == "boundaries" or "boundaries" in lane.get("width_from", [])
    if need_bnd and not ("boundary" in layers and "lane_boundary_rel" in layers):
        errs.append("boundaries 路线需要 layers.boundary + layers.lane_boundary_rel（边界线+左右关系）")
    unit = (p.get("units") or {}).get("length", "m")
    if unit not in _LEN_TO_MM:
        errs.append(f"units.length 未知: {unit}（可选 mm|cm|m）")
    return errs


def load_profile(ref: str | Path) -> dict:
    path = Path(ref)
    if not path.exists():
        cand = _ROOT / "profiles" / "shp" / f"{ref}.yaml"
        if cand.exists():
            path = cand
        else:
            raise FileNotFoundError(f"profile 不存在: {ref}（也不在 profiles/shp/ 下）")
    prof = yaml.safe_load(path.read_text(encoding="utf-8"))
    errs = validate_profile(prof)
    if errs:
        raise ValueError("profile 校验失败:\n- " + "\n- ".join(errs))
    prof["_path"] = str(path)
    return prof


class ProfileSource:
    """YAML Profile 驱动的 SHP 源（IbdSource 的通用化替身）。"""

    def __init__(self, shp_dir: str, profile: str | Path | dict):
        self.dir = Path(shp_dir)
        self.p = profile if isinstance(profile, dict) else load_profile(profile)
        errs = validate_profile(self.p)
        if errs:
            raise ValueError("profile 校验失败:\n- " + "\n- ".join(errs))
        self.enc = self.p.get("encoding", "utf-8")
        self.mm = _LEN_TO_MM[(self.p.get("units") or {}).get("length", "m")]
        self.L = self.p["layers"]
        self.derivation_stats = {"width": {"field": 0, "boundaries": 0, "spacing": 0, "default": 0},
                                 "geometry": {"field": 0, "boundaries": 0, "missing": 0},
                                 "seq_inferred": 0, "roads_synthesized": False,
                                 "junction_inferred": False}
        self._junctions = None
        self._roadlinks = None
        self._roadcenters = None
        self._lanes_by_link = None
        self._lane_by_pid = None
        self._merge_pids: set[str] = set()
        self._topo = None
        self._bnd = None
        self._origin = None
        self._stoplines_by_lane = None

    # —— 基础 ——
    def _iter(self, spec, with_shape=True):
        r = shapefile.Reader(str(self.dir / spec["file"]), encoding=self.enc)
        fields = [f[0] for f in r.fields[1:]]
        if with_shape:
            for sr in r.iterShapeRecords():
                yield dict(zip(fields, sr.record)), sr.shape
        else:
            for rec in r.iterRecords():
                yield dict(zip(fields, rec)), None

    def _m(self, pts_deg):
        pts = np.asarray(pts_deg, float)
        if self._origin is None:
            self._origin = (float(pts[0, 0]), float(pts[0, 1]))
        lon0, lat0 = self._origin
        x = np.radians(pts[:, 0] - lon0) * 6378137.0 * math.cos(math.radians(lat0))
        y = np.radians(pts[:, 1] - lat0) * 6378137.0
        return np.column_stack([x, y])

    def _deg(self, pts_m):
        lon0, lat0 = self._origin
        lon = lon0 + np.degrees(pts_m[:, 0] / (6378137.0 * math.cos(math.radians(lat0))))
        lat = lat0 + np.degrees(pts_m[:, 1] / 6378137.0)
        return np.column_stack([lon, lat])

    # —— 路口 ——
    @property
    def junctions(self) -> list[JunctionRec]:
        if self._junctions is None:
            out = []
            spec = self.L.get("junction")
            if spec:
                F = spec["fields"]
                sep = spec.get("list_sep", ";")

                def g(m, key):
                    fld = F.get(key)
                    return _s(m.get(fld, "")) if fld else ""

                for m, shp in self._iter(spec):
                    out.append(JunctionRec(
                        pid=g(m, "id"), name=g(m, "name"),
                        enter_roads=[x for x in g(m, "enter_roads").split(sep) if x],
                        leave_roads=[x for x in g(m, "leave_roads").split(sep) if x],
                        polygon=np.asarray(shp.points)))
            self._junctions = out
        return self._junctions

    def find_junction(self, lon: float, lat: float) -> tuple[JunctionRec, float]:
        if self.junctions:
            best, bd = None, 1e18
            for j in self.junctions:
                c = j.center
                d = math.hypot((c[0] - lon) * 111320 * math.cos(math.radians(lat)),
                               (c[1] - lat) * 110540)
                if d < bd:
                    best, bd = j, d
            return best, bd
        # 无路口层：道路端点聚类推断（假设数字化方向=行车方向，进 REVIEW）
        self.derivation_stats["junction_inferred"] = True
        rad = float(self.p.get("junction_radius_m", 40))
        enter, leave, pts = [], [], []
        for pid in self.roadlinks:
            geo = self.roadcenters.get(pid)
            if geo is None or len(geo) < 2:
                lanes = self.lanes_of(pid)
                if not lanes or lanes[0].geometry.shape[0] < 2:
                    continue
                geo = lanes[len(lanes) // 2].geometry
            for end_i, bucket in ((-1, enter), (0, leave)):
                q = geo[end_i]
                d = math.hypot((q[0] - lon) * 111320 * math.cos(math.radians(lat)),
                               (q[1] - lat) * 110540)
                if d < rad and pid not in enter and pid not in leave:
                    bucket.append(pid)
                    pts.append(q)
                    break
        junc = JunctionRec(pid=f"auto_{lon:.5f}_{lat:.5f}", name="inferred-junction",
                           enter_roads=enter, leave_roads=leave,
                           polygon=np.asarray(pts) if pts else np.array([[lon, lat]]))
        return junc, 0.0

    # —— 道路 ——
    @property
    def roadlinks(self) -> dict[str, RoadLinkRec]:
        if self._roadlinks is None:
            out = {}
            spec = self.L.get("road")
            if spec:
                F = spec["fields"]
                for m, shp in self._iter(spec):
                    pid = _s(m[F["id"]])
                    out[pid] = RoadLinkRec(
                        link_pid=pid,
                        name=_s(m.get(F["name"], "")) if F.get("name") else "",
                        lane_num=_i(m.get(F["lane_count"])) if F.get("lane_count") else 0,
                        s_node="", e_node="",
                        geometry=np.asarray(shp.points) if shp and shp.points else np.zeros((0, 2)))
            else:                                            # 无道路层：按车道 road 字段合成归组
                self._load_lanes()
                for rid, lanes in self._lanes_by_link.items():
                    out[rid] = RoadLinkRec(rid, f"road-{rid}", len(lanes), "", "", np.zeros((0, 2)))
                self.derivation_stats["roads_synthesized"] = True
            self._roadlinks = out
        return self._roadlinks

    @property
    def roadcenters(self) -> dict[str, np.ndarray]:
        if self._roadcenters is None:
            segs: dict[str, list[np.ndarray]] = {}
            spec = self.L.get("road_center")
            if spec:
                F = spec["fields"]
                flag = F.get("is_junction_interior")
                for m, shp in self._iter(spec):
                    if flag and _i(m.get(flag)):
                        continue
                    g = np.asarray(shp.points) if shp and shp.points else None
                    if g is not None and g.shape[0] >= 2:
                        segs.setdefault(_s(m.get(F["road"], "")), []).append(g)
            self._roadcenters = {k: _chain_segments(v) for k, v in segs.items()}
        return self._roadcenters

    # —— 边界（路线 B / 宽度推导） ——
    def fresh_reader(self):
        """Reopen original files; do not reuse cached rows for replay proof."""
        return type(self)(str(self.dir), self.p['_path'])

    def layer_raw_records(self, name):
        """Readonly mapped rows, including duplicates and multipart boundaries.

        Strict source-domain audits must not reconstruct the expected graph
        from an already exported XODR or a deduplicated topology cache.
        """
        import copy
        if name not in self.L:
            raise ValueError('unmapped source layer: ' + name)
        if not hasattr(self, '_raw_layer_rows'):
            self._raw_layer_rows = {}
        if name not in self._raw_layer_rows:
            spec = self.L[name]
            self._raw_layer_rows[name] = tuple({
                'layer': spec['file'], 'record_index': i, 'attributes': fields,
                'parts': self._raw_parts(shape)}
                for i, (fields, shape) in enumerate(self._iter(spec)))
        return copy.deepcopy(self._raw_layer_rows[name])

    @staticmethod
    def _raw_parts(shape):
        """Keep part boundaries; concatenating parts invents connecting segments."""
        if shape is None or not shape.points:
            return ()
        points = tuple(tuple(float(v) for v in p) for p in shape.points)
        starts = tuple(getattr(shape, 'parts', ())) or (0,)
        ends = starts[1:] + (len(points),)
        return tuple(points[a:b] for a, b in zip(starts, ends))

    def lane_raw_records(self, lane_pid: str) -> tuple[dict, ...]:
        """Unmerged source rows for admission checks, not the legacy lane cache.

        Record index is zero-based in its original layer. Duplicate identities,
        multipart and raw attributes are retained. Returned containers are copies.
        """
        import copy
        if not hasattr(self, '_raw_lane_rows'):
            rows = {}
            for name in ('lane', 'lane_merge'):
                spec = self.L.get(name)
                if not spec:
                    continue
                for index, (fields, shape) in enumerate(self._iter(spec)):
                    sid = _s(fields.get(spec['fields']['id'], ''))
                    rows.setdefault(sid, []).append({
                        'layer': spec['file'], 'record_index': index,
                        'source_lane_id': sid, 'attributes': fields,
                        'geometry_origin': spec.get('geometry', 'field'),
                        'parts': self._raw_parts(shape)})
            self._raw_lane_rows = rows
        return tuple(copy.deepcopy(self._raw_lane_rows.get(lane_pid, ())))

    def lane_boundary_records(self, lane_pid: str) -> tuple[dict, ...]:
        """Original boundary references without chaining, ordering or ID loss.

        SIDE is the Profile declaration, not a world-coordinate left/right
        judgement. Unknown SIDE and missing/duplicate features remain visible.
        This new strict API does not change the legacy conversion path.
        """
        import copy
        if not hasattr(self, '_raw_boundary_rows'):
            features, relations = {}, {}
            bs, rs = self.L.get('boundary'), self.L.get('lane_boundary_rel')
            if bs and rs:
                for index, (fields, shape) in enumerate(self._iter(bs)):
                    bid = _s(fields.get(bs['fields']['id'], ''))
                    features.setdefault(bid, []).append({
                        'layer': bs['file'], 'record_index': index,
                        'boundary_id': bid, 'attributes': fields,
                        'parts': self._raw_parts(shape)})
                side_of = {str(v): k for k, v in
                           (rs.get('side_values') or {'left': 1, 'right': 2}).items()}
                rf = rs['fields']
                for index, (fields, _) in enumerate(self._iter(rs, with_shape=False)):
                    sid = _s(fields.get(rf['lane'], ''))
                    raw_side = _s(fields.get(rf['side'], ''))
                    relations.setdefault(sid, []).append({
                        'relation_layer': rs['file'], 'relation_index': index,
                        'source_lane_id': sid,
                        'boundary_id': _s(fields.get(rf['boundary'], '')),
                        'declared_side': side_of.get(raw_side), 'raw_side': raw_side,
                        'attributes': fields})
            self._raw_boundary_rows = features, relations
        features, relations = self._raw_boundary_rows
        return tuple(dict(copy.deepcopy(r), records=tuple(copy.deepcopy(
            features.get(r['boundary_id'], ())))) for r in relations.get(lane_pid, ()))

    def _load_boundaries(self):
        if self._bnd is not None:
            return
        geom: dict[str, list[np.ndarray]] = {}
        rel: dict[str, dict[str, list[str]]] = {}
        bspec, rspec = self.L.get("boundary"), self.L.get("lane_boundary_rel")
        if bspec and rspec:
            BF = bspec["fields"]
            for m, shp in self._iter(bspec):
                if shp and shp.points:
                    geom.setdefault(_s(m[BF["id"]]), []).append(np.asarray(shp.points))
            RF = rspec["fields"]
            side_of = {str(v): k for k, v in
                       (rspec.get("side_values") or {"left": 1, "right": 2}).items()}
            for m, _sh in self._iter(rspec, with_shape=False):
                side = side_of.get(_s(m.get(RF["side"], "")))
                if side:
                    rel.setdefault(_s(m[RF["lane"]]), {}).setdefault(side, []) \
                       .append(_s(m[RF["boundary"]]))
        self._bnd = (geom, rel)

    def _side_pts(self, lane_pid, side):
        geom, rel = self._bnd
        bids = rel.get(lane_pid, {}).get(side, [])
        segs = [g for b in bids for g in geom.get(b, [])]
        return segs or None

    def lane_boundary_geometries(self, lane_pid: str) -> list[np.ndarray]:
        """返回车道关联的两条真实边界折线。

        图商的 left/right 枚举可能随车道记录方向变化，调用方不应依赖列表顺序；
        应在自身参考线坐标系中按横距重新判定内/外侧。分段边界在这里先按端点
        拼成连续折线，避免转换器退回“中心线 ± 半宽”的合成边缘。
        """
        self._load_boundaries()
        out = []
        for side in ("left", "right"):
            segs = self._side_pts(lane_pid, side)
            if not segs:
                continue
            valid = [np.asarray(g, float) for g in segs if np.asarray(g).shape[0] >= 2]
            if not valid:
                continue
            geom = _chain_segments(valid)
            if geom.shape[0] >= 2:
                out.append(geom)
        return out

    def _width_from_boundaries(self, lane_pid) -> int | None:
        self._load_boundaries()
        lsegs, rsegs = self._side_pts(lane_pid, "left"), self._side_pts(lane_pid, "right")
        if not lsegs or not rsegs:
            return None
        Lm, Rm = self._m(np.vstack(lsegs)), self._m(np.vstack(rsegs))
        sub = Lm[:: max(1, len(Lm) // 15)]
        d = np.linalg.norm(Rm[None, :, :] - sub[:, None, :], axis=2).min(axis=1)
        return int(round(float(np.median(d)) * 1000))

    def _geom_from_boundaries(self, lane_pid) -> np.ndarray | None:
        self._load_boundaries()
        lsegs, rsegs = self._side_pts(lane_pid, "left"), self._side_pts(lane_pid, "right")
        if not lsegs or not rsegs:
            return None
        left = _chain_segments(sorted(lsegs, key=lambda g: -g.shape[0]))
        right = _chain_segments(sorted(rsegs, key=lambda g: -g.shape[0]))
        mid_m = midline_of(self._m(left), self._m(right))
        return self._deg(mid_m)

    # —— 车道 ——
    def _load_lanes(self):
        if self._lanes_by_link is not None:
            return
        spec = self.L["lane"]
        F = spec["fields"]
        geom_mode = spec.get("geometry", "field")
        ladder = spec.get("width_from", ["field", "boundaries", "spacing", "default"])
        default_mm = int(float(spec.get("default_width_m", 3.5)) * 1000)
        by_link: dict[str, list[LaneRec]] = {}
        by_pid: dict[str, LaneRec] = {}

        def read_layer(lspec, merge):
            LF = lspec["fields"]
            for m, shp in self._iter(lspec):
                pid, rid = _s(m[LF["id"]]), _s(m.get(LF["road"], ""))
                w = int(round(_f(m.get(LF["width"])) * self.mm)) if LF.get("width") else 0
                if geom_mode == "field":
                    g = np.asarray(shp.points) if shp and shp.points else np.zeros((0, 2))
                else:
                    g = self._geom_from_boundaries(pid)
                    g = g if g is not None else np.zeros((0, 2))
                self.derivation_stats["geometry"][
                    "missing" if g.shape[0] < 2 else geom_mode] += 1
                rec = LaneRec(
                    lane_pid=pid, link_pid=rid,
                    seq=_i(m.get(LF["seq"])) if LF.get("seq") else 0,
                    width_mm=w if "field" in ladder else 0,
                    lane_type=_i(m.get(LF["lane_type"])) if LF.get("lane_type") else 0,
                    max_speed_kmh=_i(m.get(LF["max_speed"])) if LF.get("max_speed") else 0,
                    geometry=g,
                    s_width_mm=int(round(_f(m.get(LF["width_start"])) * self.mm))
                    if LF.get("width_start") else 0,
                    e_width_mm=int(round(_f(m.get(LF["width_end"])) * self.mm))
                    if LF.get("width_end") else 0,
                    s_width_known=bool(LF.get("width_start")) and _known_width(m.get(LF.get("width_start"))),
                    e_width_known=bool(LF.get("width_end")) and _known_width(m.get(LF.get("width_end"))),
                    geometry_source=(geom_mode if g.shape[0] >= 2 else "missing"),
                    width_source=("field" if w > 0 and "field" in ladder else "missing"))
                if merge:
                    self._merge_pids.add(pid)
                    by_pid.setdefault(pid, rec)
                    continue
                by_pid[pid] = rec
                by_link.setdefault(rid, []).append(rec)

        read_layer(spec, merge=False)
        if self.L.get("lane_merge"):
            read_layer(self.L["lane_merge"], merge=True)

        for rid, lanes in by_link.items():
            # SEQ 缺失：按文件序补 1..n（横向次序未证，进 REVIEW）
            if all(l.seq == 0 for l in lanes) and lanes:
                for i, l in enumerate(lanes):
                    l.seq = i + 1
                self.derivation_stats["seq_inferred"] += len(lanes)
            seen, uniq = set(), []
            for rec in sorted(lanes, key=lambda x: x.seq):
                if rec.lane_pid not in seen:
                    seen.add(rec.lane_pid)
                    uniq.append(rec)
            by_link[rid] = uniq
            # 宽度阶梯（road 内统一处理 spacing）
            missing = [l for l in uniq if l.width_mm <= 0]
            for l in uniq:
                if l.width_mm > 0:
                    self.derivation_stats["width"]["field"] += 1
            if missing and "boundaries" in ladder:
                for l in list(missing):
                    w = self._width_from_boundaries(l.lane_pid)
                    if w:
                        l.width_mm = w
                        l.width_source = "boundaries"
                        missing.remove(l)
                        self.derivation_stats["width"]["boundaries"] += 1
            if missing and "spacing" in ladder and len(uniq) >= 2:
                spac = []
                for l in uniq:
                    if l.geometry.shape[0] < 2:
                        continue
                    gm = self._m(l.geometry)[:: max(1, l.geometry.shape[0] // 12)]
                    ds = [float(np.median(np.linalg.norm(
                        self._m(o.geometry)[None, :, :] - gm[:, None, :], axis=2).min(axis=1)))
                        for o in uniq if o is not l and o.geometry.shape[0] >= 2]
                    if ds:
                        spac.append(min(ds))
                if spac:
                    w = int(round(float(np.median(spac)) * 1000))
                    for l in list(missing):
                        l.width_mm = w
                        l.width_source = "spacing"
                        missing.remove(l)
                        self.derivation_stats["width"]["spacing"] += 1
            for l in missing:
                l.width_mm = default_mm
                l.width_source = "default"
                self.derivation_stats["width"]["default"] += 1
        self._lanes_by_link, self._lane_by_pid = by_link, by_pid

    def lanes_of(self, link_pid: str) -> list[LaneRec]:
        self._load_lanes()
        return self._lanes_by_link.get(link_pid, [])

    def lane(self, lane_pid: str) -> LaneRec | None:
        self._load_lanes()
        return self._lane_by_pid.get(lane_pid)

    def is_merge(self, lane_pid: str) -> bool:
        self._load_lanes()
        return lane_pid in self._merge_pids

    @property
    def stoplines_by_lane(self) -> dict[str, list[StopLineRec]]:
        if self._stoplines_by_lane is None:
            out: dict[str, list[StopLineRec]] = {}
            spec = self.L.get("stop_line")
            if spec:
                F = spec["fields"]
                sep = spec.get("list_sep", ";")
                for m, shp in self._iter(spec):
                    refs = [x for x in _s(m.get(F.get("lane_refs"), "")).split(sep) if x]
                    rec = StopLineRec(
                        object_pid=_s(m.get(F.get("id"), "")), lane_pids=refs,
                        geometry=np.asarray(shp.points) if shp and shp.points else np.zeros((0, 2)),
                        width_mm=int(round(_f(m.get(F.get("width"))) * self.mm))
                        if F.get("width") else 0,
                    )
                    for lane_pid in refs:
                        out.setdefault(lane_pid, []).append(rec)
            self._stoplines_by_lane = out
        return self._stoplines_by_lane

    # —— 拓扑 ——
    def _load_topo(self):
        if self._topo is None:
            out, inv = {}, {}
            spec = self.L.get("topo")
            if spec:
                F = spec["fields"]
                for m, _sh in self._iter(spec, with_shape=False):
                    a, b = _s(m[F["from"]]), _s(m[F["to"]])
                    out.setdefault(a, []).append(b)
                    inv.setdefault(b, []).append(a)
            self._topo = (out, inv)

    @property
    def topo_out(self) -> dict[str, list[str]]:
        self._load_topo()
        return self._topo[0]

    @property
    def topo_in(self) -> dict[str, list[str]]:
        self._load_topo()
        return self._topo[1]


TEMPLATE_YAML = """\
# mapforge SHP Profile 模板 —— 一家图商一份，按对方交付改字段名后即可 convert --profile 使用
# 校验命令：python -m mapforge.cli profile-check <本文件> <shp目录>
profile: my-vendor-v1            # 命名：图商-规格-版本
encoding: utf-8                  # DBF 编码（IBD 实为 gbk；声明不可信，profile-check 会抽样验证）
units:
  length: m                      # 宽度等长度字段单位：mm | cm | m（IBD 是 mm）
crs:
  declared: EPSG:4326            # 声称坐标系（原样记录）
  verified: false                # 人工核验前保持 false —— 核验通过前不得进生产转换（硬约束3）

layers:
  # ============ 必填：车道表（二路线择一） ============
  lane:
    file: LANE_CENTER            # shapefile 名（不带 .shp）
    geometry: field              # field=本层自带车道中心线几何（路线A）；boundaries=由边界层合成（路线B）
    width_from: [field, spacing, default]   # 宽度获取阶梯，非 field 记 APPROXIMATED；
                                            # 配好下方边界两层后可在 field 后插入 boundaries
    default_width_m: 3.5
    fields:
      id: LANE_ID                # 必填：车道唯一 ID
      road: ROAD_ID              # 必填：所属道路（归组键）
      seq: LANE_NO               # 车道序号（缺省按文件序补，进 REVIEW）
      width: WIDTH               # 宽度字段（路线B可删）
      # width_start: S_WIDTH     # 变宽车道起/止宽（可选）
      # width_end: E_WIDTH
      # lane_type: TYPE          # 可选
      # max_speed: SPEED         # 可选

  # ============ 路线 B / 宽度推导需要：边界线 + 左右关系 ============
  # boundary:
  #   file: LANE_BORDER
  #   fields: {id: BORDER_ID}
  # lane_boundary_rel:
  #   file: LANE_BORDER_REL
  #   side_values: {left: 1, right: 2}    # SIDE 字段取值 → 左/右
  #   fields: {lane: LANE_ID, boundary: BORDER_ID, side: SIDE}

  # ============ 可选：道路层（缺省由车道 road 字段合成归组） ============
  road:
    file: ROAD_LINK
    fields: {id: ROAD_ID, name: ROAD_NAME, lane_count: LANE_NUM}

  # ============ 可选：道路中心线（缺省用中间车道替代） ============
  # road_center:
  #   file: ROAD_CENTER
  #   fields: {road: ROAD_ID, is_junction_interior: IS_JUNC}

  # ============ 可选：路口面（缺省用 --at lon,lat + 端点聚类推断，进 REVIEW） ============
  # junction:
  #   file: INTERSECTION
  #   list_sep: ";"
  #   fields: {id: JUNC_ID, name: NAME, enter_roads: ENTER, leave_roads: LEAVE}

  # ============ 可选：车道级接续拓扑（缺失则 junction 连接为空——几何推断路线未实装） ============
  # topo:
  #   file: LANE_TOPO
  #   fields: {from: IN_ID, to: OUT_ID}

# junction_radius_m: 40          # 无路口层时端点聚类半径
"""
