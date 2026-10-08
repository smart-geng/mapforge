"""Bounded, source-bound shared-boundary edit on an existing straight road.

This is one local edit capability, not a conversion or a delivery decision.
The unchanged reference, lane offsets, topology and all other physical edges
are invariants. The only control is one compact C2 cubic B-spline amplitude;
adjacent widths receive opposite additions. No solver or default is patched.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from scipy.interpolate import BSpline

from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.repair_web.model import (coefficients, complexity, extrema,
                                       intervals, lanes, parse, shifted)
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from scripts.internal_edge_jets import active, section, states

SCHEMA = "mapforge/shared-boundary-edit/v1"
MIN_NEW_SPAN_M = 6.0
MAX_NEW_WIDTH_RECORDS = 8
MAX_NORMAL_DELTA_M = 0.1


class BoundaryEditRejected(ValueError):
    """Unsupported identity/representation/target; baseline remains untouched."""


def _json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf8")


def _hash(data):
    return hashlib.sha256(data).hexdigest()


def _file_hash(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _user(lane, code):
    rows = lane.findall(f"userData[@code='{code}']")
    if len(rows) != 1:
        raise BoundaryEditRejected(f"缺失或重复来源字段 {code}")
    return rows[0].get("value")


def _identity(lane):
    sid = _user(lane, "mapforge.source_lane")
    try:
        p = json.loads(_user(lane, "mapforge.provenance/v1"))
    except (TypeError, ValueError) as exc:
        raise BoundaryEditRejected("无法读取来源角色") from exc
    if (not sid or p.get("eligibility") != "comparable"
            or p.get("boundary_evidence_trusted") is not True
            or p.get("policy_class") not in {"shp.field-leg", "shp.field-approach"}
            or p.get("support_kind") != "shp-field-centerline"
            or p.get("status") not in {"EXACT", "TRANSFORMED"}
            or p.get("travel_direction") != "with_s"
            or any(p.get(k) for k in ("exclusion_code", "geometry_adjustment", "source_role"))):
        raise BoundaryEditRejected("仅支持已绑定可信边界的普通同向道路；事件/推断/调整角色不准入")
    return sid


def _admit(data, road_id, inner_lane_id, knots):
    if (not isinstance(road_id, str) or isinstance(inner_lane_id, bool)
            or not isinstance(inner_lane_id, int) or inner_lane_id == 0):
        raise BoundaryEditRejected("道路和车道身份无效")
    if (not isinstance(knots, (list, tuple)) or len(knots) != 5
            or not all(_finite(x) for x in knots)
            or min(np.diff(knots)) < MIN_NEW_SPAN_M - 1e-9):
        raise BoundaryEditRejected("需要 5 个递增节点，独立基函数跨度至少 6m")
    root = parse(data)
    roads = root.findall(f"road[@id='{road_id}']")
    if len(roads) != 1:
        raise BoundaryEditRejected("道路身份不存在或不唯一")
    road = roads[0]
    gs = road.findall("planView/geometry")
    if (road.get("junction") != "-1" or road.get("name") == "junction_paving"
            or len(gs) != 1 or gs[0][0].tag != "line"
            or float(gs[0].get("s")) != 0
            or abs(float(gs[0].get("length")) - float(road.get("length"))) > 1e-7):
        raise BoundaryEditRejected("首工具只支持完整单 Line 普通道路")
    if not 5 <= knots[0] < knots[-1] <= float(road.get("length")) - 5:
        raise BoundaryEditRejected("编辑域必须距道路端部至少 5m")
    neighbor = inner_lane_id + (1 if inner_lane_id > 0 else -1)
    selected = [(sec, lo, hi) for sec, lo, hi in intervals(road)
                if lo < knots[-1] and hi > knots[0]]
    identities = []
    new_records = 0
    for index, (sec, lo, hi) in enumerate(selected):
        ids = []
        for lid in (inner_lane_id, neighbor):
            lane = lanes(sec).get(lid)
            if (lane is None or lane.get("type") != "driving" or lane.findall("border")
                    or not lane.findall("width")):
                raise BoundaryEditRejected("作用域内必须始终是两条相邻 width 驱动车道")
            ids.append(_identity(lane))
            widths = lane.findall("width")
            starts = [lo + float(w.get("sOffset")) for w in widths]
            if starts[0] != lo or any(b <= a for a, b in zip(starts, starts[1:])):
                raise BoundaryEditRejected("宽度记录起点或顺序无效")
            cuts = sorted(set(starts + [hi]))
            for knot in knots:
                if lo < knot < hi and not any(abs(knot - c) < 1e-8 for c in cuts):
                    before = max(c for c in cuts if c < knot)
                    after = min(c for c in cuts if c > knot)
                    if min(knot - before, after - knot) < MIN_NEW_SPAN_M - 1e-8:
                        raise BoundaryEditRejected("新宽度节点会产生不足 6m 的细分段")
                    cuts.append(knot)
                    cuts.sort()
                    new_records += 1
            if index:
                previous = lanes(selected[index - 1][0])[lid]
                a, b = previous.find("link/successor"), lane.find("link/predecessor")
                if a is None or b is None or a.get("id") != str(lid) or b.get("id") != str(lid):
                    raise BoundaryEditRejected("作用域内缺少显式同车道双向连接")
        identities.append({"section_s": lo, "source_lane_ids": ids})
    if not selected or new_records > MAX_NEW_WIDTH_RECORDS:
        raise BoundaryEditRejected("没有可编辑区间或新增宽度记录超过 8 条预算")
    return root, road, neighbor, identities, new_records


def _chain(records, start_field, end_field):
    unique = []
    seen = set()
    for row in records:
        key = (row["layer"], row["record_index"])
        if key not in seen:
            seen.add(key)
            unique.append(row)
    points, previous_end = [], None
    for row in unique:
        if len(row["parts"]) != 1 or len(row["parts"][0]) < 2:
            raise BoundaryEditRejected("共享边界链含缺失或多部件几何，不能补造连线")
        part = [list(p[:2]) for p in row["parts"][0]]
        start, end = str(row["attributes"].get(start_field)), str(row["attributes"].get(end_field))
        if previous_end is not None:
            if start != previous_end:
                raise BoundaryEditRejected("源边界记录未通过显式端节点连续性检查")
            if not np.allclose(points[-1], part[0], rtol=0, atol=1e-12):
                raise BoundaryEditRejected("源边界同节点坐标不相同")
            part = part[1:]
        points.extend(part)
        previous_end = end
    return points


def build_source_binding(data: bytes, source_dir, profile, road_id: str,
                         inner_lane_id: int, knots: list[float], *,
                         include_neighbor_context: bool = True) -> dict:
    """Build on the server from original rows; never accept client geometry.

    The local comparison projection mirrors the existing XODR producer. It is
    explicitly NOT verification of the source's absolute CRS.
    """
    root, road, neighbor, identities, _ = _admit(data, road_id, inner_lane_id, knots)
    src = ProfileSource(str(source_dir), profile)
    files = {str(Path(src.p["_path"]).resolve())}
    for name in ("lane", "lane_merge", "boundary", "lane_boundary_rel"):
        if name in src.L:
            stem = src.L[name]["file"]
            matches = list(Path(source_dir).glob(stem + ".*"))
            if not matches:
                raise BoundaryEditRejected("源图层文件缺失: " + stem)
            files.update(str(p.resolve()) for p in matches if p.is_file())
    hashes = {p: _file_hash(p) for p in sorted(files)}
    # One neighboring section supplies the real continuation on each side.
    # This does not expand the editable domain or infer a source connection.
    sections = intervals(road)
    indexes = [i for i, (_, lo, hi) in enumerate(sections) if lo < knots[-1] and hi > knots[0]]
    context = (sections[max(0, indexes[0] - 1):min(len(sections), indexes[-1] + 2)]
               if include_neighbor_context else sections[indexes[0]:indexes[-1] + 1])
    rows, chains = [], {"shared": [], "fixed_inner": [], "fixed_outer": []}
    for sec, lo, _ in context:
        pair = [_identity(lanes(sec)[lid]) for lid in (inner_lane_id, neighbor)]
        relation_pairs = []
        editable_section = any(item["section_s"] == lo for item in identities)
        for sid in pair:
            raw = src.lane_raw_records(sid)
            if not raw or (editable_section and len(raw) != 1):
                raise BoundaryEditRejected("源车道业务 ID 缺失或重名，不能挑选一条")
            relations = list(src.lane_boundary_records(sid))
            if (len(relations) != 2 or {r["declared_side"] for r in relations} != {"left", "right"}
                    or any(len(r["records"]) != 1 for r in relations)):
                raise BoundaryEditRejected("两侧边界关系必须唯一完整")
            relation_pairs.append(relations)
        common = {r["boundary_id"] for r in relation_pairs[0]} & {r["boundary_id"] for r in relation_pairs[1]}
        if len(common) != 1:
            raise BoundaryEditRejected("相邻车道没有唯一同身份共享边界")
        common_id = next(iter(common))
        shared = next(r["records"][0] for r in relation_pairs[0] if r["boundary_id"] == common_id)
        other_shared = next(r["records"][0] for r in relation_pairs[1] if r["boundary_id"] == common_id)
        if _json(shared) != _json(other_shared):
            raise BoundaryEditRejected("共享边界两侧引用并非同一源记录")
        fixed = [next(r["records"][0] for r in rels if r["boundary_id"] != common_id)
                 for rels in relation_pairs]
        chains["shared"].append(shared)
        chains["fixed_inner"].append(fixed[0])
        chains["fixed_outer"].append(fixed[1])
        rows.append({"section_s": lo, "source_lane_ids": pair, "shared_boundary_id": common_id,
                     "editable_section": editable_section,
                     "lane_records": [src.lane_raw_records(sid) for sid in pair],
                     "relations": relation_pairs})
    lat0, lon0 = _origin(root)
    fields = src.L["boundary"]["fields"]
    polylines = {key: _project(_chain(value, fields["start_node"], fields["end_node"]),
                                lat0, lon0).tolist() for key, value in chains.items()}
    if any(_file_hash(p) != digest for p, digest in hashes.items()):
        raise BoundaryEditRejected("读取期间源文件发生变化")
    return {"schema": SCHEMA + "/source", "baseline_sha256": _hash(data),
            "road_id": road_id, "inner_lane_id": inner_lane_id, "knots": list(knots),
            "identities": identities, "input_files_sha256": hashes,
            "profile_sha256": hashes[str(Path(src.p["_path"]).resolve())],
            "source_rows": rows, "local_polylines": polylines,
            "projection": {"kind": "existing-producer-local-equirectangular",
                           "lat0": lat0, "lon0": lon0, "R_m": 6378137.0,
                           "absolute_crs": "UNVERIFIED", "production_authority": False}}


@dataclass(frozen=True)
class PreparedBoundaryEdit:
    data: bytes
    binding_json: bytes
    road_id: str
    inner_lane_id: int
    neighbor: int
    knots: tuple[float, ...]
    new_width_records: int

    def capability(self) -> dict:
        return {"schema": SCHEMA, "kind": "shared_boundary_c2_normal_delta", "enabled": True,
                "baseline_sha256": _hash(self.data), "source_binding_sha256": _hash(self.binding_json),
                "road_id": self.road_id, "lane_ids": [self.inner_lane_id, self.neighbor],
                "scope_s_m": [self.knots[0], self.knots[-1]], "knots": list(self.knots),
                "normal_delta_limit_m": MAX_NORMAL_DELTA_M,
                "new_width_records": self.new_width_records,
                "constraints": ["fixed_reference", "fixed_other_edges", "fixed_topology", "endpoint_C2"],
                "qualification": "LOCAL_EDIT_ONLY", "delivery": "BLOCKED"}


def prepare_boundary_edit(data: bytes, source_binding: dict, road_id: str,
                          inner_lane_id: int, knots: list[float]) -> PreparedBoundaryEdit:
    """Validate a server-built source binding and return immutable edit input."""
    _, road, neighbor, identities, new_records = _admit(data, road_id, inner_lane_id, knots)
    expected = {"schema": SCHEMA + "/source", "baseline_sha256": _hash(data), "road_id": road_id,
                "inner_lane_id": inner_lane_id, "knots": list(knots), "identities": identities}
    if any(source_binding.get(k) != v for k, v in expected.items()):
        raise BoundaryEditRejected("源绑定与当前候选/作用域/身份不一致")
    manifest = source_binding.get("input_files_sha256")
    if not isinstance(manifest, dict) or not manifest or not source_binding.get("source_rows"):
        raise BoundaryEditRejected("源原件与关系记录绑定不完整")
    for path, digest in manifest.items():
        if not Path(path).is_file() or _file_hash(path) != digest:
            raise BoundaryEditRejected("源绑定文件已失效: " + path)
    polylines = source_binding.get("local_polylines", {})
    if set(polylines) != {"shared", "fixed_inner", "fixed_outer"}:
        raise BoundaryEditRejected("缺少两侧完整物理边界")
    # The physical source, rather than narrower derived center support_s,
    # must cover every editable station. Ambiguous crossings are rejected.
    _source_samples(road, polylines, _stations(knots[0], knots[-1]))
    return PreparedBoundaryEdit(data, _json(source_binding), road_id, inner_lane_id,
                                neighbor, tuple(float(k) for k in knots), new_records)


def _stations(lo, hi):
    return np.linspace(lo, hi, max(2, math.ceil((hi - lo) / 0.1) + 1))


def _source_samples(road, polylines, stations):
    g = road.find("planView/geometry")
    h = float(g.get("hdg"))
    tangent = np.array([math.cos(h), math.sin(h)])
    normal = np.array([-math.sin(h), math.cos(h)])
    origin = np.array([float(g.get("x")), float(g.get("y"))])
    result = {}
    for key, points in polylines.items():
        p = np.asarray(points, dtype=float)
        if p.ndim != 2 or p.shape[1] != 2 or len(p) < 2 or not np.all(np.isfinite(p)):
            raise BoundaryEditRejected("源边界几何无效")
        s, t = (p - origin) @ tangent, (p - origin) @ normal
        if np.all(np.diff(s) < 0):
            s, t = s[::-1], t[::-1]
        if np.any(np.diff(s) <= 1e-8):
            raise BoundaryEditRejected("源边界相对参考轴非单值；当前单工具不支持")
        if stations[0] < s[0] - 1e-8 or stations[-1] > s[-1] + 1e-8:
            raise BoundaryEditRejected("源物理边界不能覆盖完整编辑域")
        result[key] = np.interp(stations, s, t)
    return result


def _stats(values):
    a = np.abs(np.asarray(values))
    return {"max_m": float(a.max()), "p95_m": float(np.percentile(a, 95)),
            "mean_m": float(a.mean()), "samples": len(a)}


def source_residuals(data: bytes, prepared: PreparedBoundaryEdit) -> dict:
    """Normal residuals to the SAME physical source; no nearest-edge matching.

    Sampling is evidence only (0.1m); no sampled point is serialized as geometry.
    Centers here mean source-boundary midpoints, not inferred source centerlines.
    """
    road = parse(data).find(f"road[@id='{prepared.road_id}']")
    ss = _stations(prepared.knots[0], prepared.knots[-1])
    bound = json.loads(prepared.binding_json)
    source = _source_samples(road, bound["local_polylines"], ss)
    g = road.find("planView/geometry")
    h = float(g.get("hdg"))
    normal = np.array([-math.sin(h), math.cos(h)])
    origin = np.array([float(g.get("x")), float(g.get("y"))])
    a = np.array([states(road, prepared.inner_lane_id, float(s), False) for s in ss])
    b = np.array([states(road, prepared.neighbor, float(s), False) for s in ss])
    target = {"shared": (a[:, 1, :2] - origin) @ normal,
              "inner_lane_center": ((a[:, 0, :2] + a[:, 1, :2]) / 2 - origin) @ normal,
              "outer_lane_center": ((b[:, 0, :2] + b[:, 1, :2]) / 2 - origin) @ normal}
    expected = {"shared": source["shared"],
                "inner_lane_center": (source["shared"] + source["fixed_inner"]) / 2,
                "outer_lane_center": (source["shared"] + source["fixed_outer"]) / 2}
    return {"metric": "same-source normal residual; centers=physical-boundary midpoint",
            "scope_s_m": [prepared.knots[0], prepared.knots[-1]], "sample_spacing_max_m": 0.1,
            "items": {key: _stats(target[key] - expected[key]) for key in target},
            "shared_signed_residual_range_m": [float((expected["shared"] - target["shared"]).min()),
                                                float((expected["shared"] - target["shared"]).max())]}


def _semantic_without_pair_widths(root, road_id, pair):
    root = copy.deepcopy(root)
    road = root.find(f"road[@id='{road_id}']")
    for sec, _, _ in intervals(road):
        for lid in pair:
            lane = lanes(sec).get(lid)
            if lane is not None:
                for w in lane.findall("width"):
                    lane.remove(w)
    return ET.tostring(root)


def _width_jet(road, lid, station, left):
    sec = section(road, station, left)
    local = station - float(sec.get("s"))
    w = active(lanes(sec)[lid].findall("width"), local, left,
               lambda node: float(node.get("sOffset")))
    poly = np.polynomial.Polynomial(coefficients(w))
    offset = local - float(w.get("sOffset"))
    return np.array([poly.deriv(i)(offset) for i in range(3)])


def compile_boundary_edit(prepared: PreparedBoundaryEdit, normal_delta_m: float) -> tuple[bytes, dict]:
    """Write and read back an isolated candidate; failures never replace input.

    Local proof cannot upgrade CRS, phase/ID, whole-map validation or release.
    A nonzero accepted compile is not necessarily a source-quality improvement.
    """
    if not _finite(normal_delta_m) or abs(normal_delta_m) > MAX_NORMAL_DELTA_M:
        raise BoundaryEditRejected("法向位移必须是 ±0.1m 内有限数值")
    binding = json.loads(prepared.binding_json)
    # Replay must recheck original bytes and source files, not trust a UI token.
    check = prepare_boundary_edit(prepared.data, binding, prepared.road_id,
                                   prepared.inner_lane_id, list(prepared.knots))
    if check != prepared:
        raise BoundaryEditRejected("编辑能力输入不一致")
    root = parse(prepared.data)
    before = copy.deepcopy(root)
    road = root.find(f"road[@id='{prepared.road_id}']")
    old_road = before.find(f"road[@id='{prepared.road_id}']")
    lo_scope, hi_scope = prepared.knots[0], prepared.knots[-1]
    b = BSpline.basis_element(prepared.knots, extrapolate=False)
    anchor = (prepared.knots[1] + prepared.knots[3]) / 2
    scale = float(b(anchor))
    touched, minimum = 0, math.inf
    if normal_delta_m:
        for sec, lo, hi in intervals(road):
            if lo >= hi_scope or hi <= lo_scope:
                continue
            for lid, sign in ((prepared.inner_lane_id, 1 if prepared.inner_lane_id > 0 else -1),
                              (prepared.neighbor, -1 if prepared.inner_lane_id > 0 else 1)):
                lane = lanes(sec)[lid]
                originals = copy.deepcopy(lane.findall("width"))
                for knot in prepared.knots:
                    if lo < knot < hi and not any(abs(lo + float(w.get("sOffset")) - knot) < 1e-8
                                                  for w in lane.findall("width")):
                        c = shifted(originals, "sOffset", knot - lo)
                        node = ET.Element("width", sOffset=format(knot - lo, ".17g"),
                                          **{k: format(float(v), ".17g") for k, v in zip("abcd", c)})
                        widths = lane.findall("width")
                        following = next((w for w in widths if float(w.get("sOffset")) > knot - lo), None)
                        lane.insert(list(lane).index(following) if following is not None
                                    else list(lane).index(widths[-1]) + 1, node)
                widths = lane.findall("width")
                for i, w in enumerate(widths):
                    start = lo + float(w.get("sOffset"))
                    end = lo + float(widths[i + 1].get("sOffset")) if i + 1 < len(widths) else hi
                    mid = (start + end) / 2
                    if not lo_scope < mid < hi_scope:
                        continue
                    cm = [float(b(mid, nu=d)) / math.factorial(d) for d in range(4)]
                    poly = np.polynomial.Polynomial(cm)(np.polynomial.Polynomial([start - mid, 1]))
                    c = coefficients(w) + sign * normal_delta_m * np.pad(poly.coef, (0, 4 - len(poly.coef))) / scale
                    for key, value in zip("abcd", c):
                        w.set(key, format(float(value), ".17g"))
                    minimum = min(minimum, extrema(c, end - start))
                    touched += 1
        if minimum < -1e-9:
            raise BoundaryEditRejected(f"实际多项式出现负宽 {minimum:.9g}m")
    data = prepared.data if not normal_delta_m else ET.tostring(root, encoding="utf-8", xml_declaration=True)
    actual = parse(data)  # All proof below uses the real serialized file bytes.
    actual_road = actual.find(f"road[@id='{prepared.road_id}']")
    pair = (prepared.inner_lane_id, prepared.neighbor)
    if (_semantic_without_pair_widths(before, prepared.road_id, pair)
            != _semantic_without_pair_widths(actual, prepared.road_id, pair)):
        raise BoundaryEditRejected("宽度之外的语义发生变化")
    cb, ca = complexity(before), complexity(actual)
    expected_new = prepared.new_width_records if normal_delta_m else 0
    if any(ca[k] != cb[k] for k in cb if k != "width") or ca["width"] - cb["width"] != expected_new:
        raise BoundaryEditRejected("实际文件超出复杂度预算")
    # Compare exact cubic coefficient sums on the union of all written cuts.
    # This proves all outer edges stay fixed throughout, not merely at samples.
    sum_error, outside_error = 0.0, 0.0
    for (sec, lo, hi), (old_sec, _, _) in zip(intervals(actual_road), intervals(old_road)):
        if not set(pair) <= set(lanes(sec)):
            continue
        cuts = sorted({lo, hi, *[k for k in prepared.knots if lo < k < hi],
                       *[lo + float(w.get("sOffset")) for lid in pair
                         for s in (sec, old_sec) for w in lanes(s)[lid].findall("width")]})
        for start, end in zip(cuts, cuts[1:]):
            differences = [shifted(lanes(sec)[lid].findall("width"), "sOffset", start - lo)
                           - shifted(lanes(old_sec)[lid].findall("width"), "sOffset", start - lo)
                           for lid in pair]
            sum_error = max(sum_error, float(np.max(np.abs(sum(differences)))))
            if end <= lo_scope or start >= hi_scope:
                outside_error = max(outside_error, float(np.max(np.abs(differences))))
    if sum_error > 1e-11 or outside_error > 1e-11:
        raise BoundaryEditRejected("域外或外部物理边界的解析不变量失败")
    delta_c2_error = 0.0
    all_cuts = {lo + float(w.get("sOffset")) for sec, lo, _ in intervals(actual_road)
                for lid in pair if lid in lanes(sec) for w in lanes(sec)[lid].findall("width")}
    for station in sorted(c for c in all_cuts if lo_scope <= c <= hi_scope):
        for lid in pair:
            delta_left = (_width_jet(actual_road, lid, station, True)
                          - _width_jet(old_road, lid, station, True))
            delta_right = (_width_jet(actual_road, lid, station, False)
                           - _width_jet(old_road, lid, station, False))
            delta_c2_error = max(delta_c2_error, float(np.max(np.abs(delta_left - delta_right))))
    if delta_c2_error > 1e-10:
        raise BoundaryEditRejected("实际读回的位移场在内部断点未保持 C2")
    endpoint_error = 0.0
    shared_error = 0.0
    for station in prepared.knots:
        for left in (True, False):
            a = states(actual_road, prepared.inner_lane_id, station, left)
            other = states(actual_road, prepared.neighbor, station, left)
            shared_error = max(shared_error, float(np.max(np.abs(np.array(a[1]) - np.array(other[0])))))
            if station in (lo_scope, hi_scope):
                old = states(old_road, prepared.inner_lane_id, station, left)
                endpoint_error = max(endpoint_error, float(np.max(np.abs(np.array(a) - np.array(old)))))
    if endpoint_error > 1e-9 or shared_error > 1e-9:
        raise BoundaryEditRejected("实际 XML 端部 C2 或两侧共享边界不一致")
    before_residual = source_residuals(prepared.data, prepared)
    after_residual = source_residuals(data, prepared)
    evidence = {**prepared.capability(), "candidate_sha256": _hash(data),
                "normal_delta_m": normal_delta_m, "amplitude_anchor_s_m": anchor,
                "changed_width_records": touched, "min_changed_width_m": minimum if touched else None,
                "complexity_before": cb, "complexity_after": ca,
                "pair_width_sum_coefficient_error": sum_error, "outside_coefficient_error": outside_error,
                "delta_C2_jet_max_error": delta_c2_error,
                "endpoint_world_jet_max_error": endpoint_error, "shared_world_jet_max_error": shared_error,
                "source_before": before_residual, "source_after": after_residual,
                "readback": "PASS", "absolute_crs": "UNVERIFIED",
                "whole_map_validation": "NOT_RUN", "delivery": "BLOCKED"}
    return data, evidence
