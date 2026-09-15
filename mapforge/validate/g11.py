# -*- coding: utf-8 -*-
"""G11：OpenDRIVE 少段、横断面、连续性、动力学与兼容性门禁。

G11 不替代 G1～G10。它专门阻止两类旧门禁会放过的实现：

* 用大量米级 planView geometry 获得“数学连续”的视觉平滑；
* 减少 planView 后把复杂度转移到 laneOffset/width/laneSection。

结果使用 ``status=PASS|FAIL`` 供机器门禁，另用
``level=PASS|WARNING|FAIL`` 保留专家要求的风险信号。
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections import Counter
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from mapforge.validate.dynamics_speed import lane_limits, limit_samples, movement_bindings

from mapforge.validate.smoothness import (
    _edge_world_curvature,
    _ref_kappa_at,
    _sections,
    lane_edges_kinematics_at,
    junction_lane_interfaces,
)


def load_policy(path: str | Path) -> dict:
    path = Path(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != "mapforge/g11-policy/v1":
        raise ValueError(f"not a G11 v1 policy: {path}")
    required = (
        "consumer_profile", "reference_line", "cross_section",
        "continuity", "dynamics", "provenance",
    )
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"G11 policy missing: {', '.join(missing)}")
    data = dict(data)
    data["_path"] = str(path)
    data["_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return data


def _angle_delta(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _primitive(g: ET.Element) -> dict:
    out = {
        "s": float(g.get("s", "0")),
        "x": float(g.get("x", "0")),
        "y": float(g.get("y", "0")),
        "hdg": float(g.get("hdg", "0")),
        "length": float(g.get("length", "0")),
        "kind": "unknown", "k0": float("nan"), "k1": float("nan"),
        "parameters": {},
    }
    if g.find("line") is not None:
        out.update(kind="line", k0=0.0, k1=0.0)
    elif g.find("arc") is not None:
        child = g.find("arc")
        k = float(child.get("curvature", "0"))
        out.update(kind="arc", k0=k, k1=k, parameters=dict(child.attrib))
    elif g.find("spiral") is not None:
        child = g.find("spiral")
        out.update(
            kind="spiral",
            k0=float(child.get("curvStart", "0")),
            k1=float(child.get("curvEnd", "0")),
            parameters=dict(child.attrib),
        )
    elif g.find("poly3") is not None:
        child = g.find("poly3")
        out.update(kind="poly3", parameters=dict(child.attrib))
    elif g.find("paramPoly3") is not None:
        child = g.find("paramPoly3")
        out.update(kind="paramPoly3", parameters=dict(child.attrib))
    return out


def _primitives(road: ET.Element) -> list[dict]:
    return [_primitive(g) for g in road.findall("planView/geometry")]


def _road_role(road: ET.Element) -> str:
    if road.get("name") == "junction_paving":
        return "paving"
    if road.get("junction") not in (None, "-1"):
        return "connector"
    return "ordinary"


def _provenance(road: ET.Element) -> dict:
    values, invalid = [], 0
    for ud in road.findall(".//lane/userData"):
        if ud.get("code") != "mapforge.provenance/v1":
            continue
        try:
            value = json.loads(ud.get("value", "{}"))
        except (TypeError, ValueError):
            invalid += 1
            continue
        if isinstance(value, dict):
            values.append(value)
    statuses = Counter(str(v.get("status", "UNKNOWN")) for v in values)
    supports = Counter(str(v.get("support_kind", "UNKNOWN")) for v in values)
    roles = Counter(str(v.get("role", "UNKNOWN")) for v in values)
    return {
        "values": values,
        "statuses": dict(sorted(statuses.items())),
        "support_kinds": dict(sorted(supports.items())),
        "roles": dict(sorted(roles.items())),
        "invalid": invalid,
    }


def _connector_source_class(prov: dict, policy: dict) -> str:
    statuses = set(prov["statuses"])
    inferred = set(policy["provenance"].get("inferred_statuses", ["INFERRED"]))
    measured = set(policy["provenance"].get("measured_statuses", []))
    if statuses and statuses <= inferred:
        return "inferred"
    if statuses & measured:
        return "measured"
    supports = " ".join(prov["support_kinds"]).lower()
    if "synthetic" in supports or "mirror" in supports:
        return "inferred"
    return "unknown"


def _issue(issues: list[dict], severity: str, code: str,
           road: ET.Element | None = None, **data) -> None:
    item = {"severity": severity, "code": code}
    if road is not None:
        item["road_id"] = road.get("id")
        item["road_name"] = road.get("name", "")
    item.update(data)
    issues.append(item)


def _level(issues: list[dict]) -> str:
    if any(x["severity"] == "FAIL" for x in issues):
        return "FAIL"
    if any(x["severity"] == "WARNING" for x in issues):
        return "WARNING"
    return "PASS"


def _group(issues: list[dict], **data) -> dict:
    level = _level(issues)
    return {"status": "FAIL" if level == "FAIL" else "PASS",
            "level": level, "issues": issues, **data}


def _max_boundaries_in_window(boundaries: list[float], width: float) -> int:
    if not boundaries:
        return 0
    best = left = 0
    for right, value in enumerate(boundaries):
        while value - boundaries[left] > width + 1e-9:
            left += 1
        best = max(best, right - left + 1)
    return best


def _mergeable(a: dict, b: dict, cfg: dict) -> bool:
    if a["kind"] != b["kind"]:
        return False
    ktol = float(cfg["merge_curvature_tolerance"])
    stol = float(cfg["merge_sharpness_tolerance"])
    if a["kind"] == "line":
        return True
    if a["kind"] == "arc":
        return abs(a["k0"] - b["k0"]) <= ktol
    if a["kind"] == "spiral":
        sa = (a["k1"] - a["k0"]) / max(a["length"], 1e-12)
        sb = (b["k1"] - b["k0"]) / max(b["length"], 1e-12)
        return abs(a["k1"] - b["k0"]) <= ktol and abs(sa - sb) <= stol
    return False


def _audit_a(root: ET.Element, policy: dict) -> dict:
    cfg = policy["reference_line"]
    issues, rows = [], []
    aggregate = Counter()
    for road in root.findall("road"):
        ps = _primitives(road)
        role = _road_role(road)
        prov = _provenance(road)
        source_class = _connector_source_class(prov, policy)
        lengths = [p["length"] for p in ps]
        total = sum(lengths)
        boundaries = []
        cursor = 0.0
        for length in lengths[:-1]:
            cursor += length
            boundaries.append(cursor)
        short_flags = [x < float(cfg["short_warning_m"]) for x in lengths]
        run = best_run = 0
        for flag in short_flags:
            run = run + 1 if flag else 0
            best_run = max(best_run, run)
        rels = [x / total if total > 1e-12 else 0.0 for x in lengths]
        mergeable = [i for i, (a, b) in enumerate(zip(ps, ps[1:]))
                     if _mergeable(a, b, cfg)]
        windows = {
            str(float(w)): _max_boundaries_in_window(boundaries, float(w))
            for w in cfg.get("sliding_windows_m", [])
        }
        density = len(ps) / max(total, 1e-12) * 100.0

        if role == "ordinary" and total >= float(cfg["density_min_road_length_m"]):
            if density > float(cfg["ordinary_density_fail_per_100m"]):
                sev = "FAIL" if cfg.get("density_hard_fail_enabled") else "WARNING"
                _issue(issues, sev, "ordinary_geometry_density_high", road,
                       density_per_100m=density)
            elif density > float(cfg["ordinary_density_warning_per_100m"]):
                _issue(issues, "WARNING", "ordinary_geometry_density_warning", road,
                       density_per_100m=density)
        if role == "connector":
            if source_class == "inferred" and len(ps) > int(
                    cfg["inferred_connector_max_primitives"]):
                _issue(issues, "FAIL", "inferred_connector_too_many_primitives", road,
                       count=len(ps), limit=cfg["inferred_connector_max_primitives"])
            elif source_class != "inferred":
                if len(ps) > int(cfg["measured_connector_max_primitives"]):
                    _issue(issues, "FAIL", "measured_connector_too_many_primitives", road,
                           count=len(ps), limit=cfg["measured_connector_max_primitives"])
                elif len(ps) > int(cfg["measured_connector_warning_primitives"]):
                    _issue(issues, "WARNING", "measured_connector_many_primitives", road,
                           count=len(ps))
        for i, (length, ratio) in enumerate(zip(lengths, rels)):
            if length < float(cfg["absolute_degenerate_m"]) or ratio < float(
                    cfg["relative_degenerate_ratio"]):
                _issue(issues, "FAIL", "degenerate_primitive", road,
                       index=i, length_m=length, ratio=ratio)
            elif ratio < float(cfg["relative_review_ratio"]):
                _issue(issues, "WARNING", "short_relative_primitive", road,
                       index=i, length_m=length, ratio=ratio)
            elif length < float(cfg["short_warning_m"]):
                _issue(issues, "WARNING", "short_primitive", road,
                       index=i, length_m=length, ratio=ratio)
        if mergeable:
            _issue(issues, "FAIL", "canonicalization_mergeable_primitives", road,
                   left_indices=mergeable)
        max_window = max(windows.values(), default=0)
        if max_window > int(cfg["sliding_boundary_fail"]):
            sev = "FAIL" if cfg.get("sliding_hard_fail_enabled") else "WARNING"
            _issue(issues, sev, "sliding_window_fragment_density_high", road,
                   windows=windows)
        elif max_window > int(cfg["sliding_boundary_warning"]):
            _issue(issues, "WARNING", "sliding_window_fragment_density_warning", road,
                   windows=windows)
        rows.append({
            "road_id": road.get("id"), "name": road.get("name", ""),
            "role": role, "source_class": source_class, "length_m": total,
            "primitive_count": len(ps), "primitive_types": [p["kind"] for p in ps],
            "segment_lengths_m": lengths, "density_per_100m": density,
            "short_lt5_count": sum(short_flags), "short_run_max": best_run,
            "minimum_relative_length": min(rels, default=0.0),
            "mergeable_pairs": mergeable, "window_boundary_max": windows,
        })
        aggregate["roads"] += 1
        aggregate["primitives"] += len(ps)
        aggregate["short_lt5"] += sum(short_flags)
        aggregate[f"role_{role}"] += 1
    return _group(issues, metrics=dict(aggregate), roads=rows)


def _poly_entries(elements: list[ET.Element], start_attr: str) -> list[tuple]:
    return sorted((float(x.get(start_attr, "0")), float(x.get("a", "0")),
                   float(x.get("b", "0")), float(x.get("c", "0")),
                   float(x.get("d", "0"))) for x in elements)


def _poly_value(e: tuple, ds: float, order: int = 0) -> float:
    _, a, b, c, d = e
    if order == 0:
        return a + b * ds + c * ds ** 2 + d * ds ** 3
    if order == 1:
        return b + 2.0 * c * ds + 3.0 * d * ds ** 2
    if order == 2:
        return 2.0 * c + 6.0 * d * ds
    raise ValueError(order)


def _poly_join_jumps(entries: list[tuple]) -> list[dict]:
    out = []
    for a, b in zip(entries, entries[1:]):
        ds = b[0] - a[0]
        out.append({
            "s": b[0],
            "value": abs(_poly_value(a, ds, 0) - _poly_value(b, 0.0, 0)),
            "d1": abs(_poly_value(a, ds, 1) - _poly_value(b, 0.0, 1)),
            "d2": abs(_poly_value(a, ds, 2) - _poly_value(b, 0.0, 2)),
        })
    return out


def _active_poly(entries: list[tuple], offset: float) -> tuple | None:
    active = None
    for entry in entries:
        if entry[0] <= offset + 1e-9:
            active = entry
    return active or (entries[0] if entries else None)


def _section_lane_map(section: ET.Element) -> dict[tuple[str, int], ET.Element]:
    out = {}
    for side in ("left", "right"):
        for lane in section.findall(f"{side}/lane"):
            out[(side, int(lane.get("id", "0")))] = lane
    return out


def _linked_section_pairs(prev: ET.Element, nxt: ET.Element):
    """按 OpenDRIVE lane link 配对相邻 laneSection 的同一物理车道。

    车道生灭时 lane id 可以重排；仅按相同 id 比较会把两条不同车道误报成
    3.5m 宽度跳变。优先消费 predecessor/successor，缺链接时才同 id 回退。
    """
    pm, nm = _section_lane_map(prev), _section_lane_map(nxt)
    used = set()
    for pkey, plane in sorted(pm.items()):
        side, pid = pkey
        succ = plane.find("link/successor")
        candidates = []
        if succ is not None and succ.get("id") is not None:
            candidates.append((side, int(succ.get("id"))))
        for nkey, nlane in nm.items():
            pred = nlane.find("link/predecessor")
            if (nkey[0] == side and pred is not None
                    and pred.get("id") is not None
                    and int(pred.get("id")) == pid):
                candidates.append(nkey)
        candidates.append(pkey)
        nkey = next((x for x in candidates if x in nm and x not in used), None)
        if nkey is not None:
            used.add(nkey)
            yield pkey, nkey, plane, nm[nkey]


def _audit_b(root: ET.Element, policy: dict,
             baseline_entry: dict | None = None) -> dict:
    cfg = policy["cross_section"]
    issues, rows = [], []
    totals = Counter()
    for road in root.findall("road"):
        length = float(road.get("length", "0"))
        role = _road_role(road)
        offsets = _poly_entries(road.findall("lanes/laneOffset"), "s")
        offset_jumps = _poly_join_jumps(offsets)
        sections = road.findall("lanes/laneSection")
        width_jumps, width_records, max_lane_density = [], 0, 0.0
        for si, section in enumerate(sections):
            s0 = float(section.get("s", "0"))
            s1 = (float(sections[si + 1].get("s", "0"))
                  if si + 1 < len(sections) else length)
            span = max(s1 - s0, 1e-12)
            for lane in section.findall("left/lane") + section.findall("right/lane"):
                entries = _poly_entries(
                    lane.findall("border") or lane.findall("width"), "sOffset")
                width_records += len(entries)
                max_lane_density = max(max_lane_density, len(entries) / span * 100.0)
                width_jumps.extend(_poly_join_jumps(entries))
        # 同 lane id 的 section 边界值/导数连续性。
        section_jumps = []
        for prev, nxt in zip(sections, sections[1:]):
            ps, ns = float(prev.get("s", "0")), float(nxt.get("s", "0"))
            for pkey, nkey, plane, nlane in _linked_section_pairs(prev, nxt):
                a = _poly_entries(plane.findall("border") or plane.findall("width"),
                                  "sOffset")
                b = _poly_entries(nlane.findall("border") or nlane.findall("width"),
                                  "sOffset")
                ea, eb = _active_poly(a, ns - ps - 1e-9), _active_poly(b, 0.0)
                if ea is None or eb is None:
                    continue
                ds = ns - ps - ea[0]
                section_jumps.append({
                    "s": ns, "side": pkey[0], "lane_id": pkey[1],
                    "next_lane_id": nkey[1],
                    "value": abs(_poly_value(ea, ds, 0) - _poly_value(eb, 0.0, 0)),
                    "d1": abs(_poly_value(ea, ds, 1) - _poly_value(eb, 0.0, 1)),
                    "d2": abs(_poly_value(ea, ds, 2) - _poly_value(eb, 0.0, 2)),
                })
        all_jumps = offset_jumps + width_jumps + section_jumps
        max_value = max((x["value"] for x in all_jumps), default=0.0)
        max_d1 = max((x["d1"] for x in all_jumps), default=0.0)
        max_d2 = max((x["d2"] for x in all_jumps), default=0.0)
        # junction paving 是 restricted/unroutable 面片，不是可驾驶车道横断面；
        # 它的多项式只承担面覆盖，不能拿 driving-lane 的 C1/C2 门禁误判。
        enforce_continuity = role != "paving"
        if enforce_continuity and max_value > float(cfg["value_jump_fail_m"]):
            _issue(issues, "FAIL", "cross_section_value_jump", road, maximum=max_value)
        if enforce_continuity and max_d1 > float(cfg["first_derivative_jump_fail"]):
            _issue(issues, "FAIL", "cross_section_first_derivative_jump", road,
                   maximum=max_d1)
        if enforce_continuity and max_d2 > float(cfg["second_derivative_jump_warning"]):
            _issue(issues, "WARNING", "cross_section_second_derivative_jump", road,
                   maximum=max_d2)
        if role == "ordinary" and length >= 50.0:
            section_density = len(sections) / length * 100.0
            offset_density = len(offsets) / length * 100.0
            if section_density > float(cfg["lane_section_density_warning_per_100m"]):
                _issue(issues, "WARNING", "lane_section_density_high", road,
                       density_per_100m=section_density)
            if offset_density > float(cfg["lane_offset_density_warning_per_100m"]):
                _issue(issues, "WARNING", "lane_offset_density_high", road,
                       density_per_100m=offset_density)
        if max_lane_density > float(cfg["width_record_density_warning_per_lane_100m"]):
            _issue(issues, "WARNING", "lane_width_record_density_high", road,
                   maximum_per_lane_100m=max_lane_density)
        row = {
            "road_id": road.get("id"), "role": role, "length_m": length,
            "planview_count": len(road.findall("planView/geometry")),
            "lane_offset_count": len(offsets), "lane_section_count": len(sections),
            "width_border_record_count": width_records,
            "cross_section_complexity": len(offsets) + len(sections) + width_records,
            "join_value_max_m": max_value, "join_d1_max": max_d1,
            "join_d2_max": max_d2,
        }
        rows.append(row)
        totals.update({
            "planview": row["planview_count"], "lane_offsets": len(offsets),
            "lane_sections": len(sections), "width_border_records": width_records,
            "cross_section_complexity": row["cross_section_complexity"],
        })
    baseline_delta = None
    if baseline_entry is not None:
        base_roads = baseline_entry.get("roads", [])
        base = Counter()
        for road in base_roads:
            base.update({
                "planview": road.get("planview_count", 0),
                "lane_offsets": road.get("lane_offset_count", 0),
                "lane_sections": road.get("lane_section_count", 0),
                "width_border_records": (road.get("width_record_count", 0)
                                         + road.get("border_record_count", 0)),
                "cross_section_complexity": (
                    road.get("lane_offset_count", 0)
                    + road.get("lane_section_count", 0)
                    + road.get("width_record_count", 0)
                    + road.get("border_record_count", 0)),
            })
        baseline_delta = {key: totals[key] - base[key] for key in totals}
        if (cfg.get("complexity_transfer_warning")
                and baseline_delta.get("planview", 0) < 0
                and baseline_delta.get("cross_section_complexity", 0) > 0):
            _issue(issues, "WARNING", "cross_section_complexity_transfer",
                   planview_delta=baseline_delta["planview"],
                   cross_section_delta=baseline_delta["cross_section_complexity"])
    return _group(issues, metrics=dict(totals), baseline_delta=baseline_delta,
                  roads=rows)


def _advance(p: dict) -> tuple[float, float, float, float] | None:
    x, y, h, length = p["x"], p["y"], p["hdg"], p["length"]
    if p["kind"] == "line":
        return x + length * math.cos(h), y + length * math.sin(h), h, 0.0
    if p["kind"] == "arc":
        k = p["k0"]
        if abs(k) < 1e-15:
            return x + length * math.cos(h), y + length * math.sin(h), h, 0.0
        return (x + (math.sin(h + k * length) - math.sin(h)) / k,
                y - (math.cos(h + k * length) - math.cos(h)) / k,
                h + k * length, k)
    if p["kind"] == "spiral":
        from pyclothoids import Clothoid
        sharp = (p["k1"] - p["k0"]) / max(length, 1e-15)
        c = Clothoid.StandardParams(x, y, h, p["k0"], sharp, length)
        return float(c.XEnd), float(c.YEnd), float(c.ThetaEnd), float(c.KappaEnd)
    return None


def _audit_c(root: ET.Element, policy: dict) -> dict:
    cfg = policy["continuity"]
    issues, rows = [], []
    maxima = Counter()
    for road in root.findall("road"):
        ps = _primitives(road)
        joints = []
        for i, (a, b) in enumerate(zip(ps, ps[1:])):
            end = _advance(a)
            if end is None or not math.isfinite(b["k0"]):
                _issue(issues, "FAIL", "unsupported_internal_continuity", road,
                       index=i, primitive=a["kind"])
                continue
            pos = math.hypot(end[0] - b["x"], end[1] - b["y"])
            hdg = _angle_delta(end[2], b["hdg"])
            curv = abs(end[3] - b["k0"])
            maxima["position_m"] = max(maxima["position_m"], pos)
            maxima["heading_rad"] = max(maxima["heading_rad"], hdg)
            maxima["curvature_per_m"] = max(maxima["curvature_per_m"], curv)
            joints.append({"index": i, "position_m": pos,
                           "heading_rad": hdg, "curvature_per_m": curv})
            if pos > float(cfg["internal_position_fail_m"]):
                _issue(issues, "FAIL", "internal_position_discontinuity", road,
                       index=i, value=pos)
            if hdg > float(cfg["internal_heading_fail_rad"]):
                _issue(issues, "FAIL", "internal_heading_discontinuity", road,
                       index=i, value=hdg)
            if curv > float(cfg["internal_curvature_fail_per_m"]):
                _issue(issues, "FAIL", "internal_curvature_discontinuity", road,
                       index=i, value=curv)
        rows.append({"road_id": road.get("id"), "joints": joints})
    interfaces = junction_lane_interfaces(root)
    for row in interfaces:
        if 'error' in row:
            _issue(issues, 'FAIL', 'unresolved_road_interface', detail=row)
            continue
        for field, threshold, code in (
            ('position_m', float(cfg['road_interface_position_fail_m']), 'road_interface_position_discontinuity'),
            ('heading_deg', float(cfg['road_interface_heading_fail_deg']), 'road_interface_heading_discontinuity'),
            ('curvature_per_m', float(cfg.get('road_interface_curvature_fail_per_m', 1e-7)), 'road_interface_curvature_discontinuity'),
        ):
            maxima['interface_'+field] = max(maxima['interface_'+field], row[field])
            if row[field] > threshold:
                _issue(issues, 'FAIL', code, detail=row, value=row[field])
    return _group(issues, metrics=dict(maxima), roads=rows, interfaces=interfaces)


def _road_target_speed(road: ET.Element, role: str, cfg: dict) -> tuple[float, str]:
    values = []
    for lane in road.findall(".//lane"):
        if lane.get("type") != "driving":
            continue
        values.extend(r['max_kmh'] for r in lane_limits(lane))
    if values:
        return max(values), "lane-speed"
    key = "connector" if role == "connector" else "ordinary"
    return float(cfg["default_target_speed_kmh"][key]), "policy-fallback"


def _section_tracks(road: ET.Element, s: float, side: str) -> list[tuple]:
    edges = lane_edges_kinematics_at(road, s, side)
    tracks = list(edges)
    tracks.extend(tuple((a + b) / 2.0 for a, b in zip(x, y))
                  for x, y in zip(edges, edges[1:]))
    return tracks


def _lane_provenance(lane: ET.Element) -> dict:
    ud = lane.find("userData[@code='mapforge.provenance/v1']")
    if ud is None or not ud.get("value"):
        return {}
    try:
        return json.loads(ud.get("value"))
    except (TypeError, json.JSONDecodeError):
        return {}


def _lane_target_speed(lane: ET.Element, role: str, cfg: dict) -> tuple[float, str]:
    records = lane_limits(lane)
    if records:
        return max(r['max_kmh'] for r in records), "lane-speed"
    key = "connector" if role == "connector" else "ordinary"
    return float(cfg["default_target_speed_kmh"][key]), "policy-fallback"


def _driving_curvature_joins(road: ET.Element) -> list[dict]:
    """One-sided world curvature at every written polynomial/section join.

    Uniform distance sampling can hide small curvature jumps. In particular,
    reference-line G2 plus laneOffset/width C2 does not imply lane-center G2
    where reference curvature rate changes and the offset has nonzero slope.
    """
    sections=road.findall('lanes/laneSection')
    starts=[float(s.get('s','0')) for s in sections]
    ends=starts[1:]+[float(road.get('length','0'))]
    global_cuts=[float(g.get('s','0')) for g in road.findall('planView/geometry')]
    global_cuts += [float(o.get('s','0')) for o in road.findall('lanes/laneOffset')]
    rows=[]
    def lanes(si,side):
        return sorted(sections[si].findall(f'{side}/lane'),key=lambda x:abs(int(x.get('id'))))
    @lru_cache(maxsize=None)
    def state(s,side):
        return lane_edges_kinematics_at(road,s,side),_ref_kappa_at(road,s)
    def value(s,side,index):
        edges,reference=state(s,side)
        jet=tuple((a+b)/2 for a,b in zip(edges[index],edges[index+1]))
        return _edge_world_curvature(*jet,*reference)
    for si,(s0,s1) in enumerate(zip(starts,ends)):
        cuts=set(c for c in global_cuts if s0<c<s1)
        for lane in sections[si].findall('.//lane'):
            cuts.update(s0+float(w.get('sOffset','0')) for w in list(lane.findall('width'))+list(lane.findall('border'))
                        if s0<s0+float(w.get('sOffset','0'))<s1)
        ordered=[s0]+sorted(cuts)+[s1]
        for j,s in enumerate(ordered[1:-1],1):
            eps=min(1e-6,(s-ordered[j-1])/8,(ordered[j+1]-s)/8)
            if eps<2e-9: continue
            for side in ('left','right'):
                for i,lane in enumerate(lanes(si,side)):
                    if lane.get('type')!='driving': continue
                    left,right=value(s-eps,side,i),value(s+eps,side,i)
                    rows.append({'s_m':s,'lane_id':lane.get('id'),'next_lane_id':lane.get('id'),
                        'side':side,'join_kind':'written-polynomial','curvature_jump_per_m':abs(right-left)})
        if si+1>=len(sections): continue
        s=s1; eps=min(1e-6,(s1-s0)/8,(ends[si+1]-s1)/8)
        if eps<2e-9: continue
        for side in ('left','right'):
            after=lanes(si+1,side); after_ids=[x.get('id') for x in after]
            for i,lane in enumerate(lanes(si,side)):
                successor=lane.find('link/successor')
                if lane.get('type')!='driving' or successor is None: continue
                next_id=successor.get('id')
                if next_id not in after_ids: continue  # topology gate handles missing references
                ni=after_ids.index(next_id)
                if after[ni].get('type')!='driving': continue
                left,right=value(s-eps,side,i),value(s+eps,side,ni)
                rows.append({'s_m':s,'lane_id':lane.get('id'),'next_lane_id':next_id,
                    'side':side,'join_kind':'lane-section-link','curvature_jump_per_m':abs(right-left)})
    return rows


def _audit_d(root: ET.Element, policy: dict) -> dict:
    cfg = policy["dynamics"]
    issues, rows = [], []
    contract = cfg.get('movement_design')
    bindings = movement_bindings(root, contract)
    speed_evidence = []
    if contract is not None and contract['approval_state'] != 'approved':
        _issue(issues, 'FAIL', 'movement_design_speed_requires_approval')
    lane_diagnostics = []
    max_ay = max_jy = max_kappa = max_sharp = max_lane_join_jump = 0.0
    ds = float(cfg["sample_step_m"])
    for road in root.findall("road"):
        role = _road_role(road)
        if role == "paving":
            continue
        length = float(road.get("length", "0"))
        if length <= 1e-6:
            continue
        road_kappa = road_sharp = road_ay = road_jy = 0.0
        road_speed = 0.0
        supported = float("inf")
        evaluated_tracks = 0
        fallback_used = False
        speed_sources = set()
        excluded_tracks = 0
        excluded_codes = set(cfg.get("excluded_provenance_codes", [
            "lane-transition-taper", "physical-edge-fill", "median-non-driving",
        ]))
        # Provenance determines fidelity comparison, never whether a written
        # driving lane has dynamics. "lane-transition-taper" also occurs on
        # full-width through lanes after source-domain repairs; excluding it
        # hid real curvature/jerk spikes. Even a custom/old policy cannot grant
        # a metadata-only exemption to driving geometry.
        ignored_driving_exclusions = 0
        sections = _sections(road)
        xml_sections = road.findall("lanes/laneSection")
        for si, sec in enumerate(sections):
            s0 = float(sec[0])
            s1 = float(sections[si + 1][0]) if si + 1 < len(sections) else length
            if s1 - s0 <= 1e-6:
                continue
            margin = min(1e-4, (s1 - s0) * 0.05)
            grid = np.arange(s0 + margin, s1 - margin + 1e-12, ds)
            if len(grid) < 2:
                grid = np.linspace(s0 + margin, s1 - margin, 2)
            speed_cuts = []
            for lane in xml_sections[si].findall('left/lane') + xml_sections[si].findall('right/lane'):
                for limit in lane_limits(lane):
                    local = limit['s_offset_m']
                    if local >= s1-s0:
                        raise ValueError('lane speed record outside its laneSection')
                    if local > 0:
                        speed_cuts.extend([s0+local-margin, s0+local])
            grid = np.unique(np.r_[grid, s1-margin,
                                  np.clip(speed_cuts, s0+margin, s1-margin)])
            for side in ("left", "right"):
                lane_elements = sorted(
                    xml_sections[si].findall(f"{side}/lane"),
                    key=lambda x: abs(int(x.get("id", "0"))))
                track_values: list[list[tuple[float, float, float]]] = [
                    [] for _ in lane_elements]
                for s in grid:
                    kref, sharpref = _ref_kappa_at(road, float(s))
                    edges = lane_edges_kinematics_at(road, float(s), side)
                    if len(edges) != len(lane_elements) + 1:
                        continue
                    for i, (a, b) in enumerate(zip(edges, edges[1:])):
                        t, d1, d2 = tuple((x + y) / 2.0 for x, y in zip(a, b))
                        kval = _edge_world_curvature(t, d1, d2, kref, sharpref)
                        speed_factor = math.hypot(1.0 - kref * t, d1)
                        track_values[i].append((float(kval), max(speed_factor, 1e-9), float(s)))
                for lane, values in zip(lane_elements, track_values):
                    if not values:
                        continue
                    prov = _lane_provenance(lane)
                    code = prov.get("exclusion_code")
                    if lane.get("type") != "driving":
                        excluded_tracks += 1
                        continue
                    if code in excluded_codes:
                        ignored_driving_exclusions += 1
                    ks = np.asarray([x[0] for x in values], dtype=float)
                    factors = np.asarray([x[1] for x in values], dtype=float)
                    stations = np.asarray([x[2] for x in values], dtype=float)
                    finite = np.isfinite(ks)
                    if not np.all(finite):
                        _issue(issues, "FAIL", "non_finite_lane_curvature", road)
                        continue
                    evaluated_tracks += 1
                    road_kappa = max(road_kappa, float(np.max(np.abs(ks))))
                    lane_sharp = 0.0
                    if len(ks) > 1:
                        # 短 section 使用两点 linspace，间距并非 policy.ds。
                        dl = np.diff(stations) * (factors[:-1] + factors[1:]) / 2.0
                        sharp = np.abs(np.diff(ks)) / np.maximum(dl, 1e-9)
                        lane_sharp = float(np.max(sharp))
                        road_sharp = max(road_sharp, lane_sharp)
                    lane_kappa = float(np.max(np.abs(ks)))
                    limits = lane_limits(lane)
                    fallback = cfg['default_target_speed_kmh'][
                        'connector' if role == 'connector' else 'ordinary']
                    if not math.isfinite(float(fallback)) or float(fallback) <= 0:
                        raise ValueError('positive finite fallback design speed required')
                    written, missing = limit_samples(limits, stations-s0, fallback)
                    binding = bindings.get((road.get('id'), s0, lane.get('id')))
                    targets = (np.full(len(ks), binding['target_speed_kmh'])
                               if binding is not None else written)
                    speed_source = ('movement-design-profile' if binding is not None else
                                    'policy-fallback' if np.all(missing) else
                                    'mixed-limit-and-fallback' if np.any(missing) else 'lane-speed')
                    speed_sources.add(speed_source)
                    speed_kmh = float(max(targets))
                    v = targets / 3.6
                    ay = float(np.max(v*v*np.abs(ks)))
                    jy = (float(np.max(np.maximum(v[:-1], v[1:])**3*sharp))
                          if len(ks) > 1 else 0.)
                    # Keep the written-limit stress result alongside design
                    # conditions. Never call changing the test speed a repair.
                    stress_v = written / 3.6
                    speed_evidence.append({
                        'road_id': road.get('id'), 'lane_id': lane.get('id'),
                        'section_s_m': s0, 'section_end_s_m': s1,
                        'written_limits': limits, 'target_basis': speed_source,
                        'movement_design': binding, 'target_speed_kmh': speed_kmh,
                        'written_limit_missing_in_sample': bool(np.any(missing)),
                        'written_limit_or_fallback_stress_ay_mps2': float(np.max(stress_v**2*np.abs(ks))),
                        'written_limit_or_fallback_stress_jy_mps3': (
                            float(np.max(np.maximum(stress_v[:-1],stress_v[1:])**3*sharp))
                            if len(ks)>1 else 0.),
                        'design_ay_mps2': ay, 'design_jy_mps3': jy,
                    })
                    road_ay, road_jy = max(road_ay, ay), max(road_jy, jy)
                    road_speed = max(road_speed, speed_kmh)
                    fallback_used = fallback_used or (binding is None and bool(np.any(missing)))
                    v_ay = (math.sqrt(float(cfg["fail"]["lateral_acceleration_mps2"])
                                      / lane_kappa)
                            if lane_kappa > 1e-15 else float("inf"))
                    v_jerk = ((float(cfg["fail"]["lateral_jerk_mps3"])
                               / lane_sharp) ** (1.0 / 3.0)
                              if lane_sharp > 1e-15 else float("inf"))
                    supported = min(supported, v_ay, v_jerk)
                    if ay > float(cfg["fail"]["lateral_acceleration_mps2"]) or jy > float(cfg["fail"]["lateral_jerk_mps3"]):
                        src = lane.find("userData[@code='mapforge.source_lane']")
                        peak = int(np.argmax(sharp)) if len(ks) > 1 else 0
                        lane_diagnostics.append({
                            "road_id":road.get("id"), "lane_id":lane.get("id"),
                            "source_lane_id":src.get("value") if src is not None else None,
                            "section_s_m":s0, "section_end_s_m":s1,
                            "curvature_peak_s_m":float(stations[np.argmax(np.abs(ks))]),
                            "sharpness_peak_s_interval_m":[float(stations[peak]),
                                float(stations[min(peak+1,len(stations)-1)])],
                            "target_speed_kmh":speed_kmh,
                            "lateral_acceleration_mps2":ay, "lateral_jerk_mps3":jy,
                            "exclusion_code":code,
                        })
        joins=_driving_curvature_joins(road)
        join_max=max((x['curvature_jump_per_m'] for x in joins),default=0.)
        max_lane_join_jump=max(max_lane_join_jump,join_max)
        jump_limit=float(cfg.get('lane_curvature_jump_fail_per_m',1e-7))
        bad_joins=[x for x in joins if not math.isfinite(x['curvature_jump_per_m'])
                   or x['curvature_jump_per_m']>jump_limit]
        if bad_joins:
            _issue(issues,'FAIL','driving_lane_curvature_jump',road,value=join_max,
                   count=len(bad_joins),joints=bad_joins)
        ay, jy = road_ay, road_jy
        max_ay, max_jy = max(max_ay, road_ay), max(max_jy, road_jy)
        max_kappa, max_sharp = max(max_kappa, road_kappa), max(max_sharp, road_sharp)
        fail = cfg["fail"]
        warn = cfg["warning"]
        if ay > float(fail["lateral_acceleration_mps2"]):
            _issue(issues, "FAIL", "lateral_acceleration_exceeded", road,
                   value=ay, target_speed_kmh=road_speed)
        elif ay > float(warn["lateral_acceleration_mps2"]):
            _issue(issues, "WARNING", "lateral_acceleration_warning", road,
                   value=ay, target_speed_kmh=road_speed)
        if jy > float(fail["lateral_jerk_mps3"]):
            _issue(issues, "FAIL", "lateral_jerk_exceeded", road,
                   value=jy, target_speed_kmh=road_speed)
        elif jy > float(warn["lateral_jerk_mps3"]):
            _issue(issues, "WARNING", "lateral_jerk_warning", road,
                   value=jy, target_speed_kmh=road_speed)
        if fallback_used and cfg.get("fallback_speed_is_warning"):
            _issue(issues, "WARNING", "movement_speed_fallback", road,
                   target_speed_kmh=road_speed)
        if excluded_tracks:
            _issue(issues, "WARNING", "non_driving_tracks_not_in_dynamics",
                   road, count=excluded_tracks)
        if ignored_driving_exclusions:
            _issue(issues, "WARNING", "driving_provenance_exclusion_ignored",
                   road, count=ignored_driving_exclusions)
        rows.append({
            "road_id": road.get("id"), "role": role,
            "target_speed_kmh": road_speed,
            "speed_source": next(iter(speed_sources)) if len(speed_sources)==1 else 'mixed',
            "kappa_max_per_m": road_kappa, "sharpness_max_per_m2": road_sharp,
            "lateral_acceleration_mps2": ay, "lateral_jerk_mps3": jy,
            # No curvature-derived bound is not a license to drive at infinite
            # speed. Keep the report valid JSON and distinguish untested tracks.
            "supported_speed_kmh": supported * 3.6 if math.isfinite(supported) else None,
            "supported_speed_state": ("bounded" if math.isfinite(supported) else
                                      "no-curvature-bound" if evaluated_tracks else
                                      "no-evaluated-driving-track"),
            "evaluated_driving_tracks": evaluated_tracks,
            "excluded_transition_tracks": excluded_tracks,
            "skipped_non_driving_tracks": excluded_tracks,
            "ignored_driving_provenance_exclusions": ignored_driving_exclusions,
            "checked_driving_curvature_joins": len(joins),
            "lane_curvature_join_jump_max_per_m": join_max,
            "failed_driving_curvature_joins": bad_joins,
        })
    return _group(issues, metrics={
        "lateral_acceleration_max_mps2": max_ay,
        "lateral_jerk_max_mps3": max_jy,
        "kappa_max_per_m": max_kappa,
        "sharpness_max_per_m2": max_sharp,
        "lane_curvature_join_jump_max_per_m": max_lane_join_jump,
    }, roads=rows, failed_lane_intervals=lane_diagnostics,
       speed_evidence=speed_evidence, movement_design_contract=contract,
       dynamics_model='constant-speed-planar-proxy-not-vehicle-simulation')


def _audit_e(root: ET.Element, policy: dict) -> dict:
    cfg = policy["consumer_profile"]
    pcfg = policy["provenance"]
    issues, rows = [], []
    allowed = set(cfg.get("allowed_planview_primitives", []))
    accepted_statuses = set(pcfg.get("accepted_statuses", []))
    for road in root.findall("road"):
        role = _road_role(road)
        ps = _primitives(road)
        kinds = [p["kind"] for p in ps]
        prov = _provenance(road)
        if role == "paving" and not cfg.get("allow_auxiliary_paving"):
            _issue(issues, "FAIL", "auxiliary_paving_not_allowed", road)
        if role == "paving":
            # ASAM 1.8.1 Annex D includes restricted in its drivable-lane
            # smoothness scope. Our sim-only routing exclusion is not a
            # universal non-drivability guarantee for other consumers.
            if road.findall(".//lane[@type='restricted']"):
                _issue(issues, "WARNING", "sim_paving_requires_consumer_routing_exclusion", road)
            rid = road.get("id")
            referenced = any(c.get("incomingRoad") == rid or c.get("connectingRoad") == rid
                             for c in root.findall("junction/connection"))
            if (road.findall(".//lane[@type='driving']") or road.find("link") is not None
                    or road.findall(".//lane/link") or referenced):
                _issue(issues, "FAIL", "auxiliary_paving_has_driving_topology", road)
        for kind in kinds:
            if kind == "paramPoly3" and cfg.get("allow_param_poly3"):
                continue
            if kind not in allowed:
                _issue(issues, "FAIL", "primitive_not_allowed", road, primitive=kind)
        param_blocks = 0
        in_block = False
        for kind in kinds:
            if kind == "paramPoly3" and not in_block:
                param_blocks += 1
                in_block = True
            elif kind != "paramPoly3":
                in_block = False
        if param_blocks > 1:
            _issue(issues, "FAIL", "multiple_parampoly3_fallback_blocks", road,
                   count=param_blocks)
        if prov["invalid"]:
            _issue(issues, "FAIL", "invalid_provenance_json", road,
                   count=prov["invalid"])
        unknown = sorted(set(prov["statuses"]) - accepted_statuses)
        if unknown:
            _issue(issues, "FAIL", "unknown_provenance_status", road,
                   statuses=unknown)
        driving = road.findall(".//lane[@type='driving']")
        if cfg.get("require_provenance") and driving and not prov["values"]:
            _issue(issues, "FAIL", "missing_road_provenance", road)
        rows.append({
            "road_id": road.get("id"), "role": role,
            "primitive_types": kinds, "parampoly3_blocks": param_blocks,
            "provenance_statuses": prov["statuses"],
            "support_kinds": prov["support_kinds"],
        })
    return _group(issues, metrics={
        "roads": len(rows),
        "paving_roads": sum(x["role"] == "paving" for x in rows),
        "parampoly3_roads": sum("paramPoly3" in x["primitive_types"] for x in rows),
    }, roads=rows)


def _find_baseline_entry(manifest: dict | None, artifact: str | Path) -> dict | None:
    if not manifest:
        return None
    target = str(artifact).replace("/", "\\").lower()
    target_name = Path(artifact).name.lower()
    for entry in manifest.get("entries", []):
        value = str(entry.get("artifact", "")).replace("/", "\\").lower()
        if value == target or Path(value).name.lower() == target_name:
            return entry
    return None


def audit_file(path: str | Path, policy: dict | str | Path,
               baseline_manifest: dict | str | Path | None = None) -> dict:
    path = Path(path)
    if not isinstance(policy, dict):
        policy = load_policy(policy)
    if baseline_manifest is not None and not isinstance(baseline_manifest, dict):
        baseline_manifest = json.loads(
            Path(baseline_manifest).read_text(encoding="utf-8"))
    root = ET.parse(path).getroot()
    baseline_entry = _find_baseline_entry(baseline_manifest, path)
    groups = {
        "G11-A": _audit_a(root, policy),
        "G11-B": _audit_b(root, policy, baseline_entry),
        "G11-C": _audit_c(root, policy),
        "G11-D": _audit_d(root, policy),
        "G11-E": _audit_e(root, policy),
    }
    failures = sum(1 for group in groups.values()
                   for issue in group["issues"] if issue["severity"] == "FAIL")
    warnings = sum(1 for group in groups.values()
                   for issue in group["issues"] if issue["severity"] == "WARNING")
    level = "FAIL" if failures else ("WARNING" if warnings else "PASS")
    return {
        "schema": "mapforge/g11-result/v1",
        "gate_id": "G11",
        "name": "minimal-structure-cross-section-dynamics-compatibility",
        "status": "FAIL" if failures else "PASS",
        "level": level,
        "policy": {
            "id": policy.get("id"), "version": policy.get("version"),
            "lifecycle": policy.get("lifecycle"),
            "sha256": policy.get("_sha256"),
            "consumer_profile": policy["consumer_profile"].get("id"),
        },
        "artifact": str(path),
        "baseline": {
            "available": baseline_entry is not None,
            "baseline_id": (baseline_manifest or {}).get("baseline_id")
            if isinstance(baseline_manifest, dict) else None,
        },
        "groups": groups,
        "summary": {"failures": failures, "warnings": warnings},
    }


def write_result(path: str | Path, result: dict) -> None:
    Path(path).write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
