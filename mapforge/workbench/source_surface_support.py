"""Fixed WB11 source-supported visual paving, never a driving-path proof.

Only the unsupported road12 departure sweep is withdrawn. Every raw lane
polygon and the existing median remain. The median's missing side contact is
rebuilt from its two real adjacent boundary chains, with the existing 0.25 m
end-cap overlap inside INTERSECTION. No other road supplies source support.
"""
from __future__ import annotations

import hashlib
import json
import math

import numpy as np
from shapely import wkb
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from mapforge.ops import envelope_surface as E, junction_surface as JS

SCHEMA = "mapforge/wb11-source-surface-support/v1"
ROAD = "12"
ENTER = "2023061413430108867"
LEAVE = "2023061413440586483"
ENTRY_LANE = "2023061413450449227"
DEPARTURE_LANE = "2023061413450451686"
BOUNDARIES = {
    ENTRY_LANE: {"boundary_id": "2023061413432074441", "layer": "IBD_LANE_BOUNDARY",
                 "record_index": 3490, "part_index": 0,
                 "relation_layer": "IBD_LANE_BOUNDARY_REL", "relation_index": 5162},
    DEPARTURE_LANE: {"boundary_id": "2023061413441438013", "layer": "IBD_LANE_BOUNDARY",
                     "record_index": 3492, "part_index": 0,
                     "relation_layer": "IBD_LANE_BOUNDARY_REL", "relation_index": 5164},
}


class SupportRejected(ValueError):
    pass


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _record(geometry):
    return {"wkb_hex": geometry.wkb_hex, "sha256": _sha(geometry.wkb),
            "area_m2": float(geometry.area)}


def _holes(geometry):
    return unary_union([Polygon(r) for p in JS._polygons(geometry) for r in p.interiors])


def _read(record):
    geometry = wkb.loads(record["wkb_hex"], hex=True)
    if _sha(geometry.wkb) != record["sha256"]:
        raise SupportRejected("geometry-evidence-binding-mismatch")
    return geometry


def _json_sha(value):
    return _sha(json.dumps(value, sort_keys=True, separators=(",", ":"),
                           allow_nan=False).encode())


def bind_source_boundaries(source, project, pieces):
    """Bind the two fixed inner boundaries to actual SHP relation records.

    The caller's source snapshot binds the dataset files. This function also
    reads their identities and coordinates; constants alone never prove a
    source claim. Other pieces retain their original contents.
    """
    result = [dict(p) for p in pieces]
    for lane, expected in BOUNDARIES.items():
        link = ENTER if lane == ENTRY_LANE else LEAVE
        if len([r for r in source.lanes_of(link) if r.lane_pid == lane]) != 1:
            raise SupportRejected("source-lane-link-identity-mismatch")
        matches = [p for p in result if p["road_id"] == ROAD and p["source_lane"] == lane]
        if len(matches) != 1:
            raise SupportRejected("fixed-lane-source-identity-mismatch")
        relations = [r for r in source.lane_boundary_records(lane)
                     if r["boundary_id"] == expected["boundary_id"]]
        if len(relations) != 1:
            raise SupportRejected("source-boundary-relation-not-unique")
        relation = relations[0]
        if (relation.get("source_lane_id") != lane
                or relation.get("relation_layer") != expected["relation_layer"]
                or relation.get("relation_index") != expected["relation_index"]):
            raise SupportRejected("source-boundary-relation-identity-mismatch")
        records = relation["records"]
        if len(records) != 1:
            raise SupportRejected("source-boundary-feature-not-unique")
        record = records[0]
        if any(record.get(k) != expected[k] for k in ("boundary_id", "layer", "record_index")):
            raise SupportRejected("source-boundary-feature-identity-mismatch")
        if len(record["parts"]) != 1 or expected["part_index"] != 0:
            raise SupportRejected("source-boundary-part-not-unique")
        raw = np.asarray(record["parts"][0], dtype=float)
        coords = np.asarray(project(raw), dtype=float)
        if coords.ndim != 2 or coords.shape[1] != 2 or len(coords) < 2 or not np.isfinite(coords).all():
            raise SupportRejected("invalid-projected-source-boundary")
        binding = {"source_lane": lane, "source_link": link, **expected,
                   "source_coordinates_sha256": _json_sha(raw.tolist()),
                   "projected_boundary": _record(LineString(coords))}
        binding["sha256"] = _json_sha(binding)
        matches[0]["source_inner_boundary"] = binding
    return result


def _bound_inner_chain(piece, polygon_chain, origin, axis, start_s=-.5):
    binding = piece.get("source_inner_boundary")
    lane = piece["source_lane"]
    if not isinstance(binding, dict) or binding.get("sha256") != _json_sha(
            {k: v for k, v in binding.items() if k != "sha256"}):
        raise SupportRejected("source-boundary-binding-missing-or-changed")
    expected = {"source_lane": lane, "source_link": ENTER if lane == ENTRY_LANE else LEAVE,
                **BOUNDARIES[lane]}
    if any(binding.get(k) != v for k, v in expected.items()):
        raise SupportRejected("source-boundary-binding-identity-mismatch")
    line = _read(binding["projected_boundary"])
    if line.geom_type != "LineString" or not line.is_simple:
        raise SupportRejected("simple-source-boundary-required")
    points = np.asarray(line.coords)
    stations = (points-origin) @ axis
    if stations[0] > stations[-1]:
        points, stations = points[::-1], stations[::-1]
    if not np.all(np.diff(stations) > 0) or not stations[0] <= start_s < stations[-1]:
        raise SupportRejected("monotone-source-boundary-spanning-median-required")
    index = int(np.searchsorted(stations, start_s, side="right")-1)
    start = points[index] + (start_s-stations[index])/(stations[index+1]-stations[index])*(points[index+1]-points[index])
    chain = [start, *points[index+1:]]
    # Only binary64 projection/intersection roundoff is admitted here, never
    # the source-support 5 cm tolerance or a user-editable fit allowance.
    scale = max(1., float(np.max(np.abs(points))), float(np.max(np.abs(origin))))
    roundoff = 128*math.ulp(scale)
    discrepancy = LineString(chain).hausdorff_distance(LineString(polygon_chain))
    if discrepancy > roundoff:
        raise SupportRejected("source-boundary-does-not-match-tail-inner-chain")
    return chain, binding, float(discrepancy), roundoff


def _inner_chain(polygon, origin, axis, normal, anchor, start_s=-.5):
    """Trace the actual polygon side from its front cap back to median start.

    The same forward-cap identity used by the old bridge distinguishes the
    front transverse edge from a source side. Non-monotone/ambiguous cases are
    refused, not fitted, sampled, or searched for a favourable angle.
    """
    if polygon.geom_type != "Polygon" or not polygon.is_valid or polygon.interiors:
        raise SupportRejected("simple-source-tail-required")
    coords = np.asarray(orient(polygon, sign=1).exterior.coords)[:-1]
    local_s = (coords-origin) @ axis
    local_t = (coords-origin) @ normal
    caps = []
    for i, a in enumerate(coords):
        edge = coords[(i+1) % len(coords)]-a
        length = np.linalg.norm(edge)
        if length and np.dot([edge[1], -edge[0]], axis)/length >= .5:
            caps.append(i)
    if len(caps) != 1:
        raise SupportRejected("unique-source-front-cap-required")
    i = caps[0]
    a, b = i, (i+1) % len(coords)
    if abs(local_t[a]-anchor) == abs(local_t[b]-anchor):
        raise SupportRejected("ambiguous-inner-source-boundary")
    current, step = (a, -1) if abs(local_t[a]-anchor) < abs(local_t[b]-anchor) else (b, 1)
    chain = [coords[current].copy()]
    indices = [int(current)]
    for _ in range(len(coords)):
        previous = current
        current = (current+step) % len(coords)
        s0, s1 = local_s[previous], local_s[current]
        if s1 >= s0:
            raise SupportRejected("non-monotone-inner-source-boundary")
        if s1 <= start_s:
            if s0 <= start_s:
                raise SupportRejected("median-start-outside-source-tail")
            point = coords[current]+(start_s-s1)/(s0-s1)*(coords[previous]-coords[current])
            chain.append(point)
            indices.append(int(current))
            return list(reversed(chain)), list(reversed(indices))
        chain.append(coords[current].copy())
        indices.append(int(current))
    raise SupportRejected("unclosed-inner-source-boundary")


def _extend_to_base(chain, base, axis):
    end = chain[-1]
    hit = base.intersection(LineString([end, end+1.5*axis]))
    if hit.is_empty:
        raise SupportRejected("source-median-end-outside-existing-bridge-budget")
    gap = Point(end).distance(hit)
    if gap+.25 > 1.5:
        raise SupportRejected("median-extension-exceeds-existing-sweep-budget")
    final = end+(gap+.25)*axis
    # The only overlap after first contact must lie in the actual source base.
    overlap = LineString([end+gap*axis, final])
    roundoff = 128*math.ulp(max(1., *map(abs, base.bounds), *map(abs, final)))
    outside_length = overlap.difference(base).length
    if not base.covers(Point(final)) or outside_length > roundoff:
        raise SupportRejected("median-overlap-not-inside-source-base")
    return [*chain, final], {"source_endpoint": end.tolist(), "forward_gap_m": float(gap),
                            "existing_cap_overlap_m": .25,
                            "extension_m": float(gap+.25), "existing_max_sweep_m": 1.5,
                            "base_overlap_roundoff_bound_m": roundoff,
                            "base_overlap_numerical_outside_length_m": outside_length,
                            "extended_endpoint": final.tolist(), "status": "INFERRED"}


def build_support(base, pieces, mouths):
    """Return tails, medians, JSON evidence for this fixed road/source scope."""
    targets = [m for m in mouths if m.get("road_id") == ROAD]
    if len(targets) != 1 or targets[0].get("enter_link") != ENTER or targets[0].get("leave_links") != [LEAVE]:
        raise SupportRejected("fixed-road-source-identity-mismatch")
    spec = targets[0]
    owned = [p for p in pieces if p["road_id"] == ROAD]
    if len(owned) != 2 or {p["source_lane"] for p in owned} != set(BOUNDARIES):
        raise SupportRejected("fixed-lane-source-identity-mismatch")
    interval = spec.get("median_interval")
    if interval is None or interval[1]-interval[0] < E.MEDIAN_MIN_M:
        raise SupportRejected("existing-positive-width-median-required")
    tails, issues = E._bridge_tails(base, base.buffer(E.COVER_TOL_M), pieces, mouths)
    old_medians, median_issues, median_inside = E._median_tails(base, base.buffer(E.COVER_TOL_M), tails, mouths)
    if issues or median_issues:
        raise SupportRejected("original-tail-or-median-guard-failed")
    median_guards = []
    for mouth in mouths:
        width = mouth.get("median_interval")
        required = width is not None and width[1]-width[0] >= E.MEDIAN_MIN_M
        represented = any(m["road_id"] == mouth["road_id"] for m in old_medians)
        inside = mouth["road_id"] in median_inside
        if required and not represented and not inside:
            raise SupportRejected("original-positive-width-median-unresolved")
        median_guards.append({"road_id": mouth["road_id"], "required": required,
                              "continuation_present": represented, "covered_by_source_base": inside})
    raw = unary_union([p["geometry"] for p in pieces])
    fixed_support = unary_union([base, raw, *[m["geometry"] for m in old_medians]])
    origin = np.asarray(spec["pose"][:2], float)
    axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    normal = np.array([-axis[1], axis[0]])
    by_lane = {p["source_lane"]: p["geometry"] for p in owned}
    owned_by_lane = {p["source_lane"]: p for p in owned}
    chains, boundary_evidence = [], []
    for lane in (ENTRY_LANE, DEPARTURE_LANE):
        chain, indices = _inner_chain(by_lane[lane], origin, axis, normal, sum(interval)/2)
        chain, binding, discrepancy, roundoff = _bound_inner_chain(owned_by_lane[lane], chain, origin, axis)
        chain, extension = _extend_to_base(chain, base, axis)
        chains.append(chain)
        boundary_evidence.append({"source_lane": lane, **BOUNDARIES[lane],
            "verified_source_binding": binding,
            "polygon_chain_to_bound_source_m": discrepancy, "identity_roundoff_bound_m": roundoff,
            "source_piece_sha256": _sha(by_lane[lane].wkb),
            "oriented_polygon_vertex_indices": indices,
            "source_chain_world_xy": [p.tolist() for p in chain[:-1]], "extension": extension})
    rebuilt = Polygon([*chains[0], *chains[1][::-1]])
    if not rebuilt.is_valid or rebuilt.area <= 0:
        raise SupportRejected("invalid-source-boundary-median")
    old_own_median = unary_union([m["geometry"] for m in old_medians if m["road_id"] == ROAD])
    if old_own_median.is_empty:
        raise SupportRejected("existing-median-missing")
    new_own_median = unary_union([old_own_median, rebuilt])
    overlap_roundoff_m = 128*math.ulp(max(1., *map(abs, fixed_support.bounds)))
    added_median = new_own_median.difference(old_own_median)
    overlap_roundoff_m2 = overlap_roundoff_m*(added_median.length+raw.length)
    if added_median.intersection(raw).area > overlap_roundoff_m2:
        raise SupportRejected("new-source-median-overlaps-raw-lane-material")
    if new_own_median.difference(fixed_support.buffer(E.COVER_TOL_M)).area > E.SLIVER_M2:
        raise SupportRejected("rebuilt-median-outside-fixed-source-support")
    if new_own_median.intersection(base).area <= E.SLIVER_M2:
        raise SupportRejected("median-lacks-positive-source-base-overlap")
    new_tails, withdrawn = [], []
    for tail in tails:
        item = dict(tail)
        source_lanes = sorted(p["source_lane"] for p in pieces if p["road_id"] == tail["road_id"]
                              and p["geometry"].intersection(tail["geometry"]).area > 1e-8)
        item.update(source_lanes=source_lanes, status="INFERRED" if tail.get("sweep_m") else "TRANSFORMED",
                    support_kind="existing-supported-tail-bridge" if tail.get("sweep_m") else "raw-source-tail")
        if tail["road_id"] == ROAD and DEPARTURE_LANE in source_lanes:
            if source_lanes != [DEPARTURE_LANE]:
                raise SupportRejected("departure-component-source-ambiguous")
            original = by_lane[DEPARTURE_LANE]
            unsupported = tail["geometry"].difference(fixed_support.buffer(E.COVER_TOL_M)).area
            if not tail.get("sweep_m") or unsupported <= E.SLIVER_M2:
                raise SupportRejected("fixed-departure-sweep-is-not-unsupported")
            withdrawn.append({"old": _record(tail["geometry"]), "raw": _record(original),
                "unsupported_5cm_area_m2": unsupported,
                "reason": "withdraw-inferred-sweep-without-source-support; preserve-entire-raw-lane"})
            item.update(geometry=original, sweep_m=0., added_area_m2=0., status="TRANSFORMED", support_kind="raw-source-tail")
        if item["geometry"].difference(fixed_support.buffer(E.COVER_TOL_M)).area > E.SLIVER_M2:
            raise SupportRejected("retained-tail-outside-fixed-source-support")
        new_tails.append(item)
    if len(withdrawn) != 1:
        raise SupportRejected("fixed-departure-sweep-not-unique")
    medians = [{**m, "status": "INFERRED", "support_kind": "existing-median-continuation"}
               for m in old_medians if m["road_id"] != ROAD]
    medians.append({"road_id": ROAD, "geometry": new_own_median, "axis": axis,
                    "status": "INFERRED", "support_kind": "adjacent-source-boundary-median",
                    "source_lanes": [ENTRY_LANE, DEPARTURE_LANE]})
    expected_restricted = unary_union([base, *[t["geometry"] for t in new_tails]])
    expected_median = unary_union([m["geometry"] for m in medians])
    expected_total = unary_union([expected_restricted, expected_median])
    original_total = unary_union([base, *[t["geometry"] for t in tails],
                                  *[m["geometry"] for m in old_medians]])
    planned_new_holes = _holes(expected_total).difference(_holes(original_total).buffer(overlap_roundoff_m))
    if planned_new_holes.area > overlap_roundoff_m*(original_total.length+expected_total.length):
        raise SupportRejected("support-reconstruction-introduces-visual-hole")
    if len(JS._polygons(expected_total.buffer(overlap_roundoff_m))) != len(JS._polygons(original_total.buffer(overlap_roundoff_m))):
        raise SupportRejected("support-reconstruction-changes-visual-connectivity")
    scale = max(1., *[abs(v) for v in fixed_support.bounds])
    evidence = {"schema": SCHEMA, "fixed_scope": {"road_id": ROAD, "enter_link": ENTER, "leave_link": LEAVE},
        "support_geometry_wkb_hex": fixed_support.wkb_hex, "fixed_support": _record(fixed_support),
        "raw_source": _record(raw), "source_base": _record(base),
        "old_medians": _record(unary_union([m["geometry"] for m in old_medians])),
        "original_median_inside_base": sorted(median_inside), "original_median_guards": median_guards,
        "expected_restricted": _record(expected_restricted), "expected_median": _record(expected_median),
        "source_pieces": [{"source_lane": p["source_lane"], "road_id": p["road_id"], **_record(p["geometry"])} for p in pieces],
        "source_mouths": mouths, "median_boundary_evidence": boundary_evidence,
        "withdrawn_inferred_sweeps": withdrawn,
        "new_median_base_overlap_m2": new_own_median.intersection(base).area,
        "new_median_old_missing_m2": old_own_median.difference(new_own_median).area,
        "new_median_added_area_m2": new_own_median.difference(old_own_median).area,
        "new_median_raw_overlap_m2": new_own_median.intersection(raw).area,
        "new_median_added_raw_overlap_m2": new_own_median.difference(old_own_median).intersection(raw).area,
        "new_median_raw_overlap_roundoff_bound_m2": overlap_roundoff_m2,
        "original_visual_components": len(JS._polygons(original_total)),
        "planned_visual_components": len(JS._polygons(expected_total)),
        "planned_new_holes_m2": planned_new_holes.area,
        "original_holes": _record(_holes(original_total)),
        "median_restricted_overlap_m2": expected_median.intersection(expected_restricted).area,
        "serialization_band_upper_bound_m": scale*1e-8,
        "driving_continuity_proven": False,
        "departure_raw_asphalt_base_gap_m": by_lane[DEPARTURE_LANE].distance(base),
        "connectivity_scope": "visual auxiliary layers only; median is not a driving route",
        "candidate_accepted": False}
    evidence["input_sha256"] = _sha(json.dumps({"base": _record(base), "pieces": evidence["source_pieces"],
        "mouths": mouths}, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())
    evidence["evidence_sha256"] = _json_sha(evidence)
    return new_tails, medians, evidence


def audit_written_support(actual_restricted, actual_median, evidence, *, numeric_band_m=0.):
    """Check actual XML footprints by layer; numerical band is not 5 cm.

    The caller supplies its PREWRITING serialization error bound and still
    verifies each family against its exact generated target. Here deletion,
    type reassignment, source loss and actual topological gaps fail closed.
    """
    problems = []
    try:
        if evidence.get("schema") != SCHEMA:
            raise SupportRejected("support-evidence-schema-mismatch")
        if evidence.get("evidence_sha256") != _json_sha({k: v for k, v in evidence.items() if k != "evidence_sha256"}):
            raise SupportRejected("support-evidence-binding-mismatch")
        band = float(numeric_band_m)
        if not math.isfinite(band) or band < 0 or band > evidence["serialization_band_upper_bound_m"]:
            raise SupportRejected("invalid-numerical-serialization-band")
        fixed, raw, old_medians, base = (_read(evidence[k]) for k in ("fixed_support", "raw_source", "old_medians", "source_base"))
        expected_r, expected_m = (_read(evidence[k]) for k in ("expected_restricted", "expected_median"))
        layer_rows = []
        for kind, actual, expected in [("restricted", actual_restricted, expected_r), ("median", actual_median, expected_m)]:
            if not actual.is_valid:
                raise SupportRejected("invalid-actual-surface")
            missing = expected.difference(actual.buffer(band))
            outside = actual.difference(expected.buffer(band))
            passed = missing.is_empty and outside.is_empty
            if not passed:
                problems.append("written-"+kind+"-differs-from-supported-target")
            layer_rows.append({"layer": kind, "passed": passed, "missing_m2": missing.area, "outside_m2": outside.area})
        total = unary_union([actual_restricted, actual_median])
        raw_missing = raw.difference(actual_restricted.buffer(band))
        original_median_missing = old_medians.difference(actual_median.buffer(band))
        outside = total.difference(fixed.buffer(E.COVER_TOL_M))
        if not raw_missing.is_empty:
            problems.append("raw-source-lane-material-missing")
        if not original_median_missing.is_empty:
            problems.append("existing-median-material-missing")
        if outside.area > E.SLIVER_M2:
            problems.append("outside-fixed-source-support")
        expected_total = unary_union([expected_r, expected_m])
        numeric_total = total.buffer(band) if band else total
        numerical_hole_area = sum(Polygon(r).area for p in JS._polygons(numeric_total) for r in p.interiors)
        expected_hole_area = sum(Polygon(r).area for p in JS._polygons(expected_total) for r in p.interiors)
        if len(JS._polygons(numeric_total)) != len(JS._polygons(expected_total.buffer(band) if band else expected_total)):
            problems.append("actual-visual-surface-connectivity-changed")
        # The planned target's legitimate holes remain covered by the layer
        # identity check above; no new genuine hole may be hidden as rounding.
        if numerical_hole_area > expected_hole_area + band*total.length:
            problems.append("actual-visual-surface-new-hole")
        mouths = []
        for spec in evidence["source_mouths"]:
            o = np.asarray(spec["pose"][:2]); h = spec["pose"][2]
            n = np.array([-math.sin(h), math.cos(h)])
            cross = LineString([o-100*n, o+100*n])
            required = expected_total.intersection(cross)
            missing = required.difference(total.buffer(band))
            mouths.append({"road_id": spec["road_id"], "planned_supported_width_m": required.length,
                           "missing_written_width_m": missing.length, "passed": missing.is_empty,
                           "scope": "planned auxiliary visual coverage only; not driving full-mouth coverage"})
            if not missing.is_empty:
                problems.append("written-supported-mouth-coverage-changed")
        return {"status": "PASS" if not problems else "FAIL", "passed": not problems, "problems": problems,
            "layers": layer_rows, "raw_source_missing_m2": raw_missing.area,
            "old_median_missing_m2": original_median_missing.area, "outside_fixed_support_m2": outside.area,
            "numeric_serialization_band_m": band, "source_cover_tolerance_m": E.COVER_TOL_M,
            "actual_visual_components": len(JS._polygons(total)),
            "components_with_numeric_roundoff": len(JS._polygons(numeric_total)),
            "actual_holes_area_m2": sum(Polygon(r).area for p in JS._polygons(total) for r in p.interiors),
            "mouths": mouths,
            "connectivity_via_median": len(JS._polygons(numeric_total)) < len(JS._polygons(actual_restricted.buffer(band) if band else actual_restricted)),
            "driving_continuity_proven": False,
            "departure_raw_asphalt_base_gap_m": evidence["departure_raw_asphalt_base_gap_m"],
            "actual_driving_mouths_and_route_seams_require_separate_check": True,
            "candidate_accepted": False}
    except (KeyError, TypeError, ValueError) as exc:
        return {"status": "FAIL", "passed": False, "problems": [str(exc)], "candidate_accepted": False,
                "driving_continuity_proven": False}
