"""Experimental fixed-source reconstruction using actual supported tracks.

Only auxiliary paving is replaced. A local PASS proves its representation and
source support, never driving continuity, candidate acceptance or delivery.
The old cell experiment and the default converter remain unchanged.
"""
from __future__ import annotations

import copy
import hashlib
from itertools import count
import math
import xml.etree.ElementTree as ET

from shapely import wkb
from shapely.geometry import LineString
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.ops import envelope_surface as E, junction_surface as JS
from mapforge.ops.map_to_xodr import _mouth_apron_axes
from . import source_surface_partition as P
from . import source_surface_tracks as T
from . import source_surface_support as S

INTENT_TYPE = "source_supported_surface_tracks_v2"


def make_intent(snapshot):
    return {**P.make_intent(snapshot), "type": INTENT_TYPE,
        "affected_scope": "all-auxiliary-paving-of-bound-source-junction",
        "partition": "source-topology-proportional-parent-tracks",
        "tail_action": "retain-supported-existing-bridges-and-retract-unsupported-road12-departure-sweep",
        "median_action": "preserve-existing-median-and-complete-from-adjacent-raw-source-boundaries",
        "support_domain": "original-base-raw-pieces-and-original-median-before-reconstruction",
        "preprocess_mouth_policy": "defer-only-inherited-nonexpanding-gaps-from-fresh-partial-reference",
        "final_mouth_policy": "original-source-cover-tolerance-required",
        "padding_m": 0.0, "source_holes": "preserve",
        "driving_roads": "unchanged-before-default-postprocess",
        "driving_continuity": "requires-independent-whole-map-checks"}


def apply_rebuild(root, source, junction, project, decisions, snapshot, intent):
    if intent != make_intent(snapshot) or junction.pid != P.JUNCTION_ID:
        raise P.PartitionRejected("intent-not-registered-or-not-confirmed")
    before = P.unchanged_subtrees(root)
    mouths = JS.written_mouth_specs(root, decisions)
    targets = [m for m in mouths if m["enter_link"] == P.ENTER_LINK
               and m.get("leave_links") == [P.LEAVE_LINK]]
    if len(targets) != 1:
        raise P.PartitionRejected("source-leg-identity-ambiguous")
    target = targets[0]
    base, pieces, _, summary = JS.source_tail_regions(source, junction, project, mouths)
    if summary["issues"]:
        raise P.PartitionRejected("original-source-region-guard-failed")
    if {p["source_lane"] for p in pieces if p["road_id"] == target["road_id"]} != P.LANES:
        raise P.PartitionRejected("pre-registered-source-scope-changed")
    pieces = S.bind_source_boundaries(source, project, pieces)
    tails, medians, support_evidence = S.build_support(base, pieces, mouths)
    axes = _mouth_apron_axes(mouths)
    if not axes:
        raise P.PartitionRejected("explicit-source-paving-axis-unavailable")
    regions = [{"geometry": base, "axis": axes[0], "role": "source-polygon",
                "lane_type": "restricted", "source_lanes": [], "status": "TRANSFORMED"}]
    mouth_map = {m["road_id"]: m for m in mouths}
    for group, role, lane_type in [(tails, "source-supported-tail", "restricted"),
                                  (medians, "source-boundary-median", "median")]:
        for item in group:
            spec = mouth_map[item["road_id"]]
            regions.append({"geometry": item["geometry"],
                "axis": [math.cos(spec["pose"][2]), math.sin(spec["pose"][2])],
                "role": item.get("support_kind", role), "lane_type": lane_type, "source_road": item["road_id"],
                "source_lanes": sorted(item.get("source_lanes", {p["source_lane"] for p in pieces
                                         if p["road_id"] == item["road_id"]})),
                "status": item.get("status", "INFERRED" if lane_type == "median" or item.get("sweep_m", 0) else "TRANSFORMED"),
                "existing_bridge": item.get("existing_bridge")})
    result = copy.deepcopy(root)
    doc = W.XodrDoc("source-supported-surface-tracks")
    retained_ids = {int(r.get("id")) for r in result.findall("road")
                    if r.get("name") != "junction_paving"}
    ids = (i for i in count(50) if i not in retained_ids)
    families = []
    for index, region in enumerate(regions):
        provenance = {"eligibility": "excluded", "role": "paving", "status": region["status"],
            "support_kind": region["role"], "travel_direction": "with_s",
            "exclusion_code": "source-polygon-paving", "workbench_intent": INTENT_TYPE,
            "source_snapshot_id": snapshot["snapshot_id"], "source_content_hash": snapshot["content_hash"],
            "source_junction_id": junction.pid, "source_lanes": region["source_lanes"],
            "source_region_index": index, "source_region_sha256": hashlib.sha256(region["geometry"].wkb).hexdigest()}
        representation = T.append_polygon(doc, region["geometry"], 1, ids,
            preferred_axis=region["axis"], lane_type=region["lane_type"], provenance=provenance)
        families.append({"region_index": index, "role": region["role"],
                         "lane_type": region["lane_type"], "source_road": region.get("source_road"),
                         "existing_bridge": region.get("existing_bridge"),
                         "provenance": provenance, "representation": representation})
    previous_ids = [r.get("id") for r in result.findall("road") if r.get("name") == "junction_paving"]
    for road in list(result.findall("road")):
        if road.get("name") == "junction_paving":
            result.remove(road)
    junction_element = result.find("junction")
    insertion = list(result).index(junction_element) if junction_element is not None else len(result)
    for road in doc.roads:
        holder = ET.Element("OpenDRIVE")
        doc._road_el(holder, road)
        result.insert(insertion, holder.find("road"))
        insertion += 1
    if P.unchanged_subtrees(result) != before:
        raise P.PartitionRejected("non-paving-subtree-changed")
    written_ids = [str(r.road_id) for r in doc.roads]
    evidence = {"schema": "mapforge/workbench-source-surface-reconstruction/v1",
        "intent": intent, "families": families, "source_support": support_evidence,
        "non_paving_subtrees_before": before, "non_paving_subtrees_after": P.unchanged_subtrees(result),
        "non_paving_unchanged": True, "complexity_before": P._counts(root),
        "complexity_after": P._counts(result),
        "paving_id_diff": {"previous_partial_ids": previous_ids, "candidate_ids": written_ids,
            "removed": sorted(set(previous_ids)-set(written_ids)),
            "added": sorted(set(written_ids)-set(previous_ids)),
            "reused_with_changed_geometry": sorted(set(previous_ids)&set(written_ids))},
        "candidate_accepted": False, "production_crs_verified": False,
        "default_geometry_modified": False, "driving_continuity_proven": False}
    context = {"base": base, "target": target,
        "tails": unary_union([x["geometry"] for x in tails if x["road_id"] == target["road_id"]]),
        "support": wkb.loads(bytes.fromhex(support_evidence["support_geometry_wkb_hex"])),
        "represented_support": unary_union([r["geometry"] for r in regions])}
    return result, evidence, context


def audit_written(root, evidence, context, *, require_driving_unchanged=True):
    if require_driving_unchanged and P.unchanged_subtrees(root) != evidence["non_paving_subtrees_before"]:
        raise P.PartitionRejected("serialized-non-paving-subtree-changed")
    expected = evidence["paving_id_diff"]["candidate_ids"]
    actual = [r.get("id") for r in root.findall("road") if r.get("name") == "junction_paving"]
    if len(expected) != len(set(expected)) or set(actual) != set(expected) or len(actual) != len(expected):
        raise P.PartitionRejected("serialized-paving-id-set-changed")
    families = [T.audit_written(root, f["representation"]) for f in evidence["families"]]
    shapes = T.written_geometry(root, actual)
    layer_shapes = {}
    for lane_type in ("restricted", "median"):
        ids = [rid for f in evidence["families"] if f["lane_type"] == lane_type
               for rid in f["representation"]["road_ids"]]
        layer_shapes[lane_type] = T.written_geometry(root, ids)
    numeric_band = max(f["representation"]["numeric_serialization_band_m"] for f in evidence["families"])
    source_audit = S.audit_written_support(layer_shapes["restricted"], layer_shapes["median"],
                                           evidence["source_support"], numeric_band_m=numeric_band)
    mouths = audit_mouth_coverage(root, evidence["source_support"]["source_mouths"], layer_shapes, numeric_band)
    support = context["support"]
    outside = shapes.difference(support.buffer(E.COVER_TOL_M))
    missing = support.difference(shapes.buffer(E.COVER_TOL_M))
    represented_difference = shapes.symmetric_difference(context["represented_support"])
    gap = P.check_preserved_gap(shapes, context["tails"], context["base"], context["target"])
    material_passed = (all(f.get("passed") is True for f in families) and source_audit.get("passed") is True
                and outside.area <= E.SLIVER_M2 and missing.area <= E.SLIVER_M2
                and represented_difference.area <= E.SLIVER_M2 and gap["gap_preserved"])
    accepted = material_passed and mouths["within_existing_source_cover_tolerance"]
    return {"status": "PASS" if accepted else "FAIL", "candidate_accepted": False,
        "material_support_passed": material_passed,
        "readback": "analytic XML line and all offset/width breakpoints", "families": families,
        "source_support_audit": source_audit, "source_cover_tolerance_m": E.COVER_TOL_M,
        "written_road_mouth_coverage": mouths,
        "sliver_area_limit_m2": E.SLIVER_M2, "outside_source_support_m2": float(outside.area),
        "missing_source_support_m2": float(missing.area),
        "represented_symmetric_difference_m2": float(represented_difference.area),
        "full_paving_gap": gap, "non_paving_unchanged_checked": require_driving_unchanged,
        "driving_continuity_proven": False, "whole_map_validation_required": True}


def _paving_layers(root):
    ids = {"restricted": [], "median": []}
    for road in root.findall("road"):
        if road.get("name") == "junction_paving":
            kinds = {lane.get("type") for lane in road.findall("lanes/laneSection/right/lane")}
            if len(kinds) != 1 or not kinds <= ids.keys():
                raise P.PartitionRejected("unknown-auxiliary-material-type")
            ids[next(iter(kinds))].append(road.get("id"))
    return {kind: T.written_geometry(root, group) for kind, group in ids.items()}


def compare_preprocess_mouths(current_root, baseline_root, current_evidence, baseline_evidence, current_audit):
    """Permit original, nonexpanding raw gaps to reach the actual refit stage.

    The caller reconstructs the old reference from the same fresh partial XML,
    source and snapshot. No prior complete XODR is an input. This comparison
    does not turn a mouth FAIL into PASS; final mouths retain the 5 cm gate.
    """
    unchanged = (P.unchanged_subtrees(current_root) == P.unchanged_subtrees(baseline_root)
                 == current_evidence["non_paving_subtrees_before"])
    current_band = max(f["representation"]["numeric_serialization_band_m"] for f in current_evidence["families"])
    old_band = max(f["representation"]["numeric_serialization_band_m"] for f in baseline_evidence["families"])
    band = current_band + old_band
    if not math.isfinite(band) or band < 0:
        raise P.PartitionRejected("invalid-preprocess-comparison-band")
    current_layers, baseline_layers = _paving_layers(current_root), _paving_layers(baseline_root)
    mouths = current_evidence["source_support"]["source_mouths"]
    now = audit_mouth_coverage(current_root, mouths, current_layers, current_band)
    old = audit_mouth_coverage(baseline_root, mouths, baseline_layers, old_band)
    old_rows = {(r["road_id"], r["lane_id"]): r for r in old["rows"]}
    rows, regressions = [], []
    for row in now["rows"]:
        key = (row["road_id"], row["lane_id"])
        previous = old_rows.get(key)
        if previous is None or previous["mouth_xy"] != row["mouth_xy"] or previous["lane_type"] != row["lane_type"]:
            regressions.append({"road_id": key[0], "lane_id": key[1], "code": "physical-mouth-changed"})
            continue
        kind = "restricted" if row["lane_type"] == "driving" else "median"
        line = LineString(row["mouth_xy"])
        new_missing = line.difference(current_layers[kind])
        old_missing = line.difference(baseline_layers[kind])
        # A previously empty gap has an empty buffer. First remove only the
        # independently derived current serialization uncertainty; otherwise
        # nanometre writeout noise would look like a newly introduced gap.
        new_resolved_missing = line.difference(current_layers[kind].buffer(current_band))
        expansion = new_resolved_missing.difference(old_missing.buffer(band))
        new_beyond = line.difference(current_layers[kind].buffer(E.COVER_TOL_M + current_band))
        old_beyond = line.difference(baseline_layers[kind].buffer(E.COVER_TOL_M))
        beyond_expansion = new_beyond.difference(old_beyond.buffer(band))
        passed = (expansion.is_empty and beyond_expansion.is_empty
                  and not (previous["within_existing_source_cover_tolerance"]
                           and not row["within_existing_source_cover_tolerance"]))
        item = {"road_id": key[0], "lane_id": key[1], "passed": passed,
            "old_uncovered_width_m": old_missing.length, "new_uncovered_width_m": new_missing.length,
            "new_uncovered_beyond_numeric_band_m": new_resolved_missing.length,
            "new_missing_outside_old_numeric_band_m": expansion.length,
            "new_over_tolerance_gap_outside_old_numeric_band_m": beyond_expansion.length,
            "old_within_source_tolerance": previous["within_existing_source_cover_tolerance"],
            "new_within_source_tolerance": row["within_existing_source_cover_tolerance"]}
        rows.append(item)
        if not passed:
            regressions.append({"road_id": key[0], "lane_id": key[1], "code": "mouth-gap-expanded-or-shifted"})
    if not unchanged:
        regressions.append({"code": "non-paving-subtree-changed"})
    if current_audit.get("material_support_passed") is not True:
        regressions.append({"code": "source-material-support-failed"})
    if now != current_audit.get("written_road_mouth_coverage"):
        regressions.append({"code": "current-mouth-audit-mismatch"})
    return {"schema": "mapforge/preprocess-mouth-inheritance/v1", "allow_postprocess": not regressions,
        "current_material_support_passed": current_audit.get("material_support_passed") is True,
        "non_paving_unchanged": unchanged, "regressions": regressions, "rows": rows,
        "numeric_comparison_band_m": band, "baseline_mouth_coverage": old, "current_mouth_coverage": now,
        "deferred_failures": [{"road_id": r["road_id"], "lane_id": r["lane_id"]} for r in now["rows"]
                              if not r["within_existing_source_cover_tolerance"]],
        "baseline_origin": "old auxiliary reconstruction of the same freshly generated partial XML",
        "final_mouth_gate_unchanged": True, "candidate_accepted": False}


def audit_mouth_coverage(root, mouths, layer_shapes, numeric_band):
    """Read full-width road endpoints independently of the planned paving.

    Driving mouths use restricted paving only: median material cannot hide an
    asphalt gap. Both exact and original source-tolerance coverage are reported.
    Passing that tolerance is not an exact surface-contact or route proof.
    """
    from scripts.internal_edge_jets import states
    rows = []
    for spec in mouths:
        matches = [r for r in root.findall("road") if r.get("id") == spec["road_id"]]
        if len(matches) != 1:
            raise P.PartitionRejected("written-mouth-road-missing-or-ambiguous")
        road = matches[0]
        sections = road.findall("lanes/laneSection")
        if not sections:
            raise P.PartitionRejected("written-mouth-lane-section-missing")
        length = float(road.get("length"))
        for lane in sections[-1].findall("left/lane") + sections[-1].findall("right/lane"):
            kind = lane.get("type")
            if kind not in {"driving", "median"}:
                continue
            lid = int(lane.get("id"))
            inner, outer = states(road, lid, length, True)
            line = LineString([inner[:2], outer[:2]])
            if not math.isfinite(line.length) or line.length <= 0:
                raise P.PartitionRejected("invalid-written-mouth-width")
            layer = layer_shapes["restricted" if kind == "driving" else "median"]
            missing = line.difference(layer)
            within_numeric = line.difference(layer.buffer(numeric_band)).is_empty
            within_source = line.difference(layer.buffer(E.COVER_TOL_M)).is_empty
            rows.append({"road_id": spec["road_id"], "lane_id": lid, "lane_type": kind,
                "station_m": length, "mouth_xy": [list(inner[:2]), list(outer[:2])],
                "width_m": line.length, "uncovered_exact_width_m": missing.length,
                "within_numeric_serialization_band": within_numeric,
                "within_existing_source_cover_tolerance": within_source})
    if not rows or not any(row["lane_type"] == "driving" for row in rows):
        raise P.PartitionRejected("empty-driving-mouth-check")
    return {"rows": rows,
        "within_existing_source_cover_tolerance": all(r["within_existing_source_cover_tolerance"] for r in rows),
        "all_exact_contacts_within_numeric_band": all(r["within_numeric_serialization_band"] for r in rows),
        "numeric_serialization_band_m": numeric_band, "source_cover_tolerance_m": E.COVER_TOL_M,
        "driving_continuity_proven": False, "route_seams_evaluated": False}
