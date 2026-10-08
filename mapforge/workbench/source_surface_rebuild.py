"""Source-bound auxiliary surface reconstruction for the fixed WB11 case.

This experimental adapter replaces only auxiliary XML. It does not register an
editor capability, mutate the default converter, or waive a whole-map gate.
"""
from __future__ import annotations

import copy
import hashlib
from itertools import count
import math
import xml.etree.ElementTree as ET

from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.ops import envelope_surface as E, junction_surface as JS
from mapforge.ops.map_to_xodr import _mouth_apron_axes
from . import source_surface_partition as P
from . import source_surface_cells as C

INTENT_TYPE = "source_surface_connected_intervals_v1"


def make_intent(snapshot):
    original = P.make_intent(snapshot)
    return {**original, "type": INTENT_TYPE,
            "partition": "source-vertex-events-and-connected-transverse-intervals",
            "affected_scope": "all-auxiliary-paving-of-bound-source-junction",
            "padding_m": 0.0, "source_holes": "preserve",
            "driving_roads": "unchanged-before-default-postprocess"}


def _geometry_id(geometry):
    return hashlib.sha256(geometry.wkb).hexdigest()


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
    cover = base.buffer(E.COVER_TOL_M)
    tails, tail_issues = E._bridge_tails(base, cover, pieces, mouths)
    medians, median_issues, inside = E._median_tails(base, cover, tails, mouths)
    if summary["issues"] or tail_issues or median_issues:
        raise P.PartitionRejected("original-source-surface-guard-failed")
    own_lanes = {p["source_lane"] for p in pieces if p["road_id"] == target["road_id"]}
    if own_lanes != P.LANES or target["road_id"] in inside:
        raise P.PartitionRejected("pre-registered-source-scope-changed")
    for spec in mouths:
        interval = spec.get("median_interval")
        if (interval and interval[1]-interval[0] >= E.MEDIAN_MIN_M
                and spec["road_id"] not in inside
                and not any(m["road_id"] == spec["road_id"] for m in medians)):
            raise P.PartitionRejected("median-tail-unresolved")
    axes = _mouth_apron_axes(mouths)
    if not axes:
        raise P.PartitionRejected("explicit-source-paving-axis-unavailable")
    regions = [{"geometry": base, "axis": axes[0], "role": "source-polygon",
                "lane_type": "restricted", "source_lanes": [], "status": "TRANSFORMED"}]
    mouth_map = {m["road_id"]: m for m in mouths}
    for group, role, lane_type in [(tails, "source-tail", "restricted"),
                                  (medians, "existing-median-continuation", "median")]:
        for item in group:
            spec = mouth_map[item["road_id"]]
            regions.append({"geometry": item["geometry"],
                "axis": [math.cos(spec["pose"][2]), math.sin(spec["pose"][2])],
                "role": role, "lane_type": lane_type, "source_road": item["road_id"],
                "source_lanes": sorted({p["source_lane"] for p in pieces
                                         if p["road_id"] == item["road_id"]}),
                "status": "INFERRED" if lane_type == "median" or item.get("sweep_m", 0) else "APPROXIMATED",
                "existing_bridge": {k: v for k, v in item.items() if k not in {"geometry", "axis"}}})
    result = copy.deepcopy(root)
    doc = W.XodrDoc("source-supported-connected-intervals")
    retained_ids = {int(r.get("id")) for r in result.findall("road")
                    if r.get("name") != "junction_paving"}
    ids = (i for i in count(50) if i not in retained_ids)
    families = []
    for index, region in enumerate(regions):
        provenance = {"eligibility": "excluded", "role": "paving",
            "status": region["status"], "support_kind": region["role"],
            "travel_direction": "with_s", "exclusion_code": "source-polygon-paving",
            "workbench_intent": INTENT_TYPE, "source_snapshot_id": snapshot["snapshot_id"],
            "source_content_hash": snapshot["content_hash"], "source_junction_id": junction.pid,
            "source_lanes": region["source_lanes"], "source_region_index": index,
            "source_region_sha256": _geometry_id(region["geometry"])}
        representation = C.append_polygon(doc, region["geometry"], 1, ids,
            preferred_axis=region["axis"], lane_type=region["lane_type"], provenance=provenance)
        families.append({"region_index": index, "role": region["role"],
                         "source_road": region.get("source_road"), "provenance": provenance,
                         "existing_bridge": region.get("existing_bridge"),
                         "representation": representation})
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
    evidence = {"schema": "mapforge/workbench-source-supported-rebuild/v1",
        "intent": intent, "families": families,
        "non_paving_subtrees_before": before,
        "non_paving_subtrees_after": P.unchanged_subtrees(result),
        "non_paving_unchanged": True,
        "complexity_before": P._counts(root), "complexity_after": P._counts(result),
        "paving_id_diff": {"previous_partial_ids": previous_ids, "candidate_ids": written_ids,
            "removed": sorted(set(previous_ids)-set(written_ids)),
            "added": sorted(set(written_ids)-set(previous_ids)),
            "reused_with_changed_geometry": sorted(set(previous_ids)&set(written_ids))},
        "median_inside_source_polygon": sorted(inside),
        "original_guards": {"source_issues": summary["issues"],
                            "tail_issues": tail_issues, "median_issues": median_issues},
        "candidate_accepted": False, "production_crs_verified": False,
        "default_geometry_modified": False}
    context = {"base": base, "target": target, "regions": regions,
        "tails": unary_union([x["geometry"] for x in tails if x["road_id"] == target["road_id"]]),
        "support": unary_union([base, *[p["geometry"] for p in pieces],
                                 *[m["geometry"] for m in medians]]),
        "represented_support": unary_union([r["geometry"] for r in regions])}
    return result, evidence, context


def audit_written(root, evidence, context, *, require_driving_unchanged=True):
    if require_driving_unchanged and P.unchanged_subtrees(root) != evidence["non_paving_subtrees_before"]:
        raise P.PartitionRejected("serialized-non-paving-subtree-changed")
    expected_ids = set(evidence["paving_id_diff"]["candidate_ids"])
    actual_ids = [r.get("id") for r in root.findall("road") if r.get("name") == "junction_paving"]
    if set(actual_ids) != expected_ids or len(actual_ids) != len(expected_ids):
        raise P.PartitionRejected("serialized-paving-id-set-changed")
    families = [C.audit_written(root, f["representation"]) for f in evidence["families"]]
    shapes = C.written_geometry(root, actual_ids)
    support = context["support"]
    outside = shapes.difference(support.buffer(E.COVER_TOL_M))
    missing = support.difference(shapes.buffer(E.COVER_TOL_M))
    represented_difference = shapes.symmetric_difference(context["represented_support"])
    gap = P.check_preserved_gap(shapes, context["tails"], context["base"], context["target"])
    # Geometry is checked in both directions: deleting material cannot pass by
    # merely keeping every remaining polygon inside the source support.
    accepted = (all(f.get("passed") is True for f in families)
                and outside.area <= E.SLIVER_M2 and missing.area <= E.SLIVER_M2
                and represented_difference.area <= E.SLIVER_M2 and gap["gap_preserved"])
    return {"status": "PASS" if accepted else "FAIL", "candidate_accepted": False,
        "readback": "analytic XML line and all offset/width breakpoints", "families": families,
        "source_cover_tolerance_m": E.COVER_TOL_M, "sliver_area_limit_m2": E.SLIVER_M2,
        "outside_source_support_m2": float(outside.area), "missing_source_support_m2": float(missing.area),
        "represented_symmetric_difference_m2": float(represented_difference.area),
        "full_paving_gap": gap, "non_paving_unchanged_checked": require_driving_unchanged,
        "whole_map_validation_required": True}
