"""One source-bound WB11 experiment: keep a separator by partitioning paving.

This is not registered as a production compiler or editor capability. It never
edits the source, driving roads, topology, or the default conversion modules.
Every output remains a rejected experiment until actual XML support checks and
the normal whole-map evaluators have run. No complete input XODR is required.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from mapforge.ops import envelope_surface as E
from mapforge.ops import junction_surface as JS

JUNCTION_ID = "2023062110304177600"
ENTER_LINK = "2023061413355346000"
LEAVE_LINK = "2023061413291619261"
SNAPSHOT_ID = "c7e9b7866dad49d00f46f27a96ed60d852572e031bd102be9ee2d5716820ab71"
SNAPSHOT_CONTENT_HASH = "f8472eac81d121ba1bdb4e8639ac69b162833c902031835ef2922e24b348f8a4"
INTENT_TYPE = "source_surface_family_partition"
LANES = frozenset({"2023061413362614443", "2023061413362616440",
                   "2023061413291634834", "2023061413291638832"})


class PartitionRejected(ValueError):
    """A source/geometry condition failed; caller preserves its output evidence."""


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def _json_bytes(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf8")


def make_intent(snapshot):
    if (snapshot.get("snapshot_id") != SNAPSHOT_ID
            or snapshot.get("content_hash") != SNAPSHOT_CONTENT_HASH
            or snapshot.get("junction_id") != JUNCTION_ID):
        raise PartitionRejected("source-snapshot-not-registered")
    return {"type": INTENT_TYPE, "source_snapshot_id": SNAPSHOT_ID,
            "source_content_hash": SNAPSHOT_CONTENT_HASH,
            "junction_id": JUNCTION_ID, "enter_link": ENTER_LINK,
            "leave_links": [LEAVE_LINK], "source_lanes": sorted(LANES),
            "partition": "source-tail-components-and-existing-median",
            "gap_action": "preserve-unpaved", "confirmed": True}


def unchanged_subtrees(root):
    """Canonical XML semantics, ignoring pretty-print whitespace only."""
    result = {}
    for i, element in enumerate(root):
        if element.tag == "road" and element.get("name") == "junction_paving":
            continue
        canonical = ET.canonicalize(ET.tostring(element, encoding="unicode"), strip_text=True)
        result[f"{element.tag}:{element.get('id', str(i))}"] = _sha(canonical.encode())
    return result


def _counts(root):
    return {"roads": len(root.findall("road")),
            "paving_roads": sum(r.get("name") == "junction_paving" for r in root.findall("road")),
            "planview_primitives": len(root.findall("road/planView/geometry")),
            "width_records": len(root.findall("road/lanes/laneSection/*/lane/width")),
            "lane_sections": len(root.findall("road/lanes/laneSection"))}


def intervals(geometry, point, normal):
    cut = geometry.intersection(LineString([point-100*normal, point+100*normal]))
    result = []
    for line in getattr(cut, "geoms", [cut]):
        if line.geom_type == "LineString" and not line.is_empty:
            t = (np.asarray(line.coords)-point) @ normal
            result.append([float(t.min()), float(t.max())])
    return sorted(result)


def check_preserved_gap(paving, source_tail, base, spec, *, station=4.839):
    """Check the pre-registered source gap, including its full interior."""
    axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    normal = np.array([-axis[1], axis[0]])
    point = np.array(spec["pose"][:2]) + station*axis
    tails, junction = intervals(source_tail, point, normal), intervals(base, point, normal)
    if len(tails) != 1 or len(junction) != 1 or tails[0][1] >= junction[0][0]:
        raise PartitionRejected("pre-registered-gap-shape-changed")
    lo, hi = tails[0][1], junction[0][0]
    gap = LineString([point+lo*normal, point+hi*normal])
    covered = paving.intersection(gap)
    covered_length = float(covered.length)
    return {"station_m": station, "interval_m": [lo, hi], "length_m": hi-lo,
            "written_paving_in_gap_m": covered_length,
            "gap_preserved": covered_length <= 1e-7,
            "numerical_length_epsilon_m": 1e-7}


def audit_support(actual, support):
    """Use existing source-surface cover and sliver tolerances, unmodified."""
    outside = actual.difference(support.buffer(E.COVER_TOL_M))
    return {"outside_existing_support_tolerance_m2": float(outside.area),
            "source_cover_tolerance_m": E.COVER_TOL_M,
            "numerical_sliver_area_m2": E.SLIVER_M2,
            "supported": outside.area <= E.SLIVER_M2}


def apply_partition(root, source, junction, project, decisions, snapshot, intent):
    """Return an isolated, written-ready XML root and complete local evidence.

    The caller must serialize, read back, and run ``audit_written``. This
    function's return alone is not an accepted candidate.
    """
    from mapforge.adapters.opendrive import writer as W
    from mapforge.ops.map_to_xodr import _mouth_apron_axes

    if intent != make_intent(snapshot) or junction.pid != JUNCTION_ID:
        raise PartitionRejected("intent-not-registered-or-not-confirmed")
    before = unchanged_subtrees(root)
    mouths = JS.written_mouth_specs(root, decisions)
    targets = [m for m in mouths if m["enter_link"] == ENTER_LINK
               and m.get("leave_links") == [LEAVE_LINK]]
    if len(targets) != 1:
        raise PartitionRejected("source-leg-identity-ambiguous")
    target = targets[0]
    base, pieces, _, summary = JS.source_tail_regions(source, junction, project, mouths)
    cover = base.buffer(E.COVER_TOL_M)
    tails, tail_issues = E._bridge_tails(base, cover, pieces, mouths)
    medians, median_issues, inside = E._median_tails(base, cover, tails, mouths)
    if summary["issues"] or tail_issues or median_issues:
        raise PartitionRejected("original-source-surface-guard-failed")
    rid = target["road_id"]
    source_lanes = {p["source_lane"] for p in pieces if p["road_id"] == rid}
    if source_lanes != LANES or rid in inside:
        raise PartitionRejected("pre-registered-source-scope-changed")
    selected = [("restricted", x["geometry"]) for x in tails if x["road_id"] == rid]
    selected += [("median", x["geometry"]) for x in medians if x["road_id"] == rid]
    if [kind for kind, _ in selected] != ["restricted", "restricted", "median"]:
        raise PartitionRejected("pre-registered-partition-changed")
    if any(g.geom_type != "Polygon" or g.interiors or not g.is_valid for _, g in selected):
        raise PartitionRejected("partition-has-island-or-invalid-source")

    result = copy.deepcopy(root)
    doc = W.XodrDoc("workbench-source-partition-probe")
    retained = [r for r in result.findall("road") if r.get("name") != "junction_paving"]
    for road in retained:
        doc.add_road(W.Road(int(road.get("id"))))
    original_count = len(doc.roads)
    others = [m for m in mouths if m is not target]
    stats = E.append_source_paving(doc, source, junction, project, others,
                                   _mouth_apron_axes(mouths) or [None])
    if any(stats[k] for k in ("mouth_tail_source_issues", "mouth_tail_bridge_issues", "median_tail_issues")):
        raise PartitionRejected("unmodified-family-guard-failed")
    used = {r.road_id for r in doc.roads}
    ids = iter(i for i in range(50, 100) if i not in used)
    partition_rows = []
    for index, (kind, geometry) in enumerate(selected):
        new_id = next(ids)
        # Keep the same cross-section guard and same writer; only membership
        # differs. No caps are added here: they are subsets of separately
        # supported source base, never permission to span an unsupported gap.
        merged = E._append_surface_family(doc, geometry,
            geometry if kind == "median" else LineString(), target, new_id, 1,
            sorted(source_lanes), cover)
        partition_rows.append({"road_id": str(new_id), "kind": kind,
            "source_geometry_sha256": _sha(geometry.wkb), "source_area_m2": float(geometry.area),
            "covered_gap_stations": merged})
        for section in doc.roads[-1].sections:
            for lane in section.left + section.right:
                lane.provenance.update({"workbench_intent": INTENT_TYPE,
                    "partition_index": index, "source_snapshot_id": SNAPSHOT_ID,
                    "source_content_hash": SNAPSHOT_CONTENT_HASH,
                    "source_junction_id": JUNCTION_ID,
                    "partition_role": kind, "status": "APPROXIMATED"})

    old_ids = [r.get("id") for r in result.findall("road") if r.get("name") == "junction_paving"]
    for road in list(result.findall("road")):
        if road.get("name") == "junction_paving":
            result.remove(road)
    junction_el = result.find("junction")
    insert_at = list(result).index(junction_el) if junction_el is not None else len(result)
    for road in doc.roads[original_count:]:
        holder = ET.Element("OpenDRIVE")
        doc._road_el(holder, road)
        result.insert(insert_at, holder.find("road"))
        insert_at += 1
    if unchanged_subtrees(result) != before:
        raise PartitionRejected("non-paving-subtree-changed")
    new_ids = [r.get("id") for r in result.findall("road") if r.get("name") == "junction_paving"]
    evidence = {"schema": "mapforge/workbench-source-partition/v1",
        "status": "WRITTEN_SUPPORT_AUDIT_REQUIRED", "intent": intent,
        "non_paving_subtrees_before": before,
        "non_paving_subtrees_after": unchanged_subtrees(result),
        "non_paving_unchanged": True,
        "complexity_before": _counts(root), "complexity_after": _counts(result),
        "paving_id_diff": {"previous_partial_ids": old_ids, "candidate_ids": new_ids,
            "removed": sorted(set(old_ids)-set(new_ids)), "added": sorted(set(new_ids)-set(old_ids)),
            "reused": sorted(set(old_ids)&set(new_ids))},
        "partitions": partition_rows,
        "source_statuses": {"originals": "PASSTHROUGH", "paving": "APPROXIMATED",
            "gap": "PASSTHROUGH", "invented_driving_or_topology": False},
        "unchanged_default_other_families": stats,
        "production_crs_verified": False}
    # Geometry is in-process only, not serializable external intent data.
    context = {"base": base, "tails": unary_union([g for kind,g in selected if kind == "restricted"]),
        "support": unary_union([base, *[p["geometry"] for p in pieces],
                                 *[p["geometry"] for p in medians]]),
        "selected": selected, "spec": target}
    return result, evidence, context


def audit_written(root, evidence, context):
    from mapforge.validate.smoothness import road_surface_polygon
    if unchanged_subtrees(root) != evidence["non_paving_subtrees_before"]:
        raise PartitionRejected("serialized-non-paving-subtree-changed")
    rows, target_shapes = [], []
    # Source lane tails begin 0.75 m before the mouth. Existing physical-road
    # surfaces also support the writer's established overlap on that side.
    physical = root.find(f"road[@id='{context['spec']['road_id']}']")
    support = unary_union([context["support"], road_surface_polygon(physical, ds=.05)])
    for partition in evidence["partitions"]:
        road = root.find(f"road[@id='{partition['road_id']}']")
        actual = road_surface_polygon(road, ds=.02)
        target_shapes.append(actual)
        rows.append({"road_id":partition["road_id"], "actual_area_m2":float(actual.area),
                     **audit_support(actual, support)})
    all_paving = unary_union([road_surface_polygon(r, ds=.02) for r in root.findall("road")
                             if r.get("name") == "junction_paving"])
    gap = check_preserved_gap(all_paving, context["tails"], context["base"], context["spec"])
    target_gap = check_preserved_gap(unary_union(target_shapes), context["tails"],
                                     context["base"], context["spec"])
    passed = all(r["supported"] for r in rows) and gap["gap_preserved"]
    return {"status": "PASS" if passed else "FAIL", "candidate_accepted": passed,
        "serialized_non_paving_unchanged": True, "partition_support": rows,
        "full_paving_gap": gap, "partition_paving_gap": target_gap,
        "sampling_step_m": .02,
        "limitation": "sampled actual XML surface; not a proof of production CRS or whole-map acceptance"}
