"""Fixed-axis source surfaces represented by real-area carrier tracks.

A point fork partitions the *whole real parent cell* at a constant relative
cross-section fraction fixed by the source fork vertex.  The two shares then
continue into the two unchanged branches.  A point merge is the reverse.
Those internal seams are representation-only, not surveyed boundaries.  No
road borrows empty length, changes the source axis, or consults a length gate.

The deliberately bounded topology contract accepts one cell, one binary fork,
one binary merge, or one binary fork and its corresponding binary merge per
Polygon component.  Finite-width openings, vertical steps and more complex
graphs fail explicitly.  An isolated real short component stays short.
"""
from __future__ import annotations

from copy import deepcopy
from itertools import count
import json
import math

import numpy as np
from shapely import wkb
from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.workbench import source_surface_cells as C

SCHEMA = "mapforge/source-surface-tracks/v1"


class SurfaceTrackError(C.SurfaceCellError):
    pass


def _json_hash(value):
    return C._sha(json.dumps(value, sort_keys=True, separators=(",", ":"),
                             ensure_ascii=False, allow_nan=False).encode("utf8"))


def _normalize_adjacent_vertices(geometry, preferred_axis):
    """Coalesce only adjacent vertices with a joint two-axis roundoff witness.

    Each group retains a supplied vertex, selected by lowest original index.
    Every member's s AND t uncertainty intervals must contain that same
    vertex. Common intervals shrink as a group grows; pairwise/transitive
    closeness is insufficient. Rings retain every vertex/edge index, including
    repeated coordinates, and the original geometry remains the audit source.
    """
    polygons = C._polygons(geometry)
    axis = np.asarray(preferred_axis, dtype=float)
    if axis.shape != (2,) or not np.isfinite(axis).all() or np.linalg.norm(axis) == 0:
        raise SurfaceTrackError("explicit-finite-nonzero-axis-required")
    axis = axis/np.linalg.norm(axis)
    normal = np.array([-axis[1], axis[0]])
    origin = np.asarray(polygons[0].exterior.coords[0], dtype=float) if polygons else np.zeros(2)
    mappings, groups_report, normalized_polygons = [], [], []
    maximum_bound = 0.

    def common(members):
        lo = [max(v["intervals"][k][0] for v in members) for k in (0, 1)]
        hi = [min(v["intervals"][k][1] for v in members) for k in (0, 1)]
        candidates = [v for v in members if all(lo[k] <= v["local"][k] <= hi[k] for k in (0, 1))]
        if not candidates:
            return None
        return min(candidates, key=lambda v: v["index"]), [[lo[k], hi[k]] for k in (0, 1)]

    for component_index, polygon in enumerate(polygons):
        rings = []
        for ring_index, ring in enumerate([polygon.exterior, *polygon.interiors]):
            vertices = []
            for index, xy in enumerate(list(ring.coords)[:-1]):
                world = np.asarray(xy, dtype=float)
                if world.shape != (2,) or not np.isfinite(world).all():
                    raise SurfaceTrackError("explicit-finite-2d-source-required")
                local = [float((world-origin)@direction) for direction in (axis, normal)]
                error = [C._projection_roundoff(world, origin, direction) for direction in (axis, normal)]
                intervals = [[math.nextafter(value-e, -math.inf), math.nextafter(value+e, math.inf)]
                             for value, e in zip(local, error)]
                vertices.append({"index": index, "world": world, "local": local,
                                 "error": error, "intervals": intervals})
            groups = []
            for vertex in vertices:
                if groups and common([*groups[-1], vertex]) is not None:
                    groups[-1].append(vertex)
                else:
                    groups.append([vertex])
            if len(groups) > 1 and common([*groups[-1], *groups[0]]) is not None:
                groups[0] = [*groups[-1], *groups[0]]
                groups.pop()
            coordinates = [v["world"].copy() for v in vertices]
            for group in groups:
                representative, intersection = common(group)
                changed = []
                for vertex in group:
                    i = vertex["index"]
                    coordinates[i] = representative["world"].copy()
                    different = not np.array_equal(vertex["world"], representative["world"])
                    # Projection uncertainty is derived independently of the
                    # polygon/XML difference later being tested.
                    bound = (math.nextafter(math.hypot(*[
                        vertex["error"][k]+representative["error"][k] for k in (0, 1)]), math.inf)
                        if different else 0.)
                    distance = math.dist(vertex["world"], representative["world"])
                    if distance > bound:
                        raise SurfaceTrackError("vertex-normalization-exceeds-derived-roundoff")
                    maximum_bound = max(maximum_bound, bound)
                    mappings.append({"component_index": component_index, "ring_index": ring_index,
                        "vertex_index": i, "representative_vertex_index": representative["index"],
                        "original_xy": vertex["world"].tolist(), "normalized_xy": coordinates[i].tolist(),
                        "original_local_st_m": vertex["local"], "projection_roundoff_st_m": vertex["error"],
                        "changed": different, "world_shift_m": distance, "derived_world_shift_bound_m": bound})
                    if different:
                        changed.append(i)
                if changed:
                    groups_report.append({"component_index": component_index, "ring_index": ring_index,
                        "vertex_indices": [v["index"] for v in group], "changed_vertex_indices": changed,
                        "representative_vertex_index": representative["index"],
                        "common_projection_roundoff_intervals_st_m": intersection})
            # Keep a now-coincident last source vertex distinct from the ring
            # closure entry so original vertex/edge indices remain traceable.
            rings.append([*coordinates, coordinates[0]])
        normalized_polygons.append(Polygon(rings[0], rings[1:]))
    normalized = (geometry if not groups_report else
                  normalized_polygons[0] if isinstance(geometry, Polygon) else MultiPolygon(normalized_polygons))
    if (not normalized.is_valid or normalized.is_empty != geometry.is_empty
            or any(p.area <= 0 for p in C._polygons(normalized))):
        raise SurfaceTrackError("adjacent-roundoff-normalization-changes-topology")
    if groups_report and (not normalized.difference(geometry.buffer(maximum_bound)).is_empty
            or not geometry.difference(normalized.buffer(maximum_bound)).is_empty):
        raise SurfaceTrackError("adjacent-normalization-exceeds-derived-source-band")
    mappings.sort(key=lambda r: (r["component_index"], r["ring_index"], r["vertex_index"]))
    return normalized, {"rule": "adjacent-two-axis-common-roundoff-with-source-vertex-witness/v1",
        "representative_rule": "lowest-original-vertex-index-in-common-s-and-t-intervals",
        "source_vertex_mapping": mappings, "changed_groups": groups_report,
        "changed_vertex_count": sum(v["changed"] for v in mappings),
        "maximum_derived_source_shift_bound_m": maximum_bound,
        "normalized_geometry_wkb_hex": normalized.wkb.hex(),
        "normalized_geometry_sha256": C._sha(normalized.wkb),
        "original_normalized_symmetric_difference_m2": float(geometry.symmetric_difference(normalized).area),
        "deleted_vertices": 0, "source_holes_preserved": True,
        "geometric_tolerance_used": False}


def _start(cell):
    return cell["pieces"][0]["start_s_m"]


def _end(cell):
    return cell["pieces"][-1]["end_s_m"]


def _limits(cell, end):
    p = cell["pieces"][-1 if end else 0]
    return (p["low1"], p["high1"]) if end else (p["low0"], p["high0"])


def _event(trunk, branches, *, fork, vertices, roundoff):
    """Accept a source vertex point event, never a finite-width opening."""
    station = _end(trunk) if fork else _start(trunk)
    if any((_start(b) if fork else _end(b)) != station for b in branches):
        raise SurfaceTrackError("noncoincident-source-event-stations")
    branches = sorted(branches, key=lambda b: _limits(b, not fork)[0])
    lo, hi = _limits(trunk, fork)
    (a, q0), (q1, b) = [_limits(branch, not fork) for branch in branches]
    residual = max(abs(a-lo), abs(b-hi), abs(q0-q1))
    if residual > roundoff:
        raise SurfaceTrackError("finite-gap-or-step-at-source-event", station_m=station,
                                endpoint_residual_m=residual, arithmetic_roundoff_m=roundoff)
    q = .5*q0 + .5*q1
    if not lo < q < hi:
        raise SurfaceTrackError("positive-source-branch-width-required", station_m=station)
    witnesses = [v for v in vertices if v["component_index"] == trunk["component_index"]
                 and v["normalized_s_m"] == station and abs(v["t_m"]-q) <= roundoff]
    if not witnesses:
        raise SurfaceTrackError("source-event-vertex-witness-required", station_m=station)
    fraction = (q-lo)/(hi-lo)
    event = {"kind": "point-fork" if fork else "point-merge",
             "component_index": trunk["component_index"], "station_m": station,
             "parent_cell_index": trunk["source_cell_index"],
             "branch_cell_indices": [b["source_cell_index"] for b in branches],
             "source_cross_section_m": [lo, q, hi], "partition_fraction": fraction,
             "maximum_endpoint_roundoff_m": residual,
             "source_vertex_refs": [{k: v[k] for k in ("component_index", "ring_index", "vertex_index")}
                                    for v in witnesses],
             "seam_role": "representation-only", "seam_status": "TRANSFORMED",
             "seam_rule": "constant-relative-fraction-of-real-parent-cross-section"}
    return branches, event


def _share(cell, lower, upper, event_index=None):
    pieces = []
    for source in cell["pieces"]:
        p = deepcopy(source)
        for suffix in ("0", "1"):
            lo, hi = source["low"+suffix], source["high"+suffix]
            # Interpolate the width, not two large weighted coordinates. In
            # particular a true zero-width source tip must stay exactly that
            # same point in both shares; independent weighted sums can create
            # an artificial negative width of one ulp at such a tip.
            p["low"+suffix] = lo if lower == 0 else lo+lower*(hi-lo)
            p["high"+suffix] = hi if upper == 1 else lo+upper*(hi-lo)
        seam = {"kind": "representation-only-parent-partition", "event_index": event_index,
                "source_cell_index": cell["source_cell_index"], "status": "TRANSFORMED"}
        p["edges"] = [source["edges"][0] if lower == 0 else {**seam, "fraction": lower},
                      source["edges"][1] if upper == 1 else {**seam, "fraction": upper}]
        pieces.append(p)
    contribution = {"source_cell_index": cell["source_cell_index"],
                    "fraction_range": [lower, upper], "partition_event_index": event_index}
    return pieces, contribution


def _track(parts, component_index, roundoff):
    pieces, contributions = [], []
    for chunk, contribution in parts:
        if pieces:
            before, after = pieces[-1], chunk[0]
            if (before["end_s_m"] != after["start_s_m"]
                    or abs(before["low1"]-after["low0"]) > roundoff
                    or abs(before["high1"]-after["high0"]) > roundoff):
                raise SurfaceTrackError("track-needs-empty-padding-or-lateral-step")
        pieces.extend(chunk)
        contributions.append(contribution)
    for p in pieces:
        w0, w1 = p["high0"]-p["low0"], p["high1"]-p["low1"]
        if p["end_s_m"] <= p["start_s_m"] or min(w0, w1) < 0 or w0+w1 <= 0:
            raise SurfaceTrackError("track-piece-without-positive-real-area")
    return {"component_index": component_index, "pieces": pieces,
            "source_contributions": contributions}


def _plan(source_cells, vertices, topology, roundoff):
    tracks, events = [], []
    components = sorted({c["component_index"] for c in source_cells})
    for component in components:
        nodes = [c for c in source_cells if c["component_index"] == component]
        if any(t["component_index"] == component and t["vertical_boundary_steps"] for t in topology):
            raise SurfaceTrackError("vertical-source-step-not-supported", component_index=component)
        if len(nodes) == 1:
            tracks.append(_track([_share(nodes[0], 0, 1)], component, roundoff))
            continue
        edges = {(a["source_cell_index"], b["source_cell_index"])
                 for a in nodes for b in nodes if _end(a) == _start(b)
                 and min(_limits(a, True)[1], _limits(b, False)[1])
                 > max(_limits(a, True)[0], _limits(b, False)[0])}
        by_id = {c["source_cell_index"]: c for c in nodes}
        pred = {i: {a for a, b in edges if b == i} for i in by_id}
        succ = {i: {b for a, b in edges if a == i} for i in by_id}
        forks = [i for i in by_id if not pred[i] and len(succ[i]) == 2]
        merges = [i for i in by_id if not succ[i] and len(pred[i]) == 2]
        fork = forks[0] if len(forks) == 1 else None
        merge = merges[0] if len(merges) == 1 else None
        if (len(nodes) == 3 and fork is not None
                and all(pred[i] == {fork} and not succ[i] for i in succ[fork])):
            branches, event = _event(by_id[fork], [by_id[i] for i in succ[fork]],
                                     fork=True, vertices=vertices, roundoff=roundoff)
            event_id = len(events); events.append(event); f = event["partition_fraction"]
            for branch, limits in zip(branches, [(0, f), (f, 1)]):
                tracks.append(_track([_share(by_id[fork], *limits, event_id),
                                      _share(branch, 0, 1)], component, roundoff))
        elif (len(nodes) == 3 and merge is not None
                and all(succ[i] == {merge} and not pred[i] for i in pred[merge])):
            branches, event = _event(by_id[merge], [by_id[i] for i in pred[merge]],
                                     fork=False, vertices=vertices, roundoff=roundoff)
            event_id = len(events); events.append(event); f = event["partition_fraction"]
            for branch, limits in zip(branches, [(0, f), (f, 1)]):
                tracks.append(_track([_share(branch, 0, 1),
                                      _share(by_id[merge], *limits, event_id)], component, roundoff))
        elif (len(nodes) == 4 and fork is not None and merge is not None
                and succ[fork] == pred[merge]
                and all(pred[i] == {fork} and succ[i] == {merge} for i in succ[fork])):
            branches, first = _event(by_id[fork], [by_id[i] for i in succ[fork]],
                                     fork=True, vertices=vertices, roundoff=roundoff)
            endings, last = _event(by_id[merge], branches, fork=False,
                                   vertices=vertices, roundoff=roundoff)
            if [c["source_cell_index"] for c in branches] != [c["source_cell_index"] for c in endings]:
                raise SurfaceTrackError("source-branches-change-order")
            first_id = len(events); last_id = first_id+1; events.extend([first, last])
            f, g = first["partition_fraction"], last["partition_fraction"]
            for branch, a, b in zip(branches, [(0, f), (f, 1)], [(0, g), (g, 1)]):
                tracks.append(_track([_share(by_id[fork], *a, first_id), _share(branch, 0, 1),
                                      _share(by_id[merge], *b, last_id)], component, roundoff))
        else:
            raise SurfaceTrackError("unsupported-source-cell-topology", component_index=component,
                                    source_cell_count=len(nodes), source_cell_edges=sorted(edges))
    return tracks, events


def _partition_proof(source_cells, tracks, events):
    # Every original cell must be partitioned over its entire station domain.
    # Exact shared fractions, not an area tolerance, forbid losing a tiny arm.
    coverage = []
    for cell in source_cells:
        owners = [{"track_index": i, **part} for i, track in enumerate(tracks)
                  for part in track["source_contributions"]
                  if part["source_cell_index"] == cell["source_cell_index"]]
        owners.sort(key=lambda part: part["fraction_range"])
        position = 0
        for owner in owners:
            lo, hi = owner["fraction_range"]
            if lo != position or not lo < hi <= 1:
                raise SurfaceTrackError("source-cell-coverage-gap-or-overlap")
            position = hi
        if position != 1:
            raise SurfaceTrackError("source-cell-not-fully-owned")
        coverage.append({"source_cell_index": cell["source_cell_index"], "owners": owners})
    return {"rule": "whole-real-parent-cell-relative-partition/v1", "events": events,
            "source_cell_coverage": coverage,
            "tracks": [{"component_index": t["component_index"],
                        "source_contributions": t["source_contributions"]} for t in tracks],
            "derived_seams_are_source_boundaries": False,
            "endpoint_padding_m": 0.0, "axis_search_performed": False,
            "minimum_length_or_gate_policy_used": False}


def append_polygon(doc, geometry, junction_id, road_ids, *, preferred_axis,
                   lane_type="restricted", provenance=None):
    """Atomically append fixed-axis real-area tracks and return source proof.

    The underlying cell sweep performs source/axis/type/provenance validation.
    Its temporary XML lives only in memory; no old evidence or source is edited.
    """
    normalized, normalization = _normalize_adjacent_vertices(geometry, preferred_axis)
    raw = C.append_polygon(W.XodrDoc("source-track-sweep"), normalized, junction_id, count(),
                           preferred_axis=preferred_axis, lane_type=lane_type, provenance=provenance)
    # Retain the supplied axis rather than normalizing an already-normalized
    # axis again during audit: a second normalization can change binary64 bits.
    # The cell builder above has validated every part of this write contract.
    contract = {"junction_id": junction_id, "lane_type": lane_type,
                "preferred_axis": np.asarray(preferred_axis, dtype=float).tolist(),
                "provenance": json.loads(json.dumps(provenance or {}, allow_nan=False))}
    source_cells = [{"source_cell_index": i, "component_index": row["component_index"],
                     "pieces": deepcopy(row["source_pieces"])} for i, row in enumerate(raw["cells"])]
    roundoff = raw["numeric_arithmetic_roundoff_m"]
    tracks, events = _plan(source_cells, raw["source_vertices"], raw["natural_topology_events"], roundoff)
    partition = _partition_proof(source_cells, tracks, events)
    axis, origin = np.asarray(raw["sweep_axis"]), np.asarray(raw["sweep_origin_xy"])
    normal = np.array([-axis[1], axis[0]])
    used, ids = {r.road_id for r in doc.roads}, iter(road_ids)
    planned_roads, rows, quads = [], [], []
    for index, track in enumerate(tracks):
        try:
            rid = next(ids)
        except StopIteration as exc:
            raise SurfaceTrackError("insufficient-road-ids") from exc
        if not isinstance(rid, int) or isinstance(rid, bool) or rid < 0 or rid in used:
            raise SurfaceTrackError("invalid-or-colliding-road-id", road_id=rid)
        used.add(rid)
        prov = {**contract["provenance"], "eligibility": "excluded", "role": "paving",
                "status": contract["provenance"].get("status", "TRANSFORMED"),
                "support_kind": contract["provenance"].get("support_kind", "source-polygon-tracks"),
                "source_geometry_sha256": C._sha(geometry.wkb), "representation": SCHEMA,
                "source_track_index": index, "source_component_index": track["component_index"],
                "source_contributions": track["source_contributions"],
                "partition_sha256": _json_hash(partition),
                "derived_seams_are_source_boundaries": False,
                "exclusion_code": "source-polygon-paving"}
        road, row, _actual, shapes = C._road_from_cell(track, origin, axis, normal, rid,
                                                     junction_id, lane_type, prov)
        row["source_contributions"] = deepcopy(track["source_contributions"])
        planned_roads.append(road); rows.append(row); quads.extend(shapes)
    planned = unary_union(quads)
    source_band = roundoff+normalization["maximum_derived_source_shift_bound_m"]
    if (not planned.difference(geometry.buffer(source_band)).is_empty
            or not geometry.difference(planned.buffer(source_band)).is_empty):
        raise SurfaceTrackError("track-partition-does-not-cover-original-source",
                                symmetric_difference_m2=float(planned.symmetric_difference(geometry).area))
    max_error = max((row["expected_max_vertex_serialization_error_m"] for row in rows), default=0.)
    spans = [b-a for row in rows for a, b in zip(row["written_record_stations_m"][:-1],
                                                row["written_record_stations_m"][1:])]
    proof = {**raw, "schema": SCHEMA, "write_contract": contract,
             "source_geometry_sha256": C._sha(geometry.wkb), "source_geometry_wkb_hex": geometry.wkb.hex(),
             "source_area_m2": float(geometry.area), "source_numerical_normalization": normalization,
             "road_ids": [row["road_id"] for row in rows], "cells": rows,
             "source_cells": source_cells, "track_partition": partition,
             "track_partition_sha256": _json_hash(partition),
             "expected_max_vertex_serialization_error_m": max_error,
             "numeric_serialization_band_m": source_band+4*max_error,
             "analytic_partition_symmetric_difference_m2": float(planned.symmetric_difference(geometry).area),
             "analytic_track_overlap_area_m2": float(sum(q.area for q in quads)-planned.area),
             "complexity": {**raw["complexity"], "source_cells_before_partition": len(source_cells),
                            "line_roads": len(rows), "lane_sections": len(rows),
                            "width_records": sum(len(r["source_pieces"]) for r in rows),
                            "minimum_width_event_span_m": min(spans, default=None),
                            "minimum_road_length_m": min((r["length_m"] for r in rows), default=None)}}
    for road in planned_roads:
        doc.add_road(road)
    return proof


def written_geometry(root, road_ids):
    return C.written_geometry(root, road_ids)


def audit_written(root, evidence):
    """Check original source, deterministic partition and actual XML together.

    This family check is not a whole-map or product acceptance decision.
    """
    try:
        if evidence.get("schema") != SCHEMA:
            raise SurfaceTrackError("unknown-source-track-evidence")
        source = wkb.loads(bytes.fromhex(evidence["source_geometry_wkb_hex"]))
        if C._sha(source.wkb) != evidence["source_geometry_sha256"]:
            raise SurfaceTrackError("source-geometry-binding-mismatch")
        # Rebuild both the analytic partition and its expected XML from the
        # original source/write contract. Neither a modified XML nor modified
        # row metadata may grant itself a larger serialization tolerance or a
        # new expected hash. This is source consistency, not authentication;
        # the caller must still bind the source/contract to its snapshot.
        contract = evidence["write_contract"]
        ids = evidence["road_ids"]
        if (not isinstance(ids, list) or len(ids) != len(set(ids))
                or any(not isinstance(i, str) or not i.isascii() or not i.isdecimal()
                       or str(int(i)) != i for i in ids)):
            raise SurfaceTrackError("missing-or-duplicate-track-evidence")
        expected = append_polygon(W.XodrDoc("source-track-audit"), source,
            contract["junction_id"], [int(i) for i in ids],
            preferred_axis=contract["preferred_axis"], lane_type=contract["lane_type"],
            provenance=contract["provenance"])
        for key, value in expected.items():
            if key not in evidence or _json_hash(evidence[key]) != _json_hash(value):
                raise SurfaceTrackError("track-evidence-does-not-match-source", field=key)
        report = C.audit_written(root, {**expected, "schema": C.SCHEMA})
        report.update(representation=SCHEMA, source_partition_verified=True,
                      derived_seams_are_source_boundaries=False, whole_map_gates_evaluated=False)
        return report
    except (C.SurfaceCellError, ValueError, KeyError, TypeError, StopIteration) as exc:
        return {"status": "FAIL", "passed": False, "error": str(exc),
                "error_code": getattr(exc, "code", "invalid-source-track-evidence"),
                "representation": SCHEMA, "source_partition_verified": False,
                "whole_map_gates_evaluated": False}
