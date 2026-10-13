"""SHP outer-edge boundary deviation per contiguous run of records (SHP T2 from 0.11-draft, user decision 2026-10-13).

boundary_inside pairs each source lane on a road with its SHP boundary once: the lane's included outer-edge records
(shp_boundary_fidelity._target_outer_edges, one per laneSection, without the untrusted and unsupported ones) are stacked
into one polyline. Where records in between are left out, that polyline crosses the gap on a straight chord that is not
in the file, and the source boundary along the gap is measured against the chord. Generalization 0627 road 10, source
lane 40496 (left outermost, s 327-461): included records at s 330.1-338.6, 339.1-339.6 and 460.2-461.2, six untrusted
ones in between; 1.948 m with 184 of 288 samples over 0.35 m, while the written edge lies at most 0.186 m off its source
section by section. 0412 road 12, source lane 20063 (records at s 9.1-9.6 and 55.4-60.0): 0.864 m.

Here a lane's included records are split where the next one starts more than GAP_M from where the previous one ends,
and each run is paired with the source on its own: the same source boundary as boundary_inside (the candidate nearest in
median to all of the lane's included records on the road), clipped to the run's support, distances counted where the
nearest point lies inside the other line, both directions pooled over the file and the worse one reported. Records
that meet (consecutive laneSections share their joint; 0.5 m sampling) are stacked as before, so a lane without a gap
scores as in boundary_inside, and the excluded records stay out of both.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

from mapforge.validate import shp_boundary_fidelity as B

GAP_M = 1.0          # consecutive records further apart than this are paired separately
UNSUPPORTED = {"source-extension", "source-support-unavailable", "source-support-too-short"}


def runs(parts, gap_m=GAP_M):
    """The records (point arrays, in road order) stacked into runs, a new run wherever a record starts more than gap_m
    from where the previous one ends."""
    out = [[parts[0]]]
    for p in parts[1:]:
        if float(np.linalg.norm(p[0] - out[-1][-1][-1])) > gap_m:
            out.append([p])
        else:
            out[-1].append(p)
    return [np.vstack(r) for r in out]


def audit(xodr, shp_dir, src=None, profile="ibd-smarteditor-v1", lanes=None):
    """{"boundary_inside_runs_p95_m", "boundary_inside_runs_max_m", "boundary_inside_runs_split"} (the last: source
    lanes on a road paired in more than one run). ``lanes`` (a list) receives one row per source lane on a road."""
    root = ET.parse(str(xodr)).getroot()
    lat0, lon0 = B._origin(root)
    if src is None:
        from mapforge.adapters.shp.profile_source import ProfileSource
        src = ProfileSource(str(shp_dir), profile)
    s2t, t2s, split = [], [], 0
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        grouped = defaultdict(list)
        for record in B._target_outer_edges(road):
            # the same exclusions as boundary_inside and shp_boundary_fidelity.evaluate_shp_outer_edges
            if record.get("boundary_evidence_trusted") is False:
                continue
            if record.get("exclusion_code") in UNSUPPORTED and record.get("boundary_evidence_trusted") is not True:
                continue
            grouped[record["source_lane_id"]].append(record["points"])
        for source_id, parts in grouped.items():
            candidates = [B._densify(B._project(g, lat0, lon0)) for g in src.lane_boundary_geometries(source_id)]
            candidates = [g for g in candidates if len(g) >= 2]
            if not candidates:
                continue
            whole = np.vstack(parts)
            source = min(candidates, key=lambda g: float(np.median(B._point_polyline_distance(whole, g)[0])))
            pieces = runs(parts)
            split += len(pieces) > 1
            values = []
            for target in pieces:
                clipped = B._clip_source_to_target_support(source, target)
                a, source_inside = B._point_polyline_distance(clipped, target)
                b, target_inside = B._point_polyline_distance(target, clipped)
                s2t.extend(a[source_inside].tolist())
                t2s.extend(b[target_inside].tolist())
                values += a[source_inside].tolist() + b[target_inside].tolist()
            if lanes is not None:
                lanes.append({"road": road.get("id"), "source_lane_id": source_id, "records": len(parts),
                              "runs": len(pieces), "max_m": max(values, default=None)})
    if not s2t or not t2s:
        return {"boundary_inside_runs_p95_m": None, "boundary_inside_runs_max_m": None,
                "boundary_inside_runs_split": split}
    return {"boundary_inside_runs_p95_m": float(max(np.percentile(s2t, 95), np.percentile(t2s, 95))),
            "boundary_inside_runs_max_m": float(max(max(s2t), max(t2s))),
            "boundary_inside_runs_split": split}
