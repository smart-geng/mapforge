"""SHP outer-edge boundary deviation on the common extent only (scoreboard diagnostic, 2026-10-04).

shp_boundary_fidelity compares each ordinary road's outer edges with the SHP boundary of the same source lane.
It clips the source to the target's extent but keeps one more source sample beyond each target end, and it
counts every source point within 1.5 m of the target, also beyond the target's ends. Since the SHP mouths moved
3 m before the real lane ends (shp_mouth_envelope, 2026-10-03), the source boundary goes on for those 3 m past
every road end, so each mouth adds 0.5-0.75 m "deviations" that are the boundary's continuation, not a misplaced
edge (shp-node16: every one of 26 samples above 0.35 m lies beyond a target end). This module reports the same
pairing on the samples whose nearest point lies inside the other line only; extents stay with G8 (coverage,
endpoints). Report only: the T2 gate keeps shp_boundary_fidelity.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

from mapforge.validate import shp_boundary_fidelity as B


def audit(xodr, shp_dir, src=None, profile="ibd-smarteditor-v1"):
    """{"boundary_inside_p95_m", "boundary_inside_max_m"}: both directions pooled, worst direction reported."""
    root = ET.parse(str(xodr)).getroot()
    lat0, lon0 = B._origin(root)
    if src is None:
        from mapforge.adapters.shp.profile_source import ProfileSource
        src = ProfileSource(str(shp_dir), profile)
    s2t, t2s = [], []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        grouped = defaultdict(list)
        for record in B._target_outer_edges(road):
            # the same exclusions as shp_boundary_fidelity.evaluate_shp_outer_edges
            if record.get("boundary_evidence_trusted") is False:
                continue
            if (record.get("exclusion_code") in {"source-extension", "source-support-unavailable",
                                                 "source-support-too-short"}
                    and record.get("boundary_evidence_trusted") is not True):
                continue
            grouped[record["source_lane_id"]].append(record["points"])
        for source_id, parts in grouped.items():
            target = np.vstack(parts)
            candidates = [B._densify(B._project(g, lat0, lon0)) for g in src.lane_boundary_geometries(source_id)]
            candidates = [g for g in candidates if len(g) >= 2]
            if not candidates:
                continue
            source = min(candidates, key=lambda g: float(np.median(B._point_polyline_distance(target, g)[0])))
            source = B._clip_source_to_target_support(source, target)
            a, source_inside = B._point_polyline_distance(source, target)
            b, target_inside = B._point_polyline_distance(target, source)
            s2t.extend(a[source_inside].tolist())
            t2s.extend(b[target_inside].tolist())
    if not s2t or not t2s:
        return {"boundary_inside_p95_m": None, "boundary_inside_max_m": None}
    return {"boundary_inside_p95_m": float(max(np.percentile(s2t, 95), np.percentile(t2s, 95))),
            "boundary_inside_max_m": float(max(max(s2t), max(t2s)))}
