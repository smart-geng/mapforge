"""Junction paving holes on record-breakpoint outlines, and holes that coincide with bound source voids.

Two user decisions (2026-10-09), graded by T1 from policy 0.9-draft (cutoff unchanged):

* Sampling. smoothness.surface_continuity counts holes in the union of junction_paving outlines sampled every
  0.1 m plus laneSection starts (``paving_holes_gt1cm2``; its cutoff is in fact 0.01 m2 = 100 cm2). Between samples
  the outline is a chord, so a width (or border) or laneOffset record that starts off the grid leaves a sliver: 0621 showed a
  0.022 m2 hole entirely covered by the written surface. Here the grid also holds every planView geometry, laneOffset
  and width record start; on 0621 the union area then equals the exact XML surface (666.6663 m2) and only the real
  9.741 m2 gap is left. smoothness.road_surface_polygon itself is not changed (the workbench builds surfaces with it).
* Source voids. A hole that coincides with a source hole bound by the source-surface reconstruction evidence
  (``source_support.original_holes``: WKB with SHA256 and area) is listed apart and not counted: the reconstruction
  keeps source gaps and isolation seams instead of filling them. Coinciding is two-sided (the symmetric difference
  is at most MATCH_FRACTION of that source hole), so a larger recorded hole cannot excuse a smaller written one.
  Without such evidence (the default pipeline) every hole counts.

``paving_holes_gt1cm2`` and the other surface_continuity figures are still reported unchanged.
"""
from __future__ import annotations

import hashlib

import numpy as np

HOLE_MIN_M2 = 0.01        # the historical cutoff of paving_holes_gt1cm2 (100 cm2), kept
MATCH_FRACTION = 1e-3     # a hole and a source hole coincide when their symmetric difference is this small
OUTLINE_DS_M = 0.1


def road_outline(road, ds: float = OUTLINE_DS_M):
    """Outer outline of one road like smoothness.road_surface_polygon, with every record breakpoint on the grid."""
    from shapely.geometry import Polygon
    from mapforge.validate.smoothness import cross_edges_at, sample_road_ref

    pts, ss, hh = sample_road_ref(road, ds)
    if len(ss) < 2:
        return Polygon()
    length = float(ss[-1])
    grid = set(np.linspace(0.0, length, max(2, int(length / ds) + 2)).tolist())
    grid.update(float(x.get("s")) for x in road.findall("lanes/laneSection"))
    grid.update(float(x.get("s")) for x in road.findall("planView/geometry"))
    grid.update(float(x.get("s")) for x in road.findall("lanes/laneOffset"))
    for sec in road.findall("lanes/laneSection"):
        start = float(sec.get("s"))
        grid.update(start + float(w.get("sOffset")) for w in sec.findall(".//width") + sec.findall(".//border"))
    u = np.asarray(sorted(x for x in grid if 0.0 <= x <= length), float)
    p = np.column_stack([np.interp(u, ss, pts[:, 0]), np.interp(u, ss, pts[:, 1])])
    heading = np.interp(u, ss, hh)
    normal = np.column_stack([-np.sin(heading), np.cos(heading)])
    outer = [cross_edges_at(road, min(float(s), length - 1e-7)) for s in u]
    left = p + np.asarray([x[0] for x in outer], float)[:, None] * normal
    right = p + np.asarray([x[-1] for x in outer], float)[:, None] * normal
    return Polygon(np.vstack([left, right[::-1]])).buffer(0)


def paving_holes(root) -> list:
    """Holes larger than HOLE_MIN_M2 in the union of the junction_paving outlines."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    outlines = [road_outline(r) for r in root.findall("road") if r.get("name") == "junction_paving"]
    outlines = [g for g in outlines if not g.is_empty]
    if not outlines:
        return []
    union = unary_union(outlines)
    holes = [Polygon(ring) for g in getattr(union, "geoms", [union]) for ring in getattr(g, "interiors", [])]
    return sorted((h for h in holes if h.area > HOLE_MIN_M2), key=lambda h: (-h.area, h.bounds))


def source_holes(evidence: dict | None):
    """(source hole polygons, evidence state): 'none', 'bound', or 'invalid' when the record fails its checks."""
    if not evidence:
        return [], "none"
    from shapely import wkb
    from shapely.errors import ShapelyError
    try:
        record = evidence["source_support"]["original_holes"]
        raw = bytes.fromhex(record["wkb_hex"])
        if hashlib.sha256(raw).hexdigest() != record["sha256"]:
            return [], "invalid"
        geometry = wkb.loads(raw)
        if (not geometry.is_valid or abs(float(record["area_m2"]) - geometry.area) > 1e-9 * max(1.0, geometry.area)
                or geometry.geom_type not in {"Polygon", "MultiPolygon", "GeometryCollection"}):
            return [], "invalid"
    except (KeyError, TypeError, ValueError, ShapelyError):
        return [], "invalid"
    parts = [g for g in getattr(geometry, "geoms", [geometry]) if g.geom_type == "Polygon" and g.area > HOLE_MIN_M2]
    return parts, "bound"


def audit(root, evidence: dict | None = None) -> dict:
    """Hole counts on breakpoint outlines; holes coinciding with bound source holes are listed apart."""
    holes = paving_holes(root)
    sources, state = source_holes(evidence)
    voids, counted = [], []
    for hole in holes:
        match = next((s for s in sources
                      if hole.intersects(s) and hole.symmetric_difference(s).area <= MATCH_FRACTION * s.area), None)
        (voids if match is not None else counted).append(hole)
    return {"paving_holes_resampled": len(holes),
            "paving_holes_source_void": len(voids),
            "paving_holes_counted": len(counted),
            "paving_hole_counted_area_max_m2": max((h.area for h in counted), default=0.0),
            "paving_source_void_area_m2": float(sum(h.area for h in voids)),
            "paving_source_void_evidence": state}
