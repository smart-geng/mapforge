"""Junction surface after moving the mouths: gap-aware rebuild of the source paving.

With the source-envelope mouth candidate (mapforge.ops.shp_mouth_envelope), the area between
each new mouth and the source INTERSECTION polygon is covered by connectors plus a paving
"family" built from the real lane tails (mapforge.ops.junction_surface, unchanged). That
builder refuses any lateral gap inside a family that is not the written median, which blocked
3 of the 7 Jinfeng junctions for two reasons, neither of them a real hole:

* gap inside the source junction polygon: lane tails reach 0.75 m into the polygon (end-cap
  overlap) while the median island stops at the polygon edge, so in that strip the cross-
  section shows a gap that the polygon paving already covers;
* separate carriageways: two parallel exit groups with a ~1 m guardrail strip between them
  along the whole tail (node13 south). That strip is not road and must stay unpaved.

Here a family is split into its connected parts (each paved on its own, separators stay
open), and a cross-section gap counts as covered when the source junction polygon covers it.
Any other gap still raises, as before. Tails, median islands, caps and the surface lanes are
the existing builder's (imported, not modified).
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString
from shapely.ops import unary_union

from mapforge.ops.junction_surface import (
    _polygons, bridge_source_tails, directional_sweep, source_median_tails, source_tail_regions,
    written_mouth_specs,
)

CODE = "mapforge.envelope_surface/v1"
COVER_TOL_M = 0.05        # a gap counts as covered if the polygon (grown by this) covers it
# End caps reach this far into the source junction polygon (inside it only, never into a corner).
# The converter's 0.75 m is shorter than the ~1 m cross-section spacing of a surface family, so where
# one carriageway's tail ends earlier than its neighbour's (node3 south: entry lanes at the stop line),
# the interpolated edge cut a 0.19 m sliver at the polygon edge; 2 m puts that step inside the polygon.
CAP_M = 2.0
# Union of lane tails and median islands that share edges leaves zero-area interior rings (float
# noise). The converter then skipped the end caps of the whole family; such slivers are filled.
SLIVER_M2 = 0.01


def _fill_slivers(polygon):
    """Drop interior rings below SLIVER_M2 (numerical slivers, not islands); returns (polygon, filled)."""
    from shapely.geometry import Polygon
    keep = [r for r in polygon.interiors if Polygon(r).area >= SLIVER_M2]
    filled = len(polygon.interiors) - len(keep)
    return (Polygon(polygon.exterior, keep) if filled else polygon), filled


def _covered(cover, p, normal, a, b):
    """Is the cross-section gap [a, b] at p covered by the (slightly grown) junction polygon?"""
    return cover.covers(LineString([p + a * normal, p + b * normal]))


def _append_surface_family(doc, surface, median, spec, road_id, junction_id, source_ids, cover):
    """junction_surface._append_surface_family, with gaps covered by the junction polygon merged."""
    from scipy.interpolate import PchipInterpolator, CubicHermiteSpline
    from mapforge.adapters.opendrive import writer as W
    point = np.asarray(spec["pose"][:2], float)
    axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    normal = np.array([-axis[1], axis[0]])
    coords = np.vstack([np.asarray(p.exterior.coords) for p in _polygons(surface)])
    s = (coords - point) @ axis
    s0, s1 = float(s.min()) - .25, float(s.max()) + .25
    us = np.linspace(0., s1 - s0, max(3, int(s1 - s0) + 2))
    reference = point + s0 * axis
    rows, merged = [], 0
    for u in us:
        p = reference + u * axis
        ray = LineString([p - 100 * normal, p + 100 * normal])

        def intervals(geometry):
            cut = geometry.intersection(ray)
            result = []
            for line in getattr(cut, "geoms", [cut]):
                if line.geom_type == "LineString" and not line.is_empty:
                    t = (np.asarray(line.coords) - p) @ normal
                    result.append((float(t.min()), float(t.max())))
            return sorted(result)

        ranges = intervals(surface)
        if not ranges:
            rows.append(None)
            continue
        joined = [ranges[0]]
        for a, b in ranges[1:]:
            gap = a - joined[-1][1]
            if gap > .02 and not _covered(cover, p, normal, joined[-1][1], a):
                raise ValueError(f"unresolved gap inside source surface family: road={spec['road_id']}, "
                                 f"s={u + s0:.3f}, ranges={ranges}")
            merged += gap > .02
            joined[-1] = (joined[-1][0], max(joined[-1][1], b))
        low, high = joined[0][0], joined[-1][1]
        mid = intervals(median) if not median.is_empty else []
        if len(mid) > 1:
            raise ValueError("multiple islands need separate surface families")
        if mid:
            m0, m1 = max(mid[0][0], low), min(mid[0][1], high)
        else:
            anchor = sum(spec.get("median_interval", [low, low])) / 2
            m0 = m1 = float(np.clip(anchor, low, high))
        rows.append([low, m0, m1, high])
    valid = [i for i, r in enumerate(rows) if r is not None]
    if not valid:
        raise ValueError("empty source surface family")
    for i, r in enumerate(rows):
        if r is None:
            if valid[0] < i < valid[-1]:
                raise ValueError("unsupported interior surface interval")
            rows[i] = rows[min(valid, key=lambda j: abs(i - j))]
    values = np.asarray(rows)
    widths = np.diff(values, axis=1)
    if np.min(widths) < -1e-8:
        raise ValueError("unordered source surface samples")
    slopes = np.column_stack([PchipInterpolator(us, values[:, i]).derivative()(us) for i in range(4)])
    scales = np.ones(len(us))
    for i, h in enumerate(np.diff(us)):
        dw0, dw1 = np.diff(slopes[i]), np.diff(slopes[i + 1])
        for j in range(3):
            if dw0[j] < 0:
                scales[i] = min(scales[i], max(0., 3 * widths[i, j] / (-h * dw0[j])))
            if dw1[j] > 0:
                scales[i + 1] = min(scales[i + 1], max(0., 3 * widths[i + 1, j] / (h * dw1[j])))
    curves = [CubicHermiteSpline(us, values[:, i], slopes[:, i] * scales) for i in range(4)]
    road = W.Road(road_id, name="junction_paving", junction=junction_id)
    road.add_geometry("line", *reference, spec["pose"][2], s1 - s0)
    sec = W.LaneSection(0.)
    for j, kind in enumerate(("restricted", "median", "restricted")):
        lane = W.Lane(j + 1, kind, provenance={"eligibility": "excluded", "role": "paving",
                      "status": "APPROXIMATED", "support_kind": "source-boundary-family",
                      "source_ids": source_ids, "physical_road": spec["road_id"],
                      "travel_direction": "against_s", "exclusion_code": "source-polygon-paving"})
        for i in range(len(us) - 1):
            coeff = curves[j + 1].c[:, i] - curves[j].c[:, i]
            roots = np.roots(np.trim_zeros(np.polyder(coeff), "f"))
            probes = [0., us[i + 1] - us[i]] + [r.real for r in roots
                                                if abs(r.imag) < 1e-9 and 0 < r.real < us[i + 1] - us[i]]
            if min(np.polyval(coeff, probes)) < -1e-7:
                raise ValueError("crossing source surface boundaries")
            lane.add_width(*coeff[::-1], s_offset=float(us[i]))
        sec.left.append(lane)
    for i in range(len(us) - 1):
        road.add_offset(float(us[i]), *curves[0].c[:, i][::-1])
    road.sections.append(sec)
    doc.add_road(road)
    return merged


def append_source_paving(doc, src, junction, project, mouths, axes, junction_id=1):
    """junction_surface.append_source_paving with per-part families and polygon-covered gaps."""
    from mapforge.adapters.opendrive.writer import add_paving_road
    base, pieces, _, summary = source_tail_regions(src, junction, project, mouths)
    tails, issues = bridge_source_tails(base, pieces, mouths)
    medians, median_issues = source_median_tails(base, tails, mouths)
    stats = {"mouth_tail_source_components": summary["components"],
             "mouth_tail_source_issues": summary["issues"], "mouth_tail_bridge_issues": issues,
             "mouth_tail_bridges": [{k: v for k, v in p.items() if k not in {"geometry", "axis"}} for p in tails],
             "median_tail_issues": median_issues,
             "median_tail_areas_m2": [p["geometry"].area for p in medians],
             "surface_family_parts": [], "covered_gap_stations": 0, "filled_slivers": 0}
    cover = base.buffer(COVER_TOL_M)
    regions = [(base, axis, "source-polygon", "restricted") for axis in axes[:2]]
    used = {r.road_id for r in doc.roads}
    ids = iter(i for i in range(50, 100) if i not in used)
    count = 0
    for polygon, axis, support, lane_type in regions:
        for part in _polygons(polygon):
            if part.interiors:
                raise ValueError("paving region with an island cannot be reduced to exterior")
            rid = next(ids)
            if add_paving_road(doc, np.asarray(part.exterior.coords), junction_id,
                               road_id=rid, smooth_profile=True, preferred_axis=axis,
                               lane_type=lane_type, overlap_m=.25,
                               provenance={"eligibility": "excluded", "role": "paving",
                                           "status": "TRANSFORMED", "support_kind": support,
                                           "travel_direction": "with_s",
                                           "exclusion_code": "source-polygon-paving"}):
                count += 1
    for spec in mouths:
        supported = [x["geometry"] for x in tails if x["road_id"] == spec["road_id"]]
        islands = [x["geometry"] for x in medians if x["road_id"] == spec["road_id"]]
        if not supported:
            continue
        if spec.get("median_interval") is not None and not islands:
            stats["mouth_tail_bridge_issues"].append({"road_id": spec["road_id"], "reason": "median-tail-unresolved"})
            continue
        axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        parts = _polygons(unary_union(supported + islands))
        stats["surface_family_parts"].append({"road_id": spec["road_id"], "parts": len(parts)})
        for part in parts:
            part, filled = _fill_slivers(part)
            stats["filled_slivers"] += filled
            own = [g for g in islands if g.intersects(part)]
            caps = [directional_sweep(p, axis, CAP_M).intersection(base) for p in [part] if not p.interiors]
            family = unary_union([part, *caps])
            stats["covered_gap_stations"] += _append_surface_family(
                doc, family, unary_union(own) if own else LineString(), spec, next(ids), junction_id,
                [p["source_lane"] for p in pieces if p["road_id"] == spec["road_id"]], cover)
            count += 1
    stats.update(paving="source-boundary-regions-gap-aware", paving_roads=count)
    return stats


def replace_source_paving(root, src, junction, project, decisions):
    """Same contract as junction_surface.replace_source_paving (ElementTree root, in place)."""
    from mapforge.adapters.opendrive import writer as W
    from mapforge.ops.map_to_xodr import _mouth_apron_axes
    mouths = written_mouth_specs(root, decisions)
    doc = W.XodrDoc("candidate-source-surface")
    retained = [rd for rd in root.findall("road") if rd.get("name") != "junction_paving"]
    for rd in retained:
        doc.add_road(W.Road(int(rd.get("id"))))  # reserve IDs only
    original_count = len(doc.roads)
    stats = append_source_paving(doc, src, junction, project, mouths, _mouth_apron_axes(mouths) or [None])
    for rd in list(root.findall("road")):
        if rd.get("name") == "junction_paving":
            root.remove(rd)
    j = root.find("junction")
    at = list(root).index(j) if j is not None else len(root)
    for rd in doc.roads[original_count:]:
        temporary = ET.Element("OpenDRIVE")
        doc._road_el(temporary, rd)
        root.insert(at, temporary.find("road"))
        at += 1
    stats["source_surface_status"] = ("FAIL" if any(stats[k] for k in
        ("mouth_tail_source_issues", "mouth_tail_bridge_issues", "median_tail_issues")) else "GENERATED")
    stats["surface_builder"] = CODE
    return stats
