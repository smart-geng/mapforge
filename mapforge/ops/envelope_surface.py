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
A median written narrower than 5 cm at the mouth (closing there) is no median, as in the
converter's median tails (2026-10-08: three unseen junctions failed on such 0.1-2 mm medians),
and a median whose tail lies inside the source junction polygon needs no island: the polygon
paving covers it, as it covers any gap inside it.
Lane tails and median islands are bridged to the polygon as the converter does; where its end cap
check fails at a few points only, those points may pass the polygon edge by COVER_TOL_M or face a
notch of the polygon (a tail corner beside a curb return), as long as most of the cap reaches the
polygon within the budget and the bridged tail joins it without enclosing a hole (2026-10-08: four
unseen junctions failed on single points 1-4 cm beside or 4.8-7.6 m before the polygon).
Any other gap still raises, as before. Tails, median islands, caps and the surface lanes are
the existing builder's (imported, not modified).
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString
from shapely.ops import unary_union

from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import orient

from mapforge.ops.junction_surface import (
    _polygons, directional_sweep, source_tail_regions, written_mouth_specs,
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
# junction_surface.source_median_tails continues only medians at least this wide at the mouth (its own 0.05 m)
MEDIAN_MIN_M = 0.05
# where the converter's end-cap check fails at some points, at least this share of the cap must reach the polygon
END_CAP_REACHED = 0.5


def _bridge_one(base, cover, polygon, direction, max_sweep_m=1.5, overlap_m=.25):
    """junction_surface.bridge_source_tails for one tail polygon; returns (bridge dict, None) or (None, issue).
    Identical to the converter's where every point of the end cap reaches the polygon; otherwise the rule above."""
    if polygon.interiors:
        return None, {"reason": "source-tail-has-island"}
    # one corner touching the polygon does not close the whole end cap: rays from every forward-facing edge point
    forward_gaps, misses, points = [], 0, 0
    coords = np.asarray(orient(polygon, sign=1).exterior.coords)
    for a, b in zip(coords[:-1], coords[1:]):
        edge = b - a
        edge_len = np.linalg.norm(edge)
        if edge_len < 1e-8 or np.dot([edge[1], -edge[0]], direction) / edge_len < .5:
            continue
        for w in np.linspace(0., 1., max(2, int(edge_len / .25) + 2)):
            point = a + w * edge
            points += 1
            if base.covers(Point(point)):
                forward_gaps.append(0.)
                continue
            ray = LineString([point, point + direction * max_sweep_m])
            hit = base.intersection(ray)
            if hit.is_empty:
                hit = cover.intersection(ray)          # passes the polygon edge within COVER_TOL_M
            if hit.is_empty:
                misses += 1
            else:
                forward_gaps.append(float(Point(point).distance(hit)))
    gap = float(polygon.distance(base))
    if not forward_gaps or misses > (1. - END_CAP_REACHED) * points:
        return None, {"reason": "end-cap-outside-repair-budget", "gap_m": gap}
    shift = 0.
    if max(forward_gaps) > 1e-6:
        if directional_sweep(polygon, direction, max_sweep_m).intersection(base).area < .05:
            return None, {"reason": "gap-outside-repair-budget", "gap_m": gap}
        shift = min(max_sweep_m, max(forward_gaps) + overlap_m)
    patched = directional_sweep(polygon, direction, shift) if shift else polygon
    result = {"geometry": patched, "axis": direction, "sweep_m": shift,
              "added_area_m2": float(patched.area - polygon.area), "gap_m": gap}
    if misses:
        joined = unary_union([patched, base])
        if joined.geom_type != "Polygon":
            return None, {"reason": "end-cap-not-joined", "gap_m": gap, "end_cap_misses": misses}
        if len(joined.interiors) > len(base.interiors):
            return None, {"reason": "end-cap-encloses-gap", "gap_m": gap, "end_cap_misses": misses}
        result["end_cap_misses"] = misses
        result["end_cap_points"] = points
    return result, None


def _bridge_tails(base, cover, pieces, mouths):
    """junction_surface.bridge_source_tails with the end-cap rule of _bridge_one."""
    result, issues = [], []
    for spec in mouths:
        shape = unary_union([p["geometry"] for p in pieces if p["road_id"] == spec["road_id"]])
        direction = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        for polygon in _polygons(shape):
            bridged, issue = _bridge_one(base, cover, polygon, direction)
            if issue is not None:
                issues.append({"road_id": spec["road_id"], **issue})
            else:
                result.append({"road_id": spec["road_id"], **bridged})
    return result, issues


def _median_tails(base, cover, tails, mouths):
    """junction_surface.source_median_tails (continues only an existing median between the measured tails on its
    two sides, never across a new junction or a hole of its own), bridged with the end-cap rule of _bridge_one.
    Returns (islands, issues, roads whose median tail lies inside the junction polygon: nothing to continue)."""
    result, issues, inside = [], [], set()
    for spec in mouths:
        interval = spec.get("median_interval")
        if interval is None or interval[1] - interval[0] < MEDIAN_MIN_M:
            continue
        origin = np.asarray(spec["pose"][:2], float)
        axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        normal = np.array([-axis[1], axis[0]])
        geometry = unary_union([t["geometry"] for t in tails if t["road_id"] == spec["road_id"]])
        if geometry.is_empty:
            continue
        max_s = max(float(((np.asarray(p.exterior.coords) - origin) @ axis).max()) for p in _polygons(geometry))
        previous = sum(interval) / 2
        rows = []
        for s in np.linspace(-.5, max_s, max(3, int((max_s + .5) / .25) + 2)):
            point = origin + s * axis
            cut = geometry.intersection(LineString([point - 100 * normal, point + 100 * normal]))
            ranges = []
            for line in getattr(cut, "geoms", [cut]):
                if line.geom_type != "LineString" or line.is_empty:
                    continue
                v = (np.asarray(line.coords) - point) @ normal
                ranges.append((float(v.min()), float(v.max())))
            ranges.sort()
            gaps = [(a[1], b[0]) for a, b in zip(ranges[:-1], ranges[1:])
                    if b[0] - a[1] > .01 and a[1] - .5 <= previous <= b[0] + .5]
            if len(gaps) != 1:
                if not rows and s <= 1.:
                    continue
                if not rows:
                    issues.append({"road_id": spec["road_id"], "reason": "median-gap-ambiguous",
                                   "s": float(s), "expected": interval, "ranges": ranges})
                break
            lo, hi = gaps[0]
            if not rows and (abs(lo - interval[0]) > .5 or abs(hi - interval[1]) > .5):
                issues.append({"road_id": spec["road_id"], "reason": "median-mouth-source-mismatch"})
                break
            if not rows and s > -.49:
                anchor = origin - .5 * axis
                rows.append((anchor + normal * interval[0], anchor + normal * interval[1]))
            rows.append((point + normal * lo, point + normal * hi))
            previous = (lo + hi) / 2
            if base.covers(LineString([rows[-1][0], rows[-1][1]])):
                break
        if len(rows) < 2:
            issues.append({"road_id": spec["road_id"], "reason": "median-insufficient-boundary-support"})
            continue
        polygon = Polygon(np.vstack([[x[0] for x in rows], [x[1] for x in rows[::-1]]]))
        median_parts, join_issues = _bridge_tails(base, cover, [{"road_id": spec["road_id"], "geometry": polygon}],
                                                  [spec])
        if join_issues:
            issues.extend(join_issues)
            continue
        outside = median_parts[0]["geometry"].difference(base)
        if outside.area <= .05:
            inside.add(spec["road_id"])      # the polygon paving covers it, as any gap it covers (_covered)
        for part in _polygons(outside):
            if not part.interiors and part.area > .05:
                result.append({"road_id": spec["road_id"], "geometry": part, "axis": axis})
    return result, issues, inside


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
    cover = base.buffer(COVER_TOL_M)
    tails, issues = _bridge_tails(base, cover, pieces, mouths)
    medians, median_issues, median_inside = _median_tails(base, cover, tails, mouths)
    stats = {"mouth_tail_source_components": summary["components"],
             "mouth_tail_source_issues": summary["issues"], "mouth_tail_bridge_issues": issues,
             "mouth_tail_bridges": [{k: v for k, v in p.items() if k not in {"geometry", "axis"}} for p in tails],
             "median_tail_issues": median_issues,
             "median_tail_areas_m2": [p["geometry"].area for p in medians],
             "surface_family_parts": [], "covered_gap_stations": 0, "filled_slivers": 0}
    if median_inside:
        stats["median_tails_inside_polygon"] = sorted(median_inside)
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
        interval = spec.get("median_interval")
        # a median written narrower than MEDIAN_MIN_M at the mouth (a median closing there) has no island to continue:
        # junction_surface.source_median_tails skips it, so it is no median here either; nor is one whose tail lies
        # inside the source junction polygon, which paves it (2026-10-08)
        if (interval is not None and interval[1] - interval[0] >= MEDIAN_MIN_M and not islands
                and spec["road_id"] not in median_inside):
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
