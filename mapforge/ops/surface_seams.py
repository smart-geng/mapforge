"""Junction paving where envelope_surface fails on a seam that the source itself explains (2026-10-12).

envelope_surface.append_source_paving stays the builder. Only where it fails does this module retry, with two rules
that add nothing the source does not show; any other failure, or a rule whose checks fail, re-raises / returns the
original failure unchanged ("说不清的缝照样报错"). Both rules need the ``road_surface`` object layer (the raw IBD
ROAD_SURFACE polygons; ``traffic_barrier`` is read as evidence only); without it, or without any road surface around
the junction, nothing is retried. The user decided on 2026-10-12 to declare these layers for the Profile, in a
companion file next to it (profiles/shp/<profile>.objects.yaml) rather than in the Profile itself: the workbench
derives its source snapshot identity, every source object ID and its registered compile capabilities from the
Profile's bytes. A layer the Profile itself declares wins over the companion.

1. Median takeover (unseen 0621, road 10; the user decided on 2026-10-12 that this strip is median, not a void).
   The median tracker (envelope_surface._median_tails) follows the gap
   between the leg's own lane tails only. Where the tail band on one side of the median ends inside the junction
   area, the separator often continues between the other tail and the source INTERSECTION polygon: the tracker
   stops, the median island ends early, and the family's end cap (junction-polygon area) then leaves the rest of
   the separator as an unexplained interior gap. Here the tracker runs on the leg's tails together with the
   junction polygon, so the polygon boundary takes over the ended side:
   * the same median identity: it starts at the written median interval (as the tracker) and each 0.25 m row
     continues the previous one on both edges within the tracker's own 0.5 m identity tolerance (no jump to a
     one-sided leftover piece);
   * one gap only, both edges on source geometry (a lane-tail boundary or the INTERSECTION polygon boundary);
   * it must close against the source (the lane tail meets the polygon) before the tail ends; an open strip fails;
   * it is no wider than the written median plus that tolerance;
   * it is not road surface: the raw IBD ROAD_SURFACE polygons overlap it only within the cover tolerance band of
     its outline (the accepted islands of both unseen junctions are ROAD_SURFACE-free too). Traffic barriers lying
     inside are recorded as evidence.
   The island is written as the existing median lane of the family (restricted / median / restricted), never as
   asphalt.
2. Road-surface seam (unseen 0630, road 30). A tail whose end cap does not reach the junction polygon within the
   1.5 m sweep budget at a few points is bridged with that budget (envelope_surface._bridge_one) and rejected when
   the bridged tail encloses a hole with the polygon. The hole is accepted and paved only when the raw IBD
   ROAD_SURFACE polygons cover it (up to the existing 5 cm cover tolerance and 0.01 m2 sliver area), it lies in the
   tail's own forward corridor up to the polygon, and no other leg's tail overlaps it. The sweep budget is not
   raised; the hole is paved because the source says it is road surface.

Lanes a rule added to are marked INFERRED with the rule's evidence; the run's stats name the fallback
(``surface_builder_fallback``) and list the rules (``seam_rules``).
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from mapforge.ops import envelope_surface as E
from mapforge.ops import junction_surface as JS

CODE = "mapforge.surface_seams/v1"
STEP_M = .25              # the median tracker's own station step
IDENTITY_M = .5           # the median tracker's own identity tolerance (mouth anchor, previous mid)
GAP_MIN_M = .01           # the median tracker's own minimum gap
ROAD_SURFACE = "road_surface"         # object layer keys
TRAFFIC_BARRIER = "traffic_barrier"
COMPANION_SUFFIX = ".objects.yaml"    # profiles/shp/<profile>.objects.yaml
SEARCH_DEG = .002         # object layers are read within this many degrees of the junction centre


# ------------------------------------------------------------------ source objects
def companion_path(src):
    """The companion declaration next to the source's Profile file (None if the Profile has no file)."""
    path = (getattr(src, "p", None) or {}).get("_path")
    return Path(path).with_name(Path(path).stem + COMPANION_SUFFIX) if path else None


def object_layers(src):
    """{key: layer spec} of the two object layers: the Profile's own declaration, else the companion's."""
    own = getattr(src, "L", None) or {}
    extra = {}
    companion = companion_path(src)
    if companion is not None and companion.is_file():
        data = yaml.safe_load(companion.read_text(encoding="utf-8-sig")) or {}
        if data.get("extends") != Path(src.p["_path"]).stem:
            raise ValueError(f"{companion.name} extends {data.get('extends')!r}, not {Path(src.p['_path']).stem!r}")
        extra = data.get("layers") or {}
    layers = {}
    for key in (ROAD_SURFACE, TRAFFIC_BARRIER):
        spec = own.get(key) or extra.get(key)
        if not spec:
            continue
        if not isinstance(spec.get("file"), str) or not (spec.get("fields") or {}).get("id"):
            raise ValueError(f"object layer {key} needs file and fields.id")
        layers[key] = spec
    return layers


def has_road_surface(src):
    return ROAD_SURFACE in object_layers(src)


def _source_objects(src, junction, project, key):
    """[(object id, local geometry)] of an object layer around the junction, raw; [] if not declared."""
    from shapely.geometry import shape
    from shapely.ops import transform
    spec = object_layers(src).get(key)
    if not spec:
        return []
    id_field = (spec.get("fields") or {}).get("id")
    lon0, lat0 = float(junction.center[0]), float(junction.center[1])

    def _xy(x, y, z=None):
        p = np.asarray(project(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)])))
        return p[:, 0], p[:, 1]
    out = []
    for record, shp in src._iter(spec):
        if shp is None or not shp.points:
            continue
        b = shp.bbox
        if b[2] < lon0 - SEARCH_DEG or b[0] > lon0 + SEARCH_DEG or b[3] < lat0 - SEARCH_DEG or b[1] > lat0 + SEARCH_DEG:
            continue
        geom = transform(_xy, shape(shp.__geo_interface__))
        if geom.geom_type.endswith("Polygon") and not geom.is_valid:
            geom = geom.buffer(0)
        out.append((str(record.get(id_field, "") if id_field else ""), geom))
    return out


def road_surface(src, junction, project):
    objs = _source_objects(src, junction, project, ROAD_SURFACE)
    return unary_union([g for _, g in objs if g.geom_type.endswith("Polygon")]) if objs else Polygon(), objs


# ------------------------------------------------------------------ rule 2: road-surface seam
def bridge_one(base, cover, polygon, direction, rs, rs_objects, others):
    """envelope_surface._bridge_one, plus the road-surface seam rule for end-cap-encloses-gap."""
    bridged, issue = E._bridge_one(base, cover, polygon, direction)
    if issue is None or issue.get("reason") != "end-cap-encloses-gap" or rs.is_empty:
        return bridged, issue
    # the same sweep envelope_surface used (its result is the budget-capped tail that enclosed the hole)
    forward = []
    from shapely.geometry.polygon import orient
    coords = np.asarray(orient(polygon, sign=1).exterior.coords)
    longest = 0.
    for a, b in zip(coords[:-1], coords[1:]):
        edge = b - a
        el = np.linalg.norm(edge)
        if el < 1e-8 or np.dot([edge[1], -edge[0]], direction) / el < .5:
            continue
        for w in np.linspace(0., 1., max(2, int(el / .25) + 2)):
            p = a + w * edge
            if base.covers(Point(p)):
                forward.append(0.)
                continue
            hit = base.intersection(LineString([p, p + direction * 1.5]))
            if hit.is_empty:
                hit = cover.intersection(LineString([p, p + direction * 1.5]))
            if not hit.is_empty:
                forward.append(float(Point(p).distance(hit)))
            far = base.intersection(LineString([p, p + direction * 10.]))   # diagnostic reach, not a budget
            if not far.is_empty:
                longest = max(longest, float(Point(p).distance(far)))
    shift = min(1.5, max(forward) + .25) if forward and max(forward) > 1e-6 else 0.
    patched = JS.directional_sweep(polygon, direction, shift) if shift else polygon
    joined = unary_union([patched, base])
    if joined.geom_type != "Polygon":
        return None, issue
    base_holes = [Polygon(r) for r in base.interiors]
    holes = [Polygon(r) for r in joined.interiors if not any(Polygon(r).equals(b) for b in base_holes)]
    corridor = JS.directional_sweep(polygon, direction, longest + E.COVER_TOL_M) if longest else polygon
    rows = []
    for hole in holes:
        uncovered = hole.difference(rs.buffer(E.COVER_TOL_M)).area
        outside_corridor = hole.difference(corridor.buffer(E.COVER_TOL_M)).area
        foreign = sum(o.intersection(hole).area for o in others)
        ids = sorted(i for i, g in rs_objects if g.geom_type.endswith("Polygon") and g.intersection(hole).area > 1e-6)
        row = {"area_m2": hole.area, "road_surface_uncovered_m2": uncovered, "outside_forward_corridor_m2": outside_corridor,
               "other_tails_overlap_m2": foreign, "road_surface_objects": ids, "long_ray_max_m": longest}
        rows.append(row)
        if uncovered > E.SLIVER_M2 or outside_corridor > E.SLIVER_M2 or foreign > E.SLIVER_M2:
            return None, {**issue, "road_surface_seam": rows}
    filled = unary_union([patched, *holes])
    if filled.geom_type != "Polygon" or filled.interiors:
        return None, {**issue, "road_surface_seam": rows}
    result = {"geometry": filled, "axis": direction, "sweep_m": shift, "added_area_m2": float(filled.area - polygon.area),
              "gap_m": issue.get("gap_m"), "end_cap_misses": issue.get("end_cap_misses"),
              "road_surface_seam": rows, "status": "INFERRED", "support_kind": "source-road-surface-seam"}
    return result, None


def bridge_tails(base, cover, pieces, mouths, rs, rs_objects):
    """envelope_surface._bridge_tails with bridge_one."""
    result, issues = [], []
    for spec in mouths:
        shape = unary_union([p["geometry"] for p in pieces if p["road_id"] == spec["road_id"]])
        others = [p["geometry"] for p in pieces if p["road_id"] != spec["road_id"]]
        direction = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        for polygon in JS._polygons(shape):
            bridged, issue = bridge_one(base, cover, polygon, direction, rs, rs_objects, others)
            if issue is not None:
                issues.append({"road_id": spec["road_id"], **issue})
            else:
                result.append({"road_id": spec["road_id"], **bridged})
    return result, issues


# ------------------------------------------------------------------ rule 1: median takeover
def _intervals(geometry, p, normal):
    cut = geometry.intersection(LineString([p - 100 * normal, p + 100 * normal]))
    out = []
    for line in getattr(cut, "geoms", [cut]):
        if line.geom_type == "LineString" and not line.is_empty:
            t = (np.asarray(line.coords) - p) @ normal
            out.append((float(t.min()), float(t.max())))
    return sorted(out)


def median_takeover(base, cover, tails, spec, rs, barriers):
    """(island polygon, evidence) or (None, issue): the median tracked on own tails + junction polygon (see module)."""
    interval = spec.get("median_interval")
    if interval is None or interval[1] - interval[0] < E.MEDIAN_MIN_M:
        return None, {"reason": "no-written-median"}
    if rs.is_empty:          # "not road surface" needs road-surface data around the junction to mean anything
        return None, {"reason": "median-takeover-no-road-surface-data"}
    origin = np.asarray(spec["pose"][:2], float)
    axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    normal = np.array([-axis[1], axis[0]])
    own = unary_union([t["geometry"] for t in tails if t["road_id"] == spec["road_id"]])
    if own.is_empty:
        return None, {"reason": "no-tails"}
    support = unary_union([own, base])
    max_s = max(float(((np.asarray(p.exterior.coords) - origin) @ axis).max()) for p in JS._polygons(own))
    width_cap = (interval[1] - interval[0]) + IDENTITY_M
    prev = (float(interval[0]), float(interval[1]))
    rows, sides, closed = [], [], None
    for s in np.linspace(-.5, max_s, max(3, int((max_s + .5) / STEP_M) + 2)):
        point = origin + s * axis
        ranges = _intervals(support, point, normal)
        gaps = [(x[1], y[0]) for x, y in zip(ranges[:-1], ranges[1:]) if y[0] - x[1] > GAP_MIN_M]
        cont = [g for g in gaps if abs(g[0] - prev[0]) <= IDENTITY_M and abs(g[1] - prev[1]) <= IDENTITY_M]
        if not rows and not cont:
            if s <= 1.:
                continue
            return None, {"reason": "median-takeover-no-anchor", "s": float(s)}
        if len(cont) > 1:
            return None, {"reason": "median-takeover-ambiguous", "s": float(s), "gaps": cont}
        if not cont:
            seg = LineString([point + prev[0] * normal, point + prev[1] * normal])
            if support.buffer(E.COVER_TOL_M).covers(seg):
                closed = float(s)            # the separator ends against source geometry
                break
            return None, {"reason": "median-takeover-open", "s": float(s), "ranges": ranges, "previous": prev}
        lo, hi = cont[0]
        if not rows and (abs(lo - interval[0]) > IDENTITY_M or abs(hi - interval[1]) > IDENTITY_M):
            return None, {"reason": "median-mouth-source-mismatch"}
        if hi - lo > width_cap:
            return None, {"reason": "median-takeover-wider-than-written-median", "s": float(s), "width_m": hi - lo}
        if not rows and s > -.49:
            anchor = origin - .5 * axis
            rows.append((anchor + normal * interval[0], anchor + normal * interval[1]))
            sides.append(("anchor", "anchor"))
        a_pt, b_pt = point + normal * lo, point + normal * hi
        side = tuple("tail" if own.distance(Point(q)) < 1e-6 else ("junction-polygon" if base.distance(Point(q)) < 1e-6
                                                                  else "unknown") for q in (a_pt, b_pt))
        if "unknown" in side:
            return None, {"reason": "median-takeover-edge-without-source", "s": float(s)}
        rows.append((a_pt, b_pt))
        sides.append(side)
        prev = (lo, hi)
        if base.covers(LineString([a_pt, b_pt])):
            closed = float(s)
            break
    if closed is None:
        return None, {"reason": "median-takeover-not-closed"}
    if len(rows) < 2:
        return None, {"reason": "median-insufficient-boundary-support"}
    polygon = Polygon(np.vstack([[x[0] for x in rows], [x[1] for x in rows[::-1]]]))
    if not polygon.is_valid:
        return None, {"reason": "median-takeover-invalid-polygon"}
    parts, join_issues = E._bridge_tails(base, cover, [{"road_id": spec["road_id"], "geometry": polygon}], [spec])
    if join_issues:
        return None, {"reason": "median-takeover-join", "issues": join_issues}
    outside = [p for p in JS._polygons(parts[0]["geometry"].difference(base)) if not p.interiors and p.area > .05]
    if len(outside) != 1:
        return None, {"reason": "median-takeover-not-one-island", "parts": len(outside)}
    island = outside[0]
    taken = [i for i, sd in enumerate(sides) if "junction-polygon" in sd]
    band = island.boundary.buffer(E.COVER_TOL_M)
    rs_inside = island.intersection(rs).difference(band).area
    if rs_inside > E.SLIVER_M2:
        return None, {"reason": "median-takeover-is-road-surface", "road_surface_m2": rs_inside}
    t_rows = [rows[i] for i in taken]
    evidence = {"rule": "median-takeover", "road_id": spec["road_id"], "rows": len(rows),
                "takeover_rows": len(taken), "closed_at_s": closed,
                "takeover_s_range": [float((t_rows[0][0] - origin) @ axis), float((t_rows[-1][0] - origin) @ axis)]
                if t_rows else None,
                "takeover_width_m": [float(min(np.linalg.norm(b - a) for a, b in t_rows)),
                                     float(max(np.linalg.norm(b - a) for a, b in t_rows))] if t_rows else None,
                "written_median_width_m": float(interval[1] - interval[0]),
                "island_area_m2": float(island.area),
                "road_surface_overlap_m2": float(island.intersection(rs).area),
                "road_surface_overlap_beyond_cover_band_m2": float(rs_inside),
                "barriers": [{"object": i, "length_inside_m": float(g.intersection(island).length)}
                             for i, g in barriers if g.intersection(island).length > 0],
                "status": "INFERRED", "support_kind": "median-continued-on-source-junction-polygon"}
    return island, evidence


# ------------------------------------------------------------------ builder
def _failing(stats):
    return any(stats[k] for k in ("mouth_tail_source_issues", "mouth_tail_bridge_issues", "median_tail_issues"))


def _tag(road, lane_type, rule):
    """Mark the family lanes a rule added to as INFERRED, with the rule's source evidence (eight-state status)."""
    for sec in road.sections:
        for lane in sec.left + sec.right:
            if lane.lane_type != lane_type:
                continue
            prov = dict(lane.provenance or {})
            prov["status"] = "INFERRED"
            prov["seam_rules"] = [*prov.get("seam_rules", []), {"code": CODE, **rule}]
            lane.provenance = prov


def _append_with_rules(doc, src, junction, project, mouths, axes, junction_id=1):
    """envelope_surface.append_source_paving with the two rules (same structure, same helpers)."""
    from mapforge.adapters.opendrive.writer import add_paving_road
    base, pieces, _, summary = JS.source_tail_regions(src, junction, project, mouths)
    cover = base.buffer(E.COVER_TOL_M)
    rs, rs_objects = road_surface(src, junction, project)
    barriers = _source_objects(src, junction, project, TRAFFIC_BARRIER)
    tails, issues = bridge_tails(base, cover, pieces, mouths, rs, rs_objects)
    medians, median_issues, median_inside = E._median_tails(base, cover, tails, mouths)
    stats = {"mouth_tail_source_components": summary["components"],
             "mouth_tail_source_issues": summary["issues"], "mouth_tail_bridge_issues": issues,
             "mouth_tail_bridges": [{k: v for k, v in p.items() if k not in {"geometry", "axis"}} for p in tails],
             "median_tail_issues": median_issues,
             "median_tail_areas_m2": [p["geometry"].area for p in medians],
             "surface_family_parts": [], "covered_gap_stations": 0, "filled_slivers": 0,
             "seam_rules": []}
    for t in tails:
        if t.get("road_surface_seam"):
            stats["seam_rules"].append({"rule": "road-surface-seam", "road_id": t["road_id"], "sweep_m": t["sweep_m"],
                                        "end_cap_misses": t.get("end_cap_misses"), "holes": t["road_surface_seam"],
                                        "status": t["status"], "support_kind": t["support_kind"]})
    if median_inside:
        stats["median_tails_inside_polygon"] = sorted(median_inside)
    if _failing(stats):
        return stats
    regions = [(base, axis, "source-polygon", "restricted") for axis in axes[:2]]
    used = {r.road_id for r in doc.roads}
    ids = iter(i for i in range(50, 100) if i not in used)
    count = 0
    for polygon, axis, support, lane_type in regions:
        for part in JS._polygons(polygon):
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
        if (interval is not None and interval[1] - interval[0] >= E.MEDIAN_MIN_M and not islands
                and spec["road_id"] not in median_inside):
            stats["mouth_tail_bridge_issues"].append({"road_id": spec["road_id"], "reason": "median-tail-unresolved"})
            continue
        axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        source_ids = [p["source_lane"] for p in pieces if p["road_id"] == spec["road_id"]]

        def families(islands_now):
            parts = JS._polygons(unary_union(supported + islands_now))
            out = []
            for part in parts:
                part, filled = E._fill_slivers(part)
                own = [g for g in islands_now if g.intersects(part)]
                caps = [JS.directional_sweep(p, axis, E.CAP_M).intersection(base) for p in [part] if not p.interiors]
                out.append((unary_union([part, *caps]), unary_union(own) if own else LineString(), filled))
            return out

        planned = families(islands)
        trial = type(doc)("trial")
        tags = [("restricted", {"rule": "road-surface-seam", "road_surface_objects": h["road_surface_objects"],
                                "hole_area_m2": h["area_m2"]})
                for t in tails if t["road_id"] == spec["road_id"] for h in t.get("road_surface_seam", [])]
        try:
            for family, median, _f in planned:
                E._append_surface_family(trial, family, median, spec, 0, junction_id, source_ids, cover)
        except ValueError as exc:
            if "unresolved gap inside source surface family" not in str(exc):
                raise
            island, evidence = median_takeover(base, cover, tails, spec, rs, barriers)
            if island is None:
                stats.setdefault("seam_rule_rejections", []).append({"road_id": spec["road_id"], **evidence})
                raise
            evidence["replaced_island_area_m2"] = float(sum(g.area for g in islands))
            stats["seam_rules"].append(evidence)
            planned = families([island])
            tags.append(("median", {"rule": "median-takeover", "support_kind": evidence["support_kind"],
                                    "takeover_s_range": evidence["takeover_s_range"],
                                    "barriers": [b["object"] for b in evidence["barriers"]]}))
        stats["surface_family_parts"].append({"road_id": spec["road_id"], "parts": len(planned)})
        for family, median, filled in planned:
            stats["filled_slivers"] += filled
            stats["covered_gap_stations"] += E._append_surface_family(
                doc, family, median, spec, next(ids), junction_id, source_ids, cover)
            for lane_type, rule in tags:
                _tag(doc.roads[-1], lane_type, rule)
            count += 1
    stats.update(paving="source-boundary-regions-gap-aware", paving_roads=count)
    return stats


def append_source_paving(doc, src, junction, project, mouths, axes, junction_id=1):
    """envelope_surface.append_source_paving; only where it fails, the seam rules (else its failure unchanged)."""
    n0 = len(doc.roads)
    try:
        stats = E.append_source_paving(doc, src, junction, project, mouths, axes, junction_id)
        error = None
        if not _failing(stats) or not has_road_surface(src):
            return stats
    except ValueError as exc:
        if not has_road_surface(src):
            raise
        stats, error = None, exc
    del doc.roads[n0:]
    try:
        retry = _append_with_rules(doc, src, junction, project, mouths, axes, junction_id)
    except ValueError:
        retry = None
    if retry is not None and not _failing(retry) and retry.get("seam_rules"):
        retry["surface_builder_fallback"] = CODE
        return retry
    del doc.roads[n0:]
    if error is not None:
        raise error
    return E.append_source_paving(doc, src, junction, project, mouths, axes, junction_id)


def replace_source_paving(root, src, junction, project, decisions):
    """Same contract as envelope_surface.replace_source_paving, with append_source_paving above."""
    from mapforge.adapters.opendrive import writer as W
    from mapforge.ops.map_to_xodr import _mouth_apron_axes
    mouths = JS.written_mouth_specs(root, decisions)
    doc = W.XodrDoc("candidate-source-surface")
    for rd in [rd for rd in root.findall("road") if rd.get("name") != "junction_paving"]:
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
    stats["source_surface_status"] = "FAIL" if _failing(stats) else "GENERATED"
    stats["surface_builder"] = E.CODE
    return stats
