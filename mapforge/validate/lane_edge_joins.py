"""World-curvature jumps of every lane edge at the written joins inside a road (scoreboard diagnostic).

The scoreboard's join metric (G11 ``_driving_curvature_joins``) covers driving-lane centres only. Lane
edges are G1 by construction; their curvature still jumps where the reference sharpness changes and an
edge has a lateral slope (t * t' * dk'), or where width/offset second derivatives jump. D2 allows edges a
bounded jump; the bound is a spec v1 decision, so this module only measures it (2026-10-04).
"""
from __future__ import annotations

from mapforge.validate.smoothness import _edge_world_curvature, _geoms, _ref_kappa_at, lane_edges_kinematics_at

ONE_SIDED_M = 1e-6   # one-sided limits at each join
PAIR_TOL_M = 0.01    # an edge continues across a join if it stays this close; otherwise birth/death


def _joins(road, reference_only=False):
    cuts, s = set(), 0.0
    for geom in _geoms(road):
        cuts.add(s)
        s += geom[4]
    length = float(road.get("length"))
    if reference_only:
        return sorted(c for c in cuts if 10 * ONE_SIDED_M < c < length - 10 * ONE_SIDED_M)
    cuts.update(float(o.get("s", 0)) for o in road.findall("lanes/laneOffset"))
    for section in road.findall("lanes/laneSection"):
        s0 = float(section.get("s", 0))
        cuts.add(s0)
        for record in section.findall(".//lane/width") + section.findall(".//lane/border"):
            cuts.add(s0 + float(record.get("sOffset", 0)))
    return sorted(c for c in cuts if 10 * ONE_SIDED_M < c < length - 10 * ONE_SIDED_M)


def _edges(road, s, side):
    kappa, sharpness = _ref_kappa_at(road, s)
    return [(t, _edge_world_curvature(t, d1, d2, kappa, sharpness))
            for t, d1, d2 in lane_edges_kinematics_at(road, s, side)]


def road_edge_jumps(road, reference_only=False):
    """(jump, s, side, t) for every lane edge that continues across a join; the laneOffset edge once.

    ``reference_only``: reference primitive joins only (enough where every lateral function is C2).
    """
    out = []
    for s in _joins(road, reference_only):
        for side in ("left", "right"):
            before, after = _edges(road, s - ONE_SIDED_M, side), _edges(road, s + ONE_SIDED_M, side)
            for i, (t, k) in enumerate(before):
                if i == 0 and side == "left":
                    continue
                match = min(after, key=lambda e: abs(e[0] - t)) if after else None
                if match is None or abs(match[0] - t) > PAIR_TOL_M:
                    continue
                out.append((abs(match[1] - k), s, side, t))
    return out


def audit(root, threshold=1e-3):
    """Largest lane-edge curvature jump and the number above ``threshold`` (paving roads excluded); also the largest
    on connectors alone (2026-10-07)."""
    worst, count, conn = 0.0, 0, 0.0
    for road in root.findall("road"):
        if road.get("name") == "junction_paving":
            continue
        for jump, *_ in road_edge_jumps(road):
            worst = max(worst, jump)
            count += jump > threshold
            if road.get("junction") not in (None, "-1"):
                conn = max(conn, jump)
    return {"lane_edge_join_curvature_jump_max_per_m": worst, "lane_edge_join_jumps_gt_1e-03": count,
            "conn_edge_join_curvature_jump_max_per_m": conn}
