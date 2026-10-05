"""Counter-curvature inside turning connectors (scoreboard diagnostic, 2026-10-04).

A connector that turns by 30 deg or more should curve one way. The SHP connectors followed digitized via
arcs (kinks of a few degrees) or fitted two curvature peaks with an opposite lobe between them, so some
lane centres curve the wrong way for 2-7 m by 0.02-0.06 /m in the middle of a turn: a steering reversal.
G2 continuity metrics do not see this; this module measures it on the written lane centre.
"""
from __future__ import annotations

import math

from mapforge.validate.smoothness import _edge_world_curvature, _ref_kappa_at, lane_edges_kinematics_at

TURN_MIN_DEG = 30.0
STEP_M = 0.25


def centre_curvature_profile(road, lane_id=-1, step=STEP_M):
    """World curvature of a single-section connector lane centre at mid-step stations (travel along +s)."""
    length = float(road.get("length"))
    side = "left" if lane_id > 0 else "right"
    index = abs(lane_id) - 1
    out = []
    s = step / 2
    while s < length:
        edges = lane_edges_kinematics_at(road, s, side)
        t, d1, d2 = [(a + b) / 2 for a, b in zip(edges[index], edges[index + 1])]
        kappa, sharpness = _ref_kappa_at(road, s)
        out.append(_edge_world_curvature(t, d1, d2, kappa, sharpness))
        s += step
    return out


def counter_curvature(profile, step=STEP_M):
    """(turn in degrees, largest opposite-sign curvature, metres of opposite curvature above 0.01 /m)."""
    turn = sum(profile) * step
    sign = 1.0 if turn >= 0 else -1.0
    opposite = [-sign * k for k in profile]
    return math.degrees(turn), max(0.0, max(opposite, default=0.0)), step * sum(1 for k in opposite if k > 0.01)


def counter_curvature_zones(profile, zone_m, step=STEP_M):
    """(largest opposite-sign curvature more than ``zone_m`` from both ends, largest within ``zone_m`` of an end):
    the turning connectors may steer back gently next to the mouths, where the source via lines meet the
    linked lanes at a few degrees (user decision 2026-10-05), but not in the middle of the turn."""
    turn = sum(profile) * step
    sign = 1.0 if turn >= 0 else -1.0
    length = len(profile) * step
    mid = ends = 0.0
    for i, k in enumerate(profile):
        s = (i + 0.5) * step
        c = max(0.0, -sign * k)
        if s <= zone_m or s >= length - zone_m:
            ends = max(ends, c)
        else:
            mid = max(mid, c)
    return mid, ends


def audit(root, threshold=0.02):
    """Largest counter-curvature over turning connectors (|turn| >= TURN_MIN_DEG) and how many exceed it."""
    worst, count = 0.0, 0
    for road in root.findall("road"):
        if road.get("junction") in (None, "-1") or road.get("name") == "junction_paving":
            continue
        if len(road.findall("lanes/laneSection")) != 1 or road.find("lanes/laneSection/right/lane[@id='-1']") is None:
            continue
        turn, counter, _ = counter_curvature(centre_curvature_profile(road))
        if abs(turn) < TURN_MIN_DEG:
            continue
        worst = max(worst, counter)
        count += counter > threshold
    return {"turn_counter_curvature_max_per_m": worst, "turn_counter_curvature_gt_0.02": count}
