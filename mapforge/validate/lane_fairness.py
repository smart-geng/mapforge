"""Lane-centre fairness: how sharply driving lanes bend and how fast their curvature changes (scoreboard
report items, 2026-10-07; not tier checks).

The smoothness tier checks measure continuity at joins (curvature jumps at width/section joins, junction
interfaces and edge contacts) and segment lengths. Nothing bounds how sharply a lane centre bends between joins
or how fast its curvature changes there: a lane can be G2 everywhere and still swing 6 m sideways within 20 m
(MAP mirrored departure lanes beside a turn bay, map-node4 road 10: 0.125 /m) or carry an S-bend through a
straight junction movement (SHP straight connectors at source joint kinks: 0.03-0.06 /m). These items report it
per file, speed-free: curvature [1/m] and curvature rate along the lane [1/m^2].

Driving lanes are sampled per lane section every 0.25 m (world curvature of the lane centre, as G11-D does; the
curvature rate is the finite difference between samples, so a curvature jump at a join reads as jump / DS_M) and
grouped as
- road: ordinary-road lanes at least 2.5 m wide whose width changes by less than 0.3 m in the section (through
  lanes); births, tapers and narrow lanes are reported apart (road_event);
- straight connector: a junction road turning by less than 20 deg in all; turning connector: 20-150 deg.
"""
from __future__ import annotations

import math

import numpy as np

from mapforge.validate.smoothness import _edge_world_curvature, _geoms, _ref_kappa_at, lane_edges_kinematics_at

DS_M = 0.25
FULL_WIDTH_M = 2.5
WIDTH_CHANGE_M = 0.3
STRAIGHT_DEG = 20.0
UTURN_DEG = 150.0
S_BEND_KAPPA = 0.005   # a straight connector bending both ways by more than this (R 200 m) has an S-bend


def turn_deg(road) -> float:
    return math.degrees(sum((k0 + k1) / 2 * length for _, _, _, _, length, k0, k1 in _geoms(road)))


def lane_tracks(road, ds: float = DS_M) -> list[dict]:
    """Per lane section and driving lane: curvature samples of the lane centre and its width range."""
    length = float(road.get("length"))
    secs = road.findall("lanes/laneSection")
    starts = [float(s.get("s")) for s in secs]
    out = []
    for si, sec in enumerate(secs):
        s0 = starts[si]
        s1 = starts[si + 1] if si + 1 < len(secs) else length
        if s1 - s0 < 1e-6:
            continue
        margin = min(1e-4, (s1 - s0) * 0.05)
        grid = np.arange(s0 + margin, s1 - margin + 1e-12, ds)
        if len(grid) < 2:
            grid = np.linspace(s0 + margin, s1 - margin, 2)
        for side in ("left", "right"):
            lanes = sorted(sec.findall(f"{side}/lane"), key=lambda x: abs(int(x.get("id"))))
            values = [[] for _ in lanes]
            for s in grid:
                kref, sharp = _ref_kappa_at(road, float(s))
                edges = lane_edges_kinematics_at(road, float(s), side)
                if len(edges) != len(lanes) + 1:
                    continue
                for i, (a, b) in enumerate(zip(edges, edges[1:])):
                    t, d1, d2 = ((x + y) / 2 for x, y in zip(a, b))
                    values[i].append((float(s), _edge_world_curvature(t, d1, d2, kref, sharp),
                                      math.hypot(1 - kref * t, d1), abs(a[0] - b[0])))
            for lane, v in zip(lanes, values):
                if lane.get("type") != "driving" or len(v) < 2:
                    continue
                a = np.asarray(v)
                dl = np.diff(a[:, 0]) * (a[:-1, 2] + a[1:, 2]) / 2
                out.append({"lane": int(lane.get("id")), "s0": s0, "s1": s1, "kappa": a[:, 1],
                            "sharpness": np.abs(np.diff(a[:, 1])) / np.maximum(dl, 1e-9),
                            "width_min": float(np.min(a[:, 3])), "width_max": float(np.max(a[:, 3]))})
    return out


def classify(road, track) -> str:
    if road.get("junction") in (None, "-1"):
        through = track["width_min"] >= FULL_WIDTH_M and track["width_max"] - track["width_min"] < WIDTH_CHANGE_M
        return "road" if through else "road_event"
    turn = abs(turn_deg(road))
    return "straight_conn" if turn < STRAIGHT_DEG else ("turn_conn" if turn < UTURN_DEG else "uturn_conn")


def tracks_by_class(root) -> dict[str, list[dict]]:
    groups: dict[str, list[dict]] = {}
    for road in root.findall("road"):
        if road.get("name") == "junction_paving":
            continue
        for track in lane_tracks(road):
            track["road"] = road.get("id")
            groups.setdefault(classify(road, track), []).append(track)
    return groups


def audit(root) -> dict:
    groups = tracks_by_class(root)
    out = {}

    def peaks(name):
        return np.asarray([float(np.max(np.abs(t["kappa"]))) for t in groups.get(name, [])])

    def rates(name):
        return np.asarray([float(np.max(t["sharpness"])) for t in groups.get(name, []) if len(t["sharpness"])])

    road, event = peaks("road"), peaks("road_event")
    if len(road):
        out["fair_road_kappa_max_per_m"] = float(road.max())
        out["fair_road_kappa_p90_per_m"] = float(np.percentile(road, 90))
        out["fair_road_sharpness_max_per_m2"] = float(rates("road").max())
    if len(event):
        out["fair_road_event_kappa_max_per_m"] = float(event.max())
    straight = peaks("straight_conn")
    if len(straight):
        out["fair_straight_conn_kappa_max_per_m"] = float(straight.max())
        out["fair_straight_conn_kappa_median_per_m"] = float(np.median(straight))
        out["fair_straight_conn_s_bends"] = sum(
            1 for t in groups["straight_conn"] if t["kappa"].max() > S_BEND_KAPPA and t["kappa"].min() < -S_BEND_KAPPA)
    turn = rates("turn_conn")
    if len(turn):
        out["fair_turn_conn_sharpness_p90_per_m2"] = float(np.percentile(turn, 90))
        out["fair_turn_conn_sharpness_max_per_m2"] = float(turn.max())
    return out
