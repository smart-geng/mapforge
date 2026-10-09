"""Tight-turn connector source vias: recorded as a source conflict, left out of the graded lane centre (SHP).

Some SHP junction via lines turn far tighter than a vehicle can: 0621 road 100 turns 94 deg within 6.5 m, about a
2.7 m radius over 3 m, with 21 deg kinks at single vertices, after a straight inferred gap bridge. A connector that
keeps the converter's G2 smoothness has to start turning early and cut the corner, so its lane centre lies
0.2-0.3 m off such a window, and following it closer takes dense short segments and sharper curvature
(handoff_assets/research/0621-road100-feasibility-20261009). User decision (2026-10-09): a comparable junction-via
lane whose source window turns tighter than RADIUS_MIN_M within any WINDOW_M stretch is a source conflict,
recorded by the SHP source review (``tight-turn-source-conflict``) and not followed; the scoreboard grades SHP lane
centres without these connector lanes (whole lanes) on top of the mouth-curb-flare zones (policy 0.8-draft, same
thresholds). 5 m is about a passenger car's minimum turning radius; over the 393 junction vias of 18 SHP cases it
marks 9 lanes in 5 junctions (docs/工作台-急弯连接路原件冲突-2026-10-09.md).

G8 and its thresholds stay as they are; lane_center_noflare_* and the raw G8 statistics are still reported.
"""
from __future__ import annotations

import numpy as np

from mapforge.validate.lane_centre_flare import zones as flare_zones
from mapforge.validate.lane_fidelity import _resample, extract_target_components
from mapforge.validate.shp_source_review import FLARE_ZONE_M

RULE = "tight-turn-source-conflict"
WINDOW_M = 3.0          # the source window's heading change is measured over every stretch this long
RADIUS_MIN_M = 5.0      # a junction via turning tighter than this somewhere is a source conflict
PROBE_STEP_M = 0.1
DECISION = ("smooth-connector (user, 2026-10-09): a comparable junction via whose source window turns tighter "
            "than a 5 m radius within any 3 m is a source conflict, recorded and not followed; SHP T2 grades "
            "the lane centres without these connector lanes and without the mouth-curb-flare zones (0.8-draft)")


def turn_shape(coords) -> dict:
    """Length, total turn, the tightest mean curvature over WINDOW_M (and its radius) and the largest single-vertex
    turn of a source window polyline."""
    p = np.asarray(coords, float)[:, :2]
    seg = np.diff(p, axis=0)
    seg = seg[np.hypot(seg[:, 0], seg[:, 1]) > 1e-9]
    if len(seg) == 0:
        return {"length_m": 0.0, "total_turn_deg": 0.0, "kappa_window_max": 0.0, "radius_window_min_m": None,
                "vertex_turn_max_deg": 0.0}
    length = np.hypot(seg[:, 0], seg[:, 1])
    s = np.concatenate([[0.0], np.cumsum(length)])
    heading = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    mid = (s[:-1] + s[1:]) / 2
    if s[-1] <= WINDOW_M:
        kappa = abs(heading[-1] - heading[0]) / max(s[-1], 1e-9)
    else:
        probes = np.linspace(WINDOW_M / 2, s[-1] - WINDOW_M / 2, max(2, int(s[-1] / PROBE_STEP_M)))
        kappa = float(np.max(np.abs(np.interp(probes + WINDOW_M / 2, mid, heading)
                                    - np.interp(probes - WINDOW_M / 2, mid, heading)) / WINDOW_M))
    turns = np.degrees(np.abs(np.diff(heading))) if len(heading) > 1 else np.zeros(1)
    return {"length_m": float(s[-1]), "total_turn_deg": float(np.degrees(heading[-1] - heading[0])),
            "kappa_window_max": float(kappa), "radius_window_min_m": float(1 / kappa) if kappa > 0 else None,
            "vertex_turn_max_deg": float(turns.max())}


def findings(manifest: dict) -> list:
    """tight-turn-source-conflict findings: comparable junction-via lanes of a source manifest (G8 windows)."""
    out = []
    for lane in manifest.get("lanes", []):
        if lane.get("role") != "junction-via" or not (lane.get("comparison") or {}).get("eligible"):
            continue
        shape = turn_shape((lane.get("geometry") or {}).get("coordinates", []))
        radius = shape["radius_window_min_m"]
        if radius is None or radius >= RADIUS_MIN_M:
            continue
        out.append({"rule": RULE, "status": "RECORDED", "resolution": "smooth-connector",
                    "source_lane_id": lane["source_lane_id"], "support": (lane.get("support") or {}).get("reason"),
                    "window_length_m": round(shape["length_m"], 3),
                    "total_turn_deg": round(shape["total_turn_deg"], 2),
                    "radius_window_min_m": round(radius, 3), "window_m": WINDOW_M,
                    "vertex_turn_max_deg": round(shape["vertex_turn_max_deg"], 2)})
    return out


def lane_distances(root, manifest: dict, review: dict | None, ds: float = 1.0, zone: float = FLARE_ZONE_M,
                   zero_width_epsilon_m: float = 0.05) -> tuple[dict, int]:
    """({source lane id: (source-to-target, target-to-source)}, flare samples left out): G8's comparable lanes with
    the mouth-curb-flare zones left out exactly as lane_centre_flare.distances does for its pooled numbers."""
    from scipy.spatial import cKDTree
    sources = {lane["source_lane_id"]: lane for lane in manifest.get("lanes", [])
               if (lane.get("comparison") or {}).get("eligible")}
    components = {}
    for comp in extract_target_components(root, ds=ds, zero_width_epsilon_m=zero_width_epsilon_m)["components"]:
        components.setdefault(comp["source_lane_id"], []).append(comp)
    flared = flare_zones(review)
    out, left_out = {}, 0
    for sid in sorted(set(sources) & set(components)):
        if len(components[sid]) != 1:
            continue
        src = _resample((sources[sid].get("geometry") or {}).get("coordinates", []), ds)
        tgt = _resample(components[sid][0]["points"], ds)
        if len(src) < 2 or len(tgt) < 2:
            continue
        s2t = cKDTree(tgt).query(src)[0]
        t2s = cKDTree(src).query(tgt)[0]
        if sid in flared:
            mouths = np.asarray(flared[sid], float)
            keep_s = np.linalg.norm(src[:, None, :2] - mouths[None], axis=2).min(axis=1) > zone
            keep_t = np.linalg.norm(tgt[:, None, :2] - mouths[None], axis=2).min(axis=1) > zone
            left_out += int((~keep_s).sum() + (~keep_t).sum())
            s2t, t2s = s2t[keep_s], t2s[keep_t]
        out[sid] = (s2t, t2s)
    return out, left_out


def audit(root, manifest, review) -> dict:
    """Worst direction (as the scoreboard reports G8) of median / P95 / max without the flare zones and without the
    tight-turn conflict lanes; the number of conflicts and of the lanes and samples left out."""
    if review is None:
        return {}
    conflicts = {f["source_lane_id"] for f in findings(manifest)}
    lanes, _ = lane_distances(root, manifest, review)
    kept = [v for sid, v in lanes.items() if sid not in conflicts]
    s2t = np.concatenate([v[0] for v in kept]) if kept else np.zeros(0)
    t2s = np.concatenate([v[1] for v in kept]) if kept else np.zeros(0)
    if not len(s2t) or not len(t2s):
        return {}
    left = [v for sid, v in lanes.items() if sid in conflicts]
    out = {"tight_turn_source_conflicts": len(conflicts),
           "lane_center_conflict_lanes_left_out": len(left),
           "lane_center_conflict_samples_left_out": int(sum(len(a) + len(b) for a, b in left))}
    for name, fn in (("median", np.median), ("p95", lambda a: np.percentile(a, 95)), ("max", np.max)):
        out[f"lane_center_noconflict_{name}_m"] = float(max(fn(s2t), fn(t2s)))
    return out
