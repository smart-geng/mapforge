"""Lane-centre deviation without the curb-return flares at junction mouths (scoreboard report, not graded).

At a junction mouth the SHP lane centreline often swings outward with the curb return (the lane polygon widens
towards the corner) and the via lines start or end on the swung-out point. The converter keeps the mouth a
regular cross-section and the connectors leave and reach it tangentially, so the lane centres do not follow that
swing: the deviation sits within a few metres of such mouths (shp-node17 road 10 lane -3: connectors 103 to 106
0.3-0.6 m off in their first metres). User decision (2026-10-05): the flares are a source characteristic,
recorded by the SHP source review (``mouth-curb-flare``), not followed.

Here G8's own lane-centre statistics are recomputed (same comparable lanes, same 1 m resampling from each line's
first point, same nearest-sample distances, worst direction as the scoreboard reports G8) with the samples of
the flared lane and of the connector lanes attached at that mouth left out within FLARE_ZONE_M of the mouth's
lane-centre point. G8 and its thresholds stay as they are; this only shows how much of a G8 figure the flares
make.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from mapforge.validate.lane_fidelity import _resample, extract_target_components
from mapforge.validate.shp_source_review import FLARE_ZONE_M


def zones(review: dict | None) -> dict:
    """{source lane id: [mouth points]} of the mouth-curb-flare findings: the flared lane and the connector lanes
    attached at that mouth."""
    out = {}
    for f in (review or {}).get("findings", []):
        if f.get("rule") != "mouth-curb-flare" or not f.get("mouth"):
            continue
        xy = (float(f["mouth"]["x"]), float(f["mouth"]["y"]))
        for sid in [f["source_lane_id"], *f.get("connector_lanes", [])]:
            out.setdefault(sid, []).append(xy)
    return out


def distances(root, manifest: dict, review: dict | None, ds: float = 1.0, zone: float = FLARE_ZONE_M,
              zero_width_epsilon_m: float = 0.05):
    """(source-to-target, target-to-source, samples left out) over G8's comparable lanes."""
    from scipy.spatial import cKDTree
    sources = {lane["source_lane_id"]: lane for lane in manifest.get("lanes", [])
               if (lane.get("comparison") or {}).get("eligible")}
    components = {}
    for comp in extract_target_components(root, ds=ds, zero_width_epsilon_m=zero_width_epsilon_m)["components"]:
        components.setdefault(comp["source_lane_id"], []).append(comp)
    flared = zones(review)
    s2t_all, t2s_all, left_out = [], [], 0
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
        s2t_all.append(s2t)
        t2s_all.append(t2s)
    cat = lambda parts: np.concatenate(parts) if parts else np.zeros(0)
    return cat(s2t_all), cat(t2s_all), left_out


def audit(root, manifest, review) -> dict:
    """Worst direction (as the scoreboard reports G8) of median / P95 / max without the flare zones, the number of
    flared mouths and of the samples left out."""
    if isinstance(manifest, (str, Path)):
        manifest = json.loads(Path(manifest).read_text(encoding="utf-8"))
    if review is None:
        return {}
    s2t, t2s, left_out = distances(root, manifest, review)
    if not len(s2t) or not len(t2s):
        return {}
    out = {"mouth_curb_flares": sum(1 for f in review.get("findings", []) if f.get("rule") == "mouth-curb-flare"),
           "lane_center_flare_samples_left_out": left_out}
    for name, fn in (("median", np.median), ("p95", lambda a: np.percentile(a, 95)), ("max", np.max)):
        out[f"lane_center_noflare_{name}_m"] = float(max(fn(s2t), fn(t2s)))
    return out
