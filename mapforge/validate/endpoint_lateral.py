"""Lateral part of the G8 lane endpoint gaps (scoreboard diagnostic, 2026-10-04).

G8 measures a lane's endpoint error as the distance between the first (last) points of its source line in the
manifest and of the target lane centre, both resampled every metre. For SHP outputs most of the larger gaps lie
along the lane, not across it: the converter clips each source lane at the span stations of the leg's raw axis,
while the road ends where the fitted reference line ends, so the source window runs 0.2-0.48 m past the road
end at the mouths (shp-node13 road 11: road 29.142 m, every source window 29.50 m). This module reports the
component of the same gap across the source direction; how far a lane reaches along it stays with G8 (endpoint,
coverage). Report only: the T2 gate keeps G8's endpoint metric.
"""
from __future__ import annotations

import numpy as np

from mapforge.validate.lane_fidelity import _resample, extract_target_components


def _lateral(src, tgt, at_start):
    i, j = (0, 1) if at_start else (-1, -2)
    direction = src[j] - src[i] if at_start else src[i] - src[j]
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return None
    direction /= norm
    gap = tgt[i] - src[i]
    return abs(float(gap @ np.array([-direction[1], direction[0]])))


def audit(root, manifest, g8):
    """{"lane_endpoint_lateral_max_m"}: over the lanes G8 compares, both travel ends."""
    lanes = {lane["source_lane_id"]: lane for lane in manifest.get("lanes", [])}
    components = {}
    for component in extract_target_components(root)["components"]:
        components.setdefault(component["source_lane_id"], []).append(component)
    worst = None
    for record in (g8 or {}).get("per_lane", []):
        source_id = record["source_lane_id"]
        if source_id not in lanes or len(components.get(source_id, [])) != 1:
            continue
        src = _resample((lanes[source_id].get("geometry") or {}).get("coordinates", []), 1.0)
        tgt = _resample(components[source_id][0]["points"], 1.0)
        if len(src) < 2 or len(tgt) < 2:
            continue
        for at_start in (True, False):
            value = _lateral(src, tgt, at_start)
            if value is not None and (worst is None or value > worst):
                worst = value
    return {"lane_endpoint_lateral_max_m": worst}
