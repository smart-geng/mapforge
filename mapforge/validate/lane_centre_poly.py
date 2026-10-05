"""Lane-centre deviation measured to the polyline (scoreboard diagnostic beside G8, reported only).

G8 (lane_fidelity) resamples the source and the target lane centre at 1 m, each from its own first point,
and takes nearest-sample distances. When the two first points are offset along the lane, every sample of
one line falls between two samples of the other and the distance gains up to half the step: a lane that
lies exactly on its source but starts 0.45 m later measures about 0.45 m all along (map-node3 road 12 lane
-1, whose MAP points begin 0.45 m before the road). Here the same samples are measured to the other line
itself (point to polyline), on the same components and the same comparable lanes as G8. G8 itself and its
thresholds stay as they are (frozen); this only shows how much of a G8 figure is sampling phase.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from mapforge.validate.lane_fidelity import _point_to_polyline, _resample, extract_target_components


def deviations(root, manifest: dict, ds: float = 1.0) -> dict:
    """{source lane id: (source-to-target, target-to-source) point-to-polyline distances} like G8's pairs."""
    sources = {lane["source_lane_id"]: lane for lane in manifest.get("lanes", [])
               if (lane.get("comparison") or {}).get("eligible")}
    components = {}
    for comp in extract_target_components(root, ds=ds)["components"]:
        components.setdefault(comp["source_lane_id"], []).append(comp)
    out = {}
    for sid in sorted(set(sources) & set(components)):
        if len(components[sid]) != 1:
            continue
        src_line = np.asarray((sources[sid].get("geometry") or {}).get("coordinates", []), float)
        if src_line.ndim != 2 or len(src_line) < 2:
            continue
        src_line = src_line[:, :2]
        tgt_line = np.asarray(components[sid][0]["points"], float)
        src, tgt = _resample(src_line, ds), _resample(tgt_line, ds)
        if len(src) < 2 or len(tgt) < 2:
            continue
        out[sid] = (np.array([_point_to_polyline(p, tgt_line) for p in src]),
                    np.array([_point_to_polyline(p, src_line) for p in tgt]))
    return out


def audit(root, manifest) -> dict:
    """Worst direction (as G8 reports it) of median / P95 / max over all comparable lanes."""
    if isinstance(manifest, (str, Path)):
        manifest = json.loads(Path(manifest).read_text(encoding="utf-8"))
    pairs = deviations(root, manifest)
    if not pairs:
        return {}
    s2t = np.concatenate([a for a, _ in pairs.values()])
    t2s = np.concatenate([b for _, b in pairs.values()])
    out = {}
    for name, fn in (("median", np.median), ("p95", lambda a: np.percentile(a, 95)), ("max", np.max)):
        out[f"lane_center_poly_{name}_m"] = float(max(fn(s2t), fn(t2s)))
    return out
