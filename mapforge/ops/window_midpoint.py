"""SHP road lanes whose source centreline is off its boundaries: compared on the boundary midpoint (2026-10-05).

User decision (2026-10-04, boundary priority): lane edges follow the SHP lane boundaries and a lane centre is the
midpoint of its edges; where the SHP centreline disagrees, the difference is recorded by the source review
(``centreline-off-midpoint``). G8 still compared those lanes with the SHP centreline, i.e. with a line the
conversion is told not to follow, and measured the source's own inconsistency:

* shp-node18 road 11 lane 4 (..496860): the lane widens from 2.7 to 3.7 m over s 205-240, the converted edges lie
  within 2 cm of the boundaries, the centreline keeps to the inner boundary, 0.70-0.24 m off the midpoint; G8
  compares the 6.4 m of section 15 (0.42-0.50 m off): node18's lane-centre maximum and lateral endpoint 0.508 m;
* shp-node3 road 11 lane -3 (..865690): a two-point chord while the lane shifts and narrows, 0.61-0.02 m off:
  node3's lateral endpoint 0.310 m.

User decision (2026-10-05): for G8-comparable road lanes (approach, departure; not junction vias, whose connectors
are fitted to the via centreline itself) for which the source review's rule finds the centreline off the midpoint
(shp_source_review.off_midpoint, the same judgement), the comparison window is moved onto the midpoint of the two
source boundaries where it lies CENTRE_OFFSET_MIN_M or more off it (the rule's own threshold), blending back to the
centreline over BLEND_M on either side so that the reference has no step; elsewhere the SHP centreline stays the
reference (a written lane whose shared edge lies between two neighbours' boundaries sits nearer to a centreline
that is only slightly off: shp-node13 road 10 lane -3, 0.15 m off over 0.5 m, compared 0.06 m off its centreline
and 0.11 m off its own midpoint). The window is resampled every STEP_M; where a boundary does not reach, it keeps
its point. The change and the source offset are recorded on the window; the SHP source and the converted geometry
are not changed.
"""
from __future__ import annotations

import numpy as np

CODE = "mapforge.window_midpoint/v1"
STEP_M = 0.5
BLEND_M = 5.0          # transition from the centreline onto the midpoint, along the window
ROAD_ROLES = ("approach", "departure")


def _resample(line, step):
    line = np.asarray(line, float)
    seg = np.linalg.norm(np.diff(line, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= step:
        return line
    q = np.arange(0.0, s[-1], step)
    if s[-1] - q[-1] > 1e-9:
        q = np.append(q, s[-1])
    return np.column_stack([np.interp(q, s, line[:, 0]), np.interp(q, s, line[:, 1])])


def apply(manifest: dict, src) -> dict:
    """Replace the comparison windows (``manifest`` in place) of the road lanes the rule applies to; returns the
    report."""
    from mapforge.validate import shp_source_review as R
    from mapforge.validate.g8_model import geometry_sha256, object_sha256
    origin = manifest["comparison_crs"]["origin"]
    rows = []
    for lane in manifest.get("lanes", []):
        if lane.get("role") not in ROAD_ROLES or not (lane.get("comparison") or {}).get("eligible"):
            continue
        sid = lane["source_lane_id"]
        rec = src.lane(sid)
        if rec is None or len(rec.geometry) < 2:
            continue
        bnds = [R._local(g, origin) for g in src.lane_boundary_geometries(sid)]
        judged = R.off_midpoint(rec, R._local(rec.geometry, origin), bnds)
        if judged is None:
            continue
        w = _resample(np.asarray(lane["geometry"]["coordinates"], float)[:, :2], STEP_M)
        f1, in1 = R._foot(w, bnds[0])
        f2, in2 = R._foot(w, bnds[1])
        both = in1 & in2
        mid = np.where(both[:, None], 0.5 * (f1 + f2), w)
        off = np.where(both, np.linalg.norm(w - mid, axis=1), 0.0)
        far = off >= R.CENTRE_OFFSET_MIN_M
        if not far.any():
            continue                  # (the off stretch lies outside the compared part)
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))])
        gap = np.min(np.abs(s[:, None] - s[far][None, :]), axis=1)      # along the window to the off stretch
        x = np.clip(1.0 - gap / BLEND_M, 0.0, 1.0)
        weight = x * x * (3.0 - 2.0 * x)
        mid = w + weight[:, None] * (mid - w)
        lane["geometry"]["coordinates"] = mid.tolist()
        lane["geometry"]["geometry_sha256"] = geometry_sha256(lane["geometry"]["coordinates"])
        lane.setdefault("travel", {})
        lane["travel"]["start"], lane["travel"]["end"] = lane["geometry"]["coordinates"][0], lane["geometry"]["coordinates"][-1]
        change = {"code": CODE, "rule": "centreline-off-midpoint", "source_offset_max_m": round(float(judged[0].max()), 3),
                  "window_shift_max_m": round(float(np.linalg.norm(mid - w, axis=1).max()), 3),
                  "window_off_m": round(float(far.sum() * STEP_M), 1), "points_kept": int((~both).sum())}
        lane.setdefault("window_adjustments", []).append(change)
        rows.append({"source_lane_id": sid, "role": lane.get("role"), **{k: v for k, v in change.items() if k != "code"}})
    if rows:
        manifest.pop("manifest_sha256", None)
        manifest["manifest_sha256"] = object_sha256(manifest)
    return {"schema": CODE, "lanes": len(rows), "rows": rows}
