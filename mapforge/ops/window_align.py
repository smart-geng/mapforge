"""SHP comparison windows aligned to the written lanes at road ends (user decision 2026-10-05).

The converter cuts every lane's source comparison window (the source-lanes manifest G8 reads) at stations of the
original road axis (shp_to_xodr._clip_support, bound by evidence), while the written road ends where its fitted
reference line ends. The two disagree by 0.1-1 m at road ends: approach lanes' windows run past the road end,
departure lanes' windows begin beyond it, connector windows start or stop about 0.1 m off their connector's ends.
G8 then measures the overhang as a deviation (shp-node16 road 12 lane 2: 0.97 m at its far end) and, as it samples
both lines every metre from their own first points, an offset start shifts every sample of the lane (road 12
lane 3: G8 P95 0.446 m for a lane 0.006 m off its source, point to polyline). The geometry is not changed.

Here, after the post-process (the written lanes are final) and before the gates, every G8-comparable lane's window
is aligned at each end where its target lies on a road end (junction mouth or far end): the window is cut at the
cross-section of that road end, or, where it stops short of it, continued straight along its own end direction,
by at most ALIGN_MAX_M. Target ends inside a road (a lane birth or death, support clipped by the converter) are
left alone, so a target that really stops short still shows in G8's coverage. Every change is recorded on the
window (``window_adjustments``) and the manifest hash is renewed.
"""
from __future__ import annotations

import math

import numpy as np

CODE = "mapforge.window_align/v1"
ALIGN_MAX_M = 1.0     # a window end is cut or continued by at most this
ON_END_M = 0.05       # a target end this close to a road end's cross-section lies at that road end
MIN_CHANGE_M = 1e-3   # smaller differences are left as they are


def _road_ends(root):
    """{road id: {"start"|"end": (point, unit heading)}} of every road but the junction paving."""
    from mapforge.validate.smoothness import sample_road_ref
    out = {}
    for road in root.findall("road"):
        if road.get("name") == "junction_paving":
            continue
        pts, _, hh = sample_road_ref(road, 0.5)
        out[road.get("id")] = {c: (pts[j], np.array([math.cos(hh[j]), math.sin(hh[j])]))
                               for c, j in (("start", 0), ("end", -1))}
    return out


def _signed(points, origin, normal):
    return (np.asarray(points, float) - origin) @ normal


def _inner(line, at_least=0.1):
    """The point of ``line`` (this end last) nearest to its end but at least ``at_least`` from it (the end samples
    of a lane may repeat)."""
    d = np.linalg.norm(line - line[-1], axis=1)
    far = np.flatnonzero(d >= at_least)
    return line[far[-1]] if len(far) else line[0]


def _align_end(w, tgt_end, tgt_next, plane_point, plane_normal):
    """Window ``w`` (travel order, this end last) aligned to the plane through ``plane_point`` across
    ``plane_normal``; ``tgt_end``/``tgt_next``: the target's end point and its neighbour inside. Returns
    (new window, cut length, extension length) or None when nothing changes or the change exceeds ALIGN_MAX_M."""
    sigma = 1.0 if float((tgt_end - tgt_next) @ plane_normal) > 0.0 else -1.0
    f = sigma * _signed(w, plane_point, plane_normal)          # > 0: past the target's end, outside the road
    if f[-1] > MIN_CHANGE_M:
        j = int(np.flatnonzero(f <= 0.0)[-1]) if (f <= 0.0).any() else None
        if j is None:
            return None
        u = -f[j] / (f[j + 1] - f[j])
        p = w[j] + u * (w[j + 1] - w[j])
        cut = float(np.sum(np.linalg.norm(np.diff(np.vstack([p, w[j + 1:]]), axis=0), axis=1)))
        if cut > ALIGN_MAX_M:
            return None
        return np.vstack([w[:j + 1], p]), cut, 0.0
    if f[-1] < -MIN_CHANGE_M:
        v = w[-1] - _inner(w)
        v = v / max(float(np.linalg.norm(v)), 1e-12)
        rate = sigma * float(v @ plane_normal)
        if rate < 0.5:                                       # the window does not run towards the cross-section
            return None
        ext = -f[-1] / rate
        if ext > ALIGN_MAX_M:
            return None
        return np.vstack([w, w[-1] + ext * v]), 0.0, float(ext)
    return None


def apply(root, manifest: dict, ds: float = 1.0, zero_width_epsilon_m: float = 0.05) -> dict:
    """Align the comparable windows of ``manifest`` (in place) to the written lanes of ``root``; returns the report."""
    from mapforge.validate.g8_model import geometry_sha256, object_sha256
    from mapforge.validate.lane_fidelity import extract_target_components
    ends = _road_ends(root)
    comps = {}
    for comp in extract_target_components(root, ds=ds, zero_width_epsilon_m=zero_width_epsilon_m)["components"]:
        comps.setdefault(comp["source_lane_id"], []).append(comp)
    rows, skipped = [], []
    for lane in manifest.get("lanes", []):
        sid = lane["source_lane_id"]
        if not (lane.get("comparison") or {}).get("eligible") or len(comps.get(sid, [])) != 1:
            continue
        comp = comps[sid][0]
        t = np.asarray(comp["points"], float)[:, :2]
        coords = np.asarray(lane["geometry"]["coordinates"], float)
        if len(t) < 2 or len(coords) < 2:
            continue
        w = coords[:, :2]
        changed = []
        for which in ("start", "end"):
            tgt_end, tgt_next = (t[0], _inner(t[::-1])) if which == "start" else (t[-1], _inner(t))
            at = None
            for contact, (point, heading) in ends.get(comp["road_id"], {}).items():
                if abs(float((tgt_end - point) @ heading)) <= ON_END_M:
                    at = (contact, point, heading)
                    break
            if at is None:
                continue
            contact, point, heading = at
            ww = w if which == "end" else w[::-1]
            res = _align_end(ww, tgt_end, tgt_next, point, heading)
            if res is None:
                f = (1.0 if float((tgt_end - tgt_next) @ heading) > 0 else -1.0) * float((ww[-1] - point) @ heading)
                if abs(f) > MIN_CHANGE_M:
                    skipped.append({"source_lane_id": sid, "end": which, "road": comp["road_id"], "contact": contact,
                                    "offset_m": round(f, 3)})
                continue
            new, cut, ext = res
            w = new if which == "end" else new[::-1]
            changed.append({"code": CODE, "end": which, "road": comp["road_id"], "contact": contact,
                            **({"cut_m": round(cut, 3)} if cut else {"extended_m": round(ext, 3)})})
        if not changed:
            continue
        lane["geometry"]["coordinates"] = w.tolist()
        lane["geometry"]["geometry_sha256"] = geometry_sha256(lane["geometry"]["coordinates"])
        lane.setdefault("travel", {})
        lane["travel"]["start"], lane["travel"]["end"] = lane["geometry"]["coordinates"][0], lane["geometry"]["coordinates"][-1]
        lane.setdefault("window_adjustments", []).extend(changed)
        rows.append({"source_lane_id": sid, "road": comp["road_id"], "changes": changed})
    if rows:
        manifest.pop("manifest_sha256", None)
        manifest["manifest_sha256"] = object_sha256(manifest)
    cuts = [c.get("cut_m", 0.0) for r in rows for c in r["changes"]]
    exts = [c.get("extended_m", 0.0) for r in rows for c in r["changes"]]
    return {"schema": CODE, "lanes_aligned": len(rows), "ends_cut": sum(1 for x in cuts if x),
            "ends_extended": sum(1 for x in exts if x), "cut_max_m": max(cuts, default=0.0),
            "extended_max_m": max(exts, default=0.0), "skipped": skipped, "rows": rows}
