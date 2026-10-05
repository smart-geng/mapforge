"""MAP road far ends reaching the first MAP lane points (2026-10-04).

The MAP converter starts an approach road at one lane's first point; the other lanes' first points are not
on that cross-section and some lie up to 0.7 m before it (map-node3 road 12 lane -1: 0.45 m, map-node16
road 13 lane -1: 0.71 m). Those lanes then begin late (G8 travel-start deviation), and because G8 samples
each line every metre from its own first point, the whole lane also measures about that much in G8.

At a free road end (no predecessor/successor link: the far end of an approach) whose MAP lane end points lie
beyond it, the road is extended to the farthest of them: the first (last) reference geometry is continued
along its own curve (line, arc or spiral; the reference stays one primitive, curvature continuous), the
records after it move by the extension, the first records are re-expressed from the new start (same
values, continued into the extension), and the source-support stations of the lanes follow. Junction ends
are extended only by map_stop_line_mouth, which also cuts the connectors attached there back.
"""
from __future__ import annotations

import json
import math

import numpy as np

CODE = "mapforge.map_far_end/v1"
EXTEND_TOL_M = 0.05     # smaller gaps are left alone
EXTEND_MAX_M = 3.0      # a lane end farther out is reported, not reached
SUPPORT_END_TOL_M = 1e-3   # a source support ending this close to a road end reaches it (the written
                           # road length is rounded: map-node18 road 13 ends 6e-6 m after its support)
LATERAL_MAX_M = 20.0    # end points farther to the side do not belong to this road end


def _f(x):
    return repr(float(x))


def _rebase(a, b, c, d, u):
    """Coefficients of the same cubic with its origin moved by u."""
    return (a + u * (b + u * (c + u * d)), b + u * (2 * c + 3 * d * u), c + 3 * d * u, d)


def _set_cubic(el, coef):
    for key, value in zip(("a", "b", "c", "d"), coef):
        el.set(key, _f(value))


def _cubic(el):
    return tuple(float(el.get(k, 0.0)) for k in ("a", "b", "c", "d"))


def _pose_back(geom, delta):
    """(x, y, hdg, new primitive attributes) of the first geometry continued backwards by delta, or None."""
    x0, y0, h0 = (float(geom.get(k)) for k in ("x", "y", "hdg"))
    prim = geom[0]
    if prim.tag == "line":
        return x0 - delta * math.cos(h0), y0 - delta * math.sin(h0), h0, {}
    if prim.tag == "arc":
        k = float(prim.get("curvature"))
        if abs(k) < 1e-12:
            return x0 - delta * math.cos(h0), y0 - delta * math.sin(h0), h0, {}
        h = h0 - k * delta
        return x0 + (math.sin(h) - math.sin(h0)) / k, y0 - (math.cos(h) - math.cos(h0)) / k, h, {}
    if prim.tag == "spiral":
        k0, k1 = float(prim.get("curvStart")), float(prim.get("curvEnd"))
        rate = (k1 - k0) / float(geom.get("length"))
        u = np.linspace(-delta, 0.0, 257)
        hu = h0 + k0 * u + 0.5 * rate * u * u
        w = np.ones_like(u)
        w[1:-1:2], w[2:-1:2] = 4.0, 2.0
        step = delta / 256
        x = x0 - step / 3.0 * float(np.sum(w * np.cos(hu)))
        y = y0 - step / 3.0 * float(np.sum(w * np.sin(hu)))
        return x, y, float(hu[0]), {"curvStart": _f(k0 - rate * delta)}
    return None


def _pose_forward_ok(geom):
    return geom[0].tag in ("line", "arc", "spiral")


def free_ends(road_el):
    """Road ends without a predecessor/successor link ("start", "end")."""
    link = road_el.find("link")
    return [end for end, tag in (("start", "predecessor"), ("end", "successor"))
            if link is None or link.find(tag) is None]


def _end_gaps(road_el, centres, ref, which):
    """Longitudinal distance by which MAP lane end points lie beyond the road ends ``which``, per lane id."""
    pts, ss, hh = ref
    ends = {}
    if "start" in which:
        ends["start"] = (pts[0], hh[0], -1.0)
    if "end" in which:
        ends["end"] = (pts[-1], hh[-1], 1.0)
    out = {end: {} for end in ends}
    for lane in road_el.iter("lane"):
        sid = lane.find("userData[@code='mapforge.source_lane']")
        if sid is None or sid.get("value") not in centres:
            continue
        line = centres[sid.get("value")]
        for end, (p0, h0, sign) in ends.items():
            tangent, normal = np.array([math.cos(h0), math.sin(h0)]), np.array([-math.sin(h0), math.cos(h0)])
            for p in (line[0], line[-1]):
                along, lateral = float((p - p0) @ tangent) * sign, float((p - p0) @ normal)
                if along > 0.0 and abs(lateral) < LATERAL_MAX_M:
                    key = sid.get("value")
                    out[end][key] = max(out[end].get(key, 0.0), along)
    return out


def _shift_records(parent, tag, key, delta, rebase_first):
    """Records ``tag`` under ``parent``: the one at 0 is re-expressed from -delta, the others move by delta."""
    for el in parent.findall(tag):
        if float(el.get(key)) <= 1e-9:
            if rebase_first:
                _set_cubic(el, _rebase(*_cubic(el), -delta))
        else:
            el.set(key, _f(float(el.get(key)) + delta))


def _extend_start(road_el, delta):
    geoms = road_el.findall("planView/geometry")
    back = _pose_back(geoms[0], delta)
    if back is None:
        return False
    x, y, h, attrs = back
    first = geoms[0]
    first.set("x", _f(x)), first.set("y", _f(y)), first.set("hdg", _f(h))
    first.set("length", _f(float(first.get("length")) + delta))
    for key, value in attrs.items():
        first[0].set(key, value)
    for g in geoms[1:]:
        g.set("s", _f(float(g.get("s")) + delta))
    road_el.set("length", _f(float(road_el.get("length")) + delta))
    lanes = road_el.find("lanes")
    _shift_records(lanes, "laneOffset", "s", delta, True)
    for i, sec in enumerate(lanes.findall("laneSection")):
        if i > 0:
            sec.set("s", _f(float(sec.get("s")) + delta))
            continue
        for lane in sec.iter("lane"):
            _shift_records(lane, "width", "sOffset", delta, True)
            _shift_records(lane, "border", "sOffset", delta, True)
            for tag in ("roadMark", "speed", "material", "access", "height", "rule"):
                _shift_records(lane, tag, "sOffset", delta, False)
    for path in ("elevationProfile/elevation", "lateralProfile/superelevation", "lateralProfile/shape"):
        parent_path, tag = path.rsplit("/", 1)
        parent = road_el.find(parent_path)
        if parent is not None:
            _shift_records(parent, tag, "s", delta, True)
    for path in ("objects/object", "objects/objectReference", "signals/signal", "signals/signalReference"):
        for el in road_el.findall(path):
            el.set("s", _f(float(el.get("s")) + delta))
    return True


def _extend_end(road_el, delta):
    geoms = road_el.findall("planView/geometry")
    last = geoms[-1]
    if not _pose_forward_ok(last):
        return False
    if last[0].tag == "spiral":
        k0, k1 = float(last[0].get("curvStart")), float(last[0].get("curvEnd"))
        rate = (k1 - k0) / float(last.get("length"))
        last[0].set("curvEnd", _f(k1 + rate * delta))
    last.set("length", _f(float(last.get("length")) + delta))
    road_el.set("length", _f(float(road_el.get("length")) + delta))
    return True


def _update_support(road_el, gaps, delta_start, delta_end, old_length):
    for lane in road_el.iter("lane"):
        prov = lane.find("userData[@code='mapforge.provenance/v1']")
        sid = lane.find("userData[@code='mapforge.source_lane']")
        if prov is None or not prov.get("value"):
            continue
        record = json.loads(prov.get("value"))
        support = record.get("support_s")
        if not (isinstance(support, list) and len(support) == 2):
            continue
        key = sid.get("value") if sid is not None else None
        lo, hi = float(support[0]), float(support[1])
        new_lo = (max(0.0, delta_start - gaps.get("start", {}).get(key, 0.0)) if lo <= SUPPORT_END_TOL_M and delta_start
                  else lo + delta_start)
        new_hi = hi + delta_start
        if delta_end and hi >= old_length - SUPPORT_END_TOL_M:
            new_hi = old_length + delta_start + min(delta_end, gaps.get("end", {}).get(key, 0.0))
        record["support_s"] = [new_lo, new_hi]
        prov.set("value", json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")))


def extend(road_el, centres, ref, ends=None):
    """Extend ``ends`` (default: the free ends) of one ordinary MAP road to the MAP lane end points beyond
    them; returns a record or None. Junction ends are extended only on request (map_stop_line_mouth)."""
    gaps = _end_gaps(road_el, centres, ref, free_ends(road_el) if ends is None else ends)
    record = {}
    delta = {}
    for end, per_lane in gaps.items():
        far = max(per_lane.values(), default=0.0)
        if far <= EXTEND_TOL_M:
            continue
        if far > EXTEND_MAX_M:
            record[end] = {"not_extended_m": round(far, 3), "reason": f"beyond {EXTEND_MAX_M} m"}
            continue
        delta[end] = math.ceil(far * 1000.0) / 1000.0
    if not delta:
        return record or None
    old_length = float(road_el.get("length"))
    done = {}
    if "start" in delta:
        done["start"] = _extend_start(road_el, delta["start"])
    if "end" in delta:
        done["end"] = _extend_end(road_el, delta["end"])
    applied_start = delta["start"] if done.get("start") else 0.0
    applied_end = delta["end"] if done.get("end") else 0.0
    _update_support(road_el, gaps, applied_start, applied_end, old_length)
    for end, ok in done.items():
        record[end] = ({"extended_m": delta[end], "lanes": {k: round(v, 3) for k, v in gaps[end].items()}}
                       if ok else {"not_extended_m": delta[end], "reason": "reference geometry type"})
    return record
