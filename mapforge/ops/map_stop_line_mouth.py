"""MAP junction mouths moved to the farthest stop line of each arm (2026-10-04).

The MAP converter ends an arm at one cross-section, through the stop line of the lane it builds the road
along; MAP stop lines are per lane (the last point of each lane point list, T/CSAE 159 Appendix D) and some
lie further into the junction: map-node18 east lane 1 1.38 m, north lane 3 0.89 m, south lane 3 0.85 m,
west lane 1 0.31 m; map-node3, NODE5 and node17 0.10-0.16 m. Those lanes stopped short of their stop line
(G8 travel-end deviation, map-node18 T2).

Here such an arm is extended to its farthest stop line (map_far_end: the last reference geometry continued
along its own curve, lane records continued, source support re-mapped). Every connector attached there is
cut back to the new mouth cross-section (it starts or ends on the new mouth line; mouth_frame_align then
matches it to the mouth exactly, as for any mouth). After the road-side refit the junction paving is rebuilt
on the final mouths with the converter's own construction (rounded mouth apron swept along up to two axes,
0.75 m overlap into each mouth), so the paving keeps meeting the roads as the converter intended. Lanes whose
stop line lies behind the new mouth run on to it; their G8 target still ends at their stop line (support_s).
"""
from __future__ import annotations

import json
import math

import numpy as np
from lxml import etree

from mapforge.ops import map_far_end

CODE = "mapforge.map_stop_line_mouth/v1"
PAVING_OVERLAP_M = 0.75   # as the converter (map_to_xodr: the inferred paving overlaps every mouth)


def _f(x):
    return repr(float(x))


def _ordinary(root):
    return [r for r in root.findall("road")
            if r.get("junction") in (None, "-1") and r.get("name") != "junction_paving"]


def _connectors(root):
    return [r for r in root.findall("road")
            if r.get("junction") not in (None, "-1") and r.get("name") != "junction_paving"]


def junction_ends(road_el):
    """{"start"/"end": junction id} of the road ends linked to a junction."""
    link = road_el.find("link")
    out = {}
    for tag, end in (("predecessor", "start"), ("successor", "end")):
        el = link.find(f"{tag}[@elementType='junction']") if link is not None else None
        if el is not None:
            out[end] = el.get("elementId")
    return out


# --- reference geometry ---------------------------------------------------------------

def _spiral_rate(geom):
    prim = geom[0]
    return (float(prim.get("curvEnd")) - float(prim.get("curvStart"))) / float(geom.get("length"))


def _pose_at(geom, u):
    """(x, y, heading, curvature) at arc length u inside one planView geometry (line, arc, spiral)."""
    x0, y0, h0 = (float(geom.get(k)) for k in ("x", "y", "hdg"))
    prim = geom[0]
    if prim.tag == "line":
        return x0 + u * math.cos(h0), y0 + u * math.sin(h0), h0, 0.0
    if prim.tag == "arc":
        k = float(prim.get("curvature"))
        if abs(k) < 1e-12:
            return x0 + u * math.cos(h0), y0 + u * math.sin(h0), h0, 0.0
        h = h0 + k * u
        return x0 + (math.sin(h) - math.sin(h0)) / k, y0 - (math.cos(h) - math.cos(h0)) / k, h, k
    if prim.tag == "spiral":
        k0, rate = float(prim.get("curvStart")), _spiral_rate(geom)
        v = np.linspace(0.0, u, 513)
        hv = h0 + k0 * v + 0.5 * rate * v * v
        w = np.ones_like(v)
        w[1:-1:2], w[2:-1:2] = 4.0, 2.0
        step = u / 512
        return (x0 + step / 3.0 * float(np.sum(w * np.cos(hv))), y0 + step / 3.0 * float(np.sum(w * np.sin(hv))),
                float(hv[-1]), k0 + rate * u)
    raise ValueError(f"cannot cut a {prim.tag} reference geometry")


def _cubic(el):
    return tuple(float(el.get(k, 0.0)) for k in ("a", "b", "c", "d"))


def _set_cubic(el, coef):
    for key, value in zip(("a", "b", "c", "d"), coef):
        el.set(key, _f(value))


def _cut_records(parent, tag, key, at, cubic):
    """Records ``tag`` of ``parent`` restarted at local position ``at``: the record covering it moves to 0
    (re-expressed there if ``cubic``), earlier ones go, later ones move back by ``at``."""
    recs = sorted(parent.findall(tag), key=lambda el: float(el.get(key)))
    covering = [el for el in recs if float(el.get(key)) <= at + 1e-9]
    for el in covering[:-1]:
        parent.remove(el)
    for el in recs:
        if covering and el is covering[-1]:
            if cubic:
                _set_cubic(el, map_far_end._rebase(*_cubic(el), at - float(el.get(key))))
            el.set(key, "0")
        elif float(el.get(key)) > at + 1e-9:
            el.set(key, _f(float(el.get(key)) - at))


def _drop_records_after(parent, tag, key, at):
    """Records starting at or after local position ``at`` go (one record is always kept)."""
    recs = sorted(parent.findall(tag), key=lambda el: float(el.get(key)))
    for el in recs[1:]:
        if float(el.get(key)) >= at - 1e-9:
            parent.remove(el)


LANE_RECORDS = (("width", True), ("border", True), ("roadMark", False), ("speed", False), ("material", False),
                ("access", False), ("height", False), ("rule", False))


def cut_start(road_el, d):
    """Remove the first ``d`` metres of a road (reference, lane records); the road then starts at s = d."""
    geoms = road_el.findall("planView/geometry")
    i = max(j for j, g in enumerate(geoms) if float(g.get("s")) <= d + 1e-9)
    g = geoms[i]
    u = d - float(g.get("s"))
    x, y, h, k = _pose_at(g, u)
    for old in geoms[:i]:
        old.getparent().remove(old)
    if g[0].tag == "spiral":
        g[0].set("curvStart", _f(k))
    g.set("x", _f(x)), g.set("y", _f(y)), g.set("hdg", _f(h)), g.set("length", _f(float(g.get("length")) - u))
    for other in geoms[i:]:
        other.set("s", _f(max(0.0, float(other.get("s")) - d)))
    road_el.set("length", _f(float(road_el.get("length")) - d))
    lanes = road_el.find("lanes")
    _cut_records(lanes, "laneOffset", "s", d, True)
    secs = lanes.findall("laneSection")
    keep = max(j for j, sec in enumerate(secs) if float(sec.get("s")) <= d + 1e-9)
    for sec in secs[:keep]:
        lanes.remove(sec)
    first = secs[keep]
    local = d - float(first.get("s"))
    for lane in first.iter("lane"):
        for tag, cubic in LANE_RECORDS:
            _cut_records(lane, tag, "sOffset", local, cubic)
    first.set("s", "0")
    for sec in secs[keep + 1:]:
        sec.set("s", _f(float(sec.get("s")) - d))
    for path in ("objects/object", "signals/signal"):
        for el in road_el.findall(path):
            el.set("s", _f(max(0.0, float(el.get("s")) - d)))


def cut_end(road_el, d):
    """Remove the last ``d`` metres of a road (reference, lane records)."""
    length = float(road_el.get("length")) - d
    geoms = road_el.findall("planView/geometry")
    kept = [g for g in geoms if float(g.get("s")) < length - 1e-9] or geoms[:1]
    last = kept[-1]
    if last[0].tag not in ("line", "arc", "spiral"):
        raise ValueError(f"cannot cut a {last[0].tag} reference geometry")
    new_len = length - float(last.get("s"))
    if last[0].tag == "spiral":
        last[0].set("curvEnd", _f(float(last[0].get("curvStart")) + _spiral_rate(last) * new_len))
    for g in geoms[len(kept):]:
        g.getparent().remove(g)
    last.set("length", _f(new_len))
    road_el.set("length", _f(length))
    lanes = road_el.find("lanes")
    _drop_records_after(lanes, "laneOffset", "s", length)
    secs = lanes.findall("laneSection")
    for sec in secs[1:]:
        if float(sec.get("s")) >= length - 1e-9:
            lanes.remove(sec)
    last_sec = lanes.findall("laneSection")[-1]
    local = length - float(last_sec.get("s"))
    for lane in last_sec.iter("lane"):
        for tag, _ in LANE_RECORDS:
            _drop_records_after(lane, tag, "sOffset", local)
    for path in ("objects/object", "signals/signal"):
        for el in road_el.findall(path):
            if float(el.get("s")) > length:
                el.getparent().remove(el)


def _crossing(road_el, point, heading, at_start):
    """Arc length (from the attached end) at which the reference line crosses the mouth line through
    ``point`` square to ``heading`` (pointing into the junction)."""
    from mapforge.validate.smoothness import sample_road_ref
    pts, ss, _ = sample_road_ref(road_el, 0.01)
    f = (pts - point) @ np.array([math.cos(heading), math.sin(heading)])
    if not at_start:
        pts, ss, f = pts[::-1], ss[-1] - ss[::-1], f[::-1]
    ahead = np.flatnonzero(f >= 0.0)
    if not len(ahead) or ahead[0] == 0:
        raise ValueError("connector does not cross the new mouth line")
    j = ahead[0]
    return float(ss[j - 1] + (ss[j] - ss[j - 1]) * (-f[j - 1]) / (f[j] - f[j - 1]))


# --- mouths and paving ----------------------------------------------------------------

def _attached(root, road_id, end):
    """(connector, at its start?) of the connectors linked to ``end`` of road ``road_id``."""
    out = []
    for conn in _connectors(root):
        link = conn.find("link")
        for tag, at_start in (("predecessor", True), ("successor", False)):
            el = link.find(tag) if link is not None else None
            if (el is not None and el.get("elementType") == "road" and el.get("elementId") == road_id
                    and el.get("contactPoint") == end):
                out.append((conn, at_start))
    return out


def _new_mouth(road_el, end, delta):
    """Mouth point and heading into the junction once ``end`` is extended by ``delta`` along its own curve."""
    geoms = road_el.findall("planView/geometry")
    if end == "end":
        g = geoms[-1]
        x, y, h, _ = _pose_at(g, float(g.get("length")) + delta)
        return np.array([x, y]), h
    back = map_far_end._pose_back(geoms[0], delta)
    if back is None:
        raise ValueError(f"cannot extend a {geoms[0][0].tag} reference geometry")
    return np.array(back[:2]), back[2] + math.pi


def apply(root, centres):
    """Extend every arm to its farthest stop line and cut the attached connectors back; returns a record.

    An arm is moved only when every connector attached there crosses the new mouth line (checked first, on
    the unchanged geometry); otherwise it stays where the converter put it and the reason is recorded."""
    from mapforge.validate.smoothness import sample_road_ref
    roads, cuts, junctions = [], [], set()
    for road_el in _ordinary(root):
        ends = junction_ends(road_el)
        gaps = map_far_end._end_gaps(road_el, centres, sample_road_ref(road_el, 0.25), list(ends)) if ends else {}
        plan = {}
        for end, per_lane in gaps.items():
            far = max(per_lane.values(), default=0.0)
            if far <= map_far_end.EXTEND_TOL_M or far > map_far_end.EXTEND_MAX_M:
                continue
            delta = math.ceil(far * 1000.0) / 1000.0
            try:
                point, heading = _new_mouth(road_el, end, delta)
                plan[end] = [(conn, at_start, _crossing(conn, point, heading, at_start))
                             for conn, at_start in _attached(root, road_el.get("id"), end)]
            except ValueError as exc:
                roads.append({"road": road_el.get("id"), end: {"not_extended_m": delta, "reason": str(exc)}})
        if not plan:
            continue
        rec = map_far_end.extend(road_el, centres, sample_road_ref(road_el, 0.25), ends=list(plan))
        roads.append({"road": road_el.get("id"), **(rec or {})})
        for end, todo in plan.items():
            if "extended_m" not in (rec or {}).get(end, {}):
                continue
            junctions.add(ends[end])
            for conn, at_start, d in todo:
                (cut_start if at_start else cut_end)(conn, d)
                cuts.append({"connector": conn.get("id"), "at": "start" if at_start else "end",
                             "road": road_el.get("id"), "cut_m": round(d, 4)})
    return {"schema": CODE, "roads": roads, "connector_cuts": cuts, "junctions": sorted(junctions)}


def _mouth(road_el):
    """Road end cross-section as map_to_xodr._written_road_mouth (outer edges with heading and curvature)."""
    from mapforge.validate.smoothness import (_edge_world_curvature, _ref_kappa_at, lane_edges_kinematics_at,
                                              sample_road_ref)
    pts, _, hh = sample_road_ref(road_el, 0.25)
    length = float(road_el.get("length"))
    kappa, sharpness = _ref_kappa_at(road_el, length)
    mouth = {"pose": (float(pts[-1][0]), float(pts[-1][1]), float(hh[-1]))}
    for side in ("right", "left"):
        t, dt, ddt = lane_edges_kinematics_at(road_el, length, side)[-1]
        mouth[side + "_t"] = float(t)
        mouth[side + "_heading"] = float(hh[-1] + math.atan2(dt, 1.0 - kappa * t))
        mouth[side + "_curvature"] = float(_edge_world_curvature(t, dt, ddt, kappa, sharpness))
    return mouth


def rebuild_paving(root, junction_ids):
    """Junction paving of ``junction_ids`` rebuilt on the final mouths with the converter's construction."""
    from xml.etree import ElementTree as ET
    from mapforge.adapters.opendrive import writer as W
    from mapforge.ops import map_to_xodr as MX
    done = []
    for jid in junction_ids:
        arms = [r for r in _ordinary(root) if jid in junction_ends(r).values()]
        if any(junction_ends(r).get("start") == jid for r in arms):
            done.append({"junction": jid, "rebuilt": False, "reason": "an arm starts at the junction"})
            continue
        mouths = [_mouth(r) for r in arms]
        apron = MX._rounded_mouth_apron(mouths)
        if apron is None:
            done.append({"junction": jid, "rebuilt": False, "reason": "fewer than 3 mouths"})
            continue
        old = [r for r in root.findall("road") if r.get("name") == "junction_paving" and r.get("junction") == jid]
        prov = old[0].find(".//userData[@code='mapforge.provenance/v1']") if old else None
        provenance = json.loads(prov.get("value")) if prov is not None and prov.get("value") else {
            "eligibility": "excluded", "role": "paving", "status": "INFERRED",
            "support_kind": "mouth-envelope-rounded", "travel_direction": "with_s", "exclusion_code": "inferred-paving"}
        doc = W.XodrDoc("paving")
        for i, axis in enumerate(MX._mouth_apron_axes(mouths) or [None]):
            W.add_paving_road(doc, apron, int(jid), road_id=90 + i, smooth_profile=True, preferred_axis=axis,
                              overlap_m=PAVING_OVERLAP_M, provenance=provenance)
        if not doc.roads:
            done.append({"junction": jid, "rebuilt": False, "reason": "degenerate apron"})
            continue
        tmp = ET.Element("OpenDRIVE")
        for rd in doc.roads:
            doc._road_el(tmp, rd)
        ET.indent(tmp, space="    ")
        new = [etree.fromstring(ET.tostring(el)) for el in tmp.findall("road")]
        at = list(root).index(old[0]) if old else len(root.findall("road"))
        tail = old[0].tail if old else "\n"
        for el in old:
            root.remove(el)
        for j, el in enumerate(new):
            el.tail = tail
            root.insert(at + j, el)
        done.append({"junction": jid, "rebuilt": True, "paving_roads": [el.get("id") for el in new]})
    return done
