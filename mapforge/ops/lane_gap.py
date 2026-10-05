"""Gap lanes between the lanes of two parallel source links (SHP road sides, 2026-10-05).

The converter builds a road side from the lanes of one or more source links and puts them next to each other.
Where two links run side by side with a gap between them (shp-node13 road 10 left: lanes 3 and 4 belong to two
links whose boundaries lie 0.9-1.0 m apart over the whole road), the common edge of the two lanes could only
lie in the middle: both lanes came out about 0.5 m wider than their source and their centres about 0.25 m off,
and so were the 7 connectors that end on them (0.26-0.28 m off their source via lines at the mouth).

Here such a gap becomes a lane of its own (type ``restricted``: not to be driven; provenance INFERRED, support
kind ``link-gap``, exclusion code ``physical-edge-fill`` as the curb-return shoulders, so the frozen G8 policy knows
it; no source lane), between the two lanes, before the lane refit. The lanes outside it move out by one
id; lane links inside the road and the links of the junction connectors that end on (start from) the renumbered
lanes follow. The refit then fits each edge to its own source boundary (the gap lane has no source lane, so
each of its edges is observed from one side only) and the gap gets the source's width.

Only gaps that run over every section of a road side are handled: a gap lane that would have to start or end
inside the road needs a birth or a death of its own (none in the seven Jinfeng junctions; reported, left out).
"""
from __future__ import annotations

import json

import numpy as np
from lxml import etree

CODE = "mapforge.lane_gap/v1"
GAP_MIN_M = 0.3           # median separation of the two source boundaries that makes a gap lane
GAP_TYPE = "restricted"


def _side_lanes(sec, side):
    return sorted(sec.findall(f"{side}/lane"), key=lambda ln: abs(int(ln.get("id"))))


def _source_id(lane):
    u = lane.find("userData[@code='mapforge.source_lane']")
    return u.get("value") if u is not None else None


def _boundaries(road, src, origin, side, i, lane):
    """(inner, outer) source boundary samples (s, t) of one lane inside section i, ordered across the road."""
    from mapforge.ops.lane_refit import _project
    from mapforge.validate.shp_boundary_fidelity import _densify, _project as to_local
    sid = _source_id(lane)
    if not sid:
        return None
    s0, s1 = road.s[i], road.s[i + 1]
    cands = []
    for g in src.lane_boundary_geometries(sid):
        pts = _densify(to_local(g, *origin), 0.5)
        st = [(s, t) for s, t, clamped in _project(pts, *road.ref) if not clamped and s0 <= s <= s1]
        if len(st) >= 4:
            cands.append(np.asarray(st))
    if len(cands) != 2:
        return None
    mid = [float(np.median(c[:, 1])) for c in cands]
    order = np.argsort(mid) if side == "left" else np.argsort(mid)[::-1]
    return cands[order[0]], cands[order[-1]]


def separation(road, src, origin, side, i, k):
    """Median outward separation [m] between lane k's outer and lane k+1's inner source boundary in section i
    (positive: a gap, negative: an overlap), or None."""
    lanes = _side_lanes(road.sections[i], side)
    if k >= len(lanes):
        return None
    a = _boundaries(road, src, origin, side, i, lanes[k - 1])
    b = _boundaries(road, src, origin, side, i, lanes[k])
    if a is None or b is None:
        return None
    oa = a[1][np.argsort(a[1][:, 0])]          # source lines may run against s (left lanes)
    ib = b[0][np.argsort(b[0][:, 0])]
    lo, hi = max(oa[0, 0], ib[0, 0]), min(oa[-1, 0], ib[-1, 0])
    if hi - lo < 2.0:
        return None
    s = np.linspace(lo, hi, max(5, int((hi - lo) / 0.5)))
    d = np.interp(s, ib[:, 0], ib[:, 1]) - np.interp(s, oa[:, 0], oa[:, 1])
    return float(np.median(d if side == "left" else -d))


def find(road, src, origin):
    """[{"side", "k", "separation_m": [...per section]}]: lane pairs (k, k+1) of two different source links
    separated by more than GAP_MIN_M in every section of the road."""
    out = []
    for side in ("left", "right"):
        counts = {len(_side_lanes(sec, side)) for sec in road.sections}
        if len(counts) != 1:
            continue
        n = counts.pop()
        for k in range(1, n):
            seps, links = [], set()
            for i, sec in enumerate(road.sections):
                lanes = _side_lanes(sec, side)
                pair = (lanes[k - 1], lanes[k])
                if any(ln.get("type") != "driving" for ln in pair):
                    break
                ids = [_source_id(ln) for ln in pair]
                recs = [src.lane(x) if x else None for x in ids]
                if None in recs or recs[0].link_pid == recs[1].link_pid:
                    break
                links.add((recs[0].link_pid, recs[1].link_pid))
                sep = separation(road, src, origin, side, i, k)
                if sep is not None and sep < GAP_MIN_M:
                    break
                seps.append(sep)          # None: section too short to measure (shp-node13 road 10: 1.26 m)
            else:
                if any(x is not None for x in seps):
                    out.append({"side": side, "k": k,
                                "separation_m": [None if x is None else round(x, 3) for x in seps],
                                "links": sorted({x for pair in links for x in pair})})
    return out


def _shift_links(lane, k, sign):
    for tag in ("predecessor", "successor"):
        el = lane.find(f"link/{tag}")
        if el is not None and abs(int(el.get("id"))) > k and int(el.get("id")) * sign > 0:
            el.set("id", str(int(el.get("id")) + sign))


def insert(root, road_el, side, k, separation_m=None):
    """Insert the gap lane between lanes k and k+1 of ``side`` in every section of ``road_el``."""
    sign = 1 if side == "left" else -1
    secs = road_el.findall("lanes/laneSection")
    for i, sec in enumerate(secs):
        side_el = sec.find(side)
        for ln in side_el.findall("lane"):
            if abs(int(ln.get("id"))) > k:
                ln.set("id", str(int(ln.get("id")) + sign))
            _shift_links(ln, k, sign)
        gap = etree.Element("lane", id=str(sign * (k + 1)), type=GAP_TYPE, level="false")
        link = etree.SubElement(gap, "link")
        if i > 0:
            etree.SubElement(link, "predecessor", id=str(sign * (k + 1)))
        if i < len(secs) - 1:
            etree.SubElement(link, "successor", id=str(sign * (k + 1)))
        etree.SubElement(gap, "width", sOffset="0", a="0", b="0", c="0", d="0")
        # no roadMark (as the converter's median lanes; OpenDRIVE 1.5 would need a colour for type "none")
        # G8 (frozen policy) knows physical-edge-fill: a non-driving lane that only fills physical space so the
        # driving lanes' edges follow their source boundaries (as the curb-return shoulders)
        prov = {"eligibility": "excluded", "exclusion_code": "physical-edge-fill", "status": "INFERRED",
                "support_kind": "link-gap", "travel_direction": "with_s" if side == "right" else "against_s"}
        etree.SubElement(gap, "userData", code="mapforge.provenance/v1",
                         value=json.dumps(prov, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        etree.SubElement(gap, "userData", code=CODE,
                         value=json.dumps({"between": [k, k + 1], "separation_m": separation_m}))
        # left lanes are listed outermost first, right lanes innermost first
        lanes = side_el.findall("lane")
        inner = next(ln for ln in lanes if abs(int(ln.get("id"))) == k)
        pos = list(side_el).index(inner)
        side_el.insert(pos if side == "left" else pos + 1, gap)
    # junction connectors ending on (starting from) this road's renumbered lanes
    rid = road_el.get("id")
    for cr in root.findall("road"):
        if cr.get("junction") in (None, "-1"):
            continue
        link = cr.find("link")
        if link is None:
            continue
        for tag in ("predecessor", "successor"):
            el = link.find(tag)
            if el is None or el.get("elementType") != "road" or el.get("elementId") != rid:
                continue
            for ln in cr.findall("lanes/laneSection/*/lane"):
                lk = ln.find(f"link/{tag}")
                if lk is not None and abs(int(lk.get("id"))) > k and int(lk.get("id")) * sign > 0:
                    lk.set("id", str(int(lk.get("id")) + sign))
    # junction laneLinks where this road is the incoming road
    for conn in root.iter("connection"):
        if conn.get("incomingRoad") == rid:
            for ll in conn.findall("laneLink"):
                v = int(ll.get("from"))
                if abs(v) > k and v * sign > 0:
                    ll.set("from", str(v + sign))
    # ordinary neighbouring roads linked road-to-road are not handled (none in the seven junctions)


def apply(root, src):
    """Gap lanes on every ordinary road (before the lane refit); returns {road id: [gaps]}."""
    from mapforge.ops.lane_refit import _Road
    from mapforge.validate.shp_boundary_fidelity import _origin
    origin = _origin(root)
    out = {}
    for road_el in root.findall("road"):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        link = road_el.find("link")
        if link is not None and any(el.get("elementType") == "road" for el in link):
            continue          # road-to-road links would need their lane links renumbered too
        gaps = find(_Road(road_el), src, origin)
        # outer gaps first, so the inner ones keep their lane ids
        for g in sorted(gaps, key=lambda g: -g["k"]):
            insert(root, road_el, g["side"], g["k"], g["separation_m"])
        if gaps:
            out[road_el.get("id")] = gaps
    return out
