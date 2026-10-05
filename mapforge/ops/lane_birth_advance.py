"""Lane births and deaths moved half a corner ahead of their source events (SHP, 2026-10-04).

A lane born inside a road starts with zero width and zero width slope; otherwise its outer edge would kink.
Many SHP tapers open at full slope right at the birth point (shp-node17 road 12 lane -3: 0.17-0.2), so the
lane refit's birth blend, which can only start at the birth, needed max(12 m, 5.33 |dm| / kappa) = 24 m and
the outer edge lagged the source taper by 0.4-0.65 m (|dm| * length / 4).

Here the new lane is made to exist ``d`` = |dm| / kappa earlier (clipped to ADVANCE_MIN_M .. ADVANCE_MAX_M):
the section before the birth is split at s_b - d, and the new short section carries the new lane with zero
width next to the continuing lanes (their ids, widths, marks and provenance taken over from the section
before). The lane refit then rounds the source corner symmetrically, a corner of half length d at the
ordinary curvature target (deviation |dm| * d / 6, about 0.13 m for |dm| = 0.18). Deaths are mirrored (the
section after the death is split at s_e + d). The new pieces are marked (CODE) and carry the exclusion code
source-extension: they have no source support, so G8 and the boundary gate leave them out.

Where the neighbouring section is too short to split (SECTION_MIN_M must stay), an outermost lane is carried
with zero width through whole sections and the section beyond is split (_plan): 12 of 28 events in the seven
SHP junctions sit next to a 1-4 m section (shp-node17 road 13 lane -3: 3.28 m; its taper lagged 0.32 m).
"""
from __future__ import annotations

import copy
import json

import numpy as np
from lxml import etree

from mapforge.ops.map_far_end import _rebase

CODE = "mapforge.lane_birth_advance/v1"
ADVANCE_MIN_M = 3.0
ADVANCE_MAX_M = 8.0
SECTION_MIN_M = 2.0       # the split section keeps at least this much
ZERO_WIDTH_M = 0.05       # a lane end narrower than this is a birth/death
SLOPE_MIN = 0.02          # gentler source openings are left alone
SLOPE_WINDOW_M = 6.0      # source opening slope measured over this much after (before) the event
LANE_RECORDS = ("width", "roadMark", "material", "speed", "access", "height", "rule")   # OpenDRIVE child order


def _f(x):
    return repr(float(x))


def _width_at(lane, u):
    recs = sorted(lane.findall("width"), key=lambda w: float(w.get("sOffset")))
    rec = None
    for w in recs:
        if float(w.get("sOffset")) <= u + 1e-9:
            rec = w
    if rec is None:
        return 0.0
    du = u - float(rec.get("sOffset"))
    a, b, c, d = (float(rec.get(k, 0.0)) for k in ("a", "b", "c", "d"))
    return a + du * (b + du * (c + du * d))


def _sections(road_el):
    secs = road_el.findall("lanes/laneSection")
    s = [float(x.get("s")) for x in secs] + [float(road_el.get("length"))]
    return secs, s


def _lane(sec, lane_id):
    for side in ("left", "right"):
        for ln in sec.findall(f"{side}/lane"):
            if int(ln.get("id")) == lane_id:
                return ln
    return None


def _link_id(lane, tag):
    el = lane.find(f"link/{tag}")
    return int(el.get("id")) if el is not None else None


def _set_link(lane, tag, lane_id):
    link = lane.find("link")
    if link is None:
        link = etree.Element("link")
        lane.insert(0, link)
    el = link.find(tag)
    if lane_id is None:
        if el is not None:
            link.remove(el)
        return
    if el is None:
        el = etree.SubElement(link, tag)
        if tag == "predecessor" and link.find("successor") is not None:
            link.remove(el)
            link.insert(0, el)
    el.set("id", str(lane_id))


def events(road_el):
    """[{"kind": "birth"|"death", "section": i, "side", "lane": id, "s"}]: zero-width lane ends inside the road."""
    secs, s = _sections(road_el)
    out = []
    for i, sec in enumerate(secs):
        length = s[i + 1] - s[i]
        for side in ("left", "right"):
            for ln in sec.findall(f"{side}/lane"):
                lid = int(ln.get("id"))
                if i > 0 and _link_id(ln, "predecessor") is None and abs(_width_at(ln, 0.0)) < ZERO_WIDTH_M:
                    out.append({"kind": "birth", "section": i, "side": side, "lane": lid, "s": s[i]})
                if (i < len(secs) - 1 and _link_id(ln, "successor") is None
                        and abs(_width_at(ln, length)) < ZERO_WIDTH_M):
                    out.append({"kind": "death", "section": i, "side": side, "lane": lid, "s": s[i + 1]})
    return out


def _copy_records(src_lane, dst_lane, u0, u1, shift):
    """Records of ``src_lane`` active in [u0, u1) into ``dst_lane`` (replacing its own), local origin u0 - shift."""
    for tag in LANE_RECORDS:
        for el in dst_lane.findall(tag):
            dst_lane.remove(el)
    anchor = len(dst_lane.findall("link"))
    out = []
    for tag in LANE_RECORDS:
        recs = sorted(src_lane.findall(tag), key=lambda r: float(r.get("sOffset", 0.0)))
        active = [r for r in recs if float(r.get("sOffset", 0.0)) <= u0 + 1e-9]
        inside = [r for r in recs if u0 + 1e-9 < float(r.get("sOffset", 0.0)) < u1 - 1e-9]
        chosen = ([active[-1]] if active else []) + inside
        for r in chosen:
            new = copy.deepcopy(r)
            off = float(r.get("sOffset", 0.0))
            if off < u0 and tag == "width":
                coef = tuple(float(r.get(k, 0.0)) for k in ("a", "b", "c", "d"))
                for k, v in zip(("a", "b", "c", "d"), _rebase(*coef, u0 - off)):
                    new.set(k, _f(v))
            new.set("sOffset", _f(max(off, u0) - u0 + shift))
            out.append(new)
    for j, el in enumerate(out):
        dst_lane.insert(anchor + j, el)


def _truncate(lane, u_end):
    for tag in LANE_RECORDS:
        for r in lane.findall(tag):
            if float(r.get("sOffset", 0.0)) >= u_end - 1e-9 and float(r.get("sOffset", 0.0)) > 1e-9:
                lane.remove(r)


def _shift_records(lane, d):
    """Section start moved forward by d: the record active at d is re-expressed there, later ones move by -d."""
    for tag in LANE_RECORDS:
        recs = sorted(lane.findall(tag), key=lambda r: float(r.get("sOffset", 0.0)))
        active = [r for r in recs if float(r.get("sOffset", 0.0)) <= d + 1e-9]
        for r in recs:
            off = float(r.get("sOffset", 0.0))
            if off <= d + 1e-9 and r is not active[-1]:
                lane.remove(r)
            elif r is active[-1]:
                if tag == "width":
                    coef = tuple(float(r.get(k, 0.0)) for k in ("a", "b", "c", "d"))
                    for k, v in zip(("a", "b", "c", "d"), _rebase(*coef, d - off)):
                        r.set(k, _f(v))
                r.set("sOffset", _f(0.0))
            else:
                r.set("sOffset", _f(off - d))


def _mark_extension(lane, event, advance):
    """Provenance of the new piece: no source support (source-extension), plus the advance record."""
    prov = lane.find("userData[@code='mapforge.provenance/v1']")
    record = json.loads(prov.get("value")) if prov is not None and prov.get("value") else {}
    record.update({"eligibility": "excluded", "exclusion_code": "source-extension", "status": "INFERRED",
                   "support_kind": "lane-transition-taper"})
    record.pop("support_s", None)
    record.pop("boundary_evidence_trusted", None)
    if prov is None:
        prov = etree.SubElement(lane, "userData", code="mapforge.provenance/v1")
    prov.set("value", json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
    key = "source_birth_s" if event["kind"] == "birth" else "source_death_s"
    etree.SubElement(lane, "userData", code=CODE, value=json.dumps({key: event["s"], "advance_m": advance}))


def _zero_width(lane):
    for w in lane.findall("width"):
        lane.remove(w)
    w = etree.Element("width", sOffset="0.0", a="0.0", b="0.0", c="0.0", d="0.0")
    lane.insert(len(lane.findall("link")), w)


def _find(road_el, event):
    """Index of the section the event belongs to now (birth: the section starting at the event station, death:
    the one ending there), or None."""
    secs, s = _sections(road_el)
    for i in range(len(secs)):
        at = s[i] if event["kind"] == "birth" else s[i + 1]
        if abs(at - event["s"]) < 1e-6 and _lane(secs[i], event["lane"]) is not None:
            return i
    return None


def _plan(road_el, event, i, d):
    """[("whole", j), ...] plus an optional ("split", j, amount), and the total advance (<= d).

    Whole sections are taken only for an outermost lane whose side keeps the same lanes there (the new lane is
    then simply added outside); a split leaves SECTION_MIN_M of the section. The road's first (last) section
    is never taken whole: the lane would start at a junction mouth."""
    secs, s = _sections(road_el)
    side, k = event["side"], abs(event["lane"])
    tag = "predecessor" if event["kind"] == "birth" else "successor"
    ends_here = sum(1 for sd in ("left", "right") for ln in secs[i].findall(f"{sd}/lane") if _link_id(ln, tag) is None)
    # an outermost lane, and the only lane starting (ending) here: the others would miss the whole sections
    outermost = len(secs[i].findall(f"{side}/lane")) == k and ends_here == 1
    step = -1 if event["kind"] == "birth" else 1
    plan, need, total, j = [], d, 0.0, i + step
    while 0 <= j < len(secs) and need > 1e-9:
        room = s[j + 1] - s[j]
        if room - SECTION_MIN_M >= need:
            plan.append(("split", j, need))
            total += need
            break
        end_section = j == 0 if event["kind"] == "birth" else j == len(secs) - 1
        if not outermost or end_section or len(secs[j].findall(f"{side}/lane")) != k - 1:
            if room - SECTION_MIN_M > 0.5:
                plan.append(("split", j, room - SECTION_MIN_M))
                total += room - SECTION_MIN_M
            break
        plan.append(("whole", j))
        total += room
        need -= room
        j += step
    return plan, total


def _add_zero_lane(road_el, j, event, total):
    """Add the event lane with zero width to the whole section j (outside the existing lanes of its side)."""
    secs, _ = _sections(road_el)
    side, lid = event["side"], event["lane"]
    birth = event["kind"] == "birth"
    neighbour = secs[j + 1] if birth else secs[j - 1]
    template = _lane(neighbour, lid)
    new = copy.deepcopy(template)
    for u in new.findall(f"userData[@code='{CODE}']"):
        new.remove(u)
    _copy_records(template, new, 0.0, 1e-6, 0.0)
    _zero_width(new)
    _mark_extension(new, event, total)
    _set_link(new, "predecessor", None if birth else lid)
    _set_link(new, "successor", lid if birth else None)
    _set_link(template, "predecessor" if birth else "successor", lid)
    side_el = secs[j].find(side)
    if side_el is None:
        side_el = etree.SubElement(secs[j], side)
    if side == "left":
        side_el.insert(0, new)
    else:
        side_el.append(new)


def _set_advance(road_el, event, total):
    """The advance record on every extension piece of the event lane carries the whole advance."""
    for sec in road_el.findall("lanes/laneSection"):
        lane = _lane(sec, event["lane"])
        mark = lane.find(f"userData[@code='{CODE}']") if lane is not None else None
        if mark is None:
            continue
        value = json.loads(mark.get("value"))
        key = "source_birth_s" if event["kind"] == "birth" else "source_death_s"
        if abs(value.get(key, -1e9) - event["s"]) < 1e-6:
            value["advance_m"] = total
            mark.set("value", json.dumps(value))


def extend(road_el, event, d):
    """Advance one event by up to d (through whole short sections where needed); returns the advance made."""
    i = _find(road_el, event)
    if i is None:
        raise ValueError("event section not found")
    plan, total = _plan(road_el, event, i, d)
    if total < ADVANCE_MIN_M - 1e-9:
        return 0.0
    birth = event["kind"] == "birth"
    last = i
    for item in plan:
        if item[0] == "whole":
            _add_zero_lane(road_el, item[1], event, total)
            last = item[1]
        else:
            # split the section next to the last piece, which now holds the zero-width lane end
            (split_birth if birth else split_death)(road_el, {**event, "section": last}, item[2])
    _set_advance(road_el, event, total)
    return total


def split_birth(road_el, event, d):
    """Insert the section [s_b - d, s_b) with the new lane at zero width (layout of the birth section)."""
    secs, s = _sections(road_el)
    i = event["section"]
    prev, born_sec = secs[i - 1], secs[i]
    u0, u1 = s[i] - d - s[i - 1], s[i] - s[i - 1]
    new = copy.deepcopy(born_sec)
    new.set("s", _f(s[i] - d))
    for side in ("left", "right"):
        for ln in new.findall(f"{side}/lane"):
            lid = int(ln.get("id"))
            if _link_id(_lane(born_sec, lid), "predecessor") is None:
                # new here (the event lane, or another lane born at the same point)
                _copy_records(_lane(born_sec, lid), ln, 0.0, d, 0.0)
                _zero_width(ln)
                _mark_extension(ln, {**event, "lane": lid}, d)
                _set_link(ln, "predecessor", None)
                _set_link(ln, "successor", lid)
                continue
            p = _link_id(_lane(born_sec, lid), "predecessor")
            src_lane = _lane(prev, p)
            if src_lane is None:
                raise ValueError(f"lane {lid}: predecessor {p} missing")
            for u in list(ln.findall("userData")):
                ln.remove(u)
            for u in src_lane.findall("userData"):
                ln.append(copy.deepcopy(u))
            _copy_records(src_lane, ln, u0, u1, 0.0)
            _set_link(ln, "predecessor", p)
            _set_link(ln, "successor", lid)
    centre_prev, centre_new = prev.find("center/lane"), new.find("center/lane")
    if centre_prev is not None and centre_new is not None:
        _copy_records(centre_prev, centre_new, u0, u1, 0.0)
    for side in ("left", "right"):
        for ln in prev.findall(f"{side}/lane"):
            _truncate(ln, u0)
    if centre_prev is not None:
        _truncate(centre_prev, u0)
    for side in ("left", "right"):
        for ln in born_sec.findall(f"{side}/lane"):
            lid = int(ln.get("id"))
            _set_link(ln, "predecessor", lid)
    lanes_el = road_el.find("lanes")
    lanes_el.insert(list(lanes_el).index(born_sec), new)
    return new


def split_death(road_el, event, d):
    """Insert the section [s_e, s_e + d) with the ending lane at zero width (layout of the death section)."""
    secs, s = _sections(road_el)
    i = event["section"]
    dying_sec, nxt = secs[i], secs[i + 1]
    new = copy.deepcopy(dying_sec)
    new.set("s", _f(s[i + 1]))
    for side in ("left", "right"):
        for ln in new.findall(f"{side}/lane"):
            lid = int(ln.get("id"))
            if _link_id(_lane(dying_sec, lid), "successor") is None:
                length = s[i + 1] - s[i]
                _copy_records(_lane(dying_sec, lid), ln, length - 1e-6, length + d, 0.0)
                _zero_width(ln)
                _mark_extension(ln, {**event, "lane": lid}, d)
                _set_link(ln, "predecessor", lid)
                _set_link(ln, "successor", None)
                continue
            q = _link_id(_lane(dying_sec, lid), "successor")
            src_lane = _lane(nxt, q)
            if src_lane is None:
                raise ValueError(f"lane {lid}: successor {q} missing")
            for u in list(ln.findall("userData")):
                ln.remove(u)
            for u in src_lane.findall("userData"):
                ln.append(copy.deepcopy(u))
            _copy_records(src_lane, ln, 0.0, d, 0.0)
            _set_link(ln, "predecessor", lid)
            _set_link(ln, "successor", q)
    centre_dying, centre_new, centre_next = (dying_sec.find("center/lane"), new.find("center/lane"),
                                             nxt.find("center/lane"))
    if centre_next is not None and centre_new is not None:
        _copy_records(centre_next, centre_new, 0.0, d, 0.0)
    for side in ("left", "right"):
        for ln in nxt.findall(f"{side}/lane"):
            _shift_records(ln, d)
    if centre_next is not None:
        _shift_records(centre_next, d)
    nxt.set("s", _f(s[i + 1] + d))
    for side in ("left", "right"):
        for ln in dying_sec.findall(f"{side}/lane"):
            lid = int(ln.get("id"))
            _set_link(ln, "successor", lid)
    lanes_el = road_el.find("lanes")
    lanes_el.insert(list(lanes_el).index(nxt), new)
    return new


def _opening_slope(observe, event):
    """|outer - inner| boundary slope of the event lane over SLOPE_WINDOW_M after a birth (before a death),
    from source observations, or None."""
    i, side, k = event["section"], event["side"], abs(event["lane"])
    outer, inner = observe(side, i, k), observe(side, i, k - 1)
    if outer is None or inner is None:
        return None
    if event["kind"] == "birth":
        lo, hi = event["s"], event["s"] + SLOPE_WINDOW_M
    else:
        lo, hi = event["s"] - SLOPE_WINDOW_M, event["s"]
    slopes = []
    for obs in (outer, inner):
        sel = obs[(obs[:, 0] >= lo) & (obs[:, 0] <= hi)]
        if len(sel) < 3 or np.ptp(sel[:, 0]) < 2.0:
            return None
        slopes.append(float(np.polyfit(sel[:, 0], sel[:, 1], 1)[0]))
    return abs(slopes[0] - slopes[1])


def advance(road_el, observe, kappa_cap):
    """Split sections so births/deaths start ``d`` ahead of their source events; returns a report.

    ``observe(side, i, k)``: (s, t) source samples of boundary k in section i (lane_refit observations)."""
    report = []
    pending = events(road_el)
    sized = []
    for ev in pending:
        dm = _opening_slope(observe, ev)
        if dm is None or dm < SLOPE_MIN:
            continue
        sized.append((ev, dm))
    # one event per section boundary (the steepest; a birth and a death at the same point would split both
    # neighbouring sections), splits from the far end so earlier section indices stay valid
    best = {}
    for ev, dm in sized:
        key = round(ev["s"], 6)
        if key not in best or dm > best[key][1]:
            best[key] = (ev, dm)
    taken = []      # station ranges already given to an advance
    for ev, dm in sorted(best.values(), key=lambda item: -item[0]["s"]):
        d = min(max(dm / kappa_cap, ADVANCE_MIN_M), ADVANCE_MAX_M)
        span = (ev["s"] - d, ev["s"]) if ev["kind"] == "birth" else (ev["s"], ev["s"] + d)
        free = span
        for lo, hi in taken:
            if ev["kind"] == "birth" and lo < ev["s"] and hi > free[0]:
                free = (max(free[0], hi), free[1])
            if ev["kind"] == "death" and hi > ev["s"] and lo < free[1]:
                free = (free[0], min(free[1], lo))
        d = free[1] - free[0]
        made = 0.0
        if d >= ADVANCE_MIN_M:
            try:
                made = extend(road_el, ev, d)
            except ValueError as exc:
                report.append({**ev, "slope": round(dm, 3), "advanced_m": 0.0, "reason": str(exc)})
                continue
        if made <= 0.0:
            report.append({**ev, "slope": round(dm, 3), "advanced_m": 0.0, "reason": "no room"})
            continue
        taken.append((ev["s"] - made, ev["s"]) if ev["kind"] == "birth" else (ev["s"], ev["s"] + made))
        report.append({**ev, "slope": round(dm, 3), "advanced_m": round(made, 3)})
    return report
