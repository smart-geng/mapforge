"""Opening slopes of SHP lane births and deaths whose own section is too short to measure them (2026-10-14).

lane_birth_advance sizes the advance of a birth (death) from the opening slope over SLOPE_WINDOW_M after (before)
the event, measured on the source observations of the event section alone, and leaves the event where those span
less than 2 m (``_opening_slope`` returns None). A birth whose section ends 1-2 m after it (another source link
starts there) was therefore never advanced, and the lane refit opened the lane from zero width right at the source
event: on 0630 road 12 the source lane 4 is 0.57 m wide 0.1 m after its birth, and the written boundary between it
and lane 3 ran up to 0.78 m off its source. The same holds for 0630 road 11 lane -3, 0627 road 12 lane 4 and
shp-node18 road 13 lane 4 (sections of 1.2-1.9 m).

``observer`` wraps the lane refit's per-section observations for ``lane_birth_advance.advance``: where a boundary's
observations in a section span less than MEASURE_MIN_M, they continue through the neighbouring sections along the
lane links, up to SLOPE_WINDOW_M beyond the section, and the window of ``_opening_slope`` takes the part after a
birth (before a death). A boundary measurable in its own section gets exactly what it got before.

The window must not cost a road what the refit otherwise makes of it. ``advance`` therefore also keeps the road as
the event sections' own observations advance it wherever the window advances more, and ``keep_own_way`` refits that
road once more and puts it back where
- the refit skips the road: it keeps its converter geometry, and a zero-width piece the window added would only put
  a curvature jump between it and the unchanged taper (0707104 road 10, 0.45 /m);
- the own-section road gets more curb-return shoulders: the refit fits a chain with an advanced event relative to its
  inner neighbour and makes a shoulder of a strong mouth flare only on an absolute fit, so advancing a birth whose
  lane runs on to such a mouth would keep the flare in the driving lane (061310 road 12);
- with as many shoulders, the road's farthest point from its source (the refit's own source observations, every
  boundary) lies farther than on the own-section road, by more than WORSE_M. A chain with an advanced event is fitted
  relative to its inner neighbour and hands over to its own fit of the source only between its events' corners, so
  one more event moves that hand-over and changes the chain's own fit all along: on shp-node4 road 13 the death the
  window advances at s 152.2 moved the end of lane 4's own fit from 152.2 to 146.4 and its outer boundary at s 75,
  77 m away, went from 0.375 to 0.394 m off the source (the scoreboard's boundary maximum from 0.345 to 0.366 m,
  over 0.35).
Such a road ends as it did before.
"""
from __future__ import annotations

import copy

import numpy as np
from lxml import etree

from mapforge.ops.lane_birth_advance import SLOPE_WINDOW_M

MEASURE_MIN_M = 2.0       # lane_birth_advance._opening_slope needs observations over at least this span ...
MEASURE_MIN_N = 3         # ... and at least this many
WORSE_M = 0.001           # a road farther from its source than this beyond the own-section road's goes back


def measurable(obs):
    """The observations are enough for lane_birth_advance._opening_slope on their own."""
    return obs is not None and len(obs) >= MEASURE_MIN_N and float(np.ptp(obs[:, 0])) >= MEASURE_MIN_M


def continued(road, side, i, k, step):
    """Index of the boundary in section i + step that continues boundary k of section i along the lane links (the
    outer boundary of the linked lane; 0, the centre, stays 0), or None where the lane has no link there."""
    if k == 0:
        return 0
    lanes = road.lanes(i, side)
    if k > len(lanes):
        return None
    link = lanes[k - 1].find("link/successor" if step > 0 else "link/predecessor")
    if link is None:
        return None
    nxt = abs(int(link.get("id")))
    return nxt if 1 <= nxt <= len(road.lanes(i + step, side)) else None


def observer(road, observe):
    """``observe(side, i, k)`` continued through the neighbouring sections where section i alone is too short."""
    def extended(side, i, k):
        own = observe(side, i, k)
        if own is None or not len(own) or measurable(own):
            return own
        parts = [own]
        for step in (1, -1):
            j, kk = i, k
            while 0 <= j + step < len(road.sections):
                kk = continued(road, side, j, kk, step)
                j += step
                if kk is None:
                    break
                gap = road.s[j] - road.s[i + 1] if step > 0 else road.s[i] - road.s[j + 1]
                if gap >= SLOPE_WINDOW_M:
                    break
                more = observe(side, j, kk)
                if more is not None and len(more):
                    parts.append(more)
        return np.vstack(parts)
    return extended


def advance(road_el, road, observe, kappa_cap):
    """``lane_birth_advance.advance`` of ``road_el`` (``road`` its lane_refit._Road) with the window: (events, None),
    or, where the window advances more than the event sections' own observations do, (events, (copy of the road
    advanced on those alone, its events))."""
    from mapforge.ops import lane_birth_advance
    own = copy.deepcopy(road_el)
    own_events = lane_birth_advance.advance(own, observe, kappa_cap)      # both measure before they split
    events = lane_birth_advance.advance(road_el, observer(road, observe), kappa_cap)
    if etree.tostring(own) == etree.tostring(road_el):
        return events, None
    return events, (own, own_events)


def _shoulders(row):
    return len(row.get("curb_return_shoulders") or [])


def worst(road_el, src, origin):
    """Largest |written - source| of a refitted road: every boundary of every section against the source observations
    the lane refit fits it to (lane_refit._source_observations), in m."""
    from mapforge.ops import lane_refit
    road = lane_refit._Road(road_el)
    out = 0.0
    for side in ("right", "left"):
        for i in range(len(road.sections)):
            for k in range(len(road.lanes(i, side)) + 1):
                try:
                    obs = lane_refit._source_observations(road, src, origin, side, i, k)
                except IndexError:
                    obs = None
                for s, t in ([] if obs is None else obs):
                    out = max(out, abs(road.current(i, side, k, s) - t))
    return float(out)


def keep_own_way(root, src, params, report, advanced, own_way):
    """Refit every road the window advanced more (``own_way``: road id -> the second item of ``advance``) once more as
    its event sections' own observations advance it, alone under a copy of the header and with ``params``, and put
    that road back where the refit skipped the window's road (``report["skipped"]``), or, both written
    (``report["rows"]``), where it gets more curb-return shoulders than the window's or, with as many, where the
    window's lies farther from its source (``worst``) by more than WORSE_M. Updates ``report`` (the road's entry
    replaced where it stands) and ``advanced`` (road id -> events); returns a note per road put back: why, the events
    only the window advanced and whether the second refit wrote the road (and both distances where that was why)."""
    from mapforge.ops import lane_refit
    from mapforge.validate.shp_boundary_fidelity import _origin
    skipped = {row["road"]: row for row in report["skipped"]}
    written = {row.get("road"): row for row in report["rows"]}
    notes = []
    for rid, (own, own_events) in own_way.items():
        alone = etree.Element(root.tag)
        alone.append(copy.deepcopy(root.find("header")))
        alone.append(own)
        again = lane_refit.refit_tree(alone, src, params)
        el = next(r for r in root.findall("road") if r.get("id") == rid)
        note = {}
        if rid in skipped:
            reason = "skipped"
            at = report["skipped"].index(skipped[rid])
            report["skipped"][at:at + 1] = again["skipped"]
            report["rows"].extend(again["rows"])
        elif rid in written and again["rows"]:
            mine, theirs = _shoulders(again["rows"][0]), _shoulders(written[rid])
            if mine > theirs:
                reason = "curb shoulder"
            elif mine < theirs:
                continue
            else:
                origin = _origin(root)
                far, near = worst(el, src, origin), worst(own, src, origin)
                if far <= near + WORSE_M:
                    continue
                reason, note = "source deviation", {"worst_m": {"window": round(far, 4), "own": round(near, 4)}}
            at = report["rows"].index(written[rid])
            report["rows"][at:at + 1] = again["rows"]
            report["skipped"].extend(again["skipped"])
            report["rewritten"] -= 1
        else:
            continue
        el.getparent().replace(el, own)
        report["rewritten"] += again["rewritten"]
        notes.append({"road": rid, "reason": reason,
                      "window_events": [ev for ev in advanced.get(rid, []) if ev not in own_events],
                      "refit": "written" if again["rewritten"] else "skipped", **note})
        if own_events:
            advanced[rid] = own_events
        else:
            advanced.pop(rid, None)
    return notes
