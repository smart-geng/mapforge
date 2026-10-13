"""Third boundary refit of an SHP road that lane_refit would leave unchanged for a negative lane width (2026-10-13).

After the zero-start retry (mapforge.ops.refit_zero_start) a road can still fail where a boundary carried on over a
free road end without source crosses its inner boundary. Generalization 0627 road 10 (461 m, free start, ends at the
junction): over its first 60 m the departure lanes have no source (the converter extended them, source-extension). The
refit carried the median's outer edge on from where its source starts as a straight line (t 3.58); its inner edge, the
centre boundary, follows the approach lane 35292 there from 3.94 to 3.22. The two crossed over s 0-15.9 (-0.357 m at
s=0). The median is 0.36 m wide where its source starts, so the zero-start rule does not apply; the whole road kept the
converter's geometry, its outer edges up to 1.72 m off their source boundaries section by section.

retry: the lanes whose width went negative keep, over a free chain end without source, the width they have where their
source starts (ends), as an offset from their inner boundary; past the source start the boundary hands over to its own
fit as after a zero-width start (lane_refit._advanced_fit). The zero-start lanes of the second refit stay as they were.
Written only when no width is negative any more, else the road stays skipped as before. Only roads that both earlier
refits leave skipped are refitted a third time, so no other output changes.
"""
from __future__ import annotations

import numpy as np

from mapforge.ops import lane_refit as LR


def holds(road, side, chain, obs, fitted, start, end, a, b):
    """{"birth"/"death": (station, None, width)}: at a free chain end (no end condition, no birth/death) with more than
    GRID of road before (after) its first (last) source sample, that station and the source width there relative to
    the fitted inner boundary (lane_refit's advanced-event form, held as the fill width)."""
    secs = sorted(chain["members"])
    inner = [fitted.get((side, i, chain["members"][i] - 1)) for i in secs]
    if not all(inner) or not len(obs):
        return {}
    base = [p for part in inner for p in part]
    lo, hi = float(np.min(obs[:, 0])), float(np.max(obs[:, 0]))
    out = {}
    if start is None and not chain.get("birth") and lo > a + LR.GRID:
        first = obs[obs[:, 0] <= lo + LR.GRID]
        out["birth"] = (lo, None, float(np.median(first[:, 1])) - LR._at(base, lo))
    if end is None and not chain.get("death") and hi < b - LR.GRID:
        last = obs[obs[:, 0] >= hi - LR.GRID]
        out["death"] = (hi, None, float(np.median(last[:, 1])) - LR._at(base, hi))
    return out


def retry(road_el, src, origin, params, report, zero_note):
    """(report of the third refit, written to ``road_el``, or None with the road unchanged; note on the attempt or
    None where none was made). ``zero_note``: the failed zero-start retry's note, whose lanes are refitted with their
    widths held. Only with ``protect_unobserved_ends`` (the SHP boundary step)."""
    if not (params or {}).get("protect_unobserved_ends") or not (zero_note or {}).get("lanes"):
        return None, None
    lanes = {(x["side"], x["section"], x["lane"]) for x in zero_note["lanes"]}
    note = {}
    try:
        road2, fitted2, again = LR.refit_road(road_el, src, origin,
                                              {**params, "zero_start_lanes": lanes, "hold_width_lanes": lanes})
        note["held"] = again.get("held_widths", [])
        if not note["held"]:
            note["result"] = "no free end without source"
            return None, note
        again["params"] = report["params"]
        again["hold_width_retry"] = {"lanes": zero_note["lanes"], "first_refit": report.get("skipped"),
                                     "zero_start_retry": zero_note.get("result")}
        if LR.write_road(road2, fitted2, again):
            return again, note
        note["result"] = again.get("skipped")
    except (ValueError, KeyError, StopIteration, IndexError) as exc:
        note["result"] = f"{type(exc).__name__}: {exc}"
    return None, note
