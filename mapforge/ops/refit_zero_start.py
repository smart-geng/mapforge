"""Second boundary refit of an SHP road that lane_refit would leave unchanged for a negative lane width (2026-10-12).

lane_refit keeps a road's converter geometry when a fitted lane width stays below -NEGATIVE_WIDTH_TOL after the
crossing repair (deeper than REPAIR_MAX_M, or at a road end, where the repair bump is zero). 0630 road 13 (source
lanes 74026 / 76005): the source boundaries of lane -3 leave together 0.40 m into the road, a lane opening from
zero width; fitted each on its own they run on back to the road start and cross there (-0.124 m), so the whole
road kept the converter's geometry, which follows the source centre lines (both lanes' start in the middle of
74026) and lay 2.3 m off the source boundaries at the road start. _zero_starts holds such a lane at zero width up
to its source start, but only after more than UNOBSERVED_END_M of road without source; here there is 0.40 m.

retry: the lanes whose width went negative get _zero_starts without that minimum and the road is refitted once;
the result is written only when no width is negative any more, else the road stays skipped as before. Roads whose
first refit is written are never refitted twice, so their output does not change.
"""
from __future__ import annotations

from mapforge.ops import lane_refit as LR


def negative_lanes(road, fitted):
    """[(side, section, lane, minimum width)] of the lanes whose fitted width goes below -NEGATIVE_WIDTH_TOL (the
    widths write_road checks, before its crossing repair)."""
    out = []
    for i in range(len(road.sections)):
        for side, sign in (("right", 1.0), ("left", -1.0)):
            for lane in road.lanes(i, side):
                k = abs(int(lane.get("id")))
                width = LR._difference(fitted[(side, i, k - 1)], fitted[(side, i, k)], road.s[i], road.s[i + 1], sign)
                low = min(v for _, v in LR._samples(width))
                if low < -LR.NEGATIVE_WIDTH_TOL:
                    out.append((side, i, k, low))
    return out


def retry(road_el, src, origin, params, road, fitted, report):
    """(report of the second refit, written to ``road_el``, or None with the road unchanged; note on the attempt
    or None where none was made). Only with ``protect_unobserved_ends`` (the SHP boundary step)."""
    if not (params or {}).get("protect_unobserved_ends"):
        return None, None
    lanes = negative_lanes(road, fitted)
    if not lanes:
        return None, None
    note = {"lanes": [{"side": side, "section": i, "lane": k, "min_width_m": round(low, 4)}
                      for side, i, k, low in lanes]}
    try:
        road2, fitted2, again = LR.refit_road(road_el, src, origin,
                                              {**params, "zero_start_lanes": {(side, i, k) for side, i, k, _ in lanes}})
        again["params"] = report["params"]
        again["zero_start_retry"] = {**note, "first_refit": report.get("skipped")}
        if LR.write_road(road2, fitted2, again):
            return again, note
        note["result"] = again.get("skipped")
    except (ValueError, KeyError, StopIteration, IndexError) as exc:
        note["result"] = f"{type(exc).__name__}: {exc}"
    return None, note
