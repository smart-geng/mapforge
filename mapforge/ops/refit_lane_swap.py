"""SHP lane swaps fitted together: the outer edge on its own source, the divider carries the swap (2026-10-14).

SHP draws a lane that takes over from its neighbour as two lanes whose dividing line runs across the lane: one lane opens
from zero width while the other closes to zero, and the road's outer edge stays where it is. Generalization 0711 road
11, right: lane 57656 is born at s 60.69 inside lane 60016, which dies at s 64.53; their divider crosses 2.85 m in 3.8 m.
0411 road 13, left: lane 04800 is born at s 71.18 outside lane 03248, which dies at s 83.21; the divider crosses 4 m in
12 m. lane_refit fits each outer lane boundary as its own chain, as a width on its inner neighbour near the lane's
births and deaths (lane_birth_advance), so the outer edge was the closing (or opening) width on top of the divider. Each
width keeps runs of at least SEG_MIN and the two were fitted apart, so the opening and the closing did not add up to the
source's constant width: 0711's outer edge dented inward by 0.98 m at s 62.5 (and 0.62 m at s 4.5, where the chain's
relative fit stood on a lane dying at the road's far end).

A swap here is a lane X and the lane Y inside it on one road side where
* (type "inner-birth") Y is born at an advanced birth s_b and X dies at an advanced death s_d > s_b, converging onto Y,
  X starts before Y's first section and Y runs on after X's last section; or
* (type "outer-birth") X is born at an advanced birth s_b, peeling off Y, and Y dies at an advanced death s_d > s_b,
  Y starts before X's first section and X runs on after Y's last section;
with Y's outer boundary the inner neighbour of X's in every section between, and the source's outer edge running on
across the event of the outer lane: X's outer boundary and Y's, each extrapolated over EDGE_WINDOW_M from its side, meet
within SWAP_EDGE_M there.

find rebuilds the two boundary chains: the outer edge becomes one chain across the swap (X's outer boundary, and Y's
before X is born or after X dies), fitted as any chain. Where X is a zero-width extension its own boundary has no
source, so the edge takes Y's outer boundary there ("extra"); its fits relative to an inner boundary (births and deaths
of its own) stand on Y's inner boundary where the divider lies in between (stand_in). The divider, Y's outer boundary
from the swap's first section to its last, is built by divider. It leaves Y's inner boundary at s_b on one side of the
swap and joins the outer edge at s_d on the other (inner-birth; outer-birth the other way round): near the event on
the inner side it is Y's inner boundary plus a width, near the other the outer edge minus a width, so each width is
exactly zero beyond its event. Each width runs straight from its event toward the middle of the swap and turns at the
event in a G2 corner sized as lane_refit sizes corners (the curvature target KAPPA_CAP, at most half the run between the
events and the room the zero-width extension leaves); their slopes are chosen so the two lines meet at the middle in
position and direction, where a C2 transition over FRAME_M either side passes from one to the other.
"""
from __future__ import annotations

import numpy as np

from mapforge.ops import lane_refit as LR

SWAP_EDGE_M = 0.3        # the source's outer edge runs on across the swap within this
EDGE_WINDOW_M = 3.0      # ... each side extrapolated linearly from this much of its outer boundary
FRAME_M = 0.5            # the divider passes from its inner frame to its outer one over this either side of the middle


def _owner(chains):
    return {(i, k): c for c in chains for i, k in c["members"].items()}


def _obs(road, obs_at, chain, lo, hi):
    """Source samples of the chain's boundary in its sections overlapping [lo, hi]."""
    rows = []
    for i, k in sorted(chain["members"].items()):
        if road.s[i + 1] < lo or road.s[i] > hi:
            continue
        o = obs_at(i, k)
        if o is not None and len(o):
            rows.append(o)
    return np.vstack(rows) if rows else None


def _edge_at(obs, x, before):
    """t of an outer boundary at x, extrapolated linearly from its samples within EDGE_WINDOW_M before (after) x."""
    if obs is None:
        return None
    sel = (obs[:, 0] >= x - EDGE_WINDOW_M) & (obs[:, 0] <= x) if before else \
        (obs[:, 0] >= x) & (obs[:, 0] <= x + EDGE_WINDOW_M)
    if sel.sum() < 2 or np.ptp(obs[sel, 0]) < 2 * LR.GRID:
        return None
    slope, icpt = np.polyfit(obs[sel, 0], obs[sel, 1], 1)
    return float(slope * x + icpt)


def _adjacent(x, y, lo, hi):
    """Chain y's boundary is the inner neighbour of chain x's in every section lo..hi."""
    return all(i in x["members"] and y["members"].get(i) == x["members"][i] - 1 for i in range(lo, hi + 1))


def _inner_birth(road, side, c, owner, obs_at):
    """(edge chain, divider chain, extra observations, row, Y's chain) where c's lane X dies at an advanced death onto
    the lane Y inside it, born at an advanced birth before that, and the source's outer edge runs on across X's death;
    else None."""
    if not c.get("death") or c.get("converge") == "outer":
        return None
    death = LR._advanced_events(road, side, c).get("death")
    i1 = max(c["members"])
    y = owner.get((i1, c["members"][i1] - 1))
    if death is None or y is None or y is c or y.get("swap") or not y.get("birth") or y.get("peel") == "outer":
        return None
    birth = LR._advanced_events(road, side, y).get("birth")
    i0 = min(y["members"])
    if (birth is None or not birth[0] < death[0] or i0 > i1 or max(y["members"]) <= i1 or min(c["members"]) >= i0
            or not _adjacent(c, y, i0, i1)):
        return None
    s_b, s_d = birth[0], death[0]
    before = _edge_at(_obs(road, obs_at, c, s_d - EDGE_WINDOW_M, s_d), s_d, True)
    after = _edge_at(_obs(road, obs_at, y, s_d, s_d + EDGE_WINDOW_M), s_d, False)
    if before is None or after is None or abs(before - after) > SWAP_EDGE_M:
        return None
    # X's sections past its death are a zero-width extension: the edge takes Y's outer boundary there
    extra = [(i, k - 1) for i, k in sorted(c["members"].items()) if road.s[i] >= s_d - 1e-6]
    edge = {"members": {**c["members"], **{i: k for i, k in y["members"].items() if i > i1}},
            "birth": c.get("birth", False), "peel": c.get("peel"), "death": y.get("death", False),
            "converge": y.get("converge")}
    divider = {"members": {i: k for i, k in y["members"].items() if i <= i1}, "birth": True, "peel": y.get("peel"),
               "death": True, "converge": "outer"}
    row = {"type": "inner-birth", "s_b": s_b, "s_d": s_d, "edge_gap_m": abs(before - after)}
    return edge, divider, extra, row, y


def _outer_birth(road, side, c, owner, obs_at):
    """(edge chain, divider chain, extra observations, row, Y's chain) where c's lane X is born at an advanced birth on
    the lane Y inside it, which dies at an advanced death after that, and the source's outer edge runs on across X's
    birth; else None."""
    if not c.get("birth") or c.get("peel") == "outer":
        return None
    birth = LR._advanced_events(road, side, c).get("birth")
    i0 = min(c["members"])
    y = owner.get((i0, c["members"][i0] - 1))
    if birth is None or y is None or y is c or y.get("swap") or not y.get("death") or y.get("converge") == "outer":
        return None
    death = LR._advanced_events(road, side, y).get("death")
    i1 = max(y["members"])
    if (death is None or not birth[0] < death[0] or i1 < i0 or min(y["members"]) >= i0 or max(c["members"]) <= i1
            or not _adjacent(c, y, i0, i1)):
        return None
    s_b, s_d = birth[0], death[0]
    before = _edge_at(_obs(road, obs_at, y, s_b - EDGE_WINDOW_M, s_b), s_b, True)
    after = _edge_at(_obs(road, obs_at, c, s_b, s_b + EDGE_WINDOW_M), s_b, False)
    if before is None or after is None or abs(before - after) > SWAP_EDGE_M:
        return None
    # X's sections before its birth are a zero-width extension: the edge takes Y's outer boundary there
    extra = [(i, k - 1) for i, k in sorted(c["members"].items()) if road.s[i + 1] <= s_b + 1e-6]
    edge = {"members": {**{i: k for i, k in y["members"].items() if i < i0}, **c["members"]},
            "birth": y.get("birth", False), "peel": y.get("peel"), "death": c.get("death", False),
            "converge": c.get("converge")}
    divider = {"members": {i: k for i, k in y["members"].items() if i >= i0}, "birth": True, "peel": "outer",
               "death": True, "converge": y.get("converge")}
    row = {"type": "outer-birth", "s_b": s_b, "s_d": s_d, "edge_gap_m": abs(before - after)}
    return edge, divider, extra, row, y


def find(road, side, chains, obs_at, report=None):
    """The chains of one road side with every lane swap rebuilt into an outer-edge chain and a divider chain (see the
    module doc), in fit order; unchanged without swaps. ``obs_at(i, k)``: source samples of boundary k in section i.
    ``report`` receives one row per swap under "lane_swaps"."""
    rows = []
    while True:
        owner = _owner(chains)
        found = None
        for c in chains:
            if not c.get("swap"):
                found = _inner_birth(road, side, c, owner, obs_at) or _outer_birth(road, side, c, owner, obs_at)
                if found:
                    found = (c, *found)
                    break
        if found is None:
            break
        c, edge, divider, extra, row, y = found
        row = {"side": side, **{k: round(v, 3) for k, v in row.items() if k != "type"}, "type": row["type"],
               "edge": {str(i): k for i, k in sorted(edge["members"].items())},
               "divider": {str(i): k for i, k in sorted(divider["members"].items())}}
        divider["swap"] = {"role": "divider", "type": row["type"], "s_b": row["s_b"], "s_d": row["s_d"], "row": row}
        edge["swap"] = {"role": "edge", "extra": extra, "divider": divider}
        chains = [x for x in chains if x is not c and x is not y] + [edge, divider]
        rows.append(row)
    if not rows:
        return chains
    if report is not None:
        report.setdefault("lane_swaps", []).extend(rows)
    return LR._fit_order(chains)


def stand_in(side, edge, fitted):
    """Before the outer edge is fitted: its divider's boundary stands in as the divider's inner boundary, so the edge's
    fits relative to its inner boundary (its own births and deaths, zero-width starts) stand on that."""
    divider = edge["swap"]["divider"]
    for i, k in divider["members"].items():
        inner = fitted.get((side, i, k - 1))
        if inner is not None:
            fitted[(side, i, k)] = inner


def _rising(s0, mu, h, upto):
    """A width zero up to s0 - h that turns at s0 in a G2 corner of half length h and rises at mu: pieces up to upto."""
    pieces = LR._g2_corner(s0, 0.0, 0.0, mu, h)
    if upto > s0 + h + 1e-9:
        pieces.append((s0 + h, upto, mu * h, mu, 0.0, 0.0))
    return [p for p in LR._split(pieces, [upto]) if p[0] < upto - 1e-9]


def _falling(s1, mu, h, start):
    """A width falling at mu that turns at s1 in a G2 corner of half length h and is zero from s1 + h: pieces from
    start."""
    pieces = LR._g2_corner(s1, 0.0, -mu, 0.0, h)
    if start < s1 - h - 1e-9:
        pieces.insert(0, (start, s1 - h, mu * (s1 - start), -mu, 0.0, 0.0))
    return [p for p in LR._split(pieces, [start]) if p[1] > start + 1e-9]


def divider(road, side, chain, fitted, kappa=None, obs=None):
    """Pieces of a divider chain over its span (see the module doc), or None where its inner or outer boundary is not
    fitted yet, its corners find no room or it would leave a lane width outside [0, the swap's full width]
    (NEGATIVE_WIDTH_TOL). ``obs``: the divider's source samples, for the row's divider_dev_m."""
    sw = chain["swap"]
    kappa = kappa or LR.KAPPA_CAP
    secs = sorted(chain["members"])
    if any((side, i, chain["members"][i] + d) not in fitted for i in secs for d in (-1, 1)):
        return None
    a, b = road.s[secs[0]], road.s[secs[-1] + 1]
    inner = [p for i in secs for p in fitted[(side, i, chain["members"][i] - 1)]]
    outer = [p for i in secs for p in fitted[(side, i, chain["members"][i] + 1)]]
    sign = 1.0 if side == "left" else -1.0
    full = [(p[0], p[1], *(sign * v for v in p[2:])) for p in LR._combine(outer, inner, a, b, -1.0)]
    s_b, s_d = sw["s_b"], sw["s_d"]
    run, mid = s_d - s_b, 0.5 * (sw["s_b"] + sw["s_d"])
    if not (a + LR.GRID < s_b and s_d < b - LR.GRID and run > 2 * FRAME_M):
        return None
    w, w1 = LR._at(full, mid), LR._at(full, mid, 1)
    # the width rising from s_b and the one falling to s_d, meeting at the middle in position and direction
    mu0, mu1 = w / run + 0.5 * w1, w / run - 0.5 * w1
    if min(mu0, mu1) <= 0:
        return None

    def size(mu, room):
        return min(max(mu / kappa, LR.H_MIN), 0.5 * run, room)
    h0, h1 = size(mu0, s_b - a), size(mu1, b - s_d)
    if min(h0, h1) < LR.GRID:
        return None
    lo, hi, m0, m1 = s_b - h0, s_d + h1, mid - FRAME_M, mid + FRAME_M
    opening = sw["type"] == "inner-birth"            # Y opens: on its inner boundary at s_b, on the outer edge at s_d

    def frame(base, sgn, width, x0, x1):
        part = [p for p in LR._split(base, [x0, x1]) if x0 - 1e-6 <= p[0] < x1 - 1e-6]
        return LR._combine(part, width, x0, x1, sgn * sign)
    rising, falling = _rising(s_b, mu0, h0, m0), _falling(s_d, mu1, h1, m1)
    left = frame(inner, 1.0, rising, lo, m0) if opening else frame(outer, -1.0, rising, lo, m0)
    right = frame(outer, -1.0, falling, m1, hi) if opening else frame(inner, 1.0, falling, m1, hi)
    join = LR._g2_transition(m0, m1, tuple(LR._at(left, m0, o) for o in range(3)),
                             tuple(LR._at(right, m1, o) for o in range(3)))
    before, after = (inner, outer) if opening else (outer, inner)
    pieces = ([p for p in LR._split(before, [lo]) if p[1] <= lo + 1e-6] + left + join + right
              + [p for p in LR._split(after, [hi]) if p[0] >= hi - 1e-6])
    xs = np.linspace(lo, hi, 400)
    ys, ws = sign * (LR._eval_many(pieces, xs) - LR._eval_many(inner, xs)), LR._eval_many(full, xs)
    if ys.min() < -LR.NEGATIVE_WIDTH_TOL or (ws - ys).min() < -LR.NEGATIVE_WIDTH_TOL:
        return None
    row = sw["row"]
    row.update({"h_m": [round(h0, 3), round(h1, 3)], "span": [round(lo, 3), round(hi, 3)]})
    if obs is not None and len(obs):
        sel = obs[(obs[:, 0] >= s_b) & (obs[:, 0] <= s_d)]
        if len(sel):
            row["divider_dev_m"] = round(float(np.max(np.abs(LR._eval_many(pieces, sel[:, 0]) - sel[:, 1]))), 3)
    return pieces
