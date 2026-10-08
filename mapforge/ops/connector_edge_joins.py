"""Connector lane edges at reference joins: width slopes chosen so the edges barely jump in curvature (2026-10-07).

A connector's lane centre follows its reference line (aligned frame) plus a small lateral offset; its two edges lie
half a width either side. Where the clothoid reference changes its curvature rate (a join, Delta kappa'), an edge at
offset t with lateral slope t' jumps in curvature by about t * t' * Delta kappa' / (1 - kappa t)^3: on the inner
edge of a tight turn (1 - kappa t ~ 0.6) that is four to six times t * t' * Delta kappa'. In the definitive run
20261007-default-c2-fair 54 internal edge joins jumped by more than 1e-3 /m, the largest 0.021 /m (map-node16 road
103), 53 of them at reference joins of turning connectors. The lane centre itself is exactly G2 there.

A clothoid chain cannot carry a continuous curvature rate, and keeping both edge slopes zero at every join would move
the lane centre (the driving path). Here only the width changes, the lane centre stays point for point:
- at each internal reference join where an edge jumps by more than TRIGGER_PER_M, the width slope is chosen so the
  larger of the two edge jumps is smallest (the lane-centre slope c' stays, the edge slopes are c' +- w'/2);
- the width is written again as a C2 cubic spline (knots at the joins and at most KNOT_M apart): value, slope and
  curvature at both mouths as before, the chosen slopes at those joins, otherwise as close to the old width as it can;
- the lane offset becomes centre +- new width / 2 (exact: piecewise cubic on the union of the record boundaries).
A connector keeps its old records unless the lane centre and both mouths are unchanged, the width stays above
MIN_WIDTH_M without a new bulge, and its largest edge jump drops. One-lane, one-section connectors only.
"""
from __future__ import annotations

import copy
import json
import math

import numpy as np
from lxml import etree
from scipy.interpolate import BSpline, PPoly

from mapforge.validate.smoothness import _edge_world_curvature, _geoms, _ref_kappa_at, lane_edges_kinematics_at

CODE = "mapforge.connector_edge_joins/v1"
TRIGGER_PER_M = 1e-3      # aim: no edge jump above this (the draft bound for lane-centre joins); joins already within
                          # it keep their width slope, others get the slope closest to the old one that reaches it,
                          # or, where none does, the slope with the smallest larger edge jump
KNOT_M = 2.5
MIN_WIDTH_M = 0.5
BULGE_SLACK_M = 0.05      # the new width may leave the old width's range by at most this (T1 bulge limit 0.30 m)
SLOPE_SPAN = 0.08         # searched half-width slopes |w'/2| around the lane-centre slope


def _records(parent, tag, key):
    return sorted((float(e.get(key)), float(e.get("a")), float(e.get("b")), float(e.get("c")), float(e.get("d")))
                  for e in parent.findall(tag))


def _poly(recs, x, base=0.0):
    r = ([q for q in recs if q[0] + base <= x + 1e-9] or recs[:1])[-1]
    u = x - (r[0] + base)
    return r[1] + u * (r[2] + u * (r[3] + u * r[4])), r[2] + u * (2 * r[3] + 3 * r[4] * u), 2 * r[3] + 6 * r[4] * u


def _joins(road):
    cuts, s = [], 0.0
    for g in _geoms(road)[:-1]:
        s += g[4]
        cuts.append(s)
    return cuts


def _edge_jump(t, tp, tpp, kappa, rate0, rate1):
    return abs(_edge_world_curvature(t, tp, tpp, kappa, rate1) - _edge_world_curvature(t, tp, tpp, kappa, rate0))


def _best_half_slope(c, cp, w, wpp, cpp, kappa, r0, r1, x_now=0.0, target=TRIGGER_PER_M):
    """Half width slope x (edges c +- w/2 with slopes c' +- x): the one closest to ``x_now`` whose larger edge jump is
    within 0.9 ``target``; where no slope reaches that, the one with the smallest larger edge jump (both edges then
    jump alike: the lane-centre slope c' is the floor, and moving the lane centre is not worth it)."""
    def worst(x):
        return max(_edge_jump(c + w / 2, cp + x, cpp + wpp / 2, kappa, r0, r1),
                   _edge_jump(c - w / 2, cp - x, cpp - wpp / 2, kappa, r0, r1))
    xs = np.linspace(-SLOPE_SPAN, SLOPE_SPAN, 1281)
    vals = np.array([worst(x) for x in xs])
    ok = np.nonzero(vals <= 0.9 * target)[0]
    if len(ok):
        x = float(xs[ok[np.argmin(np.abs(xs[ok] - x_now))]])
        return x, worst(x)
    i = int(np.argmin(vals))
    lo, hi = xs[max(0, i - 1)], xs[min(len(xs) - 1, i + 1)]
    for _ in range(40):                          # golden-section refinement
        a, b = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        if worst(a) < worst(b):
            hi = b
        else:
            lo = a
    x = (lo + hi) / 2
    return x, worst(x)


def _spline(xs, ys, a, b, knots, end_a, end_b, slopes):
    """C2 cubic spline on [a, b]: least squares to (xs, ys), value/slope/curvature fixed at both ends and the given
    slopes at interior points [(x, slope)]."""
    inner = sorted(k for k in knots if a + 1e-6 < k < b - 1e-6)
    t = np.r_[[a] * 4, inner, [b] * 4]
    nb = len(t) - 4
    X = BSpline.design_matrix(np.clip(xs, a, b), t, 3).toarray()
    basis = [BSpline(t, np.eye(nb)[i], 3) for i in range(nb)]
    rows, vals = [], []
    for x, st in ((a, end_a), (b, end_b)):
        for o in range(3):
            rows.append([bs.derivative(o)(x) if o else bs(x) for bs in basis])
            vals.append(st[o])
    for x, sl in slopes:
        rows.append([bs.derivative(1)(x) for bs in basis])
        vals.append(sl)
    C, d = np.array(rows), np.array(vals)
    K = np.block([[X.T @ X + 1e-9 * np.eye(nb), C.T], [C, np.zeros((len(C), len(C)))]])
    coef = np.linalg.solve(K, np.r_[X.T @ ys, d])[:nb]
    return PPoly.from_spline(BSpline(t, coef, 3))


def _pieces(fn, cuts):
    """Exact cubic records (x0, a, b, c, d) of a piecewise-cubic function fn(x) -> value on the intervals of cuts."""
    out = []
    for x0, x1 in zip(cuts[:-1], cuts[1:]):
        if x1 - x0 < 1e-9:
            continue
        us = np.linspace(0.0, x1 - x0, 4)
        vals = np.array([fn(x0 + u) for u in us])
        A = np.vander(us, 4, increasing=True)
        out.append((x0, *np.linalg.solve(A, vals)))
    return out


def _replace(parent, tag, pieces, attr, base, before):
    old = parent.findall(tag)
    at = list(parent).index(old[0]) if old else list(parent).index(before)
    tail = old[-1].tail if old else "\n"
    for el in old:
        parent.remove(el)
    for j, (x0, a, b, c, d) in enumerate(pieces):
        el = etree.Element(tag, **{attr: repr(round(float(x0 - base), 9))}, a=repr(float(a)), b=repr(float(b)),
                           c=repr(float(c)), d=repr(float(d)))
        el.tail = tail
        parent.insert(at + j, el)


def fix_connector(road):
    """Choose the width slopes at the reference joins of one connector; returns a report row (or None)."""
    secs = road.findall("lanes/laneSection")
    if len(secs) != 1 or float(secs[0].get("s", 0)) > 1e-9:
        return None
    lanes = secs[0].findall("left/lane") + secs[0].findall("right/lane")
    if len(lanes) != 1 or lanes[0].get("type") != "driving" or lanes[0].find("border") is not None:
        return None
    lane = lanes[0]
    side = "left" if int(lane.get("id")) > 0 else "right"
    sign = 1.0 if side == "left" else -1.0           # outer edge = offset + sign * width
    L = float(road.get("length"))
    offs = _records(road.find("lanes"), "laneOffset", "s")
    wids = _records(lane, "width", "sOffset")
    if not offs or not wids:
        return None

    def state(x):
        o, w = _poly(offs, x), _poly(wids, x)
        c = tuple(o[i] + sign * w[i] / 2 for i in range(3))    # lane centre offset and derivatives
        return c, w

    joins = []
    for sj in _joins(road):
        if not 0.5 < sj < L - 0.5:
            continue
        kappa, r0 = _ref_kappa_at(road, sj - 1e-7)
        _, r1 = _ref_kappa_at(road, sj + 1e-7)
        (c, cp, cpp), (w, wp, wpp) = state(sj)
        now = max(_edge_jump(c + w / 2, cp + wp / 2, cpp + wpp / 2, kappa, r0, r1),
                  _edge_jump(c - w / 2, cp - wp / 2, cpp - wpp / 2, kappa, r0, r1))
        if now <= TRIGGER_PER_M:
            continue
        # the edges are c +- w/2 for either side: w' = 2 x
        x, best = _best_half_slope(c, cp, w, wpp, cpp, kappa, r0, r1, x_now=wp / 2)
        if best < now:
            joins.append((sj, 2 * x, now, best))
    if not joins:
        return None
    keep_offs = copy.deepcopy(road.find("lanes"))
    xs = np.linspace(0.0, L, max(50, int(L / 0.1)))
    w_now = np.array([_poly(wids, x)[0] for x in xs])
    knots = set(j[0] for j in joins)
    cuts = max(1, int(math.ceil(L / KNOT_M)))
    knots.update(L * np.arange(1, cuts) / cuts)
    end_a, end_b = _poly(wids, 0.0), _poly(wids, L)
    pp = _spline(xs, w_now, 0.0, L, sorted(knots), end_a, end_b, [(sj, slope) for sj, slope, _, _ in joins])
    w_new = pp(xs)
    lo, hi = w_now.min() - BULGE_SLACK_M, w_now.max() + BULGE_SLACK_M
    if w_new.min() < max(MIN_WIDTH_M, lo) or w_new.max() > hi:
        return {"road": road.get("id"), "skipped": "width range", "joins": len(joins)}
    # new records: width on its own knots; lane offset = centre - sign * new width / 2 on the union of boundaries
    w_pieces = [(x0, cf[3], cf[2], cf[1], cf[0]) for x0, x1, cf in zip(pp.x[:-1], pp.x[1:], pp.c.T) if x1 - x0 > 1e-9]
    cutset = sorted({0.0, L} | {r[0] for r in offs} | {r[0] for r in wids} | {p[0] for p in w_pieces})
    centre = lambda x: _poly(offs, x)[0] + sign * _poly(wids, x)[0] / 2          # noqa: E731
    new_off = lambda x: centre(x) - sign * float(pp(x)) / 2                     # noqa: E731
    o_pieces = _pieces(new_off, cutset)
    first_section = road.find("lanes/laneSection")
    _replace(road.find("lanes"), "laneOffset", o_pieces, "s", 0.0, first_section)
    _replace(lane, "width", w_pieces, "sOffset", 0.0, None if lane.find("width") is not None else lane[-1])
    # checks
    offs2, wids2 = _records(road.find("lanes"), "laneOffset", "s"), _records(lane, "width", "sOffset")
    centre_err = max(abs(_poly(offs2, x)[0] + sign * _poly(wids2, x)[0] / 2 - centre(x)) for x in xs)
    mouth = max(abs(p - q) for x in (0.0, L)
                for p, q in zip(_poly(offs, x) + _poly(wids, x), _poly(offs2, x) + _poly(wids2, x)))
    after = []
    for sj, _, _, _ in joins:
        kappa, r0 = _ref_kappa_at(road, sj - 1e-7)
        _, r1 = _ref_kappa_at(road, sj + 1e-7)
        e = lane_edges_kinematics_at(road, sj, side)
        after.append(max(_edge_jump(t, tp, tpp, kappa, r0, r1) for t, tp, tpp in e))
    before = max(j[2] for j in joins)
    row = {"road": road.get("id"), "joins": len(joins), "edge_jump_max_per_m": [round(before, 6), round(max(after), 6)],
           "width_change_max_m": round(float(np.max(np.abs(w_new - w_now))), 4), "centre_error_m": centre_err,
           "mouth_change": mouth}
    if centre_err > 1e-6 or mouth > 1e-6 or max(after) >= before:
        lanes_el = road.find("lanes")
        road.replace(lanes_el, keep_offs)
        row["skipped"] = "centre moved" if centre_err > 1e-6 else "mouth moved" if mouth > 1e-6 else "no better"
    return row


def apply(root):
    rows = []
    for road in root.findall("road"):
        if road.get("junction") in (None, "-1") or road.get("name") == "junction_paving":
            continue
        row = fix_connector(road)
        if row is None:
            continue
        rows.append(row)
        if "skipped" not in row:
            road.append(etree.Element("userData", code=CODE, value=json.dumps(row, ensure_ascii=False)))
    return {"schema": CODE, "fixed": sum(1 for r in rows if "skipped" not in r), "rows": rows}


def apply_file(path):
    tree = etree.parse(str(path), etree.XMLParser(remove_blank_text=False))
    report = apply(tree.getroot())
    tree.write(str(path), xml_declaration=True, encoding="UTF-8")
    return report
