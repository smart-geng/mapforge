"""MAP departure sides eased (2026-10-07).

MAP gives no departure geometry: the converter and map_lane_refit mirror the approach side about the centre line
(INFERRED). Where the approach's inner edge moves (a turn bay opening on the inside, an inner lane widening), each
mirrored departure lane moves by that much twice within the same few metres: 6 m in 20 m and 0.125 /m beside the
bay of map-node4 road 10. In the definitive run 20261005-default-c2-midpoint 17 of 28 MAP roads had departure
through lanes curving above 0.03 /m, while the approach through lanes stayed below 0.014 /m (P90). Nothing in the
source says how the departure lanes make room, so here they do it as smoothly as possible:

- per road, departure stretches where some departure lane boundary bends (|t''| > MOVE_D2) are padded by PAD_M
  and widened to whole lane sections. Outside them nothing changes; at a stretch end inside the road the new
  widths continue the old ones with value, slope and curvature, and at the mouth every departure boundary keeps
  its value, slope and curvature, so the junction and its connectors stay as they were;
- a departure lane is followed from the stretch's last section back by its predecessor links or, where there is
  none, by the same-id lane of the previous section whose width it continues: the converter writes a mirrored
  taper as unlinked pieces (deliberately, map_lane_refit.link_tapers) and keeps zero-width placeholder lanes;
  links are never added or removed;
- inside a stretch the departure widths are chosen again (a quadratic programme solved by ADMM) to minimise the
  sum over the departure lane boundaries of their squared third differences, subject to
  * a lane that is closed (narrower than THROUGH_MIN_M) somewhere before the mouth, a mirrored bay, may open at
    most PAD_M earlier than now and then only widens toward the mouth (no pinch); a lane born inside the stretch
    without a lane to continue is zero before its birth;
  * an open lane keeps its width within BAND_M of the current one (and within the range it had in the stretch);
  * widths stay >= 0, so the departure side never crosses the centre line;
  lanes outside these chains (placeholders, at most PLACEHOLDER_M wide) keep their widths;
- the widths are written back as C2 cubic splines (knots at the section starts and at most KNOT_M apart). A road
  keeps its old geometry unless the result has no new step at a section boundary, keeps the mouth and the stretch
  ends, stays non-negative and bends less.
The approach side, the centre line (lane offset), the reference line, the junction and G8 (departure lanes are not
compared) are untouched.
"""
from __future__ import annotations

import copy
import json
import math

import numpy as np
from lxml import etree
from scipy import sparse
from scipy.interpolate import BSpline, CubicSpline, PPoly
from scipy.sparse.linalg import splu

from mapforge.validate.smoothness import lane_edges_kinematics_at, section_boundary_steps

CODE = "mapforge.map_departure_ease/v1"
STEP_M = 0.5          # QP samples
MOVE_D2 = 0.004       # a departure boundary bending laterally by more than this [1/m] is moving
PAD_M = 30.0          # stretch padding, and the furthest a closed lane may open earlier
KNOT_M = 4.0          # spline knots at most this far apart
PLACEHOLDER_M = 0.02  # a lane at most this wide all along its section is a zero-width placeholder
THROUGH_MIN_M = 0.05  # narrower than this a lane is closed (not yet open toward the mouth)
CONTINUE_M = 0.01     # an unlinked lane continues the same-id lane of the previous section within this
BETA = 1e-6           # pull toward the current widths (tie-breaker)
GAMMA = 0.2           # weight of each lane's own width smoothness against the boundaries' (no kinks at openings)
BAND_M = 0.5          # an open lane's width stays within this of its current width
MONO_TOL_M = 0.01     # a lane narrowing by less than this per metre after it opens counts as only widening
END_TOL = 1e-4        # value / slope / curvature kept at the mouth and continued across section boundaries
STEP_TOL_M = 1e-3     # section_boundary_steps compares positions 1 mm either side: sloped boundaries read ~0.2 mm


# ---------------------------------------------------------------- width records

def _records(lane):
    return sorted((float(w.get("sOffset")), float(w.get("a")), float(w.get("b")), float(w.get("c")),
                   float(w.get("d"))) for w in lane.findall("width"))


def _width(lane, ds, order=0):
    recs = _records(lane)
    if not recs:
        return 0.0
    s0, a, b, c, d = ([r for r in recs if r[0] <= ds + 1e-9] or recs[:1])[-1]
    u = ds - s0
    if order == 0:
        return a + u * (b + u * (c + u * d))
    if order == 1:
        return b + u * (2 * c + 3 * d * u)
    return 2 * c + 6 * d * u


def _state(lane, ds):
    return tuple(_width(lane, ds, o) for o in range(3))


def _placeholder(lane, span):
    return (lane.find("link/successor") is None and lane.find("link/predecessor") is None
            and max(abs(_width(lane, x)) for x in np.linspace(0.0, span, 9)) <= PLACEHOLDER_M)


# ---------------------------------------------------------------- road structure

class _Road:
    def __init__(self, el):
        self.el = el
        self.length = float(el.get("length"))
        self.sections = el.findall("lanes/laneSection")
        self.s = [float(x.get("s")) for x in self.sections] + [self.length]

    def left(self, i):
        return sorted(self.sections[i].findall("left/lane"), key=lambda ln: int(ln.get("id")))

    def lane(self, i, lid):
        return next((ln for ln in self.sections[i].findall("left/lane") if ln.get("id") == lid), None)

    def section_at(self, x):
        return int(min(max(np.searchsorted(self.s, x, side="right") - 1, 0), len(self.sections) - 1))


def _stretches(road: _Road):
    """[(first section, end section)] whose departure boundaries bend, padded and widened to whole sections."""
    moving = [x for x in np.arange(0.0, road.length, STEP_M)
              if any(abs(d2) > MOVE_D2 for _, _, d2 in lane_edges_kinematics_at(road.el, float(x), "left")[1:])]
    if not moving:
        return []
    spans, lo, hi = [], moving[0], moving[0]
    for x in moving[1:]:
        if x - hi > 2 * PAD_M:
            spans.append((lo, hi))
            lo = x
        hi = x
    spans.append((lo, hi))
    out = []
    for lo, hi in spans:
        i0 = road.section_at(max(0.0, lo - PAD_M))
        i1 = road.section_at(min(road.length, hi + PAD_M) - 1e-9) + 1
        if out and i0 <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], i1))
        else:
            out.append((i0, i1))
    return out


def _previous(road: _Road, i, lane, owned):
    """The lane continuing into ``lane`` from section i-1: its predecessor link, or else (the converter writes a
    departure taper as unlinked pieces, deliberately: map_lane_refit.link_tapers) the same id without links whose
    width meets ``lane``'s within CONTINUE_M."""
    p = lane.find("link/predecessor")
    if p is not None:
        return road.lane(i - 1, p.get("id")), True
    ln = road.lane(i - 1, lane.get("id"))
    if (ln is None or id(ln) in owned or ln.find("link/successor") is not None
            or abs(_width(ln, road.s[i] - road.s[i - 1]) - _width(lane, 0.0)) > CONTINUE_M):
        return None, False
    return ln, False


def _chains(road: _Road, i0, i1):
    """Departure lanes of the stretch's last section traced back to section i0 (links or unlinked continuation).

    Returns (chains, problem). A chain: {"members": {section: lane}, "start": first section, "start_state":
    (w, w', w'') at its start, "end_state": at the stretch end}. A chain starting after i0 is born there."""
    chains, owned = [], set()
    for lane in road.left(i1 - 1):
        members, i, ln = {i1 - 1: lane}, i1 - 1, lane
        owned.add(id(lane))
        while i > i0:
            prev, linked = _previous(road, i, ln, owned)
            if prev is None:
                if linked:
                    return None, "broken predecessor link"
                break
            i -= 1
            ln = prev
            members[i] = ln
            owned.add(id(ln))
        first = min(members)
        if first > i0 and _width(members[first], 0.0) >= THROUGH_MIN_M:
            return None, f"lane {lane.get('id')} starts {_width(members[first], 0.0):.2f} m wide inside the stretch"
        chains.append({"members": members, "start": first,
                       "start_state": _state(members[first], 0.0) if first == i0 else (0.0, 0.0, 0.0),
                       "end_state": _state(lane, road.s[i1] - road.s[i1 - 1])})
    for i in range(i0, i1):
        span = road.s[i + 1] - road.s[i]
        for ln in road.left(i):
            if id(ln) not in owned and not _placeholder(ln, span):
                return None, f"lane {ln.get('id')} of section {i} ends inside the stretch"
    return chains, None


def _layout(road: _Road, chains, xs):
    """Current widths W of the chains and the fixed part F of each chain's outer boundary (centre line plus the
    placeholders inside it) at xs; F is NaN where the chain does not exist yet."""
    m, n = len(chains), len(xs)
    lane_of = {}
    for k, c in enumerate(chains):
        for ln in c["members"].values():
            lane_of[id(ln)] = k
    W, F = np.zeros((m, n)), np.full((m, n), np.nan)
    t0 = np.zeros(n)
    for j, x in enumerate(xs):
        i = road.section_at(x)
        t0[j] = lane_edges_kinematics_at(road.el, float(x), "left")[0][0]
        fixed = 0.0
        for ln in road.left(i):
            w = _width(ln, x - road.s[i])
            k = lane_of.get(id(ln))
            if k is None:
                fixed += w
            else:
                W[k, j] = w
                F[k, j] = t0[j] + fixed
    return t0, W, F


# ---------------------------------------------------------------- QP

def _qp(P, q, M, lo, hi, x0, sigma=1e-6, alpha=1.6, max_iter=30000, eps_abs=1e-8, eps_rel=1e-7):
    """min 1/2 x'Px + q'x  s.t.  lo <= Mx <= hi: OSQP's ADMM iteration with adaptive rho (scipy sparse LU)."""
    eq = (hi - lo) < 1e-12
    eye = sparse.identity(P.shape[0], format="csc")

    def factor(rho):
        rv = np.where(eq, rho * 1e3, rho)
        return rv, splu((P + sigma * eye + M.T @ sparse.diags(rv) @ M).tocsc())

    rho = 0.1
    rv, lu = factor(rho)
    x = np.asarray(x0, float).copy()
    z = np.clip(M @ x, lo, hi)
    y = np.zeros(M.shape[0])
    it = 0
    for it in range(max_iter):
        xt = lu.solve(sigma * x - q + M.T @ (rv * z - y))
        zt = M @ xt
        x = alpha * xt + (1 - alpha) * x
        zr = alpha * zt + (1 - alpha) * z
        z_new = np.clip(zr + y / rv, lo, hi)
        y = y + rv * (zr - z_new)
        z = z_new
        if it % 25 == 0:
            Mx, Px, MTy = M @ x, P @ x, M.T @ y
            r_prim, r_dual = np.max(np.abs(Mx - z)), np.max(np.abs(Px + q + MTy))
            n_prim = max(np.max(np.abs(Mx)), np.max(np.abs(z)), 1e-12)
            n_dual = max(np.max(np.abs(Px)), np.max(np.abs(MTy)), np.max(np.abs(q)), 1e-12)
            if r_prim < eps_abs + eps_rel * n_prim and r_dual < eps_abs + eps_rel * n_dual:
                break
            new = float(np.clip(rho * math.sqrt((r_prim / n_prim) / max(r_dual / n_dual, 1e-30)), 1e-6, 1e6))
            if new > 5 * rho or new < rho / 5:
                rho = new
                rv, lu = factor(rho)
    return x, it


def _solve(W, F, chains, xs, i0, i1):
    """Departure widths chosen again: the smoothest chain boundaries F_k + sum_{i<=k} w_i."""
    m, n = W.shape
    D3 = sparse.diags([-1.0, 3.0, -3.0, 1.0], [0, 1, 2, 3], shape=(n - 3, n), format="csr")
    starts = [int(np.searchsorted(xs, c["start_x"] - 1e-9)) for c in chains]
    blocks_A, blocks_b = [], []
    for k in range(m):
        rows = D3[starts[k]:]
        S = sparse.hstack([sparse.identity(n) if i <= k else sparse.csr_matrix((n, n)) for i in range(m)]).tocsr()
        blocks_A.append(rows @ S)
        blocks_b.append(-(rows @ np.nan_to_num(F[k])))
    A = sparse.vstack(blocks_A).tocsr()
    b = np.concatenate(blocks_b)
    scale = 1.0 / (20.0 * m)
    own = sparse.block_diag([(D3.T @ D3)] * m)
    P = (2 * scale * (A.T @ A + GAMMA * own) + 2 * BETA * sparse.identity(m * n)).tocsc()
    q = -2 * scale * (A.T @ b) - 2 * BETA * W.ravel()
    lo, hi, mono = [], [], []
    ahead = int(round(PAD_M / STEP_M))
    for k, c in enumerate(chains):
        w = W[k]
        l, h = np.maximum(w - BAND_M, w.min()), np.minimum(w + BAND_M, w.max())
        opened = np.nonzero(w > THROUGH_MIN_M)[0]
        open_j = int(opened[0]) if len(opened) else n
        if open_j > 0:
            # closed at the stretch start: may open at most PAD_M earlier than now; a lane that then only widens
            # (a bay) keeps widening (no pinch), any other keeps its band once open
            fixed_until = max(starts[k], open_j - ahead)
            l[fixed_until:open_j], h[fixed_until:open_j] = 0.0, w.max()
            if np.all(np.diff(w[open_j:]) >= -MONO_TOL_M * STEP_M):
                l[open_j:], h[open_j:] = 0.0, w.max()
                mono.append((k, fixed_until, n - 4))
            l[:fixed_until] = h[:fixed_until] = w[:fixed_until]
        if starts[k] > 0:
            l[:starts[k]] = h[:starts[k]] = 0.0
            l[starts[k]] = h[starts[k]] = c["start_state"][0]
        else:
            l[:3] = h[:3] = w[:3]
        l[-3:] = h[-3:] = w[-3:]
        lo.append(l)
        hi.append(h)
    M_rows = [sparse.identity(m * n, format="csr")]
    lo, hi = list(np.concatenate(lo)), list(np.concatenate(hi))
    for k, j0, j1 in mono:
        if j1 <= j0:
            continue
        d = sparse.lil_matrix((j1 - j0, m * n))
        for r, j in enumerate(range(j0, j1)):
            d[r, k * n + j], d[r, k * n + j + 1] = -1.0, 1.0
        M_rows.append(d.tocsr())
        lo += [0.0] * (j1 - j0)
        hi += [np.inf] * (j1 - j0)
    M = sparse.vstack(M_rows).tocsr()
    x0 = np.clip(W.ravel(), np.array(lo[:m * n]), np.array(hi[:m * n]))
    x, it = _qp(P, q, M, np.array(lo), np.array(hi), x0)
    return x.reshape(m, n), it


# ---------------------------------------------------------------- C2 spline per lane

def _fit(xs, ys, a, b, knots, end_a, end_b):
    """C2 cubic spline on [a, b] fitted to the samples by least squares, with value/slope/curvature fixed at both
    ends (end_a, end_b: (value, slope, curvature)) and, if the plain fit dips below zero (a lane opening from zero
    width), kept non-negative at the samples. Returns a PPoly."""
    inner = sorted(k for k in knots if a + 1e-6 < k < b - 1e-6)
    t = np.r_[[a] * 4, inner, [b] * 4]
    nb = len(t) - 4
    X = BSpline.design_matrix(np.clip(xs, a, b), t, 3).toarray()
    basis = [BSpline(t, np.eye(nb)[i], 3) for i in range(nb)]
    C = np.array([[bs.derivative(o)(x) if o else bs(x) for bs in basis] for x in (a, b) for o in range(3)])
    d = np.r_[end_a, end_b]
    K = np.block([[X.T @ X + 1e-10 * np.eye(nb), C.T], [C, np.zeros((6, 6))]])
    coef = np.linalg.solve(K, np.r_[X.T @ ys, d])[:nb]
    if np.min(X @ coef) < 0.0 <= min(end_a[0], end_b[0]):
        M = sparse.csr_matrix(np.vstack([C, X]))
        lo = np.r_[d, np.zeros(len(xs))]
        hi = np.r_[d, np.full(len(xs), np.inf)]
        P = sparse.csc_matrix(2 * (X.T @ X) + 1e-10 * np.eye(nb))
        coef, _ = _qp(P, -2 * (X.T @ ys), M, lo, hi, coef)
    return PPoly.from_spline(BSpline(t, coef, 3))


def _write(lane, pieces, s0):
    """Replace the lane's width records by spline pieces [(x0, a, b, c, d)] (x0 absolute)."""
    old = lane.findall("width")
    at = list(lane).index(old[0]) if old else len(lane.findall("link"))
    tail = old[-1].tail if old else "\n"
    for el in old:
        lane.remove(el)
    for j, (x0, a, b, c, d) in enumerate(pieces):
        el = etree.Element("width", sOffset=repr(round(float(x0 - s0), 9)), a=repr(float(a)), b=repr(float(b)),
                           c=repr(float(c)), d=repr(float(d)))
        el.tail = tail
        lane.insert(at + j, el)


# ---------------------------------------------------------------- checks

def _centre_bend(road_el, xs):
    """Largest lateral bend |t''| / (1 + t'^2)^1.5 of the departure lane centres (lanes over 0.5 m) over xs."""
    worst = 0.0
    for x in xs:
        e = lane_edges_kinematics_at(road_el, float(x), "left")
        for (t1, a1, b1), (t2, a2, b2) in zip(e, e[1:]):
            if abs(t2 - t1) > 0.5:
                d1, d2 = (a1 + a2) / 2, (b1 + b2) / 2
                worst = max(worst, abs(d2) / (1 + d1 * d1) ** 1.5)
    return worst


def _continuity(road_el, x):
    """Largest jump of value, slope and curvature among the departure boundaries across x (boundaries matched by
    position; coincident boundaries of zero-width lanes are matched as a group by their outermost slope)."""
    before = lane_edges_kinematics_at(road_el, x - 1e-7, "left")
    after = lane_edges_kinematics_at(road_el, x + 1e-7, "left")
    worst = 0.0
    for t, d1, d2 in after:
        cand = [e for e in before if abs(e[0] - t) < 1e-3]
        if not cand:
            continue
        worst = max(worst, min(max(abs(e[0] - t), abs(e[1] - d1), abs(e[2] - d2)) for e in cand))
    return worst


# ---------------------------------------------------------------- road

def ease_stretch(road: _Road, i0, i1):
    A, B = road.s[i0], road.s[i1]
    chains, problem = _chains(road, i0, i1)
    if problem:
        return {"from_m": A, "to_m": B, "skipped": problem}
    xs = np.linspace(A, B, max(8, int(round((B - A) / STEP_M)) + 1))
    xe = xs.copy()
    xe[0], xe[-1] = A + 1e-6, B - 1e-6
    for c in chains:
        c["start_x"] = road.s[c["start"]]
    t0, W, F = _layout(road, chains, xe)
    Wn, iters = _solve(W, F, chains, xs, i0, i1)
    bends_x = xe[::2]
    before = _centre_bend(road.el, bends_x)
    mouth = B >= road.length - 1e-9
    mouth_old = lane_edges_kinematics_at(road.el, B - 1e-6, "left") if mouth else None
    steps_old = max(section_boundary_steps(road.el), default=0.0)
    joins = [x for x in road.s[i0:i1 + 1] if 1e-9 < x < road.length - 1e-9]
    join_old = max((_continuity(road.el, x) for x in joins), default=0.0)
    keep = copy.deepcopy(road.el)
    for k, c in enumerate(chains):
        a = road.s[c["start"]]
        knots = set(road.s[c["start"]:i1 + 1])
        for i in range(c["start"], i1):
            span = road.s[i + 1] - road.s[i]
            cuts = max(1, int(math.ceil(span / KNOT_M - 1e-9)))
            knots.update(road.s[i] + span * np.arange(1, cuts) / cuts)
        sel = xs >= a - 1e-9
        dense = np.linspace(a, B, max(16, int(round((B - a) / 0.1)) + 1))
        fit_y = CubicSpline(xs[sel], Wn[k][sel])(dense) if sel.sum() >= 4 else np.interp(dense, xs, Wn[k])
        pp = _fit(dense, fit_y, a, B, sorted(knots), c["start_state"], c["end_state"])
        lanes = c["members"]
        for i in range(c["start"], i1):
            pieces = [(x0, cf[3], cf[2], cf[1], cf[0]) for x0, x1, cf in zip(pp.x[:-1], pp.x[1:], pp.c.T)
                      if x1 - x0 > 1e-9 and road.s[i] - 1e-9 <= x0 < road.s[i + 1] - 1e-9]
            _write(lanes[i], pieces, road.s[i])
    # checks on the written result
    fine = np.linspace(A + 1e-6, B - 1e-6, max(16, int((B - A) / 0.25)))
    widths = min(min(e2[0] - e1[0] for e1, e2 in zip(e, e[1:])) if len(e) > 1 else 0.0
                 for e in (lane_edges_kinematics_at(road.el, float(x), "left") for x in fine))
    steps_new = max(section_boundary_steps(road.el), default=0.0)
    join_jump = max((_continuity(road.el, x) for x in joins), default=0.0)
    mouth_jump = 0.0
    if mouth:
        new = lane_edges_kinematics_at(road.el, B - 1e-6, "left")
        mouth_jump = max((abs(p - q) for ea, eb in zip(mouth_old, new) for p, q in zip(ea, eb)), default=0.0) \
            if len(new) == len(mouth_old) else 1e9
    after = _centre_bend(road.el, bends_x)
    opened = []
    for k in range(len(chains)):
        old = np.nonzero(W[k] > THROUGH_MIN_M)[0]
        new = np.nonzero(Wn[k] > THROUGH_MIN_M)[0]
        if len(old) and old[0] > 0 and len(new):
            opened.append(round(float(xs[old[0]] - xs[new[0]]), 2))
    row = {"from_m": round(A, 3), "to_m": round(B, 3), "lanes": len(chains), "opened_earlier_m": opened,
           "qp_iterations": iters, "centre_bend_max_per_m": [round(before, 5), round(after, 5)],
           "width_change_max_m": round(float(np.max(np.abs(Wn - W))), 4),
           "checks": {"width_min_m": round(widths, 6), "section_step_max_m": [steps_old, steps_new],
                      "boundary_jump": [join_old, join_jump], "mouth_jump": mouth_jump}}
    reason = ("negative width" if widths < -1e-6 else
              "step at a section boundary" if steps_new > max(steps_old, STEP_TOL_M) + 1e-9 else
              "section boundary not continued" if join_jump > join_old + END_TOL else
              "mouth moved" if mouth_jump > END_TOL else
              "no smoother" if after >= before else None)
    if reason:
        road.el.getparent().replace(road.el, keep)
        road.__init__(keep)
        row["skipped"] = reason
    return row


def ease_road(road_el):
    road = _Road(road_el)
    rows = []
    for i0, i1 in _stretches(road):
        rows.append(ease_stretch(road, i0, i1))
        road = _Road(road.el)
    return road.el, rows


def apply(root):
    """Ease the mirrored departure side of every ordinary MAP road (left lanes, junction at the road end)."""
    out = {"schema": CODE, "roads": []}
    for road_el in list(root.findall("road")):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        succ = road_el.find("link/successor")
        if succ is None or succ.get("elementType") != "junction":
            continue
        roles = {json.loads(u.get("value")).get("role") for u in road_el.iter("userData")
                 if u.get("code") == "mapforge.provenance/v1" and u.getparent().getparent().tag == "left"}
        if roles != {"departure"}:
            continue
        el, rows = ease_road(road_el)
        if rows:
            el.append(etree.Element("userData", code=CODE, value=json.dumps(rows, ensure_ascii=False)))
            out["roads"].append({"road": el.get("id"), "stretches": rows})
    out["eased"] = sum(1 for r in out["roads"] for x in r["stretches"] if "skipped" not in x)
    return out
