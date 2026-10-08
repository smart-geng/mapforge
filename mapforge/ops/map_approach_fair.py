"""MAP approach sides faired within a band of their MAP point lists (2026-10-07).

map_lane_refit fits the approach lane boundaries so that each lane centre follows its MAP point list as a
densified polyline (the chords between the points), with G2 corners at level A (corner curvature 0.04 /m, RDP
5 cm, 5 m runs). MAP point lists are thinned (T/CSAE 159 Appendix D: the chord between two points stays within
the generator's tolerance of the true centre line, a tolerance the files do not state) and sparse: 11-53 m between
points with up to 11 deg at a point. So the lane centres turn within a few metres at each point: up to 0.057 /m
on approach through lanes after the departure easing (map-node18 road 11: points 52 m apart, 9-11 deg turns).

Here the approach side of each ordinary MAP road (right lanes, junction at the road end) is faired: the lane
offset (the centre line, the approach side's inner boundary) and the approach lane widths are chosen again (a
quadratic programme solved by ADMM, map_departure_ease._qp) to minimise the squared third differences of all
approach lane boundaries, subject to
- a lane centre with a MAP point list never gets farther from that list (measured across the road at every
  sample) than it is now or than TAU_M, whichever is larger: no point moves away from its source beyond TAU_M,
  and where the fit is already farther, not at all; where the list does not reach, within TAU_M of now.
  (T/CSAE 159-2020 only asks the chords to stay within ActualError <= 0.5 m of the true lane centre, 6.4.7.6 and
  Appendix D. Against the SHP HD map of the same junctions the MAP points lie 4-10 mm from the SHP lane centres,
  median, while the chords are 5-11 cm off and up to 0.13-0.77 m, P95; still, a 0.10 m band with the points held
  within 5 cm left the distance to the SHP lanes unchanged, +-5 mm, and a smooth curve through the points was
  farther from them in five of seven junctions: the chords are the better guess, so TAU_M stays 5 cm, 2026-10-07);
- an open lane keeps its width within BAND_M of the current one, a closed (zero-width) stretch stays as it is,
  widths stay >= 0;
- value, slope and curvature of the centre line and of every approach width stay as they are at both road ends
  (the stop-line mouth and the far end).
The departure lanes stack on the centre line, so they follow it; map_departure_ease runs afterwards. A road keeps
its old geometry unless the result holds the band, the ends and non-negative widths and bends less.
"""
from __future__ import annotations

import copy
import json
import math

import numpy as np
from lxml import etree
from scipy import sparse

from mapforge.ops.map_departure_ease import (BETA, CONTINUE_M, END_TOL, GAMMA, KNOT_M, PLACEHOLDER_M,
                                             THROUGH_MIN_M, _fit, _qp, _state, _width, _write)
from mapforge.validate.smoothness import lane_edges_kinematics_at, sample_road_ref

CODE = "mapforge.map_approach_fair/v1"
STEP_M = 0.5
TAU_M = 0.05          # a lane centre may get this far from its MAP point list (the refit's RDP tolerance)
BAND_M = 0.3          # an open approach lane's width stays within this of its current width
BAND_TOL_M = 0.002    # written result vs the band
FIT_MARGIN_M = 0.01   # the QP keeps this much inside the band, for the C2 spline written from its samples
MIN_BEND = 0.003      # a road whose approach lanes bend less than this [1/m] is left alone


def _right(sec):
    return sorted(sec.findall("right/lane"), key=lambda ln: -int(ln.get("id")))


def _chains(secs, s):
    """Approach lanes of the last section traced back by predecessor links or same-id continuation (as
    map_departure_ease._chains, right side). Returns (chains, problem)."""
    last = len(secs) - 1
    chains, owned = [], set()
    for lane in _right(secs[last]):
        members, i, ln = {last: lane}, last, lane
        owned.add(id(lane))
        while i > 0:
            p = ln.find("link/predecessor")
            if p is not None:
                prev = next((x for x in secs[i - 1].findall("right/lane") if x.get("id") == p.get("id")), None)
                if prev is None:
                    return None, "broken predecessor link"
            else:
                prev = next((x for x in secs[i - 1].findall("right/lane") if x.get("id") == ln.get("id")), None)
                if (prev is None or id(prev) in owned or prev.find("link/successor") is not None
                        or abs(_width(prev, s[i] - s[i - 1]) - _width(ln, 0.0)) > CONTINUE_M):
                    break
            i -= 1
            ln = prev
            members[i] = ln
            owned.add(id(ln))
        first = min(members)
        if first > 0 and _width(members[first], 0.0) >= THROUGH_MIN_M:
            return None, f"lane {lane.get('id')} starts {_width(members[first], 0.0):.2f} m wide"
        src = lane.find("userData[@code='mapforge.source_lane']")
        chains.append({"members": members, "start": first, "source": src.get("value") if src is not None else None})
    for i, sec in enumerate(secs):
        span = s[i + 1] - s[i]
        for ln in _right(sec):
            if id(ln) not in owned and max(abs(_width(ln, x)) for x in np.linspace(0, span, 9)) > PLACEHOLDER_M:
                return None, f"lane {ln.get('id')} of section {i} ends inside the road"
    return chains, None


def _across(points, normals, line):
    """Signed offset along each normal at which the polyline ``line`` crosses it (NaN where it does not)."""
    a, b = line[:-1], line[1:]
    d = b - a
    out = np.full(len(points), np.nan)
    for j, (p, nrm) in enumerate(zip(points, normals)):
        # p + u n = a + v d  ->  [n, -d] [u, v]' = a - p
        det = nrm[0] * (-d[:, 1]) - nrm[1] * (-d[:, 0])
        ok = np.abs(det) > 1e-12
        r = a - p
        u = np.where(ok, (r[:, 0] * (-d[:, 1]) - r[:, 1] * (-d[:, 0])) / np.where(ok, det, 1.0), np.nan)
        v = np.where(ok, (nrm[0] * r[:, 1] - nrm[1] * r[:, 0]) / np.where(ok, det, 1.0), np.nan)
        hit = ok & (v >= -1e-9) & (v <= 1 + 1e-9)
        if hit.any():
            out[j] = u[hit][np.argmin(np.abs(u[hit]))]
    return out


def fair_road(road_el, centres):
    """Fair the approach side of one road (see module doc); returns a report row."""
    L = float(road_el.get("length"))
    secs = road_el.findall("lanes/laneSection")
    s = [float(x.get("s")) for x in secs] + [L]
    chains, problem = _chains(secs, s)
    if problem:
        return {"skipped": problem}
    n = max(8, int(round(L / STEP_M)) + 1)
    xs = np.linspace(0.0, L, n)
    xe = xs.copy()
    xe[0], xe[-1] = 1e-6, L - 1e-6
    sec_of = np.clip(np.searchsorted(s, xe, side="right") - 1, 0, len(secs) - 1)
    m = len(chains)
    lane_of = {id(ln): k for k, c in enumerate(chains) for ln in c["members"].values()}
    W, fixed, t0 = np.zeros((m, n)), np.zeros((m, n)), np.zeros(n)
    for j, x in enumerate(xe):
        i = int(sec_of[j])
        t0[j] = lane_edges_kinematics_at(road_el, float(x), "right")[0][0]
        acc = 0.0
        for ln in _right(secs[i]):
            w = _width(ln, x - s[i])
            k = lane_of.get(id(ln))
            if k is None:
                acc += w
            else:
                W[k, j], fixed[k, j] = w, acc
    def bend(el):
        worst = 0.0
        for xx in xe[::2]:
            e = lane_edges_kinematics_at(el, float(xx), "right")
            for (ta, a1, a2), (tb, b1, b2) in zip(e, e[1:]):
                if abs(ta - tb) > 2.5:
                    d1, d2 = (a1 + b1) / 2, (a2 + b2) / 2
                    worst = max(worst, abs(d2) / (1 + d1 * d1) ** 1.5)
        return worst

    before = bend(road_el)
    if before < MIN_BEND:
        return {"lanes": m, "centre_bend_max_per_m": [round(before, 5), round(before, 5)], "skipped": "straight"}
    centre_now = np.array([t0 - W[:k].sum(axis=0) - 0.5 * W[k] - fixed[k] for k in range(m)])
    # the band: across the road at every sample, from the MAP point list where it reaches
    pts, ss, hh = sample_road_ref(road_el, ds=0.05)
    idx = np.clip(np.searchsorted(ss, xe), 0, len(ss) - 1)
    ref, normals = pts[idx], np.column_stack([-np.sin(hh[idx]), np.cos(hh[idx])])
    lo_c, hi_c = centre_now - TAU_M, centre_now + TAU_M
    banded = 0
    for k, c in enumerate(chains):
        line = centres.get(c["source"]) if c["source"] else None
        if line is None or len(line) < 2:
            continue
        u = _across(ref, normals, np.asarray(line, float)[:, :2])
        reach = np.isfinite(u)
        allow = np.maximum(TAU_M, np.abs(centre_now[k] - np.where(reach, u, 0.0)))
        lo_c[k] = np.where(reach, u - allow, lo_c[k])
        hi_c[k] = np.where(reach, u + allow, hi_c[k])
        banded += int(reach.sum())
    # QP in x = [t0, w_1 .. w_m]; boundaries E_0 = t0, E_k = t0 - sum_{j<=k} w_j - fixed_k
    N = (m + 1) * n
    eye, zero = sparse.identity(n, format="csr"), sparse.csr_matrix((n, n))
    D3 = sparse.diags([-1.0, 3.0, -3.0, 1.0], [0, 1, 2, 3], shape=(n - 3, n), format="csr")
    blocks, rhs = [D3 @ sparse.hstack([eye] + [zero] * m)], [np.zeros(n - 3)]
    for k in range(m):
        blocks.append(D3 @ sparse.hstack([eye] + [-eye if i <= k else zero for i in range(m)]))
        rhs.append(D3 @ fixed[k])
    A = sparse.vstack(blocks).tocsr()
    b = np.concatenate(rhs)
    scale = 1.0 / (20.0 * (m + 1))
    own = sparse.block_diag([zero] + [D3.T @ D3] * m)
    cur = np.concatenate([t0, W.ravel()])
    P = (2 * scale * (A.T @ A + GAMMA * own) + 2 * BETA * sparse.identity(N)).tocsc()
    q = -2 * scale * (A.T @ b) - 2 * BETA * cur
    lo, hi = [np.full(n, -np.inf)], [np.full(n, np.inf)]
    for v in (lo[0], hi[0]):
        v[:3], v[-3:] = t0[:3], t0[-3:]
    for k in range(m):
        w = W[k]
        l, h = np.maximum(w - BAND_M, 0.0), w + BAND_M
        closed = w <= THROUGH_MIN_M
        l[closed], h[closed] = w[closed], w[closed]
        l[:3], h[:3], l[-3:], h[-3:] = w[:3], w[:3], w[-3:], w[-3:]
        lo.append(l)
        hi.append(h)
    rows = [sparse.identity(N, format="csr")]
    for k in range(m):
        rows.append(sparse.hstack([eye] + [-eye if i < k else (-0.5 * eye if i == k else zero) for i in range(m)]))
        mid, half = (lo_c[k] + hi_c[k]) / 2, (hi_c[k] - lo_c[k]) / 2
        half = np.maximum(half - FIT_MARGIN_M, np.minimum(half, TAU_M - FIT_MARGIN_M))
        lo.append(mid - half + fixed[k])
        hi.append(mid + half + fixed[k])
    M = sparse.vstack(rows).tocsr()
    lo, hi = np.concatenate(lo), np.concatenate(hi)
    x, iters = _qp(P, q, M, lo, hi, np.clip(cur, lo[:N], hi[:N]))
    t_new, W_new = x[:n], x[n:].reshape(m, n)
    keep = copy.deepcopy(road_el)
    ends_old = [(lane_edges_kinematics_at(road_el, x, "right"), lane_edges_kinematics_at(road_el, x, "left"))
                for x in (1e-6, L - 1e-6)]
    # write the lane offset (whole road) and the approach widths as C2 splines
    offs = road_el.findall("lanes/laneOffset")
    first_offset = offs[0] if offs else None
    o_state = [lane_edges_kinematics_at(road_el, x, "right")[0] for x in (0.0, L - 1e-9)]
    knots = set(s)
    for i in range(len(secs)):
        span = s[i + 1] - s[i]
        cuts = max(1, int(math.ceil(span / KNOT_M - 1e-9)))
        knots.update(s[i] + span * np.arange(1, cuts) / cuts)
    knots = sorted(knots)
    dense = np.linspace(0.0, L, max(16, int(round(L / 0.1)) + 1))
    pp = _fit(dense, np.interp(dense, xs, t_new), 0.0, L, knots, tuple(o_state[0]), tuple(o_state[1]))
    lanes_el = road_el.find("lanes")
    at = list(lanes_el).index(first_offset) if first_offset is not None else 0
    tail = first_offset.tail if first_offset is not None else "\n"
    for el in offs:
        lanes_el.remove(el)
    pieces = [(x0, cf[3], cf[2], cf[1], cf[0]) for x0, x1, cf in zip(pp.x[:-1], pp.x[1:], pp.c.T) if x1 - x0 > 1e-9]
    for j, (x0, a, bb, c, d) in enumerate(pieces):
        el = etree.Element("laneOffset", s=repr(round(float(x0), 9)), a=repr(float(a)), b=repr(float(bb)),
                           c=repr(float(c)), d=repr(float(d)))
        el.tail = tail
        lanes_el.insert(at + j, el)
    for k, c in enumerate(chains):
        a = s[c["start"]]
        sel = xs >= a - 1e-9
        start_state = _state(c["members"][c["start"]], 0.0)
        end_state = _state(c["members"][len(secs) - 1], L - s[-2])
        dense_k = np.linspace(a, L, max(16, int(round((L - a) / 0.1)) + 1))
        ppk = _fit(dense_k, np.interp(dense_k, xs[sel], W_new[k][sel]), a, L,
                   [x for x in knots if x >= a - 1e-9], start_state, end_state)
        for i in range(c["start"], len(secs)):
            pcs = [(x0, cf[3], cf[2], cf[1], cf[0]) for x0, x1, cf in zip(ppk.x[:-1], ppk.x[1:], ppk.c.T)
                   if x1 - x0 > 1e-9 and s[i] - 1e-9 <= x0 < s[i + 1] - 1e-9]
            _write(c["members"][i], pcs, s[i])
    # checks
    after = bend(road_el)
    t_chk, W_chk = np.zeros(n), np.zeros((m, n))
    for j, x in enumerate(xe):
        i = int(sec_of[j])
        t_chk[j] = lane_edges_kinematics_at(road_el, float(x), "right")[0][0]
        for ln in _right(secs[i]):
            k = lane_of.get(id(ln))
            if k is not None:
                W_chk[k, j] = _width(ln, x - s[i])
    centre_new = np.array([t_chk - W_chk[:k].sum(axis=0) - 0.5 * W_chk[k] - fixed[k] for k in range(m)])
    band_miss = float(max(np.max(lo_c - centre_new), np.max(centre_new - hi_c), 0.0))
    widths_min = float(np.min(W_chk))
    ends_new = [(lane_edges_kinematics_at(road_el, x, "right"), lane_edges_kinematics_at(road_el, x, "left"))
                for x in (1e-6, L - 1e-6)]
    end_jump = max(abs(p - q) for (ro, lo_), (rn, ln_) in zip(ends_old, ends_new)
                   for ea, eb in list(zip(ro, rn)) + list(zip(lo_, ln_)) for p, q in zip(ea, eb))
    row = {"lanes": m, "banded_samples": banded, "qp_iterations": iters,
           "centre_bend_max_per_m": [round(before, 5), round(after, 5)],
           "centre_shift_max_m": round(float(np.max(np.abs(centre_new - centre_now))), 4),
           "offset_change_max_m": round(float(np.max(np.abs(t_chk - t0))), 4),
           "checks": {"band_miss_m": band_miss, "width_min_m": widths_min, "end_jump": end_jump}}
    reason = ("band" if band_miss > BAND_TOL_M else "negative width" if widths_min < -1e-6 else
              "ends moved" if end_jump > END_TOL else "no smoother" if after >= before else None)
    if reason:
        road_el.getparent().replace(road_el, keep)
        row["skipped"] = reason
        return row, keep
    return row, road_el


def apply(root, centres):
    """Fair the approach side of every ordinary MAP road (right lanes, junction at the road end).
    ``centres``: {source lane id: MAP point list (local xy)} (map_lane_refit.load_centres)."""
    out = {"schema": CODE, "roads": []}
    for road_el in list(root.findall("road")):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        succ = road_el.find("link/successor")
        if succ is None or succ.get("elementType") != "junction":
            continue
        roles = {json.loads(u.get("value")).get("role") for u in road_el.iter("userData")
                 if u.get("code") == "mapforge.provenance/v1" and u.getparent().getparent().tag == "right"}
        if roles != {"approach"}:
            continue
        res = fair_road(road_el, centres)
        row, el = res if isinstance(res, tuple) else (res, road_el)
        el.append(etree.Element("userData", code=CODE, value=json.dumps(row, ensure_ascii=False)))
        out["roads"].append({"road": el.get("id"), **row})
    out["faired"] = sum(1 for r in out["roads"] if "skipped" not in r)
    return out
