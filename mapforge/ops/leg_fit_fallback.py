"""SHP leg reference lines the converter's own fit rejects (2026-10-08).

``refline_fit.fit_leg_refline`` (bound by evidence) fits each leg's ROADLINK centre polyline with at most five
clothoid primitives under 60 km/h caps (|kappa| <= 0.009, |dkappa/ds| <= 0.000216, both end curvatures 0) and
raises when nothing fits within 1.5 m. On 16 SHP junctions the scoreboard never sees, 8 conversions stopped there.
The legs fell into three kinds:

* long legs (a 594 m link): five primitives are too few;
* centre lines that jump where the lane count changes (a lane added on one side moves the carriageway centre by
  half a lane width, 1-2 m in 10-17 m) while the lanes themselves run straight: no smooth line stays within 1.5 m of
  the jump, and none needs to, because the converter places the lanes by laneOffset and widths from their own
  boundaries (shp_to_xodr: "the reference line may lie inside the lane group");
* far chained links that turn hard (a 9 m S-shift in 20 m 170 m out, a 34 degree bend): the leg is an approach
  corridor, and ``_chained_links`` already stops at corners (> 50 degrees).

Here, only when the converter's fit raises, the leg is fitted again (the converter itself is not changed):

1. a G2 clothoid chain with equal segments 25 m (then 15 m) long, both end curvatures 0, the same caps, fitted with
   a Huber term to the centre polyline and a Huber term to the lane heading field (median heading of the leg's lane
   centre lines and boundaries, where at least two of them cover a station). It is accepted when it runs with the
   lanes (heading off the field by P95 <= HEAD_P95_DEG and max <= HEAD_MAX_DEG, i.e. widths along the reference
   normal off by at most 1 %) and either stays as close to the centre polyline as the converter's own fallback
   (median 0.6 m, P95 1.25 m, max 1.5 m) outside the last ZONE_M at the junction, with at most JN_TOL_M inside, or,
   where the lane field covers the leg (FIELD_LANE), keeps the median within 0.6 m and the maximum within JN_TOL_M
   (a centre line jumping by up to a lane width);
2. otherwise the conversion is run again with the leg shortened by its farthest chained link (its departure chains
   capped at the same length), until the fit passes or only the seed link is left;
3. a single-link leg that still fails may use higher caps, each a lower design speed with the same lateral
   acceleration and jerk (|kappa| <= k, |dkappa/ds| <= J (k/A)^1.5), and free end curvatures; the converter takes
   the leg's end curvature into the mouth poses and connectors.

Every accepted leg, shortened leg and relaxed leg is recorded (``report``). Legs the converter fits itself are never
touched, so conversions that succeeded before are byte-identical.
"""
from __future__ import annotations

import contextlib
import math
import re

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from mapforge.ops import refline_fit as RF

CODE = "mapforge.leg_fit_fallback/v1"
BASE_KAPPA, BASE_SHARP = 0.009, 0.000216   # fit_leg_refline's caps (60 km/h, 2.5 m/s^2, 1.0 m/s^3)
A_LAT, J_LAT = 2.5, 1.0                    # m/s^2, m/s^3: the same comfort limits at lower design speeds
RELAXED_KAPPA = (0.015, 0.025, 0.04, 0.065, 0.1)
SPACINGS_M = (25.0, 15.0)
ZONE_M, ZONE_FRAC = 30.0, 0.4              # junction-end zone: min(ZONE_M, ZONE_FRAC * leg length)
JN_TOL_M, FAR_TOL_M = 3.5, 1.5             # lateral end offsets (junction / far end); 3.5 m = one lane
POS_MED_M, POS_P95_M, POS_MAX_M = 0.6, 1.25, 1.5   # fit_leg_refline's own fallback tolerances
HEAD_P95_DEG, HEAD_MAX_DEG = 5.0, 8.0      # 1 - cos(8 deg) = 1 % width error along the reference normal
FIELD_MIN, FIELD_LANE = 0.3, 0.7           # lane-field coverage: heading checked / centre jumps tolerated
FLIPS_PER_100M = 8.0                       # fit_leg_refline's flips_per_100m_cap
FIELD_BAND_M = 30.0
MAX_REBUILDS = 12


def sharp_for(kappa):
    """|dkappa/ds| cap of a curvature cap at the same lateral acceleration and jerk: J (k / A)^1.5."""
    return J_LAT * (kappa / A_LAT) ** 1.5


def _arclen(p):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]


def _at_fraction(p, frac):
    s = _arclen(p)
    q = np.asarray(frac) * s[-1]
    return np.column_stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])])


def _project(poly, pts, chunk=2048):
    """Station and signed lateral offset (left +) of points on a polyline (nearest point on its segments)."""
    s = _arclen(poly)
    a, ab = poly[:-1], np.diff(poly, axis=0)
    l2 = np.maximum((ab * ab).sum(1), 1e-12)
    st, off = [], []
    for k in range(0, len(pts), chunk):
        p = pts[k:k + chunk][:, None, :]
        t = np.clip(((p - a[None]) * ab[None]).sum(2) / l2[None], 0.0, 1.0)
        d = np.linalg.norm(p - (a[None] + t[..., None] * ab[None]), axis=2)
        i = np.argmin(d, axis=1)
        r = np.arange(len(i))
        cross = ab[i, 0] * (p[r, 0, 1] - a[i, 1]) - ab[i, 1] * (p[r, 0, 0] - a[i, 0])
        st.append(s[i] + t[r, i] * np.sqrt(l2[i]))
        off.append(np.copysign(d[r, i], cross))
    return np.concatenate(st), np.concatenate(off)


def heading_field(center, polylines, step=1.0, smooth_m=3.0):
    """Median heading of lane centre lines/boundaries along ``center`` on a station grid; NaN where fewer than two
    of them (within FIELD_BAND_M) cover a station. Returns (grid, heading, centre heading)."""
    total = _arclen(center)
    grid = np.arange(0.0, total[-1] + 1e-9, step)
    seg = np.diff(center, axis=0)
    own = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    mids = 0.5 * (total[1:] + total[:-1])
    base = np.interp(grid, mids, own) if len(mids) > 1 else np.full(len(grid), own[0])
    bins = [dict() for _ in grid]
    for k, line in enumerate(polylines):
        line = np.asarray(line, float)
        if len(line) < 2 or _arclen(line)[-1] < 1.0:
            continue
        q = _at_fraction(line, np.linspace(0, 1, max(3, int(_arclen(line)[-1] / 0.5) + 1)))
        d = np.diff(q, axis=0)
        st, off = _project(center, 0.5 * (q[1:] + q[:-1]))
        ok = (np.abs(off) <= FIELD_BAND_M) & (st > 1e-6) & (st < total[-1] - 1e-6)
        if not ok.any():
            continue
        rel = np.angle(np.exp(1j * (np.arctan2(d[:, 1], d[:, 0]) - np.interp(st, grid, base))))
        if np.median(np.abs(rel[ok])) > math.pi / 2:          # a lane of the other direction
            rel = np.angle(np.exp(1j * (rel + math.pi)))
        for s_, r_ in zip(st[ok], rel[ok]):
            if abs(r_) < math.radians(60):
                bins[int(min(len(grid) - 1, round(s_ / step)))].setdefault(k, []).append(r_)
    med = np.array([np.median([np.median(v) for v in b.values()]) if len(b) >= 2 else np.nan for b in bins])
    w = max(0, int(round(smooth_m / step)))
    out = np.full(len(grid), np.nan)
    for i in np.flatnonzero(np.isfinite(med)):
        v = med[max(0, i - w):i + w + 1]
        out[i] = np.median(v[np.isfinite(v)])
    return grid, base + out, base


class _Chain:
    """Clothoid chain of n equal segments with knot curvatures, integrated on a fine grid (optimiser only)."""

    def __init__(self, n, length, kappa_cap):
        self.n = n
        ds = min(0.5, length / 300.0, 0.05 / max(kappa_cap, 1e-3))
        self.m = max(4, int(math.ceil(length / n / ds)))
        self.sig = np.linspace(0.0, 1.0, n * self.m + 1)
        self.knots = np.linspace(0.0, 1.0, n + 1)

    def eval(self, x0, y0, h0, length, kap):
        k = np.interp(self.sig, self.knots, kap)
        ds = length / (self.n * self.m)
        th = h0 + np.r_[0.0, np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)]
        c, s = np.cos(th), np.sin(th)
        x = x0 + np.r_[0.0, np.cumsum(0.5 * (c[1:] + c[:-1]) * ds)]
        y = y0 + np.r_[0.0, np.cumsum(0.5 * (s[1:] + s[:-1]) * ds)]
        return np.column_stack([x, y]), th


def _huber(e, d):
    a = np.abs(e)
    return np.where(a <= d, 0.5 * a * a, d * (a - 0.5 * d))


def _fit_once(src, field, *, n, kappa_cap, sharp_cap, junction_end, free_ends, w_head=200.0, delta=0.5):
    total = float(_arclen(src)[-1])
    chain = _Chain(n, total, kappa_cap)
    target = _at_fraction(src, chain.sig)
    grid, heading, base = field
    f_sig = np.interp(chain.sig * total, grid, heading)
    has_f = np.isfinite(f_sig)
    th0 = np.interp(chain.sig * total, grid, np.where(np.isfinite(heading), heading, base))
    k_init = np.clip(np.interp(chain.knots, chain.sig, np.gradient(th0, chain.sig * total)), -kappa_cap, kappa_cap)
    fixed = [] if free_ends else [0, n]
    free = [j for j in range(n + 1) if j not in fixed]
    k_init[fixed] = 0.0

    def unpack(z):
        kap = np.zeros(n + 1)
        kap[free] = z[4:]
        return z[0], z[1], z[2], z[3], kap

    def objective(z):
        x0, y0, h0, length, kap = unpack(z)
        pts, th = chain.eval(x0, y0, h0, length, kap)
        val = _huber(np.linalg.norm(pts - target, axis=1), delta).mean()
        if has_f.any():
            val += w_head * _huber(np.angle(np.exp(1j * (th[has_f] - f_sig[has_f]))), 0.05).mean()
        sharp = np.diff(kap) / (length / n)
        return float(val + 0.02 * np.mean(sharp * sharp) * length * length)

    def ends(z):
        x0, y0, h0, length, kap = unpack(z)
        pts, th = chain.eval(x0, y0, h0, length, kap)
        t0 = np.array([math.cos(th[0]), math.sin(th[0])])
        t1 = np.array([math.cos(th[-1]), math.sin(th[-1])])
        e0, e1 = src[0] - pts[0], src[-1] - pts[-1]
        return np.array([e0 @ t0, e1 @ t1, e0 @ [-t0[1], t0[0]], e1 @ [-t1[1], t1[0]]])

    tol = np.array([FAR_TOL_M, JN_TOL_M] if junction_end == "end" else [JN_TOL_M, FAR_TOL_M])
    cons = [{"type": "eq", "fun": lambda z: ends(z)[:2]},                 # the leg ends at the source stations
            {"type": "ineq", "fun": lambda z: tol - np.abs(ends(z)[2:])},
            {"type": "ineq", "fun": lambda z: np.r_[sharp_cap * z[3] / n - np.diff(unpack(z)[4]),
                                                    sharp_cap * z[3] / n + np.diff(unpack(z)[4])]}]
    z0 = np.r_[src[0, 0], src[0, 1], RF._robust_endpoint_heading(src, True), total, k_init[free]]
    bounds = [(None, None)] * 3 + [(0.8 * total, 1.2 * total)] + [(-kappa_cap, kappa_cap)] * len(free)
    sol = minimize(objective, z0, method="SLSQP", bounds=bounds, constraints=cons,
                   options={"maxiter": 400, "ftol": 1e-10})
    x0, y0, h0, length, kap = unpack(sol.x)
    pv = RF._canonicalize_connector(RF._chain_from_knots((x0, y0, h0), np.full(n, length / n), kap))
    return pv, {"solver_success": bool(sol.success), "end_offsets_m": [round(float(e), 4) for e in ends(sol.x)]}


def assess(pv, src, field, junction_end):
    """Exact (pyclothoids) deviations from the centre polyline, split at the junction-end zone, and the heading off
    the lane field at the source stations."""
    total = sum(s.length for s in pv.segs)
    curve = RF._planview_at_s(pv, np.linspace(0.0, total, max(200, int(total / 0.1) + 1)))
    s = _arclen(src)
    qs = np.linspace(0.0, s[-1], max(41, int(s[-1] / 0.25) + 1))
    q = np.column_stack([np.interp(qs, s, src[:, 0]), np.interp(qs, s, src[:, 1])])
    d_s2t, idx = cKDTree(curve).query(q)
    d_t2s, _ = cKDTree(q).query(curve)
    zone = min(ZONE_M, ZONE_FRAC * float(s[-1]))
    inz = ((s[-1] - qs) if junction_end == "end" else qs) <= zone
    out = {"zone_m": round(zone, 3), "median_m": float(np.median(d_s2t)),
           "outside_median_m": float(np.median(d_s2t[~inz])) if (~inz).any() else 0.0,
           "outside_p95_m": float(np.percentile(d_s2t[~inz], 95)) if (~inz).any() else 0.0,
           "outside_max_m": float(d_s2t[~inz].max()) if (~inz).any() else 0.0,
           "zone_max_m": float(d_s2t[inz].max()), "max_m": float(max(d_s2t.max(), d_t2s.max()))}
    grid, heading, _ = field
    dc = np.gradient(curve, axis=0)
    f = np.interp(qs, grid, heading)
    ok = np.isfinite(f)
    dh = np.degrees(np.abs(np.angle(np.exp(1j * (np.arctan2(dc[:, 1], dc[:, 0])[idx] - f)))))[ok]
    out["field_coverage"] = float(ok.mean())
    out["heading_p95_deg"] = float(np.percentile(dh, 95)) if ok.any() else None
    out["heading_max_deg"] = float(dh.max()) if ok.any() else None
    quality = RF.planview_quality(pv)
    out.update(kappa_max=quality["kappa_max"], sharpness_max=quality["sharpness_max"],
               flips_per_100m=quality["sharp_sign_flips_per_100m"], primitives=quality["n_segs"],
               min_primitive_m=quality["seg_min_len"])
    return out


def accepted(a):
    """(ok, reasons) under the rules of the module docstring."""
    heading = a["field_coverage"] < FIELD_MIN or (a["heading_p95_deg"] <= HEAD_P95_DEG
                                                  and a["heading_max_deg"] <= HEAD_MAX_DEG)
    near = (a["outside_median_m"] <= POS_MED_M and a["outside_p95_m"] <= POS_P95_M
            and a["outside_max_m"] <= POS_MAX_M and a["max_m"] <= JN_TOL_M + 1e-6)
    lanes = a["field_coverage"] >= FIELD_LANE and a["median_m"] <= POS_MED_M and a["max_m"] <= JN_TOL_M + 1e-6
    return bool(heading and (near or lanes) and a["flips_per_100m"] <= FLIPS_PER_100M), \
        {"heading": bool(heading), "near_centre": bool(near), "with_lanes": bool(lanes)}


def fit_leg(gxy, polylines, junction_end, relaxed=False):
    """Fallback fit of one leg. Returns (planview, record) with planview None when no candidate is accepted."""
    src, impulse = RF._remove_impulse_outliers(np.asarray(gxy, float))
    field = heading_field(src, polylines)
    total = float(_arclen(src)[-1])
    levels = [(BASE_KAPPA, BASE_SHARP, False)] + ([(k, sharp_for(k), True) for k in RELAXED_KAPPA] if relaxed else [])
    record = {"impulse_filter": impulse, "tried": []}
    for kappa_cap, sharp_cap, free in levels:
        for spacing in SPACINGS_M:
            n = max(3, int(math.ceil(total / spacing)))
            while n > 3 and total / n < max(3.0, 0.03 * total):
                n -= 1
            pv, info = _fit_once(src, field, n=n, kappa_cap=kappa_cap, sharp_cap=sharp_cap,
                                 junction_end=junction_end, free_ends=free)
            a = assess(pv, src, field, junction_end)
            ok, why = accepted(a)
            row = {"kappa_cap": kappa_cap, "sharp_cap": sharp_cap, "free_end_curvature": free, "segments": n,
                   **info, **a, **why, "accepted": ok}
            record["tried"].append(row)
            if ok:
                record["chosen"] = row
                return pv, record
    return None, record


# ---------------------------------------------------------------- converter hooks
class State:
    """One junction conversion: chain caps, relaxed legs, the call trace and the report."""

    def __init__(self, src):
        self.src = src
        self.calls = []            # (seed, is_enter, chain, proj) since the last leg fit
        self.chains = {}           # (seed, is_enter) -> [(pid, length_m)] junction side first
        self.link_caps = {}        # enter seed -> links kept
        self.length_caps = {}      # departure seed -> metres kept
        self.relaxed = set()       # (kind, seed)
        self.fitted = []           # legs fitted here
        self.last_leg = None

    def report(self):
        return {"schema": CODE, "legs": self.fitted,
                "truncated_enter_links": {k: v for k, v in sorted(self.link_caps.items())},
                "capped_departure_m": {k: round(v, 3) for k, v in sorted(self.length_caps.items())},
                "relaxed": sorted(f"{k}:{s}" for k, s in self.relaxed)}


def _chain_hook(state, original):
    def chained_links(src, seed_pid, is_enter, proj, cj, max_len=160.0, hops=5):
        if not is_enter and seed_pid in state.length_caps:
            max_len = min(max_len, state.length_caps[seed_pid])
        chain = original(src, seed_pid, is_enter, proj, cj, max_len=max_len, hops=hops)
        if not chain:
            return chain
        if is_enter and seed_pid in state.link_caps and len(chain) > state.link_caps[seed_pid]:
            chain = chain[-state.link_caps[seed_pid]:]          # enter chains run upstream -> junction
        if not is_enter and seed_pid in state.length_caps:
            # departure chains run junction -> downstream; no longer than the shortened leg plus the 8 m the
            # converter allows before it extends a leg's reference from the opposite chain
            while len(chain) > 1 and sum(_polylen(proj(c)) for _p, c in chain) > state.length_caps[seed_pid] + 8.0:
                chain = chain[:-1]
        ordered = chain[::-1] if is_enter else chain
        state.chains[(seed_pid, is_enter)] = [(p, _polylen(proj(c))) for p, c in ordered]
        state.calls.append((seed_pid, is_enter, chain, proj))
        return chain
    return chained_links


def _polylen(g):
    g = np.asarray(g, float)
    return float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()) if len(g) > 1 else 0.0


def _fit_hook(state, original):
    def fit_leg_refline(pts, *args, **kwargs):
        enters = [c for c in state.calls if c[1]]
        leg = enters[-1] if enters else (state.calls[-1] if state.calls else None)
        state.calls = []
        kind = "enter" if (leg and leg[1]) else "independent_leave"
        state.last_leg = (kind, leg[0] if leg else None)
        try:
            return original(pts, *args, **kwargs)
        except RF.ReflineFitError:
            if leg is None:
                raise
            seed, _is_enter, chain, proj = leg
            polylines = []
            for pid, _c in chain:
                for lane in state.src.lanes_of(pid):
                    if len(lane.geometry) >= 2:
                        polylines.append(proj(lane.geometry))
                    for b in state.src.lane_boundary_geometries(lane.lane_pid):
                        if len(b) >= 2:
                            polylines.append(proj(b))
            pv, record = fit_leg(pts, polylines, "end" if kind == "enter" else "start",
                                 relaxed=(kind, seed) in state.relaxed)
            entry = {"kind": kind, "seed": seed, "links": [p for p, _c in chain], "accepted": pv is not None,
                     "chosen": record.get("chosen"), "tried": len(record["tried"])}
            if pv is None:
                entry["last"] = record["tried"][-1]
                state.fitted.append(entry)
                raise
            state.fitted.append(entry)
            chosen = record["chosen"]
            pv.fit_meta.update({"impulse_filter": record["impulse_filter"], "fit_selection": f"{CODE}",
                                "leg_fit_fallback": chosen})
            return pv, float(chosen["max_m"]), True
    return fit_leg_refline


@contextlib.contextmanager
def installed(state):
    """Route the converter's leg fits and chains through this module for the duration of one build."""
    from mapforge.ops import shp_to_xodr as S
    fit, chain = S.fit_leg_refline, S._chained_links
    S.fit_leg_refline, S._chained_links = _fit_hook(state, fit), _chain_hook(state, chain)
    try:
        yield state
    finally:
        S.fit_leg_refline, S._chained_links = fit, chain


_LEG = re.compile(r"(enter_link|independent_leave_link)=(\d+)")


def build(src, junc, out_path, **kwargs):
    """``shp_to_xodr.build_junction_xodr`` with the fallback; returns (stats, report or None). The report is None
    when the converter fitted every leg itself (the build is then exactly the converter's)."""
    from mapforge.ops import shp_to_xodr as S
    state = State(src)
    for _attempt in range(MAX_REBUILDS):
        state.calls, state.chains = [], {}
        try:
            with installed(state):
                stats = S.build_junction_xodr(src, junc, out_path, **kwargs)
        except RF.ReflineFitError as exc:
            if not _next_step(state, exc):
                raise
            continue
        except S.CandidateSurfaceError as exc:
            if state.fitted or state.link_caps or state.length_caps:
                exc.stats["leg_fit_fallback"] = state.report()
            raise
        used = bool(state.fitted or state.link_caps or state.length_caps)
        if used:
            stats["leg_fit_fallback"] = state.report()
        return stats, (state.report() if used else None)
    raise RF.ReflineFitError(f"leg fit fallback: no result after {MAX_REBUILDS} rebuilds")


def _next_step(state, exc):
    """After a failed leg: shorten it by its farthest chained link, else allow the relaxed caps once."""
    message = " ".join(str(exc).split())
    m = _LEG.search(message)
    if m is None or state.last_leg is None or state.last_leg != (
            "enter" if m.group(1) == "enter_link" else "independent_leave", m.group(2)):
        return False
    kind, seed = state.last_leg
    is_enter = kind == "enter"
    chain = state.chains.get((seed, is_enter)) or []
    if len(chain) > 1 and (kind, seed) not in state.relaxed:
        keep = chain[:-1]
        # _chained_links adds links while the chain is shorter than max_len: just below the kept length
        cap = sum(length for _p, length in keep) - 1e-6
        if is_enter:
            state.link_caps[seed] = len(keep)
            leaves = re.search(r"leave_links=\[([^\]]*)\]", message)
            for lp in re.findall(r"\d+", leaves.group(1) if leaves else ""):
                state.length_caps[lp] = min(state.length_caps.get(lp, math.inf), cap)
        else:
            state.length_caps[seed] = cap
        return True
    if (kind, seed) in state.relaxed:
        return False
    state.relaxed.add((kind, seed))
    return True
