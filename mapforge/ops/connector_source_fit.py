"""Fidelity-first G2 refit of a one-lane connector to its source via line (2026-10-04).

The connector candidates of mouth_frame_align follow the converter's old path, or a few-segment chain that
only has to stay within the G8 ceilings (source-guided: the fewest segments, 1 to 5, with a lane P95 up to
0.75 m). SHP via lines are digitized polylines: a through movement across an offset junction is a straight
diagonal with an 8 deg kink at each end (shp-node17 roads 102, 119), a turn meets its lanes at up to
14 deg; a 3-4 segment chain cannot follow that, so SHP connector lanes stayed 0.2-1.5 m (lane P95) off their
via lines while ordinary lanes are within 0.03 m.

Here the lane centre is a clothoid chain (piecewise-linear curvature, G2) with knots about every
``seg_m`` along the via line and no segment shorter than MIN_SEGMENT_M (the project floor: no 0.x m pieces
that only look smooth). It starts and ends on the linked lane centres with their headings and curvatures,
and its knot curvatures and segment lengths minimise the two-way closest-point distance to the via line plus
a penalty on every change of the curvature rate (least squares, then an exact end-pose polish). Without
that penalty the chain chased the kinks where SHP lane tails meet their via lines (8-21 deg, the
source-topology-gap-bridge ends) with 3 m S-bends of 0.1-0.18 /m on straight-through movements, and the lane
edges jumped in curvature at its knots; with it, a source kink is rounded instead of followed, at the cost of
a larger distance there. Knots the penalty leaves inactive are merged, so straight and constant-curvature
stretches are single primitives. In a turn a second fit keeps every knot curvature on the turn's side.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

MIN_SEGMENT_M = 3.0
KAPPA_MAX = 0.25
SEGMENT_STEPS_M = (6.0, 4.0, 3.0)
SAMPLE_M = 0.5
EVAL_STEP_M = 0.1
END_WEIGHT = 100.0
TURN_SIGN_TOL = 0.005
RATE_JUMP_WEIGHT = 10.0    # per 1/m^2 of curvature-rate change, against metres of source distance
PRIMITIVE_COST_M = 0.01    # one more primitive must bring a chain this much closer to its source (P95); used for the
                           # knot merge here and in the connector pick (mouth_frame_align)
MERGE_RATE_JUMPS = (0.02, 0.005, 0.00125)   # inner knots with a smaller rate change are merged after the fit
MAX_NFEV = 400
GOOD_P95_M = 0.08         # a finer segment step is tried only while the fit stays farther than this
END_ZONE = (10.0, 0.02)   # end-zone turn fits: within this many metres of either end the inner knots may curve
                          # this much against the turn (user decision 2026-10-05)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _resample(pts, step=None, count=None):
    pts = np.asarray(pts, float)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    n = count if count is not None else max(2, int(math.ceil(s[-1] / step)) + 1)
    q = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(q, s, pts[:, 0]), np.interp(q, s, pts[:, 1])])


class _Chain:
    """Fast sampled clothoid chain from a start pose (heading integrated exactly, position by trapezoid)."""

    def __init__(self, x0, y0, h0):
        self.x0, self.y0, self.h0 = x0, y0, h0

    def sample(self, lens, ks, step=EVAL_STEP_M):
        lens, ks = np.asarray(lens, float), np.asarray(ks, float)
        a, b = ks[:-1], ks[1:]
        counts = np.maximum(2, np.ceil(lens / step).astype(int))
        seg = np.repeat(np.arange(len(lens)), counts)
        first = np.concatenate([[0], np.cumsum(counts)[:-1]])
        u = (np.arange(len(seg)) - first[seg] + 1) / counts[seg] * lens[seg]
        h_start = self.h0 + np.concatenate([[0.0], np.cumsum(a * lens + 0.5 * (b - a) * lens)[:-1]])
        s_start = np.concatenate([[0.0], np.cumsum(lens)[:-1]])
        s = np.concatenate([[0.0], s_start[seg] + u])
        hh = np.concatenate([[self.h0], h_start[seg] + a[seg] * u + 0.5 * (b - a)[seg] / lens[seg] * u * u])
        ds = np.diff(s)
        x = self.x0 + np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(hh[1:]) + np.cos(hh[:-1])) * ds)])
        y = self.y0 + np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(hh[1:]) + np.sin(hh[:-1])) * ds)])
        return s, np.column_stack([x, y]), hh

    def curvatures(self, lens, ks, step=EVAL_STEP_M):
        """Curvature at every sample of ``sample``."""
        lens, ks = np.asarray(lens, float), np.asarray(ks, float)
        counts = np.maximum(2, np.ceil(lens / step).astype(int))
        seg = np.repeat(np.arange(len(lens)), counts)
        first = np.concatenate([[0], np.cumsum(counts)[:-1]])
        f = (np.arange(len(seg)) - first[seg] + 1) / counts[seg]
        return np.concatenate([[ks[0]], ks[:-1][seg] + (ks[1:] - ks[:-1])[seg] * f])

    def derivatives(self, lens, ks, s, hh, step=EVAL_STEP_M):
        """d(heading), d(s), d(x), d(y) of every sample (as sampled by ``sample``) with respect to the segment
        lengths and the inner knot curvatures ks[1:-1]: four (samples x (2n - 1)) arrays. A sample keeps its
        fraction of its segment; the sample counts per segment are held fixed."""
        lens, ks = np.asarray(lens, float), np.asarray(ks, float)
        n = len(lens)
        a, b = ks[:-1], ks[1:]
        counts = np.maximum(2, np.ceil(lens / step).astype(int))
        seg = np.repeat(np.arange(n), counts)
        first = np.concatenate([[0], np.cumsum(counts)[:-1]])
        f = (np.arange(len(seg)) - first[seg] + 1) / counts[seg]
        rows = np.arange(len(seg))
        below = np.tril(np.ones((n, n)), -1)                      # [i, m] = 1 for m < i
        dth_l = (below * (0.5 * (a + b)))[seg]
        dth_l[rows, seg] += a[seg] * f + 0.5 * (b - a)[seg] * f * f
        ds_l = below[seg]
        ds_l[rows, seg] += f
        dth_k = np.zeros((len(seg), n - 1))
        if n > 1:
            j = np.arange(1, n)                                   # inner knot j: a of segment j, b of segment j - 1
            idx = np.arange(n)[:, None]
            heads = 0.5 * lens[j][None, :] * (j[None, :] < idx) + 0.5 * lens[j - 1][None, :] * (j[None, :] <= idx)
            dth_k = heads[seg]
            own = (seg >= 1)
            dth_k[rows[own], seg[own] - 1] += (f[own] - 0.5 * f[own] ** 2) * lens[seg[own]]
            nxt = (seg + 1 <= n - 1)
            dth_k[rows[nxt], seg[nxt]] += 0.5 * f[nxt] ** 2 * lens[seg[nxt]]
        zero = np.zeros((1, 2 * n - 1))
        dth = np.vstack([zero, np.hstack([dth_l, dth_k])])
        dss = np.vstack([zero, np.hstack([ds_l, np.zeros((len(seg), n - 1))])])
        c, sn = np.cos(hh), np.sin(hh)
        dstep = np.diff(s)[:, None]
        dds = dss[1:] - dss[:-1]
        tx = -0.5 * (sn[:-1, None] * dth[:-1] + sn[1:, None] * dth[1:]) * dstep + 0.5 * (c[:-1] + c[1:])[:, None] * dds
        ty = 0.5 * (c[:-1, None] * dth[:-1] + c[1:, None] * dth[1:]) * dstep + 0.5 * (sn[:-1] + sn[1:])[:, None] * dds
        dx = np.vstack([zero, np.cumsum(tx, axis=0)])
        dy = np.vstack([zero, np.cumsum(ty, axis=0)])
        return dth, dss, dx, dy


def _seed_curvatures(source, knots_s, total):
    q = _resample(source, step=1.0)
    d = np.diff(q, axis=0)
    h = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    k = np.convolve(np.gradient(h, 1.0), np.ones(5) / 5.0, mode="same")
    sq = np.arange(len(k)) + 0.5
    return np.interp(knots_s * (sq[-1] / max(total, 1e-9)), sq, k)


def _end_pose_exact(start, lens, ks):
    """End pose of the chain with exact clothoid propagation (pyclothoids)."""
    from pyclothoids import Clothoid
    x, y, h = start[0], start[1], start[2]
    for L, a, b in zip(lens, ks[:-1], ks[1:]):
        cl = Clothoid.StandardParams(x, y, h, a, (b - a) / L, L)
        x, y, h = float(cl.XEnd), float(cl.YEnd), float(cl.ThetaEnd)
    return x, y, h


def _exact_end(start, end, lens, ks, lo_k, hi_k, min_seg):
    """Smallest change of the last segments (lengths and inner knot curvatures) that lands the chain exactly on
    ``end`` (position 1e-7 m, heading 1e-9 rad); returns (lens, ks) or None. ``lo_k``/``hi_k``: bounds of the
    inner knot curvatures, one value or one per inner knot."""
    n = len(lens)
    lo_inner = np.broadcast_to(np.asarray(lo_k, float), (max(n - 1, 0),))
    hi_inner = np.broadcast_to(np.asarray(hi_k, float), (max(n - 1, 0),))
    for m in (1, 2, 3, 4, 5):
        if m >= n:
            break
        li = list(range(n - 1 - m, n))           # last m+1 lengths
        ki = list(range(n - m, n))               # last m inner knots (ks index)
        z0 = np.concatenate([lens[li], ks[ki]])

        def apply(z):
            L, K = lens.copy(), ks.copy()
            L[li], K[ki] = z[:len(li)], z[len(li):]
            return L, K

        def residual(z):
            L, K = apply(z)
            x, y, h = _end_pose_exact(start, L, K)
            return np.concatenate([1e4 * np.array([x - end[0], y - end[1], _wrap(h - end[2])]), 1e-2 * (z - z0)])

        lower = np.concatenate([np.full(len(li), min_seg), lo_inner[np.asarray(ki) - 1]])
        upper = np.concatenate([np.full(len(li), np.inf), hi_inner[np.asarray(ki) - 1]])
        try:
            sol = least_squares(residual, np.clip(z0, lower + 1e-12, upper - 1e-12), bounds=(lower, upper),
                                xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=200)
        except ValueError:
            continue
        L, K = apply(sol.x)
        x, y, h = _end_pose_exact(start, L, K)
        if math.hypot(x - end[0], y - end[1]) <= 1e-7 and abs(_wrap(h - end[2])) <= 1e-9:
            return L, K
    return None


def _lateral(s, total, lateral):
    """Lane-centre offset c(s) from the chain, dc/ds, and dc/d(chain length) at fixed s: the cubic with value and
    slope (c0, m0) at 0 and (c1, m1) at ``total`` (the connector's edge model, i.e. the mouth tilt cubic)."""
    if lateral is None:
        zero = np.zeros_like(s)
        return zero, zero, zero
    c0, m0, c1, m1 = lateral
    u = s / total
    h00, h10, h01, h11 = 2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u, -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2
    d00, d10, d01, d11 = 6 * u ** 2 - 6 * u, 3 * u ** 2 - 4 * u + 1, -6 * u ** 2 + 6 * u, 3 * u ** 2 - 2 * u
    c = c0 * h00 + m0 * total * h10 + c1 * h01 + m1 * total * h11
    c_s = (c0 * d00 + m0 * total * d10 + c1 * d01 + m1 * total * d11) / total
    return c, c_s, m0 * h10 + m1 * h11 - u * c_s


def _lane_points(s, xy, hh, lateral):
    """Lane-centre samples: the chain samples moved by the lateral offset along the chain normal."""
    c = _lateral(s, s[-1], lateral)[0]
    return xy + c[:, None] * np.column_stack([-np.sin(hh), np.cos(hh)])


def _resample_s(s, pts, count):
    """``count`` points at even chain parameter s (linear between samples)."""
    sigma = np.linspace(0.0, s[-1], count)
    return np.column_stack([np.interp(sigma, s, pts[:, 0]), np.interp(sigma, s, pts[:, 1])])


def _solve(src, src_tree, start, end, lens0, inner0, lo_k, hi_k, min_seg, end_rates, max_nfev, lateral=None):
    """Least squares of segment lengths and inner knot curvatures (see fit); returns (lens, ks)."""
    n = len(lens0)
    x0, y0, h0, k0 = start
    x1, y1, h1, k1 = end
    chain = _Chain(x0, y0, h0)
    total = float(np.sum(lens0))

    def unpack(z):
        return z[:n], np.concatenate([[k0], z[n:], [k1]])

    def residual(z):
        return _residual_and_jacobian(chain, src, src_tree, end, end_rates, *unpack(z), lateral)[0]

    def jacobian(z):
        return _residual_and_jacobian(chain, src, src_tree, end, end_rates, *unpack(z), lateral,
                                      with_jacobian=True)[1]

    lower = np.concatenate([np.full(n, min_seg), np.full(n - 1, lo_k)])
    upper = np.concatenate([np.full(n, 3.0 * total), np.full(n - 1, hi_k)])
    z0 = np.clip(np.concatenate([lens0, inner0]), lower + 1e-9, upper - 1e-9)
    sol = least_squares(residual, z0, jac=jacobian, bounds=(lower, upper), max_nfev=max_nfev, xtol=1e-8, ftol=1e-8)
    return unpack(sol.x)


def _residual_and_jacobian(chain, src, src_tree, end, end_rates, lens, ks, lateral=None, with_jacobian=False):
    """Residual of _solve: end pose, curvature-rate changes, source -> lane centre and lane centre -> source
    distances (lane centre = chain + ``lateral`` offset); with ``with_jacobian`` also its derivative by the lengths
    and inner knot curvatures (the nearest samples held, the lane centre -> source points at even chain parameter)."""
    x1, y1, h1 = end[0], end[1], end[2]
    s, xy, hh = chain.sample(lens, ks)
    total = s[-1]
    c, c_s, c_L = _lateral(s, total, lateral)
    nx, ny = -np.sin(hh), np.cos(hh)
    lane = xy + c[:, None] * np.column_stack([nx, ny])
    d_src, i_src = cKDTree(lane).query(src)
    pts = _resample_s(s, lane, len(src))
    d_tgt, i_tgt = src_tree.query(pts)
    e = np.array([xy[-1, 0] - x1, xy[-1, 1] - y1, _wrap(hh[-1] - h1)])
    rates = np.concatenate([[end_rates[0]], np.diff(ks) / lens, [end_rates[1]]])
    res = np.concatenate([END_WEIGHT * e * np.array([1.0, 1.0, 5.0]), RATE_JUMP_WEIGHT * np.diff(rates),
                          d_src, 0.5 * d_tgt])
    if not with_jacobian:
        return res, None
    n = len(lens)
    dth, dss, dx, dy = chain.derivatives(lens, ks, s, hh)
    rows = [END_WEIGHT * dx[-1], END_WEIGHT * dy[-1], 5.0 * END_WEIGHT * dth[-1]]
    d_rates = np.zeros((n + 2, 2 * n - 1))
    i = np.arange(n)
    d_rates[i + 1, i] = -(ks[1:] - ks[:-1]) / lens ** 2
    inner = i[i >= 1]
    d_rates[inner + 1, n + inner - 1] = -1.0 / lens[inner]
    head = i[i + 1 <= n - 1]
    d_rates[head + 1, n + head] = 1.0 / lens[head]
    jac_rates = RATE_JUMP_WEIGHT * np.diff(d_rates, axis=0)
    # lane centre q = p + c n: dq = dp + n dc + c dn, dc = c_s ds + c_L dL, dn = -(cos h, sin h) dh
    d_total = dss[-1]
    dc = c_s[:, None] * dss + c_L[:, None] * d_total[None, :]
    cos_h, sin_h = np.cos(hh), np.sin(hh)
    dqx = dx + nx[:, None] * dc - (c * cos_h)[:, None] * dth
    dqy = dy + ny[:, None] * dc - (c * sin_h)[:, None] * dth
    with np.errstate(invalid="ignore", divide="ignore"):
        ux = np.where(d_src > 1e-12, (src[:, 0] - lane[i_src, 0]) / d_src, 0.0)
        uy = np.where(d_src > 1e-12, (src[:, 1] - lane[i_src, 1]) / d_src, 0.0)
    jac_src = -(ux[:, None] * dqx[i_src] + uy[:, None] * dqy[i_src])
    # lane centre -> source: points at fixed fractions of the chain parameter
    kk = chain.curvatures(lens, ks)
    qsx = (1 - c * kk) * cos_h + c_s * nx          # dq/ds
    qsy = (1 - c * kk) * sin_h + c_s * ny
    fx = dqx - qsx[:, None] * dss                  # d(lane point at a fixed s)
    fy = dqy - qsy[:, None] * dss
    sigma = np.linspace(0.0, total, len(src))
    t = np.clip(np.searchsorted(s, sigma, side="right") - 1, 0, len(s) - 2)
    w = (sigma - s[t]) / (s[t + 1] - s[t])
    frac = sigma / max(total, 1e-12)
    gx = ((1 - w) * qsx[t] + w * qsx[t + 1]) * frac
    gy = ((1 - w) * qsy[t] + w * qsy[t + 1]) * frac
    dpx = (1 - w)[:, None] * fx[t] + w[:, None] * fx[t + 1] + gx[:, None] * d_total[None, :]
    dpy = (1 - w)[:, None] * fy[t] + w[:, None] * fy[t + 1] + gy[:, None] * d_total[None, :]
    with np.errstate(invalid="ignore", divide="ignore"):
        vx = np.where(d_tgt > 1e-12, (pts[:, 0] - src[i_tgt, 0]) / d_tgt, 0.0)
        vy = np.where(d_tgt > 1e-12, (pts[:, 1] - src[i_tgt, 1]) / d_tgt, 0.0)
    jac_tgt = 0.5 * (vx[:, None] * dpx + vy[:, None] * dpy)
    return res, np.vstack([np.array(rows), jac_rates, jac_src, jac_tgt])


def _merge_knots(lens, ks, tol):
    """Drop inner knots where the curvature rate changes by less than ``tol`` (the two segments become one with
    the same end curvatures); greedy, smallest change first. Returns (lens, ks)."""
    lens, ks = list(lens), list(ks)
    while len(lens) > 2:
        rates = [(b - a) / L for L, a, b in zip(lens, ks[:-1], ks[1:])]
        jumps = [abs(rates[i] - rates[i - 1]) for i in range(1, len(rates))]
        i = int(np.argmin(jumps)) + 1
        if jumps[i - 1] >= tol:
            break
        lens[i - 1:i + 1] = [lens[i - 1] + lens[i]]
        del ks[i]
    return np.array(lens), np.array(ks)


def _measure(src, src_tree, start, lens, ks, lateral=None):
    s, xy, hh = _Chain(*start[:3]).sample(lens, ks)
    lane = _lane_points(s, xy, hh, lateral)
    d_src = cKDTree(lane).query(src)[0]
    d_tgt = src_tree.query(_resample_s(s, lane, len(src)))[0]
    return float(max(np.percentile(d_src, 95), np.percentile(d_tgt, 95))), float(max(d_src.max(), d_tgt.max()))


def _knot_bounds(lens, turn_sign, kappa_max, end_zone):
    """Per inner knot (lower, upper) curvature bounds of a turn-side fit: TURN_SIGN_TOL against the turn, relaxed
    to ``end_zone`` = (metres, curvature) for knots within that distance of either end."""
    st = np.cumsum(lens)[:-1]
    total = float(np.sum(lens))
    lo = np.full(len(st), -TURN_SIGN_TOL if turn_sign > 0 else -kappa_max)
    hi = np.full(len(st), TURN_SIGN_TOL if turn_sign < 0 else kappa_max)
    zone, rev = end_zone
    near = (st <= zone) | (st >= total - zone)
    if turn_sign > 0:
        lo[near] = -rev
    else:
        hi[near] = rev
    return lo, hi


def fit(source, start, end, seg_m, turn_sign=0, kappa_max=KAPPA_MAX, min_seg=MIN_SEGMENT_M, end_rates=(0.0, 0.0),
        lateral=None, end_zone=None):
    """Lane-centre chain from ``start`` to ``end`` ((x, y, heading, curvature)) closest to ``source``.

    Least squares on the two-way closest-point distances with a soft end pose and a penalty on every change of
    the curvature rate (RATE_JUMP_WEIGHT): at the inner knots, and at both mouths against ``end_rates`` (the
    linked roads' reference rates). A lane edge at offset t with slope t' jumps in curvature by about
    t * t' * (rate change) / (1 - t * kappa)^3 there (the mouth edge residual is the same at the contacts), and
    a fit free to change its rate every 3 m rings around digitized kinks (shp-node17 road 103: knot
    curvatures -0.05, +0.18, -0.10 within 9 m for a 21 deg kink). Knots whose rate change stays below each of
    MERGE_RATE_JUMPS are then merged and the chain is solved again; every version is landed exactly on its last
    segments, and the one with the smallest source distance P95 plus PRIMITIVE_COST_M per primitive is kept.
    ``lateral`` (c0, m0, c1, m1): the lane centre is the chain plus the cubic offset with these values and slopes at
    its two ends (the connector's edge model turns the road cross-section heading into the lane-centre heading at
    the mouths); the distances are measured on that lane centre. Without it the written lane centre bowed off the
    fitted chain by up to 0.68 m on 40-65 m connectors with mouth tilts of 1-5.5 deg (shp-node13 roads 108, 128).
    Returns (lengths, knot curvatures, info) or None; ``turn_sign`` +1/-1 bounds the interior knot curvatures
    to that side (TURN_SIGN_TOL of the other); with ``end_zone`` (metres, curvature) knots that close to either end
    may curve that much against the turn (_knot_bounds), where the source via line meets the linked lane at a few
    degrees and the strict bound pushed the whole turn off it (shp-node3 road 101: the via holds 4 deg past the
    departure heading for 12 m, 0.8 m off)."""
    src = _resample(source, step=SAMPLE_M)
    total = float(np.sum(np.linalg.norm(np.diff(src, axis=0), axis=1)))
    n = max(3, int(round(total / seg_m)))
    if total < n * min_seg:
        n = max(1, int(total // min_seg))
    if n < 2:
        return None
    lens0 = np.full(n, total / n)
    ks0 = _seed_curvatures(src, np.concatenate([[0.0], np.cumsum(lens0)]), total)
    zoned = bool(end_zone) and bool(turn_sign)
    if zoned:
        lo_k, hi_k = _knot_bounds(lens0, turn_sign, kappa_max, end_zone)
        ks0 = ks0.copy()
        ks0[1:-1] = np.clip(ks0[1:-1], lo_k, hi_k)
    else:
        lo_k = -TURN_SIGN_TOL if turn_sign > 0 else -kappa_max
        hi_k = TURN_SIGN_TOL if turn_sign < 0 else kappa_max
        ks0 = np.clip(ks0, lo_k, hi_k)
    src_tree = cKDTree(src)
    lens, ks = _solve(src, src_tree, start, end, lens0, ks0[1:-1], lo_k, hi_k, min_seg, end_rates, MAX_NFEV, lateral)
    versions, tried = [], {len(lens)}
    full = _exact_end(start, end, lens, ks, lo_k, hi_k, min_seg)
    if full is not None:
        versions.append((full, 0))
    for tol in MERGE_RATE_JUMPS:
        m_lens, m_ks = _merge_knots(lens, ks, tol)
        if len(m_lens) in tried:          # a smaller tolerance merges a subset: same count, same chain
            continue
        tried.add(len(m_lens))
        if zoned:
            m_lo, m_hi = _knot_bounds(m_lens, turn_sign, kappa_max, end_zone)
            m_lens, m_ks = _solve(src, src_tree, start, end, m_lens, np.clip(m_ks[1:-1], m_lo, m_hi), m_lo, m_hi,
                                  min_seg, end_rates, MAX_NFEV // 2, lateral)
        else:
            m_lo, m_hi = lo_k, hi_k
            m_lens, m_ks = _solve(src, src_tree, start, end, m_lens, m_ks[1:-1], lo_k, hi_k, min_seg, end_rates,
                                  MAX_NFEV // 2, lateral)
        m_landed = _exact_end(start, end, m_lens, m_ks, m_lo, m_hi, min_seg)
        if m_landed is not None:
            versions.append((m_landed, len(lens) - len(m_lens)))
    if not versions:
        return None
    (lens, ks), merged = min(versions, key=lambda v: _measure(src, src_tree, start, *v[0], lateral)[0]
                             + PRIMITIVE_COST_M * len(v[0][0]))
    p95, worst = _measure(src, src_tree, start, lens, ks, lateral)
    rates = np.concatenate([[end_rates[0]], np.diff(ks) / lens, [end_rates[1]]])
    jumps = np.abs(np.diff(rates))
    return lens, ks, {"segments": len(lens), "merged": merged, "seg_m": seg_m, "min_segment_m": float(lens.min()),
                      "kappa_max": float(np.abs(ks).max()), "end_rate_mismatch": float(max(jumps[0], jumps[-1])),
                      "rate_jump_max": float(jumps[1:-1].max()) if len(jumps) > 2 else 0.0,
                      "p95_m": p95, "max_m": worst, "turn_sign": turn_sign,
                      **({"end_zone": list(end_zone)} if zoned else {})}


def primitives(start, lens, ks):
    """Writer primitives (kind, x, y, hdg, L, k0, k1) with exactly propagated start poses."""
    from pyclothoids import Clothoid
    x, y, h = start[0], start[1], start[2]
    out = []
    for L, a, b in zip(lens, ks[:-1], ks[1:]):
        kind = "spiral" if abs(b - a) > 1e-11 else ("line" if abs(a) <= 1e-11 else "arc")
        out.append((kind, float(x), float(y), float(h), float(L), float(a), float(b)))
        cl = Clothoid.StandardParams(x, y, h, a, (b - a) / L, L)
        x, y, h = float(cl.XEnd), float(cl.YEnd), float(cl.ThetaEnd)
    return out


def candidates(source, start, end, turn_sign=0, end_rates=(0.0, 0.0), lateral=None, end_zone=None):
    """[(prims, info)], best first by P95: SEGMENT_STEPS_M from coarse to fine until one reaches GOOD_P95_M; in a
    turn, a step whose free fit curves against the turn also gets a turn-side fit, and with ``end_zone`` a turn-side
    fit allowed to steer back that much near the ends."""
    out = []
    for seg_m in SEGMENT_STEPS_M:
        fits = []
        try:
            free = fit(source, start, end, seg_m, end_rates=end_rates, lateral=lateral)
        except (ValueError, np.linalg.LinAlgError):
            free = None
        if free is not None:
            fits.append(free)
        if turn_sign and (free is None or _counter(free[1], turn_sign) > TURN_SIGN_TOL):
            try:
                bounded = fit(source, start, end, seg_m, turn_sign=turn_sign, end_rates=end_rates, lateral=lateral)
            except (ValueError, np.linalg.LinAlgError):
                bounded = None
            if bounded is not None:
                fits.append(bounded)
            if end_zone:
                try:
                    zoned = fit(source, start, end, seg_m, turn_sign=turn_sign, end_rates=end_rates, lateral=lateral,
                                end_zone=end_zone)
                except (ValueError, np.linalg.LinAlgError):
                    zoned = None
                if zoned is not None:
                    fits.append(zoned)
        out += [(primitives(start, lens, ks), info) for lens, ks, info in fits]
        if any(info["p95_m"] <= GOOD_P95_M for _, _, info in fits):
            break
    return sorted(out, key=lambda c: c[1]["p95_m"])


def _counter(ks, turn_sign):
    """Largest interior knot curvature against the turn."""
    inner = np.asarray(ks[1:-1]) * turn_sign
    return float(max(0.0, -inner.min())) if len(inner) else 0.0
