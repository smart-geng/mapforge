"""Event-aligned lane-boundary refit for ordinary SHP roads (Phase 2, step 1).

The default pipeline fits lane widths with long smooth curves; at SHP lane-shift
tapers (polylines with ~10 deg corners) its edges stray up to ~2 m from the source
boundaries. Here every physical boundary is refitted directly to its source
polyline, relative to the unchanged reference line:

* boundary chains: the centre boundary (laneOffset) runs over the whole road; each
  outer lane boundary is chained across laneSections through the lane links and
  starts/ends on its inner neighbour where a lane is born/dies (zero width, zero
  width slope);
* fitter: RDP vertices of the source polyline (runs >= SEG_MIN), straight runs followed
  exactly and a corner blend at each vertex sized from the curvature target KAPPA_CAP:
  "g2" = two cubics with triangular t'' (curvature continuous, vertex deviation
  |dm|*h/6), "c1" = one parabola-like cubic (|dm|*h/4); laneSection boundaries are
  always knots; births/deaths ramp over at least RAMP_MIN_M and are anchored at the
  ramp edge when the source opens abruptly;
* junction mouths: no vertex within MOUTH_ZONE_M, end slope clipped to
  +/-MOUTH_SLOPE_CAP so connectors can meet the edge smoothly; a steeper source flare
  (> MOUTH_FLARE_MAX) is kept and reported (``strong_mouth_flares``); with ``mouth_anchor`` (2026-10-09) a
  source corner dropped from a mouth zone leaves a vertex on the data at the zone edge, so the run before it still
  follows the source (0621 road 10: a corner 7.8 m before the mouth had pulled 16 m of the median edge 0.23 m off);
  where the source does not run straight from there into the mouth (a flare), the mouth keeps the slope it had
  without the anchor, and a flare steeper than MOUTH_FLARE_MAX (a curb return) gets no anchor (2026-10-10);
* curb returns: a strong flare on the outermost boundary at a mouth is the start of the
  corner curb return, not lane width (no connector can continue it without a width bulge).
  The driving lane keeps the pre-flare edge into the mouth; a non-driving shoulder lane,
  born with zero width at the flare start, carries the flare so the physical outer edge
  still follows the source (CURB_SHOULDER; provenance code physical-edge-fill);
* small crossings of independently fitted boundaries (<= REPAIR_MAX_M) are lifted with
  a C1 bump taken from the next lane out; larger ones leave the road unchanged;
* SHP boundary step (2026-10-04; variant parameters, SHP only): lane births/deaths moved half a corner ahead of
  their source events (mapforge.ops.lane_birth_advance) are fitted as width on their inner boundary near the
  event, with the shortest corner whose boundary curvature stays within the target, and hand over to the
  boundary's own fit after it (_advanced_fit); free road ends where a source lane starts narrow get zero width
  up to it (_zero_starts); short runs stay where their corners fit the curvature target (short_kappa); vertex
  values are set by least squares on the finished curve, mouth ends and mouth-run starts staying on the data
  (lsq_refine); a gap between the lanes of two parallel source links becomes a lane of its own before the refit
  (mapforge.ops.lane_gap, link_gaps, 2026-10-05);
* boundaries without any source observation keep their current geometry.

Only laneOffset and width records change; reference lines, laneSections, lane ids,
links and provenance stay. Unsupported roads are reported and left unchanged.
"""
from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[2]
CODE = "mapforge.lane_refit/v1"
RDP_TOL = 0.05
GRID = 0.5
RAMP_MIN_M = 12.0
RAMP_KAPPA_CAP = None     # curvature target of birth/death end blends [1/m]; None: the corner target KAPPA_CAP
UNOBSERVED_END_M = 1.0    # zero_starts: a free-end stretch without source longer than this
ZERO_START_M = 0.3        # ... before a source lane that starts this narrow (a taper start) is zero width
TAIL_WIDTH_M = 0.001      # ... held at this width instead where the lane ends there in its travel direction
FREE_END_KAPPA_MAX = 0.1  # free_end_runs: a kept free-end run may round its corner at most this sharply [1/m]
ADVANCED_H_STEP = 0.25    # advanced corners: half lengths tried from H_MIN up in this step [m]
ADVANCED_H_MAX = 8.0      # ... up to the advance (or this, for zero-width starts at free ends) [m]
CUT_GAP_M = 1.0           # local relative fits hand over to the boundary's own fit in a straight width stretch this long
LSQ_ITER = 2              # lsq_refine: least-squares passes over the vertex values (corner sizes renewed in between)
LSQ_RIDGE = 1e-6          # ... with this ridge on the changes, for vertices the data barely constrains
# Junction mouths: boundary slope relative to the reference is clipped to +/-MOUTH_SLOPE_CAP
# (lanes nearly parallel at the mouth); strong source flares are not forced flat.
MOUTH_SLOPE_CAP = 0.05
END_BLEND_MAX_M = 12.0
MOUTH_ZONE_M = 8.0        # no corner vertex this close to a junction end
MOUTH_FLARE_MAX = 0.12    # steeper source flare at a mouth is kept (reported), not forced flat
NEGATIVE_WIDTH_TOL = 0.02
# Curb-return shoulder for strong mouth flares on the outermost boundary (see module doc).
# Off by default (variant parameter "curb_shoulder"), so earlier registered variants keep their meaning.
CURB_SHOULDER = False
FLARE_SEARCH_M = 25.0     # a flare starts at most this far before the last source observation
FLARE_MIN_RUN_M = 6.0     # driving boundary keeps at least this much source before the flare
# Beyond the last source point the curb edge levels off at this curvature (radius 4 m) and holds.
CURB_HOLD_KAPPA = 0.25
SHOULDER_CODE = "physical-edge-fill"


# --- piecewise cubics: (s0, s1, a, b, c, d) with local u = s - s0 ---------------------

def _eval(piece, s, order=0):
    s0, _, a, b, c, d = piece
    u = s - s0
    return (a + u * (b + u * (c + u * d)), b + u * (2 * c + 3 * d * u), 2 * c + 6 * d * u)[order]


def _rebase(piece, new_s0):
    """Same polynomial expressed with origin ``new_s0``."""
    s0, s1, a, b, c, d = piece
    u = new_s0 - s0
    return (new_s0, s1, a + u * (b + u * (c + u * d)), b + u * (2 * c + 3 * d * u), c + 3 * d * u, d)


def _at(pieces, s, order=0):
    for p in pieces:
        if p[0] - 1e-9 <= s <= p[1] + 1e-9:
            return _eval(p, s, order)
    p = pieces[0] if s < pieces[0][0] else pieces[-1]
    return _eval(p, s, order)


def _hermite_piece(s0, s1, y0, m0, y1, m1):
    length = s1 - s0
    delta = (y1 - y0) / length
    return (s0, s1, y0, m0, (3 * delta - 2 * m0 - m1) / length, (m0 + m1 - 2 * delta) / length ** 2)


def _split(pieces, cuts):
    out = []
    for p in pieces:
        inner = sorted(c for c in cuts if p[0] + 1e-6 < c < p[1] - 1e-6)
        start = p
        for c in inner:
            out.append((start[0], c, *start[2:]))
            start = _rebase((start[0], p[1], *start[2:]), c)
        out.append(start)
    return out


# --- C1 smoothed polyline ----------------------------------------------------------

def _rdp(x, y, tol):
    keep = np.zeros(len(x), bool)
    keep[[0, -1]] = True
    stack = [(0, len(x) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        xs, ys = x[i:j + 1], y[i:j + 1]
        dx, dy = x[j] - x[i], y[j] - y[i]
        norm = math.hypot(dx, dy) or 1e-12
        dist = np.abs(dy * (xs - x[i]) - dx * (ys - y[i])) / norm
        k = int(np.argmax(dist))
        if dist[k] > tol:
            keep[i + k] = True
            stack += [(i, i + k), (i + k, j)]
    return np.where(keep)[0]


def _grid_profile(obs_s, obs_t, a, b):
    """Median-binned observation profile on a regular grid over [a, b]."""
    n = max(2, int(math.ceil((b - a) / GRID)) + 1)
    grid = np.linspace(a, b, n)
    idx = np.clip(np.round((obs_s - a) / (b - a) * (n - 1)).astype(int), 0, n - 1)
    vals = np.full(n, np.nan)
    for i in np.unique(idx):
        vals[i] = float(np.median(obs_t[idx == i]))
    ok = ~np.isnan(vals)
    if ok.sum() < 2:
        raise ValueError("too few observations for a boundary")
    vals = np.interp(grid, grid[ok], vals[ok])
    return grid, vals


MODE = "g2"           # corner blend: "g2" (triangular t'', curvature continuous) or "c1" (parabola)
KAPPA_CAP = 0.04      # lane-boundary curvature target at corners [1/m]
H_MIN = 3.0           # shortest corner half-length [m]
SEG_MIN = 6.0         # shortest straight run between corner vertices [m]
FLARES: list = []     # strong mouth flares met during the current road (reset per road)


def _prune(vs, vt, seg_min, protected=(), keep_first=False, keep_last=False, protected_runs=False,
           short_kappa=None):
    """Drop the least important interior vertex until all runs are >= seg_min.

    A short run between protected vertices and chain ends stays (an advanced birth's zero-width run): it used
    to drop some other vertex instead, which took the taper start of shp-node3 road 10 lane 5 33 m away.
    ``protected_runs``: a short run between a protected vertex and an ordinary one also stays when that vertex
    is a real source corner and both corners still fit (_keep_protected_run).
    ``short_kappa``: an interior short run also stays when both its corners fit at that curvature
    (_short_run_fits)."""
    vs, vt = list(vs), list(vt)

    def importance(i):
        (x0, y0), (x1, y1), (x2, y2) = (vs[i - 1], vt[i - 1]), (vs[i], vt[i]), (vs[i + 1], vt[i + 1])
        return abs((y2 - y0) * (x1 - x0) - (x2 - x0) * (y1 - y0)) / math.hypot(x2 - x0, y2 - y0)

    while len(vs) > 2:
        lengths = np.diff(vs)
        drop = None
        for k in np.argsort(lengths, kind="stable"):
            if lengths[k] >= seg_min:
                break
            if (keep_first and k == 0) or (keep_last and k == len(vs) - 2):
                continue
            if protected_runs and _keep_protected_run(vs, vt, k, protected):
                continue
            if short_kappa and _short_run_fits(vs, vt, k, short_kappa):
                continue
            candidates = [i for i in (k, k + 1) if 0 < i < len(vs) - 1 and vs[i] not in protected]
            if candidates:
                drop = min(candidates, key=importance)
                break
        if drop is None:
            break
        del vs[drop], vt[drop]
    return np.asarray(vs), np.asarray(vt)


def _short_run_fits(vs, vt, k, kappa):
    """Interior run k is shorter than SEG_MIN, but the corners at both its ends fit in half of each adjacent run
    (a quarter of an end run, which also carries an end blend) at curvature ``kappa``: SEG_MIN keeps big corners
    from being squeezed, a small corner needs little room. Pruning such a vertex slid shp-node4 road 12 lane 3
    0.15 m off its source for 30 m: a 0.15 m source bump came back to the line 5 m after its peak, and the
    vertex where it did was dropped, so the edge ran from the peak in one 142 m chord."""
    n = len(vs)
    if not 0 < k < n - 2:
        return False
    slopes = np.diff(vt) / np.diff(vs)
    lengths = np.diff(vs)
    for i in (k, k + 1):
        left = lengths[i - 1] * (0.25 if i == 1 else 0.5)
        right = lengths[i] * (0.25 if i == n - 2 else 0.5)
        if abs(slopes[i] - slopes[i - 1]) > kappa * min(left, right):
            return False
    return True


def _eval_many(pieces, s):
    """Values of piecewise cubics (sorted, contiguous) at the stations s."""
    starts = np.array([p[0] for p in pieces])
    coef = np.array([p[2:6] for p in pieces])
    idx = np.clip(np.searchsorted(starts, s, side="right") - 1, 0, len(pieces) - 1)
    u = s - starts[idx]
    c0, c1, c2, c3 = coef[idx].T
    return c0 + u * (c1 + u * (c2 + u * c3))


def _keep_protected_run(vs, vt, k, protected):
    """Run k (vs[k]..vs[k+1]) joins a protected vertex and an ordinary interior vertex that lies more than
    RDP_TOL off the chord of its neighbours, and both corners fit in half the run at FREE_END_KAPPA_MAX or less.
    Pruning that vertex slid the source corner into the protected corner's run: shp-node16 road 10 lane 4
    (a taper steepening 4.8 m before its advanced death) followed one long chord, 0.38 m inside."""
    prot = [i for i in (k, k + 1) if vs[i] in protected]
    if len(prot) != 1:
        return False
    o = k + 1 if prot[0] == k else k
    if not 0 < o < len(vs) - 1:
        return False
    (x0, y0), (x1, y1), (x2, y2) = (vs[o - 1], vt[o - 1]), (vs[o], vt[o]), (vs[o + 1], vt[o + 1])
    if abs((y2 - y0) * (x1 - x0) - (x2 - x0) * (y1 - y0)) / math.hypot(x2 - x0, y2 - y0) <= RDP_TOL:
        return False
    run = vs[k + 1] - vs[k]
    slopes = np.diff(vt) / np.diff(vs)

    def kappa(i, cap):
        dm = abs(slopes[i] - slopes[i - 1])
        return dm / max(min(max(dm / KAPPA_CAP, H_MIN), cap), 1e-9)
    other = vs[o + 1] - vs[o] if o == k + 1 else vs[o] - vs[o - 1]
    return (kappa(o, min(0.5 * run, 0.5 * other)) <= FREE_END_KAPPA_MAX
            and kappa(prot[0], 0.5 * run) <= FREE_END_KAPPA_MAX)


def _g2_corner(s_i, t_i, m1, m2, h):
    """Two cubic pieces with triangular t'' (0 -> dm/h -> 0): curvature-continuous corner.

    Lands exactly on both lines at s_i -/+ h; vertex deviation dm*h/6, peak t'' = dm/h.
    """
    dm = m2 - m1
    A = dm / h
    left = (s_i - h, s_i, t_i - m1 * h, m1, 0.0, A / (6 * h))
    right = (s_i, s_i + h, t_i + dm * h / 6, m1 + dm / 2, A / 2, -A / (6 * h))
    return [left, right]


# A slope-changing G2 transition pivoting on a line peaks at 16/3 |dm| / length (C1 Hermite: 4 |dm| / length).
G2_BLEND_PEAK = 16.0 / 3.0


def _g2_transition(s0, s1, start, end):
    """C2 piecewise cubic on [s0, s1] matching (value, slope, second derivative) at both ends.

    Three cubics (knots at 1/4 and 3/4); t'' is piecewise linear a0 -> A -> B -> a1, with A and B
    solved from the end value and slope. Replaces the C1 Hermite end blends, births/deaths and
    repair bumps where the lane-centre curvature must not jump (T2 lane_join_curvature_jump).
    """
    v0, m0, a0 = start
    v1, m1, a1 = end
    q = (s1 - s0) / 4.0
    knots = [s0, s0 + q, s0 + 3 * q, s1]

    def build(A, B):
        acc = [a0, A, B, a1]
        pieces, v, m = [], v0, m0
        for i in range(3):
            h = knots[i + 1] - knots[i]
            c, d = acc[i] / 2.0, (acc[i + 1] - acc[i]) / (6.0 * h)
            pieces.append((knots[i], knots[i + 1], v, m, c, d))
            v, m = v + m * h + c * h * h + d * h ** 3, m + 2 * c * h + 3 * d * h * h
        return pieces, v, m

    _, vz, mz = build(0.0, 0.0)
    _, va, ma = build(1.0, 0.0)
    _, vb, mb = build(0.0, 1.0)
    A, B = np.linalg.solve(np.array([[va - vz, vb - vz], [ma - mz, mb - mz]]), np.array([v1 - vz, m1 - mz]))
    return build(float(A), float(B))[0]


def fit_boundary(obs_s, obs_t, a, b, start=None, end=None, ramp_start=False, ramp_end=False,
                 mode=None, kappa_cap=None, rdp_tol=None, seg_min=None, c2_ends=False, protect=(), corner_h=None,
                 protect_values=None, free_end_runs=False, info=None, short_kappa=False, lsq_refine=False,
                 mouth_anchor=False):
    """Piecewise cubic over [a, b]: straight runs between RDP vertices with corner blends.

    Corner half-length follows the curvature target (g2: |dm|/kappa, c1: |dm|/(2 kappa)),
    at least H_MIN and at most half of each adjacent run; runs shorter than SEG_MIN are
    removed first, so no corner blend is squeezed into a short run.
    ``start``/``end``: optional (value or None, slope or None[, second derivative]) end conditions.
    ``ramp_start``/``ramp_end``: births/deaths — no vertex within RAMP_MIN_M of that end.
    ``c2_ends``: end blends are G2 transitions (also matching the given second derivative, e.g. of the
    inner boundary at a birth) instead of C1 Hermite pieces, so the whole boundary is C2.
    ``protect``: stations kept as corner vertices through the run pruning, placed exactly there (the nearest grid
    point moves onto the station; ``protect_values`` {station: value} fixes its value, e.g. zero width at a source
    birth, which the 0.5 m grid otherwise took from the first source sample past it); ``corner_h`` {station: half
    length} caps their corners, which otherwise follow the curvature target (next to a chain end without an
    end blend the whole run is available).
    ``free_end_runs``: at a free chain end (no condition, so no end blend) the first (last) run is kept however
    short and the corner at its inner end may use all of it. Pruning it slid a source jog at a road's far end
    into one long slope (shp-node17 road 12: 0.7 m in 5 m became 28 m, 0.61 m off; shp-NODE5 road 10: 0.29 m).
    ``info``: a dict that receives the vertices ("vs", "vt"), corner half lengths ("h") and end blends ("h0", "hn").
    ``short_kappa``: short interior runs whose corners fit at the curvature target stay (_short_run_fits).
    ``lsq_refine``: the vertex values (all but those fixed by end values, junction-mouth ends and protected values) are
    set by least
    squares so the finished curve, corners and end blends included, fits the observation profile (LSQ_ITER
    passes, corner sizes renewed in between). RDP puts vertices on the data, so the chords cut inside every
    smooth bend and the corner blends cut inside once more (shp-node13 road 10 lane 5: a rounded knee 0.3 m
    inside); the curve is linear in the vertex values for fixed corner sizes, so this is a linear problem.
    ``mouth_anchor``: a junction-mouth zone that loses source vertices gets one at its inner edge, on the data, like
    a birth/death ramp, so the run before it no longer stretches over the dropped corner (0621 road 10: a corner
    7.8 m before the mouth pulled 16 m of the median edge 0.23 m off; 2026-10-09). Where the source runs straight
    (within ``rdp_tol``) from there into the mouth, the corner sat at the zone edge and the mouth slope follows the
    run from the anchor (capped as before). Where it does not, the dropped corner is a mouth flare or curb return:
    the mouth keeps the slope the fit without the anchor has, and the end blend inside the zone takes up the
    difference (2026-10-10). Steepening the mouth there bent the connectors meeting it (shp-node17 road 10, flare
    0.5-4.5 m before the mouth: edge contact curvature 0.0009 -> 0.0033 /m, T2 FAIL), so the first version left such
    zones unanchored (shp-node18 boundary P95 inside 0.097 m; anchored this way 0.048). A strong flare is left as it
    was: one the fit without the anchor keeps (> MOUTH_FLARE_MAX), or a run from the anchor into the mouth steeper
    than that, a curb return across the whole zone (generalization 0412 road 30: -0.25 against a held -0.05; the end
    blend bent the outer edge at 0.26 /m). The zone never gets a corner vertex either way.
    """
    mode = mode or MODE
    kappa_cap = kappa_cap or KAPPA_CAP
    rdp_tol = rdp_tol or RDP_TOL
    seg_min = seg_min or SEG_MIN
    grid, vals = _grid_profile(np.asarray(obs_s, float), np.asarray(obs_t, float), a, b)
    if start and start[0] is not None:
        vals[0] = start[0]
    if end and end[0] is not None:
        vals[-1] = end[0]
    pinned = []
    for x in protect:
        if not (a + GRID < x < b - GRID):
            continue
        j = int(np.argmin(np.abs(grid[1:-1] - x))) + 1
        grid[j] = x
        if protect_values and x in protect_values:
            vals[j] = protect_values[x]
        pinned.append(j)
    keep = list(_rdp(grid, vals, rdp_tol))
    ramp = min(RAMP_MIN_M, 0.45 * (b - a))
    zone = min(MOUTH_ZONE_M, 0.45 * (b - a))
    last = len(grid) - 1

    def clear(keep, lo, hi, anchor=None):
        """Drop vertices strictly inside (lo, hi). For births/deaths anchor a vertex at ``anchor`` when
        any were dropped, so the boundary still reaches the source there (a lane that opens abruptly).
        Mouth zones get no anchor unless ``mouth_anchor``: one straight run into the mouth, slope capped at the
        mouth."""
        inside = [i for i in keep if lo < grid[i] < hi]
        keep = [i for i in keep if i not in inside]
        if inside and anchor is not None:
            keep.append(int(np.argmin(np.abs(grid - anchor))))
        return sorted(set(keep))

    if ramp_start:
        keep = clear(keep, a, a + ramp, a + ramp)
    if ramp_end:
        keep = clear(keep, b - ramp, b, b - ramp)
    def straight_into_mouth(edge, k):
        """The source runs straight (within rdp_tol) from the grid point at ``edge`` into the mouth end k."""
        j = int(np.argmin(np.abs(grid - edge)))
        lo, hi = sorted((j, k))
        if hi - lo < 2:
            return True
        x = grid[lo + 1:hi]
        line = vals[j] + (vals[k] - vals[j]) * (x - grid[j]) / (grid[k] - grid[j])
        return bool(np.max(np.abs(vals[lo + 1:hi] - line)) <= rdp_tol)

    held = {}                 # mouth end -> slope it keeps from the fit without the anchor

    def mouth_anchor_at(which, lo, hi, edge, k):
        """The zone-edge anchor of a mouth zone that drops source vertices (None: no anchor)."""
        if not (mouth_anchor and any(lo < grid[i] < hi for i in keep)):
            return None
        if straight_into_mouth(edge, k):
            return edge
        j = int(np.argmin(np.abs(grid - edge)))
        if abs((vals[k] - vals[j]) / (grid[k] - grid[j])) > MOUTH_FLARE_MAX:
            return None       # a strong flare across the zone (a curb return): left as it was
        n0, plain = len(FLARES), {}
        fit_boundary(obs_s, obs_t, a, b, start=start, end=end, ramp_start=ramp_start, ramp_end=ramp_end, mode=mode,
                     kappa_cap=kappa_cap, rdp_tol=rdp_tol, seg_min=seg_min, c2_ends=c2_ends, protect=protect,
                     corner_h=corner_h, protect_values=protect_values, free_end_runs=free_end_runs, info=plain,
                     short_kappa=short_kappa, lsq_refine=lsq_refine)
        del FLARES[n0:]
        slope = plain["m0" if which == "start" else "mn"]
        if slope is None:     # a strong flare the plain fit keeps: left as it was
            return None
        held[which] = slope
        return edge

    if start and start[1] == "mouth":
        keep = clear(keep, a, a + zone, mouth_anchor_at("start", a, a + zone, a + zone, 0))
    if end and end[1] == "mouth":
        keep = clear(keep, b - zone, b, mouth_anchor_at("end", b - zone, b, b - zone, last))
    keep = [i for i in keep if 0 <= i <= last]
    protected = {float(grid[j]) for j in pinned}
    keep = sorted(set(keep) | set(pinned))
    free_start, free_end = free_end_runs and start is None, free_end_runs and end is None

    def end_corner_kappa(vs_, vt_, first):
        """Curvature of the corner next to a kept free-end run (its half length at most that run)."""
        if len(vs_) < 3:
            return 0.0
        sl, ln = np.diff(vt_) / np.diff(vs_), np.diff(vs_)
        dm, room, other = ((abs(sl[1] - sl[0]), ln[0], ln[1]) if first else (abs(sl[-1] - sl[-2]), ln[-1], ln[-2]))
        h_end = min(max(dm / kappa_cap, H_MIN), room, 0.5 * other)
        return dm / max(h_end, 1e-9)

    short = kappa_cap if short_kappa else None
    vs, vt = _prune(grid[keep], vals[keep], min(seg_min, 0.5 * (b - a)), protected=protected,
                    keep_first=free_start, keep_last=free_end, protected_runs=bool(protected), short_kappa=short)
    # a kept free-end run so short that its corner would be sharper than FREE_END_KAPPA_MAX is pruned after all
    if free_start and end_corner_kappa(vs, vt, True) > FREE_END_KAPPA_MAX:
        free_start = False
    if free_end and end_corner_kappa(vs, vt, False) > FREE_END_KAPPA_MAX:
        free_end = False
    if free_end_runs and (start is None or end is None) and not (free_start and free_end):
        vs, vt = _prune(grid[keep], vals[keep], min(seg_min, 0.5 * (b - a)), protected=protected,
                        keep_first=free_start, keep_last=free_end, protected_runs=bool(protected), short_kappa=short)
    def target(cond, run_slope, which):
        if not cond or cond[1] is None:
            return None
        if cond[1] == "mouth" and which in held:
            return held[which]
        if cond[1] == "mouth":
            if abs(run_slope) > MOUTH_FLARE_MAX:
                flares.append(float(run_slope))
                return None  # strong source flare: keep it, report it
            return float(np.clip(run_slope, -MOUTH_SLOPE_CAP, MOUTH_SLOPE_CAP))
        return cond[1]

    flares = []

    peak = G2_BLEND_PEAK if c2_ends else 4.0

    def end_h(ramped, length, dm):
        # end blend peaks at peak*|dm|/h: size it from the curvature target
        if ramped:
            # at most half the run, so the next corner always keeps room for its blend
            return min(max(ramp, peak * abs(dm) / (RAMP_KAPPA_CAP or kappa_cap)), 0.5 * length)
        want = min(max(H_MIN, peak * abs(dm) / kappa_cap), END_BLEND_MAX_M)
        return min(want, 0.45 * length)

    a_start = float(start[2]) if (c2_ends and start and len(start) > 2) else 0.0
    a_end = float(end[2]) if (c2_ends and end and len(end) > 2) else 0.0

    def sizes(vt_):
        """Run slopes, end slopes, end blend and corner half lengths for the vertex values vt_."""
        slopes = np.diff(vt_) / np.diff(vs)
        lengths = np.diff(vs)
        m_start, m_end = target(start, slopes[0], "start"), target(end, slopes[-1], "end")
        h0 = end_h(ramp_start, lengths[0], (m_start or 0.0) - slopes[0]) if m_start is not None else 0.0
        hn = end_h(ramp_end, lengths[-1], (m_end or 0.0) - slopes[-1]) if m_end is not None else 0.0
        if m_start is not None and abs(m_start - slopes[0]) < 1e-12 and abs(a_start) < 1e-12:
            h0 = 0.0
        if m_end is not None and abs(m_end - slopes[-1]) < 1e-12 and abs(a_end) < 1e-12:
            hn = 0.0
        # corner half-lengths from the curvature target, never overlapping neighbours or end blends
        h = np.zeros(len(vs))
        for i in range(1, len(vs) - 1):
            dm = abs(slopes[i] - slopes[i - 1])
            need = dm / kappa_cap if mode == "g2" else dm / (2 * kappa_cap)
            room_left = max(lengths[i - 1] - (h0 if i == 1 else 0.0), 0.0)
            room_right = max(lengths[i] - (hn if i == len(vs) - 2 else 0.0), 0.0)
            if float(vs[i]) in protected or (free_start and i == 1) or (free_end and i == len(vs) - 2):
                forced = None
                for x, hx in (corner_h or {}).items():
                    if abs(x - float(vs[i])) <= GRID:
                        forced = hx
                left_cap = room_left if (i == 1 and h0 == 0.0) else 0.5 * room_left
                right_cap = room_right if (i == len(vs) - 2 and hn == 0.0) else 0.5 * room_right
                want = max(need, H_MIN)
                h[i] = min(want if forced is None else min(want, forced), left_cap, right_cap)
                continue
            h[i] = min(max(need, H_MIN), 0.5 * room_left, 0.5 * room_right)
        return slopes, m_start, m_end, h0, hn, h

    def build(vt, slopes, m_start, m_end, h0, hn, h):
        pieces = []

        def line(s0, s1, k):
            if s1 - s0 > 1e-6:
                y0 = vt[k] + slopes[k] * (s0 - vs[k])
                pieces.append((s0, s1, y0, slopes[k], 0.0, 0.0))

        cursor = vs[0]
        if h0 > 0:
            s1 = vs[0] + h0
            if c2_ends:
                pieces += _g2_transition(vs[0], s1, (vt[0], m_start, a_start),
                                         (vt[0] + slopes[0] * h0, slopes[0], 0.0))
            else:
                pieces.append(_hermite_piece(vs[0], s1, vt[0], m_start, vt[0] + slopes[0] * h0, slopes[0]))
            cursor = s1
        for i in range(1, len(vs) - 1):
            if h[i] <= 1e-6:
                raise ValueError("corner without room for a blend")
            left, right = vs[i] - h[i], vs[i] + h[i]
            line(cursor, left, i - 1)
            if mode == "g2":
                pieces += _g2_corner(vs[i], vt[i], slopes[i - 1], slopes[i], h[i])
            else:
                pieces.append(_hermite_piece(left, right, vt[i] - slopes[i - 1] * h[i], slopes[i - 1],
                                             vt[i] + slopes[i] * h[i], slopes[i]))
            cursor = right
        last = len(vs) - 2
        if hn > 0:
            left = vs[-1] - hn
            line(cursor, left, last)
            if c2_ends:
                pieces += _g2_transition(left, vs[-1], (vt[-1] - slopes[last] * hn, slopes[last], 0.0),
                                         (vt[-1], m_end, a_end))
            else:
                pieces.append(_hermite_piece(left, vs[-1], vt[-1] - slopes[last] * hn, slopes[last], vt[-1], m_end))
        else:
            line(cursor, vs[-1], last)
        return pieces

    if lsq_refine and len(vs) > 2:
        # given end values stay, and so do junction-mouth ends at their source value: connectors meet the lane
        # centres there (left free, the mouth ends of curved outermost boundaries moved 0.2-0.5 m and the
        # connector fits followed them away from their source, shp-node17 connectors 103-106: 0.04 -> 0.3 m)
        fixed = {0} if (start and (start[0] is not None or start[1] == "mouth")) else set()
        if end and (end[0] is not None or end[1] == "mouth"):
            fixed.add(len(vs) - 1)
        # ... and the vertex that starts a mouth run stays on the data as well: the run into a mouth is straight
        # by design, a source flare inside it cannot be followed, and the least-squares compromise moved that
        # vertex off the data instead (shp-node17 road 10 boundary 3: 0.23 m, 25 m before the mouth)
        if start and start[1] == "mouth":
            fixed.add(1)
        if end and end[1] == "mouth":
            fixed.add(len(vs) - 2)
        fixed |= {j for j, x in enumerate(vs) if protect_values and float(x) in protected
                  and float(x) in {float(k) for k in protect_values}}
        free = [j for j in range(len(vs)) if j not in fixed]
        refined = np.asarray(vt, float).copy()
        try:
            for _ in range(LSQ_ITER):
                sl, ms, me, h0_, hn_, h_ = sizes(refined)
                base = _eval_many(build(refined, sl, ms, me, h0_, hn_, h_), grid)
                cols = []
                for j in free:
                    e = refined.copy()
                    e[j] += 1.0
                    cols.append(_eval_many(build(e, np.diff(e) / np.diff(vs), ms, me, h0_, hn_, h_), grid) - base)
                design = np.vstack([np.column_stack(cols), math.sqrt(LSQ_RIDGE) * np.eye(len(free))])
                rhs = np.concatenate([vals - base, np.zeros(len(free))])
                refined[free] += np.linalg.lstsq(design, rhs, rcond=None)[0]
            vt = refined
        except (ValueError, np.linalg.LinAlgError):
            pass                     # keep the RDP values
        flares.clear()
    slopes, m_start, m_end, h0, hn, h = sizes(vt)
    FLARES.extend(flares)
    if info is not None:
        info.update({"vs": np.asarray(vs, float), "vt": np.asarray(vt, float), "h": h.copy(), "h0": h0, "hn": hn,
                     "m0": m_start, "mn": m_end})
    if len(vs) == 2 and m_start is not None and m_end is not None and (ramp_start or ramp_end) and not c2_ends:
        # (C1 variants) one Hermite over the whole chain. With c2_ends the end blends stay local:
        # one transition across a long chain amplifies an end curvature into a deep dip.
        return [_hermite_piece(vs[0], vs[1], vt[0], m_start, vt[1], m_end)]
    return build(vt, slopes, m_start, m_end, h0, hn, h)


def _flare_start(obs_s, obs_t, a, b, outward, rdp_tol=None):
    """Start station of the outward flare that ends a boundary at a mouth (None if no flare).

    Walks back over the RDP runs of the observed profile (never the held value after the
    last observation) while the run turns outward faster than MOUTH_SLOPE_CAP.
    """
    obs_s, obs_t = np.asarray(obs_s, float), np.asarray(obs_t, float)
    end_obs = min(b, float(obs_s.max()))
    if end_obs - a < 2 * GRID:
        return None
    grid, vals = _grid_profile(obs_s, obs_t, a, end_obs)
    keep = _rdp(grid, vals, rdp_tol or RDP_TOL)
    vs, vt = grid[keep], vals[keep]
    j = len(vs) - 2
    while (j >= 0 and outward * (vt[j + 1] - vt[j]) / (vs[j + 1] - vs[j]) > MOUTH_SLOPE_CAP
           and vs[j] > end_obs - FLARE_SEARCH_M):
        j -= 1
    s_f = float(vs[j + 1])
    return s_f if s_f < end_obs - GRID else None


# --- reference projection and source observations ----------------------------------

def _project(points, ref_pts, ref_s, ref_h):
    """(s, t, clamped) of points on the sampled reference line."""
    a, b = ref_pts[:-1], ref_pts[1:]
    ab = b - a
    l2 = np.maximum((ab * ab).sum(axis=1), 1e-18)
    out = []
    for p in points:
        u = ((p - a) * ab).sum(axis=1) / l2
        uc = np.clip(u, 0.0, 1.0)
        q = a + uc[:, None] * ab
        d = np.linalg.norm(p - q, axis=1)
        i = int(np.argmin(d))
        clamped = (i == 0 and u[i] < 0) or (i == len(a) - 1 and u[i] > 1)
        s = ref_s[i] + uc[i] * (ref_s[i + 1] - ref_s[i])
        h = ref_h[i]
        t = -(p[0] - q[i, 0]) * math.sin(h) + (p[1] - q[i, 1]) * math.cos(h)
        out.append((s, t, clamped))
    return out


class _Road:
    """Sections, boundary tables and lane links of one ordinary road."""

    def __init__(self, road):
        from mapforge.validate.smoothness import lane_edges_at, sample_road_ref
        self.el = road
        self.length = float(road.get("length"))
        self.sections = road.findall("lanes/laneSection")
        self.s = [float(sec.get("s")) for sec in self.sections] + [self.length]
        self.ref = sample_road_ref(road, 0.25)
        self.lane_edges_at = lane_edges_at
        link = road.find("link")
        self.junction_start = link is not None and link.find("predecessor[@elementType='junction']") is not None
        self.junction_end = link is not None and link.find("successor[@elementType='junction']") is not None

    def lanes(self, i, side):
        return sorted(self.sections[i].findall(f"{side}/lane"), key=lambda ln: abs(int(ln.get("id"))))

    def current(self, i, side, k, s):
        """Current XODR offset of boundary k (0 = centre) on ``side`` at s."""
        return self.lane_edges_at(self.el, s, side)[k]


def _source_observations(road: _Road, src, origin, side, i, k):
    """(s, t) samples of boundary k in section i from the adjacent source lanes."""
    from mapforge.validate.shp_boundary_fidelity import _densify, _project as to_local
    s0, s1 = road.s[i], road.s[i + 1]
    lanes = road.lanes(i, side)
    obs = []
    for lane, which in ((lanes[k - 1] if k >= 1 else None, "outer"), (lanes[k] if k < len(lanes) else None, "inner")):
        if lane is None:
            continue
        sid = lane.find("userData[@code='mapforge.source_lane']")
        if sid is None:
            continue
        candidates = []
        for g in src.lane_boundary_geometries(sid.get("value")):
            pts = _densify(to_local(g, *origin), 0.5)
            st = [(s, t) for s, t, clamped in _project(pts, *road.ref) if not clamped and s0 <= s <= s1]
            if len(st) >= 1:
                candidates.append(np.asarray(st))
        if len(candidates) < 2:
            continue
        lane_k = abs(int(lane.get("id")))
        # inner/outer by lateral order (centre side has the smaller |t| offset ordering per side)
        mid = [float(np.median(c[:, 1])) for c in candidates]
        order = np.argsort(mid) if side == "left" else np.argsort(mid)[::-1]
        inner, outer = candidates[order[0]], candidates[order[-1]]
        obs.append(outer if which == "outer" else inner)
        _ = lane_k
    return np.vstack(obs) if obs else None


# --- chains -------------------------------------------------------------------------

def _chains(road: _Road, side):
    """Outer-boundary chains of one side: [{boundary at section i: k}, ...] via lane links."""
    chains, open_by_lane = [], {}
    for i in range(len(road.sections)):
        current = {}
        for lane in road.lanes(i, side):
            lid = int(lane.get("id"))
            k = abs(lid)
            pred = lane.find("link/predecessor")
            chain = None
            if i > 0 and pred is not None and int(pred.get("id")) in open_by_lane:
                chain = open_by_lane[int(pred.get("id"))]
            if chain is None:
                chain = {"members": {}, "birth": i > 0}
                chains.append(chain)
            chain["members"][i] = k
            current[lid] = chain
        for lid, chain in open_by_lane.items():
            if not any(c is chain for c in current.values()):
                chain["death"] = True
        open_by_lane = current
    return chains


def _obs_slope(obs, s_lo, s_hi):
    """Least-squares slope of (s, t) observations inside [s_lo, s_hi] (None if too few)."""
    if obs is None:
        return None
    sel = obs[(obs[:, 0] >= s_lo) & (obs[:, 0] <= s_hi)]
    if len(sel) < 4 or np.ptp(sel[:, 0]) < 1.0:
        return None
    return float(np.polyfit(sel[:, 0], sel[:, 1], 1)[0])


SPLIT_WINDOW_M = 10.0     # observations this far either side of a birth/death decide which line continues
SPLIT_SLOPE_MARGIN = 0.01


def _rechain(road: _Road, side, chains, obs_at):
    """Births and deaths where the *inner* boundary is the one that turns away.

    The chains assume a new lane peels its outer boundary off its inner neighbour (and a dying lane
    converges onto it). Where the source keeps the outer line straight and the inner boundary turns
    away instead (a median giving way to a left-turn lane), the straight line is the one that
    continues: the chains are swapped there, and the turning boundary peels off (or converges onto)
    its outer neighbour ("peel"/"converge" = "outer"). Decided by the observed slopes next to the
    event; ``obs_at(i, k)`` gives the (s, t) observations of boundary k in section i.
    """
    owner = lambda: {(i, k): c for c in chains for i, k in c["members"].items()}
    w = SPLIT_WINDOW_M
    for c in list(chains):
        # a lane that is born and dies on the same chain keeps the plain model: swapping one end would
        # make the two swapped chains depend on each other (birth parent <-> death parent)
        if not c.get("birth") or c.get("death"):
            continue
        i0 = min(c["members"])
        k = c["members"][i0]
        a = owner().get((i0, k - 1))
        if a is None or a is c or (i0 - 1) not in a["members"]:
            continue
        s = road.s[i0]
        pre = _obs_slope(obs_at(i0 - 1, a["members"][i0 - 1]), s - w, s)
        post_c = _obs_slope(obs_at(i0, k), s, s + w)
        post_a = _obs_slope(obs_at(i0, k - 1), s, s + w)
        if None in (pre, post_c, post_a) or not abs(post_c - pre) + SPLIT_SLOPE_MARGIN < abs(post_a - pre):
            continue
        a_post = {i: kk for i, kk in a["members"].items() if i >= i0}
        a["members"] = {**{i: kk for i, kk in a["members"].items() if i < i0}, **c["members"]}
        c["members"] = a_post
        a["death"], c["death"] = c.get("death", False), a.get("death", False)
        c["birth"], c["peel"] = True, "outer"
    for d in list(chains):
        if not d.get("death") or d.get("peel") or d.get("birth"):
            continue
        i1 = max(d["members"])
        k = d["members"][i1]
        e = owner().get((i1, k - 1))
        if e is None or e is d or (i1 + 1) not in e["members"]:
            continue
        s = road.s[i1 + 1]
        post = _obs_slope(obs_at(i1 + 1, e["members"][i1 + 1]), s, s + w)
        pre_d = _obs_slope(obs_at(i1, k), s - w, s)
        pre_e = _obs_slope(obs_at(i1, k - 1), s - w, s)
        if None in (post, pre_d, pre_e) or not abs(pre_d - post) + SPLIT_SLOPE_MARGIN < abs(pre_e - post):
            continue
        e_pre = {i: kk for i, kk in e["members"].items() if i <= i1}
        e["members"] = {**d["members"], **{i: kk for i, kk in e["members"].items() if i > i1}}
        d["members"] = e_pre
        e["birth"], d["birth"] = d.get("birth", False), e.get("birth", False)
        d["death"], d["converge"] = True, "outer"
    return chains


def _fit_order(chains):
    """Chains ordered so every birth/death parent boundary is fitted before the chain that needs it."""
    owner = {(i, k): idx for idx, c in enumerate(chains) for i, k in c["members"].items()}
    deps = []
    for c in chains:
        need = set()
        if c.get("birth"):
            i0 = min(c["members"])
            need.add(owner.get((i0, c["members"][i0] + (1 if c.get("peel") == "outer" else -1))))
        if c.get("death"):
            i1 = max(c["members"])
            need.add(owner.get((i1, c["members"][i1] + (1 if c.get("converge") == "outer" else -1))))
        deps.append({x for x in need if x is not None})
    done, order = set(), []
    pending = sorted(range(len(chains)), key=lambda idx: min(chains[idx]["members"].values()))
    while pending:
        ready = [idx for idx in pending if deps[idx] <= done or deps[idx] == {idx}]
        pick = ready[0] if ready else pending[0]
        order.append(chains[pick])
        done.add(pick)
        pending.remove(pick)
    return order


def _relative_cond(cond, base, x):
    """An end condition (value, slope[, second derivative] or None/"mouth") relative to ``base`` at x."""
    if cond is None:
        return None
    if cond[1] == "mouth":
        return (None if cond[0] is None else cond[0] - _at(base, x), "mouth")
    return tuple(None if v is None else v - _at(base, x, order) for order, v in enumerate(cond))


def refit_road(road_el, src, origin, params=None, observe=None, relative=None):
    """``observe(road, side, i, k, fitted)``: (s, t) source samples of boundary k in section i (default: the
    SHP lane boundaries of ``src``; MAP: mapforge.ops.map_lane_refit). ``fitted`` holds the boundaries fitted
    so far ((side, i, k) -> pieces; inner boundaries are fitted first).
    ``relative(road, side, chain)``: True fits that chain's boundary as an offset from its fitted inner
    boundary (the lane width), so a zero-width stretch stays exactly zero and an opening taper never crosses
    its inner boundary (MAP turn bays); default: every boundary is fitted on its own."""
    params = dict(params or {})
    curb_shoulder = params.pop("curb_shoulder", CURB_SHOULDER)
    rechain = params.pop("rechain", False)
    # free road ends where a source lane starts (ends) narrow some way in: zero width up to there (SHP)
    zero_starts = params.pop("protect_unobserved_ends", False)
    if zero_starts:
        params["free_end_runs"] = True
    FLARES.clear()
    road = _Road(road_el)
    n_sec = len(road.sections)
    cuts = road.s[1:-1]
    fitted = {}  # (side, i, k) -> pieces covering section i (k = 0 centre shared)
    report = {"road": road_el.get("id"), "chains": 0, "fallback_boundaries": 0,
              "c2_ends": bool(params.get("c2_ends"))}
    shoulders = []
    if observe is None:
        def observe(road_, side, i, k, fitted_=None):
            return _source_observations(road_, src, origin, side, i, k)

    def observations(side_list, members):
        """Source samples over a whole chain; current geometry only if the chain has none at all.

        Sections without source samples inside a chain are bridged by the fit, never filled
        with the current (possibly wrong) geometry."""
        found = [o for i, k in members for side in side_list
                 for o in [observe(road, side, i, k, fitted)] if o is not None]
        if found:
            return np.vstack(found), True
        rows = []
        for i, k in members:
            s = np.linspace(road.s[i], road.s[i + 1], max(3, int((road.s[i + 1] - road.s[i]) / GRID) + 1))
            rows += [(x, road.current(i, side_list[0], k, x)) for x in s]
        return np.asarray(rows), False

    def mouth(at_start):
        return (None, "mouth") if (road.junction_start if at_start else road.junction_end) else None

    # centre boundary over the whole road
    obs, any_src = observations(("right", "left"), [(i, 0) for i in range(n_sec)])
    pieces = _split(fit_boundary(obs[:, 0], obs[:, 1], 0.0, road.length,
                                 start=mouth(True), end=mouth(False), **params), cuts)
    for side in ("right", "left"):
        for i in range(n_sec):
            fitted[(side, i, 0)] = [p for p in pieces if road.s[i] - 1e-6 <= p[0] < road.s[i + 1] - 1e-6]
    report["chains"] += 1
    report["fallback_boundaries"] += 0 if any_src else 1

    for side in ("right", "left"):
        chains = sorted(_chains(road, side), key=lambda c: min(c["members"].values()))
        if rechain:
            def obs_at(i, k, side=side):
                try:
                    return observe(road, side, i, k, fitted)
                except IndexError:
                    return None
            chains = _fit_order(_rechain(road, side, chains, obs_at))
            report.setdefault("rechained", []).extend(
                {"side": side, "members": {str(i): k for i, k in c["members"].items()},
                 "peel": c.get("peel"), "converge": c.get("converge")}
                for c in chains if c.get("peel") or c.get("converge"))
        for chain in chains:
            secs = sorted(chain["members"])
            a, b = road.s[secs[0]], road.s[secs[-1] + 1]
            obs, any_src = observations((side,), [(i, chain["members"][i]) for i in secs])
            first_k, last_k = chain["members"][secs[0]], chain["members"][secs[-1]]
            inner_start = fitted.get((side, secs[0], first_k + (1 if chain.get("peel") == "outer" else -1)))
            inner_end = fitted.get((side, secs[-1], last_k + (1 if chain.get("converge") == "outer" else -1)))
            if chain.get("birth") and not inner_start:
                report.setdefault("parent_missing", []).append({"side": side, "at": "birth", "s": round(a, 3)})
            if chain.get("death") and not inner_end:
                report.setdefault("parent_missing", []).append({"side": side, "at": "death", "s": round(b, 3)})
            if chain.get("birth") and inner_start:
                start = (_at(inner_start, a), _at(inner_start, a, 1), _at(inner_start, a, 2))
            else:
                start = mouth(True) if secs[0] == 0 else None
            if chain.get("death") and inner_end:
                end = (_at(inner_end, b), _at(inner_end, b, 1), _at(inner_end, b, 2))
            else:
                end = mouth(False) if secs[-1] == n_sec - 1 else None
            ramp_start, ramp_end = bool(chain.get("birth")), bool(chain.get("death"))
            flares_before = len(FLARES)
            base = None
            advanced = _advanced_events(road, side, chain) if chain.get("birth") or chain.get("death") else {}
            if zero_starts and any_src:
                advanced.update(_zero_starts(road, side, chain, obs, fitted, start, end, a, b))
            if advanced or (relative is not None and relative(road, side, chain)):
                inner = [fitted.get((side, i, chain["members"][i] - 1)) for i in secs]
                base = [p for part in inner for p in part] if all(inner) else None
            if base and not advanced:
                rel = obs[:, 1] - np.array([_at(base, x) for x in obs[:, 0]])
                whole = _combine(fit_boundary(obs[:, 0], rel, a, b, start=_relative_cond(start, base, a),
                                              end=_relative_cond(end, base, b), ramp_start=ramp_start,
                                              ramp_end=ramp_end, **params), base, a, b, 1.0)
                report.setdefault("relative_chains", 0)
                report["relative_chains"] += 1
            elif base:
                rel = obs[:, 1] - np.array([_at(base, x) for x in obs[:, 0]])
                obs_s = obs[:, 0]
                protect, corner_h, fills = [], {}, {}
                for kind, (x, d, fill_w) in advanced.items():
                    # the lane exists d ahead of its source event with zero width; the source corner at x is
                    # rounded symmetrically (half length d, or as the curvature target asks where d is None)
                    # instead of by a one-sided ramp after the event
                    fill = np.arange(a, x, GRID) if kind == "birth" else np.arange(b, x, -GRID)
                    keep = (obs_s > x) if kind == "birth" else (obs_s < x)
                    obs_s, rel = np.concatenate([fill, obs_s[keep]]), np.concatenate([np.full(len(fill), fill_w), rel[keep]])
                    protect.append(x)
                    fills[x] = fill_w
                    if d is not None:
                        corner_h[x] = d
                    if kind == "birth":
                        ramp_start = False
                    else:
                        ramp_end = False
                whole, cut = _advanced_fit(obs_s, rel, obs, a, b, base, start, end, ramp_start, ramp_end,
                                           [(kind, x) for kind, (x, _, _) in advanced.items()], corner_h, params,
                                           fills)
                report.setdefault("advanced_events", 0)
                report["advanced_events"] += len(advanced)
                if cut is not None:
                    report.setdefault("local_relative", []).append([round(cut[0], 3), round(cut[1], 3)])
            else:
                whole = fit_boundary(obs[:, 0], obs[:, 1], a, b, start=start, end=end,
                                     ramp_start=ramp_start, ramp_end=ramp_end, **params)
            shoulder = None
            if (curb_shoulder and base is None and any_src and len(FLARES) > flares_before and end == (None, "mouth")
                    and last_k == len(road.lanes(secs[-1], side))):
                shoulder = _curb_shoulder(road, side, chain, obs, a, b, start, ramp_start, params)
            if shoulder is not None:
                del FLARES[flares_before:]          # handled: reported as a curb-return shoulder
                whole, outer, info = shoulder
                for i in info["sections"]:
                    fitted[(side, i, chain["members"][i] + 1)] = [
                        p for p in _split(outer, cuts) if road.s[i] - 1e-6 <= p[0] < road.s[i + 1] - 1e-6]
                shoulders.append(info)
            pieces = _split(whole, cuts)
            for i in secs:
                fitted[(side, i, chain["members"][i])] = [p for p in pieces
                                                          if road.s[i] - 1e-6 <= p[0] < road.s[i + 1] - 1e-6]
            report["chains"] += 1
            report["fallback_boundaries"] += 0 if any_src else 1
    report["strong_mouth_flares"] = [round(x, 3) for x in FLARES]
    report["curb_return_shoulders"] = shoulders
    return road, fitted, report


def _advanced_fit(obs_s, rel, obs, a, b, base, start, end, ramp_start, ramp_end, events, corner_h, params,
                  fills=None):
    """Boundary of a lane with advanced births/deaths (or zero-width free-end starts): (pieces, own-fit span).

    Near each event the boundary is its inner boundary plus the fitted width (zero up to the event, so the lane
    exists with exactly zero width). The corner at the event is the shortest one (from H_MIN in ADVANCED_H_STEP,
    up to the advance) whose *boundary* curvature, inner boundary plus width, stays within the curvature target,
    or within 5 % of the lowest reachable: sizing it from the width kink alone (|dm| / kappa) made it far too long
    where the inner boundary turns away at the same point (shp-node18 road 10 lane -3: width kink 0.28 of which
    0.18 is the inner boundary's own corner; 6.9 m half length, 0.19 m off at the event, 0.315 m after).
    Away from the events the width would carry both boundaries' corners (a later corner of the same lane came out
    0.29 m off instead of 0.13 m), so in the first straight width stretch past the event (CUT_GAP_M) the boundary
    hands over, curvature continuously, to its own fit of the source; without a later width vertex it stays
    relative. ``fills`` {station: width} is the width before (after) the event, 0 unless a tail (TAIL_WIDTH_M)."""
    kappa = params.get("kappa_cap") or KAPPA_CAP
    protect = [x for _, x in events]
    fills = {x: (fills or {}).get(x, 0.0) for x in protect}
    flares0 = len(FLARES)

    # a free chain end before a birth (after a death) is fixed at zero width, not left to the fit
    rel_start, rel_end = _relative_cond(start, base, a), _relative_cond(end, base, b)
    for kind, x in events:
        if kind == "birth" and rel_start is None:
            rel_start = (fills[x], None)
        if kind == "death" and rel_end is None:
            rel_end = (fills[x], None)

    def run(ch, info=None):
        out = fit_boundary(obs_s, rel, a, b, start=rel_start, end=rel_end,
                           ramp_start=ramp_start, ramp_end=ramp_end, protect=protect, corner_h=ch,
                           protect_values=fills, info=info, **params)
        del FLARES[flares0:]        # mouth flares are judged on the absolute fit, not on the width
        return out

    chosen = dict(corner_h)
    for x in protect:
        if not (a + GRID < x < b - GRID):
            continue
        cap = corner_h.get(x, ADVANCED_H_MAX)
        tried = []
        for hx in np.arange(H_MIN, max(cap, H_MIN) + 1e-9, ADVANCED_H_STEP):
            info = {}
            width = run({**chosen, x: float(hx)}, info)
            i = int(np.argmin(np.abs(info["vs"] - x)))
            hu = float(info["h"][i])
            ss = np.linspace(x - hu, x + hu, 41)
            tried.append((float(hx), max(abs(_at(base, q, 2) + _at(width, q, 2)) for q in ss)))
        bound = max(kappa, 1.05 * min(k for _, k in tried))
        chosen[x] = next(hx for hx, k in tried if k <= bound + 1e-12)
    info = {}
    width = run(chosen, info)
    whole = _combine(width, base, a, b, 1.0)
    vs, h, h0, hn = info["vs"], info["h"], info["h0"], info["hn"]
    n = len(vs)
    gaps = [(vs[j] + (h0 if j == 0 else h[j]), vs[j + 1] - (hn if j + 1 == n - 1 else h[j + 1])) for j in range(n - 1)]
    lo, hi = a, b
    for kind, x in events:
        i = int(np.argmin(np.abs(vs - x)))
        if kind == "birth":
            # a straight stretch after the event corner with another width vertex still to come
            cand = [g for j, g in enumerate(gaps) if j >= i and j + 1 < n - 1 and g[1] - g[0] >= CUT_GAP_M]
            lo = 0.5 * (cand[0][0] + cand[0][1]) if cand else b
        else:
            cand = [g for j, g in enumerate(gaps) if j < i and j > 0 and g[1] - g[0] >= CUT_GAP_M]
            hi = 0.5 * (cand[-1][0] + cand[-1][1]) if cand else a
    # hand over only where the relative fit lies on the source: a cut on a run that already misses it (a chord
    # into a rounded knee, shp-node13 road 10 lane 5: 0.27 m) would pass that error on to the own fit
    for q in (lo, hi):
        if a < q < b:
            near = obs[np.abs(obs[:, 0] - q) <= 1.0]
            if not len(near) or float(np.median(np.abs([_at(whole, x) - t for x, t in near]))) > RDP_TOL:
                lo, hi = (b, hi) if q == lo else (lo, a)
    if hi - lo < 2 * H_MIN:
        return whole, None

    def state(q):
        return (_at(whole, q), _at(whole, q, 1), _at(whole, q, 2))
    sel = (obs[:, 0] >= lo) & (obs[:, 0] <= hi)
    if sel.sum() < 4:
        return whole, None
    own = fit_boundary(obs[sel, 0], obs[sel, 1], lo, hi, start=state(lo) if lo > a else start,
                       end=state(hi) if hi < b else end, ramp_start=ramp_start and lo == a,
                       ramp_end=ramp_end and hi == b, **params)
    keep = [p for p in _split(whole, [lo, hi]) if p[1] <= lo + 1e-6 or p[0] >= hi - 1e-6]
    return sorted(keep + own, key=lambda p: p[0]), (lo, hi)


def _zero_starts(road, side, chain, obs, fitted, start, end, a, b):
    """{"birth"/"death": (station, None)} where a free chain end has no source for more than UNOBSERVED_END_M and
    the source lane there starts (ends) narrower than ZERO_START_M: shp-node13 road 10 lane 5 exists from the
    road start, its source opens 4 m in from zero width; the refit held the first value, pruned the short
    held stretch and the lane widened from the road start (0.81 m off at the source start)."""
    secs = sorted(chain["members"])
    inner = [fitted.get((side, i, chain["members"][i] - 1)) for i in secs]
    if not all(inner) or not len(obs):
        return {}
    base = [p for part in inner for p in part]
    out = {}
    lo, hi = float(np.min(obs[:, 0])), float(np.max(obs[:, 0]))
    # where the lane ends there in its travel direction (right lanes run with s, left lanes against it) the width
    # is held at TAIL_WIDTH_M, not zero: vehicles would drive into an exact zero width up to the road's free end,
    # where no section boundary marks the lane end (esmini moves them to the next lane: a 2 m jump on shp-node13
    # road 10 lane 2). Held on the width, not on the edge position: an edge held flat crossed its inner boundary
    # (lane 5 of the same road, -0.107 m at the road start).
    with_s = _travels_with_s(road, side, chain)
    sign = 1.0 if side == "left" else -1.0
    if start is None and not chain.get("birth") and lo > a + UNOBSERVED_END_M:
        first = obs[obs[:, 0] <= lo + GRID]
        if abs(float(np.median(first[:, 1])) - _at(base, lo)) < ZERO_START_M:
            out["birth"] = (lo, None, 0.0 if with_s else sign * TAIL_WIDTH_M)
    if end is None and not chain.get("death") and hi < b - UNOBSERVED_END_M:
        last = obs[obs[:, 0] >= hi - GRID]
        if abs(float(np.median(last[:, 1])) - _at(base, hi)) < ZERO_START_M:
            out["death"] = (hi, None, sign * TAIL_WIDTH_M if with_s else 0.0)
    return out


def _travels_with_s(road, side, chain):
    """Travel direction of the chain's lane from its provenance (``travel_direction``), else by side (right
    lanes with s)."""
    secs = sorted(chain["members"])
    lane = next((ln for ln in road.lanes(secs[0], side) if abs(int(ln.get("id"))) == chain["members"][secs[0]]), None)
    prov = lane.find("userData[@code='mapforge.provenance/v1']") if lane is not None else None
    if prov is not None and prov.get("value"):
        direction = json.loads(prov.get("value")).get("travel_direction")
        if direction in ("with_s", "against_s"):
            return direction == "with_s"
    return side == "right"


def _advanced_events(road, side, chain):
    """{"birth"/"death": (source event station, advance)} from lane_birth_advance markers on the chain's lanes."""
    from mapforge.ops.lane_birth_advance import CODE as ADVANCE_CODE
    out = {}
    secs = sorted(chain["members"])
    for kind, i in (("birth", secs[0]), ("death", secs[-1])):
        if not chain.get(kind):
            continue
        lane = next((ln for ln in road.lanes(i, side) if abs(int(ln.get("id"))) == chain["members"][i]), None)
        mark = lane.find(f"userData[@code='{ADVANCE_CODE}']") if lane is not None else None
        if mark is None:
            continue
        value = json.loads(mark.get("value"))
        station = value.get("source_birth_s" if kind == "birth" else "source_death_s")
        if station is not None:
            out[kind] = (float(station), float(value["advance_m"]), 0.0)
    return out


def _curb_tail(obs_s, obs_t, s_f, observed_end, b, v0, m0):
    """Curb-return edge from the flare start: one least-squares cubic tangent to the driving edge.

    A curb return is one smooth arc-like curve, not a polyline with corners, so the observed
    flare is fitted by t = v0 + m0*u + c*u^2 + d*u^3 (u = s - s_f). Beyond the last source point
    the edge levels off towards the driving-edge slope at most at CURB_HOLD_KAPPA, then holds.
    """
    keep = (obs_s >= s_f) & (obs_s <= observed_end)
    u = obs_s[keep] - s_f
    rhs = obs_t[keep] - v0 - m0 * u
    (c, d), *_ = np.linalg.lstsq(np.column_stack([u * u, u ** 3]), rhs, rcond=None)
    pieces = [(s_f, observed_end, v0, m0, float(c), float(d))]
    if observed_end < b - 1e-6:
        v, m = _eval(pieces[0], observed_end), _eval(pieces[0], observed_end, 1)
        dm = m - m0
        run = min(b - observed_end, abs(dm) / CURB_HOLD_KAPPA) if abs(dm) > 1e-9 else 0.0
        c2 = -math.copysign(min(CURB_HOLD_KAPPA, abs(dm) / run), dm) if run > 1e-9 else 0.0
        if run > 1e-6:
            pieces.append((observed_end, observed_end + run, v, m, c2 / 2, 0.0))
        if observed_end + run < b - 1e-6:
            p = pieces[-1]
            end = observed_end + run
            pieces.append((end, b, _eval(p, end), _eval(p, end, 1), 0.0, 0.0))
    return pieces


def _curb_shoulder(road, side, chain, obs, a, b, start, ramp_start, params):
    """Split a strong end flare into a pre-flare driving boundary and a curb-return shoulder edge.

    Returns (driving pieces on [a, b], shoulder outer pieces on [first shoulder section, b], info)
    or None when the flare cannot be separated cleanly (then the flare is kept and reported).
    """
    outward = 1.0 if side == "left" else -1.0
    s_f = _flare_start(obs[:, 0], obs[:, 1], a, b, outward, params.get("rdp_tol"))
    if s_f is None or s_f - a < FLARE_MIN_RUN_M:
        return None
    secs = sorted(chain["members"])
    sections = [i for i in secs if road.s[i + 1] > s_f + 1e-6]
    if any(chain["members"][i] != len(road.lanes(i, side)) for i in sections):
        return None                                  # the shoulder must stay the outermost lane
    before = obs[obs[:, 0] < s_f]
    after = obs[obs[:, 0] >= s_f - GRID]
    if len(before) < 4 or len(after) < 4:
        return None
    flares_before = len(FLARES)
    drive = fit_boundary(before[:, 0], before[:, 1], a, b, start=start, end=(None, "mouth"),
                         ramp_start=ramp_start, **params)
    if len(FLARES) > flares_before:                  # still flaring without the flare: not a curb return
        del FLARES[flares_before:]
        return None
    v0, m0 = _at(drive, s_f), _at(drive, s_f, 1)
    observed_end = min(b, float(obs[:, 0].max()))
    if observed_end - s_f < 2.0:
        return None
    tail = _curb_tail(after[:, 0], after[:, 1], s_f, observed_end, b, v0, m0)
    s_first = road.s[sections[0]]
    outer = [p for p in _split(drive, [s_first, s_f]) if s_first - 1e-6 <= p[0] < s_f - 1e-6] + tail
    widths = [outward * (_at(outer, s) - _at(drive, s)) for s in np.linspace(s_f, b, 200)]
    if min(widths) < -NEGATIVE_WIDTH_TOL:
        return None
    info = {"side": side, "driving_lane": chain["members"][sections[-1]],
            "lane": chain["members"][sections[-1]] + 1, "sections": sections,
            "members": {i: chain["members"][i] for i in sections},
            "flare_start_s": round(s_f, 3), "flare_length_m": round(b - s_f, 3),
            "width_max_m": round(float(max(widths)), 3), "width_at_mouth_m": round(float(widths[-1]), 3),
            "held_after_source_m": round(max(0.0, b - observed_end), 3)}
    return drive, outer, info


def _difference(inner, outer, s0, s1, sign):
    """Width pieces sign*(inner - outer) on [s0, s1] at the union of both knot sets."""
    knots = sorted({s0, s1} | {p[0] for p in inner + outer if s0 < p[0] < s1})
    out = []
    for lo, hi in zip(knots[:-1], knots[1:]):
        mid = (lo + hi) / 2
        pi = next(p for p in inner if p[0] - 1e-9 <= mid <= p[1] + 1e-9)
        po = next(p for p in outer if p[0] - 1e-9 <= mid <= p[1] + 1e-9)
        ri, ro = _rebase(pi, lo), _rebase(po, lo)
        out.append((lo, hi, *(sign * (x - y) for x, y in zip(ri[2:], ro[2:]))))
    return out


def _fmt(v):
    return format(float(v), ".15g")


REPAIR_MAX_M = 0.15


def _samples(pieces, n=20):
    return [(p[0] + (p[1] - p[0]) * j / n, _eval(p, p[0] + (p[1] - p[0]) * j / n)) for p in pieces for j in range(n + 1)]


def _combine(a, b, s0, s1, sign):
    """Pieces of a + sign*b on [s0, s1] at the union of both knot sets."""
    knots = sorted({s0, s1} | {p[0] for p in a + b if s0 < p[0] < s1})
    out = []
    for lo, hi in zip(knots[:-1], knots[1:]):
        mid = (lo + hi) / 2
        pa = next(p for p in a if p[0] - 1e-9 <= mid <= p[1] + 1e-9)
        pb = next(p for p in b if p[0] - 1e-9 <= mid <= p[1] + 1e-9)
        ra, rb = _rebase(pa, lo), _rebase(pb, lo)
        out.append((lo, hi, *(x + sign * y for x, y in zip(ra[2:], rb[2:]))))
    return out


def _bump(s0, s1, lo, peak_s, hi, height):
    """C1 bump: 0 outside [lo, hi], ``height`` at peak_s, flat ends; zero pieces elsewhere in [s0, s1]."""
    pieces = []
    if lo > s0 + 1e-9:
        pieces.append((s0, lo, 0.0, 0.0, 0.0, 0.0))
    pieces.append(_hermite_piece(lo, peak_s, 0.0, 0.0, height, 0.0))
    pieces.append(_hermite_piece(peak_s, hi, height, 0.0, 0.0, 0.0))
    if hi < s1 - 1e-9:
        pieces.append((hi, s1, 0.0, 0.0, 0.0, 0.0))
    return pieces


def _bump_c2(s0, s1, lo, peak_s, hi, height):
    """C2 bump (G2 transitions up and down): 0 outside [lo, hi], ``height`` at peak_s."""
    pieces = []
    if lo > s0 + 1e-9:
        pieces.append((s0, lo, 0.0, 0.0, 0.0, 0.0))
    pieces += _g2_transition(lo, peak_s, (0.0, 0.0, 0.0), (height, 0.0, 0.0))
    pieces += _g2_transition(peak_s, hi, (height, 0.0, 0.0), (0.0, 0.0, 0.0))
    if hi < s1 - 1e-9:
        pieces.append((hi, s1, 0.0, 0.0, 0.0, 0.0))
    return pieces


def _repair_crossings(road, widths, report):
    """Lift small negative widths with a bump (C2 with c2_ends, else C1), taken from the next lane out so
    outer edges stay put."""
    bump_fn = _bump_c2 if report.get("c2_ends") else _bump
    repairs = []
    for (i, side, k), pieces in list(widths.items()):
        vals = _samples(pieces)
        low_s, low = min(vals, key=lambda x: x[1])
        if low >= -NEGATIVE_WIDTH_TOL:
            continue
        if low < -REPAIR_MAX_M:
            return False
        s0, s1 = road.s[i], road.s[i + 1]
        neg = [s for s, v in vals if v < 0]
        lo, hi = max(s0, min(neg) - 2.0), min(s1, max(neg) + 2.0)
        peak = min(max(low_s, lo + 0.25 * (hi - lo)), hi - 0.25 * (hi - lo))
        for factor in (1.1, 1.5, 2.0, 3.0):
            fixed = _combine(pieces, bump_fn(s0, s1, lo, peak, hi, -low * factor), s0, s1, 1.0)
            if min(v for _, v in _samples(fixed)) >= -NEGATIVE_WIDTH_TOL:
                break
        else:
            return False
        bump = bump_fn(s0, s1, lo, peak, hi, -low * factor)
        widths[(i, side, k)] = fixed
        if (i, side, k + 1) in widths:
            widths[(i, side, k + 1)] = _combine(widths[(i, side, k + 1)], bump, s0, s1, -1.0)
        repairs.append({"section": i, "side": side, "lane": k, "min_width_m": low, "lift_m": -low * factor})
    report["crossing_repairs"] = repairs
    return all(min(v for _, v in _samples(p)) >= -NEGATIVE_WIDTH_TOL for p in widths.values())


def write_road(road: _Road, fitted, report):
    """Replace laneOffset and width records; returns False (road unchanged) on negative widths."""
    lanes_el = road.el.find("lanes")
    widths = {}
    for i, sec in enumerate(road.sections):
        s0, s1 = road.s[i], road.s[i + 1]
        for side, sign in (("right", 1.0), ("left", -1.0)):
            for lane in road.lanes(i, side):
                k = abs(int(lane.get("id")))
                widths[(i, side, k)] = _difference(fitted[(side, i, k - 1)], fitted[(side, i, k)], s0, s1, sign)
    for info in report.get("curb_return_shoulders", []):
        side, sign = info["side"], (1.0 if info["side"] == "right" else -1.0)
        for i, k in info["members"].items():
            widths[(i, side, k + 1)] = _difference(fitted[(side, i, k)], fitted[(side, i, k + 1)],
                                                   road.s[i], road.s[i + 1], sign)
    if not _repair_crossings(road, widths, report):
        worst = min(v for p in widths.values() for _, v in _samples(p))
        report["skipped"] = f"negative width {worst:.3f} m"
        return False
    worst = min(v for p in widths.values() for _, v in _samples(p))
    old_offsets = lanes_el.findall("laneOffset")
    insert_at = list(lanes_el).index(old_offsets[0]) if old_offsets else 0
    tail = old_offsets[-1].tail if old_offsets else "\n"
    for el in old_offsets:
        lanes_el.remove(el)
    centre = [p for i in range(len(road.sections)) for p in fitted[("right", i, 0)]]
    for j, p in enumerate(centre):
        el = etree.Element("laneOffset", s=_fmt(p[0]), a=_fmt(p[2]), b=_fmt(p[3]), c=_fmt(p[4]), d=_fmt(p[5]))
        el.tail = tail
        lanes_el.insert(insert_at + j, el)
    for i, sec in enumerate(road.sections):
        s0 = road.s[i]
        for side in ("right", "left"):
            for lane in road.lanes(i, side):
                k = abs(int(lane.get("id")))
                old = lane.findall("width")
                at = list(lane).index(old[0]) if old else len(lane.findall("link"))
                w_tail = old[-1].tail if old else "\n"
                for el in old:
                    lane.remove(el)
                for j, p in enumerate(widths[(i, side, k)]):
                    a = max(p[2], 0.0) if abs(p[2]) < NEGATIVE_WIDTH_TOL else p[2]
                    el = etree.Element("width", sOffset=_fmt(p[0] - s0), a=_fmt(a), b=_fmt(p[3]),
                                       c=_fmt(p[4]), d=_fmt(p[5]))
                    el.tail = w_tail
                    lane.insert(at + j, el)
    for info in report.get("curb_return_shoulders", []):
        _write_shoulder(road, info, widths)
        info["members"] = {str(i): k for i, k in info["members"].items()}  # JSON-safe report
    report["width_records"] = sum(len(v) for v in widths.values())
    report["min_width_m"] = worst
    return True


# Lanes without source support keep their own exclusion code, so the shoulder is compared exactly
# where its driving lane was (shp_boundary_fidelity skips these codes).
UNSUPPORTED_CODES = {"source-extension", "source-support-unavailable", "source-support-too-short"}


def _write_shoulder(road, info, widths):
    """Insert the curb-return shoulder lane (outermost, non-driving, no junction link)."""
    side = info["side"]
    sign = 1 if side == "left" else -1
    secs = info["sections"]
    for n, i in enumerate(secs):
        k = info["members"][i]
        s0 = road.s[i]
        side_el = road.sections[i].find(side)
        driving = next(ln for ln in side_el.findall("lane") if abs(int(ln.get("id"))) == k)
        lane = etree.Element("lane", id=str(sign * (k + 1)), type="shoulder", level="false")
        lane.text, lane.tail = driving.text, driving.tail
        if n > 0 or n + 1 < len(secs):
            link = etree.SubElement(lane, "link")
            if n > 0:
                etree.SubElement(link, "predecessor", id=str(sign * (info["members"][secs[n - 1]] + 1)))
            if n + 1 < len(secs):
                etree.SubElement(link, "successor", id=str(sign * (info["members"][secs[n + 1]] + 1)))
        for p in widths[(i, side, k + 1)]:
            a = max(p[2], 0.0) if abs(p[2]) < NEGATIVE_WIDTH_TOL else p[2]
            etree.SubElement(lane, "width", sOffset=_fmt(p[0] - s0), a=_fmt(a), b=_fmt(p[3]),
                             c=_fmt(p[4]), d=_fmt(p[5]))
        etree.SubElement(lane, "roadMark", sOffset="0", type="none", weight="standard", color="standard",
                         width="0", laneChange="none")
        source = driving.find("userData[@code='mapforge.source_lane']")
        if source is not None:
            etree.SubElement(lane, "userData", code="mapforge.source_lane", value=source.get("value"))
        prov = driving.find("userData[@code='mapforge.provenance/v1']")
        record = json.loads(prov.get("value")) if prov is not None and prov.get("value") else {}
        code = record.get("exclusion_code")
        record.pop("policy_class", None)
        record.update(eligibility="excluded", role="shoulder", support_kind="physical-edge-fill",
                      status="TRANSFORMED", curb_return=True, derived_from_lane=sign * k,
                      exclusion_code=code if code in UNSUPPORTED_CODES else SHOULDER_CODE)
        etree.SubElement(lane, "userData", code="mapforge.provenance/v1",
                         value=json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        side_el.append(lane)
        ordered = sorted(side_el.findall("lane"), key=lambda ln: -int(ln.get("id")))  # left to right
        for ln in ordered:
            side_el.remove(ln)
        for ln in ordered:
            side_el.append(ln)


def refit_tree(root, src, params=None):
    from mapforge.validate.shp_boundary_fidelity import _origin
    origin = _origin(root)
    rows, skipped = [], []
    for road_el in root.findall("road"):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        if road_el.find("lanes/laneSection/*/lane/border") is not None:
            skipped.append({"road": road_el.get("id"), "reason": "border records"})
            continue
        if road_el.find(f"userData[@code='{CODE}']") is not None:
            skipped.append({"road": road_el.get("id"), "reason": "already refitted"})
            continue
        try:
            road, fitted, report = refit_road(road_el, src, origin, params)
            report["params"] = params or {"mode": MODE, "kappa_cap": KAPPA_CAP}
            if write_road(road, fitted, report):
                road_el.append(etree.Element("userData", code=CODE, value=json.dumps(report, ensure_ascii=False)))
                rows.append(report)
            else:
                skipped.append({"road": road_el.get("id"), "reason": report.get("skipped")})
        except (ValueError, KeyError, StopIteration, IndexError) as exc:
            skipped.append({"road": road_el.get("id"), "reason": f"{type(exc).__name__}: {exc}"})
    return {"schema": CODE, "rewritten": len(rows), "skipped": skipped, "rows": rows}


def advance_births(root, src):
    """lane_birth_advance on every ordinary road (before the refit); returns {road id: events}."""
    from mapforge.ops import lane_birth_advance
    from mapforge.validate.shp_boundary_fidelity import _origin
    origin = _origin(root)
    out = {}
    for road_el in root.findall("road"):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        road = _Road(road_el)

        def observe(side, i, k, road=road):
            try:
                return _source_observations(road, src, origin, side, i, k)
            except IndexError:
                return None
        events = lane_birth_advance.advance(road_el, observe, KAPPA_CAP)
        if events:
            out[road_el.get("id")] = events
    return out


_SOURCE = None


def _shp_source():
    global _SOURCE
    if _SOURCE is None:
        from mapforge.adapters.shp.profile_source import ProfileSource
        _SOURCE = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    return _SOURCE


def apply(xodr_in, xodr_out, params=None):
    """Lane refit only (SHP outputs; MAP outputs only with ``params["map_refit"]``, else copied unchanged).

    ``params["merge_short_ref"]``: first merge reference primitives shorter than 5 m on ordinary roads
    (both pipelines, mapforge.ops.refline_merge).
    ``params["map_refit"]``: MAP outputs get their road-side boundaries refitted to the MAP lane point
    lists of the source manifest next to ``xodr_in`` (mapforge.ops.map_lane_refit).
    ``params["approach_fair"]``: after that refit, the MAP approach sides are faired within 5 cm of their MAP point
    lists (never farther than now where already farther; mapforge.ops.map_approach_fair; road ends kept).
    ``params["departure_ease"]``: then the mirrored MAP departure sides are eased where they bend
    (mapforge.ops.map_departure_ease; mouths, far ends and approach sides kept).
    ``params["birth_advance"]``, ``["short_kappa"]``, ``["lsq_refine"]``, ``["link_gaps"]``: SHP only
    (lane_birth_advance, the short run rule and the least-squares vertex values of fit_boundary, gap lanes between
    parallel source links: lane_gap); MAP refits never see them.
    """
    params = dict(params or {})
    merge = params.pop("merge_short_ref", False)
    map_refit = params.pop("map_refit", False)
    departure_ease = params.pop("departure_ease", False)
    approach_fair = params.pop("approach_fair", False)
    params.pop("edge_joins", None)          # a connector step of apply_with_mouths
    birth_advance = params.pop("birth_advance", False)
    link_gaps = params.pop("link_gaps", False)
    shp_only = {k: params.pop(k) for k in ("short_kappa", "lsq_refine") if k in params}
    tree = etree.parse(str(xodr_in), etree.XMLParser(strip_cdata=False, remove_blank_text=False))
    root = tree.getroot()
    merged = None
    if merge:
        from mapforge.ops import refline_merge
        merged = refline_merge.merge_tree(root)
    is_shp = any(u.get("value", "").startswith("20") for u in root.iter("userData")
                 if u.get("code") == "mapforge.source_lane")
    if is_shp:
        params = {**params, **shp_only}
        gaps = None
        if link_gaps:
            from mapforge.ops import lane_gap
            gaps = lane_gap.apply(root, _shp_source())
        advanced = advance_births(root, _shp_source()) if birth_advance else None
        report = refit_tree(root, _shp_source(), {**params, "protect_unobserved_ends": True} if birth_advance else params)
        if advanced is not None:
            report["birth_advance"] = advanced
        if gaps is not None:
            report["link_gaps"] = gaps
    elif map_refit:
        from mapforge.ops import map_lane_refit
        report = map_lane_refit.refit_tree(root, Path(xodr_in).with_suffix(".source-lanes.json"), params)
        if approach_fair:
            from mapforge.ops import map_approach_fair
            report["approach_fair"] = map_approach_fair.apply(
                root, map_lane_refit.load_centres(Path(xodr_in).with_suffix(".source-lanes.json")))
        if departure_ease:
            from mapforge.ops import map_departure_ease
            report["departure_ease"] = map_departure_ease.apply(root)
    else:
        report = {"schema": CODE, "rewritten": 0, "skipped": [], "rows": [], "note": "MAP output: unchanged"}
    if merged is not None:
        report["refline_merge"] = merged
    Path(xodr_out).parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(xodr_out), xml_declaration=True, encoding="UTF-8")
    return report


def apply_with_mouths(xodr_in, xodr_out, params=None):
    """Lane refit on ordinary roads, then junction-mouth alignment on connectors.

    ``params["mouth_blend_kappa"]`` / ``["kappa_bound_scale"]`` / ``["mouth_blend_pick"]``: see
    mouth_frame_align.align_tree.
    ``params["edge_joins"]``: last, one-lane connectors get width slopes at their reference joins that keep their lane
    edges from jumping in curvature there, the lane centre unchanged (mapforge.ops.connector_edge_joins).
    """
    from mapforge.ops import mouth_frame_align
    params = dict(params or {})
    edge_joins = params.pop("edge_joins", False)
    align = {k: params.pop(k) for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided",
                                        "width_local_slopes", "match_end_curvature", "monotone_turns",
                                        "aligned_frame", "source_fit", "turn_end_zone") if k in params}
    tmp = Path(xodr_out).with_suffix(".lane-refit.xodr")
    first = apply(xodr_in, tmp, params)
    second = mouth_frame_align.apply(tmp, xodr_out, manifest=Path(xodr_in).with_suffix(".source-lanes.json"),
                                     **align)
    tmp.unlink()
    out = {"schema": CODE + "+" + mouth_frame_align.CODE, "lane_refit": first, "mouths": second,
           "rewritten": first["rewritten"] + second["rewritten"]}
    if edge_joins:
        from mapforge.ops import connector_edge_joins
        out["edge_joins"] = connector_edge_joins.apply_file(xodr_out)
    return out


# Registered scoreboard variants: corner mode x curvature target, always with mouth alignment.
# Two tolerance levels: A = fidelity first, B = smoothness first.
LEVEL_A = {"kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05}
LEVEL_B = {"kappa_cap": 0.02, "seg_min": 12.0, "rdp_tol": 0.10}
VARIANTS = {"g2-k04": {"mode": "g2", **LEVEL_A}, "g2-k02": {"mode": "g2", **LEVEL_B},
            "c1-k04": {"mode": "c1", **LEVEL_A}, "c1-k02": {"mode": "c1", **LEVEL_B},
            # T1 wrap-up: strong mouth flares on the outermost boundary become curb-return shoulders
            "g2-k04-curb": {"mode": "g2", **LEVEL_A, "curb_shoulder": True},
            # ... connectors carry lateral shifts at the mouths only near the mouths (C2 blend, 0.01 /m),
            # and the connector refit may keep the connector's own curvature (squeezed node18 connectors)
            "g2-k04-curb-local": {"mode": "g2", **LEVEL_A, "curb_shoulder": True,
                                  "mouth_blend_kappa": 0.01, "kappa_bound_scale": 1.25},
            # ... per connector, spread or localized, whichever lies closer to the source via centreline
            "g2-k04-curb-pick": {"mode": "g2", **LEVEL_A, "curb_shoulder": True,
                                 "mouth_blend_kappa": 0.01, "kappa_bound_scale": 1.25, "mouth_blend_pick": True},
            # ... plus a connector refit to the source centreline itself, where it meets both mouths
            "g2-k04-curb-guided": {"mode": "g2", **LEVEL_A, "curb_shoulder": True,
                                   "mouth_blend_kappa": 0.01, "kappa_bound_scale": 1.25, "mouth_blend_pick": True,
                                   "source_guided": True},
            # ... and every lane boundary C2: G2 end blends, G2 births/deaths, C2 repair bumps; lane centres
            # G2 across the mouths as well (match_end_curvature, added after the first default run); turning
            # connectors curve one way (monotone_turns, 2026-10-04); references may end on the linked lane
            # centres so mouth shifts do not tilt the lane offset along the connector (aligned_frame, 2026-10-04);
            # MAP road sides refitted to the MAP lane point lists (map_refit, 2026-10-04); connectors may follow their
            # source via line with fidelity-first chains, picked by source distance first (source_fit, 2026-10-04);
            # SHP lane births/deaths moved half a corner ahead of their source events, short runs kept where their
            # corners fit the curvature target, vertex values by least squares (birth_advance, short_kappa,
            # lsq_refine, 2026-10-04 boundary step; SHP only); gap lanes between parallel source links (link_gaps,
            # 2026-10-05; SHP only); turning SHP connectors may steer back gently next to the mouths
            # (turn_end_zone: connector_source_fit.END_ZONE, user decision 2026-10-05); mirrored MAP departure
            # sides eased where they bend (departure_ease, 2026-10-07; MAP only), and before that the MAP approach
            # sides faired within 5 cm of their point lists (approach_fair, 2026-10-07; MAP only); one-lane connectors
            # width slopes at reference joins chosen so their edges barely jump in curvature (edge_joins, 2026-10-07); a source corner
            # dropped from a junction-mouth zone leaves a vertex on the data at the zone edge (mouth_anchor, 2026-10-09;
            # a flare zone keeps its mouth slope, 2026-10-10)
            "g2-k04-c2": {"mode": "g2", **LEVEL_A, "curb_shoulder": True, "c2_ends": True, "merge_short_ref": True,
                          "rechain": True, "birth_advance": True, "short_kappa": True, "lsq_refine": True,
                          "link_gaps": True, "approach_fair": True, "departure_ease": True, "edge_joins": True,
                          "mouth_blend_kappa": 0.01, "kappa_bound_scale": 1.25, "mouth_blend_pick": True,
                          "source_guided": True, "width_local_slopes": True, "match_end_curvature": True,
                          "monotone_turns": True, "aligned_frame": True, "map_refit": True, "source_fit": True,
                          "turn_end_zone": True, "mouth_anchor": True}}


def _variant(name):
    def run(xodr_in, xodr_out):
        return apply_with_mouths(xodr_in, xodr_out, VARIANTS[name])
    run.__name__ = "apply_" + name.replace("-", "_")
    return run


apply_g2_k04 = _variant("g2-k04")
apply_g2_k02 = _variant("g2-k02")
apply_c1_k04 = _variant("c1-k04")
apply_c1_k02 = _variant("c1-k02")
apply_g2_k04_curb = _variant("g2-k04-curb")
apply_g2_k04_curb_local = _variant("g2-k04-curb-local")
apply_g2_k04_curb_pick = _variant("g2-k04-curb-pick")
apply_g2_k04_curb_guided = _variant("g2-k04-curb-guided")
apply_g2_k04_c2 = _variant("g2-k04-c2")
