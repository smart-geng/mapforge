"""Guard against a vertex pruning that throws an SHP lane boundary far off its source (2026-10-13).

fit_boundary drops vertices until every run between them is at least SEG_MIN long: each time the shortest run loses
the less important of its two vertices, never a protected one (an advanced birth or death, lane_birth_advance) nor a
chain end. Where a short run ends at such a fixed vertex, its other vertex goes whatever the profile does there.
Generalization 0711 road 11, right lane -2/-3 (source lanes 92872 / 60016), fitted as width on its inner boundary up to
an advanced death at s 64.53: the source width is 2.82-2.85 m from s 6 to 60 and closes over the last 4.5 m, and it
opens over the first 5 m of the free road start. The run into the protected death vertex lost the vertex at s 60, the
start run lost the one at s 5 (its free-end corner at 0.104 /m, just over FREE_END_KAPPA_MAX), and the least-squares
vertex values made the width one straight line from 3.9 m at s 0 to zero at s 64.5: the outer edge 2.1 m off its
source at s 52 and 3.4-3.7 m at the road start. The lane born next to it at s 60.7 (2.8 m wide 3.7 m later) became a
17 m ramp, 2.3 m off.

guard: where the pruned vertex polyline lies more than GUARD_M off the vertices it was pruned from, the pruning is done
again with the same rules for which short runs stay, but each short run is resolved by whichever of dropping one of its
free vertices or moving one away from the other end to SEG_MIN (value from the profile there) keeps the polyline
closest to those vertices. Used only when that lies closer; runs still end up at least SEG_MIN long. A free-end run
moved out to SEG_MIN rounds its corner over all of it, as a kept free-end run does, when that corner then fits at
FREE_END_KAPPA_MAX (fit_boundary).
"""
from __future__ import annotations

import numpy as np

from mapforge.ops import lane_refit as LR

GUARD_M = 0.5        # a pruned polyline this far off its RDP vertices is redone


def error(ref_s, ref_t, vs, vt):
    """Largest distance (in t) between the polyline (vs, vt) and the polyline of the RDP vertices (ref_s, ref_t), at
    the union of their vertices, where the largest difference of two polylines lies."""
    at = np.union1d(ref_s, vs)
    return float(np.max(np.abs(np.interp(at, vs, vt) - np.interp(at, ref_s, ref_t))))


def _options(vs, vt, k, grid, vals, seg_min, protected):
    """(vs, vt) after each way of resolving the short run k: drop a free vertex of it, or move one away from the other
    end to seg_min from it, inside its outer neighbour."""
    n, out = len(vs), []
    for i in (k, k + 1):
        if 0 < i < n - 1 and vs[i] not in protected:
            out.append((vs[:i] + vs[i + 1:], vt[:i] + vt[i + 1:]))
    for i, j in ((k, k + 1), (k + 1, k)):
        if not 0 < i < n - 1 or vs[i] in protected:
            continue
        x = vs[j] - seg_min if i < j else vs[j] + seg_min
        if (i < j and x - vs[i - 1] < LR.GRID) or (i > j and vs[i + 1] - x < LR.GRID):
            continue
        out.append((vs[:i] + [x] + vs[i + 1:], vt[:i] + [float(np.interp(x, grid, vals))] + vt[i + 1:]))
    return out


def prune(ref_s, ref_t, grid, vals, seg_min, protected=(), keep_first=False, keep_last=False, protected_runs=False,
          short_kappa=None):
    """lane_refit._prune with error-aware choices (see the module doc); None where it does not settle within its step
    budget (a move can undo another)."""
    vs, vt = [float(x) for x in ref_s], [float(x) for x in ref_t]
    for _ in range(4 * len(vs) + 4):
        if len(vs) <= 2:
            break
        lengths = np.diff(vs)
        options = None
        for k in np.argsort(lengths, kind="stable"):
            if lengths[k] >= seg_min:
                break
            if (keep_first and k == 0) or (keep_last and k == len(vs) - 2):
                continue
            if protected_runs and LR._keep_protected_run(vs, vt, k, protected):
                continue
            if short_kappa and LR._short_run_fits(vs, vt, k, short_kappa):
                continue
            options = _options(vs, vt, int(k), grid, vals, seg_min, protected)
            if options:
                break
        if not options:
            break
        vs, vt = min(options, key=lambda o: error(ref_s, ref_t, o[0], o[1]))
    else:
        return None
    return np.asarray(vs), np.asarray(vt)


def guard(ref_s, ref_t, vs, vt, grid, vals, seg_min, protected, keep_first, keep_last, protected_runs, short_kappa,
          info=None):
    """(vs, vt): the pruned polyline, or the guarded pruning's where the first lies more than GUARD_M off the RDP
    vertices (ref_s, ref_t) and the guarded one closer. ``info`` receives the errors when the guard was tried."""
    ref_s, ref_t = np.asarray(ref_s, float), np.asarray(ref_t, float)
    first = error(ref_s, ref_t, vs, vt)
    if first <= GUARD_M:
        return vs, vt
    guarded = prune(ref_s, ref_t, grid, vals, seg_min, protected, keep_first, keep_last, protected_runs, short_kappa)
    second = None if guarded is None else error(ref_s, ref_t, *guarded)
    used = second is not None and second < first
    if info is not None:
        info.update({"pruned_error_m": round(first, 3), "guarded_error_m": None if second is None else round(second, 3),
                     "used": used})
    return guarded if used else (vs, vt)
