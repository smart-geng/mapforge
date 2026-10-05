"""Align connecting-road cross-sections with the linked roads at junction mouths (Phase 2, step 0b).

Hermite edges alone (mouth_edge_match) leave an along-track stagger of up to 12 cm:
the linked road's end cross-section is normal to *its* reference line, the
connector's to the lane centre, and the two differ by 2-5 deg where lanes flare at
the mouth. Here the connector reference line is refitted with the existing
few-segment G2 connector fitter (refline_fit.fit_connector_minimal: <=5 clothoids,
>=3 m each) between the same lane-centre end points, but with the linked roads'
reference headings, so both cross-sections coincide. The lane-centre heading
difference is carried by a cubic lane offset; the fitter's target is the old
lane-centre path minus that offset, so the lane centre stays on the old path.
Connectors whose refit misses the tolerance are reported and left unchanged.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np
from lxml import etree

from mapforge.ops.mouth_edge_match import (
    WIDTH_OVERSHOOT_TOL_M, _fmt, _frame, _hermite, _local, _poly, _replace, _supported, _targets, edge_offsets,
)

CODE = "mapforge.mouth_frame_align/v1"
MAX_TILT_DEG = 15.0
# Reference vs target (old lane centre minus the cubic offset). 0.15 m = the draft T2 lane-centre P95;
# every connector's actual lane-centre shift is reported, never hidden.
FIT_TOL = {"endpoint_tol": 0.02, "median_tol": 0.03, "p95_tol": 0.08, "max_dev_tol": 0.15}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _travel_frame(road, contact, forward):
    x, y, h, k = _frame(road, contact)
    return (x, y, h, k) if forward else (x, y, h + math.pi, -k)


def _records(parent, tag, key):
    return sorted((float(e.get(key)), *(float(e.get(c)) for c in "abcd")) for e in parent.findall(tag))


def _piecewise(rows, s, order=0):
    """Value or derivative of OpenDRIVE piecewise cubic records at ``s``."""
    row = rows[0]
    for r in rows:
        if r[0] <= s + 1e-12:
            row = r
    a, b, c, d = row[1:]
    u = s - row[0]
    return (a + u * (b + u * (c + u * d)), b + u * (2 * c + 3 * d * u), 2 * c + 6 * d * u)[order]


def _offset_path(pts, ss, hh, offset):
    t = np.array([offset(s) for s in ss])
    return np.column_stack([pts[:, 0] - t * np.sin(hh), pts[:, 1] + t * np.cos(hh)])


def _to_polyline(points, line):
    """Distance from each point to the polyline (segments, not vertices)."""
    a, b = line[:-1], line[1:]
    ab = b - a
    t = ((points[:, None, :] - a[None]) * ab[None]).sum(axis=2) / np.maximum((ab * ab).sum(axis=1), 1e-18)[None]
    t = np.clip(t, 0.0, 1.0)
    proj = a[None] + t[..., None] * ab[None]
    return np.sqrt(((points[:, None, :] - proj) ** 2).sum(axis=2)).min(axis=1)


def _max_gap(a, b):
    """Symmetric Hausdorff distance between two sampled curves (point-to-polyline)."""
    return float(max(_to_polyline(a, b).max(), _to_polyline(b, a).max()))


def _station_on(path, ss, point):
    """Arc-length station of ``point`` projected onto a sampled lane-centre path."""
    a, b = path[:-1], path[1:]
    ab = b - a
    t = np.clip(((point - a) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-18), 0.0, 1.0)
    d = np.linalg.norm(a + t[:, None] * ab - point, axis=1)
    i = int(np.argmin(d))
    return float(ss[i] + t[i] * (ss[i + 1] - ss[i])), float(d[i])


def _remap_support_s(lane, old_path, old_ss, new_path, new_ss):
    """Move s-based provenance (``support_s``) from the old to the new reference parameter."""
    el = lane.find("userData[@code='mapforge.provenance/v1']")
    if el is None:
        return None
    record = json.loads(el.get("value"))
    if "support_s" not in record:
        return None
    old = [float(s) for s in record["support_s"]]
    new = []
    for s in old:
        point = np.array([np.interp(s, old_ss, old_path[:, 0]), np.interp(s, old_ss, old_path[:, 1])])
        new.append(_station_on(new_path, new_ss, point)[0])
    record["support_s"] = new
    el.set("value", json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
    return {"old": old, "new": new}


def _end_pose(prims):
    from pyclothoids import Clothoid
    _, x, y, h, length, k0, k1 = prims[-1]
    c = Clothoid.StandardParams(x, y, h, k0, (k1 - k0) / length, length)
    return c.XEnd, c.YEnd, c.ThetaEnd


def _write_planview(cr, prims):
    plan = cr.find("planView")
    for g in plan.findall("geometry"):
        plan.remove(g)
    s0 = 0.0
    for kind, x, y, h, length, ka, kb in prims:
        g = etree.SubElement(plan, "geometry", s=_fmt(s0), x=_fmt(x), y=_fmt(y), hdg=_fmt(h), length=_fmt(length))
        if kind == "line":
            etree.SubElement(g, "line")
        elif kind == "arc":
            etree.SubElement(g, "arc", curvature=_fmt(ka))
        else:
            etree.SubElement(g, "spiral", curvStart=_fmt(ka), curvEnd=_fmt(kb))
        s0 += length
    cr.set("length", _fmt(s0))
    return s0


def _parallel_curvature(frame, point):
    """Curvature of the curve parallel to ``frame``'s reference through ``point``."""
    x, y, h, k = frame
    t = -(point[0] - x) * math.sin(h) + (point[1] - y) * math.cos(h)
    return k / (1 - k * t)


# Turning connectors (|turn| >= 30 deg) curve one way: with ``turn_sign`` the interior knot curvatures keep the
# turn's sign (TURN_SIGN_TOL of the other allowed). The fits followed digitized via kinks, or split a turn into
# two peaks with an opposite lobe between them (shp-node17 roads 106/107/123: +0.04-0.065 /m inside right turns).
TURN_MIN_RAD = math.radians(30.0)
TURN_SIGN_TOL = 0.005
# Pick: lane-centre curvature against the turn above this counts as a steering reversal.
WIGGLE_MAX = 0.01
# A turn-sign bounded refit may leave the old (reversing) path by up to this; the G8 ceilings judge it.
MONOTONE_DEV_MAX = 1.5


def _turn_sign(p0, p1):
    turn = _wrap(p1[2] - p0[2])
    return (1 if turn > 0 else -1) if abs(turn) >= TURN_MIN_RAD else 0


def _counter_knots(prims, sign):
    """Largest knot curvature against ``sign`` (0 if none or no turn)."""
    if not sign:
        return 0.0
    knots = [p[5] for p in prims] + [prims[-1][6]]
    return max(0.0, max(-sign * k for k in knots))


def _structure_refit(old_geoms, p0, p1, k0, k1, tc, length_old, target, kappa_bound=0.3, turn_sign=0):
    """Same primitive count and near-same lengths as the old connector; least squares to ``target``.

    Seed: old knot curvatures minus the offset's second derivative (first-order
    curvature of the target), so no segment is added or shortened below 3 m.
    ``turn_sign``: +1/-1 keeps every interior knot curvature on that side (TURN_SIGN_TOL of the other).
    """
    from scipy.optimize import least_squares
    from mapforge.ops.refline_fit import _chain_from_knots, _planview_at_s, _resample_count, planview_prims

    lengths = np.array([g[4] for g in old_geoms], float)
    knots = np.array([g[5] for g in old_geoms] + [old_geoms[-1][6]], float)
    s_knots = np.concatenate([[0.0], np.cumsum(lengths)])
    knots = knots - np.array([2 * tc[2] + 6 * tc[3] * s for s in s_knots])
    knots[0], knots[-1] = k0, k1
    n = len(lengths)
    # Correspond by the target's own arc length, not the old reference parameter.
    target = _resample_count(np.asarray(target, float), max(41, int(length_old / 0.5) + 1))
    frac = np.linspace(0.0, 1.0, len(target))

    def unpack(z):
        return z[:n], np.concatenate([[k0], z[n:], [k1]])

    def residual(z):
        lens, ks = unpack(z)
        pv = _chain_from_knots(p0, lens, ks)
        _, (xe, ye, he) = planview_prims(pv)
        end = 1e3 * np.array([xe - p1[0], ye - p1[1], _wrap(he - p1[2])])
        return np.concatenate([end, (_planview_at_s(pv, frac * lens.sum()) - target).ravel()])

    def end_error_vec(z):
        lens, ks = unpack(z)
        _, (xe, ye, he) = planview_prims(_chain_from_knots(p0, lens, ks))
        return np.array([xe - p1[0], ye - p1[1], _wrap(he - p1[2])])

    def fit_error(z):
        lens, ks = unpack(z)
        d = _planview_at_s(_chain_from_knots(p0, lens, ks), frac * lens.sum()) - target
        return float((d * d).sum()) / len(target)

    from scipy.optimize import minimize
    z0 = np.concatenate([lengths, knots[1:-1]])
    # Never shorter than the connector's own shortest primitive: no new short segments.
    knot_lo = -TURN_SIGN_TOL if turn_sign > 0 else -kappa_bound
    knot_hi = TURN_SIGN_TOL if turn_sign < 0 else kappa_bound
    lower = np.concatenate([np.full(n, min(lengths.min(), 6.0)), np.full(n - 1, knot_lo)])
    upper = np.concatenate([np.full(n, 3.0 * length_old), np.full(n - 1, knot_hi)])
    sol = least_squares(residual, np.clip(z0, lower + 1e-9, upper - 1e-9), bounds=(lower, upper),
                        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=400)
    # Exact end pose: equality-constrained polish (the penalty above only seeds it).
    polish = minimize(fit_error, sol.x, method="SLSQP", bounds=list(zip(lower, upper)),
                      constraints=[{"type": "eq", "fun": end_error_vec}],
                      options={"maxiter": 300, "ftol": 1e-14})
    best = polish.x if np.abs(end_error_vec(polish.x)).max() < np.abs(end_error_vec(sol.x)).max() else sol.x
    lens, ks = unpack(best)
    pv = _chain_from_knots(p0, lens, ks)
    prims, (xe, ye, he) = planview_prims(pv)
    end_error = max(math.hypot(xe - p1[0], ye - p1[1]), abs(_wrap(he - p1[2])))
    dense = _planview_at_s(pv, np.linspace(0.0, lens.sum(), max(81, int(lens.sum() / 0.25) + 1)))
    deviation = _max_gap(dense, target)
    return prims, end_error, deviation


SOURCE_TOL = {"endpoint_tol": 0.30, "median_tol": 0.05, "p95_tol": 0.15, "max_dev_tol": 0.30}


def load_via_centrelines(manifest_path):
    """Source lane centrelines (comparison CRS = XODR local frame) keyed by source lane id."""
    path = Path(manifest_path) if manifest_path else None
    if path is None or not path.exists():
        return {}
    manifest = json.loads(path.read_text(encoding="utf-8"))
    out = {}
    for lane in manifest.get("lanes", []):
        geom = (lane.get("geometry") or {}).get("coordinates")
        if lane.get("role") == "junction-via" and geom and len(geom) >= 2:
            out[lane["source_lane_id"]] = np.asarray(geom, float)[:, :2]
    return out


def _source_target(source, pts, ss, hh, tc, length):
    """Source via centreline points minus the cubic offset, inside the connector only."""
    a, b = pts[:-1], pts[1:]
    ab = b - a
    l2 = np.maximum((ab * ab).sum(axis=1), 1e-18)
    out, stations = [], []
    for p in source:
        u = ((p - a) * ab).sum(axis=1) / l2
        uc = np.clip(u, 0.0, 1.0)
        q = a + uc[:, None] * ab
        i = int(np.argmin(np.linalg.norm(p - q, axis=1)))
        if (i == 0 and u[i] < 0) or (i == len(a) - 1 and u[i] > 1):
            continue
        s = ss[i] + uc[i] * (ss[i + 1] - ss[i])
        h = hh[i]
        shift = _poly(tc, s)
        out.append((p[0] + shift * math.sin(h), p[1] - shift * math.cos(h)))
        stations.append(s)
    if len(out) < 5:
        return None, False
    covers = min(stations) < 0.5 and max(stations) > length - 0.5
    return np.asarray(out), covers


# Pulling connectors towards the source via centre was tried (2026-10-03): most refits then missed the
# tolerance because source via centrelines do not meet the mouths. Off by default; the guard stays.
USE_SOURCE_BLEND = False
SOURCE_BLEND_END_M = 10.0
# Guard: alignment must not push a via lane over the active G8 ceilings (0.05 m safety margin,
# because this check samples differently from the G8 evaluator).
G8_POLICY = Path(__file__).resolve().parents[2] / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"
GUARD_MARGIN_M = 0.05


def _via_limits():
    import yaml
    cls = yaml.safe_load(G8_POLICY.read_text(encoding="utf-8"))["classes"]["shp.field-via"]["source_to_target"]
    return cls["median_max_m"] - GUARD_MARGIN_M, cls["p95_max_m"] - GUARD_MARGIN_M


def _source_profile(source, pts, ss, hh):
    """Lateral offset t_src(s) of the source centreline along the old reference (None if unusable)."""
    a, b = pts[:-1], pts[1:]
    ab = b - a
    l2 = np.maximum((ab * ab).sum(axis=1), 1e-18)
    rows = []
    for p in source:
        u = ((p - a) * ab).sum(axis=1) / l2
        uc = np.clip(u, 0.0, 1.0)
        q = a + uc[:, None] * ab
        i = int(np.argmin(np.linalg.norm(p - q, axis=1)))
        if (i == 0 and u[i] < 0) or (i == len(a) - 1 and u[i] > 1):
            continue
        h = hh[i]
        rows.append((ss[i] + uc[i] * (ss[i + 1] - ss[i]), -(p[0] - q[i, 0]) * math.sin(h) + (p[1] - q[i, 1]) * math.cos(h)))
    if len(rows) < 5:
        return None
    rows = np.asarray(sorted(rows))
    return rows


# G8 field-via / MAP lane classes: both directions, coverage within 1.5 m on both sides >= 0.95.
COVERAGE_RADIUS_M = 1.5
COVERAGE_MIN = 0.95


def _resample_1m(poly):
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    q = np.linspace(0.0, s[-1], max(2, int(s[-1]) + 1))
    return np.column_stack([np.interp(q, s, poly[:, 0]), np.interp(q, s, poly[:, 1])])


def _source_fidelity(source, path):
    """Median and P95 distance from the source centreline points to a lane-centre path.

    Also, on 1 m samples as the G8 evaluator does: the reverse direction (path to source) and the smaller
    of both coverages within COVERAGE_RADIUS_M less the guard margin. The pick checked one direction only
    until node3 road 113 (1.4966 m from its source at worst) left G8 coverage after a 1 cm change.
    """
    if source is None or len(source) == 0:
        return None
    src = np.asarray(source, float)
    d = _to_polyline(src, path)
    forward, reverse = _to_polyline(_resample_1m(src), path), _to_polyline(_resample_1m(path), src)
    radius = COVERAGE_RADIUS_M - GUARD_MARGIN_M
    return {"median_m": float(np.median(d)), "p95_m": float(np.percentile(d, 95)),
            "reverse_median_m": float(np.median(reverse)), "reverse_p95_m": float(np.percentile(reverse, 95)),
            "coverage": float(min((forward <= radius).mean(), (reverse <= radius).mean()))}


# --- localized mouth blend: piecewise cubics (s0, s1, a, b, c, d), local u = s - s0 -------------

def _pc_eval(pieces, s, order=0):
    piece = pieces[-1]
    for p in pieces:
        if p[0] - 1e-9 <= s <= p[1] + 1e-9:
            piece = p
            break
    _, _, a, b, c, d = piece
    u = s - piece[0]
    return (a + u * (b + u * (c + u * d)), b + u * (2 * c + 3 * d * u), 2 * c + 6 * d * u)[order]


def _blend_down(d):
    """C2 blend from 1 at s=0 to 0 at s=d (zero slope and curvature at both ends).

    Three cubics with B'' = 0 -> -8/d^2 -> +8/d^2 -> 0, so a lateral shift delta adds at most
    8*delta/d^2 of curvature.
    """
    beta, q = 8.0 / d ** 2, d / 4.0
    p1 = (0.0, q, 1.0, 0.0, 0.0, -beta / (6 * q))
    a2, b2, c2 = 1.0 - beta * q * q / 6, -beta * q / 2, -beta / 2
    p2 = (q, 3 * q, a2, b2, c2, beta / (6 * q))
    u = 2 * q
    a3 = a2 + b2 * u + c2 * u * u + beta / (6 * q) * u ** 3
    b3 = b2 + 2 * c2 * u + beta / (2 * q) * u * u
    p3 = (3 * q, d, a3, b3, beta / 2, -beta / (6 * q))
    return [p1, p2, p3]


def _mirror(pieces, length):
    """f(length - s) as pieces in s."""
    out = []
    for s0, s1, a, b, c, d in reversed(pieces):
        h = s1 - s0
        out.append((length - s1, length - s0, a + h * (b + h * (c + h * d)), -(b + h * (2 * c + 3 * d * h)),
                    c + 3 * d * h, -d))
    return out


def _sum_pieces(components, length):
    """Sum of piecewise cubics (each zero outside its own support) on [0, length]."""
    knots = sorted({0.0, length} | {p[0] for comp in components for p in comp if 0 < p[0] < length}
                   | {p[1] for comp in components for p in comp if 0 < p[1] < length})
    out = []
    for lo, hi in zip(knots[:-1], knots[1:]):
        mid = (lo + hi) / 2
        coef = np.zeros(4)
        for comp in components:
            for p in comp:
                if p[0] - 1e-9 <= mid <= p[1] + 1e-9:
                    u = lo - p[0]
                    a, b, c, d = p[2:]
                    coef += (a + u * (b + u * (c + u * d)), b + u * (2 * c + 3 * d * u), c + 3 * d * u, d)
                    break
        out.append((lo, hi, *coef))
    return out


def _localized_offsets(left, right, length, kappa):
    """laneOffset pieces whose lane centre reaches the mouths' shifts only near each mouth.

    The single Hermite edge cubic carries a lateral shift at a mouth (linked road lane centre
    moved by a road-side refit or a curb-return shoulder) along the whole connector and drags
    its middle off the source. Here the centre is the slope-only cubic (zero shift, same end
    headings) plus a C2 blend of each end shift over d = sqrt(8|shift|/kappa) (<= length/2);
    the width stays the single cubic, so edges still meet both mouths in value and slope.
    """
    width = tuple(x - y for x, y in zip(left, right))
    centre = tuple((x + y) / 2 for x, y in zip(left, right))
    shift0 = _poly(centre, 0.0)
    shift1 = _poly(centre, length)
    slope0 = centre[1]
    slope1 = centre[1] + 2 * centre[2] * length + 3 * centre[3] * length ** 2
    base = _hermite(0.0, slope0, 0.0, slope1, length)
    comps = [[(0.0, length, *base)], [(0.0, length, width[0] / 2, width[1] / 2, width[2] / 2, width[3] / 2)]]
    lens = [min(math.sqrt(8 * abs(x) / kappa), length / 2) if abs(x) > 1e-4 else 0.0 for x in (shift0, shift1)]
    if lens[0] > 0:
        comps.append([(p[0], p[1], *(shift0 * v for v in p[2:])) for p in _blend_down(lens[0])])
    if lens[1] > 0:
        comps.append([(p[0], p[1], *(shift1 * v for v in p[2:])) for p in _mirror(_blend_down(lens[1]), length)])
    offset = _sum_pieces(comps, length)
    centre_pieces = [(p[0], p[1], *(o - w / 2 for o, w in zip(p[2:], _rebase_w(width, p[0])))) for p in offset]
    info = {"start_shift_m": shift0, "end_shift_m": shift1, "start_blend_m": lens[0], "end_blend_m": lens[1],
            "kappa": kappa, "offset_records": len(offset)}
    return offset, centre_pieces, width, info


def _rebase_w(width, s0):
    """Width cubic re-expanded at s0 (a, b, c, d in local u = s - s0)."""
    a, b, c, d = width
    return (a + s0 * (b + s0 * (c + s0 * d)), b + s0 * (2 * c + 3 * d * s0), c + 3 * d * s0, d)


def _scaled(pieces, k):
    return [(p[0], p[1], *(k * v for v in p[2:])) for p in pieces]


BLEND_SNAP_M = 0.5


def _localized_from(centre, width_pieces, length, kappa, snap=None):
    """_localized_offsets for a centre cubic and piecewise width: (offset, centre pieces, width pieces, info).

    ``snap``: per-end transition lengths shared with the width and curvature corrections. A blend shorter
    than END_CURVATURE_MIN_BLEND_M (its shift is below 1.1 cm, so the lane centre moves less than that), or
    within BLEND_SNAP_M of the shared length, takes the shared length (where shortened, curvature at most
    (1 + 0.5/3)^2 of ``kappa``); other blends keep their length. Lengthening longer blends moved a connector
    lane centre by centimetres (node3 road 113 out of G8 coverage, 2026-10-04).
    """
    shift0, shift1 = _poly(centre, 0.0), _poly(centre, length)
    slope0 = centre[1]
    slope1 = centre[1] + 2 * centre[2] * length + 3 * centre[3] * length ** 2
    comps = [[(0.0, length, *_hermite(0.0, slope0, 0.0, slope1, length))]]
    lens = [min(math.sqrt(8 * abs(x) / kappa), length / 2) if abs(x) > 1e-4 else 0.0 for x in (shift0, shift1)]
    if snap:
        lens = [s if (0.0 < d < END_CURVATURE_MIN_BLEND_M or abs(d - s) < BLEND_SNAP_M) else d
                for d, s in zip(lens, snap)]
    if lens[0] > 0:
        comps.append(_scaled(_blend_down(lens[0]), shift0))
    if lens[1] > 0:
        comps.append(_scaled(_mirror(_blend_down(lens[1]), length), shift1))
    centre_pieces = _sum_pieces(comps, length)
    offset = _sum_pieces([centre_pieces, _scaled(width_pieces, 0.5)], length)
    info = {"start_shift_m": shift0, "end_shift_m": shift1, "start_blend_m": lens[0], "end_blend_m": lens[1],
            "kappa": kappa, "offset_records": len(offset)}
    return offset, centre_pieces, width_pieces, info


# Unit end-slope correction (value 0 -> 0, slope 1 -> 0, zero curvature at both ends) over unit length:
# largest value 0.2056 (so a slope correction g over d bulges at most 0.2056 |g| d), peak t'' 16/3.
SLOPE_CORRECTION_MAX = 0.2056


def _width_local_slopes(local, length, budget):
    """Width that meets both mouths in value and slope without bulging more than ``budget``.

    The single width cubic of edge_offsets bulges where the end slopes are large against the width
    change; its slopes were then limited (edge heading kink up to ~0.4 deg). Here the width is a
    monotone cubic smoothstep between the end widths plus a C2 end-slope correction confined near
    each mouth: d = min(length / 2, budget / (0.2056 |g|)), so the bulge stays within the budget and
    both edge headings match exactly. Returns (centre cubic, width pieces, info).
    """
    from mapforge.ops.lane_refit import _g2_transition
    (l0, dl0, _), (r0, dr0, _) = local[("start", "left")], local[("start", "right")]
    (l1, dl1, _), (r1, dr1, _) = local[("end", "left")], local[("end", "right")]
    centre = _hermite((l0 + r0) / 2, (dl0 + dr0) / 2, (l1 + r1) / 2, (dl1 + dr1) / 2, length)
    w0, w1, g0, g1 = l0 - r0, l1 - r1, dl0 - dr0, dl1 - dr1
    comps = [[(0.0, length, w0, 0.0, 3 * (w1 - w0) / length ** 2, -2 * (w1 - w0) / length ** 3)]]
    lens = []
    for g, at_start in ((g0, True), (g1, False)):
        if abs(g) < 1e-9:
            lens.append(0.0)
            continue
        d = min(length / 2, budget / (SLOPE_CORRECTION_MAX * abs(g)))
        unit = _g2_transition(0.0, d, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0))
        comps.append(_scaled(unit, g) if at_start else _scaled(_mirror(unit, length), -g))
        lens.append(d)
    pieces = _sum_pieces(comps, length)
    return centre, pieces, {"width_local_slopes": True, "width_slope_lengths_m": lens}


def _edge_model(local, length, width_local_slopes):
    """(centre cubic, width pieces, shape) of a connector: edge_offsets, or local slopes where it had to limit."""
    left, right, shape = edge_offsets(local, length)
    if width_local_slopes and shape["width_slope_limited"]:
        centre, width_pieces, info = _width_local_slopes(local, length, WIDTH_OVERSHOOT_TOL_M)
        return centre, width_pieces, {**shape, **info, "edge_heading_kink_deg": 0.0}
    centre = tuple((x + y) / 2 for x, y in zip(left, right))
    return centre, [(0.0, length, *(x - y for x, y in zip(left, right)))], shape


# Source-guided candidate: with mouths moved before the real lane ends (shp_mouth_envelope), the via
# source is the real incoming-tail -> via -> outgoing-head chain and meets both mouths, so the connector
# can be refitted to the source itself between the linked roads' current lane centres.
SOURCE_MOUTH_TOL_M = 1.0   # the source must pass this close to both road lane centres
SOURCE_GUIDE_END_M = 10.0  # its ends are blended onto the lane centres within this distance
# The converter's own preferred source tolerance. A tighter one pulls the fit onto the abrupt curvature
# onset of digitized source arcs with short (3 m) transition clothoids; vehicles need the longer ones.
SOURCE_GUIDE_TOL = {"endpoint_tol": 0.02, "median_tol": 0.35, "p95_tol": 0.75, "max_dev_tol": 1.50}
# Shortest primitive: the converter's 6 m first; only if no 6 m chain can follow the source (tight
# turns, e.g. node16 road 103: 98.5 deg in 22.8 m), down to 3 m. Still <= 5 G2 clothoids with the same
# curvature and sharpness caps: a few-segment curve, not a dense chain.
SOURCE_GUIDE_MIN_SEGMENT_M = (6.0, 3.0)


def _tilt_cubic(targets, fa, fc, p0, p1):
    """(c0, m0, c1, m1): values and slopes at both ends of the lane-centre offset cubic that _edge_model gives a
    reference from ``p0`` with the start road's heading to ``p1`` with the end road's heading.

    The reference keeps the road cross-section heading at a mouth (so its lane edges meet the linked lane edges
    exactly); the offset's slope turns that into the lane-centre heading. A reference fitted to the source alone
    leaves the written lane centre bowed off the source by that cubic, up to 0.68 m on a 47 m connector with a
    5.5 deg tilt (shp-node13 road 108); a structure-preserving refit already fits reference + cubic."""
    f0 = (p0[0], p0[1], fa[2], _parallel_curvature(fa, p0))
    f1 = (p1[0], p1[1], fc[2], _parallel_curvature(fc, p1))
    (l0, dl0, _), (r0, dr0, _) = _local(f0, targets["start"]["left"]), _local(f0, targets["start"]["right"])
    (l1, dl1, _), (r1, dr1, _) = _local(f1, targets["end"]["left"]), _local(f1, targets["end"]["right"])
    return (l0 + r0) / 2, (dl0 + dr0) / 2, (l1 + r1) / 2, (dl1 + dr1) / 2


def _without_tilt(path, tilt):
    """``path`` moved by minus the tilt cubic (over the path length) along its own normals."""
    from mapforge.ops.connector_source_fit import _lateral
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    tangent = np.gradient(path, s, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    c = _lateral(s, s[-1], tilt)[0]
    return path - c[:, None] * np.column_stack([-tangent[:, 1], tangent[:, 0]])


def _source_guided(source, targets, fa, fc, old_min, monotone=False):
    """Few-segment G2 refits of the connector to its source centreline: [(prims, local, info)] or None.

    The target is the source between its points nearest to the two road lane centres, with the
    small end gaps blended away (cubic smoothstep over SOURCE_GUIDE_END_M); both ends carry the
    linked roads' headings and parallel curvatures, as in the old-path refit.
    ``monotone``: in a turn, a fit with knot curvatures against the turn is also re-polished with the
    same primitives under the turn-sign bounds; that version is a second fit if it still meets the
    source tolerance (the pick decides).
    """
    from mapforge.ops.refline_fit import fit_connector_minimal
    src = np.asarray(source, float)
    centre = {c: np.array([(targets[c]["left"]["x"] + targets[c]["right"]["x"]) / 2,
                           (targets[c]["left"]["y"] + targets[c]["right"]["y"]) / 2]) for c in ("start", "end")}
    i0 = int(np.argmin(np.linalg.norm(src - centre["start"], axis=1)))
    i1 = int(np.argmin(np.linalg.norm(src - centre["end"], axis=1)))
    gaps = [float(np.linalg.norm(src[i0] - centre["start"])), float(np.linalg.norm(src[i1] - centre["end"]))]
    if i1 <= i0 + 2 or max(gaps) > SOURCE_MOUTH_TOL_M:
        return None
    path = src[i0:i1 + 1]
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    if s[-1] < 2.0:
        return None
    q = np.linspace(0.0, s[-1], max(9, int(s[-1] / 0.5) + 1))
    path = np.column_stack([np.interp(q, s, path[:, 0]), np.interp(q, s, path[:, 1])])
    d = min(SOURCE_GUIDE_END_M, s[-1] / 3)
    x0 = np.clip(q / d, 0.0, 1.0)
    x1 = np.clip((s[-1] - q) / d, 0.0, 1.0)
    w0 = (1 - x0 * x0 * (3 - 2 * x0))[:, None]
    w1 = (1 - x1 * x1 * (3 - 2 * x1))[:, None]
    target = path + w0 * (centre["start"] - path[0]) + w1 * (centre["end"] - path[-1])
    p0 = (float(centre["start"][0]), float(centre["start"][1]), fa[2])
    p1 = (float(centre["end"][0]), float(centre["end"][1]), fc[2])
    # the reference is fitted to the target minus the mouth tilt cubic, so that reference + cubic follows it
    target = _without_tilt(target, _tilt_cubic(targets, fa, fc, p0, p1))
    fit = None
    for floor in SOURCE_GUIDE_MIN_SEGMENT_M:
        fit = fit_connector_minimal(target, p0, p1, k0=_parallel_curvature(fa, p0), k1=_parallel_curvature(fc, p1),
                                    max_segments=5, max_kappa=0.25, sharpness_cap=0.20,
                                    min_segment_m=min(old_min, floor), **SOURCE_GUIDE_TOL)
        if fit is not None:
            break
    if fit is None:
        return None
    versions = [(fit.primitives, fit.method, False)]
    sign = _turn_sign(p0, p1) if monotone else 0
    if _counter_knots(fit.primitives, sign) > TURN_SIGN_TOL:
        k0, k1 = _parallel_curvature(fa, p0), _parallel_curvature(fc, p1)
        mono, end_error, deviation = _structure_refit(fit.primitives, p0, p1, k0, k1, (0.0, 0.0, 0.0, 0.0),
                                                      float(sum(p[4] for p in fit.primitives)), target, 0.25, sign)
        if end_error <= 1e-6 and deviation <= SOURCE_GUIDE_TOL["max_dev_tol"]:
            versions.append((mono, fit.method + "+monotone", True))
    out = []
    for prims, method, is_mono in versions:
        xe, ye, he = _end_pose(prims)
        start = (prims[0][1], prims[0][2], prims[0][3], prims[0][5])
        end = (xe, ye, he, prims[-1][6])
        try:
            local = {(c, side): _local(start if c == "start" else end, targets[c][side])
                     for c in ("start", "end") for side in ("left", "right")}
        except ValueError:
            continue
        out.append((prims, local, {"method": method, "source_end_gap_m": gaps, "primitives": len(prims),
                                   "min_primitive_m": min(p[4] for p in prims), "monotone": is_mono}))
    return out or None


# Source fit (source_fit): fidelity-first lane-centre chains to the source via line (connector_source_fit), only for
# connectors whose old lane centre lies farther than this from the via line (G8-like P95, both directions).
SOURCE_FIT_TRIGGER_M = 0.10
# Pick (source_fit): after the G8 ceilings and the smoothness flags, the smallest source distance (worst-direction
# P95) plus connector_source_fit.PRIMITIVE_COST_M per primitive: one more primitive must bring the lane centre
# that much closer to the source.


def _contact_rate(road, contact):
    """Reference curvature rate of a linked road at its contact point (the same value along either direction)."""
    from mapforge.validate.smoothness import _ref_kappa_at
    return _ref_kappa_at(road, 0.0 if contact == "start" else float(road.get("length")))[1]


def _source_fits(source, targets, fa, fc, rates, monotone=False, end_zone=None):
    """[(prims, info)] of fidelity-first chains from the start to the end road lane centre (connector_source_fit).

    ``rates``: the linked roads' reference curvature rates at the two contacts; the chain's end segments are
    pulled towards them (the mouth edge residual is width * width slope * rate difference / 4). The fit measures
    the lane centre the candidate will write: the chain plus the mouth tilt cubic (_tilt_cubic)."""
    from mapforge.ops import connector_source_fit
    centre = {c: ((targets[c]["left"]["x"] + targets[c]["right"]["x"]) / 2,
                  (targets[c]["left"]["y"] + targets[c]["right"]["y"]) / 2) for c in ("start", "end")}
    p0 = (centre["start"][0], centre["start"][1], fa[2])
    p1 = (centre["end"][0], centre["end"][1], fc[2])
    start = (*p0, _parallel_curvature(fa, p0))
    end = (*p1, _parallel_curvature(fc, p1))
    return connector_source_fit.candidates(source, start, end, _turn_sign(p0, p1) if monotone else 0, end_rates=rates,
                                           lateral=_tilt_cubic(targets, fa, fc, p0, p1), end_zone=end_zone)


# Below this shift at both mouths the aligned refit would only repeat the old-path refit: skipped.
ALIGN_MIN_SHIFT_M = 0.01
# The aligned refit follows the localized candidate's lane-centre path; it may leave that path by up to this
# (the draft T2 lane-centre maximum), the G8 ceilings in the pick judge it against the source.
ALIGN_DEV_MAX = 0.35


def _blend_weight(arc, d):
    """C2 weight 1 -> 0 over [0, d] along ``arc`` (the shape of _blend_down; 0 beyond d)."""
    if d <= 0:
        return np.zeros_like(arc)
    pieces = _blend_down(d)
    return np.array([_pc_eval(pieces, a) if a < d else 0.0 for a in arc])


def _aligned_refits(old_geoms, old_path, ss, hh, targets, fa, fc, length_old, bound, sign, old_min, kappa):
    """Reference refits whose ends sit on the linked lanes' centres: ([(prims, name)], info) (2026-10-04).

    The old-path refit starts and ends at the old lane centre; where the linked lanes moved (lane refit,
    mouths moved 3 m) the shift, up to 0.5 m, was then carried by the lane-centre offset over most of the
    connector, and an offset slope turns every reference sharpness change into edge curvature jumps (up to
    0.024 /m on the inner edge of tight turns). Here the old path's ends are moved onto the linked lane
    centres with the localized candidate's own blend (length sqrt(8 |shift| / kappa), same C2 shape), so the
    lane-centre path is that candidate's, but the G2 reference carries the shift and the offset keeps only
    the small tilt cubic. Structure-preserving refit first, the few-segment fitter if that misses FIT_TOL;
    with ``sign`` a turn-sign bounded version is added as well.
    """
    from mapforge.ops.refline_fit import fit_connector_minimal
    centre = {c: np.array([(targets[c]["left"]["x"] + targets[c]["right"]["x"]) / 2,
                           (targets[c]["left"]["y"] + targets[c]["right"]["y"]) / 2]) for c in ("start", "end")}
    heading = {c: targets[c]["left"]["heading"] + _wrap(targets[c]["right"]["heading"] - targets[c]["left"]["heading"]) / 2
               for c in ("start", "end")}
    beta0, beta1 = _wrap(heading["start"] - fa[2]), _wrap(heading["end"] - fc[2])
    if max(abs(beta0), abs(beta1)) > math.radians(MAX_TILT_DEG):
        return [], {"reason": "tilt too large"}
    tc = _hermite(0.0, math.tan(beta0), 0.0, math.tan(beta1), length_old)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(old_path, axis=0), axis=1))])
    shift0, shift1 = centre["start"] - old_path[0], centre["end"] - old_path[-1]
    if max(np.linalg.norm(shift0), np.linalg.norm(shift1)) < ALIGN_MIN_SHIFT_M:
        return [], {"reason": "no shift at the mouths"}
    d0, d1 = (min(math.sqrt(8 * float(np.linalg.norm(x)) / kappa), arc[-1] / 2) if np.linalg.norm(x) > 1e-4 else 0.0
              for x in (shift0, shift1))
    path = old_path + _blend_weight(arc, d0)[:, None] * shift0 + _blend_weight(arc[-1] - arc, d1)[:, None] * shift1
    t = np.array([_poly(tc, s) for s in ss])
    target = np.column_stack([path[:, 0] + t * np.sin(hh), path[:, 1] - t * np.cos(hh)])
    p0 = (float(centre["start"][0]), float(centre["start"][1]), fa[2])
    p1 = (float(centre["end"][0]), float(centre["end"][1]), fc[2])
    k0, k1 = _parallel_curvature(fa, p0), _parallel_curvature(fc, p1)
    info = {"shift_m": [float(np.linalg.norm(shift0)), float(np.linalg.norm(shift1))], "blend_m": [d0, d1]}
    prims, end_error, deviation = _structure_refit(old_geoms, p0, p1, k0, k1, tc, length_old, target, bound)
    info.update(method="structure-preserving", deviation_m=deviation)
    if end_error > 1e-6 or deviation > ALIGN_DEV_MAX:
        fit = fit_connector_minimal(target, p0, p1, k0=k0, k1=k1, max_segments=5, max_kappa=0.25,
                                    sharpness_cap=0.20, min_segment_m=min(old_min, 6.0),
                                    **{**FIT_TOL, "max_dev_tol": ALIGN_DEV_MAX})
        if fit is None:
            info["reason"] = f"refit missed the tolerance (structure-preserving max {deviation:.3f} m)"
            return [], info
        prims, info["method"] = fit.primitives, fit.method
    out = [(prims, "aligned")]
    if _counter_knots(prims, sign) > TURN_SIGN_TOL:
        mono, end_error, deviation = _structure_refit(prims, p0, p1, k0, k1, (0.0, 0.0, 0.0, 0.0),
                                                      float(sum(p[4] for p in prims)), target, bound, sign)
        if end_error <= 1e-6 and deviation <= MONOTONE_DEV_MAX:
            out.append((mono, "aligned-monotone"))
    return out, info


# T2: lane-centre curvature jump at written joins (draft threshold 1e-3 /m).
JOIN_JUMP_MAX = 1.0e-3
# Pick (aligned_frame): lane-edge curvature jumps at the connector's own joins (reported scoreboard metric),
# compared in steps of EDGE_JOIN_STEP after the 6 m segment rule: only a substantial difference outranks the
# source distance (1e-3 steps traded 0.6 m of fidelity for 1e-3 /m, shp-node13 road 107, 2026-10-04).
EDGE_JOIN_STEP = 5.0e-3

# Lane-centre G2 across the mouths (G11-C: 1e-7 /m). The refit gives the reference the parallel
# curvature of the linked road, but the linked lane centre's world curvature also carries its own
# lateral slope and second derivative, and the connector's centre/width cubics have theirs: the
# aligned connectors jumped by up to 0.01 /m at the mouths (2026-10-03, default-c2 run).
END_CURVATURE_BLEND_M = 10.0


def _linked_lanes(roads, cr, connection):
    """(road, lane id, contact, forward) of the lanes linked to connector lane -1 at its start and end."""
    lane = cr.find("lanes/laneSection/right/lane[@id='-1']")
    out = {}
    for role, contact in (("predecessor", "start"), ("successor", "end")):
        link = cr.find("link/" + role)
        if link is None or link.get("elementType") != "road":
            raise ValueError("missing endpoint road")
        oc = link.get("contactPoint")
        if role == "predecessor":
            pairs = [p for p in connection.findall("laneLink") if p.get("to") == "-1"]
            if len(pairs) != 1:
                raise ValueError("connector lane -1 needs exactly one incoming laneLink")
            out[contact] = (roads[link.get("elementId")], int(pairs[0].get("from")), oc, oc == "end")
        else:
            nxt = lane.find("link/successor")
            if nxt is None:
                raise ValueError("connector lane -1 has no successor lane")
            out[contact] = (roads[link.get("elementId")], int(nxt.get("id")), oc, oc == "start")
    return out


# With the lane centre exact, both edges keep the residual w * w' * (connector - linked reference sharpness) / 4
# where the linked lane tapers at the mouth: the cross-section is not consistent to second order across two
# reference lines whose sharpness differs, and laneOffset and width have only two degrees of freedom for three
# curves. Bounding the connector's end sharpness instead was tried (2026-10-04) and rejected: in a 25 m
# quarter turn a 10 % cut at both ends already moved the path 0.19 m, 25 % could not be fitted at all.
# Curvature corrections reuse the width's own end-slope transition where it is at least this long (no new
# knots, no near-duplicate knots); otherwise both laneOffset and width take END_CURVATURE_BLEND_M.
END_CURVATURE_MIN_BLEND_M = 3.0
# A width correction that would change both edges by no more than this is skipped where the width has no
# transition at that end (saves three width records; the edges keep that much residual).
WIDTH_CORRECTION_SKIP = 2.5e-4


def _end_blends(width_blends, length, centre_blends=None):
    """Transition length per end shared by the width, the centre blend and the curvature corrections.

    The width's own end-slope transition if at least END_CURVATURE_MIN_BLEND_M, else a centre blend that
    long, else END_CURVATURE_BLEND_M (at most half the connector; stretched to meet the other end where
    they would leave less than BLEND_SNAP_M between them).
    """
    out = []
    for own, blend in zip(width_blends or (0.0, 0.0), centre_blends or (0.0, 0.0)):
        if own >= END_CURVATURE_MIN_BLEND_M:
            out.append(own)
        elif blend >= END_CURVATURE_MIN_BLEND_M:
            out.append(min(blend, length / 2))
        else:
            out.append(None)
    free = [i for i, d in enumerate(out) if d is None]
    for i in free:
        out[i] = min(END_CURVATURE_BLEND_M, length / 2)
    gap = length - out[0] - out[1]
    if free and 0.0 < gap < BLEND_SNAP_M:
        for i in free:
            out[i] += gap / len(free)
    return tuple(out)


def _match_end_curvature(prims, offset_pieces, width_pieces, centre_targets, edge_targets, width_blends=None,
                         centre_blends=None):
    """laneOffset/width corrections so the lane centre meets both linked lanes' world curvature exactly.

    With position and heading fixed at a mouth, each lane curve's world curvature is linear in its
    lateral second derivative. The laneOffset change a and width change b there make the centre exact
    (a - b/2 = its need) and leave the two edges equal, smallest residuals (b = left need - right need).
    Each change is a G2 transition (three cubics) from (0, 0, t'') back to zero over the end's shared
    transition length (_end_blends of ``width_blends`` / ``centre_blends``: existing transition lengths at
    the start/end, 0 = none), so it adds no knot next to an existing one. Values and slopes at both
    mouths, and the connector beyond the blends, stay as they were.
    Returns (offset, centre, width pieces, info).
    """
    from mapforge.ops.lane_refit import _g2_transition
    from mapforge.validate.smoothness import _edge_world_curvature
    length = float(sum(p[4] for p in prims))
    ends = {"start": (0.0, prims[0][5], (prims[0][6] - prims[0][5]) / prims[0][4]),
            "end": (length, prims[-1][6], (prims[-1][6] - prims[-1][5]) / prims[-1][4])}
    comps_o, comps_w, info = [offset_pieces], [width_pieces], {}
    blends = _end_blends(width_blends, length, centre_blends)
    for i, (c, (u, kappa, sharp)) in enumerate(ends.items()):
        own = (width_blends or (0.0, 0.0))[i]
        d = blends[i]
        o = [_pc_eval(offset_pieces, u, n) for n in range(3)]
        w = [_pc_eval(width_pieces, u, n) for n in range(3)]
        curves = {"left": o, "right": [x - y for x, y in zip(o, w)], "centre": [x - y / 2 for x, y in zip(o, w)]}
        need = {}
        for side, (t, dt, ddt) in curves.items():
            target = (centre_targets[c] if side == "centre" else edge_targets[c][side])["curvature"]
            along = 1.0 - kappa * t
            need[side] = (target - _edge_world_curvature(t, dt, ddt, kappa, sharp)) * \
                (along * along + dt * dt) ** 1.5 / along
        b = need["left"] - need["right"]
        if own < END_CURVATURE_MIN_BLEND_M and abs(b) / 2 <= WIDTH_CORRECTION_SKIP:
            b = 0.0
        a = need["centre"] + b / 2
        for comps, value in ((comps_o, a), (comps_w, b)):
            if abs(value) > 1e-15:
                unit = _g2_transition(0.0, d, (0.0, 0.0, value), (0.0, 0.0, 0.0))
                comps.append(unit if c == "start" else _mirror(unit, length))
        # t'' each edge still lacks after the change (left, right)
        info[c] = {"blend_m": d, "centre_t2_change": need["centre"], "offset_t2_change": a, "width_t2_change": b,
                   "edge_t2_residual": [need["left"] - a, need["right"] - (a - b)]}
    offset = _sum_pieces(comps_o, length)
    width = _sum_pieces(comps_w, length)
    return offset, _sum_pieces([offset, _scaled(width, -0.5)], length), width, info


# Edge-residual trim (2026-10-05): with the lane centre exact at a mouth, both lane edges keep about
# w * w' * (connector - road reference curvature rate) / 4 there (_match_end_curvature: laneOffset and width give
# two second derivatives for three curves). Where the picked candidate still leaves more than JOIN_JUMP_MAX at a
# mouth, its chain is solved once more with that end segment's curvature rate moved towards the road's, by as much
# as brings the residual to TRIM_MARGIN of the bound (_end_rate_trim, more if that is not enough); the trimmed
# version is one more candidate under the same pick. shp-node17 road 107, a right turn from the flared approach
# lane onto the flared departure lane, kept 1.008e-3 at its end.
TRIM_MARGIN = 0.9
TRIM_STEPS = (1.0, 0.8, 0.6)     # the rate change is tried at these multiples of the linear estimate's scale
TRIM_DEV_MAX_M = 0.10            # the trimmed reference may leave the picked one by at most this


def _end_rate_trim(prims, rates, kappa_bound, turn_sign=0):
    """``prims`` (a clothoid chain) solved again with the curvature rate of its first segment equal to
    ``rates["start"]`` and/or of its last to ``rates["end"]``: the same start pose, end pose and end curvatures, the
    same segment count, no segment shorter than the old shortest one or 6 m (as _structure_refit), knot curvatures
    within ``kappa_bound`` and, with
    ``turn_sign``, on the turn's side; otherwise as close to the old chain as possible. Returns (prims, largest
    distance from the old chain [m]) or None."""
    from scipy.optimize import minimize
    from mapforge.ops.refline_fit import _chain_from_knots, _planview_at_s, planview_prims
    lengths = np.array([q[4] for q in prims], float)
    knots = np.array([q[5] for q in prims] + [prims[-1][6]], float)
    n = len(lengths)
    if n < 2 or (len(rates) == 2 and n < 3):
        return None
    p0 = (prims[0][1], prims[0][2], prims[0][3])
    p1 = _end_pose(prims)
    k0, k1 = knots[0], knots[-1]
    total = float(lengths.sum())
    frac = np.linspace(0.0, 1.0, max(41, int(total / 0.5) + 1))
    old = _planview_at_s(_chain_from_knots(p0, lengths, knots), frac * total)

    def chain(z):
        return z[:n], np.concatenate([[k0], z[n:], [k1]])

    def end_error(z):
        lens, ks = chain(z)
        _, (x, y, h) = planview_prims(_chain_from_knots(p0, lens, ks))
        return np.array([x - p1[0], y - p1[1], _wrap(h - p1[2])])

    def rate_error(z):
        lens, ks = chain(z)
        out = []
        if "start" in rates:
            out.append((ks[1] - ks[0]) / lens[0] - rates["start"])
        if "end" in rates:
            out.append((ks[-1] - ks[-2]) / lens[-1] - rates["end"])
        return np.array(out)

    def fit_error(z):
        lens, ks = chain(z)
        d = _planview_at_s(_chain_from_knots(p0, lens, ks), frac * lens.sum()) - old
        return float((d * d).sum()) / len(frac)

    knot_lo = -TURN_SIGN_TOL if turn_sign > 0 else -kappa_bound
    knot_hi = TURN_SIGN_TOL if turn_sign < 0 else kappa_bound
    lower = np.concatenate([np.full(n, min(lengths.min(), 6.0)), np.full(n - 1, knot_lo)])
    upper = np.concatenate([np.full(n, 3.0 * total), np.full(n - 1, knot_hi)])
    z0 = np.clip(np.concatenate([lengths, knots[1:-1]]), lower, upper)
    res = minimize(fit_error, z0, method="SLSQP", bounds=list(zip(lower, upper)),
                   constraints=[{"type": "eq", "fun": end_error}, {"type": "eq", "fun": rate_error}],
                   options={"maxiter": 300, "ftol": 1e-14})
    if np.abs(end_error(res.x)).max() > 1e-6 or np.abs(rate_error(res.x)).max() > 1e-9:
        return None
    lens, ks = chain(res.x)
    pv = _chain_from_knots(p0, lens, ks)
    new, _ = planview_prims(pv)
    return new, float(np.linalg.norm(_planview_at_s(pv, frac * lens.sum()) - old, axis=1).max())


def _write_lane_records(lanes, lane, offset_pieces, width_pieces):
    _replace(lanes, "laneOffset", [etree.Element(
        "laneOffset", s=_fmt(p[0]), a=_fmt(p[2]), b=_fmt(p[3]), c=_fmt(p[4]), d=_fmt(p[5])) for p in offset_pieces])
    _replace(lane, "width", [etree.Element(
        "width", sOffset=_fmt(p[0]), a=_fmt(p[2]), b=_fmt(p[3]), c=_fmt(p[4]), d=_fmt(p[5])) for p in width_pieces])


# Knot curvature bound of the structure-preserving refit: never tighter than the connector's own
# curvature (a squeezed old connector at 0.38-0.40 /m could not even be reproduced under +/-0.3).
KAPPA_BOUND = 0.3


def align_tree(root, via_centrelines=None, mouth_blend_kappa=None, kappa_bound_scale=None, mouth_blend_pick=False,
               source_guided=False, width_local_slopes=False, match_end_curvature=False, monotone_turns=False,
               aligned_frame=False, source_fit=False, turn_end_zone=None):
    """Refit one-lane connector reference lines onto the linked roads' cross-sections.

    With source via centrelines (G8 manifest), the refit targets the source lane centre
    first (tolerance SOURCE_TOL); otherwise, or if that misses, the old lane centre.
    ``mouth_blend_kappa``: carry lateral shifts at the mouths only near the mouths
    (_localized_offsets); default None keeps the single edge cubic.
    ``kappa_bound_scale``: knot curvature bound max(KAPPA_BOUND, scale * old max); default None
    keeps +/-KAPPA_BOUND.
    ``mouth_blend_pick``: per connector, write whichever of spread / localized lies closer to the
    source via centreline (default False: always localized when ``mouth_blend_kappa`` is set).
    ``source_guided``: also try a refit to the source centreline itself (_source_guided) and write,
    among all candidates, one within the G8 ceilings that lies closest to the source.
    ``width_local_slopes``: where the width cubic would bulge past the budget, meet both mouths' edge
    headings exactly with local end-slope corrections instead of limiting the slopes (_width_local_slopes).
    ``match_end_curvature``: every candidate's lane centre also meets the linked lanes' world curvature
    at both mouths (_match_end_curvature), so the lane centre is G2 across the junction.
    ``monotone_turns``: in connectors turning 30 deg or more, the refits keep the knot curvatures on the
    turn's side (falling back to the free refit where that misses the tolerance), and the pick prefers,
    after the G8 ceilings, a candidate whose lane centre does not curve against the turn (WIGGLE_MAX).
    ``aligned_frame``: also offer references whose ends sit on the linked lane centres (_aligned_refits), and
    prefer, after the 6 m segment rule, a candidate with smaller lane-edge curvature jumps at its own joins
    (in EDGE_JOIN_STEP steps).
    ``source_fit``: connectors more than SOURCE_FIT_TRIGGER_M off their source via line (lanes G8 compares
    only) also get fidelity-first lane-centre chains (_source_fits, segments >= 3 m), and the pick ranks, right
    after the G8 ceilings and the smoothness flags, the source distance (P95) plus PRIMITIVE_COST_M per primitive,
    then the 6 m segment rule.
    ``turn_end_zone`` (metres, curvature): with ``monotone_turns``, a turn may steer back up to that curvature
    within that distance of either mouth, where the source via lines meet the linked lanes at a few degrees
    (user decision 2026-10-05); the source fits get end-zone turn fits as well, and the reversal flag judges
    the middle of the turn by WIGGLE_MAX and the end zones by that curvature.
    """
    via_centrelines = via_centrelines or {}
    from mapforge.ops.refline_fit import fit_connector_minimal, planview_prims
    from mapforge.validate.g11 import _driving_curvature_joins
    from mapforge.validate.smoothness import _geoms, lane_endpoint_state, sample_road_ref
    from mapforge.validate.turn_shape import centre_curvature_profile, counter_curvature, counter_curvature_zones
    from mapforge.validate.lane_edge_joins import road_edge_jumps

    roads = {r.get("id"): r for r in root.findall("road")}
    connections = {c.get("connectingRoad"): c for c in root.findall("junction/connection")}
    rows, skipped = [], []
    for cr in root.findall("road"):
        if cr.get("junction") in (None, "-1") or cr.get("name") == "junction_paving":
            continue
        rid = cr.get("id")
        reason = _supported(cr)
        connection = connections.get(rid)
        if reason is None and (connection is None or connection.get("contactPoint") != "start"):
            reason = "no start-contact junction connection"
        if reason is None and any(float(e.get("s", 0)) > 0 for e in cr.findall("elevationProfile/elevation")):
            reason = "connector has s-dependent elevation records"
        if reason:
            skipped.append({"connecting_road": rid, "reason": reason})
            continue
        lanes = cr.find("lanes")
        lane = lanes.find("laneSection/right/lane[@id='-1']")
        old_offset, old_width = _records(lanes, "laneOffset", "s"), _records(lane, "width", "sOffset")
        old_geoms = _geoms(cr)

        def centre_old(s, order=0):
            return _piecewise(old_offset, s, order) - _piecewise(old_width, s, order) / 2

        try:
            pred, succ = cr.find("link/predecessor"), cr.find("link/successor")
            fa = _travel_frame(roads[pred.get("elementId")], pred.get("contactPoint"),
                               pred.get("contactPoint") == "end")
            fc = _travel_frame(roads[succ.get("elementId")], succ.get("contactPoint"),
                               succ.get("contactPoint") == "start")
            length_old = float(cr.get("length"))
            old0, old1 = _frame(cr, "start"), _frame(cr, "end")
            lane0 = old0[2] + math.atan2(centre_old(0, 1), 1 - old0[3] * centre_old(0))
            lane1 = old1[2] + math.atan2(centre_old(length_old, 1), 1 - old1[3] * centre_old(length_old))
            beta0, beta1 = _wrap(lane0 - fa[2]), _wrap(lane1 - fc[2])
            if max(abs(beta0), abs(beta1)) > math.radians(MAX_TILT_DEG):
                raise ValueError(f"cross-section tilt {math.degrees(max(abs(beta0), abs(beta1))):.1f} deg too large")
            # Lane-centre offset relative to the new reference: zero at both mouths, with the
            # slope that turns the road's cross-section heading into the lane-centre heading.
            tc = _hermite(0.0, math.tan(beta0), 0.0, math.tan(beta1), length_old)
            pts, ss, hh = sample_road_ref(cr, ds=0.25)
            old_path = _offset_path(pts, ss, hh, centre_old)
            sid = lane.find("userData[@code='mapforge.source_lane']")
            source = via_centrelines.get(sid.get("value")) if sid is not None else None
            profile = _source_profile(source, pts, ss, hh) if (source is not None and USE_SOURCE_BLEND) else None
            if profile is not None:
                # middle of the connector follows the source lane centre; both mouths keep the old
                # (mouth-consistent) path; smooth weight, only inside the source support
                lo, hi = profile[0, 0], profile[-1, 0]
                d = min(SOURCE_BLEND_END_M, length_old / 4)

                def weight(s):
                    x = min(max(min(s - max(lo, 0.0), min(hi, length_old) - s) / d, 0.0), 1.0)
                    return x * x * (3 - 2 * x)

                def centre_target(s):
                    w = weight(s)
                    t_src = float(np.interp(s, profile[:, 0], profile[:, 1]))
                    return centre_old(s) + w * (t_src - centre_old(s))
            else:
                centre_target = centre_old
            target = _offset_path(pts, ss, hh, lambda s: centre_target(s) - _poly(tc, s))
            p0 = (float(old_path[0, 0]), float(old_path[0, 1]), fa[2])
            p1 = (float(old_path[-1, 0]), float(old_path[-1, 1]), fc[2])
            # Same cross-section rotation rate as the linked roads at the lane centre, so the
            # lane-centre heading (not only the edges) is continuous across the mouth.
            k0, k1 = _parallel_curvature(fa, p0), _parallel_curvature(fc, p1)
            old_min = min(g[4] for g in old_geoms)
            bound = KAPPA_BOUND
            if kappa_bound_scale:
                bound = max(KAPPA_BOUND, kappa_bound_scale * max(max(abs(g[5]), abs(g[6])) for g in old_geoms))
            prims, end_error, deviation = _structure_refit(old_geoms, p0, p1, k0, k1, tc, length_old, target, bound)
            method = ("source-blend:" if profile is not None else "") + "structure-preserving"
            if end_error > 1e-6 or deviation > FIT_TOL["max_dev_tol"]:
                fit = fit_connector_minimal(target, p0, p1, k0=k0, k1=k1, max_segments=5, max_kappa=0.25,
                                            sharpness_cap=0.20, min_segment_m=min(old_min, 6.0), **FIT_TOL)
                if fit is None:
                    raise ValueError(f"refit missed the tolerance (structure-preserving max {deviation:.3f} m)")
                prims, method = fit.primitives, ("source-blend:" if profile is not None else "") + fit.method
            # A turn whose refit curves against itself also gets a turn-sign bounded refit of the same old path.
            # The old path may carry the reversal itself, so the bounded refit is judged against the source (G8
            # ceilings in the pick), not by FIT_TOL; both versions stay candidates.
            sign = _turn_sign(p0, p1) if monotone_turns else 0
            mono_prims = None
            if _counter_knots(prims, sign) > TURN_SIGN_TOL:
                mono, err_m, dev_m = _structure_refit(prims, p0, p1, k0, k1, (0.0, 0.0, 0.0, 0.0),
                                                      float(sum(p[4] for p in prims)), target, bound, sign)
                if err_m <= 1e-6 and dev_m <= MONOTONE_DEV_MAX:
                    mono_prims = mono
            xe, ye, he = _end_pose(prims)
            start = (prims[0][1], prims[0][2], prims[0][3], prims[0][5])
            end = (xe, ye, he, prims[-1][6])
            targets = _targets(roads, cr, connection)
            local = {(c, side): _local(start if c == "start" else end, targets[c][side])
                     for c in ("start", "end") for side in ("left", "right")}
            aligned, aligned_info = [], None
            if aligned_frame:
                try:
                    aligned, aligned_info = _aligned_refits(old_geoms, old_path, ss, hh, targets, fa, fc, length_old,
                                                            bound, sign, old_min, mouth_blend_kappa or 0.01)
                except (ValueError, ZeroDivisionError, np.linalg.LinAlgError) as exc:
                    aligned, aligned_info = [], {"reason": f"{type(exc).__name__}: {exc}"}
        except (KeyError, ValueError, ZeroDivisionError, AttributeError) as exc:
            skipped.append({"connecting_road": rid, "reason": f"{type(exc).__name__}: {exc}"})
            continue
        # Candidates for carrying the lateral shifts at the mouths: the single edge cubic (spread over the
        # connector) and, if enabled, the localized blend. A shift that moves a lane centre onto its source
        # is best spread; one that moves it off on purpose (curb-return shoulder) is best kept local. The
        # candidate whose lane centre is closer to the source via centreline (median) is written.
        # Each candidate: (name, prims, offset pieces, centre pieces, width pieces, info, shape, local).
        def frame_candidates(c_prims, suffix, with_local=True):
            c_len = float(sum(p[4] for p in c_prims))
            xe_, ye_, he_ = _end_pose(c_prims)
            c_start, c_end = (c_prims[0][1], c_prims[0][2], c_prims[0][3], c_prims[0][5]), (xe_, ye_, he_, c_prims[-1][6])
            c_local = {(c, side): _local(c_start if c == "start" else c_end, targets[c][side])
                       for c in ("start", "end") for side in ("left", "right")}
            c_centre, c_width, c_shape = _edge_model(c_local, c_len, width_local_slopes)
            c_cen = [(0.0, c_len, *c_centre)]
            out = [("spread" + suffix, c_prims, _sum_pieces([c_cen, _scaled(c_width, 0.5)], c_len), c_cen, c_width,
                    None, c_shape, c_local)]
            if mouth_blend_kappa and with_local:
                snap = _end_blends(c_shape.get("width_slope_lengths_m"), c_len) if match_end_curvature else None
                off, cen, wid, info = _localized_from(c_centre, c_width, c_len, mouth_blend_kappa, snap)
                out.append(("local" + suffix, c_prims, off, cen, wid, info, c_shape, c_local))
            return out

        try:
            candidates = frame_candidates(prims, "")
            if mono_prims is not None:
                candidates += frame_candidates(mono_prims, "-monotone")
            for a_prims, a_name in aligned:
                # no shift left at the mouths: the spread and local candidates would coincide
                candidates += [(a_name, *c[1:]) for c in frame_candidates(a_prims, "", with_local=False)]
        except ValueError as exc:
            skipped.append({"connecting_road": rid, "reason": f"ValueError: {exc}"})
            continue
        if source_guided and source is not None:
            for g_prims, g_local, g_info in _source_guided(source, targets, fa, fc, min(g[4] for g in old_geoms),
                                                           monotone_turns) or []:
                g_len = float(sum(p[4] for p in g_prims))
                g_centre, g_width, g_shape = _edge_model(g_local, g_len, width_local_slopes)
                g_cen = [(0.0, g_len, *g_centre)]
                candidates.append(("source-guided" + ("-monotone" if g_info.get("monotone") else ""), g_prims,
                                   _sum_pieces([g_cen, _scaled(g_width, 0.5)], g_len), g_cen, g_width, g_info,
                                   g_shape, g_local))
        prov = lane.find("userData[@code='mapforge.provenance/v1']")
        comparable = prov is not None and json.loads(prov.get("value") or "{}").get("eligibility") == "comparable"
        if source_fit and source is not None and comparable:
            # (a lane G8 excludes, e.g. minimal-chain-source-fidelity-unmet, has a source no sane chain follows)
            before = _source_fidelity(source, old_path)
            if before is None or max(before["p95_m"], before["reverse_p95_m"]) > SOURCE_FIT_TRIGGER_M:
                rates = (_contact_rate(roads[pred.get("elementId")], pred.get("contactPoint")),
                         _contact_rate(roads[succ.get("elementId")], succ.get("contactPoint")))
                for f_prims, f_info in _source_fits(source, targets, fa, fc, rates, monotone_turns,
                                                    turn_end_zone if monotone_turns else None):
                    f_len = float(sum(p[4] for p in f_prims))
                    xe_, ye_, he_ = _end_pose(f_prims)
                    f_start = (f_prims[0][1], f_prims[0][2], f_prims[0][3], f_prims[0][5])
                    f_end = (xe_, ye_, he_, f_prims[-1][6])
                    try:
                        f_local = {(c, side): _local(f_start if c == "start" else f_end, targets[c][side])
                                   for c in ("start", "end") for side in ("left", "right")}
                    except ValueError:
                        continue
                    f_centre, f_width, f_shape = _edge_model(f_local, f_len, width_local_slopes)
                    f_cen = [(0.0, f_len, *f_centre)]
                    name_ = f"source-fit-{f_info['seg_m']:g}m" + ("-ends" if f_info.get("end_zone") else
                                                                  "-monotone" if f_info["turn_sign"] else "")
                    candidates.append((name_, f_prims, _sum_pieces([f_cen, _scaled(f_width, 0.5)], f_len), f_cen,
                                       f_width, f_info, f_shape, f_local))
        if match_end_curvature:
            try:
                linked = _linked_lanes(roads, cr, connection)
                centre_targets = {c: lane_endpoint_state(r, lid, oc, forward=fw) for c, (r, lid, oc, fw) in linked.items()}
            except (KeyError, ValueError) as exc:
                skipped.append({"connecting_road": rid, "reason": f"end curvature targets: {type(exc).__name__}: {exc}"})
                continue
            matched = []
            for name_, c_prims, _off, _cen, c_wid, c_info, c_shape, c_local in candidates:
                blends = (c_info["start_blend_m"], c_info["end_blend_m"]) if c_info and "start_blend_m" in c_info else None
                off, cen, wid, curv = _match_end_curvature(c_prims, _off, c_wid, centre_targets, targets,
                                                           c_shape.get("width_slope_lengths_m"), blends)
                matched.append((name_, c_prims, off, cen, wid, c_info, {**c_shape, "end_curvature": curv}, c_local))
            candidates = matched
        widths_ok = [c for c in candidates
                     if min(_pc_eval(c[4], sum(p[4] for p in c[1]) * i / 200) for i in range(201)) > 0.5]
        if not widths_ok:
            skipped.append({"connecting_road": rid, "reason": "Hermite width would drop below 0.5 m"})
            continue
        candidates = widths_ok

        def edge_trim(c):
            """The picked candidate ``c`` with the curvature rate at a mouth whose edge residual exceeds
            JOIN_JUMP_MAX moved towards the linked road's (_end_rate_trim), as a candidate of its own; None if
            no trim brings both mouths within the bound."""
            name_, c_prims, _off, _cen, _wid, c_info, c_shape, _loc = c
            curv = c_shape.get("end_curvature")
            if not curv or (c_info and "start_blend_m" in c_info):
                return None                  # (localized blends are not rebuilt here)
            road_rate = {"start": _contact_rate(roads[pred.get("elementId")], pred.get("contactPoint")),
                         "end": _contact_rate(roads[succ.get("elementId")], succ.get("contactPoint"))}
            own_rate = {"start": (c_prims[0][6] - c_prims[0][5]) / c_prims[0][4],
                        "end": (c_prims[-1][6] - c_prims[-1][5]) / c_prims[-1][4]}
            over = {e: max(abs(r) for r in curv[e]["edge_t2_residual"]) for e in ("start", "end")}
            over = {e: r for e, r in over.items() if r > JOIN_JUMP_MAX}
            if not over:
                return None
            knots = [q[5] for q in c_prims] + [c_prims[-1][6]]
            turn = sign if sign and _counter_knots(c_prims, sign) <= TURN_SIGN_TOL else 0
            for step in TRIM_STEPS:
                rates = {e: road_rate[e] + step * TRIM_MARGIN * JOIN_JUMP_MAX / r * (own_rate[e] - road_rate[e])
                         for e, r in over.items()}
                out = _end_rate_trim(c_prims, rates, max(0.25, max(abs(k) for k in knots)), turn)
                if out is None or out[1] > TRIM_DEV_MAX_M:
                    continue
                t_prims = out[0]
                t_len = float(sum(q[4] for q in t_prims))
                xe_, ye_, he_ = _end_pose(t_prims)
                t_ends = {"start": (t_prims[0][1], t_prims[0][2], t_prims[0][3], t_prims[0][5]),
                          "end": (xe_, ye_, he_, t_prims[-1][6])}
                try:
                    t_local = {(e, side): _local(t_ends[e], targets[e][side])
                               for e in ("start", "end") for side in ("left", "right")}
                except ValueError:
                    continue
                t_centre, t_width, t_shape = _edge_model(t_local, t_len, width_local_slopes)
                t_cen = [(0.0, t_len, *t_centre)]
                off, cen, wid, t_curv = _match_end_curvature(t_prims, _sum_pieces([t_cen, _scaled(t_width, 0.5)], t_len),
                                                             t_width, centre_targets, targets,
                                                             t_shape.get("width_slope_lengths_m"), None)
                if max(abs(r) for e in ("start", "end") for r in t_curv[e]["edge_t2_residual"]) > JOIN_JUMP_MAX:
                    continue
                if min(_pc_eval(wid, t_len * i / 200) for i in range(201)) <= 0.5:
                    continue
                info = {**(c_info or {}), "edge_trim": {"rates_per_m2": {e: round(v, 6) for e, v in rates.items()},
                                                        "road_rates_per_m2": {e: round(road_rate[e], 6) for e in over},
                                                        "own_rates_per_m2": {e: round(own_rate[e], 6) for e in over},
                                                        "deviation_m": round(out[1], 4)}}
                return (name_ + "-trim", t_prims, off, cen, wid, info, {**t_shape, "end_curvature": t_curv}, t_local)
            return None

        snapshot = [copy.deepcopy(child) for child in cr]
        snapshot_length = cr.get("length")
        med_max, p95_max = _via_limits()
        within = lambda f: f["median_m"] <= med_max and f["p95_m"] <= p95_max
        def score(c):
            _write_planview(cr, c[1])
            c_pts, c_ss, c_hh = sample_road_ref(cr, ds=0.25)
            path = _offset_path(c_pts, c_ss, c_hh, lambda s, c=c: _pc_eval(c[3], s))
            jump = None
            fidelity_c = _source_fidelity(source, path)
            if source_guided:
                # lane-centre curvature jumps at the candidate's own knots (reference rate changes with an
                # offset slope also jump: G11 _driving_curvature_joins); written provisionally here
                _write_lane_records(lanes, lane, c[2], c[4])
                jump = max((j["curvature_jump_per_m"] for j in _driving_curvature_joins(cr)), default=0.0)
                if monotone_turns and sign and fidelity_c is not None:
                    # lane-centre curvature against the turn (steering reversal), on the written records
                    profile_c = centre_curvature_profile(cr)
                    fidelity_c["counter_curvature_per_m"] = counter_curvature(profile_c)[1]
                    if turn_end_zone:
                        mid_c, ends_c = counter_curvature_zones(profile_c, turn_end_zone[0])
                        fidelity_c["counter_mid_per_m"], fidelity_c["counter_ends_per_m"] = mid_c, ends_c
                if aligned_frame and fidelity_c is not None:
                    # lane-edge curvature jumps at the connector's own joins (t * t' * reference sharpness step)
                    # (only reference joins can jump: the lateral functions are C2)
                    fidelity_c["edge_join_jump_per_m"] = max((j for j, *_ in road_edge_jumps(cr, reference_only=True)),
                                                              default=0.0)
            return (c, path, (c_pts, c_ss, c_hh), fidelity_c, jump)

        def pick_from(scored):
            pick, flags = 0, None
            meds = [(x[3] or {}).get("median_m") for x in scored]
            if source_guided and None not in meds:
                # within the G8 ceilings first; then no lane-centre curvature jump above JOIN_JUMP_MAX; then no
                # primitive shorter than the converter's 6 m (a 3 m source-guided chain only where nothing else
                # stays within the ceilings); then the closest to the source (median)
                short = [min(p[4] for p in x[0][1]) < SOURCE_GUIDE_MIN_SEGMENT_M[0] - 1e-6 for x in scored]
                jumpy = [(x[4] or 0.0) > JOIN_JUMP_MAX for x in scored]
                # lane-edge curvature residual at the mouths (match_end_curvature) above the T2 draft bound
                edge_res = [max((abs(r) for c in ("start", "end")
                                 for r in x[0][6].get("end_curvature", {}).get(c, {}).get("edge_t2_residual", [])),
                                default=0.0) for x in scored]
                edgy = [r > JOIN_JUMP_MAX for r in edge_res]
                # source_fit (lanes G8 compares; an excluded lane's source ranks nothing): how far over the bound
                # counts too (whole JOIN_JUMP_MAX steps), so that among candidates that all miss it the smallest
                # residual wins before the source distance
                fit_pick = source_fit and comparable
                edge_steps = [max(0, math.ceil(r / JOIN_JUMP_MAX - 1e-9) - 1) if fit_pick else int(e)
                              for r, e in zip(edge_res, edgy)]
                if match_end_curvature:
                    # G8 checks both directions and coverage: so does the pick (same guard margin)
                    inside = [within(x[3]) and x[3]["reverse_median_m"] <= med_max and x[3]["reverse_p95_m"] <= p95_max
                              and x[3]["coverage"] >= COVERAGE_MIN for x in scored]
                else:
                    inside = [within(x[3]) for x in scored]
                # steering reversal inside a turn (monotone_turns), right after the G8 ceilings
                if turn_end_zone:
                    # the middle of the turn by WIGGLE_MAX, the zones next to the mouths by the end-zone curvature
                    wiggly = [x[3].get("counter_mid_per_m", x[3].get("counter_curvature_per_m", 0.0)) > WIGGLE_MAX
                              or x[3].get("counter_ends_per_m", 0.0) > turn_end_zone[1] for x in scored]
                else:
                    wiggly = [x[3].get("counter_curvature_per_m", 0.0) > WIGGLE_MAX for x in scored]
                # lane-edge curvature jumps at the connector's own joins (aligned_frame), in EDGE_JOIN_STEP steps,
                # after the 6 m segment rule and before the source distance
                edge_jumpy = [max(0, math.ceil(x[3].get("edge_join_jump_per_m", 0.0) / EDGE_JOIN_STEP - 1.0)) for x in scored]
                # source_fit: right after the smoothness flags, the source distance (worst direction) plus a cost per
                # primitive (connector_source_fit.PRIMITIVE_COST_M): a denser chain has to be that much closer per
                # extra primitive (a 0.02 m equal band let node18 road 114 take 16 primitives for 0.03 m over 4)
                from mapforge.ops.connector_source_fit import PRIMITIVE_COST_M
                p95 = [max(x[3]["p95_m"], x[3].get("reverse_p95_m", x[3]["p95_m"])) for x in scored]
                cost = [p95[i] + PRIMITIVE_COST_M * len(x[0][1]) if fit_pick else 0.0 for i, x in enumerate(scored)]
                pick = min(range(len(scored)), key=lambda i: (not inside[i], wiggly[i], jumpy[i], edge_steps[i], cost[i],
                                                              short[i], edge_jumpy[i], meds[i]))
                flags = {x[0][0]: {"within_g8": bool(inside[i]), "counter_curvature": bool(wiggly[i]),
                                   "lane_centre_jump": bool(jumpy[i]), "edge_residual": bool(edgy[i]),
                                   "edge_residual_per_m": edge_res[i], "cost_m": cost[i],
                                   "primitives": len(x[0][1]),
                                   "edge_join_jump": int(edge_jumpy[i]), "short": bool(short[i]),
                                   "reverse_p95_m": x[3].get("reverse_p95_m"), "coverage": x[3].get("coverage"),
                                   "counter_curvature_per_m": x[3].get("counter_curvature_per_m"),
                                   "counter_mid_per_m": x[3].get("counter_mid_per_m"),
                                   "counter_ends_per_m": x[3].get("counter_ends_per_m"),
                                   "edge_join_jump_per_m": x[3].get("edge_join_jump_per_m")}
                         for i, x in enumerate(scored)}
            elif mouth_blend_kappa:
                # registered curb-local (always local) / curb-pick (local if closer) behaviour
                pick = 1 if not mouth_blend_pick or None in meds[:2] or meds[1] < meds[0] else 0
            return pick, flags

        scored = [score(c) for c in candidates]
        pick, flags = pick_from(scored)
        if match_end_curvature and flags and flags.get(scored[pick][0][0], {}).get("edge_residual"):
            # the picked candidate misses the edge bound at a mouth: its end-rate trim competes too
            trimmed = edge_trim(scored[pick][0])
            if trimmed is not None:
                scored.append(score(trimmed))
                pick, flags = pick_from(scored)
        chosen, new_path, (new_pts, new_ss, new_hh), _, chosen_jump = scored[pick]
        name, prims, offset_pieces, centre_pieces, width_pieces, blend, shape, local = chosen
        length_new = float(sum(p[4] for p in prims))
        min_width = min(_pc_eval(width_pieces, length_new * i / 200) for i in range(201))
        if len(scored) > 1:
            blend = {**(blend or {}), "chosen": name,
                     "source_median_m": {x[0][0]: (x[3] or {}).get("median_m") for x in scored},
                     "source_p95_m": {x[0][0]: (x[3] or {}).get("p95_m") for x in scored}}
            if flags:
                blend["pick_flags"] = flags
        if name.startswith("source-guided"):
            method = "source-guided:" + chosen[5]["method"]
        elif name.startswith("source-fit"):
            method = name
        _write_planview(cr, prims)
        _write_lane_records(lanes, lane, offset_pieces, width_pieces)
        shift = _max_gap(old_path, new_path)
        fidelity = {"before": _source_fidelity(source, old_path), "after": _source_fidelity(source, new_path)}
        med_max, p95_max = _via_limits()
        within = lambda f: f["median_m"] <= med_max and f["p95_m"] <= p95_max
        if fidelity["before"] and within(fidelity["before"]) and not within(fidelity["after"]):
            # guard: aligning must not move the lane centre away from its source lane centre
            for child in list(cr):
                cr.remove(child)
            for child in snapshot:
                cr.append(child)
            cr.set("length", snapshot_length)
            skipped.append({"connecting_road": rid, "reason": "source-fidelity guard: alignment would push the via "
                            f"lane over the G8 ceiling (median {fidelity['before']['median_m']:.3f} -> "
                            f"{fidelity['after']['median_m']:.3f} m)"})
            continue
        remapped = _remap_support_s(lane, old_path, ss, new_path, new_ss)
        record = {"tilt_start_deg": math.degrees(beta0), "tilt_end_deg": math.degrees(beta1),
                  "fit_method": method, "primitives": len(prims),
                  "min_primitive_m": min(p[4] for p in prims),
                  "min_primitive_old_m": min(g[4] for g in old_geoms),
                  "length_old_m": length_old, "length_new_m": length_new,
                  "start_width_m": _pc_eval(width_pieces, 0.0), "end_width_m": _pc_eval(width_pieces, length_new),
                  "min_width_m": min_width,
                  **shape,
                  "lane_centre_shift_max_m": shift,
                  "mouth_blend": blend,
                  "lane_centre_join_jump_max_per_m": chosen_jump,
                  "along_track_residual_m": max(abs(v[2]) for v in local.values()),
                  "support_s_remapped": remapped,
                  "source_fidelity": fidelity,
                  "turn": {"sign": sign, "monotone_candidate": mono_prims is not None,
                           "lane_centre_counter_curvature_per_m": (scored[pick][3] or {}).get("counter_curvature_per_m")}
                  if monotone_turns else None,
                  "aligned": aligned_info if aligned_frame else None,
                  "edge_curvature": ("lane centre G2 at both mouths; edges least squares (edge_t2_residual)"
                                     if match_end_curvature else "not matched (G1 plus bounded jump)")}
        lane.append(etree.Element("userData", code=CODE, value=json.dumps(record, ensure_ascii=False)))
        rows.append({"connecting_road": rid, **record})
    return {"schema": CODE, "rewritten": len(rows), "skipped": skipped, "rows": rows}


def apply(xodr_in, xodr_out, manifest=None, mouth_blend_kappa=None, kappa_bound_scale=None, mouth_blend_pick=False,
          source_guided=False, width_local_slopes=False, match_end_curvature=False, monotone_turns=False,
          aligned_frame=False, source_fit=False, turn_end_zone=False):
    """Align where the refit holds; elsewhere fall back to step-0 Hermite edges on the old frame.

    ``turn_end_zone``: True applies connector_source_fit.END_ZONE to SHP outputs only (MAP outputs stay as they
    were: their connectors have no source via line to meet)."""
    from mapforge.ops.mouth_edge_match import match_tree

    tree = etree.parse(str(xodr_in), etree.XMLParser(strip_cdata=False, remove_blank_text=False))
    manifest_path = Path(manifest or Path(xodr_in).with_suffix(".source-lanes.json"))
    via = load_via_centrelines(manifest_path)
    zone = None
    if turn_end_zone and manifest_path.exists():
        from mapforge.ops.connector_source_fit import END_ZONE
        if json.loads(manifest_path.read_text(encoding="utf-8")).get("source_format") == "shp":
            zone = END_ZONE
    report = align_tree(tree.getroot(), via, mouth_blend_kappa=mouth_blend_kappa, kappa_bound_scale=kappa_bound_scale,
                        mouth_blend_pick=mouth_blend_pick, source_guided=source_guided,
                        width_local_slopes=width_local_slopes, match_end_curvature=match_end_curvature,
                        monotone_turns=monotone_turns, aligned_frame=aligned_frame, source_fit=source_fit,
                        turn_end_zone=zone)
    report["turn_end_zone"] = zone
    report["source_guided"] = source_guided
    report["source_fit"] = source_fit
    report["width_local_slopes"] = width_local_slopes
    report["match_end_curvature"] = match_end_curvature
    report["monotone_turns"] = monotone_turns
    report["aligned_frame"] = aligned_frame
    report["mouth_blend_kappa"] = mouth_blend_kappa
    report["mouth_blend_pick"] = mouth_blend_pick
    report["kappa_bound_scale"] = kappa_bound_scale
    report["source_blend_targets"] = sum(1 for r in report["rows"] if r["fit_method"].startswith("source-blend"))
    fallback_ids = {s["connecting_road"] for s in report["skipped"]
                    if not s["reason"].startswith("connector ")}
    if fallback_ids:
        report["fallback_edge_match"] = match_tree(tree.getroot(), only=fallback_ids)
    Path(xodr_out).parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(xodr_out), xml_declaration=True, encoding="UTF-8")
    return report
