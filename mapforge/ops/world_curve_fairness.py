"""World-space fairing of a cubic-offset line/arc/spiral ribbon.

Evaluation intervals are NOT additional output geometry. Curvature numerator
roots partition each existing interval into constant-turn-sign pieces. Root
finding is numerical, not an interval-arithmetic certificate. Source S-bends
are never assigned a global curvature sign.
"""
import numpy as np
from functools import lru_cache
from numpy.polynomial import Polynomial as P
from numpy.polynomial.legendre import leggauss
from spikes.road_boundary_family import world_kinematics


@lru_cache(maxsize=8)
def gaussian_nodes(order):
    nodes,weights=leggauss(order)
    nodes.setflags(write=False);weights.setflags(write=False)
    return nodes,weights


def source_turn_evidence(raw, *, endpoint_headings=None):
    if endpoint_headings is not None and set(endpoint_headings)!=set(raw):
        raise ValueError('original endpoint headings must cover exactly the source fields')
    evidence = {}
    for field, points in raw.items():
        points = np.asarray(points, float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError('finite complete planar source required')
        delta = np.diff(points, axis=0)
        delta = delta[np.linalg.norm(delta, axis=1) > 1e-8]
        if len(delta) < 2:
            raise ValueError('nondegenerate source headings required')
        heading = np.arctan2(delta[:, 1], delta[:, 0])
        if endpoint_headings is not None:
            ends=np.asarray(endpoint_headings[field],float)
            if ends.shape!=(2,) or not np.isfinite(ends).all():
                raise ValueError('two original endpoint headings per source field required')
            # Tangents retain source turn evidence as a clipped tail tends to
            # zero length. No point/road segment or configurable allowance is added.
            heading=np.r_[ends[0],heading,ends[1]]
        heading=np.unwrap(heading)
        net = float(heading[-1] - heading[0]); sign = 1 if net >= 0 else -1
        reverse = float(np.degrees(np.maximum(-sign*np.diff(heading), 0).sum()))
        evidence[field] = dict(sign=sign, net_turn_deg=float(np.degrees(net)),
            reverse_turn_deg=reverse, single_turn=bool(abs(np.degrees(net)) >= 30 and reverse <= .5))
    return evidence


def source_turn_slacks(fairness, allowance_deg=.5):
    """Budget every observed field, including source S/compound curves.

    `single_turn` is only descriptive metadata, never an exemption. The
    original reverse-turn budget is already subtracted by ribbon_fairness.
    """
    if set(fairness) != {'left', 'right', 'center'}:
        raise ValueError('all three original fields must constrain fairing')
    if not np.isfinite(allowance_deg) or not 0 <= allowance_deg <= .5:
        raise ValueError('construction allowance must not relax final review')
    extra=np.array([fairness[k]['additional_reverse_turn_deg'] for k in ('left','right','center')])
    if not np.isfinite(extra).all():raise ValueError('finite source-relative turning required')
    return np.radians(allowance_deg-extra)*20


def interval_fairness(k0, dk, coefficients, length, *, order=16):
    """Exact polynomial numerator; numerical roots and Gaussian integration.

    World curve r'=A*T+B*N, A=1-k*t, B=t'. Curvature is
    (A*(k*A+t'')+B*(dk*t+2*k*B))/(A*A+B*B)**1.5.
    Integrate (d curvature / d world arc length)**2 against world arc length.
    """
    c = np.asarray(coefficients, float)
    if c.shape != (4,) or not np.isfinite(c).all() or not np.isfinite([k0, dk, length]).all() or length <= 0:
        raise ValueError('finite positive existing curve interval required')
    if not isinstance(order, int) or order < 8:
        raise ValueError('at least eight Gaussian evaluation nodes required')
    # Use u in [0,1] to avoid ill-conditioning of roots on long metre spans.
    t = P(c*length**np.arange(4)); d1 = t.deriv()/length; d2 = d1.deriv()/length
    k = P([k0, dk*length]); A = 1-k*t; B = d1
    numerator = A*(k*A+d2)+B*(dk*t+2*k*B)
    roots = numerator.trim(tol=max(1e-16, np.max(abs(numerator.coef))*1e-14)).roots()
    sites = np.unique(np.r_[0., [float(r.real) for r in roots if abs(r.imag) < 1e-7 and 0 < r.real < 1], 1.])
    critical = A.deriv().roots()
    forward = float(min(A(np.r_[0., 1., [r.real for r in critical if abs(r.imag) < 1e-8 and 0 < r.real < 1]])))
    if forward <= 0:
        # The existing regularity constraint must restore this first; do not
        # pretend atan2 angle differences are valid through a fold.
        return dict(regular=False, forward_min=forward, positive_turn_deg=1e5,
            negative_turn_deg=1e5, curvature_variation_energy=1e8, bending_energy=1e8)
    heading = k0*(sites*length)+.5*dk*(sites*length)**2+np.arctan2(B(sites), A(sites))
    turns = np.diff(heading)
    g, weights = gaussian_nodes(order); u = (g+1)/2; s = u*length
    jets = np.stack([t(u), d1(u), d2(u), np.full(len(u), 6*c[3])], axis=-1)
    curvature, rate = world_kinematics(jets, k0+dk*s, dk).T
    speed = np.hypot(A(u), B(u)); measure = weights*length/2*speed
    return dict(regular=True, forward_min=forward,
        positive_turn_deg=float(np.degrees(np.maximum(turns, 0).sum())),
        negative_turn_deg=float(np.degrees(np.maximum(-turns, 0).sum())),
        curvature_variation_energy=float(measure@rate**2), bending_energy=float(measure@curvature**2))


def ribbon_fairness(cls, knots, coefficients, evidence, *, order=16):
    refs = np.r_[0., np.cumsum([c.length for c in cls])]
    knots = np.asarray(knots, float)
    if not np.isclose(knots[-1], refs[-1]) or not np.all(np.diff(knots) > 0):
        raise ValueError('complete structural interval domain required')
    fields = dict(coefficients, center=(coefficients['left']+coefficients['right'])/2)
    result = {}
    for field, rows in fields.items():
        totals = dict(positive_turn_deg=0., negative_turn_deg=0., curvature_variation_energy=0., bending_energy=0.)
        regular = True
        for i, (start, end) in enumerate(zip(knots[:-1], knots[1:])):
            # Split for EVALUATION if a caller's polynomial spans a reference
            # boundary. Rebase the SAME polynomial, never add a free parameter.
            cuts = np.r_[start, refs[(refs > start+1e-9) & (refs < end-1e-9)], end]
            for lo, hi in zip(cuts[:-1], cuts[1:]):
                index = np.clip(np.searchsorted(refs, (lo+hi)/2)-1, 0, len(cls)-1)
                ref = cls[index]; shifted = P(rows[i])(P([lo-start, 1.])).coef
                co = np.pad(shifted, (0, 4-len(shifted)))
                item = interval_fairness(ref.KappaStart+ref.dk*(lo-refs[index]), ref.dk, co, hi-lo, order=order)
                regular = regular and item['regular']
                for key in totals: totals[key] += item[key]
        ev = evidence[field]
        reverse = totals['negative_turn_deg' if ev['sign'] > 0 else 'positive_turn_deg']
        result[field] = dict(totals, regular=regular, source=ev, reverse_turn_deg=reverse,
            additional_reverse_turn_deg=reverse-ev['reverse_turn_deg'], continuous_certificate=False)
    return result
