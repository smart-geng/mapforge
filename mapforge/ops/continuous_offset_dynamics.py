"""Whole-span dynamics of a cubic lateral offset on Line/Arc/Spiral.

Checks subdivide parameter intervals, NEVER the represented road. Bernstein
control bounds and outward-rounded interval arithmetic bound actual world
lane curvature and its derivative w.r.t. LANE arclength. Finite resolution is
UNKNOWN, not PASS. Float coefficient construction is not a formal proof of
the original source geometry; keep that distinction in the report.
"""
from dataclasses import dataclass
from math import comb, factorial

import numpy as np

from spikes.road_boundary_family import world_kinematics


@dataclass(frozen=True)
class Interval:
    lo: float
    hi: float

    @staticmethod
    def point(x):
        return Interval(float(x), float(x))

    @staticmethod
    def outward(lo, hi):
        return Interval(float(np.nextafter(lo, -np.inf)), float(np.nextafter(hi, np.inf)))

    def __add__(self, other):
        b = other if isinstance(other, Interval) else self.point(other)
        return self.outward(self.lo+b.lo, self.hi+b.hi)

    __radd__ = __add__

    def __neg__(self):
        return Interval(-self.hi, -self.lo)

    def __sub__(self, other):
        return self + (-other if isinstance(other, Interval) else -float(other))

    def __rsub__(self, other):
        return -self + other

    def __mul__(self, other):
        b = other if isinstance(other, Interval) else self.point(other)
        v = [self.lo*b.lo, self.lo*b.hi, self.hi*b.lo, self.hi*b.hi]
        return self.outward(min(v), max(v))

    __rmul__ = __mul__

    def __truediv__(self, other):
        b = other if isinstance(other, Interval) else self.point(other)
        if b.lo <= 0 <= b.hi:
            raise ArithmeticError('interval denominator includes zero')
        return self*self.outward(1/b.hi, 1/b.lo)

    def square(self):
        lo = 0. if self.lo <= 0 <= self.hi else min(self.lo**2, self.hi**2)
        return self.outward(lo, max(self.lo**2, self.hi**2))

    def sqrt(self):
        if self.lo < 0:
            raise ArithmeticError('negative interval square root')
        return self.outward(np.sqrt(self.lo), np.sqrt(self.hi))

    def maximum_abs(self):
        return max(abs(self.lo), abs(self.hi))


def bernstein_controls(power):
    """Ascending power coefficients on u in [0,1], no degree change."""
    p = np.asarray(power, float)
    n = len(p)-1
    controls = np.array([sum(p[j]*comb(i,j)/comb(n,j) for j in range(i+1)) for i in range(n+1)])
    # Accounts conservatively for the short power->Bernstein summation, but
    # does not certify upstream B-spline evaluation/coefficient formation.
    padding = 128*np.finfo(float).eps*max(1., float(np.sum(abs(p))))
    return controls, padding


def split_controls(c):
    levels = [np.asarray(c)]
    while len(levels[-1]) > 1:
        levels.append((levels[-1][:-1]+levels[-1][1:])*.5)
    return np.array([x[0] for x in levels]), np.array([x[-1] for x in levels[::-1]])


def _world_bounds(jets, k, kp):
    t, d1, d2, d3 = jets
    a, b = 1-k*t, d1
    ap, bp = -kp*t-k*d1, d2
    c, d = -kp*t-2*k*d1, k*a+d2
    cp, dp = -3*kp*d1-2*k*d2, kp*a+k*ap+d3
    q = a.square()+b.square()
    n = a*d-b*c
    nd = ap*d+a*dp-bp*c-b*cp
    qd = 2*(a*ap+b*bp)
    return a, n/(q*q.sqrt()), (nd*q-1.5*n*qd)/(q*q*q)


def audit_span(coefficients, length, curvature, sharpness, speed_kmh, *,
               limits=(2.5, 1.), max_depth=12, minimum_check_span=1e-6):
    """Coefficients t(ds)=a+b ds+c ds²+d ds³ at the EXACT span start.

    v²*k and v³*dk/dl are constant-speed lateral-demand diagnostics. The
    second is not total inertial jerk and includes no speed/longitudinal
    acceleration controller. Limits are unchanged project research targets,
    not an automotive safety standard or an approved turning design speed.
    """
    co = np.asarray(coefficients, float)
    if (co.shape != (4,) or not np.isfinite(co).all() or
            not np.isfinite([length, curvature, sharpness, speed_kmh]).all() or
            length <= 0 or speed_kmh <= 0 or tuple(limits) != (2.5, 1.) or
            not isinstance(max_depth, int) or not 0 <= max_depth <= 18 or
            not np.isfinite(minimum_check_span) or minimum_check_span <= 0):
        raise ValueError('finite cubic/span/source speed and unchanged research limits required')
    polys = [np.polynomial.polynomial.polyder(co, j) for j in range(4)]
    controls, pads = zip(*(bernstein_controls(p*length**np.arange(len(p))) for p in polys))
    curvature_controls, curvature_pad = bernstein_controls([curvature, sharpness*length])
    controls = tuple(controls)+(curvature_controls,)
    pads = tuple(pads)+(curvature_pad,)
    scales = np.array([(speed_kmh/3.6)**2, (speed_kmh/3.6)**3])
    leaves = []; worst = [None, None]; evaluations = 0
    # A common source-domain interval can be tiny without introducing a new
    # polynomial. Keep it, but do not call ill-conditioned extraction PASS.
    unresolved_input = length <= minimum_check_span
    stack = [(0., 1., controls, 0)]
    while stack:
        lo, hi, cc, depth = stack.pop()
        u = np.array([lo, (lo+hi)/2, hi]); ss = length*u
        jets = np.stack([np.polynomial.polynomial.polyval(ss,p) for p in polys], axis=-1)
        kval = curvature+sharpness*ss; regular = 1-kval*jets[:,0]
        if np.min(regular) <= .2:
            leaves.append(dict(u=[lo,hi], status='FAIL', reason='nonregular-world-lane',
                               witness_u=float(u[np.argmin(regular)])))
            continue
        val = abs(world_kinematics(jets, kval, sharpness))*scales
        evaluations += len(ss)
        for j in range(2):
            i = int(np.argmax(val[:,j])); row = dict(value=float(val[i,j]), u=float(u[i]))
            if worst[j] is None or row['value'] > worst[j]['value']: worst[j] = row
        if np.any(val > np.asarray(limits)*(1+1e-10)):
            idx = np.unravel_index(np.argmax(val/limits),val.shape)
            leaves.append(dict(u=[lo,hi], status='FAIL', reason='actual-point-dynamic-exceedance',
                metric=('ay_mps2','lateral_rate_mps3')[idx[1]], witness_u=float(u[idx[0]]),
                witness_value=float(val[idx]), limit=limits[idx[1]]))
            continue
        ii = [Interval.outward(float(min(c))-pad*(depth+1), float(max(c))+pad*(depth+1))
              for c,pad in zip(cc,pads)]
        upper = None
        try:
            a, k, dk = _world_bounds(ii[:4],ii[4],Interval.point(sharpness))
            if a.lo > .2:
                upper = np.array([k.maximum_abs(),dk.maximum_abs()])*scales
        except ArithmeticError:
            pass
        if not unresolved_input and upper is not None and np.all(upper <= limits):
            leaves.append(dict(u=[lo,hi],status='BOUNDED',upper=upper.tolist()))
        elif depth == max_depth or length*(hi-lo) <= minimum_check_span:
            leaves.append(dict(u=[lo,hi],status='UNKNOWN',reason='interval-or-extraction-resolution',
                               upper=upper.tolist() if upper is not None else None))
        else:
            halves = [split_controls(c) for c in cc]; mid = (lo+hi)/2
            stack.append((mid,hi,tuple(h[1] for h in halves),depth+1))
            stack.append((lo,mid,tuple(h[0] for h in halves),depth+1))
    states = {x['status'] for x in leaves}
    status = 'FAIL' if 'FAIL' in states else 'UNKNOWN' if 'UNKNOWN' in states else 'BOUNDED'
    return dict(status=status, length_m=float(length), speed_kmh=float(speed_kmh), limits=list(limits),
        observed_maxima=worst, interval_leaves=leaves, evaluations=evaluations,
        all_parameter_intervals_accounted=abs(sum(r['u'][1]-r['u'][0] for r in leaves)-1)<1e-12,
        bound_scope='float-model one-sided smooth span; guarded Bernstein intervals',
        formal_source_certificate=False, joins_checked=False, geometry_segments_added=0, export_allowed=False)


def parent_spans(state):
    """All original physical midpoint domains, including ordinary transitions."""
    from mapforge.ops.source_transition_domains import midpoint_domains
    model, x = state['model'], state['coefficients']
    for domain in midpoint_domains(model):
        left, right, a, b = [domain[k] for k in ('left','right','a','b')]
        if b <= a: raise ValueError('nonpositive physical source midpoint domain')
        families = [model.families[model.owner[k]] for k in (left,right)]
        cuts = np.unique(np.concatenate(([a,b],*[f.knots[(f.knots>a)&(f.knots<b)] for f in families])))
        for lo, hi in zip(cuts[:-1],cuts[1:]):
            # Midpoint Taylor expansion selects the correct polynomial on
            # BOTH sides of a knot. Evaluating derivatives at lo alone can
            # select the other span and silently miss a jerk jump.
            mid = (lo+hi)/2
            c = np.array([.5*(model.expression(left,mid,j)+model.expression(right,mid,j))@x/factorial(j)
                          for j in range(4)])
            translated = np.polynomial.Polynomial(c)(np.polynomial.Polynomial([lo-mid,1.])).coef
            yield dict(identity=dict(road=model.road,source_lanes=domain['source_lanes'],
                                     domain_kind=domain['domain_kind'], source_lane=domain['source_lane']),
                start=float(lo), end=float(hi), coefficients=np.pad(translated,(0,4-len(translated))),
                curvature=model.reference_curvature,sharpness=0.,speed_kmh=domain['source_speed_kmh'])


def turn_spans(turn):
    """Exact intersections of existing reference and width spans, no refit."""
    cls, knots, co = turn['result'][:3]
    refs = np.r_[0.,np.cumsum([c.length for c in cls])]
    knots = np.asarray(knots,float)
    if (knots[0] != 0 or abs(knots[-1]-refs[-1])>1e-8 or np.any(np.diff(knots)<=0)):
        raise ValueError('complete ordered turn width domain required')
    cuts = np.unique(np.r_[refs,knots]); cuts = cuts[cuts<=refs[-1]]
    for lo,hi in zip(cuts[:-1],cuts[1:]):
        mid=(lo+hi)/2
        i=min(int(np.searchsorted(refs,mid,side='right')-1),len(cls)-1)
        j=min(int(np.searchsorted(knots,mid,side='right')-1),len(knots)-2)
        center=.5*(np.asarray(co['left'][j])+np.asarray(co['right'][j]))
        p=np.polynomial.Polynomial(center)(np.polynomial.Polynomial([lo-knots[j],1.])).coef
        yield dict(start=float(lo),end=float(hi),coefficients=np.pad(p,(0,4-len(p))),
            curvature=float(cls[i].KappaStart+cls[i].dk*(lo-refs[i])), sharpness=float(cls[i].dk))
