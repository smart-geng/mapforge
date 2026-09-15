"""An exact line/arc coordinate chart and full-segment source enclosures.

Certificate cells are integration/checking intervals, NOT exported geometry.
The input is a measured polyline, never a previously fitted target curve.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ArcChart:
    origin: tuple
    heading: float
    curvature: float

    def __post_init__(self):
        if np.shape(self.origin) != (2,) or not np.isfinite([*self.origin, self.heading, self.curvature]).all():
            raise ValueError('finite planar arc chart required')

    def frame(self, s):
        s = np.asarray(s, float)
        if not np.isfinite(s).all() or np.any(abs(self.curvature*s) >= np.pi/2):
            raise ValueError('chart outside its single forward half-plane')
        z = self.curvature*s
        # Stable at k=0 and at very large radii, without changing arc to line.
        u = s*np.sinc(z/np.pi)
        v = .5*self.curvature*s*s*np.sinc(z/(2*np.pi))**2
        e0 = np.array([np.cos(self.heading), np.sin(self.heading)])
        n0 = np.array([-e0[1], e0[0]])
        xy = np.asarray(self.origin)+u[..., None]*e0+v[..., None]*n0
        e = np.stack([np.cos(self.heading+z), np.sin(self.heading+z)], axis=-1)
        n = np.stack([-e[..., 1], e[..., 0]], axis=-1)
        return xy, e, n

    def world(self, st):
        st = np.asarray(st, float)
        if st.shape[-1:] != (2,) or not np.isfinite(st).all():
            raise ValueError('finite s/t coordinates required')
        if np.any(1-self.curvature*st[..., 1] <= 0):
            raise ValueError('nonregular offset crosses reference curvature center')
        xy, _, n = self.frame(st[..., 0])
        return xy+st[..., 1, None]*n

    def project(self, xy):
        xy = np.asarray(xy, float)
        if xy.shape[-1:] != (2,) or not np.isfinite(xy).all():
            raise ValueError('finite source XY required')
        e = np.array([np.cos(self.heading), np.sin(self.heading)])
        n = np.array([-e[1], e[0]])
        d = xy-np.asarray(self.origin); u, v = d@e, d@n; k = self.curvature
        if k == 0:
            return np.stack([u, v], axis=-1)
        a, b = k*u, 1-k*v
        if np.any(b <= 0):
            raise ValueError('source outside forward arc chart; no wrapping or sorting')
        s = np.arctan2(a, b)/k
        # (1-hypot(ku,1-kv))/k, rationalized to avoid cancellation.
        t = (2*v-k*(u*u+v*v))/(1+np.hypot(a, b))
        return np.stack([s, t], axis=-1)


class ArcSourceTrace:
    """Each original straight segment is intersected with reference normals."""
    def __init__(self, chart, xy):
        self.chart = chart
        self.xy = np.array(xy, float, copy=True)
        self.st = chart.project(self.xy)
        if self.xy.ndim != 2 or len(self.xy) < 2 or np.any(np.diff(self.st[:, 0]) <= 1e-9):
            raise ValueError('source must be strictly monotone; no sorting or deleting vertices')
        delta = np.diff(self.xy, axis=0); lengths = np.linalg.norm(delta, axis=1)
        if np.any(lengths == 0):
            raise ValueError('zero source segment')
        self.normals = np.c_[-delta[:, 1], delta[:, 0]]/lengths[:, None]

    def values(self, segment, s):
        s = np.asarray(s, float)
        a, b = self.st[segment:segment+2, 0]
        if np.any(s < a-1e-8) or np.any(s > b+1e-8):
            raise ValueError('no extrapolation of source segment')
        p, _, n = self.chart.frame(s); normal = self.normals[segment]
        denominator = n@normal
        if np.any(abs(denominator) < .1):
            raise ValueError('near-tangent normal intersection is not a source graph')
        return (self.xy[segment]-p)@normal/denominator

    def chord_error_bound(self, segment, a, b):
        """Bound |t(s)-linear_interpolation(t(a),t(b))| on the WHOLE cell.

        Implicit line intersection gives t''=-k(1-kt)(1+2q²),
        q=(source_normal dot tangent)/(source_normal dot normal).
        Lipschitz bounds on the unit frame enclose its denominator and t.
        The linear interpolation remainder is <= sup|t''|*(b-a)^2/8.
        """
        if not np.isfinite([a, b]).all() or b <= a:
            raise ValueError('positive source certificate interval required')
        self.values(segment, [a, b])  # Verify original segment support.
        k = self.chart.curvature
        if k == 0:
            return 0.
        mid = (a+b)/2; half = (b-a)/2
        p, e, n = self.chart.frame(mid); normal = self.normals[segment]
        denominator = abs(float(n@normal))-abs(k)*half
        if denominator <= .1:
            raise ValueError('cannot certify a nonfolding source interval')
        tmax = (abs(float((self.xy[segment]-p)@normal))+half)/denominator
        qmax = min(1., abs(float(e@normal))+abs(k)*half)/denominator
        second = abs(k)*(1+abs(k)*tmax)*(1+2*qmax*qmax)
        return second*(b-a)**2/8

    def cells(self, a, b, knots, step=2.):
        """Yield bounded source cells without modifying knots or raw vertices."""
        if not np.isfinite([a, b, step]).all() or step <= 0 or b <= a:
            raise ValueError('finite source support and positive certificate step required')
        if a < self.st[0, 0]-1e-8 or b > self.st[-1, 0]+1e-8:
            raise ValueError('source certificate outside raw support')
        knots = np.asarray(knots, float)
        for i, (sa, sb) in enumerate(zip(self.st[:-1, 0], self.st[1:, 0])):
            lo, hi = max(sa, a), min(sb, b)
            if hi <= lo:
                continue
            cuts = np.unique(np.r_[lo, np.arange(lo, hi, step), knots[(knots > lo)&(knots < hi)], hi])
            for ca, cb in zip(cuts[:-1], cuts[1:]):
                va, vb = self.values(i, [ca, cb])
                yield float(ca), float(cb), float(va), float(vb), self.chord_error_bound(i, ca, cb)
