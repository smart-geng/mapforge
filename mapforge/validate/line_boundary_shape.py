"""Actual-XML Line-chart boundary curvature extrema and total variation.

Critical points of the rational curvature functions are found algebraically,
not by a visual sampling grid. Float polynomial roots are numerical evidence,
NOT a formal/source-uncertainty certificate or a dynamics operating case.
"""
import math
import numpy as np
from numpy.polynomial import Polynomial as P

from mapforge.repair_web.model import parse, digest
from scripts.check_outer_event_written import boundary_poly


def _unit_roots(poly):
    return sorted(float(r.real) for r in poly.roots() if abs(r.imag) < 1e-8 and 0 < r.real < 1)


def span_shape(coefficients, length):
    c = np.asarray(coefficients, float)
    if c.shape != (4,) or not np.isfinite(c).all() or not math.isfinite(length) or length <= 0:
        raise ValueError('Finite cubic and positive length required')
    t = P(c*length**np.arange(4))
    d1, d2, d3 = (t.deriv(j)/length**j for j in (1, 2, 3))
    q = 1+d1*d1
    n = d3*q-3*d1*d2*d2
    def k(u): return d2(u)/q(u)**1.5
    def rate(u): return n(u)/q(u)**3  # d curvature / d world boundary arclength
    extrema_k = [0., *_unit_roots(n), 1.]
    extrema_rate = [0., *_unit_roots(n.deriv()*q-3*n*q.deriv()), 1.]
    ks, rs = np.array(k(np.array(extrema_k))), np.array(rate(np.array(extrema_rate)))
    if not np.isfinite(ks).all() or not np.isfinite(rs).all(): raise ValueError('Nonfinite world geometry')
    return dict(max_abs_curvature=float(max(abs(ks))), max_abs_curvature_rate=float(max(abs(rs))),
                curvature_variation=float(sum(abs(np.diff(ks)))),
                curvature_start=float(k(0.)), curvature_end=float(k(1.)))


def written_boundary_shape(data, scope, *, count=4):
    road = next(r for r in parse(data).findall('road') if r.get('id') == scope['road'])
    gs = road.findall('planView/geometry')
    if len(gs) != 1 or gs[0][0].tag != 'line': raise ValueError('Exact Line chart required')
    cuts = {scope['knots'][0], scope['knots'][-1]}
    cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        s = float(sec.get('s')); cuts.add(s)
        cuts.update(s+float(e.get('sOffset')) for e in sec.findall('.//width'))
    births = dict(scope['births']); result = {}
    for edge in range(count+1):
        lo, hi = births.get(edge, scope['knots'][0]), scope['knots'][-1]
        ss = sorted(s for s in cuts | {lo, hi} if lo <= s <= hi)
        rows = [dict(s=[a, b], **span_shape(boundary_poly(road, edge, a), b-a)) for a, b in zip(ss, ss[1:])]
        jumps = sum(abs(a['curvature_end']-b['curvature_start']) for a, b in zip(rows, rows[1:]))
        result[str(edge)] = dict(max_abs_curvature=max(r['max_abs_curvature'] for r in rows),
                                 max_abs_curvature_rate=max(r['max_abs_curvature_rate'] for r in rows),
                                 curvature_total_variation=sum(r['curvature_variation'] for r in rows)+jumps,
                                 join_jump_sum=jumps, intervals=len(rows))
    return dict(xodr_sha256=digest(data), by_edge=result, map_accepted=False, formal_certificate=False,
                method='actual XML cubic critical points in fixed Line chart; no geometry resampling')
