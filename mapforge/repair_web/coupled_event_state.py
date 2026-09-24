"""Seven-road coordinates for the reviewed event, NOT a fitter or writer.

Five long reference primitives per turn and one cubic transverse interval per
primitive. Unlike the older joint kernel there are no half-cap width knots.
Actual world-edge/center joins remain explicit constraints, not hidden fits.
Source, shape and surface admission is still required before any trial.
"""
import math

import numpy as np
from scipy.interpolate import BSpline

from mapforge.ops.endpoint_jet_coordinates import complete_endpoint_coefficients
from mapforge.ops.long_connector_chain import chain
from mapforge.ops.source_connector_ribbon import minimum_forward_factor
from mapforge.repair_web.outer_event import cubic_range
from mapforge.repair_web.split_event_admission import END, CONNECTORS
from mapforge.validate.smoothness import _edge_world_curvature
from spikes.connector_cross_section import edge_jet


def long_basis(lengths):
    lengths = np.asarray(lengths, float)
    if lengths.shape != (5,) or not np.isfinite(lengths).all() or min(lengths) < 6.:
        raise ValueError('Exactly five finite >=6m independent reference/width spans required')
    stations = np.r_[0., np.cumsum(lengths)]
    # C1 is intentional: transverse second derivatives compensate reference
    # sharpness jumps. WORLD G2 of both boundaries and center is checked below.
    knots = np.r_[[0.]*4, np.repeat(stations[1:-1]/stations[-1], 2), [1.]*4]
    return stations, BSpline(knots, np.eye(len(knots)-4), 3, extrapolate=False)


def world_state(ref, co, u):
    t = np.polynomial.polynomial.polyval(u, co)
    dt = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(co))
    ddt = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(co, 2))
    k = ref.KappaStart+ref.dk*u; h = ref.Theta(u); A = 1-k*t
    if A <= .1: raise ValueError('Non-forward long ribbon frame')
    return np.array([ref.X(u)-t*math.sin(h), ref.Y(u)+t*math.cos(h),
                     h+math.atan2(dt, A), _edge_world_curvature(t, dt, ddt, k, ref.dk)])


def difference(a, b):
    d = np.asarray(a)-np.asarray(b)
    d[2] = math.atan2(math.sin(d[2]), math.cos(d[2]))
    return d


class EventTurnBlock:
    """19 variables: five lengths, two curvatures and 12 free width controls."""
    nvar = 19

    def __init__(self, road, frames):
        self.road = road.get('id')
        total = float(road.get('length'))
        lengths = np.full(5, total/5.)
        _, basis = long_basis(lengths)
        from mapforge.validate.smoothness import _geoms
        geoms = _geoms(road)
        starts = np.r_[0., np.cumsum([g[4] for g in geoms])]
        def old_k(s):
            i = min(len(geoms)-1, int(np.searchsorted(starts, s, side='right')-1)); g = geoms[i]
            return g[5]+(g[6]-g[5])*(s-starts[i])/g[4]
        # A diagnostic coordinate, NOT a projection or fit of the old shape.
        # Only inherited total length/curvature and fixed cross-section widths
        # define it. Its reference closure residual is retained, never polished.
        q = np.r_[lengths, 20*old_k(total*.4), 20*old_k(total*.6)]
        free = []
        for side in ('left', 'right'):
            vals = []
            for frame in frames:
                x, y, h = frame['pose']; e = frame['edges'][side]
                vals.append(-(e['x']-x)*math.sin(h)+(e['y']-y)*math.cos(h))
            free.extend(np.linspace(*vals, len(basis.c))[3:-3])
        self.diagnostic = np.r_[q, free]
        if self.diagnostic.shape != (self.nvar,): raise ValueError('Unexpected long-ribbon dimension')

    def evaluate(self, values, frames):
        v = np.asarray(values, float)
        if v.shape != (self.nvar,) or not np.isfinite(v).all(): raise ValueError('Complete finite turn state required')
        stations, basis = long_basis(v[:5]); L = stations[-1]
        a, b = frames; refs = chain(v[:7], a, b, (True, True), 3)
        coefficients = {}; controls = {}
        for i, side in enumerate(('left', 'right')):
            targets = np.r_[edge_jet(a, a['edges'][side], refs[0].KappaStart, refs[0].dk),
                            edge_jet(b, b['edges'][side], refs[-1].KappaEnd, refs[-1].dk)]
            c = complete_endpoint_coefficients(basis, L, v[7+6*i:13+6*i], targets)
            controls[side] = c
            coefficients[side] = np.array([[basis(s/L, d)@c/L**d/math.factorial(d)
                                           for d in range(4)] for s in stations[:-1]])
        joins = []; contacts = []
        for side in ('left', 'right', 'center'):
            co = coefficients[side] if side != 'center' else (coefficients['left']+coefficients['right'])/2
            for i in range(4):
                delta = difference(world_state(refs[i+1], co[i+1], 0.), world_state(refs[i], co[i], refs[i].length))
                joins.append(dict(field=side, station_m=float(stations[i+1]), delta=delta.tolist()))
            for index, frame, u in ((0, a, 0.), (4, b, refs[-1].length)):
                target = frame['center'] if side == 'center' else frame['edges'][side]
                actual = world_state(refs[index], co[index], u)
                delta = difference(actual, [target[k] for k in ('x', 'y', 'heading', 'curvature')])
                contacts.append(dict(field=side, end='start' if index == 0 else 'end', delta=delta.tolist()))
        width = min(cubic_range(c, h)[0] for c, h in zip(coefficients['left']-coefficients['right'], v[:5]))
        forward = minimum_forward_factor(refs, stations, coefficients)
        endpoint = refs[-1]
        reference_closure = difference([endpoint.XEnd, endpoint.YEnd, endpoint.ThetaEnd, endpoint.KappaEnd],
                                       [*b['pose'], b['k']])
        return dict(refs=refs, stations=stations, basis=basis, coefficients=coefficients, controls=controls,
            joins=joins, contacts=contacts, reference_closure=reference_closure,
            minimum_width_m=float(width), minimum_forward_factor=float(forward),
            source_fidelity_evaluated=False, shape_admitted=False, map_accepted=False)


class CoupledEventState:
    """All dependent endpoints are derived from the same parent coordinates."""
    def __init__(self, contract):
        self.contract = contract; p = contract.parent
        # Algebraic interface diagnostic: preserve old mouth jets at this ONE
        # coordinate, without pretending its interior is the original road.
        targets = np.array([p.old_power(e, END, True)[d]*math.factorial(d)
                            for e in range(5) for d in (1, 2)])
        y = np.linalg.lstsq(contract.jet_matrix@p.Z, targets-contract.jet_matrix@p.origin, rcond=None)[0]
        x = p.origin+p.Z@y
        if np.max(abs(contract.jet_matrix@x-targets)) > 1e-8: raise ValueError('Mouth diagnostic rank failure')
        self.parent_slice = slice(0, len(y)); self.slices = {}; self.turns = {}
        frames = contract.frames(x); values = list(y)
        for cid in CONNECTORS:
            turn = EventTurnBlock(contract.graph.roads[cid], frames[cid]); self.turns[cid] = turn
            self.slices[cid] = slice(len(values), len(values)+turn.nvar); values.extend(turn.diagnostic)
        self.diagnostic = np.asarray(values); self.nvar = len(values)

    def evaluate(self, vector):
        v = np.asarray(vector, float)
        if v.shape != (self.nvar,) or not np.isfinite(v).all(): raise ValueError('Complete finite seven-road state required')
        p = self.contract.parent; x = p.origin+p.Z@v[self.parent_slice]
        frames = self.contract.frames(x)
        return dict(parent_coefficients=x, frames=frames,
            turns={cid: self.turns[cid].evaluate(v[self.slices[cid]], frames[cid]) for cid in CONNECTORS},
            solver_calls=0, source_fidelity_evaluated=False, shape_admitted=False, export_allowed=False)

    def solve(self, *args, **kwargs):
        raise ValueError('Full source/shape/surface/layout admission and registered single-trial budget required')

    compile = solve
