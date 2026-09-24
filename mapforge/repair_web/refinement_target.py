"""R2 single-variable full-guard solve on the admitted R1 representation.

The position is an exact affine equality. No new knots, layout/seed search,
proxy derivative caps, or relaxed source/shape guards. Failure is retained;
only a fully checked state may reach the mechanical writer.
"""
from dataclasses import asdict
import math
import time

import numpy as np
from scipy.optimize import minimize

from .local_refinement import LO, HI, EDGE, CUTS, _shift, _cuts
from .model import digest, json_bytes, parse, extrema
from .outer_event_shape import wrong_way_distance
from .unique_edit import METRICS, METRIC_EPS
from mapforge.validate.line_boundary_shape import span_shape, written_boundary_shape
from scripts.check_outer_event_shape import written_shape
from scripts.check_outer_event_written import boundary_poly


class BudgetEnded(RuntimeError):
    pass


def target_line(handle, displacement):
    h = np.asarray(handle, float)
    if (h.shape != (2,) or not np.isfinite(h).all() or np.linalg.norm(h) < 1e-10
            or type(displacement) not in (int, float) or not math.isfinite(displacement)
            or abs(displacement) > 2):
        raise ValueError('Finite responsive two-dimensional handle and original +/-2m target cap')
    return h*displacement/(h@h), np.array([-h[1], h[0]])/np.linalg.norm(h)


def affine_interval(base, direction, lower, upper):
    """Intersect scalar linear guards; retain witnesses, not a generic no-solution claim."""
    a, d, low, high = map(lambda v: np.asarray(v, float), (base, direction, lower, upper))
    if not (a.ndim == 1 and a.shape == d.shape == low.shape == high.shape
            and np.isfinite(a).all() and np.isfinite(d).all()
            and not np.isnan(low).any() and not np.isnan(high).any() and np.all(low <= high)):
        raise ValueError('Invalid scalar guards')
    lo, hi, lo_id, hi_id = -np.inf, np.inf, None, None
    for i, (value, slope, l, h) in enumerate(zip(a, d, low, high)):
        if slope == 0:
            if not l <= value <= h:
                return dict(feasible=False, fixed_conflict_row=i, lower=None, upper=None)
            continue
        p, q = sorted(((l-value)/slope, (h-value)/slope))
        if p > lo: lo, lo_id = float(p), i
        if q < hi: hi, hi_id = float(q), i
    return dict(feasible=lo <= hi, lower=lo, upper=hi, lower_row=lo_id, upper_row=hi_id)


def abs_extremum(c, length):
    roots = np.polynomial.polynomial.polyroots([c[1], 2*c[2], 3*c[3]])
    points = [0., length]+[float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < length]
    at = max(points, key=lambda s: abs(np.polynomial.polynomial.polyval(s, c)))
    return abs(float(np.polynomial.polynomial.polyval(at, c))), at


def bernstein_rows(length):
    L = length
    return np.array([[1, 0, 0, 0], [1, L/3, 0, 0],
                     [1, 2*L/3, L*L/3, 0], [1, L, L*L, L**3]])


class RefinementProblem:
    """Frozen affine polynomial cells, all rebuilt from R1 + original observations."""
    def __init__(self, event, control, model):
        model._verify()
        if (model.reference != control.reference or model.baseline != event.data
                or model.contract['source_packet_sha256'] != digest(json_bytes(event.packet))):
            raise ValueError('R1/source binding mismatch')
        self.event, self.control, self.model = event, control, model
        self.scope = asdict(event.scope)
        self.caps = written_boundary_shape(model.reference, self.scope)
        self.reversal_caps = written_shape(model.reference, event)['by_edge']
        self.source_tolerance = event.scope.source_tolerance_m
        self.spans, self.sources, self.widths, self.linear = [], [], [], []
        self.road = parse(model.reference).find("road[@id='11']")
        full_cuts = sorted(set(_cuts(self.road)) | set(event.scope.knots) | set(CUTS))

        def cell(edge, lo, hi):
            c = model.power(edge, float(lo), [0., 0.])
            D = np.column_stack([model.delta_power(float(lo), v) if edge == EDGE else np.zeros(4)
                                 for v in ([1., 0.], [0., 1.])])
            return dict(edge=edge, lo=float(lo), hi=float(hi), c=c, D=D)

        self.handle = np.array([model.delta_power(control.point.station, v)[0]
                                for v in ([1., 0.], [0., 1.])])
        for edge in range(5):
            start = event.births.get(edge, event.start)
            ss = sorted(s for s in set(full_cuts) | {start, event.end} if start <= s <= event.end)
            self.spans.extend(cell(edge, a, b) for a, b in zip(ss, ss[1:]))
        # Use all original source identities/segments, not selected vertices.
        for trace, lo, hi, source in event.source_cells():
            for s in np.linspace(lo, hi, max(3, math.ceil((hi-lo)/2)+1)):
                cp = cell(trace['edge'], float(s), float(s))
                source_t = np.polynomial.polynomial.polyval(s-lo, source)
                self.linear.append(dict(c=float(cp['c'][0]-source_t), D=cp['D'][0], low=-self.source_tolerance,
                                        high=self.source_tolerance, kind='original_sampled_source', key=trace['key'], s=float(s)))
            ss = sorted({float(lo), float(hi)} | {s for s in full_cuts if lo < s < hi})
            for a, b in zip(ss, ss[1:]):
                cp = cell(trace['edge'], a, b)
                cp.update(source=_shift(source, a-lo), key=trace['key'], record=trace['record'], part=trace['part'])
                self.sources.append(cp)
        # Same nonnegative Bernstein guard, on genuine new independent spans;
        # mandatory XML section cuts are not extra freedom/guard subdivision.
        long_cuts = sorted(set(event.scope.knots) | set(CUTS))
        for edge in range(1, 5):
            start = event.births.get(edge, event.start)
            ss = [s for s in long_cuts if start <= s <= event.end]
            for a, b in zip(ss, ss[1:]):
                inner, outer = cell(edge-1, a, b), cell(edge, a, b)
                cp = dict(edge=edge, lo=a, hi=b, c=inner['c']-outer['c'], D=inner['D']-outer['D'])
                self.widths.append(cp)
                T = bernstein_rows(b-a)
                for j, (c, row) in enumerate(zip(T@cp['c'], T@cp['D'])):
                    self.linear.append(dict(c=float(c), D=row, low=0., high=np.inf,
                                            kind='original_width_bernstein_new_representation', lane=-edge,
                                            interval_m=[a, b], control=j))
        self.A = np.array([r['D'] for r in self.linear]); self.a = np.array([r['c'] for r in self.linear])
        self.low = np.array([r['low'] for r in self.linear]); self.high = np.array([r['high'] for r in self.linear])
        nodes, weights = np.polynomial.legendre.leggauss(4)
        rows = []
        for cp in self.spans:
            if cp['edge'] != EDGE: continue
            L = cp['hi']-cp['lo']
            for d, w in zip((nodes+1)*L/2, weights*L/2):
                rows.extend([np.array([1., d, d*d, d**3])@cp['D']*np.sqrt(w),
                             np.array([0., 1., 2*d, 3*d*d])@cp['D']*(10*np.sqrt(w))])
        M = np.asarray(rows); self.H = M.T@M

    def evaluate(self, state):
        u = np.asarray(state, float)
        if u.shape != (2,) or not np.isfinite(u).all(): raise ValueError('Invalid state')
        source_rows, widths, reversal = [], [], np.zeros(5)
        shape_rows = [[] for _ in range(5)]
        for cp in self.sources:
            c = cp['c']+cp['D']@u; L = cp['hi']-cp['lo']
            error, at = abs_extremum(c-cp['source'], L)
            source_rows.append(dict(key=cp['key'], record=cp['record'], part=cp['part'], edge=cp['edge'],
                                    interval_m=[cp['lo'], cp['hi']], max_m=error, witness_s=cp['lo']+at))
            reversal[cp['edge']] += wrong_way_distance(c, L, float(np.sign(cp['source'][1])))
        for cp in self.widths:
            c = cp['c']+cp['D']@u; L = cp['hi']-cp['lo']
            widths.append(dict(lane=-cp['edge'], interval_m=[cp['lo'], cp['hi']],
                               minimum_m=extrema(c, L), bernstein_min_m=float(min(bernstein_rows(L)@c))))
        for cp in self.spans:
            shape_rows[cp['edge']].append(span_shape(cp['c']+cp['D']@u, cp['hi']-cp['lo']))
        shapes = []
        for edge, rows in enumerate(shape_rows):
            jumps = sum(abs(a['curvature_end']-b['curvature_start']) for a, b in zip(rows, rows[1:]))
            shapes.append(dict(edge=edge, max_abs_curvature=max(r['max_abs_curvature'] for r in rows),
                               max_abs_curvature_rate=max(r['max_abs_curvature_rate'] for r in rows),
                               curvature_total_variation=sum(r['curvature_variation'] for r in rows)+jumps))
        failures, margins = [], []
        def upper(code, origin, value, cap, eps, scale):
            margins.append((cap+eps-value)/scale)
            if value > cap+eps:
                failures.append(dict(code=code, origin=origin, observed=float(value), limit=float(cap), epsilon=eps))
        for row in source_rows:
            upper('SOURCE_ENVELOPE', 'existing_L01_source_policy', row['max_m'], self.source_tolerance, 1e-7, 1.)
        for row in widths:
            upper('NEGATIVE_WIDTH', 'physical_width', -row['minimum_m'], 0., 1e-7, 1.)
            upper('WIDTH_BERNSTEIN', 'existing_conservative_width_guard', -row['bernstein_min_m'], 0., 1e-7, 1.)
        linear = self.a+self.A@u
        linear_violation = float(max(0., np.max(self.low-linear), np.max(linear-self.high)))
        upper('ORIGINAL_LINEAR_GUARD', 'existing_sampled_source_and_width', linear_violation, 0., 1e-7, 1.)
        for edge, shape in enumerate(shapes):
            for key, eps in zip(METRICS, METRIC_EPS):
                cap = self.caps['by_edge'][str(edge)][key]
                upper(f'EDGE_{edge}_{key}', 'research_reference_nonregression', shape[key], cap, float(eps), max(cap, 1e-8))
            cap = self.reversal_caps[str(edge)]
            upper(f'EDGE_{edge}_SOURCE_DIRECTION_REVERSAL', 'research_reference_nonregression', reversal[edge], cap, 1e-7, max(cap, .01))
        return dict(source=dict(max_m=max(r['max_m'] for r in source_rows), rows=source_rows), widths=widths,
                    shape_by_edge=shapes, reversal_by_edge_m=reversal.tolist(), linear_violation=linear_violation,
                    failures=failures, margins=np.asarray(margins), map_accepted=False)

    def compile_checked(self, state, displacement):
        """No use of the permissive diagnostic entry point to approve a shape."""
        report = self.evaluate(state)
        if report['failures'] or abs(float(self.handle@state)-displacement) > 1e-8:
            raise ValueError('Full source/width/shape/target gates must pass BEFORE any XML serialization')
        # Only after the R2 gate may the already-tested mechanical writer run.
        data = self.model._serialize_probe(state)
        readback = self.model.audit_compiled(data, state)
        actual = written_boundary_shape(data, self.scope)
        reversal = written_shape(data, self.event)
        target = boundary_poly(parse(data).find("road[@id='11']"), EDGE, self.control.point.station)[0]
        if abs(target-self.control.reference_t-displacement) > 1e-8:
            raise ValueError('Actual XML target mismatch')
        for edge in range(5):
            for key, eps in zip(METRICS, METRIC_EPS):
                if actual['by_edge'][str(edge)][key] > self.caps['by_edge'][str(edge)][key]+eps:
                    raise ValueError('Actual XML shape guard failed')
            if reversal['by_edge'][str(edge)] > self.reversal_caps[str(edge)]+1e-7:
                raise ValueError('Actual XML reversal guard failed')
        readback.update(r2_shape=actual, r2_reversal=reversal,
                        r2_target_error_m=float(abs(target-self.control.reference_t-displacement)),
                        purpose='R2_FULL_GUARDS_NOT_MAP', user_target_checked=True)
        return data, readback


def solve_once(problem, displacement, *, started=None, progress=None):
    """One SLSQP call / one deterministic initial state / 160 iterations / 90s."""
    started = time.monotonic() if started is None else started
    def budget():
        if time.monotonic()-started > 90: raise BudgetEnded('Total R2 budget exhausted')
    budget()
    particular, direction = target_line(problem.handle, displacement)
    domain = affine_interval(problem.a+problem.A@particular, problem.A@direction,
                             problem.low-1e-7, problem.high+1e-7)
    if not domain['feasible']:
        return None, dict(status='R2_AFFINE_POLICY_CONFLICT', domain=domain, optimizer_calls=0,
                          iterations=0, evaluations=0, real_target_candidates=1,
                          displacement_m=displacement, particular=particular.tolist(), direction=direction.tolist(),
                          elapsed_seconds=time.monotonic()-started, automatic_retry_allowed=False,
                          map_accepted=False), None
    if not math.isfinite(domain['lower']) or not math.isfinite(domain['upper']):
        raise ValueError('Unbounded scalar domain: no arbitrary search bounds are invented')
    H = problem.H; hz = float(direction@H@direction)
    if hz <= 0: raise ValueError('No positive deformation objective')
    initial = float(np.clip(-direction@H@particular/hz, domain['lower'], domain['upper']))
    cache, history = {}, []
    calls = 0; last_z = initial; optimizer_calls = 0
    def count_check():
        nonlocal calls
        budget()
        if calls >= 600: raise BudgetEnded('Registered evaluation cap exhausted')
        calls += 1
    def evaluate(z):
        budget(); z = float(z[0])
        if cache.get('z') != z:
            count_check()
            result = problem.evaluate(particular+direction*z)
            budget(); cache.update(z=z, result=result)
        return cache['result']
    def callback(z):
        nonlocal last_z
        last_z = float(z[0]); result = evaluate(z)
        row = dict(iteration=len(history)+1, scalar=last_z,
                   normalized_violation=float(max(0., -min(result['margins']))))
        history.append(row)
        if progress and (len(history) == 1 or len(history)%10 == 0): progress(row)
    factor = max(1., hz)
    def objective(z):
        budget(); u = particular+direction*z[0]
        return float(.5*u@H@u/factor)
    def gradient(z):
        budget(); return np.array([direction@H@(particular+direction*z[0])/factor])
    status = 'R2_REJECTED'; solver = None; data = None; readback = None
    try:
        if domain['lower'] == domain['upper']:
            solver = dict(success=False, message='Singleton linear domain: direct guard check only', iterations=0)
        else:
            optimizer_calls = 1
            fit = minimize(objective, [initial], jac=gradient, method='SLSQP',
                           bounds=[(domain['lower'], domain['upper'])],
                           constraints=[dict(type='ineq', fun=lambda z: evaluate(z)['margins'])],
                           callback=callback, options=dict(maxiter=160, ftol=1e-10, eps=1e-6))
            last_z = float(fit.x[0])
            solver = dict(success=bool(fit.success), message=str(fit.message), iterations=int(fit.nit))
        budget()
        state = particular+direction*last_z
        result = evaluate([last_z]); budget()
        if not result['failures']:
            count_check()  # compile_checked independently repeats the full state gate
            data, readback = problem.compile_checked(state, displacement); budget()
            status = 'R2_LOCAL_GUARDS_PASS_NOT_MAP'
    except BudgetEnded as exc:
        status = 'R2_BUDGET_EXHAUSTED'; data = None
        solver = dict(success=False, message=str(exc), iterations=len(history))
        state = particular+direction*last_z
        result = cache.get('result') if cache.get('z') == last_z else None
    except ValueError as exc:
        status = 'R2_READBACK_OR_NUMERIC_REJECTED'; data = None
        solver = dict(success=False, message=str(exc), iterations=len(history))
        state = particular+direction*last_z
        result = cache.get('result') if cache.get('z') == last_z else None
    report = dict(status=status, displacement_m=displacement, target_error_m=abs(float(problem.handle@state)-displacement),
                  particular=particular.tolist(), direction=direction.tolist(), scalar=last_z,
                  initial_scalar=initial, domain=domain, solver=solver, history=history,
                  optimizer_calls=optimizer_calls, evaluations=calls, iterations=solver['iterations'],
                  elapsed_seconds=time.monotonic()-started, readback=readback,
                  guard_evaluation=None if result is None else {k:v for k,v in result.items() if k != 'margins'},
                  map_accepted=False, operating_dynamics='NOT_EVALUATED', formal_certificate=False,
                  automatic_retry_allowed=False, real_target_candidates=1)
    return state, report, data
