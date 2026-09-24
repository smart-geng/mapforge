"""Exact, replayable control of a source-owned shared-boundary event.

Two bounded support solves provide VERIFIED witnesses, not a claim that every
possible edit has been found. Dragging interpolates their common coefficient
states, not sampled points. Convex source/width/reversal guards hold throughout
this inner interval; every compiled XML is checked again. Outside is rejected,
never clipped or labelled mathematically impossible from a solver timeout.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy import sparse
from scipy.optimize import linprog

from .model import digest, parse
from .outer_event_shape import _convex, reversal_support
from scripts.check_outer_event_written import boundary_poly, check
from scripts.check_outer_event_shape import written_shape


@dataclass(frozen=True)
class ControlPoint:
    source_key: str
    record: int
    part: int
    edge: int
    station: float


def _road(data, event):
    return next(r for r in parse(data).findall('road') if r.get('id') == event.scope.road)


def recover_state(event, data):
    """Recover ONLY a file representable by the registered long common basis.

Compare whole polynomial coefficients at every actual XML cut, not selected
vertices; an inserted short bump cannot hide between recovery samples.
"""
    road = _road(data, event)
    cuts = {event.start, event.end, *event.scope.knots}
    cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        lo = float(sec.get('s'))
        cuts.add(lo)
        cuts.update(lo+float(e.get('sOffset')) for e in sec.findall('.//width'))
    cuts = sorted(s for s in cuts if event.start <= s <= event.end)
    rows, values = [], []
    for edge in range(event.count+1):
        for a, b in zip(cuts, cuts[1:]):
            scale = np.array([1., b-a, (b-a)**2, (b-a)**3])
            rows.extend(event.power(edge, a)*scale[:, None])
            values.extend(boundary_poly(road, edge, a)*scale)
    A, y = np.asarray(rows), np.asarray(values)
    z, _, rank, _ = np.linalg.lstsq(A@event.Z, y-A@event.origin, rcond=None)
    x = event.origin+event.Z@z
    if rank != event.Z.shape[1] or np.max(abs(A@x-y)) > 1e-8:
        raise ValueError('Reference XML is not in the registered common long-curve space')
    event.compile(x)
    verified = check(data, event.data, event.packet, event.scope.__dict__, event.inventory)
    if verified['status'] != 'PASS_LOCAL_EVENT_NOT_MAP':
        raise ValueError('Reference XML failed independent event/source readback')
    return x


class EventControl:
    """One explicitly owned handle; all five boundary states move together.

Values are absolute lateral displacements from this immutable reference SHA,
not incremental pointer deltas. Monotonicity and derivative bounds are local
nonregression guards, NOT full static smoothness or driving acceptance.
"""
    def __init__(self, event, reference, point):
        self.event, self.reference, self.point = event, reference, point
        if (type(point.edge) is not int or not 0 <= point.edge <= event.count
                or isinstance(point.station, bool) or not isinstance(point.station, (int, float))
                or not math.isfinite(point.station)
                or not event.births.get(point.edge, event.start) < point.station < event.end
                or type(point.record) is not int or type(point.part) is not int):
            raise ValueError('Finite station inside an existing physical boundary required')
        matches = [t for t in event.traces if (t['key'], t['record'], t['part'], t['edge']) ==
                   (point.source_key, point.record, point.part, point.edge)
                   and t['st'][0, 0] <= point.station <= t['st'][-1, 0]]
        if len(matches) != 1:
            raise ValueError('Handle must bind one exact original boundary occurrence')
        self.trace = matches[0]
        self.current = recover_state(event, reference)
        self.handle = event.row(point.edge, point.station)
        self.reference_t = float(boundary_poly(_road(reference, event), point.edge, point.station)[0])
        self.reference_sha256 = digest(reference)
        # Budgets come from the actual CURRENT reference file, not an older,
        # worse candidate or an arbitrary editable tolerance.
        shape = written_shape(reference, event)
        self.budgets = np.array([shape['by_edge'][str(i)] for i in range(event.count+1)])
        _, _, self.C, self.low, self.high, source_count, _, _ = event.linear_model()
        self.families = ['source-envelope']*source_count + ['whole-span-width']*(len(self.C)-source_count)
        self._add_derivative_guards()
        self.witnesses = {-1: self.current.copy(), 1: self.current.copy()}
        self.report = None
        if not self._guard(self.current)[0]:
            raise ValueError('Reference does not satisfy the registered control guards')

    def _add_derivative_guards(self):
        # Bound the WHOLE boundary's derivative magnitude. Per-span caps would
        # freeze the old hump's distribution and make moving it nearly
        # impossible. These are conservative linear derivative guards, not
        # a claim of curvature nonregression or a full smoothness certificate.
        event = self.event
        rows, low, high = [], [], []
        self.derivative_guards = []
        for edge, bs in event.splines.items():
            cuts = sorted(set(bs.t))
            edge_rows = {2: [], 3: []}
            for a, b in zip(cuts, cuts[1:]):
                p = event.power(edge, a)
                for derivative, rr in ((2, np.array([2*p[2], 2*p[2]+6*(b-a)*p[3]])),
                                       (3, np.array([6*p[3]]))):
                    edge_rows[derivative].extend(rr)
            for derivative, rr in edge_rows.items():
                cap = float(np.max(abs(np.asarray(rr)@self.current)))
                rows.extend(rr); low.extend([-cap]*len(rr)); high.extend([cap]*len(rr))
                self.derivative_guards.append(dict(edge=edge, s=[float(cuts[0]), float(cuts[-1])],
                                                   derivative=derivative, absolute_cap=cap))
        self.C = np.vstack([self.C, rows])
        self.low = np.r_[self.low, low]; self.high = np.r_[self.high, high]
        self.families.extend(['derivative-nonregression']*len(rows))

    def _guard(self, state):
        event = self.event
        source = event.source_error(state)
        values, support = reversal_support(event, state)
        linear = float(max(0., np.max(self.low-self.C@state), np.max(self.C@state-self.high)))
        equality = float(np.max(abs(event.E@state-event.e)))
        ok = (np.isfinite(state).all() and source['max_m'] <= event.scope.source_tolerance_m+1e-7
              and np.max(values-self.budgets) <= 1e-7 and linear <= 1e-7 and equality <= 1e-7)
        return ok, dict(source_max_m=source['max_m'], reversal_by_edge_m=values.tolist(),
                        linear_violation=linear, equality_residual=equality), source, support

    def prepare_range(self, *, max_rounds=24, progress=None):
        if type(max_rounds) is not int or not 1 <= max_rounds <= 24:
            raise ValueError('Bounded range budget must be 1..24 rounds')
        event = self.event; Z = event.Z
        # Centre at a known verified file to improve scaling at zero-reversal
        # and fixed endpoint constraints. NEVER centre at an unverified trial.
        x0 = self.current
        C = self.C.copy(); low = self.low.copy(); high = self.high.copy()
        C = np.vstack([C, self.handle]); low = np.r_[low, self.reference_t-2.]
        high = np.r_[high, self.reference_t+2.]
        families = self.families + ['interaction-2m-cap']
        supports = [reversal_support(event, x0)[1]]
        n = Z.shape[1]; history = []
        for sign in (-1, 1):
            for iteration in range(max_rounds):
                D = C@Z; lb, ub = low-C@x0, high-C@x0
                flo, fhi = np.isfinite(lb), np.isfinite(ub)
                constraints = [-D[flo], D[fhi]]; rhs = [-lb[flo], ub[fhi]]
                for G in supports:
                    constraints.append(G@Z); rhs.append(self.budgets-G@x0)
                z, info = _convex(sparse.csc_matrix((n, n)), -sign*(self.handle@Z),
                                  sparse.csc_matrix(np.vstack(constraints)), np.concatenate(rhs))
                row = dict(direction=sign, round=iteration, solver=info)
                history.append(row)
                if z is None:
                    row['termination'] = 'UNDETERMINED_KEEP_VERIFIED_REFERENCE'
                    if progress: progress(row)
                    break
                trial = x0+Z@z
                ok, guard, source, support = self._guard(trial)
                delta = float(self.handle@trial-self.reference_t)
                row.update(guard=guard, requested_extreme_m=delta)
                # A bounded solve is not itself a safety/feasibility proof.
                if ok and abs(delta) <= 2.+1e-7:
                    event.compile(trial)
                    if sign*delta > sign*float(self.handle@self.witnesses[sign]-self.reference_t):
                        self.witnesses[sign] = trial
                    row['termination'] = 'VERIFIED_ENDPOINT'
                    if progress: progress(row)
                    break
                supports.append(support)
                for r in source['rows']:
                    if r['max_m'] > event.scope.source_tolerance_m+1e-8:
                        C = np.vstack([C, event.row(r['edge'], r['witness_s'])])
                        low = np.r_[low, r['source_t']-event.scope.source_tolerance_m]
                        high = np.r_[high, r['source_t']+event.scope.source_tolerance_m]
                        families.append('source-envelope-extremum')
                if progress: progress(row)
        # Retain the FINITE outer relaxation solely for numerical exclusion
        # diagnostics. It is not the verified inner interval used to drag.
        D = C@Z; lb, ub = low-C@x0, high-C@x0
        flo, fhi = np.isfinite(lb), np.isfinite(ub)
        self.relaxation_A = np.vstack([-D[flo], D[fhi], *(G@Z for G in supports)])
        self.relaxation_b = np.concatenate([-lb[flo], ub[fhi], *(self.budgets-G@x0 for G in supports)])
        self.relaxation_families = ([f for f, keep in zip(families, flo) if keep]
                                   + [f for f, keep in zip(families, fhi) if keep]
                                   + [f'reversal-edge-{i}' for _ in supports for i in range(event.count+1)])
        minimum, maximum = [0. if np.array_equal(self.witnesses[s], self.current) else
                            float(self.handle@self.witnesses[s]-self.reference_t) for s in (-1, 1)]
        # Include exact zero; it replays the original byte sequence.
        minimum, maximum = min(0., minimum), max(0., maximum)
        source_t = float(np.interp(self.point.station, self.trace['st'][:, 0], self.trace['st'][:, 1]))
        self.report = dict(schema='mapforge/event-control-range/v1', status='VERIFIED_INNER_INTERVAL',
            source_key=self.point.source_key, edge=self.point.edge, station_m=self.point.station,
            reference_sha256=self.reference_sha256, reference_t_m=self.reference_t,
            source_t_m=source_t, source_point_delta_m=source_t-self.reference_t,
            verified_delta_m=[minimum, maximum], history=history,
            reversal_budgets_m=self.budgets.tolist(), derivative_guards=self.derivative_guards,
            guard_scope='registered source/width/C2/reversal/derivative guards ONLY',
            beyond_interval='UNVERIFIED_NOT_PROVEN_IMPOSSIBLE', map_accepted=False,
            static_shape_pass=False, dynamics='NOT_EVALUATED', new_curve_knots=0)
        return self.report

    def diagnose_target(self, displacement):
        """Read-only LP exclusion tests, no relaxed map written or admitted.

Omitting a guard is a DIAGNOSTIC sensitivity calculation only. An LP outer
relaxation containing the target is inconclusive about true reachability.
Float dual residuals are reported; never label this a formal certificate.
"""
        if self.report is None or isinstance(displacement, bool) or not isinstance(displacement, (int, float)) or not math.isfinite(displacement):
            raise ValueError('Verified preparation and finite target required')
        sign = -1 if displacement < 0 else 1
        c = -sign*(self.handle@self.event.Z)
        rows = []
        for omit in (None, 'derivative-nonregression', 'reversal-edge-', 'source-envelope'):
            keep = np.array([omit is None or not f.startswith(omit) for f in self.relaxation_families])
            A, b = self.relaxation_A[keep], self.relaxation_b[keep]
            fit = linprog(c, A_ub=A, b_ub=b, bounds=[(None, None)]*len(c), method='highs',
                          options={'time_limit': 5., 'dual_feasibility_tolerance': 1e-9,
                                   'primal_feasibility_tolerance': 1e-9})
            row = dict(omitted_diagnostic_only=omit, solver_status=int(fit.status), message=str(fit.message))
            if fit.success:
                dual = fit.ineqlin.marginals
                stationarity = float(np.max(abs(A.T@dual-c)))
                primal = float(max(0., np.max(A@fit.x-b)))
                gap = float(abs(c@fit.x-b@dual))
                bound = float(-sign*(b@dual)+self.handle@self.current-self.reference_t)
                excluded = sign*(displacement-bound) > 1e-5 and max(stationarity, primal, gap) < 1e-7
                row.update(numerical_outer_bound_delta_m=bound, stationarity_residual=stationarity,
                           primal_violation=primal, duality_gap=gap,
                           target_status='NUMERICALLY_EXCLUDED' if excluded else 'INCONCLUSIVE')
                row['active_families'] = sorted({f for f, d in zip(np.array(self.relaxation_families)[keep], dual)
                                                if abs(d) > 1e-7})
            rows.append(row)
        return dict(schema='mapforge/control-target-diagnosis/v1', requested_m=displacement,
                    reference_sha256=self.reference_sha256, rows=rows,
                    formal_certificate=False, omitted_guards_accepted=False, candidate_written=False,
                    conclusion_scope='fixed source chart, long basis, endpoint/birth locks and current guard set')

    def preview(self, displacement):
        if isinstance(displacement, bool) or not isinstance(displacement, (int, float)) or not math.isfinite(displacement):
            raise ValueError('Finite absolute displacement from reference required')
        if self.report is None:
            raise ValueError('Prepare and verify the reachable interval first')
        lo, hi = self.report['verified_delta_m']
        if not lo <= displacement <= hi:
            raise ValueError(f'超出已验证区间 [{lo:.6f}, {hi:.6f}]m；不自动吸附，不代表已证明无解')
        if displacement == 0:
            state, data = self.current, self.reference
        else:
            sign = 1 if displacement > 0 else -1
            endpoint = self.witnesses[sign]
            extreme = float(self.handle@endpoint-self.reference_t)
            factor = displacement/extreme
            if not 0. <= factor <= 1.:
                raise ValueError('Unverified extrapolation forbidden')
            state = self.current+factor*(endpoint-self.current)
            ok, _, _, _ = self._guard(state)
            if not ok: raise ValueError('Interpolated common state failed original guards')
            data = self.event.compile(state)
        actual = float(boundary_poly(_road(data, self.event), self.point.edge, self.point.station)[0])-self.reference_t
        # The handle is linear in the same common coefficients, not a soft
        # objective. Check the SERIALIZED XML, even for a zero/no-op replay.
        if abs(actual-displacement) > 1e-8:
            raise ValueError('Written control point differs from requested target')
        shape = written_shape(data, self.event)
        if np.max(np.array(list(shape['by_edge'].values()))-self.budgets) > 1e-7:
            raise ValueError('Written boundary reversal regression')
        readback = check(data, self.event.data, self.event.packet, self.event.scope.__dict__, self.event.inventory)
        if readback['status'] != 'PASS_LOCAL_EVENT_NOT_MAP':
            raise ValueError('Written event check failed')
        return data, dict(status='LOCAL_CONTROL_PREVIEW_NOT_MAP', xodr_sha256=digest(data),
            reference_sha256=self.reference_sha256, requested_m=displacement, achieved_m=actual,
            error_m=abs(actual-displacement), source_max_m=readback['source_event_max_m'],
            outside_max_change_m=readback['outside_event_max_change_m'],
            reversal_by_edge_m=shape['by_edge'], derivative_guards='BOUNDED_RELATIVE_TO_REFERENCE',
            new_curve_knots=0, map_accepted=False, static_shape_pass=False,
            dynamics='NOT_EVALUATED', readback=readback)
