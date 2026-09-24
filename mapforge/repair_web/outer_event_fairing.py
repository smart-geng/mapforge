"""Hard-position minimum-deformation edit with actual boundary curvature.

The old derivative proxy is not used. Nonlinear point constraints receive
critical-point witnesses from the represented long cubics. Reversal and total
curvature variation are evaluated on complete intervals, with active analytic
gradients (nonsmooth at switches; local solver failure is NOT impossibility).
"""
import math
import time

import numpy as np
from numpy.polynomial import Polynomial as P
from scipy.linalg import null_space
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint

from .outer_event_shape import reversal_support
from .model import parse
from mapforge.validate.line_boundary_shape import written_boundary_shape
from scripts.check_outer_event_written import boundary_poly, check


def world_geometry(jets):
    """(k, dk/d world arclength), and exact partials wrt t',t'',t'''."""
    a, b, c = np.moveaxis(np.asarray(jets, float), -1, 0)
    q = 1+a*a
    value = np.stack((b/q**1.5, c/q**2-3*a*b*b/q**3), axis=-1)
    jac = np.stack((np.stack((-3*a*b/q**2.5, 1/q**1.5, np.zeros_like(a)), axis=-1),
                    np.stack((-4*a*c/q**3-3*b*b/q**3+18*a*a*b*b/q**4,
                              -6*a*b/q**3, 1/q**2), axis=-1)), axis=-2)
    return value, jac


def critical_parameters(coefficients, length):
    t = P(np.asarray(coefficients)*length**np.arange(4))
    a, b, c = (t.deriv(j)/length**j for j in (1, 2, 3))
    q = 1+a*a; n = c*q-3*a*b*b
    def roots(p):
        return [0., *sorted(float(r.real) for r in p.roots() if abs(r.imag)<1e-8 and 0<r.real<1), 1.]
    return roots(n), roots(n.deriv()*q-3*n*q.deriv())


class CurvatureFairing:
    def __init__(self, control):
        self.control, self.event = control, control.event
        self.reference = control.current.copy()
        written = written_boundary_shape(control.reference, self.event.scope.__dict__)
        self.caps = np.array([[written['by_edge'][str(e)][k] for k in
                              ('max_abs_curvature', 'max_abs_curvature_rate', 'curvature_total_variation')]
                             for e in range(self.event.count+1)])
        self.spans, self.witnesses = [], []
        self.witness_keys = set()
        objective_rows = []
        # Four Gauss points integrate degree-six squared displacement exactly.
        nodes, weights = np.polynomial.legendre.leggauss(4)
        for edge, bs in self.event.splines.items():
            cuts = sorted(set(bs.t))
            for lo, hi in zip(cuts, cuts[1:]):
                p = self.event.power(edge, lo)
                self.spans.append(dict(edge=edge, lo=lo, hi=hi, p=p))
                for u, weight in zip((nodes+1)/2, weights*(hi-lo)/2):
                    d = u*(hi-lo)
                    objective_rows.extend([np.array([1.,d,d*d,d**3])@p*np.sqrt(weight),
                                           np.array([0.,1.,2*d,3*d*d])@p*np.sqrt(weight)*10.])
        self.M = np.array(objective_rows)
        self.objective_reference = self.M@self.reference
        for index, span in enumerate(self.spans):
            critical = critical_parameters(span['p']@self.reference, span['hi']-span['lo'])
            for u in {0., .5, 1., *critical[0], *critical[1]}: self.add_witness(index, u)

    def jet_rows(self, index, u):
        span = self.spans[index]; d = u*(span['hi']-span['lo'])
        return np.array([[0.,1.,2*d,3*d*d], [0.,0.,2.,6*d], [0.,0.,0.,6.]])@span['p']

    def add_witness(self, index, u):
        key = (index, round(float(u), 10))
        if key in self.witness_keys: return
        self.witness_keys.add(key)
        edge = self.spans[index]['edge']
        self.witnesses.append(dict(span=index, u=float(u), edge=edge, rows=self.jet_rows(index, u)))

    def sampled(self, state):
        rows = np.array([w['rows'] for w in self.witnesses])
        values, jac = world_geometry(rows@state)
        caps = np.maximum(np.array([self.caps[w['edge'], :2] for w in self.witnesses]), 1e-12)
        return values/caps, np.einsum('wij,wjn->win', jac/caps[:, :, None], rows)

    def total_variation(self, state):
        values = np.zeros(self.event.count+1)
        gradients = np.zeros((self.event.count+1, self.event.nvar))
        for index, span in enumerate(self.spans):
            # At an interior k extremum, the motion of the root contributes
            # k'(root)*root' = 0. At a switch this is an active gradient only.
            us = critical_parameters(span['p']@state, span['hi']-span['lo'])[0]
            rows = np.array([self.jet_rows(index, u) for u in us])
            k, jac = world_geometry(rows@state)
            g = np.einsum('wj,wjn->wn', jac[:, 0, :], rows)
            changes = np.diff(k[:, 0]); edge = span['edge']
            values[edge] += np.sum(abs(changes))
            gradients[edge] += np.sum(np.sign(changes)[:, None]*np.diff(g, axis=0), axis=0)
        return values, gradients

    def nonlinear(self, state):
        v, g = self.sampled(state)
        reversal, rg = reversal_support(self.event, state)
        tv, tg = self.total_variation(state)
        # Small reference reversal is not converted into a relaxed floor gate;
        # only numerical row scaling is floored. Bounds retain original values.
        rs = np.maximum(self.control.budgets, .01)
        ts = np.maximum(self.caps[:, 2], 1e-8)
        values = np.r_[v.ravel(), reversal/rs, tv/ts]
        jac = np.vstack([g.reshape(-1, self.event.nvar), rg/rs[:, None], tg/ts[:, None]])
        lower = np.r_[np.full(v.size, -1.), np.full(len(rs)+len(ts), -np.inf)]
        upper = np.r_[np.ones(v.size), self.control.budgets/rs, np.ones(len(ts))]
        return values, jac, lower, upper

    def continuous(self, state):
        maxima = np.zeros((self.event.count+1, 2)); new_points = []
        for index, span in enumerate(self.spans):
            edge = span['edge']
            for metric, us in enumerate(critical_parameters(span['p']@state, span['hi']-span['lo'])):
                rows = np.array([self.jet_rows(index, u) for u in us])
                values = abs(world_geometry(rows@state)[0][:, metric])
                maxima[edge, metric] = max(maxima[edge, metric], float(max(values)))
                for u, value in zip(us, values):
                    if value > self.caps[edge, metric]+1e-9: new_points.append((index, u))
        return maxima, new_points

    def solve(self, displacement, *, max_iterations=160, max_seconds=90., progress=None, start_state=None):
        if (isinstance(displacement, bool) or not isinstance(displacement, (int, float))
                or not math.isfinite(displacement) or abs(displacement)>2
                or type(max_iterations) is not int or not 1<=max_iterations<=160
                or not isinstance(max_seconds, (int, float)) or not 0<max_seconds<=90):
            raise ValueError('Explicit finite target and bounded 160-iteration/90s trial required')
        event, control = self.event, self.control
        started = time.monotonic(); history = []
        # Affine elimination of BOTH original shared endpoint/birth equalities
        # and the exact new position. Position is never an objective penalty.
        a = control.handle@event.Z
        if np.linalg.norm(a)<1e-10: raise ValueError('Control position is locked by event equalities')
        target = control.reference_t+displacement
        seed = self.reference if start_state is None else np.asarray(start_state,float)
        # Continuation may only use an already verified state in this SAME
        # common model and original reference budgets. It does not replace
        # the source or reset the nonregression baseline at every small step.
        if start_state is not None:
            event.compile(seed)
            peaks,_=self.continuous(seed);tv,_=self.total_variation(seed)
            rev,_=reversal_support(event,seed)
            if (np.max(peaks-self.caps[:,:2])>1e-9 or max(tv-self.caps[:,2])>1e-8
                    or max(rev-control.budgets)>1e-7):
                raise ValueError('Continuation requires a verified prior state, not a failed trial')
        x0 = seed+event.Z@a*((target-control.handle@seed)/(a@a))
        Z = event.Z@null_space(a[None, :])
        B = self.M@Z; offset = self.M@x0-self.objective_reference
        H, c = B.T@B, B.T@offset
        scale = max(1., float(np.linalg.norm(H, 2)))
        H, c = H/scale, c/scale
        _, _, C, low, high, _, _, _ = event.linear_model()
        z = np.zeros(Z.shape[1]); used = 0; result_state = None
        terminal = 'EXCHANGE_BUDGET_EXHAUSTED'; last_trial = None; written_delta = None

        class BudgetEnded(Exception): pass

        for exchange in range(4):
            D = C@Z; lb, ub = low-C@x0, high-C@x0
            norms = np.linalg.norm(D, axis=1); live = norms>1e-9
            if np.any(lb[~live]>1e-7) or np.any(ub[~live]<-1e-7):
                terminal = 'FIXED_AFFINE_TARGET_CONFLICT'; break
            D, lb, ub = D[live]/norms[live, None], lb[live]/norms[live], ub[live]/norms[live]
            _, _, nl, nu = self.nonlinear(x0+Z@z)
            cache = {}
            def evaluated(value):
                if time.monotonic()-started>max_seconds: raise BudgetEnded()
                key = value.tobytes()
                if cache.get('key') != key:
                    values, jac, _, _ = self.nonlinear(x0+Z@value)
                    cache.update(key=key, values=values, jac=jac@Z)
                return cache
            def callback(value):
                nonlocal used
                used += 1
                if progress and used%10 == 0:
                    report = evaluated(value)['values']
                    progress(dict(iteration=used, exchange=exchange,
                                  normalized_violation=float(max(0., max(nl-report), max(report-nu)))))
                if time.monotonic()-started>max_seconds: raise BudgetEnded()
            try:
                iteration_start=used
                fit = minimize(lambda v:.5*v@H@v+c@v, z, jac=lambda v:H@v+c, method='SLSQP',
                               constraints=[LinearConstraint(D, lb, ub),
                                            NonlinearConstraint(lambda v:evaluated(v)['values'], nl, nu,
                                                                jac=lambda v:evaluated(v)['jac'])],
                               callback=callback, options={'maxiter':max_iterations-used, 'ftol':1e-10})
            except BudgetEnded:
                terminal = 'TIME_BUDGET_EXHAUSTED'; break
            # SLSQP nit includes terminal iterations that do not invoke the
            # callback. Count them too when enforcing the shared work budget.
            used=iteration_start+int(fit.nit)
            z = fit.x; x = x0+Z@z; last_trial = x.copy()
            source = event.source_error(x); reversal, _ = reversal_support(event, x)
            maxima, witnesses = self.continuous(x); variation, _ = self.total_variation(x)
            violation = float(max(0., max(low-C@x), max(C@x-high)))
            row = dict(exchange=exchange, optimizer_success=bool(fit.success), message=str(fit.message),
                       iterations=int(fit.nit), source_max_m=source['max_m'], linear_violation=violation,
                       reversal_m=reversal.tolist(), peak_curvature_and_rate=maxima.tolist(),
                       total_curvature_variation=variation.tolist(), new_critical_witnesses=len(witnesses),
                       displacement_energy=float(np.sum((self.M@x-self.objective_reference)**2)))
            history.append(row)
            if progress: progress(row)
            good = (source['max_m']<=event.scope.source_tolerance_m+1e-7 and violation<=1e-7
                    and max(reversal-control.budgets)<=1e-7 and np.max(maxima-self.caps[:, :2])<=1e-9
                    and max(variation-self.caps[:, 2])<=1e-8
                    and abs(control.handle@x-target)<=1e-8)
            if good:
                data = event.compile(x)
                independent = check(data, event.data, event.packet, event.scope.__dict__, event.inventory)
                shape = written_boundary_shape(data, event.scope.__dict__)
                actual_caps = np.array([[shape['by_edge'][str(e)][k] for k in
                                        ('max_abs_curvature','max_abs_curvature_rate','curvature_total_variation')]
                                       for e in range(event.count+1)])
                if independent['status']=='PASS_LOCAL_EVENT_NOT_MAP' and np.all(actual_caps<=self.caps+np.array([1e-9,1e-9,1e-8])):
                    road=next(r for r in parse(data).findall('road') if r.get('id')==event.scope.road)
                    written_delta=float(boundary_poly(road,control.point.edge,control.point.station)[0])-control.reference_t
                    if abs(written_delta-displacement)>1e-8:
                        terminal='WRITTEN_TARGET_MISMATCH'; break
                    result_state=x; terminal='LOCAL_FAIRING_CANDIDATE_NOT_MAP'; break
                terminal='INDEPENDENT_READBACK_REJECTED'; break
            if used>=max_iterations:
                terminal='ITERATION_BUDGET_EXHAUSTED'; break
            added=0
            for r in source['rows']:
                if r['max_m']>event.scope.source_tolerance_m+1e-8:
                    C=np.vstack([C,event.row(r['edge'],r['witness_s'])])
                    low=np.r_[low,r['source_t']-event.scope.source_tolerance_m]
                    high=np.r_[high,r['source_t']+event.scope.source_tolerance_m]
                    added+=1
            old_witnesses=len(self.witnesses)
            for index, u in witnesses: self.add_witness(index, u)
            row['source_extrema_added']=added
            row['curvature_witnesses_added']=len(self.witnesses)-old_witnesses
            # A failed finite-witness local solve is not the full interval
            # problem. Continue ONLY if its actual violation supplies a new
            # interval constraint, from the SAME state and SAME total budget.
            # No new start, random seed or second requested target.
            if not fit.success and added+len(self.witnesses)-old_witnesses==0:
                terminal='LOCAL_SOLVER_REJECTED_NOT_IMPOSSIBILITY'; break
        return result_state, dict(schema='mapforge/world-curvature-fairing/v1', status=terminal,
            requested_m=displacement, reference_sha256=control.reference_sha256, history=history,
            iterations=used, elapsed_seconds=time.monotonic()-started, nvar=event.nvar,
            free_with_hard_target=Z.shape[1], new_curve_knots=0, parameter_derivative_proxy_caps=False,
            reference_caps=self.caps.tolist(), static_map_accepted=False, operating_dynamics='NOT_EVALUATED',
            map_accepted=False, formal_certificate=False,
            rejected_state=None if result_state is not None or last_trial is None else last_trial.tolist(),
            actual_written_m=None if result_state is None else written_delta)

    def continuation(self, displacement, *, progress=None):
        """Four deterministic pointer steps, one final target, <=160 iterations.

No rejected intermediate becomes the next seed. No published intermediate
map; partial reach is not reported as success of the final user intention.
"""
        if isinstance(displacement,bool) or not isinstance(displacement,(int,float)) or not math.isfinite(displacement) or abs(displacement)>2:
            raise ValueError('Finite target within two metres required')
        started=time.monotonic();state=self.reference.copy();rows=[];used=0;success=True
        for step in range(1,5):
            remaining=90-(time.monotonic()-started)
            if remaining<=0: success=False;break
            target=displacement*step/4
            result,report=self.solve(target,max_iterations=40,max_seconds=min(remaining,90),
                                     progress=progress,start_state=state)
            used+=report['iterations'];rows.append(report)
            if progress:progress(dict(continuation_step=step,requested_m=target,status=report['status']))
            if result is None:success=False;break
            state=result
        return (state if success else None),dict(schema='mapforge/world-curvature-continuation/v1',
            status='LOCAL_FAIRING_CANDIDATE_NOT_MAP' if success else 'CONTINUATION_REJECTED',
            requested_m=displacement,reference_sha256=self.control.reference_sha256,
            iterations=used,elapsed_seconds=time.monotonic()-started,steps=rows,
            exact_final_target_met=success,intermediates_published=False,new_curve_knots=0,
            map_accepted=False,formal_certificate=False)
