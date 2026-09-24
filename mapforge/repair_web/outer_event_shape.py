"""Source-direction preserving objective on the existing shared C2 event.

No extra curve knots, source changes or speed optimisation. A lexicographic
convex solve first minimises a Bernstein upper bound on wrong-way travel of
each source-owned boundary, then minimises the existing fit/roughness cost.
Endpoint locks can force nonzero reversal; that is reported, never waived.
"""
import math

import clarabel
import numpy as np
from scipy import sparse


def _convex(H, c, A, b):
    """Same installed conic backend as the joint solver; bounded, no seed scan."""
    norms = np.sqrt(np.asarray(A.power(2).sum(axis=1)).ravel())
    keep = norms > 1e-12
    if np.any(b[~keep] < -1e-9):
        return None, dict(status='fixed-constraint-conflict', iterations=0)
    # Null-space endpoint equalities can leave 1e-11 derivative rows. Scaling
    # those to unit norm manufactures a false infeasibility. Retain them with
    # bounded positive scaling, then verify ALL unscaled source/shape rows.
    scales = np.maximum(norms[keep], 1e-6)
    A = (sparse.diags(1/scales) @ A[keep]).tocsc()
    b = b[keep]/scales
    scale = max(1., float(np.max(abs(c))), float(np.max(np.asarray(abs(H).sum(axis=1)))))
    settings = clarabel.DefaultSettings()
    settings.verbose = False; settings.max_iter = 120; settings.time_limit = 20.
    settings.tol_gap_abs = 1e-10; settings.tol_gap_rel = 1e-10; settings.tol_feas = 1e-10
    result = clarabel.DefaultSolver(sparse.triu(H/scale).tocsc(), c/scale, A, b,
                [clarabel.NonnegativeConeT(len(b))], settings).solve()
    info = dict(status=str(result.status), iterations=result.iterations, residual=float(result.r_prim))
    if str(result.status) not in ('Solved', 'AlmostSolved'):
        return None, info
    return np.array(result.x), info


def direction_rows(event):
    """Derivative Bernstein controls over SOURCE/fitted-span intersections.

Subdividing constraints is not subdividing the exported geometry. Positive
weights integrate the negative-part upper bound; source IDs remain separate.
"""
    rows, weights = [], []
    for trace, lo, hi, source in event.source_cells():
        L = hi-lo
        p = event.power(trace['edge'], lo)
        controls = np.array([p[1], p[1]+L*p[2], p[1]+2*L*p[2]+3*L*L*p[3]])
        signs = [np.sign(source[1])] if source[1] != 0 else [-1., 1.]
        for sign in signs:
            rows.extend(sign*controls)
            weights.extend([L/3]*3)
    return np.asarray(rows), np.asarray(weights)


def solve_shape(event, displacement=0., *, progress=None):
    A, y, C, low, high, _, handle, station = event.linear_model(displacement)
    R, weights = direction_rows(event)
    Z, x0 = event.Z, event.origin
    n, m = Z.shape[1], len(R)
    B = A@Z; target = y-A@x0
    H0 = B.T@B
    hessian = sparse.block_diag([sparse.csc_matrix(H0), sparse.csc_matrix((m, m))]).tocsc()
    linear = np.r_[-B.T@target, np.zeros(m)]
    shapeA = sparse.hstack([-sparse.csc_matrix(R@Z), -sparse.eye(m)]).tocsc()
    nonnegative = sparse.hstack([sparse.csc_matrix((m, n)), -sparse.eye(m)]).tocsc()
    history = []; state = None
    for iteration in range(9):
        D = C@Z; lower, upper = low-C@x0, high-C@x0
        finite_lo, finite_hi = np.isfinite(lower), np.isfinite(upper)
        U = np.vstack([-D[finite_lo], D[finite_hi]])
        v = np.r_[-lower[finite_lo], upper[finite_hi]]
        constraints = sparse.vstack([sparse.hstack([sparse.csc_matrix(U), sparse.csc_matrix((len(U), m))]),
                                     shapeA, nonnegative]).tocsc()
        rhs = np.r_[v, R@x0, np.zeros(m)]
        obj1 = np.r_[np.zeros(n), weights]
        initial, stage1 = _convex(sparse.csc_matrix((n+m, n+m)), obj1, constraints, rhs)
        row = dict(round=iteration, reversal_minimization=stage1)
        if initial is None:
            history.append(row); break
        upper_bound = float(weights@initial[n:])
        # Numerical optimisation allowance only, not a shape/source tolerance.
        budget = max(0., upper_bound)+1e-7
        constraints = sparse.vstack([constraints, sparse.csc_matrix(obj1[None, :])]).tocsc()
        rhs = np.r_[rhs, budget]
        fitted, stage2 = _convex(hessian, linear, constraints, rhs)
        row.update(fit_minimization=stage2, reversal_upper_bound_min_m=upper_bound)
        history.append(row)
        if progress: progress(row)
        if fitted is None: break
        state = x0+Z@fitted[:n]
        original_violation = float(max(0., np.max(low-C@state), np.max(C@state-high)))
        shape_violation = float(max(0., np.max(-R@state-fitted[n:]), -min(fitted[n:]), weights@fitted[n:]-budget))
        if max(original_violation, shape_violation) > 1e-7:
            row.update(rejection='unscaled-constraint-violation', violation=max(original_violation, shape_violation))
            state = None; break
        source = event.source_error(state)
        violations = [r for r in source['rows'] if r['max_m'] > event.scope.source_tolerance_m+1e-8]
        row.update(source_max_m=source['max_m'], source_extrema_added=len(violations))
        if not violations: break
        if iteration == 8:
            # The fitting refinement aims for 1e-8. The unchanged constructor
            # and independent written-source gate use 1e-7 numeric tolerance.
            # Keep the tighter aim, but use that ORIGINAL final gate at exit.
            if source['max_m'] > event.scope.source_tolerance_m+1e-7:
                row['rejection'] = 'source-exchange-budget-exhausted'; state = None
            else:
                row['termination'] = 'original-source-gate-met-at-bounded-exit'
            break
        for violation in violations:
            C = np.vstack([C, event.row(violation['edge'], violation['witness_s'])])
            low = np.r_[low, violation['source_t']-event.scope.source_tolerance_m]
            high = np.r_[high, violation['source_t']+event.scope.source_tolerance_m]
    report = dict(schema='mapforge/source-direction-shape/v1', history=history,
                  status='GEOMETRY_REJECTED' if state is None else 'LOCAL_SHAPE_CANDIDATE_NOT_MAP',
                  nvar=event.nvar, free_variables=n, derivative_slacks=m,
                  new_curve_knots=0, map_accepted=False, operating_dynamics='NOT_EVALUATED',
                  static_shape_pass=False,
                  objective='lexicographic Bernstein reversal upper bound then fit; not proof of zero reversal')
    if state is not None:
        # Final constructor guard independently checks exact source extrema/width.
        event.compile(state)
        report.update(source=event.source_error(state), handle_requested_m=displacement,
                      handle_achieved_m=float(handle@state-event.old_power(event.count, station)[0]),
                      equality_residual=float(np.max(abs(event.E@state-event.e))),
                      reversal_upper_bound_m=float(weights@np.maximum(0., -R@state)))
    return state, report


def wrong_way_distance(coefficients, length, direction):
    """Exact cubic extrema/integral in metres; no sampled endpoint-only PASS."""
    c = np.asarray(coefficients, float)
    if c.shape != (4,) or not np.isfinite(c).all() or not math.isfinite(length) or length <= 0 or direction not in (-1., 0., 1.):
        raise ValueError('finite cubic, positive span and explicit source direction required')
    roots = np.polynomial.polynomial.polyroots([c[1], 2*c[2], 3*c[3]])
    cuts = [0.] + sorted(float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<length) + [length]
    values = np.polynomial.polynomial.polyval(cuts, c)
    delta = np.diff(values)
    return float(np.sum(abs(delta)) if direction == 0 else np.sum(np.maximum(0., -direction*delta)))


def reversal_support(event, state):
    """Exact convex wrong-way integral and a supporting affine row per edge.

Roots only subdivide the integral. The represented spline basis never changes.
For f(x)=integral max(0,-sign(source)*t'), a row selected by any x is a global
lower bound at every other x. Cutting planes remove the conservative Bernstein
gap without multiplying curve knots or permitting another edge to get worse.
"""
    rows=np.zeros((event.count+1,event.nvar))
    for trace,lo,hi,source in event.source_cells():
        edge=trace['edge'];c=event.power(edge,lo)@state
        roots=np.polynomial.polynomial.polyroots([c[1],2*c[2],3*c[3]])
        points=[lo]+sorted(lo+float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<hi-lo)+[hi]
        direction=np.sign(source[1])
        for a,b in zip(points,points[1:]):
            delta=event.row(edge,b)-event.row(edge,a)
            d=float(delta@state)
            if direction==0:
                rows[edge]+=np.sign(d)*delta
            elif direction*d<0:
                rows[edge]-=direction*delta
    return rows@state,rows


def solve_shape_no_regression(event, budgets, displacement=0., *, progress=None):
    """Bounded improvement, not an endless hunt for an exact global minimum.

Fit subject to half the prior total reversal AND no worse individual edge.
50% is this candidate's optimisation target, not a map acceptance threshold.
"""
    budgets=np.asarray(budgets,float)
    if budgets.shape!=(event.count+1,) or not np.isfinite(budgets).all() or np.any(budgets<0):
        raise ValueError('one finite nonnegative written-reference budget per edge required')
    original_budgets=budgets.copy()
    total_target=float(sum(budgets)*.5)
    # Tighten optimisation constraints to leave numerical headroom. The
    # ORIGINAL source and written nonregression gates remain unchanged.
    margin=1e-5
    budgets=np.maximum(0.,budgets-margin)
    A,y,C,low,high,source_count,handle,station=event.linear_model(displacement)
    low[:source_count]+=margin;high[:source_count]-=margin
    Z,x0=event.Z,event.origin;n=Z.shape[1];m=len(budgets)
    B=A@Z;target=y-A@x0
    H=sparse.block_diag([sparse.csc_matrix(B.T@B),sparse.csc_matrix((m,m))]).tocsc()
    c=np.r_[-B.T@target,np.zeros(m)]
    obj=np.r_[np.zeros(n),np.ones(m)]
    _,initial_support=reversal_support(event,x0)
    supports=[initial_support]
    qlimits=sparse.hstack([sparse.csc_matrix((m,n)),sparse.eye(m)]).tocsc()
    history=[];state=None
    for iteration in range(24):
        D=C@Z;lb,ub=low-C@x0,high-C@x0
        flo,fhi=np.isfinite(lb),np.isfinite(ub)
        U=np.vstack([-D[flo],D[fhi]]);v=np.r_[-lb[flo],ub[fhi]]
        constraints=[sparse.hstack([sparse.csc_matrix(U),sparse.csc_matrix((len(U),m))]),qlimits,-qlimits]
        rhs=[v,budgets,np.zeros(m)]
        for support in supports:
            # q >= support*x = support*(x0+Z*z), not its negative.
            constraints.append(sparse.hstack([sparse.csc_matrix(support@Z),-sparse.eye(m)]))
            rhs.append(-support@x0)
        constraints.append(sparse.csc_matrix(obj[None,:]));rhs.append(np.array([total_target-margin]))
        qconstraints=sparse.vstack(constraints).tocsc();qrhs=np.concatenate(rhs)
        fitted,stage2=_convex(H,c,qconstraints,qrhs)
        row=dict(round=iteration,fit=stage2,total_reversal_target_m=total_target)
        history.append(row)
        if fitted is None:break
        trial=x0+Z@fitted[:n]
        values,support=reversal_support(event,trial);supports.append(support)
        source=event.source_error(trial)
        budget_error=float(np.max(values-original_budgets))
        target_error=float(max(0.,np.sum(values)-total_target))
        constraint_error=float(max(0.,np.max(low-C@trial),np.max(C@trial-high)))
        row.update(source_max_m=source['max_m'],by_edge_reversal_m=values.tolist(),
                   budget_error_m=budget_error,target_error_m=target_error,
                   original_constraint_violation=constraint_error)
        if progress:progress(row)
        # Final source fidelity and per-edge nonregression use original gates,
        # not the optimiser's claims or its deliberately tighter constraints.
        if (source['max_m']<=event.scope.source_tolerance_m+1e-7 and budget_error<=1e-7
                and target_error<=1e-7 and constraint_error<=1e-7):
            state=trial;break
        violations=[r for r in source['rows'] if r['max_m']>event.scope.source_tolerance_m+1e-8]
        for r in violations:
            C=np.vstack([C,event.row(r['edge'],r['witness_s'])])
            low=np.r_[low,r['source_t']-event.scope.source_tolerance_m+margin]
            high=np.r_[high,r['source_t']+event.scope.source_tolerance_m-margin]
    report=dict(schema='mapforge/source-direction-shape/v2',history=history,
                status='GEOMETRY_REJECTED' if state is None else 'LOCAL_SHAPE_CANDIDATE_NOT_MAP',
                nvar=event.nvar,free_variables=n,derivative_slacks=m,new_curve_knots=0,
                map_accepted=False,operating_dynamics='NOT_EVALUATED',static_shape_pass=False,
                prior_written_reversal_budgets_m=original_budgets.tolist(),
                total_reversal_target_m=total_target,optimisation_headroom_m=margin,
                objective='fit with exact reversal at most half prior total; no per-edge regression')
    if state is not None:
        event.compile(state)
        report.update(source=event.source_error(state),handle_requested_m=displacement,
                      handle_achieved_m=float(handle@state-event.old_power(event.count,station)[0]),
                      equality_residual=float(np.max(abs(event.E@state-event.e))),
                      reversal_by_edge_m=reversal_support(event,state)[0].tolist())
    return state,report
