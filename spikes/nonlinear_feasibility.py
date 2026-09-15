"""Trust-region feasibility restoration; temporary slack is never deliverable.

The caller partitions search residuals and hard bounds; EVERY supplied
constraint must pass before a state is returned. Search residuals can include
sources/endpoints/dynamics. No input data or final limits are changed.
Local convergence/failure is not a global feasibility certificate.
"""
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import csc_matrix, vstack, hstack


def restore_joint(initial,E,er,C,lo,evaluate,jacobian,H,g,*,radii,max_iter=20,tolerance=5e-8,
                  hard_nonlinear=None,hard_jacobian=None):
    """Restore all caller residuals while retaining the supplied hard bounds.

    ``evaluate`` returns g(x)<=0, including BOTH signs of endpoint equalities.
    Slack only exists inside the search. A nonzero residual never returns a
    state for writing. LP finds minimum linearized violation; a second QP
    selects a fair state at that level, not an arbitrary noisy LP vertex.
    """
    from spikes.clarabel_joint_candidate import interior_qp
    from scipy.linalg import null_space
    x=np.asarray(initial,float).copy();radii=np.asarray(radii,float)
    if x.shape!=radii.shape or not np.isfinite(radii).all() or np.any(radii<=0):
        raise ValueError('positive per-variable restoration radii required')
    if max_iter<1 or not np.isfinite(tolerance) or tolerance<=0:
        raise ValueError('positive restoration budget and tolerance required')
    history=[];radius=1.
    if (hard_nonlinear is None)!=(hard_jacobian is None):
        raise ValueError('hard nonlinear values and Jacobian must be supplied together')
    Z=null_space(E) if len(E) else np.eye(len(x))
    def hard(v):
        extra=np.asarray(hard_nonlinear(v),float) if hard_nonlinear else np.zeros(0)
        if not np.isfinite(extra).all():return float('inf')
        return max(float(np.max(lo-C@v,initial=0.)),float(np.max(abs(E@v-er),initial=0.)),
                   float(np.max(extra,initial=0.)))
    def report(reason,**extra):
        residuals=np.asarray(evaluate(x),float)
        worst=[dict(row=int(i),violation=float(residuals[i]))
               for i in np.argsort(-residuals)[:12] if np.isfinite(residuals[i]) and residuals[i]>1e-8]
        return dict(status='REJECTED',reason=reason,iterations=history,
                    worst_positive_rows=worst,global_infeasibility_claimed=False,**extra)
    for it in range(max_iter):
        values=np.asarray(evaluate(x),float);score=float(np.max(values,initial=0.))
        if not np.isfinite(values).all():return None,report('non-finite exact shared residual')
        if score<=tolerance and hard(x)<=1e-7:
            return x,dict(status='RESTORED',iterations=history,exact_violation=score,
                          hard_violation=hard(x),global_optimum_claimed=False)
        J=np.asarray(jacobian(x),float)
        if J.shape!=(len(values),len(x)) or not np.isfinite(J).all():
            return None,report('invalid shared restoration Jacobian')
        rhs=J@x-values;box=radius*radii
        alpha=1.
        if hard_nonlinear:
            hv=np.asarray(hard_nonlinear(x));HJ=np.asarray(hard_jacobian(x))
            HC=np.vstack([C,-HJ]);hl=np.r_[lo,-HJ@x+hv+1e-6]
        else:HC,hl=C,lo
        # Eliminate exact end jets before LP, as in the fair QP. Redundant
        # source/zero-width rows otherwise produced HiGHS Unknown, not a
        # mathematical infeasibility certificate, on the real NODE5 case.
        base=x-np.linalg.lstsq(E,E@x-er,rcond=None)[0] if len(E) else x
        mat=np.vstack([np.c_[-HC@Z,np.zeros(len(HC))],np.c_[J@Z,-np.ones(len(J))],
                       np.c_[Z,np.zeros(len(x))],np.c_[-Z,np.zeros(len(x))]])
        bound=np.r_[HC@base-hl,rhs-J@base,x+box-base,base-x+box]
        scale=np.maximum(np.linalg.norm(mat,axis=1),1e-10)
        lp=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=csc_matrix(mat/scale[:,None]),
            b_ub=bound/scale,bounds=[(None,None)]*Z.shape[1]+[(0.,None)],method='highs',
            options={'primal_feasibility_tolerance':1e-8,'dual_feasibility_tolerance':1e-8,'time_limit':30.})
        item=dict(iteration=it,before_exact_violation=score,radius_multiplier=radius,
                  lp_status=lp.message,initial_hard_violation=hard(x),accepted=False);history.append(item)
        if not lp.success:return None,report('restoration hard-bound subproblem failed')
        tau=max(0.,float(lp.x[-1]));item['linearized_minimum_violation']=tau
        # Tiny numerical band around the optimal violation; no source slack.
        allowance=tau+max(1e-9,tau*1e-3)
        CC=np.vstack([HC,-J,np.eye(len(x)),-np.eye(len(x))])
        lower=np.r_[hl,-rhs-allowance,x-box,-x-box]
        # The restoration objective is secondary to violation reduction.
        # A proximal term prevents nearly-unobservable boundary gauges from
        # creating ill-conditioned, arbitrarily large fairing coefficients.
        proximal=np.diag(.01/radii**2)
        if hard_nonlinear and tau<=1e-10:
            # Once the linearized system can close exactly, take a scaled
            # minimum-change normal step. Continuing to move along the
            # fairing objective here can repeatedly reopen nonlinear ends
            # and trigger tiny feasible line-search steps. This is not an
            # arbitrary LP vertex or a relaxation of the final constraints.
            qH=np.diag(1/radii**2);qg=qH@x;item['secondary_objective']='minimum-change normal step'
        else:
            qH=H+proximal;qg=g+proximal@x;item['secondary_objective']='proximal fairing'
        proposed,qp=interior_qp(qH,qg,E,er,CC,lower,base+Z@lp.x[:-1]);item['fair_qp']=qp
        if proposed is None:return None,report('restoration fair QP rejected; LP vertex not accepted')
        if hard_nonlinear:
            alpha=1.;direction=proposed-x
            while hard(proposed)>1e-7 and alpha>1e-5:
                alpha*=.5;proposed=x+alpha*direction
            item['source_backtrack_alpha']=alpha
        exact=np.asarray(evaluate(proposed),float);after=float(np.max(exact,initial=0.))
        ratio=(score-after)/max(alpha*(score-tau),1e-12)
        item.update(after_exact_violation=after,hard_violation=hard(proposed),reduction_ratio=ratio)
        print('RESTORE',it,'exact',score,'->',after,'linear',tau,flush=True)
        if (np.isfinite(exact).all() and hard(proposed)<=1e-7
            and ((after<=tolerance) or (score-after>1e-10 and ratio>=.1))):
            x=proposed;item['accepted']=True
            if ratio>.75:radius=min(4.,radius*1.5)
        else:radius*=.5
        if radius<1e-4:return None,report('restoration locally stalled',best_exact_violation=score)
    values=np.asarray(evaluate(x),float);score=float(np.max(values,initial=0.))
    if np.isfinite(values).all() and score<=tolerance and hard(x)<=1e-7:
        return x,dict(status='RESTORED',iterations=history,exact_violation=score,hard_violation=hard(x),global_optimum_claimed=False)
    return None,report('restoration iteration budget exhausted',best_exact_violation=score)


def restore(initial, E, er, C, lo, evaluate, jacobian, *, max_iter=50, radius=.5):
    x = np.array(initial, float, copy=True)
    records = []
    def residual(v):
        return max(float(np.max(lo-C@v, initial=0.)), float(np.max(abs(E@v-er), initial=0.)))
    if residual(x) > 1e-6:
        return None, {'status': 'REJECTED', 'reason': 'initial hard constraints violated'}
    for iteration in range(max_iter):
        values = np.asarray(evaluate(x)).ravel()
        score = float(np.max(abs(values), initial=0.))
        if not np.isfinite(score):
            return None, {'status': 'REJECTED', 'reason': 'non-finite dynamics', 'iterations': records}
        if score <= 1.+1e-6:
            return x, {'status': 'RESTORED', 'exact_ratio': score, 'hard_residual': residual(x),
                       'iterations': records, 'global_feasibility_claimed': False}
        J = np.asarray(jacobian(x)).reshape(len(values), len(x))
        rhs = J@x-values
        # C*x >= lo remains hard. Only |linearized dynamics| <= 1 + tau is elastic.
        inequalities = vstack([hstack([-csc_matrix(C), csc_matrix((len(C), 1))]),
                               hstack([csc_matrix(J), -np.ones((len(J), 1))]),
                               hstack([-csc_matrix(J), -np.ones((len(J), 1))])]).tocsc()
        equalities = hstack([csc_matrix(E), csc_matrix((len(E), 1))]).tocsc() if len(E) else None
        lp = linprog(np.r_[np.zeros(len(x)), 1.], A_ub=inequalities,
                     b_ub=np.r_[-lo, 1.+rhs, 1.-rhs], A_eq=equalities,
                     b_eq=er if len(E) else None, bounds=list(zip(x-radius, x+radius))+[(0., None)],
                     method='highs', options={'primal_feasibility_tolerance': 1e-8,
                                             'dual_feasibility_tolerance': 1e-8,
                                             'time_limit': 30.})
        record = {'iteration': iteration, 'before_exact_ratio': score, 'radius_m': radius,
                  'lp_status': lp.message, 'accepted': False}
        records.append(record)
        if not lp.success:
            return None, {'status': 'REJECTED', 'reason': 'restoration linear program failed', 'iterations': records}
        proposed = lp.x[:-1]; hard = residual(proposed)
        after = float(np.max(abs(evaluate(proposed)), initial=0.))
        predicted = max(0., score-(1.+float(lp.x[-1])))
        actual = score-after
        ratio = actual/max(predicted, 1e-12)
        record.update(predicted_ratio=1.+float(lp.x[-1]), after_exact_ratio=after,
                      hard_residual=hard, reduction_ratio=ratio)
        if np.isfinite(after) and hard <= 1e-6 and actual > 1e-8 and ratio >= .1:
            x = proposed; record['accepted'] = True
            if ratio > .75:
                radius = min(2., radius*1.5)
        else:
            radius *= .5
        if radius < 1e-5:
            return None, {'status': 'REJECTED', 'reason': 'restoration locally stalled',
                          'best_exact_ratio': float(np.max(abs(evaluate(x)))), 'iterations': records,
                          'global_infeasibility_claimed': False}
    return None, {'status': 'REJECTED', 'reason': 'restoration iteration limit',
                  'best_exact_ratio': float(np.max(abs(evaluate(x)))), 'iterations': records,
                  'global_infeasibility_claimed': False}
