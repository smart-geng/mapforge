"""Bounded joint feasibility steps with HARD original parent inequalities.

Uses the same geometric residual and Jacobian as the shared problem. A QP
limits each Gauss-Newton step to the original linear source polytope and the
declared search box. Backtracking checks the actual nonlinear residual; this
is not a global convergence or map acceptance certificate.
"""
import numpy as np
from scipy import sparse


def source_constraints(problem):
    n=len(problem.initial);blocks=[];lower=[]
    for rid,parent in sorted(problem.parents.items()):
        block=np.zeros((len(parent.slack0),n));block[:,problem.parent_slices[rid]]=parent.CZ
        blocks.append(block);lower.extend(-parent.slack0)
    if not blocks:raise ValueError('explicit source-backed parents required')
    return np.vstack(blocks),np.array(lower)


def qp_step(J,residual,C,lower,bounds,radius,damping=1e-5):
    """C*d >= lower; bounds and radius are in the current shared coordinates."""
    import clarabel
    J=sparse.csc_matrix(J);n=J.shape[1]
    scale=np.maximum(np.asarray(J.power(2).sum(axis=0)).ravel(),1e-6)
    H=J.T@J+sparse.diags(damping*scale)
    g=np.asarray(J.T@residual).ravel()
    lo=np.maximum(bounds[:,0],-radius);hi=np.minimum(bounds[:,1],radius)
    if np.any(lo>hi):raise ValueError('empty declared step box')
    # Null-space elimination leaves roundoff-sized remnants of exact equalities.
    # Normalizing a 1e-15 row to unit norm would amplify that noise into a false
    # strict constraint. Omit only rows whose ENTIRE step-box expression has
    # magnitude <=1e-12. This is much stricter than the existing 1e-7 original
    # source-state check; no source point, geometric budget or final row is lost.
    row_envelope=np.abs(C)@np.maximum(abs(lo),abs(hi))+abs(lower)
    numerical_rows=row_envelope<=1e-12
    # Some exact endpoint rows have a 1e-17 coefficient but a +0.9
    # regularity margin. Unit-normalizing them creates an artificial 1e17
    # RHS. Omit only if the ORIGINAL linear inequality is satisfied over
    # the entire explicit box, with a conservative floating error margin.
    box_lower=np.sum(np.where(C>=0,C*lo,C*hi),axis=1)
    certified_margin=box_lower-lower
    redundant_rows=certified_margin>1e-9+64*np.finfo(float).eps*row_envelope
    retained=~(numerical_rows|redundant_rows)
    qp_C=C[retained];qp_lower=lower[retained]
    A=sparse.vstack([-sparse.csc_matrix(qp_C),sparse.eye(n),-sparse.eye(n)]).tocsc()
    b=np.r_[-qp_lower,hi,-lo]
    # Positive variable/row/objective scaling is an equivalent QP, not a
    # rescaling of source tolerances. Always verify the ORIGINAL step below.
    variable_scale=1./np.sqrt(np.maximum(H.diagonal(),1e-10))
    D=sparse.diags(variable_scale)
    H=D@H@D;g=variable_scale*g;A=(A@D).tocsc()
    row_norm=np.sqrt(np.asarray(A.power(2).sum(axis=1)).ravel())
    keep=row_norm!=0.
    if np.any(b[~keep]<0.):
        return None,dict(status='exact-zero-row-conflict',iterations=0)
    A=(sparse.diags(1./row_norm[keep])@A[keep]).tocsc();b=b[keep]/row_norm[keep]
    objective_scale=max(1.,float(np.max(np.asarray(abs(H).sum(axis=1)))))
    H=H/objective_scale;g=g/objective_scale
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=150;settings.time_limit=30.
    settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-10;settings.tol_feas=1e-10
    result=clarabel.DefaultSolver(sparse.triu(H).tocsc(),g,A,b,
        [clarabel.NonnegativeConeT(len(b))],settings).solve()
    info=dict(status=str(result.status),iterations=int(result.iterations),primal_residual=float(result.r_prim),
        equivalent_positive_scaling=True,variable_scale_min=float(min(variable_scale)),
        variable_scale_max=float(max(variable_scale)),nonzero_constraint_norm_min=float(min(row_norm[keep])),
        nonzero_constraint_norm_max=float(max(row_norm[keep])),
        numerically_null_step_rows=int(sum(numerical_rows)),
        box_redundant_step_rows=int(sum(redundant_rows)),
        box_redundant_minimum_original_margin=(float(min(certified_margin[redundant_rows])) if np.any(redundant_rows) else None),
        numerical_rows_whole_box_violation_bound=float(np.max(row_envelope[numerical_rows],initial=0.)),
        original_constraints_all_rechecked=False)
    if str(result.status) not in ('Solved','AlmostSolved'):return None,info
    step=variable_scale*np.asarray(result.x)
    violation=float(max(np.max(lower-C@step,initial=0.),np.max(lo-step,initial=0.),np.max(step-hi,initial=0.)))
    info['original_step_violation']=violation
    info['original_constraints_all_rechecked']=True
    return (step if np.isfinite(step).all() and violation<=1e-7 else None),info


def solve(problem,initial,*,max_iterations=6,progress=None):
    x=np.array(initial,float,copy=True);C,lower=source_constraints(problem)
    if x.shape!=problem.initial.shape or not np.isfinite(x).all():raise ValueError('finite full shared state required')
    def source_violation(v):return float(max(0.,np.max(lower-C@v,initial=0.)))
    if source_violation(x)>1e-7 or np.any(x<problem.bounds[:,0]-1e-8) or np.any(x>problem.bounds[:,1]+1e-8):
        raise ValueError('initial state violates original source or search box')
    regularity=getattr(problem,'regular_geometry',None)
    if regularity is not None and not regularity(x)['regular']:
        raise ValueError('initial shared ribbons require a regular positive-width chart')
    radius=np.full(len(x),.5)
    for sl in problem.parent_slices.values():radius[sl]=.15
    for rid,sl in problem.turn_slices.items():
        nr=problem.kernels[rid]['nr'];core=problem.kernels[rid]['core_count']
        radius[sl.start:sl.start+nr]=2.
        radius[sl.start+nr:sl.start+nr+core-1]=.25
    history=[]
    for i in range(max_iterations):
        r=problem.residual(x);cost=float(r@r)
        if problem.summary(x)['status']=='REQUIRES_FILE_READBACK':break
        if progress:progress(dict(iteration=i+1,stage='jacobian',squared_residual=cost))
        J=problem.jacobian(x);accepted=False;record={}
        for damp in (1e-5,1e-3,.1):
            step,qp=qp_step(J,r,C,lower-C@x,problem.bounds-x[:,None],radius,damp)
            record=dict(iteration=i+1,before_squared_residual=cost,qp=qp,damping=damp)
            if step is None:continue
            for alpha in (1.,.5,.25,.125,.0625):
                trial=x+alpha*step
                if source_violation(trial)>1e-7:continue
                try:
                    after=problem.residual(trial);newcost=float(after@after)
                    if regularity is not None and not regularity(trial)['regular']:continue
                except (ValueError,FloatingPointError):continue
                if np.isfinite(newcost) and newcost<cost*(1.-1e-7):
                    x=trial;record.update(accepted=True,alpha=alpha,after_squared_residual=newcost,
                        source_inequality_violation=source_violation(x));accepted=True;break
            if accepted:break
        record.setdefault('accepted',False);history.append(record)
        if progress:progress(record)
        if not accepted:break
    return x,dict(method='hard-source-bounded-Gauss-Newton-QP-with-backtracking',iterations=len(history),
        history=history,source_inequality_violation=source_violation(x),
        global_convergence_claimed=False,source_priority_stage_ran=False,production_accepted=False)
