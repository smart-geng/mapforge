"""Box-limited SQP restarts with an explicit feasibility filter.

An engineering safeguard around SLSQP, not a claim of a globally convergent
trust-region algorithm. Failed inner steps cannot replace a feasible anchor.
"""
import numpy as np
from scipy.optimize import minimize


def solve(evaluate, initial, bounds, radii, *, max_iterations, callback=None, optimizer=minimize):
    x=np.asarray(initial,float).copy();box=np.asarray(bounds,float);radii=np.asarray(radii,float)
    if (box.shape!=(len(x),2) or radii.shape!=x.shape or not np.isfinite(radii).all()
        or min(radii)<=0 or max_iterations<1):raise ValueError('finite positive step bounds required')
    if np.any(x<box[:,0]-1e-8) or np.any(x>box[:,1]+1e-8):raise ValueError('seed outside declared search box')
    def measures(v):
        eq,ineq,obj=evaluate(v)
        feasible=max(abs(eq))<=1e-6 and min(ineq)>=-1e-5
        violation=float(np.linalg.norm(np.r_[10*eq,np.minimum(ineq,0.)]))
        return feasible,violation,float(obj)
    history=[];used=0;factor=1.
    while used<max_iterations and factor>=1/32:
        anchor=x.copy();before=measures(anchor);candidates=[anchor]
        local=np.c_[np.maximum(box[:,0],anchor-radii*factor),np.minimum(box[:,1],anchor+radii*factor)]
        # Optimize normalized steps; metre and curvature/coefficient variables
        # must not share an unscaled finite-difference displacement.
        def unpack(y):return anchor+radii*y
        def record(y):
            v=unpack(y);candidates.append(v.copy())
            if callback is not None:callback(v)
        budget=min(25,max_iterations-used)
        result=optimizer(lambda y:evaluate(unpack(y))[2],np.zeros_like(x),method='SLSQP',
            bounds=(local-anchor[:,None])/radii[:,None],
            constraints=[dict(type='eq',fun=lambda y:evaluate(unpack(y))[0]),
                         dict(type='ineq',fun=lambda y:evaluate(unpack(y))[1])],
            callback=record,options=dict(maxiter=budget,ftol=1e-10,eps=1e-6))
        trial=unpack(result.x)
        if np.any(trial<local[:,0]-1e-7) or np.any(trial>local[:,1]+1e-7):
            raise ValueError('optimizer escaped its explicit local step box')
        candidates.append(trial);used+=max(1,int(result.nit))
        valid=[(v,measures(v)) for v in candidates if np.isfinite(v).all()]
        feasible=[p for p in valid if p[1][0]]
        if feasible:chosen,after=min(feasible,key=lambda p:p[1][2])
        else:chosen,after=min(valid,key=lambda p:p[1][1])
        accepted=(after[0] and (not before[0] or after[2]<before[2]-1e-10)
                  or not before[0] and after[1]<before[1]*(1-1e-5))
        history.append(dict(iterations=int(result.nit),inner_success=bool(result.success),
            message=str(result.message),box_factor=factor,accepted=bool(accepted),
            before_violation=before[1],after_violation=after[1],after_objective=after[2]))
        if accepted:x=chosen.copy()
        else:factor/=2
        if after[0] and result.success and np.max(abs(chosen-anchor)/radii)<1e-5:break
    return x,dict(method='scaled-step-limited-SLSQP-with-feasibility-filter',iterations=used,
        stages=history,success=bool(measures(x)[0]),global_optimality_claimed=False)
