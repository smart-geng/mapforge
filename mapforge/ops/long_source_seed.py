"""Source-guided bounded initialization; never an acceptance or short fallback.

An unconstrained root iteration can visit negative lengths before the actual
constrained solve begins. This stage keeps every trial in the same positive,
long-primitive family, then supplies the full world-G2 transverse seed.
"""
import numpy as np
from scipy.linalg import qr
from scipy.optimize import least_squares,minimize
from mapforge.ops.long_connector_chain import chain
from mapforge.ops.source_connector_ribbon import reference_samples,project_observations
from spikes.measured_connector_caps import distances,needs_cap,seed_chain,ribbon
from mapforge.ops.joint_connector_fit import basis_for,coefficients_to_basis,resolve_contact_caps


class ReferenceInitializationError(ValueError):
    """Preserve the bounded failure state; it is not a map candidate."""
    def __init__(self,report):
        super().__init__('no admissible source reference/ribbon initialization; refuse shared solve')
        self.report=report


def reference_closure(parameters,a,b,caps,core_count):
    """Endpoint equations only; original-source distances are a separate stage."""
    end=chain(parameters,a,b,caps,core_count)[-1];dh=end.ThetaEnd-b['pose'][2]
    return np.array([end.XEnd-b['pose'][0],end.YEnd-b['pose'][1],
        20*np.arctan2(np.sin(dh),np.cos(dh))])


def restore_reference_closure(parameters,a,b,caps,core_count,lower,upper):
    """Local bounded three-coordinate equality restoration, no translation.

    An underdetermined full TRF solve near active length bounds can stall at a
    small nonzero endpoint residual. Select a full-rank free subspace and keep
    all other reference parameters fixed. This only supplies a possible seed;
    original-source/chart checks below still decide whether it is retained.
    """
    q=np.array(parameters,float,copy=True);lo=np.asarray(lower);hi=np.asarray(upper)
    def closure(v):
        return reference_closure(v,a,b,caps,core_count)
    before=closure(q);report=dict(attempted=False,before_maximum_scaled_residual=float(max(abs(before))))
    if max(abs(before))>0.1 or max(abs(before))<=1e-9:return q,report
    free=np.flatnonzero(np.minimum(q-lo,hi-q)>1e-4)
    if len(free)<3:return q,dict(report,reason='fewer than three non-bound coordinates')
    J=[]
    for index in free:
        h=min(1e-5*max(1.,abs(q[index])),(hi[index]-q[index])/2)
        v=q.copy();v[index]+=h;J.append((closure(v)-before)/h)
    J=np.asarray(J).T
    norms=np.maximum(np.linalg.norm(J,axis=0),1e-12)
    _,R,pivots=qr(J/norms,mode='economic',pivoting=True)
    if min(abs(np.diag(R[:3,:3])))<1e-8:return q,dict(report,reason='local closure rank deficient')
    chosen=free[pivots[:3]]
    def reduced(values):
        v=q.copy();v[chosen]=values;return closure(v)
    result=least_squares(reduced,q[chosen],bounds=(lo[chosen],hi[chosen]),method='trf',
        x_scale='jac',max_nfev=40,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    candidate=q.copy();candidate[chosen]=result.x;after=closure(candidate)
    bounded=bool(np.all(candidate>=lo) and np.all(candidate<=hi) and np.isfinite(candidate).all())
    improved=bounded and max(abs(after))<max(abs(before))
    report.update(attempted=True,selected_coordinates=chosen.tolist(),bounded=bounded,
        improved=bool(improved),after_maximum_scaled_residual=float(max(abs(after))),
        parameter_change_max=float(max(abs(candidate-q))),optimizer_success=bool(result.success))
    return (candidate if improved else q),report


def initialize(a,b,raw,*,parameters=None,core_count=3,max_iterations=60,contact_caps=None,
               allow_open_reference=False):
    if set(raw)!={'left','right','center'} or any(np.asarray(v).ndim!=2 or
        np.asarray(v).shape[1]!=2 or len(v)<2 or not np.isfinite(v).all() for v in raw.values()):
        raise ValueError('complete finite original curves required')
    caps=resolve_contact_caps(a,b,contact_caps);nr=core_count+sum(caps)
    if core_count not in (2,3,4) or not 3<=nr<=5:raise ValueError('three to at most five long reference primitives')
    if not isinstance(max_iterations,int) or not 1<=max_iterations<=100:raise ValueError('bounded seed budget required')
    if parameters is None:
        if core_count==4:raise ValueError('explicit fourth-core seed required')
        cls=seed_chain(a,b,6. if caps[0] else 0.,6. if caps[1] else 0.)
        i=int(caps[0])
        if core_count==3:
            parameters=np.r_[[c.length for c in cls],20*cls[i].KappaEnd,20*cls[i+1].KappaEnd]
        else:
            # Two-core study: retain the analytic seed's total heading and
            # total core length, NOT its shape. Then solve the same exact
            # endpoints/source conditions in this smaller fixed family.
            left=max(6.,cls[i].length+cls[i+1].length/2)
            right=max(6.,cls[i+2].length+cls[i+1].length/2)
            angle=sum((c.KappaStart+c.KappaEnd)*c.length/2 for c in cls[i:i+3])
            middle=(2*angle-cls[i].KappaStart*left-cls[i+2].KappaEnd*right)/(left+right)
            parameters=np.r_[([6.] if caps[0] else [])+[left,right]+([6.] if caps[1] else []),20*middle]
    q=np.asarray(parameters,float)
    if q.shape!=(nr+core_count-1,) or not np.isfinite(q).all():raise ValueError('invalid complete long-reference seed')
    lo=np.r_[np.full(nr,6.),np.full(core_count-1,-6.)]
    hi=np.r_[np.full(nr,100.),np.full(core_count-1,6.)]
    q=np.clip(q,lo,hi);cache={}
    def evaluate(v):
        key=tuple(v)
        if key in cache:return cache[key]
        cls=chain(v,a,b,caps,core_count);end=cls[-1]
        dh=end.ThetaEnd-b['pose'][2]
        eq=np.array([end.XEnd-b['pose'][0],end.YEnd-b['pose'][1],20*np.arctan2(np.sin(dh),np.cos(dh))])
        starts=np.r_[0.,np.cumsum([c.length for c in cls])]
        # Fixed evaluation cardinality. No source vertex creates a fitted span.
        ss=np.unique(np.concatenate([np.linspace(x,y,81) for x,y in zip(starts[:-1],starts[1:])]))
        xy,_=reference_samples(cls,ss)
        objective=float(np.mean(distances(raw['center'],xy)**2)+np.mean(distances(xy,raw['center'])**2))
        # A reference near the center can still fold underneath an outer edge.
        # Reject that coordinate chart before a transverse optimizer is seeded.
        forward=[]
        for field in ('left','right'):
            station,t=project_observations(cls,raw[field])
            index=np.clip(np.searchsorted(starts,station,side='right')-1,0,nr-1)
            k=np.array([c.KappaStart for c in cls])[index]+np.array([c.dk for c in cls])[index]*(station-starts[index])
            forward.extend(1-k*t)
        if len(cache)>64:cache.clear()
        cache[key]=(eq,objective,float(min(forward)));return cache[key]
    rr=least_squares(lambda v:reference_closure(v,a,b,caps,core_count),q,bounds=(lo,hi),method='trf',x_scale='jac',
        max_nfev=120,ftol=1e-11,xtol=1e-11,gtol=1e-11)
    retained=[]
    def remember(v):
        if (np.max(abs(evaluate(v)[0]))<=1e-7 and evaluate(v)[2]>=.1 and
                np.all(v>=lo-1e-9) and np.all(v<=hi+1e-9)):
            retained.append(np.array(v,copy=True))
    remember(q);remember(rr.x)
    closed,closure_restore=restore_reference_closure(rr.x,a,b,caps,core_count,lo,hi)
    remember(closed)
    opt=minimize(lambda v:evaluate(v)[1],closed,method='SLSQP',bounds=list(zip(lo,hi)),
        constraints=[dict(type='eq',fun=lambda v:evaluate(v)[0]),
                     dict(type='ineq',fun=lambda v:evaluate(v)[2]-.1)],callback=remember,
        options=dict(maxiter=max_iterations,ftol=1e-9,eps=1e-5))
    remember(opt.x)
    restored,source_restore=restore_reference_closure(opt.x,a,b,caps,core_count,lo,hi)
    remember(restored)
    regular_failures=[]
    if allow_open_reference:
        # A genuine coupled restoration can start with nonzero endpoint
        # residual: parent ports are variables, not immutable targets. Never
        # relabel this state as closed or as a successful standalone fit.
        from mapforge.ops.regular_ribbon_seed import initialize_regular_ribbon
        options=retained or [v for v in (q,rr.x,closed,opt.x,restored) if np.isfinite(v).all()
            and np.all(v>=lo-1e-9) and np.all(v<=hi+1e-9) and evaluate(v)[2]>=.1]
        for candidate in sorted(options,key=lambda v:float(evaluate(v)[0]@evaluate(v)[0])):
            cls=chain(candidate,a,b,caps,core_count)
            try:coefficients,regular=initialize_regular_ribbon(cls,a,b,raw)
            except ValueError as exc:
                regular_failures.append(dict(parameters=np.asarray(candidate).tolist(),reason=str(exc)))
                continue
            is_closed=bool(max(abs(evaluate(candidate)[0]))<=1e-7)
            return candidate,coefficients,dict(status=('REGULAR_RIBBON_JOINT_INITIALIZATION_ONLY' if is_closed else 'OPEN_REFERENCE_JOINT_INITIALIZATION_ONLY'),
                reference_closed=is_closed,requires_simultaneous_feasibility_restoration=True,
                primitive_lengths_m=[c.length for c in cls],
                maximum_scaled_closure_residual=float(max(abs(evaluate(candidate)[0]))),
                closure_residual=evaluate(candidate)[0].tolist(),source_center_objective=evaluate(candidate)[1],
                projected_original_edge_forward_minimum=evaluate(candidate)[2],
                actual_ribbon_forward_minimum=regular['actual_ribbon_forward_minimum'],
                actual_ribbon_width_minimum_m=regular['actual_ribbon_width_minimum_m'],regular_ribbon=regular,
                feasible_references_retained=len(retained),bounded_equality_restoration=[closure_restore,source_restore],
                source_changed=False,source_vertices_removed=0,production_accepted=False)
    if not retained or allow_open_reference:
        def state(v):
            eq,obj,forward=evaluate(v)
            return dict(parameters=np.asarray(v).tolist(),primitive_lengths_m=np.asarray(v[:nr]).tolist(),
                maximum_scaled_closure_residual=float(max(abs(eq))),closure_residual=eq.tolist(),
                source_center_objective=obj,projected_original_edge_forward_minimum=forward)
        raise ReferenceInitializationError(dict(status='REJECTED_INITIALIZATION_NOT_MAP',
            caps=list(caps),core_count=core_count,initial=state(q),bounded_closure=state(rr.x),
            source_guided=state(opt.x),closure_optimizer_success=bool(rr.success),
            closure_optimizer_message=str(rr.message),source_optimizer_success=bool(opt.success),
            source_optimizer_message=str(opt.message),feasible_references_retained=len(retained),
            regular_ribbon_failures=regular_failures,
            bounded_equality_restoration=[closure_restore,source_restore],
            global_impossibility_proven=False,production_accepted=False))
    chosen=min(retained,key=lambda v:evaluate(v)[1])
    cls=chain(chosen,a,b,caps,core_count);kn,co=ribbon(cls,a,b,raw,True);_,B,_=basis_for(cls)
    coefficients=coefficients_to_basis(kn,co,B)
    report=dict(status='INITIALIZATION_ONLY_NOT_ACCEPTANCE',method='bounded-closure-then-original-center-guided',
        bounded_optimizer='TRF then SLSQP',primitive_lengths_m=[c.length for c in cls],
        maximum_scaled_closure_residual=float(max(abs(evaluate(chosen)[0]))),
        source_center_objective=evaluate(chosen)[1],feasible_references_retained=len(retained),
        projected_original_edge_forward_minimum=evaluate(chosen)[2],
        optimizer_success=bool(opt.success),optimizer_message=str(opt.message),
        bounded_equality_restoration=[closure_restore,source_restore],
        source_vertices_removed=0,source_changed=False,production_accepted=False)
    return chosen,coefficients,report
