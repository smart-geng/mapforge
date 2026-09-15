"""Sequential convex minimax of actual midpoint dynamics, source held hard.

No XODR writer, no change to speed/roles. Local minimax failure is not a
global infeasibility certificate. Sampling never creates curve parameters.
"""
from time import monotonic

import numpy as np
from scipy.linalg import null_space
from scipy.optimize import linprog
from spikes.clarabel_joint_candidate import interior_qp
from mapforge.ops.source_transition_domains import midpoint_domains, source_transition_domains


def graph_kinematics_and_gradient(jets):
    """k and dk/d(physical arc length), exact for y(x), including gradients."""
    p,r,u = np.moveaxis(np.asarray(jets),-1,0)[1:]
    q=1+p*p
    k=r/q**1.5; j=u/q**2-3*p*r*r/q**3
    g=np.zeros(np.asarray(jets).shape[:-1]+(2,4))
    g[...,0,1]=-3*p*r/q**2.5; g[...,0,2]=q**-1.5
    g[...,1,1]=-4*p*u/q**3-3*r*r/q**3+18*p*p*r*r/q**4
    g[...,1,2]=-6*p*r/q**3; g[...,1,3]=q**-2
    return np.stack([k,j],axis=-1),g


def midpoint_rows(model, step=1.):
    if not np.isfinite(step) or step<=0 or step>1:
        raise ValueError('dynamics sampling step must be in (0,1]')
    rows=[];scale=[];labels=[]
    for domain in midpoint_domains(model):
        sid,left,right,a,b=[domain[k] for k in ('source_lane','left','right','a','b')]
        lf,rf=[model.families[model.owner[k]] for k in (left,right)]
        cuts=np.unique(np.r_[a,b,lf.knots[(lf.knots>a)&(lf.knots<b)],rf.knots[(rf.knots>a)&(rf.knots<b)]])
        speed=domain['source_speed_kmh']
        if speed is None or not np.isfinite(speed) or speed<=0:
            raise ValueError('explicit unchanged source speed required')
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            ss=np.unique(np.r_[np.nextafter(lo,hi),np.arange(lo,hi,step),np.nextafter(hi,lo)])
            ss=np.clip(ss,np.nextafter(lo,hi),np.nextafter(hi,lo))
            for s in ss:
                rows.append([.5*(model.expression(left,s,d)+model.expression(right,s,d)) for d in range(4)])
                scale.append([(speed/3.6)**2,(speed/3.6)**3])
                labels.append({'source_lane':sid,'s':float(s),'source_speed_kmh':speed,
                               'domain_kind':domain['domain_kind'],'source_lanes':domain['source_lanes']})
    return np.array(rows),np.array(scale),labels


def audit_transition_bands(model, x, step=.02):
    from math import factorial
    from spikes.road_boundary_family import world_kinematics
    if not np.isfinite(step) or not 0<step<=.02:raise ValueError('dense transition audit requires step <=2cm')
    x=np.asarray(x,float);coverage=source_transition_domains(model);rows=[];width_min=[]
    for d in coverage['intervals']:
        left,right,a,b=[d[k] for k in ('left','right','a','b')]
        lf,rf=[model.families[model.owner[k]] for k in (left,right)]
        cuts=np.unique(np.r_[a,b,lf.knots[(lf.knots>a)&(lf.knots<b)],rf.knots[(rf.knots>a)&(rf.knots<b)]])
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            co=np.array([d['sign']*(model.expression(left,lo,j)-model.expression(right,lo,j))@x/factorial(j)
                         for j in range(model.degree+1)])
            roots=np.polynomial.polynomial.polyroots(np.arange(1,len(co))*co[1:])
            query=[0.,hi-lo]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<hi-lo]
            width_min.append(float(min(np.polynomial.polynomial.polyval(query,co))))
            ss=np.unique(np.r_[np.nextafter(lo,hi),np.arange(lo,hi,step),np.nextafter(hi,lo)])
            ss=np.clip(ss,np.nextafter(lo,hi),np.nextafter(hi,lo))
            jets=np.array([[.5*(model.expression(left,s,j)+model.expression(right,s,j))@x for j in range(4)] for s in ss])
            v=d['source_speed_kmh']/3.6
            values=abs(world_kinematics(jets,getattr(model,'reference_curvature',0.),0.))*[v*v,v**3]
            for j,name in enumerate(('ay_mps2','jerk_mps3')):
                i=int(np.argmax(values[:,j]));rows.append(dict(source_lane=d['source_lane'],source_lanes=d['source_lanes'],
                    event_index=d['event_index'],domain_kind=d['domain_kind'],metric=name,value=float(values[i,j]),
                    s=float(ss[i]),source_speed_kmh=d['source_speed_kmh'],span_m=[float(lo),float(hi)],
                    numerically_resolved_span=bool(hi-lo>1e-8)))
    # Keep raw near-coincident-knot values visible, never silently remove a
    # failure. Their peak is not a reliable finite road-interval measurement;
    # distinguish it from the same curve on numerically resolved intervals.
    resolved=[r for r in rows if r['numerically_resolved_span']]
    return dict(coverage=coverage,sample_step_m=step,rows=rows,
                exact_width_min_m=min(width_min) if width_min else None,
                resolved_span_dynamics_worst=[max((r for r in resolved if r['metric']==name),key=lambda r:r['value'])
                    for name in ('ay_mps2','jerk_mps3')] if resolved else [],
                near_coincident_span_diagnostics=[r for r in rows if not r['numerically_resolved_span']],
                roundoff_span_threshold_m=1e-8,export_allowed=False)


def evaluate(rows,scale,x,limits,reference_curvature=0.):
    jets=rows@x
    if reference_curvature==0:
        values,gradient=graph_kinematics_and_gradient(jets)
    else:
        from spikes.road_boundary_family import world_kinematics
        if not np.isfinite(reference_curvature) or np.any(1-reference_curvature*jets[:,0]<=0):
            raise ValueError('regular finite reference curvature required')
        values=world_kinematics(jets,reference_curvature,0.)
        gradient=np.empty(jets.shape[:-1]+(2,4))
        for j in range(4):
            perturbed=jets.astype(complex);perturbed[:,j]+=1e-25j
            gradient[...,j]=world_kinematics(perturbed,reference_curvature,0.).imag/1e-25
    factor=scale/np.array(limits)
    return values*factor, np.einsum('nmj,njk->nmk',gradient*factor[...,None],rows).reshape(-1,len(x))


def refine_dynamics(model, initial, *, limits=(2.5,1.), iterations=12, seconds=90.):
    if tuple(limits)!=(2.5,1.) or iterations<1 or seconds<=0:
        raise ValueError('research run retains acceleration 2.5 / jerk 1.0; positive budgets required')
    x=np.array(initial,float,copy=True)
    if x.shape!=(model.nvar,) or not np.isfinite(x).all():raise ValueError('invalid initial coefficients')
    if max(np.max(model.lower-model.C@x),np.max(abs(model.E@x),initial=0.))>1e-6:
        raise ValueError('initial shared state violates hard source/contact/width constraints')
    rows,scale,labels=midpoint_rows(model);Z=null_space(model.E) if len(model.E) else np.eye(model.nvar)
    k=getattr(model,'reference_curvature',0.)
    start=monotonic();history=[];radius=.25
    def ratio(state):return float(np.max(abs(evaluate(rows,scale,state,limits,k)[0])))
    initial_ratio=ratio(x)
    H=model.A.T@model.A+np.eye(model.nvar)*1e-10;g=model.A.T@model.y
    for _ in range(iterations):
        if monotonic()-start>=seconds:break
        values,jac=evaluate(rows,scale,x,limits,k);v=values.ravel();base=float(max(abs(v)))
        if base<=1.-1e-3:break
        # Source/width inequalities and C2 null space are unchanged. The only
        # slack is a dimensionless dynamics ratio; never a source tolerance.
        D=jac@Z;hard=model.C@Z
        A=np.vstack([np.c_[-hard,np.zeros(len(hard))],
                     np.c_[D,-np.ones(len(D))],np.c_[-D,-np.ones(len(D))],
                     np.c_[Z,np.zeros(len(Z))],np.c_[-Z,np.zeros(len(Z))]])
        rhs=np.r_[model.C@x-model.lower,-v,v,np.full(2*len(Z),radius)]
        answer=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=A,b_ub=rhs,
                       bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs',
                       options={'time_limit':min(15.,max(1.,seconds-(monotonic()-start)))})
        row={'iteration':len(history)+1,'current_max_ratio':base,'trust_radius_m':radius,
             'linear_solver_success':bool(answer.success),'linear_solver_message':str(answer.message)}
        if not answer.success:history.append(row);break
        # A minimax LP has unconstrained ties on already-good long straights.
        # Select the least unfair state within a 0.1% epigraph tie band before
        # accepting any step. Never display an arbitrary LP vertex as repair.
        target=max(1.,float(answer.x[-1]))*1.001
        C=np.vstack([model.C,-jac,jac,np.eye(model.nvar),-np.eye(model.nvar)])
        lower=np.r_[model.lower,v-jac@x-target,-v+jac@x-target,x-radius,-x-radius]
        proposal,secondary=interior_qp(H,g,model.E,np.zeros(len(model.E)),C,lower,x+Z@answer.x[:-1])
        row['secondary_fairing']=secondary
        if proposal is None:
            row['accepted']=False;row['reason']='no validated least-unfair tie solution';history.append(row);break
        delta=proposal-x;accepted=False
        for alpha in (1.,.5,.25,.125,.0625):
            trial=x+alpha*delta;new=ratio(trial)
            if new<base-1e-7 and np.max(model.lower-model.C@trial)<=1e-6:
                x=trial;accepted=True;row.update(step=alpha,achieved_max_ratio=new);break
        row.update(accepted=accepted,linearized_target_ratio=float(answer.x[-1]));history.append(row)
        if not accepted:radius*=.25
        if radius<1e-5:break
    values=evaluate(rows,scale,x,limits,k)[0];ix=np.unravel_index(np.argmax(abs(values)),values.shape)
    return x,{'status':'SAMPLED_TARGET_REACHED' if np.max(abs(values))<=1 else 'TARGET_NOT_REACHED',
        'formulation':'shared coefficients, hard source/width/C2, local minimax dynamics ratio',
        'initial_max_ratio':initial_ratio,'final_max_ratio':float(np.max(abs(values))),
        'worst_sample':dict(labels[ix[0]],metric=('acceleration','jerk')[ix[1]]),
        'sample_step_m':1.,'sample_count':len(rows),'limits':list(limits),'history':history,
        'source_tolerance_relaxed':False,'source_speed_changed':False,'global_impossibility_proven':False,
        'full_curve_dynamics_certified':False,'export_allowed':False}


def verify_refinement(model, initial, final, record):
    """Recompute saved state metrics; never trust optimizer status or history."""
    initial=np.asarray(initial,float);final=np.asarray(final,float)
    if any(x.shape!=(model.nvar,) or not np.isfinite(x).all() for x in (initial,final)):
        raise ValueError('invalid saved dynamics coefficients')
    rows,scale,labels=midpoint_rows(model)
    k=getattr(model,'reference_curvature',0.)
    values=evaluate(rows,scale,final,(2.5,1.),k)[0]
    first=float(np.max(abs(evaluate(rows,scale,initial,(2.5,1.),k)[0])));last=float(np.max(abs(values)))
    ix=np.unravel_index(np.argmax(abs(values)),values.shape)
    expected={'initial_max_ratio':first,'final_max_ratio':last,'sample_step_m':1.,
              'sample_count':len(rows),'limits':[2.5,1.],
              'worst_sample':dict(labels[ix[0]],metric=('acceleration','jerk')[ix[1]]),
              'status':'SAMPLED_TARGET_REACHED' if last<=1 else 'TARGET_NOT_REACHED'}
    for key,value in expected.items():
        if key in ('initial_max_ratio','final_max_ratio'):
            if not np.isclose(record.get(key,np.inf),value,rtol=1e-12,atol=1e-12):
                raise ValueError('saved dynamics metrics do not match coefficients')
        elif record.get(key)!=value:raise ValueError('saved dynamics scope/status differs')
    for key in ('source_tolerance_relaxed','source_speed_changed','global_impossibility_proven',
                'full_curve_dynamics_certified','export_allowed'):
        if record.get(key) is not False:raise ValueError('unsupported dynamics acceptance claim')
    if last>first+1e-6:raise ValueError('saved dynamics state regresses sampled minimax objective')
    for x in (initial,final):
        if max(np.max(model.lower-model.C@x),np.max(abs(model.E@x),initial=0.))>1e-6:
            raise ValueError('saved dynamics state breaks hard geometry constraints')
    return expected
