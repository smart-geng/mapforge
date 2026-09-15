"""Joint finite-dimensional reference + transverse solve, research only.

No inner width optimum is frozen while an outer source gate is evaluated.
Reference and cubic coefficients share ONE nonlinear constrained problem.
"""
import numpy as np
import json
from scipy.interpolate import BSpline
from scipy.optimize import minimize,least_squares

from spikes.measured_connector_caps import (chain,needs_cap,ribbon,sample,distances,source_slacks,
    write_ribbon,polish_endpoint)
from spikes.connector_cross_section import edge_jet
from mapforge.ops.source_connector_ribbon import minimum_forward_factor


def resolve_contact_caps(a,b,requested=None):
    observed=(needs_cap(a),needs_cap(b))
    if requested is None:return observed
    caps=tuple(requested)
    if len(caps)!=2 or any(not isinstance(c,bool) for c in caps):
        raise ValueError('two explicit contact-cap booleans required')
    if any(required and not selected for required,selected in zip(observed,caps)):
        raise ValueError('cannot remove a currently required contact cap')
    return caps


def basis_for(cls):
    lens=np.array([c.length for c in cls]);refs=np.r_[0.,np.cumsum(lens)];L=refs[-1]
    mid=len(cls)//2
    kn=np.sort(np.r_[refs,lens[0]/2,L-lens[-1]/2,(refs[mid]+refs[mid+1])/2])
    breaks=kn/L;cut=np.array([breaks[np.argmin(abs(kn-s))] for s in refs[1:-1]])
    interior=[]
    for t in breaks[1:-1]:interior.extend([t]*(2 if np.min(abs(cut-t))<1e-12 else 1))
    knots=np.r_[[0.]*4,interior,[1.]*4]
    return kn,BSpline(knots,np.eye(len(knots)-4),3,extrapolate=False),cut


def coefficients_to_basis(kn,co,B):
    sites=np.array([np.mean(B.t[i+1:i+4]) for i in range(len(B.c))]);ss=sites*kn[-1]
    idx=np.clip(np.searchsorted(kn,ss,side='right')-1,0,len(kn)-2);u=ss-kn[idx]
    rows=[]
    for side in ('left','right'):
        a,b,c,d=co[side][idx].T;rows.extend(np.linalg.solve(B(sites),a+u*(b+u*(c+u*d))))
    return np.array(rows)


def fit_joint(road,a,b,raw,via,parameters,*,initial_coefficients=None,evaluation_step=.1,source_margin=.02,
              fair_world=False,max_iterations=100,progress=None,core_count=3,source_tails=(),_problem_only=False,
              contact_caps=None):
    if not np.isfinite(evaluation_step) or not .02<=evaluation_step<=.1:
        raise ValueError('joint construction evaluation step must be 0.02..0.1m')
    if not np.isfinite(source_margin) or not .01<=source_margin<.35:
        raise ValueError('construction margin must tighten the unchanged source gate')
    if not isinstance(fair_world,bool) or not isinstance(max_iterations,int) or max_iterations<1:
        raise ValueError('explicit fairing mode and positive iteration budget required')
    supplied=initial_coefficients is not None
    from mapforge.ops.world_curve_fairness import source_turn_evidence,ribbon_fairness,source_turn_slacks
    from mapforge.ops.long_connector_chain import chain as build_chain,polish_endpoint as close_chain
    evidence=source_turn_evidence(raw) if fair_world else None
    caps=resolve_contact_caps(a,b,contact_caps);nr=core_count+sum(caps);nq=nr+core_count-1;q=np.array(parameters,float)
    if core_count not in (2,3,4) or not 3<=nr<=5:raise ValueError('three to at most five long reference primitives required')
    if len(q)!=nq:raise ValueError('same-parent shape parameters required')
    chain=lambda values,aa,bb,cc:build_chain(values,aa,bb,cc,core_count)
    polish=None
    if initial_coefficients is None:
        trial,polish=close_chain(q,a,b,caps,core_count)
        if min(trial[:nr])>=6.-1e-8 and max(abs(np.array(polish['after'])))<1e-7:q=trial
    cls=chain(q,a,b,caps);kn,B,_=basis_for(cls);nc=len(B.c)
    if initial_coefficients is None:
        _,co=ribbon(cls,a,b,raw,True);initial_coefficients=coefficients_to_basis(kn,co,B)
    initial_coefficients=np.asarray(initial_coefficients,float)
    if initial_coefficients.shape!=(2*nc,) or not np.isfinite(initial_coefficients).all():
        raise ValueError('complete finite coefficients of the same structural basis required')
    v0=np.r_[q,initial_coefficients];cache={}
    bounds=np.array([(6.,100.)]*nr+[(-6.,6.)]*(core_count-1)+[(-100.,100.)]*(2*nc))
    if fair_world:
        # Explicit local research domain, not an infeasibility proof outside it.
        radius=np.r_[np.full(nr,6.),np.full(core_count-1,1.5),np.full(2*nc,4.)]
        bounds=np.c_[np.maximum(bounds[:,0],v0-radius),np.minimum(bounds[:,1],v0+radius)]
    from mapforge.ops.fixed_ribbon_sampling import sample_fixed
    tails=list(source_tails)
    for tail in tails:
        if tail['role'] not in ('predecessor','successor') or set(tail['curves'])!={'left','right','center'}:
            raise ValueError('original topology-bound complete source tail required')
    def evaluate(v, dynamic_frames=None, source_data=None):
        aa,bb=(a,b) if dynamic_frames is None else dynamic_frames
        active_raw = raw if source_data is None else source_data['raw']
        active_via = via if source_data is None else source_data['via']
        active_tails = tails if source_data is None else source_data['tails']
        key=(tuple(v),json.dumps((aa,bb),sort_keys=True))
        if source_data is None and key in cache:return cache[key]
        cls=chain(v[:nq],aa,bb,caps);kn,B,jumps=basis_for(cls);L=kn[-1]
        coeff=[v[nq+i*nc:nq+(i+1)*nc] for i in (0,1)]
        co={side:np.array([[B(s/L,d)@c/L**d/(1,1,2,6)[d] for d in range(4)] for s in kn[:-1]])
            for side,c in zip(('left','right'),coeff)}
        # Evaluation density is independent of the number of fitted records.
        # The old 0.5m grid admitted a 0.749m raw-via P95 that became 0.754m
        # on actual 0.05m readback. Tighten construction, never the final gate.
        if fair_world:pts=sample_fixed(cls,kn,co,bounds[:nr,1],evaluation_step)
        else:pts,_,_=sample(cls,kn,co,step=evaluation_step)
        errors={k:{'source_to_target':distances(active_raw[k],pts[k]),'target_to_source':distances(pts[k],active_raw[k])} for k in active_raw}
        if source_data is not None and source_data.get('separate_movement_ids'):
            # Approved movement observations must not re-enter physical
            # center fitting through the connector's concatenated raw path.
            # Keep both error directions on each KNOWN physical fragment;
            # do not bridge excluded fragments to manufacture a centerline.
            from spikes.measured_connector_caps import crop_at_projection
            parts=source_data.get('physical_center_parts',[])
            if not parts:raise ValueError('physical via center source must remain present')
            forward=[];reverse=[]
            for part in parts:
                forward.extend(distances(part,pts['center']))
                target=crop_at_projection(pts['center'],part[0],True)
                target=crop_at_projection(target,part[-1],False)
                reverse.extend(distances(target,part))
            errors['center']={'source_to_target':np.array(forward),'target_to_source':np.array(reverse)}
        errors.update({'raw_via:'+k:{'source_to_target':distances(vv,pts[k])} for k,vv in active_via.items()})
        tail_errors={}
        from spikes.measured_connector_caps import crop_at_projection
        for tail in active_tails:
            for side in ('left','right'):
                source=tail['curves'][side];after=tail['role']=='successor'
                tip=source[0 if after else -1]
                actual=crop_at_projection(pts[side],tip,after)
                tail_errors[tail['role']+':'+side]=distances(actual,source)
        eq=[]
        for frame,c,at,k,dk in ((aa,cls[0],0.,cls[0].KappaStart,cls[0].dk),(bb,cls[-1],1.,cls[-1].KappaEnd,cls[-1].dk)):
            for side,values in zip(('left','right'),coeff):
                target=edge_jet(frame,frame['edges'][side],k,dk)
                eq.extend((np.array([B(at,d)@values/L**d for d in range(3)])-target)*[1,20,100])
        k=np.array([c.KappaEnd for c in cls[:-1]]);dj=np.diff([c.dk for c in cls]);slopes=[]
        for values in coeff:
            t=B(jumps)@values;dt=B(jumps,1)@values/L;den=1-k*t
            dd=(B(jumps,2)-B(np.nextafter(jumps,-np.inf),2))@values/L**2
            eq.extend(20*(dd+dj*t*dt/den));slopes.append(dt/den)
        eq.extend(slopes[0]-slopes[1])
        end=cls[-1];dh=end.ThetaEnd-bb['pose'][2]
        eq.extend([end.XEnd-bb['pose'][0],end.YEnd-bb['pose'][1],20*np.arctan2(np.sin(dh),np.cos(dh))])
        width=[]
        for c,span in zip(co['left']-co['right'],np.diff(kn)):
            roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(c))
            ss=[0.,span]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<span]
            width.extend(np.polynomial.polynomial.polyval(ss,c))
        forward=minimum_forward_factor(cls,kn,co)
        inequality=np.r_[min(width)-.1,forward-.1,source_slacks(errors,source_margin)]
        # These pieces belong to the original ordinary-road boundary family,
        # whose 0.35m gate is stricter than a via's 1.5m maximum. Do not replace
        # that ownership gate by the looser connector P95 test.
        inequality=np.r_[inequality,[.349-max(e) for e in tail_errors.values()]]
        objective=sum(np.mean(e**2) for field in errors.values() for e in field.values())+.0002*sum(v[:nr])
        source_objective=objective
        fairness=None
        if fair_world:
            active_evidence = evidence if source_data is None else source_turn_evidence(active_raw,
                endpoint_headings=source_data.get('original_endpoint_headings'))
            if source_data is not None and source_data.get('separate_movement_ids'):
                # Via center remains physical. Ordinary-center fairing is
                # evaluated in the parent state; excluded navigation paths
                # must not prescribe a physical-center turn budget.
                active_evidence['center']=source_turn_evidence(active_via)['center']
            fairness=ribbon_fairness(cls,kn,co,active_evidence)
            # The acceptance diagnostic allows at most one extra degree. Use
            # half a degree in construction, keeping the final gate unchanged.
            inequality=np.r_[inequality,source_turn_slacks(fairness)]
            objective=.05*objective+1000*sum(r['curvature_variation_energy'] for r in fairness.values())
        # Retain a complete finite-difference stencil across objective and both
        # constraint calls instead of computing the same expensive curves thrice.
        result=(cls,kn,co,errors,np.array(eq),inequality,objective,min(width),forward,fairness,tail_errors,source_objective)
        if source_data is None:
            if len(cache)>=256:cache.clear()
            cache[key]=result
        return result
    active=np.arange(len(v0))
    def unpack(y,dynamic_frames=None):return y
    if fair_world:
        from mapforge.ops.endpoint_jet_coordinates import complete_endpoint_coefficients
        active=np.r_[np.arange(nq),np.arange(nq+3,nq+nc-3),np.arange(nq+nc+3,nq+2*nc-3)]
        def unpack(y,dynamic_frames=None):
            aa,bb=(a,b) if dynamic_frames is None else dynamic_frames
            full=v0.copy();full[active]=y
            refs=chain(full[:nq],aa,bb,caps);kk,basis,_=basis_for(refs)
            for i,side in enumerate(('left','right')):
                start=nq+i*nc;end=start+nc
                jets=np.r_[edge_jet(aa,aa['edges'][side],refs[0].KappaStart,refs[0].dk),
                           edge_jet(bb,bb['edges'][side],refs[-1].KappaEnd,refs[-1].dk)]
                full[start:end]=complete_endpoint_coefficients(basis,kk[-1],full[start+3:end-3],jets)
            return full
    def state(y):
        result=evaluate(unpack(y))
        return result[4][12:] if fair_world else result[4],result[5],result[6]
    initial=unpack(v0[active])
    if _problem_only:
        if not fair_world:raise ValueError('shared-parent kernel requires world geometry constraints')
        return dict(initial=initial[active].copy(),bounds=bounds[active].copy(),unpack=unpack,evaluate=evaluate,
            caps=caps,nq=nq,nr=nr,nc=nc,active=active.copy(),core_count=core_count,
            source_metric_budgets={'max-m':1.5-source_margin,'p95-m':.75-source_margin,
                                  'median-m':.35-source_margin,'ordinary-tail:max-m':.349},
            source_evidence=evidence,source_tails=tails,road=road)
    def feasible(v):
        state=evaluate(v)
        return max(abs(state[4]))<=1e-6 and min(state[5])>=-1e-5 and min(v[:nr])>=6.-1e-7
    pool=[];iterations=0
    def remember(v):
        nonlocal iterations
        iterations+=1
        if feasible(v):pool.append(v.copy())
        if progress is not None and iterations%5==0:
            state=evaluate(v)
            progress(dict(iteration=iterations,equality=float(max(abs(state[4]))),slack=float(min(state[5])),
                          objective=float(state[6]),feasible_count=len(pool)))
    remember(initial);restore=None;optimizer=None;chosen=initial
    if not pool:
        # Feasibility is a separate phase, not an objective trade-off. Penalty
        # convergence is not a pass: the original unscaled gates below decide.
        rr=least_squares(lambda y:np.r_[10*state(y)[0],np.minimum(state(y)[1],0.)],initial[active],
            bounds=bounds[active].T,x_scale='jac',diff_step=1e-5,max_nfev=120,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        restored=unpack(rr.x)
        restore=dict(success=bool(rr.success),message=str(rr.message),evaluations=int(rr.nfev),
                     equality_error=float(max(abs(evaluate(restored)[4]))),slack=float(min(evaluate(restored)[5])))
        remember(restored);chosen=restored
        if not pool and not fair_world:
            result=minimize(lambda v:evaluate(v)[6],rr.x,method='SLSQP',bounds=bounds,
                constraints=[dict(type='eq',fun=lambda v:evaluate(v)[4]),dict(type='ineq',fun=lambda v:evaluate(v)[5])],
                callback=remember,options=dict(maxiter=max_iterations,ftol=1e-9,eps=1e-5))
            optimizer=dict(success=bool(result.success),message=str(result.message),iterations=int(result.nit))
            remember(result.x);chosen=result.x
    if fair_world:
        start=min(pool,key=lambda v:evaluate(v)[6]) if pool else chosen
        from mapforge.ops.step_limited_sqp import solve
        radii=np.r_[np.full(nr,2.),np.full(core_count-1,.25),np.full(2*nc,.5)]
        opt,optimizer=solve(state,start[active],bounds[active],radii[active],
            max_iterations=max_iterations,callback=lambda y:remember(unpack(y)),optimizer=minimize)
        chosen=unpack(opt)
        remember(chosen)
    if pool:chosen=min(pool,key=lambda v:evaluate(v)[6])
    cls,kn,co,errors,eq,ineq,_,width,forward,fairness,tail_errors,_=evaluate(chosen)
    passed=feasible(chosen)
    candidate=write_ribbon(road,cls,kn,co)
    report=dict(road=road.get('id'),status='GEOMETRY_REVIEW_CANDIDATE' if passed else 'REJECTED',
        method='joint-reference-and-cubic-coefficients',joint_variables=len(v0),
        optimizer=optimizer,feasibility_restoration=restore,feasible_shapes_retained=len(pool),
        shape_parameters=chosen[:nq].tolist(),joint_coefficients=chosen[nq:].tolist(),reference_core_count=core_count,
        primitive_lengths_m=[c.length for c in cls],width_records=len(kn)-1,
        minimum_width_record_span_m=float(min(np.diff(kn))),minimum_width_exact_m=float(width),
        minimum_forward_factor_exact=float(forward),maximum_scaled_equality_error=float(max(abs(eq))),
        minimum_construction_slack=float(min(ineq)),source={k:{d:dict(median_m=float(np.median(e)),p95_m=float(np.percentile(e,95)),max_m=float(max(e)))
          for d,e in fields.items()} for k,fields in errors.items()},production_accepted=False,
        source_speed_changed=False,dynamics_status='REQUIRES_INDEPENDENT_READBACK',seed_polish=polish)
    report.update(construction_evaluation_step_m=evaluation_step,construction_source_margin_m=source_margin,
        final_source_thresholds_unchanged=True,initial_coefficients_supplied=supplied,
        world_fairing_enabled=fair_world,world_fairness=fairness,
        fairing_energy_weight=1000 if fair_world else None,
        added_reverse_turn_construction_budget_deg=.5 if fair_world else None)
    report.update(source_turn_budget_scope='all-three-original-fields' if fair_world else None,
        optimization_search_bounds=bounds[active].tolist(),fixed_evaluation_grid=bool(fair_world),
        independent_optimization_variables=len(active),endpoint_jets_eliminated=bool(fair_world),
        original_tail_reverse_errors_m={k:float(max(v)) for k,v in tail_errors.items()},
        original_tail_construction_max_m=.349,original_tail_final_max_m=.35,
        source_tail_ids=[t['source_lane_id'] for t in tails])
    return candidate,report
