"""Joint event geometry and actual driving-center dynamics in ONE state.

The fixed source chart and long cubic representation are not silently changed.
An inexpensive necessary-condition LP precedes nonlinear optimization: a
failed local optimization is not a proof that all map representations fail.
"""
from __future__ import annotations

import math
import numpy as np
from scipy.optimize import linprog, minimize, LinearConstraint, NonlinearConstraint

from .model import intervals,lanes
from scripts.internal_edge_jets import active
from mapforge.ops.continuous_offset_dynamics import audit_span


def normalized_dynamics(jets,speed_kmh):
    """Signed (v²*k/2.5, v³*dk/dl/1) and exact derivatives wrt t',t'',t'''."""
    jets=np.asarray(jets,float)
    a,b,c=np.moveaxis(jets,-1,0)
    q=1+a*a
    k=b/q**1.5
    rate=c/q**2-3*a*b*b/q**3
    jac=np.stack((np.stack((-3*a*b/q**2.5,1/q**1.5,np.zeros_like(a)),axis=-1),
                  np.stack((-4*a*c/q**3-3*b*b/q**3+18*a*a*b*b/q**4,
                            -6*a*b/q**3,1/q**2),axis=-1)),axis=-2)
    v=np.asarray(speed_kmh)/3.6
    scale=np.stack((v*v/2.5,v**3),axis=-1)
    return np.stack((k,rate),axis=-1)*scale,jac*scale[...,None]


class DynamicEvent:
    def __init__(self,event,packet):
        self.event=event
        self.spans=[]
        # Intersect all actual written lane sections, source-speed cuts and
        # shape knots, including newborn lanes. None are dynamics-exempt.
        for sec,lo,hi in intervals(event.road):
            a,b=max(lo,event.start),min(hi,event.end)
            if b<=a:continue
            for lid,lane in lanes(sec).items():
                sid=next(e.get('value') for e in lane.findall('userData') if e.get('code')=='mapforge.source_lane')
                speed=packet['observations'][sid]['source_max_speed_kmh']
                if isinstance(speed,bool) or not isinstance(speed,(int,float)) or not np.isfinite(speed) or speed<=0:
                    raise ValueError('Explicit positive original speed required')
                cuts=sorted({a,b}|{s for s in event.scope.knots if a<s<b}|
                            {lo+float(e.get('sOffset')) for e in lane.findall('speed') if a<lo+float(e.get('sOffset'))<b})
                for start,end in zip(cuts,cuts[1:]):
                    written=active(lane.findall('speed'),start-lo,False,lambda e:float(e.get('sOffset')))
                    if written.get('unit','m/s')!='m/s' or abs(float(written.get('max'))*3.6-speed)>1e-5:
                        raise ValueError('Written speed not equal to original source speed')
                    mid=(start+end)/2
                    # Translate one interior Taylor polynomial to span start:
                    # includes the left limit of the third derivative at end.
                    co=np.array([.5*(event.row(-lid-1,mid,j)+event.row(-lid,mid,j))/math.factorial(j) for j in range(4)])
                    u=start-mid
                    shift=np.array([[1,u,u*u,u**3],[0,1,2*u,3*u*u],[0,0,1,3*u],[0,0,0,1]])
                    self.spans.append({'lane':lid,'source_lane':sid,'start':start,'end':end,
                                       'speed_kmh':speed,'power_rows':shift@co})
        if not self.spans:raise ValueError('Empty actual driving-center dynamic scope')
        self.witnesses=[]
        for i,span in enumerate(self.spans):
            for u in (0.,.5,1.):self.add_witness(i,u)

    def add_witness(self,span_index,u):
        if not 0<=u<=1:raise ValueError('Dynamic witness outside span')
        if any(w['span']==span_index and abs(w['u']-u)<1e-9 for w in self.witnesses):return
        span=self.spans[span_index];ds=(span['end']-span['start'])*u
        m=np.array([[0,1,2*ds,3*ds*ds],[0,0,2,6*ds],[0,0,0,6]])@span['power_rows']
        self.witnesses.append({'span':span_index,'u':u,'s':span['start']+ds,
                               'lane':span['lane'],'speed_kmh':span['speed_kmh'],'jet_rows':m})

    def sampled(self,x):
        rows=np.asarray([w['jet_rows'] for w in self.witnesses])
        speeds=np.asarray([w['speed_kmh'] for w in self.witnesses])
        value,jac=normalized_dynamics(rows@x,speeds)
        return value,np.einsum('wij,wjn->win',jac,rows)

    def interval_audit(self,x):
        rows=[]
        for i,s in enumerate(self.spans):
            check=audit_span(s['power_rows']@x,s['end']-s['start'],0.,0.,s['speed_kmh'])
            rows.append({'span':i,'lane':s['lane'],'source_lane':s['source_lane'],
                         'start':s['start'],'end':s['end'],'check':check})
        counts={k:sum(r['check']['status']==k for r in rows) for k in ('FAIL','UNKNOWN','BOUNDED')}
        return {'status':'FAIL' if counts['FAIL'] else 'UNKNOWN' if counts['UNKNOWN'] else 'BOUNDED',
                'counts':counts,'rows':rows,'map_accepted':False,'source_speed_changed':False}

    def necessary_conditions(self,displacement=0.,progress=None):
        """A relaxation, not a sufficient dynamics test or a map impossibility claim.

        Source interval requirements imply these finite sample inequalities.
        Nonnegative width at samples is NECESSARY (do not use the stricter
        Bernstein-control certificate to claim representation infeasibility).
        LP bounds on each t' give M. Then |k|<=K necessarily implies
        |t''|<=K*(1+M²)^1.5. The rate bound additionally requires
        |t'''|<=J*(1+M²)^2 + 3*M*bound(t'')². Combining these conditions may
        reject the fixed chart before any blind nonlinear seed search.
        """
        event=self.event;x0,Z=event.origin,event.Z
        _,_,C,lower,upper,source_n,_,_=event.linear_model(displacement)
        C=list(C[:source_n]);lower=list(lower[:source_n]);upper=list(upper[:source_n])
        for edge in range(1,event.count+1):
            start=event.births.get(edge,event.start)
            cuts=[s for s in event.scope.knots if s>=start]
            for a,b in zip(cuts,cuts[1:]):
                for s in np.linspace(a,b,5):
                    C.append(event.row(edge-1,s)-event.row(edge,s));lower.append(0.);upper.append(np.inf)
        C,lower,upper=map(np.asarray,(C,lower,upper))
        D=C@Z;lb=lower-C@x0;ub=upper-C@x0
        live=np.linalg.norm(D,axis=1)>1e-9
        if np.any(lb[~live]>1e-7) or np.any(ub[~live]<-1e-7):raise ValueError('Inconsistent fixed affine source constraints')
        D,lb,ub=D[live],lb[live],ub[live]
        finite=np.isfinite(ub)
        U=np.vstack((-D,D[finite]));v=np.r_[-lb,ub[finite]]
        bounds=[(None,None)]*Z.shape[1]
        def lp(c,UU=U,vv=v):return linprog(c,A_ub=UU,b_ub=vv,bounds=bounds,method='highs')
        initial=lp(np.zeros(Z.shape[1]))
        if not initial.success:
            return {'status':'BASE_RELAXATION_NOT_FEASIBLE','message':initial.message,'map_accepted':False}
        dynrows=[];dynrhs=[];checks=[]
        for i,w in enumerate(self.witnesses):
            p=w['jet_rows'][0];a=w['jet_rows'][1]
            direction=p@Z
            mini,maxi=lp(direction),lp(-direction)
            if not mini.success or not maxi.success:
                return {'status':'UNBOUNDED_OR_UNKNOWN_SLOPE','witness':i,'map_accepted':False}
            lower_p=float(p@x0+mini.fun);upper_p=float(p@x0-maxi.fun)
            # Conservative numerical padding; this is still a numerical
            # relaxation, not a formal interval certificate of LP solutions.
            M=max(abs(lower_p),abs(upper_p))+1e-6
            bound=2.5/(w['speed_kmh']/3.6)**2*(1+M*M)**1.5
            jerk_bound=1./(w['speed_kmh']/3.6)**3*(1+M*M)**2+3*M*bound**2
            for row,limit in ((a,bound),(w['jet_rows'][2],jerk_bound)):
                for sign in (1.,-1.):
                    dynrows.append(sign*row@Z/limit);dynrhs.append(1.-sign*row@x0/limit)
            checks.append({'index':i,'lane':w['lane'],'s':w['s'],
                           'slope_range':[lower_p,upper_p],'necessary_abs_second_max':bound,
                           'necessary_abs_third_max':jerk_bound})
            if progress and i%25==0:progress({'checked_dynamic_witnesses':i+1,'total':len(self.witnesses)})
        DU=np.asarray(dynrows);dv=np.asarray(dynrhs)
        combined=lp(np.zeros(Z.shape[1]),np.vstack((U,DU)),np.r_[v,dv])
        # Separate diagnosis: minimum additional normalized derivative allowance.
        # This slack is NOT substituted into the real acceleration gate.
        elastic=linprog(np.r_[np.zeros(Z.shape[1]),1.],
            A_ub=np.vstack((np.c_[U,np.zeros(len(U))],np.c_[DU,-np.ones(len(DU))])),
            b_ub=np.r_[v,dv],bounds=bounds+[(0,None)],method='highs')
        active_dynamic=[]
        if elastic.success:
            dual=np.asarray(elastic.ineqlin.marginals[len(U):])
            for index in np.argsort(abs(dual))[::-1][:12]:
                if abs(dual[index])<1e-8:continue
                witness=checks[int(index)//4]
                active_dynamic.append({**witness,'derivative_order':2 if int(index)%4<2 else 3,
                                       'sign':1 if int(index)%2==0 else -1,'dual_weight':float(dual[index])})
        return {'status':'NECESSARY_CONDITIONS_FEASIBLE' if combined.success else
                'FIXED_REPRESENTATION_NUMERICALLY_INFEASIBLE' if combined.status==2 else 'UNKNOWN',
                'message':combined.message,'source_relaxation_feasible':True,
                'minimum_extra_normalized_allowance':float(elastic.x[-1]) if elastic.success else None,
                'elastic_diagnostic_active_dynamic_rows':active_dynamic,
                'witnesses':checks,'formal_proof':False,'map_accepted':False,
                'scope':'this fixed Line chart, registered births, long cubic basis, endpoint jets and original source envelope only',
                'not_claimed':'no impossibility claim for alternative road charts, source-domain layouts or driving paths',
                'source_speed_changed':False}

    def solve(self,displacement=0.,progress=None):
        necessary=self.necessary_conditions(displacement,progress)
        if necessary['status']!='NECESSARY_CONDITIONS_FEASIBLE':
            return None,{'status':'REJECTED_BEFORE_NONLINEAR_SEARCH','necessary':necessary,
                         'map_accepted':False,'export_allowed':False,'nonlinear_iterations':0}
        event=self.event;x0,Z=event.origin,event.Z
        seed,geometric=event.solve(displacement)
        if not geometric['accepted']:return None,{'status':'GEOMETRIC_SEED_REJECTED','map_accepted':False}
        A,y,C,lb,ub,*_=event.linear_model(displacement)
        B=A@Z;target=y-A@x0;H=B.T@B+np.eye(Z.shape[1])*1e-10;f=-B.T@target
        D=C@Z;lower=lb-C@x0;upper=ub-C@x0
        norm=np.linalg.norm(D,axis=1);live=norm>1e-9
        D,lower,upper=D[live],lower[live],upper[live]
        z=np.linalg.lstsq(Z,seed-x0,rcond=None)[0]
        passes=[]
        for turn in range(6):
            def dynamics(z):return self.sampled(x0+Z@z)[0].ravel()
            def jacobian(z):return (self.sampled(x0+Z@z)[1]@Z).reshape(-1,Z.shape[1])
            fit=minimize(lambda z:.5*z@H@z+f@z,z,jac=lambda z:H@z+f,method='SLSQP',
                         constraints=[LinearConstraint(D,lower,upper),NonlinearConstraint(dynamics,-1.,1.,jac=jacobian)],
                         options={'maxiter':120,'ftol':1e-9})
            z=fit.x;x=x0+Z@z;source=event.source_error(x);audit=self.interval_audit(x)
            passes.append({'iteration':turn,'optimizer_success':bool(fit.success),'message':str(fit.message),
                           'iterations':int(fit.nit),'source_max_m':source['max_m'],'dynamics':audit['counts']})
            if progress:progress(passes[-1])
            if fit.success and source['max_m']<=event.scope.source_tolerance_m+1e-7 and audit['status']=='BOUNDED':
                return x,{'status':'JOINT_EVENT_CANDIDATE_NOT_MAP','passes':passes,'necessary':necessary,
                          'map_accepted':False,'dynamic_audit':audit,'export_allowed':False}
            if not fit.success:break
            for r in source['rows']:
                if r['max_m']>event.scope.source_tolerance_m+1e-8:
                    row=event.row(r['edge'],r['witness_s']);D=np.vstack((D,row@Z))
                    lower=np.r_[lower,r['source_t']-event.scope.source_tolerance_m-row@x0]
                    upper=np.r_[upper,r['source_t']+event.scope.source_tolerance_m-row@x0]
            for row in audit['rows']:
                for leaf in row['check']['interval_leaves']:
                    if leaf['status']!='BOUNDED':self.add_witness(row['span'],leaf.get('witness_u',sum(leaf['u'])/2))
        return None,{'status':'JOINT_EVENT_REJECTED','passes':passes,'necessary':necessary,
                     'map_accepted':False,'export_allowed':False}
