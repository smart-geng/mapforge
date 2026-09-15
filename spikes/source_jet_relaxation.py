"""Necessary continuous-graph dynamics checks, with NO spline basis/export.

Unknowns are boundary value/first/second derivatives at witness stations.
Taylor remainders bound physical lane midpoints between stations. A feasible
relaxation is NOT a curve and cannot be exported. A numerical conflict is
conditional on the unchanged source tube, contacts and midpoint/speed model.
"""
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, hstack
from mapforge.ops.source_transition_domains import midpoint_domains, transition_source_profile


def graph_derivative_bounds(raw, interval, tolerance, curvature, curvature_rate):
    """Necessary graph derivative bounds from source positions and |k|.

    q=sin(theta)=p/sqrt(1+p*p); dq/dx=k. The mean value theorem supplies
    one p in every source secant. Propagate its q bound to the WHOLE query
    interval, without an assumed heading or small-angle approximation.
    """
    raw=np.asarray(raw,float);a,b=map(float,interval)
    if (raw.ndim!=2 or raw.shape[1]!=2 or len(raw)<2 or
        not np.isfinite(raw).all() or np.any(np.diff(raw[:,0])<=0) or
        not np.isfinite([a,b,tolerance,curvature,curvature_rate]).all() or
        b<=a or tolerance<0 or curvature<0 or curvature_rate<0 or
        a<raw[0,0]-1e-8 or b>raw[-1,0]+1e-8):
        raise ValueError('ordered source support, finite nonnegative limits required')
    # Intermediate anchors are justified by the WHOLE original linear tube,
    # not invented measured points. They only sharpen a necessary bound.
    ss=np.unique(np.r_[raw[:,0],np.arange(raw[0,0],raw[-1,0],5.)])
    yy=np.interp(ss,raw[:,0],raw[:,1]);ii,jj=np.triu_indices(len(ss),1)
    secant=(abs(yy[jj]-yy[ii])+2*tolerance)/(ss[jj]-ss[ii])
    distance=np.maximum.reduce([abs(a-ss[ii]),abs(a-ss[jj]),abs(b-ss[ii]),abs(b-ss[jj])])
    candidates=secant/np.hypot(1.,secant)+curvature*distance
    best_index=int(np.argmin(candidates));best=float(candidates[best_index])
    witness=[float(ss[ii[best_index]]),float(ss[jj[best_index]])]
    if best>=1.-1e-10:
        return dict(status='UNAVAILABLE',reason='source/curvature do not bound graph slope away from vertical')
    p=best/np.sqrt(1-best*best)
    # k=r/(1+p^2)^1.5; k_s=u/(1+p^2)^2-3*p*r^2/(1+p^2)^3.
    r=curvature*(1+p*p)**1.5
    u=(curvature_rate+3*p*curvature**2)*(1+p*p)**2
    return dict(status='BOUNDED',slope=p,second=r,third=u,
                sin_heading_bound=best,secant_support_m=witness)


def add_rows(*terms):
    result={}
    for factor,row in terms:
        for k,v in row.items():result[k]=result.get(k,0.)+factor*v
    return {k:v for k,v in result.items() if v!=0.}


class SourceJetRelaxation:
    """Relax all source-boundary families together, not each lane separately."""
    def __init__(self,model,step=2.):
        if getattr(model,'reference_curvature',0.)!=0:
            raise ValueError('graph derivative relaxation requires a Line chart, not an Arc')
        if not np.isfinite(step) or step<=0 or step>5:
            raise ValueError('witness step must be in (0,5] and is NOT a geometry span')
        self.model=model;self.step=step;self.columns={};self.variables=[]
        stations=np.unique(np.concatenate([r[:,0] for k,r in model.raw.items()]))
        stations=np.unique(np.r_[stations,np.arange(stations[0],stations[-1],step)])
        self.stations=stations;self.nvar=0;self.owner=model.owner
        # Preserve exact source float stations, including near-duplicate
        # witnesses. They are not separate output geometry primitives.
        self.family_stations=[]
        for fi,f in enumerate(model.families):
            lo=min(model.raw[k][0,0] for k in f.features);hi=max(model.raw[k][-1,0] for k in f.features)
            grid=stations[(stations>=lo)&(stations<=hi)];self.family_stations.append(grid)
            for s in grid:
                for d in range(3):
                    self.columns[(fi,float(s),d)]=self.nvar
                    self.variables.append(dict(family=fi,station_m=float(s),derivative=d,scale=20.**d))
                    self.nvar+=1
        self.inequalities=[];self.rhs=[];self.labels=[];self.equalities=[]
        self.dynamic_intervals=[];self.unbounded_intervals=[];self.skipped_short_intervals=0
        self._build()

    def row(self,feature,s,derivative=0):
        return {self.columns[(self.owner[feature],float(s),derivative)]:20.**(-derivative)}

    def center(self,left,right,s,d=0):
        return add_rows((.5,self.row(left,s,d)),(.5,self.row(right,s,d)))

    def bound(self,row,upper,label):
        self.inequalities.append(row);self.rhs.append(float(upper));self.labels.append(label)

    def absolute(self,row,limit,label):
        self.bound(row,limit,dict(label,sign=1))
        self.bound({k:-v for k,v in row.items()},limit,dict(label,sign=-1))

    def _build(self):
        m=self.model
        for feature,fi in m.owner.items():
            raw=m.raw[feature];grid=self.family_stations[fi]
            for s in grid[(grid>=raw[0,0])&(grid<=raw[-1,0])]:
                value=float(np.interp(s,raw[:,0],raw[:,1]));row=self.row(feature,s)
                label=dict(kind='source-boundary',feature=feature,station_m=float(s))
                self.bound(row,value+m.source_tol,dict(label,sign=1))
                self.bound({k:-v for k,v in row.items()},-value+m.source_tol,dict(label,sign=-1))
        # Same C2 physical relations; no lane identity chosen by proximity.
        for e in m.equality_labels:
            fa,fb=e['features'];sa,sb=e['stations'];d=e['derivative']
            # Contact source xy->station and stored raw vertex projections
            # may differ at ~1e-14. Lookup nearest exact endpoint only.
            def endpoint(feature,s):
                grid=self.family_stations[self.owner[feature]];v=grid[np.argmin(abs(grid-s))]
                if abs(v-s)>1e-8:raise ValueError('contact station missing from original support')
                return v
            row=add_rows((20.**d,self.row(fa,endpoint(fa,sa),d)),
                         (-20.**d,self.row(fb,endpoint(fb,sb),d)))
            self.equalities.append(row)
        for domain in midpoint_domains(m):
            sid,left,right,a,b,sign=[domain[k] for k in ('source_lane','left','right','a','b','sign')]
            grid=self.stations[(self.stations>=a)&(self.stations<=b)]
            if len(grid)<2:raise ValueError('missing common source support')
            raw_l,raw_r=m.raw[left],m.raw[right]
            if domain['domain_kind']=='source_transition_band':
                source=transition_source_profile(m,domain,self.stations)
            else:
                source=np.c_[grid,.5*(np.interp(grid,raw_l[:,0],raw_l[:,1])+np.interp(grid,raw_r[:,0],raw_r[:,1]))]
            speed=domain['source_speed_kmh']
            if speed is None or not np.isfinite(speed) or speed<=0:raise ValueError('explicit positive source speed required')
            K=2.5/(speed/3.6)**2;J=1./(speed/3.6)**3
            raw=m.raw['lane:'+sid]
            physical=m.roles['feature_roles']['lane:'+sid]['role']=='physical_lane_center_observation'
            for s in grid:
                label=dict(source_lane=sid,station_m=float(s),source_lanes=domain['source_lanes'],domain_kind=domain['domain_kind'])
                self.bound(add_rows((-sign,self.row(left,s)),(sign,self.row(right,s))),0.,dict(label,kind='width'))
                if physical and raw[0,0]<=s<=raw[-1,0]:
                    row=self.center(left,right,s);value=float(np.interp(s,raw[:,0],raw[:,1]))
                    self.bound(row,value+m.source_tol,dict(label,kind='source-center',sign=1))
                    self.bound({k:-v for k,v in row.items()},-value+m.source_tol,dict(label,kind='source-center',sign=-1))
            for lo,hi in zip(grid[:-1],grid[1:]):
                h=hi-lo
                limits=graph_derivative_bounds(source,[lo,hi],m.source_tol,K,J)
                label=dict(source_lane=sid,interval_m=[float(lo),float(hi)],source_speed_kmh=speed,
                           source_lanes=domain['source_lanes'],domain_kind=domain['domain_kind'])
                if limits['status']!='BOUNDED':self.unbounded_intervals.append(label);continue
                self.dynamic_intervals.append(dict(label,**limits))
                a0,a1,a2=[self.center(left,right,lo,d) for d in range(3)]
                b0,b1,b2=[self.center(left,right,hi,d) for d in range(3)]
                B=limits['third']
                # Integral remainder inequalities, valid for C2 graphs with
                # absolutely continuous second derivative and bounded jerk.
                for direction,rows in [('forward',[
                    (add_rows((1,b0),(-1,a0),(-h,a1),(-h*h/2,a2)),B*h**3/6),
                    (add_rows((1,b1),(-1,a1),(-h,a2)),B*h*h/2)]),('backward',[
                    (add_rows((1,a0),(-1,b0),(h,b1),(-h*h/2,b2)),B*h**3/6),
                    (add_rows((1,a1),(-1,b1),(h,b2)),B*h*h/2)])]:
                    for derivative,(row,remainder) in enumerate(rows):
                        self.absolute(row,remainder,dict(label,kind='taylor',direction=direction,derivative=derivative))
                self.absolute(add_rows((1,b2),(-1,a2)),B*h,dict(label,kind='taylor',derivative=2))
                for s,row1,row2 in [(lo,a1,a2),(hi,b1,b2)]:
                    self.absolute(row1,limits['slope'],dict(label,kind='slope-necessary',station_m=float(s)))
                    self.absolute(row2,limits['second'],dict(label,kind='curvature-necessary',station_m=float(s)))

    def matrix(self,rows):
        rr=[];cc=[];vv=[]
        for i,row in enumerate(rows):
            for j,v in row.items():rr.append(i);cc.append(j);vv.append(v)
        return coo_matrix((vv,(rr,cc)),shape=(len(rows),self.nvar)).tocsr()

    def basis_mapping(self):
        """Optional diagnostic restriction to the existing long source basis."""
        rows=[]
        for variable in self.variables:
            feature=self.model.families[variable['family']].features[0]
            rows.append(variable['scale']*self.model.expression(feature,variable['station_m'],variable['derivative']))
        return np.asarray(rows)

    def solve(self,restrict_basis=False):
        A=self.matrix(self.inequalities);b=np.array(self.rhs);E=self.matrix(self.equalities)
        count=self.nvar
        if restrict_basis:
            T=self.basis_mapping();A=coo_matrix(A@T).tocsr();E=coo_matrix(E@T).tocsr()
            count=self.model.nvar
        scale=np.maximum(np.asarray(abs(A).sum(axis=1)).ravel(),1.)
        N=A.multiply((1/scale)[:,None]).tocsr();rhs=b/scale
        problem=hstack([N,-np.ones((len(b),1))]).tocsr()
        answer=linprog(np.r_[np.zeros(count),1.],A_ub=problem,b_ub=rhs,
            A_eq=hstack([E,np.zeros((E.shape[0],1))]).tocsr() if E.shape[0] else None,
            b_eq=np.zeros(E.shape[0]) if E.shape[0] else None,
            bounds=[(None,None)]*count+[(0.,None)],method='highs',
            options={'time_limit':60.,'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
        report=dict(status='UNAVAILABLE',solver_message=answer.message,
            scope='necessary relaxation of same-chart whole-source tubes, fixed contacts and physical midpoint dynamics',
            spline_basis_used=bool(restrict_basis),curve_reconstructed=False,export_allowed=False,
            source_tolerance_m=self.model.source_tol,source_speed_changed=False,
            acceleration_limit_mps2=2.5,jerk_limit_mps3=1.,variables=count,
            inequalities=len(b),contact_equalities=E.shape[0],witness_step_m=self.step,
            propagation_intervals=len(self.dynamic_intervals),unbounded_intervals=self.unbounded_intervals,
            omitted_near_duplicate_propagation=self.skipped_short_intervals,
            all_source_boundary_features=len(self.model.owner),global_map_impossibility_proven=False)
        if not answer.success:return None,report
        x=answer.x[:-1];tau=float(answer.x[-1]);mu=np.asarray(answer.ineqlin.marginals)
        nu=np.asarray(answer.eqlin.marginals) if E.shape[0] else np.empty(0)
        if (x.shape!=(count,) or mu.shape!=(len(b),) or nu.shape!=(E.shape[0],)
            or not np.isfinite(np.r_[x,tau,mu,nu]).all() or tau < -1e-6):
            report['solver_message']='invalid primal/dual solution despite solver status'
            return None,report
        balance=N.T@mu+(E.T@nu if E.shape[0] else 0)
        dual=float(rhs@mu);primal=max(float(np.max(N@x-rhs-tau,initial=0)),
                                      float(np.max(abs(E@x),initial=0)))
        dual_error=max(float(np.max(abs(balance),initial=0)),float(np.max(mu,initial=0)),
                       abs(-float(mu.sum())-1.) if tau>1e-7 else max(-float(mu.sum())-1.,0.))
        report.update(minimum_normalized_slack=tau,dual_objective=dual,
                      primal_residual=primal,dual_residual=dual_error,duality_gap=abs(tau-dual),
                      dual_support=[dict(self.labels[i],weight=float(-mu[i]/scale[i]))
                          for i in np.argsort(mu) if mu[i]<-1e-8])
        if max(primal,dual_error,abs(tau-dual))>1e-6:return None,report
        report['status']='NUMERICAL_CONFLICT' if tau>1e-6 else 'RELAXATION_FEASIBLE_ONLY'
        # No polynomial interpolation of these relaxed jets is justified.
        return x,report
