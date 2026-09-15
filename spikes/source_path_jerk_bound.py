"""Necessary jerk lower bound for a source path, independent of curve basis.

No branch contacts, boundary midpoint construction, lane widths or reference
primitive choice are imposed. Point/derivative unknowns are NOT output geometry.
Assumption: a monotone world-plane graph stays in the full source polyline tube.
"""
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix
from spikes.source_jet_relaxation import graph_derivative_bounds


class SourcePathJerkBound:
    def __init__(self,source,speed_kmh,tolerance=.35,step=2.,tube='euclidean_superset'):
        raw=np.asarray(source,float)
        if (raw.ndim!=2 or raw.shape[1]!=2 or len(raw)<2 or not np.isfinite(raw).all() or
            np.any(np.diff(raw[:,0])<=0) or not np.isfinite([speed_kmh,tolerance,step]).all() or
            speed_kmh<=0 or not 0<tolerance<=.35 or not 0<step<=5 or
            tube not in ('same_station','euclidean_superset')):
            raise ValueError('ordered finite source path and unchanged valid contract required')
        self.original_source=raw.copy();self.speed_kmh=float(speed_kmh);self.step=step;self.tube=tube
        self.original_tolerance=tolerance
        self.slope_max=float(np.max(abs(np.diff(raw[:,1])/np.diff(raw[:,0]))))
        # If dist((x,y), source graph)<=eps, a segment's Lipschitz bound gives
        # |y-f(x)|<=eps*sqrt(1+M^2). Using the GLOBAL M is a conservative
        # necessary SUPERSET, not permission to increase map fitting error.
        self.tolerance=tolerance*(np.hypot(1.,self.slope_max) if tube=='euclidean_superset' else 1.)
        # A candidate endpoint is allowed to move within the source budget.
        # Only this inner domain is guaranteed covered by EVERY monotone
        # candidate with source-matched endpoints; do not pin endpoint x.
        a,b=raw[0,0]+tolerance,raw[-1,0]-tolerance
        if b-a<=2*step:raise ValueError('path too short for this necessary interior-domain check')
        anchors=np.unique(np.r_[a,raw[(raw[:,0]>a)&(raw[:,0]<b),0],b])
        self.source=np.c_[anchors,np.interp(anchors,raw[:,0],raw[:,1])]
        grid=np.arange(a,b,step)
        # Discard ONLY redundant generated witnesses near original anchors,
        # never source vertices. This avoids ill-scaled tiny Taylor rows.
        keep=np.min(abs(grid[:,None]-anchors[None,:]),axis=1)>=min(.05,step/4)
        self.redundant_witnesses_omitted=int(np.sum(~keep))
        self.stations=np.unique(np.r_[anchors,grid[keep]])
        self.nvar=3*len(self.stations)+1;self.inequalities=[];self.rhs=[];self.labels=[];self.bounds=[]
        self._build()

    def row(self,i,d):return {3*i+d:20.**(-d)}

    def bound(self,row,upper,label):
        self.inequalities.append(row);self.rhs.append(float(upper));self.labels.append(label)

    def absolute(self,row,base,factor,label):
        for sign in (1,-1):
            r={k:sign*v for k,v in row.items()};r[self.nvar-1]=-factor
            self.bound(r,base,dict(label,sign=sign))

    def _build(self):
        from spikes.source_jet_relaxation import add_rows
        v=self.speed_kmh/3.6;K=2.5/v**2;J=1./v**3
        for i,s in enumerate(self.stations):
            y=float(np.interp(s,self.source[:,0],self.source[:,1]))
            self.bound(self.row(i,0),y+self.tolerance,dict(kind='source',station_m=float(s),sign=1))
            self.bound({3*i:-1.},-y+self.tolerance,dict(kind='source',station_m=float(s),sign=-1))
        for i,(lo,hi) in enumerate(zip(self.stations[:-1],self.stations[1:])):
            h=hi-lo;limits=graph_derivative_bounds(self.source,[lo,hi],self.tolerance,K,0.)
            if limits['status']!='BOUNDED':raise ValueError('source does not bound graph heading')
            self.bounds.append(dict(interval_m=[float(lo),float(hi)],**limits))
            M=limits['slope'];B0=3*M*K*K*(1+M*M)**2;B1=J*(1+M*M)**2
            a0,a1,a2=[self.row(i,j) for j in range(3)];b0,b1,b2=[self.row(i+1,j) for j in range(3)]
            label=dict(interval_m=[float(lo),float(hi)])
            for direction,rows in [('forward',[
                (add_rows((1,b0),(-1,a0),(-h,a1),(-h*h/2,a2)),h**3/6),
                (add_rows((1,b1),(-1,a1),(-h,a2)),h*h/2)]),('backward',[
                (add_rows((1,a0),(-1,b0),(h,b1),(-h*h/2,b2)),h**3/6),
                (add_rows((1,a1),(-1,b1),(h,b2)),h*h/2)])]:
                for derivative,(row,factor) in enumerate(rows):
                    self.absolute(row,B0*factor,B1*factor,dict(label,kind='taylor',direction=direction,derivative=derivative))
            self.absolute(add_rows((1,b2),(-1,a2)),B0*h,B1*h,dict(label,kind='taylor',derivative=2))
            for j in (i,i+1):
                self.absolute(self.row(j,1),M,0.,dict(kind='slope',station_m=float(self.stations[j])))
                self.absolute(self.row(j,2),limits['second'],0.,dict(kind='curvature',station_m=float(self.stations[j])))

    def matrix(self):
        ri=[];ci=[];va=[]
        for i,row in enumerate(self.inequalities):
            for j,v in row.items():ri.append(i);ci.append(j);va.append(v)
        return coo_matrix((va,(ri,ci)),shape=(len(self.rhs),self.nvar)).tocsr()

    def verify_certificate(self,packet):
        A=self.matrix();b=np.asarray(self.rhs);objective=np.zeros(self.nvar);objective[-1]=1.
        x=np.asarray(packet['primal'],float);mu=np.asarray(packet['dual_inequality'],float)
        lower=np.asarray(packet['dual_lower_bounds'],float)
        if x.shape!=(self.nvar,) or mu.shape!=(len(b),) or lower.shape!=(self.nvar,) or not np.isfinite(np.r_[x,mu,lower]).all():
            raise ValueError('invalid necessary-bound certificate dimensions/values')
        primal=max(float(np.max(A@x-b,initial=0.)),-float(x[-1]),0.)
        dual=float(b@mu);de=max(float(np.max(abs(A.T@mu+lower-objective),initial=0.)),
            float(np.max(mu,initial=0.)),float(np.max(abs(lower[:-1]),initial=0.)),max(-float(lower[-1]),0.))
        gap=abs(float(x[-1])-dual)
        return dict(minimum_necessary_jerk_ratio=float(x[-1]),dual_lower_bound=dual,
                    primal_residual=primal,dual_residual=de,duality_gap=gap,
                    valid_numerical_certificate=max(primal,de,gap)<=1e-6)

    def solve(self):
        A=self.matrix();b=np.asarray(self.rhs);objective=np.zeros(self.nvar);objective[-1]=1.
        # Very close ORIGINAL stations can give h^3 jerk coefficients below
        # HiGHS' small-matrix cutoff. Positive row scaling preserves the LP;
        # do not merge/delete source points or simply trust solver success.
        minimum=np.array([min(abs(v) for v in row.values() if v!=0) for row in self.inequalities])
        scale=np.maximum(1.,1e-5/minimum)
        N=A.multiply(scale[:,None]).tocsr()
        # Keep source metre rows hard. rho multiplies ONLY the allowed jerk
        # bound in a necessary test, never source tolerance or speed fields.
        r=linprog(objective,A_ub=N,b_ub=b*scale,bounds=[(None,None)]*(self.nvar-1)+[(0,None)],method='highs',
                  options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9,'time_limit':60.})
        out=dict(status='UNAVAILABLE',solver_message=str(r.message),tube_model=self.tube,
            source_tolerance_m=self.original_tolerance,necessary_vertical_superset_m=float(self.tolerance),
            source_polyline_max_abs_slope=self.slope_max,source_speed_kmh=self.speed_kmh,
            acceleration_limit_mps2=2.5,jerk_target_mps3=1.,witness_step_m=self.step,
            variables=self.nvar,inequalities=len(b),source_points=len(self.original_source),
            guaranteed_interior_domain_m=self.source[[0,-1],0].tolist(),source_endpoints_pinned=False,
            redundant_generated_witnesses_omitted=self.redundant_witnesses_omitted,
            minimum_witness_interval_m=float(np.min(np.diff(self.stations))),
            constraint_scaling='positive row factors only; certificate checked in ORIGINAL equations',
            maximum_row_scale=float(np.max(scale)),
            curve_basis_imposed=False,physical_boundary_midpoint_imposed=False,branch_contacts_imposed=False,
            scope='monotone planar graph in source path tube; necessary lower bound, not a generated curve',
            speed_changed=False,source_modified=False,xodr_generated=False,export_allowed=False,
            general_map_impossibility_proven=False)
        if not r.success:return None,out
        x=r.x;mu=np.asarray(r.ineqlin.marginals)*scale;lower=np.asarray(r.lower.marginals)
        if not np.isfinite(np.r_[x,mu,lower]).all():return None,out
        packet=dict(primal=x.tolist(),dual_inequality=mu.tolist(),dual_lower_bounds=lower.tolist())
        values=self.verify_certificate(packet)
        out.update(values,certificate=packet,
            dual_support=[dict(self.labels[i],weight=float(-mu[i])) for i in np.argsort(mu) if mu[i]<-1e-8])
        if not values['valid_numerical_certificate']:return None,out
        out['status']='NECESSARY_JERK_CONFLICT' if values['dual_lower_bound']>1.+1e-6 else 'NOT_EXCLUDED_NOT_A_CURVE'
        return x,out
