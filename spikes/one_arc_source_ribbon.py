"""Single smooth chart for a measured turn; fixed source-fitted cubic edges.

The intersection of the two ACTUAL port normal lines fixes the circle center.
Its radius is a coordinate choice; laneOffset carries the lane's real shape.
No changed parent ports, endpoint snapping, short reference chain or new IDs.
This is an explicit research alternative, never a silent conversion fallback.
"""
import math
import copy
import numpy as np
from scipy.interpolate import BSpline
from pyclothoids import Clothoid
from mapforge.ops.arc_source_chart import ArcChart
from spikes.connector_cross_section import edge_jet
from spikes.clarabel_joint_candidate import interior_qp
from spikes.source_contact_fit import polynomial_bernstein
from spikes.measured_connector_caps import write_ribbon


def turn_chart(a,b,raw):
    h0,h1=a['pose'][2],b['pose'][2]
    delta=math.remainder(h1-h0,2*math.pi)
    if not .15<abs(delta)<2.7:raise ValueError('single circular chart needs nonparallel corner ports')
    normals=[np.array([-math.sin(h),math.cos(h)]) for h in (h0,h1)]
    p0,p1=np.array(a['pose'][:2]),np.array(b['pose'][:2])
    M=np.c_[normals[0],-normals[1]]
    if abs(np.linalg.det(M))<.1:raise ValueError('ill-conditioned port-normal intersection')
    q=np.linalg.solve(M,p1-p0);center=p0+q[0]*normals[0]
    sign=float(np.sign(delta));radius=float(np.median(np.linalg.norm(np.vstack(list(raw.values()))-center,axis=1)))
    if radius<3:raise ValueError('source radius below regular chart budget')
    start=center-sign*radius*normals[0];k=sign/radius;length=abs(delta)*radius
    c=Clothoid.StandardParams(*start,h0,k,0.,length)
    mid=ArcChart((c.X(length/2),c.Y(length/2)),h0+delta/2,k)
    frames=[]
    for old,pose in ((a,(c.XStart,c.YStart,c.ThetaStart)),(b,(c.XEnd,c.YEnd,c.ThetaEnd))):
        new=copy.deepcopy(old);new['pose']=pose;frames.append(new)
    return c,mid,frames,dict(center_xy=center.tolist(),radius_m=radius,turn_rad=delta,
                            reference_is_lane_center=False,parent_ports_changed=False)


def fit(road,a,b,raw,tube=1.45):
    # Construction only; original median/P95/max gates still decide the file.
    if set(raw)!={'left','right','center'} or not 0<tube<1.5:raise ValueError('complete source and construction tube below existing maximum required')
    c,chart,ends,meta=turn_chart(a,b,raw);L=c.length
    spans=min(8,int(L/3.))
    if spans<4:raise ValueError('insufficient long cross-section spans')
    breaks=np.linspace(0.,1.,spans+1);knots=np.r_[[0.]*4,breaks[1:-1],[1.]*4]
    n=len(knots)-4;B=BSpline(knots,np.eye(n),3,extrapolate=False)
    Esmall=np.array([B(s,d) for s in (0.,1.) for d in range(3)])
    E=np.zeros((12,2*n));E[:6,:n]=Esmall;E[6:,n:]=Esmall;rhs=[]
    for side in ('left','right'):
        for frame in ends:rhs.extend(edge_jet(frame,frame['edges'][side],c.KappaStart,0.)*[1,L,L*L])
    A=[];y=[];C=[];lo=[];projected={}
    for field,wl,wr in (('left',1.,0.),('right',0.,1.),('center',.5,.5)):
        st=chart.project(raw[field]);ss=st[:,0]+L/2
        if np.any(np.diff(ss)<-1e-6):raise ValueError('original turn not monotone in single arc chart; do not reorder points')
        # Source points outside the exact port plane are not removed. The
        # independent full world readback charges their Euclidean distance.
        basis=B(np.clip(ss/L,0,1));matrix=np.c_[wl*basis,wr*basis];target=st[:,1]
        A.append(matrix/np.sqrt(len(ss)));y.append(target/np.sqrt(len(ss)))
        C.extend([matrix,-matrix]);lo.extend([target-tube,-target-tube])
        projected[field]=dict(source_points=len(ss),outside_port_planes=int(sum((ss<0)|(ss>L))))
    nodes,weights=np.polynomial.legendre.leggauss(3)
    for u,v in zip(breaks[:-1],breaks[1:]):
        s=(u+v)/2+(v-u)*nodes/2;D=B(s,2)/L**2*np.sqrt(.001*L*(v-u)/2*weights)[:,None]
        zero=np.zeros_like(D);A.extend([np.c_[D,zero],np.c_[zero,D]]);y.extend([np.zeros(3),np.zeros(3)])
        bern=polynomial_bernstein(lambda s,d=0:B(s,d),u,v,3)
        C.append(np.c_[bern,-bern]);lo.append(np.full(4,.1))
        zero=np.zeros_like(bern)
        C.extend([np.c_[-c.KappaStart*bern,zero],np.c_[zero,-c.KappaStart*bern]])
        lo.extend([np.full(4,-.9),np.full(4,-.9)])
    A=np.vstack(A);y=np.concatenate(y);C=np.vstack(C);lo=np.concatenate(lo)
    x,phase=interior_qp(A.T@A,A.T@y,E,np.asarray(rhs),C,lo,np.zeros(2*n))
    report=dict(road=road.get('id'),method='one-arc-coordinate-chart-shared-cubic-edges',chart=meta,phase=phase,
        reference_primitives=1,reference_length_m=L,width_spans=spans,minimum_width_span_m=L/spans,
        full_source_projection=projected,tube_m=tube,parent_ports_changed=False,production_accepted=False)
    if x is None:return None,dict(report,status='REJECTED_NO_XODR')
    co={}
    for i,side in enumerate(('left','right')):
        coeff=x[i*n:(i+1)*n]
        co[side]=np.array([[B(s,j)@coeff/(L**j*math.factorial(j)) for j in range(4)] for s in breaks[:-1]])
    new=write_ribbon(road,[c],breaks*L,co)
    return new,dict(report,status='SOURCE_CHART_CANDIDATE_REQUIRES_READBACK',coefficients=x.tolist(),knots=knots.tolist())
