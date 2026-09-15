"""Convex regular-chart initialization on the SAME long cubic basis.

End-edge jets are exact. Internal world-G2/source distance/turn-angle gates
remain unsatisfied constraints for the genuine shared solve, never removed.
The initial ribbon must be positive-width and have a regular offset chart;
requiring an independently solved G2 optimum beforehand freezes the problem
that the shared solver is supposed to resolve.
"""
from math import comb
import numpy as np
from scipy.linalg import null_space
from scipy import sparse
from mapforge.ops.joint_connector_fit import basis_for
from mapforge.ops.source_connector_ribbon import project_observations,minimum_forward_factor
from mapforge.ops.source_feasible_gauss_newton import qp_step
from spikes.connector_cross_section import edge_jet


def initialize_regular_ribbon(cls,a,b,raw):
    if set(raw)!={'left','right','center'}:raise ValueError('complete three-field source required')
    kn,B,_=basis_for(cls);L=kn[-1];nc=len(B.c)
    if not 3<=len(cls)<=5 or min(c.length for c in cls)<6.-1e-8 or min(np.diff(kn))<3.-1e-8:
        raise ValueError('fixed long reference and width basis required')
    E=np.array([B(at,d)/L**d for at in (0.,1.) for d in range(3)])
    norm=np.linalg.norm(E,axis=1);scaled=E/norm[:,None];Z=null_space(scaled);dim=Z.shape[1]
    origin=[];targets=[]
    for side in ('left','right'):
        jet=np.r_[edge_jet(a,a['edges'][side],cls[0].KappaStart,cls[0].dk),
                  edge_jet(b,b['edges'][side],cls[-1].KappaEnd,cls[-1].dk)]
        p=np.linalg.lstsq(scaled,jet/norm,rcond=None)[0];origin.append(p);targets.append(jet)
        if max(abs(E@p-jet))>1e-8:raise ValueError('inconsistent exact source end jets')
    A=[];rhs=[]
    for name,wl,wr in [('left',1.,0.),('right',0.,1.),('center',.5,.5)]:
        ss,t=project_observations(cls,raw[name]);V=B(ss/L);w=1/np.sqrt(len(ss))
        A.append(w*np.c_[wl*(V@Z),wr*(V@Z)])
        rhs.extend(w*(t-V@(wl*origin[0]+wr*origin[1])))
    # Degree-eight Bernstein coefficients give sufficient continuous linear
    # positivity constraints, without adding records or source observations.
    q=np.linspace(0,1,9);bern=np.array([[comb(8,j)*t**j*(1-t)**(8-j) for j in range(9)] for t in q])
    starts=np.r_[0.,np.cumsum([c.length for c in cls])];G=[];margin=[]
    for lo,hi in zip(kn[:-1],kn[1:]):
        s=lo+(hi-lo)*q;i=int(np.clip(np.searchsorted(starts,(lo+hi)/2,side='right')-1,0,len(cls)-1))
        c=cls[i];k=c.KappaStart+c.dk*(s-starts[i]);V=B(s/L)
        F=np.linalg.solve(bern,-k[:,None]*V);W=np.linalg.solve(bern,V)
        for side in (0,1):
            M=F@Z;zero=np.zeros_like(M);G.append(np.c_[M,zero] if side==0 else np.c_[zero,M])
            margin.extend(.9+F@origin[side])
        G.append(np.c_[W@Z,-W@Z]);margin.extend(W@(origin[0]-origin[1])-.1)
    G=np.vstack(G);margin=np.asarray(margin)
    z,qp=qp_step(sparse.csc_matrix(np.vstack(A)),-np.asarray(rhs),G,-margin,
        np.tile([-100.,100.],(2*dim,1)),np.full(2*dim,100.),damping=1e-8)
    if z is None:raise ValueError('bounded regular source-ribbon QP failed: '+str(qp['status']))
    coefficients=np.concatenate([origin[i]+Z@z[i*dim:(i+1)*dim] for i in (0,1)])
    co={side:np.array([[B(s/L,d)@coefficients[i*nc:(i+1)*nc]/L**d/(1,1,2,6)[d]
        for d in range(4)] for s in kn[:-1]]) for i,side in enumerate(('left','right'))}
    width=[]
    for poly,span in zip(co['left']-co['right'],np.diff(kn)):
        roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(poly))
        ss=[0.,span]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<span]
        width.extend(np.polynomial.polynomial.polyval(ss,poly))
    forward=float(minimum_forward_factor(cls,kn,co));gap=max(max(abs(E@coefficients[i*nc:(i+1)*nc]-targets[i])) for i in (0,1))
    if gap>1e-8 or min(width)<.1-1e-8 or forward<.1-1e-8:
        raise ValueError('actual regular-ribbon/end-jet readback failed')
    return coefficients,dict(status='REGULAR_RIBBON_INITIALIZATION_NOT_ACCEPTANCE',
        actual_ribbon_forward_minimum=forward,actual_ribbon_width_minimum_m=float(min(width)),
        maximum_end_jet_error=float(gap),qp=qp,reference_primitive_count=len(cls),
        minimum_reference_span_m=float(min(c.length for c in cls)),
        minimum_width_span_m=float(min(np.diff(kn))),internal_world_G2_satisfied=False,
        source_fidelity_accepted=False,production_accepted=False)
