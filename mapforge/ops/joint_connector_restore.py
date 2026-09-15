"""Project an already source-fitted joint trial back onto exact end/world-G2 jets.

This is numerical feasibility restoration, not geometric approval. Keep the
same reference/width basis and source observations; independently recheck the
written source error, positive width, regularity and all parent interfaces.
"""
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import least_squares

from mapforge.ops.joint_connector_fit import basis_for
from spikes.measured_connector_caps import chain, needs_cap, polish_endpoint, write_ribbon
from spikes.connector_cross_section import edge_jet


def restore(road, a, b, parameters, coefficients,core_count=3):
    from mapforge.ops.long_connector_chain import chain as make_chain,polish_endpoint as close_chain
    caps=(needs_cap(a), needs_cap(b)); nr=core_count+sum(caps)
    q=np.asarray(parameters,float)
    if q.shape!=(nr+core_count-1,) or not np.isfinite(q).all():
        raise ValueError('finite same-parent reference state required')
    q,polish=close_chain(q,a,b,caps,core_count)
    if min(q[:nr])<6.-1e-8 or max(abs(np.asarray(polish['after'])))>1e-7:
        raise ValueError('exact closure would violate the long-reference basis')
    cls=make_chain(q,a,b,caps,core_count);kn,B,jumps=basis_for(cls);L=kn[-1];n=len(B.c)
    if min(np.diff(kn))<3.-1e-8:raise ValueError('short independent width span')
    old=np.asarray(coefficients,float)
    if old.shape!=(2*n,) or not np.isfinite(old).all():
        raise ValueError('complete finite same-basis transverse coefficients required')
    old=old.reshape(2,n)
    E=np.array([B(t,d)/L**d for t in (0.,1.) for d in range(3)])
    norms=np.linalg.norm(E,axis=1);EE=E/norms[:,None];Z=null_space(EE)
    particular=[]
    for side in ('left','right'):
        target=np.r_[edge_jet(a,a['edges'][side],cls[0].KappaStart,cls[0].dk),
                     edge_jet(b,b['edges'][side],cls[-1].KappaEnd,cls[-1].dk)]
        particular.append(np.linalg.lstsq(EE,target/norms,rcond=None)[0])
    dim=Z.shape[1]
    z0=np.concatenate([Z.T@(old[i]-particular[i]) for i in (0,1)])
    k=np.array([c.KappaEnd for c in cls[:-1]]);dj=np.diff([c.dk for c in cls])
    B0=B(jumps); B1=B(jumps,1)/L
    D2=(B(jumps,2)-B(np.nextafter(jumps,-np.inf),2))/L**2
    def coeff(z):return [particular[i]+Z@z[i*dim:(i+1)*dim] for i in (0,1)]
    def constraint(z, jac=False):
        residual=[]; rows=[]; slopes=[]; derivatives=[]
        for i,c in enumerate(coeff(z)):
            t=B0@c;dt=B1@c;den=1-k*t
            if min(abs(den))<1e-9:raise ValueError('singular transverse join')
            residual.extend(20*(D2@c+dj*t*dt/den))
            J=20*(D2+dj[:,None]*(dt[:,None]/den[:,None]**2*B0+(t/den)[:,None]*B1))@Z
            rows.append(np.c_[J,np.zeros_like(J)] if i==0 else np.c_[np.zeros_like(J),J])
            slopes.append(dt/den)
            derivatives.append((B1/den[:,None]+(k*dt/den**2)[:,None]*B0)@Z)
        residual.extend(slopes[0]-slopes[1]);rows.append(np.c_[derivatives[0],-derivatives[1]])
        return np.vstack(rows) if jac else np.asarray(residual)
    solved=least_squares(constraint,z0,jac=lambda z:constraint(z,True),
        max_nfev=200,ftol=1e-13,xtol=1e-13,gtol=1e-13)
    residual=float(max(abs(constraint(solved.x))))
    if residual>1e-9:raise ValueError('world-G2 projection not converged')
    co={side:np.array([[B(s/L,d)@c/L**d/(1,1,2,6)[d] for d in range(4)] for s in kn[:-1]])
        for side,c in zip(('left','right'),coeff(solved.x))}
    return write_ribbon(road,cls,kn,co),dict(road=road.get('id'),status='PROJECTED_REQUIRES_INDEPENDENT_REVIEW',
        shape_parameters=q.tolist(),joint_coefficients=np.concatenate(coeff(solved.x)).tolist(),reference_core_count=core_count,
        primitive_lengths_m=[c.length for c in cls],width_records=len(kn)-1,
        minimum_width_record_span_m=float(min(np.diff(kn))),projection_residual=residual,
        projection_coefficient_change=float(np.max(abs(np.array(coeff(solved.x))-old))),
        reference_closure=polish,geometry_accepted=False,production_accepted=False)
