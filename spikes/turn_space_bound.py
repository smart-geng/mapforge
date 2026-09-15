"""Necessary displacement bound for a monotone, zero-end-curvature turn.

With heading theta increasing from 0 to phi, d(k^2)/dtheta=2*k'.
Thus k(theta) <= min(K, sqrt(2*D*theta), sqrt(2*D*(phi-theta))).
Projection on the heading bisector is integral cos(theta-phi/2)/k dtheta.
For 0<phi<pi its weight is positive, giving a necessary lower bound.
This does not decide feasibility for nonmonotone turns, movable mouths, or
all-map reconstruction. Quadrature error is explicitly reported.
"""
import math
import numpy as np
from scipy.integrate import quad


def audit(p0,p1,k0=0.,k1=0.,*,speed_kmh=15.,ay_limit=2.5,jerk_limit=1.):
    turn=(p1[2]-p0[2]+math.pi)%(2*math.pi)-math.pi
    phi=abs(turn)
    context={'scope':'fixed mouths, monotone heading, zero endpoint curvature',
             'global_infeasibility_claimed':False,'speed_kmh':speed_kmh}
    if not 1e-5<phi<math.pi-1e-5 or max(abs(k0),abs(k1))>1e-10:
        return dict(context,status='NOT_APPLICABLE')
    v=speed_kmh/3.6
    if min(v,ay_limit,jerk_limit)<=0:raise ValueError('positive explicit limits required')
    cap=ay_limit/v**2;rate=jerk_limit/v**3
    # theta=phi*sin(u)^2 removes the endpoint square-root singularities.
    def integrand(u):
        theta=phi*math.sin(u)**2
        return (math.cos(theta-phi/2)*2*phi*math.sin(u)*math.cos(u)
                /min(cap,math.sqrt(2*rate*theta),math.sqrt(2*rate*(phi-theta))))
    corners=[phi/2,cap**2/(2*rate),phi-cap**2/(2*rate)]
    split=sorted({0.,math.pi/2,*[math.asin(math.sqrt(t/phi)) for t in corners if 0<t<phi]})
    values=[quad(integrand,a,b,epsabs=1e-9,epsrel=1e-10) for a,b in zip(split,split[1:])]
    lower=sum(x[0] for x in values);error=sum(x[1] for x in values)
    h=p0[2]+turn/2
    actual=float(np.dot(np.asarray(p1[:2])-p0[:2],[math.cos(h),math.sin(h)]))
    return dict(context,status='VIOLATES_NECESSARY_BOUND' if lower-actual>max(1e-6,10*error)
                else 'NOT_EXCLUDED',turn_rad=turn,projected_displacement_m=actual,
                necessary_minimum_m=lower,quadrature_error_estimate_m=error,
                deficit_m=lower-actual)
