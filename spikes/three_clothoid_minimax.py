"""Fixed three long clothoids: endpoint G2, minimax curvature/sharpness.

Research only. No speed changes, no extra primitive, no previous geometry as
source truth. A locally best result is not a global feasibility proof.
"""
import math
import numpy as np
from pyclothoids import Clothoid
from scipy.optimize import minimize
from mapforge.ops.refline_fit import solve_g2_balanced


def optimize(p0, p1, k0=0., k1=0., *, speed_kmh=15., ay_limit=2.5,
             jerk_limit=1., length_growth=.10, minimum_primitive_m=5.):
    base, meta = solve_g2_balanced(p0, p1, k0, k1)
    total = sum(c.length for c in base)
    v = speed_kmh / 3.6
    if min(v, ay_limit, jerk_limit) <= 0:
        raise ValueError('positive explicit evaluation limits required')
    kcap, dcap = ay_limit / v**2, jerk_limit / v**3
    scale = max(total, 1.)
    turn = (p1[2]-p0[2]+math.pi) % (2*math.pi)-math.pi
    monotone_turn=math.pi/6<abs(turn)<5*math.pi/6 and max(abs(k0),abs(k1))<1e-10

    def unpack(z):
        return z[:3]*scale, np.r_[k0, z[3:5]/scale, k1]

    def chain(z):
        lens, ks = unpack(z)
        x, y, h = map(float, p0)
        result = []
        for length, a, b in zip(lens, ks, ks[1:]):
            c = Clothoid.StandardParams(x, y, h, a, (b-a)/length, length)
            result.append(c)
            x, y, h = c.XEnd, c.YEnd, c.ThetaEnd
        return tuple(result)

    def ratios(z):
        lens, ks = unpack(z)
        return np.r_[np.abs(ks)/kcap, np.abs(np.diff(ks)/lens)/dcap]

    def endpoint(z):
        end = chain(z)[-1]
        return np.array([(end.XEnd-p1[0])/scale, (end.YEnd-p1[1])/scale,
                         end.ThetaEnd-p0[2]-turn])

    def inequalities(z):
        lens, ks = unpack(z)
        # No micro-segments or increase of total length by adding a loop.
        values = [z[5]-ratios(z), [total*(1+length_growth)-sum(lens)],
                  lens-.10*sum(lens)]
        # Exact heading extrema of each quadratic heading polynomial. Keep
        # the original turn branch, allowing a small endpoint-curvature flare.
        h = p0[2]
        lo, hi = sorted((h, h+turn))
        headings = [h]
        for length, a, b in zip(lens, ks, ks[1:]):
            d = (b-a)/length
            t=float(np.clip(-a/d,0.,length)) if abs(d)>1e-14 else 0.
            headings.append(h+a*t+.5*d*t*t)
            h += .5*(a+b)*length; headings.append(h)
        values += [np.asarray(headings)-lo+math.radians(5),
                   hi+math.radians(5)-np.asarray(headings)]
        return np.concatenate(values)

    z0=np.r_[[c.length/scale for c in base],
              np.array([base[0].KappaEnd,base[1].KappaEnd])*scale,0.]
    z0[-1]=max(ratios(z0))
    peak=max(abs(c.KappaStart) for c in base)
    peak=max(peak,abs(base[-1].KappaEnd),.01)*1.1
    bounds=[(max(minimum_primitive_m,.03*total)/scale,total*(1+length_growth)/scale)]*3
    kbounds=(0.,peak*scale) if turn>0 else (-peak*scale,0.)
    bounds += [kbounds if monotone_turn else (-peak*scale,peak*scale)]*2+[(0.,max(100.,2*z0[-1]))]
    sol=minimize(lambda z:z[5],z0,method='SLSQP',bounds=bounds,
                 constraints=[{'type':'eq','fun':endpoint},
                              {'type':'ineq','fun':inequalities}],
                 options={'maxiter':300,'ftol':1e-11})
    valid=(sol.success and np.max(np.abs(endpoint(sol.x)))<1e-7
           and min(inequalities(sol.x))>-1e-7
           and max(ratios(sol.x))<=z0[-1]+1e-7)
    chosen=chain(sol.x) if valid else base
    chosen_lengths=np.array([c.length for c in chosen])
    chosen_k=np.array([c.KappaStart for c in chosen]+[chosen[-1].KappaEnd])
    shape_ok=(min(chosen_lengths)>=minimum_primitive_m-1e-7
              and min(chosen_lengths)>=.10*sum(chosen_lengths)-1e-7
              and (not monotone_turn or min(np.sign(turn)*chosen_k)>=-1e-9))
    selected={'length_m':sum(c.length for c in chosen),
              'segment_lengths_m':[c.length for c in chosen],
              'min_primitive_m':min(c.length for c in chosen),
              'kappa_max_per_m':max(max(abs(c.KappaStart),abs(c.KappaEnd)) for c in chosen),
              'sharpness_max_per_m2':max(abs(c.dk) for c in chosen)}
    return chosen,dict(meta, selected=selected, balanced_seed=meta['selected'],
        optimizer='fixed-three-clothoid-minimax',
        evaluation_speed_kmh=speed_kmh, source_speed_changed=False,
        baseline_dynamic_ratio=float(z0[-1]),
        dynamic_ratio=float(max(ratios(sol.x))) if valid else float(z0[-1]),
        optimized=bool(valid), solver_message=str(sol.message),
        global_optimality_claimed=False, length_growth_cap=length_growth,
        ay_limit_mps2=ay_limit, jerk_limit_mps3=jerk_limit,
        primitive_count=3, minimum_relative_length=.10,minimum_primitive_m=minimum_primitive_m,
        monotone_turn_required=monotone_turn,shape_constraints_satisfied=bool(shape_ok),
        evaluation_status='PASS' if max(selected['kappa_max_per_m']/kcap,
            selected['sharpness_max_per_m2']/dcap)<=1.+1e-7 else 'FAIL')
