"""Analytic construction checks, not a substitute for dynamic/source gates."""
import math
import numpy as np


def turn_branch(target, seed):
    """Keep the existing half-turn side; never admit an extra revolution."""
    if not all(math.isfinite(v) for v in (target,seed)):
        raise ValueError('initial connector heading must be finite')
    turn=(target+math.pi)%(2*math.pi)-math.pi
    if abs((seed-turn+math.pi)%(2*math.pi)-math.pi)>1e-6 or abs(seed)>math.pi+1e-6:
        raise ValueError('initial connector has an inconsistent heading or extra revolution')
    if abs(abs(turn)-math.pi)<1e-6:
        if turn*seed<0:turn+=math.copysign(2*math.pi,seed)
    return turn


def heading_envelope(turn):
    # A near-straight cross-lane connection needs a single S transition.
    # A +/-45 degree bisector cone guarantees forward motion in this category;
    # it is an explicit construction policy, not an OpenDRIVE requirement.
    if abs(turn)<=math.pi/6:
        return turn/2,math.pi/4,'forward-single-S'
    return turn/2,abs(turn)/2+math.radians(5),'bounded-turn'


def heading_values(lengths,curvatures,turn):
    """Normalized exact quadratic-heading extrema, including every endpoint."""
    mid,half,_=heading_envelope(turn);theta=0.;result=[]
    for length,a,b in zip(lengths,curvatures,curvatures[1:]):
        rate=(b-a)/length
        u=float(np.clip(-a/rate,0.,length)) if abs(rate)>1e-14 else 0.
        result.append((theta+a*u+.5*rate*u*u-mid)/half)
        theta+=.5*(a+b)*length;result.append((theta-mid)/half)
    return np.asarray(result)
