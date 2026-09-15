"""Fixed evaluation sites for differentiable long-ribbon construction.

Counts are fixed from the optimization box's maximum primitive lengths.
No measured point creates a fit knot, and no evaluation site is written out.
Independent XML readback still uses its own denser sampling implementation.
"""
import numpy as np
from mapforge.ops.source_connector_ribbon import reference_samples


def sample_fixed(cls, knots, co, maximum_lengths, step):
    maximum_lengths=np.asarray(maximum_lengths,float)
    lengths=np.array([c.length for c in cls])
    if (maximum_lengths.shape!=lengths.shape or not np.isfinite(maximum_lengths).all()
        or min(maximum_lengths)<=0 or not np.isfinite(step) or step<=0
        or np.any(lengths>maximum_lengths+1e-8)):
        raise ValueError('fixed grid requires a finite bounding length for every primitive')
    starts=np.r_[0.,np.cumsum(lengths)]
    # Even interval counts always include each primitive midpoint, hence every
    # current structural width knot. Do not unique/append moving knots: exact
    # coincidences would change sample counts under finite differences.
    sites=np.concatenate([starts[i]+np.linspace(0,L,2*int(np.ceil(upper/(2*step)))+1)[:-1]
        for i,(L,upper) in enumerate(zip(lengths,maximum_lengths))]+[starts[-1:]])
    xy,normal=reference_samples(cls,sites)
    idx=np.clip(np.searchsorted(knots,sites,side='right')-1,0,len(knots)-2)
    u=sites-np.asarray(knots)[idx]
    points={}
    for side in ('left','right'):
        a,b,c,d=co[side][idx].T
        points[side]=xy+(a+u*(b+u*(c+u*d)))[:,None]*normal
    points['center']=(points['left']+points['right'])/2
    return points
