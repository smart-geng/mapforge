"""Bounded long-primitive reference family, never per-source-point segments."""
import numpy as np
from pyclothoids import Clothoid
from scipy.optimize import root


def chain(parameters,a,b,caps,core_count=3):
    if core_count not in (2,3,4) or not 3<=core_count+sum(caps)<=5:
        raise ValueError('two/three/four cores with at least three and at most five total primitives')
    n=core_count+sum(caps);q=np.asarray(parameters,float)
    if q.shape!=(n+core_count-1,) or not np.isfinite(q).all() or min(q[:n])<=0:
        raise ValueError('finite positive long-chain state required')
    lengths=q[:n];ks=[a['k']]
    if caps[0]:ks.append(a['k']+a['dk']*lengths[0])
    ks.extend(q[n:]/20.)
    if caps[1]:ks.append(b['k']-b['dk']*lengths[-1])
    ks.append(b['k']);x,y,h=a['pose'];out=[]
    for length,k0,k1 in zip(lengths,ks[:-1],ks[1:]):
        c=Clothoid.StandardParams(x,y,h,k0,(k1-k0)/length,length);out.append(c)
        x,y,h=c.XEnd,c.YEnd,c.ThetaEnd
    return out


def expand_core(parameters,a,b,caps):
    """Losslessly split ONE >=12m core into two >=6m model intervals.

    This initial curve is unchanged, but the later solve gets one extra long
    reference degree of freedom. Source count/spacing never enters selection.
    """
    cls=chain(parameters,a,b,caps,3)
    if len(cls)>=5:raise ValueError('five-reference budget already exhausted')
    start=int(caps[0]);indices=list(range(start,start+3))
    index=max(indices,key=lambda i:cls[i].length)
    if cls[index].length<12.-1e-8:raise ValueError('no core can split into two long intervals')
    split=cls[index];half=split.length/2;pieces=cls[:index]+[
        Clothoid.StandardParams(split.XStart,split.YStart,split.ThetaStart,split.KappaStart,split.dk,half),
        Clothoid.StandardParams(split.X(half),split.Y(half),split.Theta(half),split.KappaStart+split.dk*half,split.dk,half)
    ]+cls[index+1:]
    return np.r_[[c.length for c in pieces],[pieces[i].KappaEnd*20 for i in range(start,start+3)]]


def polish_endpoint(parameters,a,b,caps,core_count=3):
    q=np.asarray(parameters,float);n=core_count+sum(caps)
    indices=([int(caps[0]),int(caps[0])+1,n] if core_count==2 else [int(caps[0])+1,n,n+1])
    def residual(values):
        work=q.copy();work[indices]=values;end=chain(work,a,b,caps,core_count)[-1]
        dh=end.ThetaEnd-b['pose'][2]
        return np.array([end.XEnd-b['pose'][0],end.YEnd-b['pose'][1],20*np.arctan2(np.sin(dh),np.cos(dh))])
    solved=root(residual,q[indices],tol=1e-10);new=q.copy();new[indices]=solved.x
    return new,dict(solver_success=bool(solved.success),before=residual(q[indices]).tolist(),
        after=residual(solved.x).tolist(),shape_shift_max=float(max(abs(new-q))))
