"""Research transverse family: one global freedom beyond fixed seven spans.

Eight cubic spans are fixed by the five reference lengths. The additional
knot bisects the longest middle primitive; it is NOT fitted to sample noise.
Endpoint jets, C2 joins and t'=0 at reference sharpness jumps remain exact.
The sole remaining scalar minimizes integral squared second derivative.
This is a diagnostic alternative, not enabled in production conversion.
"""
import numpy as np
from scipy.linalg import null_space
from spikes.connector_cross_section import jet


def fair_join_spline(cls, start, end):
    lens=np.array([c.length for c in cls]);length=sum(lens)
    ref=np.r_[0.,np.cumsum(lens)]
    middle=1+int(np.argmax(lens[1:-1]))
    knots=np.sort(np.r_[ref,lens[0]/2,length-lens[-1]/2,
                        (ref[middle]+ref[middle+1])/2])
    spans=np.diff(knots)/length;n=len(spans)
    flat=[int(np.argmin(abs(knots-s))) for s in ref[1:-1]]
    scale=np.array([1.,length,length**2])
    def residual(v):
        c=v.reshape(n,4)
        r=[*(jet(c[0],0)-start*scale),*(jet(c[-1],spans[-1])-end*scale)]
        for i in range(n-1):r.extend(jet(c[i+1],0)-jet(c[i],spans[i]))
        r.extend(c[i,1] for i in flat)
        return np.asarray(r)
    z=np.zeros(4*n);rhs=residual(z)
    A=np.column_stack([residual(e)-rhs for e in np.eye(4*n)])
    p=np.linalg.lstsq(A,-rhs,rcond=None)[0];N=null_space(A)
    if N.shape[1]!=1:raise ValueError('expected one global transverse freedom')
    # Exact integral of (2c+6du)^2 on each normalized interval.
    Q=np.zeros((4*n,4*n))
    for i,h in enumerate(spans):
        Q[4*i+2:4*i+4,4*i+2:4*i+4]=[[4*h,6*h*h],[6*h*h,12*h**3]]
    q=N.T@Q@N
    p-=N@np.linalg.solve(q,N.T@Q@p)
    if max(abs(residual(p)))>1e-7:raise ValueError('transverse constraints inaccurate')
    return knots,p.reshape(n,4)/np.array([1.,length,length**2,length**3])


def probe(source,target):
    """Record rejected/unevaluated mathematical alternatives, never maps."""
    import json
    import xml.etree.ElementTree as ET
    from spikes.measured_connector_caps import frames,seed_chain,sample
    from spikes.connector_cross_section import edge_jet
    root=ET.parse(source).getroot();rs={r.get('id'):r for r in root.findall('road')}
    rows=[]
    for rid in ('104','108','122'):
        a,b=frames(root,rs[rid]);cs=seed_chain(a,b,6.,6.);co={}
        for side in ('left','right'):
            kn,co[side]=fair_join_spline(cs,edge_jet(a,a['edges'][side],a['k'],a['dk']),
                                       edge_jet(b,b['edges'][side],b['k'],b['dk']))
        _,j,d=sample(cs,kn,co)
        rows.append({'road':rid,'status':'DIAGNOSTIC_NOT_ACCEPTED','width_records':len(kn)-1,
                     'sampled_min_width_m':float(min(j['left'][:,0]-j['right'][:,0])),
                     'sampled_dynamics_ratio_15kmh':float(np.max(abs(d)*[(15/3.6)**2/2.5,(15/3.6)**3])),
                     'source_fidelity_checked':False})
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps({'status':'BLOCKED','source':str(source),'rows':rows},indent=2),encoding='utf-8')
    return rows


if __name__=='__main__':
    import argparse
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path)
    a=p.parse_args();print(probe(a.source,a.target));raise SystemExit(2)
