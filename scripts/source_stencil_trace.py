"""Read-only diagnostics of the current kernel's source correspondence.

Uses the SAME fixed evaluation grid and raw polylines to explain a stencil,
not a new/looser geometry metric or a map acceptance renderer.
"""
import numpy as np
from mapforge.ops.fixed_ribbon_sampling import sample_fixed
from spikes.measured_connector_caps import distances,crop_at_projection


def projection_trace(curve,point,keep_after):
    g=np.asarray(curve);p=np.asarray(point);v=np.diff(g,axis=0);den=np.sum(v*v,axis=1)
    f=np.clip(np.sum((p-g[:-1])*v,axis=1)/np.maximum(den,1e-20),0,1)
    q=g[:-1]+f[:,None]*v;d=np.linalg.norm(q-p,axis=1);i=int(np.argmin(d))
    lengths=np.sqrt(den);stations=np.r_[0.,np.cumsum(lengths)][:-1]+f*lengths
    nonlocal_ids=np.flatnonzero(abs(stations-stations[i])>1.)
    competitor=None
    if len(nonlocal_ids):
        j=int(nonlocal_ids[np.argmin(d[nonlocal_ids])])
        competitor=dict(index=j,station_m=float(stations[j]),distance_m=float(d[j]),
                        distance_difference_m=float(d[j]-d[i]))
    cropped=crop_at_projection(g,p,keep_after)
    return cropped,dict(nearest_index=i,station_m=float(stations[i]),distance_m=float(d[i]),
        point=q[i].tolist(),raw_point=p.tolist(),curve_point_count=len(g),cropped_point_count=len(cropped),
        nonlocal_competitor=competitor)


def trace_turn(system,snapshot,cid):
    turn=snapshot.full['turns'][cid];r=turn['result'];support=turn['source_support']
    kernel=system.geometry.kernels[cid]
    points=sample_fixed(r[0],r[1],r[2],kernel['bounds'][:kernel['nr'],1],.1)
    tails={}
    for tail in support['tails']:
        for side in ('left','right'):
            after=tail['role']=='successor';source=tail['curves'][side];tip=source[0 if after else -1]
            cropped,info=projection_trace(points[side],tip,after)
            errors=distances(cropped,source);i=int(np.argmax(errors))
            tails[tail['role']+':'+side]=dict(info,maximum_m=float(errors[i]),worst_target_point=cropped[i].tolist(),
                kernel_maximum_m=float(max(r[10][tail['role']+':'+side])))
    return dict(road=cid,raw_counts={k:len(v) for k,v in support['raw'].items()},
        fairness=r[9],original_endpoint_headings=support.get('original_endpoint_headings'),
        physical_center_fragment_counts=[len(p) for p in support['physical_center_parts']],
        errors={k:{d:dict(count=len(v),maximum_m=float(max(v)),median_m=float(np.median(v)),
            p95_m=float(np.percentile(v,95))) for d,v in fields.items()} for k,fields in r[3].items()},
        tails=tails)
