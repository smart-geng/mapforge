"""Read-only check of SHP lane centers versus their associated boundaries.

No source record is repaired or excluded. A taper's surveyed path can join an
adjacent lane center even where its boundary width reaches zero; this report
makes that distinction observable instead of assuming midpoint equivalence.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.smoothness import sample_road_ref


def compare_lane(center_st, boundaries_st):
    curves=[]
    for raw in boundaries_st:
        g=np.asarray(raw,float)
        if np.all(np.diff(g[:,0])<=1e-8): g=g[::-1]
        if np.any(np.diff(g[:,0])<-.05):
            return {'status':'UNRESOLVED','reason':'non-monotone boundary projection'}
        g=g[np.r_[True,np.diff(g[:,0])>1e-7]]
        if len(g)>=2: curves.append(g)
    if len(curves)!=2:
        return {'status':'UNRESOLVED','reason':'expected exactly two associated boundaries'}
    observations=[]
    for index,(s,t) in enumerate(center_st):
        # Endpoint cross-sections are slanted in measured data. Only permit a
        # 5cm projection tolerance; never extrapolate an entire absent branch.
        if not all(g[0,0]-.05<=s<=g[-1,0]+.05 for g in curves): continue
        edges=sorted(float(np.interp(s,g[:,0],g[:,1])) for g in curves)
        width=edges[1]-edges[0]; midpoint=sum(edges)/2
        observations.append({'index':index,'s_m':float(s),'center_t_m':float(t),
            'boundary_t_m':edges,'boundary_width_m':width,
            'center_midpoint_difference_m':float(abs(t-midpoint)),
            'center_outside_distance_m':float(max(edges[0]-t,t-edges[1],0))})
    if not observations: return {'status':'UNRESOLVED','reason':'no common source support'}
    return {'status':'OBSERVED','samples':observations,
            'center_midpoint_max_m':max(o['center_midpoint_difference_m'] for o in observations),
            'center_outside_max_m':max(o['center_outside_distance_m'] for o in observations)}


def run(path,road_id,output):
    root=etree.parse(str(path)).getroot(); lat,lon=_origin(root)
    road=next(r for r in root.findall('road') if r.get('id')==road_id)
    xy,ss,hh=sample_road_ref(road,.025); kd=cKDTree(xy)
    def project_st(raw):
        p=_project(raw,lat,lon); _,ii=kd.query(p); delta=p-xy[ii]; h=hh[ii]
        return np.c_[ss[ii]+delta[:,0]*np.cos(h)+delta[:,1]*np.sin(h),
                     -delta[:,0]*np.sin(h)+delta[:,1]*np.cos(h)]
    ids=sorted({u.get('value') for u in road.findall(".//lane/userData[@code='mapforge.source_lane']")})
    src=shp_source(); rows=[]; payload={}
    for sid in ids:
        lane=src.lane(sid)
        if lane is None: rows.append({'source_lane_id':sid,'status':'MISSING'}); continue
        center=project_st(lane.geometry); edges=[project_st(g) for g in src.lane_boundary_geometries(sid)]
        row={'source_lane_id':sid,'source_width_start_mm':lane.s_width_mm,
             'source_width_end_mm':lane.e_width_mm,**compare_lane(center,edges)}
        rows.append(row); payload[sid]=(center,edges)
    worst=max((r for r in rows if r['status']=='OBSERVED'),key=lambda r:r['center_outside_max_m'])
    center,edges=payload[worst['source_lane_id']]
    fig,axes=plt.subplots(2,1,figsize=(12,8),dpi=160,sharex=True)
    for j,g in enumerate(edges): axes[0].plot(*g.T,'o-',label=f'Raw boundary {j+1}',lw=2,ms=4)
    axes[0].plot(*center.T,'o--',color='green',label='Raw LANE_LINK points',lw=2,ms=5)
    for o in worst['samples']:
        if o['center_outside_distance_m']>.1:
            axes[0].annotate(f"outside {o['center_outside_distance_m']:.2f}m",(o['s_m'],o['center_t_m']),
                             xytext=(10,-20),textcoords='offset points',color='crimson')
    axes[0].legend(); axes[0].set_ylabel('reference transverse t (m)')
    samples=worst['samples']; st=[o['s_m'] for o in samples]
    axes[1].plot(st,[o['boundary_width_m'] for o in samples],'o-',label='Boundary-derived width')
    axes[1].plot(st,[o['center_midpoint_difference_m'] for o in samples],'o-',label='Center vs midpoint')
    axes[1].set_ylabel('meters'); axes[1].set_xlabel('reference s (m)'); axes[1].legend()
    for ax in axes: ax.grid(alpha=.25)
    fig.suptitle(f"Original SHP only: road {road_id}, source lane {worst['source_lane_id']}")
    fig.tight_layout(); output.parent.mkdir(parents=True,exist_ok=True); fig.savefig(output); plt.close(fig)
    report={'input':str(path),'road_id':road_id,'source_modified':False,
            'interpretation':'observation only; no automatic source repair, no eligibility exclusion',
            'lanes':rows,'worst_source_lane':worst}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'source_lanes':len(rows),'worst_source_lane':worst},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('xodr',type=Path); p.add_argument('road_id'); p.add_argument('output',type=Path)
    a=p.parse_args(); run(a.xodr,a.road_id,a.output)
