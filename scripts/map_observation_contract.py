"""Read-only MAP/SHP observation contract probe; NEVER an identity override.

Keep every close SHP candidate and check raw TOPO paths between observations.
Normal cuts use associated original boundaries, not an arbitrary XODR chart.
No data repair, fitting, or map export is performed here.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import CASES,shp_source
from scripts.recheck_speed_contract import sha,dump
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops.map_to_xodr import _project


def project_point(p,g):
    a=np.asarray(g)[:-1];v=np.diff(g,axis=0);dd=np.sum(v*v,axis=1)
    valid=dd>1e-14
    if not valid.any():raise ValueError('degenerate source center')
    indices=np.flatnonzero(valid);a=a[valid];v=v[valid];dd=dd[valid]
    u=np.clip(np.sum((p-a)*v,axis=1)/dd,0.,1.);q=a+u[:,None]*v
    j=int(np.argmin(np.linalg.norm(q-p,axis=1)))
    return dict(distance_m=float(np.linalg.norm(q[j]-p)),point=q[j],tangent=v[j]/np.sqrt(dd[j]),
                source_segment=int(indices[j]),fraction=float(u[j]))


def normal_cut(center,tangent,boundaries):
    """Exact line/polyline intersections. Multiple hits remain ambiguous."""
    if len(boundaries)!=2:return dict(status='UNAVAILABLE',reason='expected two associated boundaries')
    t=np.asarray(tangent,float);n=np.array([-t[1],t[0]])
    if abs(np.linalg.norm(t)-1)>1e-8:raise ValueError('unit tangent required')
    intersections=[]
    for boundary in boundaries:
        g=np.asarray(boundary,float);values=[]
        for a,b in zip(g[:-1],g[1:]):
            v=b-a;den=float(v@t)
            if abs(den)<1e-12:
                if np.linalg.norm(v)>1e-12 and abs((a-center)@t)<1e-8:
                    return dict(status='UNAVAILABLE',reason='normal overlaps a boundary segment')
                continue
            u=float((center-a)@t/den)
            if -1e-8<=u<=1+1e-8:
                value=float((a+np.clip(u,0.,1.)*v-center)@n)
                if not any(abs(value-old)<1e-7 for old in values):values.append(value)
        intersections.append(sorted(values))
    if any(len(v)!=1 for v in intersections):
        return dict(status='UNAVAILABLE',reason='missing or ambiguous normal intersection',intersections=intersections)
    a,b=sorted(v[0] for v in intersections)
    return dict(status='OBSERVED',signed_intersections_m=[a,b],normal_width_m=b-a,
                center_outside_m=max(a,-b,0.))


def paths_between(starts,ends,successors,allowed,limit=2000):
    queue=[[s] for s in sorted(set(starts))];found=[];seen=0;depth_limited=False
    while queue and seen<limit:
        path=queue.pop(0);seen+=1;sid=path[-1]
        if sid in ends:found.append(path);continue
        if len(path)>=20:
            depth_limited |= any(n not in path and allowed(n) for n in successors.get(sid,[]))
            continue
        queue.extend(path+[n] for n in successors.get(sid,[]) if n not in path and allowed(n))
    return dict(status='SEARCH_LIMIT' if queue or depth_limited else ('PATH_FOUND' if found else 'NO_LOCAL_PATH'),paths=found,
                visited_states=seen,identity_confirmed=False)


def run(case,link_name,output):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('new output directory required')
    path=ROOT/'v2x_map_xml'/dict(CASES)[case];node=parse_map_xml(str(path))
    link=next(l for l in node.links if l.name==link_name)
    raw={l.lane_id:_project(l.points,node.ref_lat,node.ref_lon) for l in link.lanes}
    all_points=np.vstack(list(raw.values()));origin=all_points[0]
    chord=raw[link.lanes[0].lane_id][-1]-origin;t=chord/np.linalg.norm(chord);n=np.array([-t[1],t[0]])
    st=lambda xy:np.c_[(np.asarray(xy)-origin)@t,(np.asarray(xy)-origin)@n]
    proj=lambda xy:_project(xy,node.ref_lat,node.ref_lon)
    families={'IBD_LANE_LINK','IBD_LANE_LINK_MERGE','IBD_LANE_BOUNDARY','IBD_LANE_BOUNDARY_REL','IBD_LANE_TOPO_DETAIL'}
    files=[path,ROOT/'profiles/shp/ibd-smarteditor-v1.yaml']
    files += [p for p in (ROOT/'shp_0222-0326').rglob('*') if p.is_file() and p.stem in families]
    for d in ('mapforge','spikes','scripts'):files+=list((ROOT/d).rglob('*.py'))
    hashes={str(p.relative_to(ROOT)):sha(p) for p in files}
    src=shp_source();src._load_lanes();nearby={};boundaries={}
    for lane in src._lane_by_pid.values():
        if len(lane.geometry)<2:continue
        g=proj(lane.geometry)
        if np.any(g.max(axis=0)<all_points.min(axis=0)-2) or np.any(g.min(axis=0)>all_points.max(axis=0)+2):continue
        nearby[lane.lane_pid]=(lane,g)
    fig,axes=plt.subplots(2,1,figsize=(14,9),layout='constrained',dpi=150)
    rows=[];topology=[]
    for sid,(lane,g) in nearby.items():
        axes[0].plot(*st(g).T,color='#8ab9d4',lw=1,alpha=.8)
    for lane in link.lanes:
        xy=raw[lane.lane_id];xy_st=st(xy);previous=None
        axes[0].plot(*xy_st.T,'o-',ms=4,label=f'Original MAP lane {lane.lane_id}, width {lane.width_cm}cm')
        for i,p in enumerate(xy):
            candidates=[]
            for sid,(shp,g) in nearby.items():
                projection=project_point(p,g)
                if projection['distance_m']>.06:continue
                if sid not in boundaries:boundaries[sid]=[proj(g) for g in src.lane_boundary_geometries(sid)]
                cut=normal_cut(projection['point'],projection['tangent'],boundaries[sid])
                candidates.append(dict(source_lane_id=sid,nearest_center_m=projection['distance_m'],
                    source_segment=projection['source_segment'],fraction=projection['fraction'],
                    source_width_mm=shp.width_mm,source_start_width_mm=shp.s_width_mm,
                    source_end_width_mm=shp.e_width_mm,normal_cut=cut))
                if cut['status']=='OBSERVED':axes[1].scatter(xy_st[i,0],cut['normal_width_m'],marker='x',color='#bb4e3f')
            row=dict(map_lane=lane.lane_id,point_index=i,raw_lonlat=list(lane.points[i]),local_xy=p.tolist(),
                     s_m=float(xy_st[i,0]),explicit_width_cm=lane.width_cm,shp_candidates=candidates)
            rows.append(row)
            ids={c['source_lane_id'] for c in candidates}
            if previous is not None:
                lo,hi=sorted([previous['s_m'],row['s_m']])
                def allowed(sid):
                    if sid not in nearby:return False
                    g=st(nearby[sid][1])
                    return g[-1,0]>=g[0,0] and g[0,0]<=hi+.5 and g[-1,0]>=lo-.5
                relation=paths_between(previous['ids'],ids,src.topo_out,allowed)
                topology.append(dict(map_lane=lane.lane_id,point_indices=[i-1,i],**relation))
            previous=dict(s_m=row['s_m'],ids=ids)
            if lane.width_cm is not None:axes[1].scatter(xy_st[i,0],lane.width_cm/100,color='#27754c',s=15)
    for gs in boundaries.values():
        for g in gs:axes[0].plot(*st(g).T,color='#888888',alpha=.5,lw=.7)
    axes[0].legend(fontsize=8);axes[0].set(ylabel='transverse t in display chart (m)')
    axes[1].plot([],[],'o',color='#27754c',label='MAP supplied scalar')
    axes[1].plot([],[],'x',color='#bb4e3f',label='Nearby SHP associated-boundary normal cut')
    axes[1].legend();axes[1].set(xlabel='longitudinal display coordinate (m)',ylabel='width (m)')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle(f'{case}/{link_name}: original MAP + SHP observations only\nNearest/path candidates do NOT authorize ID binding or MAP correction; transverse scale exaggerated')
    if any(sha(ROOT/p)!=h for p,h in hashes.items()):raise ValueError('source or code changed during probe')
    output.mkdir(parents=True);fig.savefig(output/'observations.png');plt.close(fig)
    dump(output/'report.json',dict(status='DIAGNOSTIC_ONLY',source_and_code_sha256=hashes,
        case=case,link=link_name,source_changed=False,identity_confirmed=False,xodr_emitted=False,
        projection=dict(lat_0=node.ref_lat,lon_0=node.ref_lon,method='existing local eqc; not absolute CRS validation'),
        points=rows,local_topology=topology,
        nearby_source_geometry={sid:dict(points_lonlat=np.asarray(l.geometry).tolist(),
             boundaries_lonlat=[np.asarray(g).tolist() for g in src.lane_boundary_geometries(sid)],
             successors=src.topo_out.get(sid,[])) for sid,(l,_) in nearby.items()}))
    print(case,link_name,len(rows),'raw points',len(topology),'topology checks',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('case',choices=dict(CASES));p.add_argument('link');p.add_argument('output')
    a=p.parse_args();run(a.case,a.link,a.output)
