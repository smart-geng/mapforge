"""Read-only source/written cross-section diagnosis (no excluded driving lanes)."""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import shp_source
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from mapforge.validate.smoothness import (sample_road_ref, lane_edges_kinematics_at,
    _edge_world_curvature, _ref_kappa_at)


def project_st(points, road):
    xy, ss, hh = sample_road_ref(road, .05)
    _, ids = cKDTree(xy).query(points)
    d = points - xy[ids]
    h = hh[ids]
    return np.column_stack([ss[ids]+d[:,0]*np.cos(h)+d[:,1]*np.sin(h),
                            -d[:,0]*np.sin(h)+d[:,1]*np.cos(h)])


def inspect(path, rid, start, end, output, compare=None):
    root = etree.parse(str(path)).getroot()
    road = next(r for r in root.findall('road') if r.get('id') == rid)
    old = None
    if compare:
        old = next(r for r in etree.parse(str(compare)).findall('road') if r.get('id') == rid)
    src = shp_source()
    lat0, lon0 = _origin(root)
    sections = road.findall('lanes/laneSection')
    source_ids, details = set(), []
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, dpi=150)
    for si, sec in enumerate(sections):
        s0 = float(sec.get('s'))
        s1 = float(sections[si+1].get('s')) if si+1<len(sections) else float(road.get('length'))
        if s1<start or s0>end:
            continue
        q = np.linspace(max(start,s0)+1e-6, min(end,s1)-1e-6,
                        max(3,int((min(end,s1)-max(start,s0))/.05)+1))
        row = {'s0':s0,'s1':s1,'lanes':[]}
        for side in ('left','right'):
            lanes = sorted(sec.findall(f'{side}/lane'), key=lambda l:abs(int(l.get('id'))))
            edges = np.array([lane_edges_kinematics_at(road,float(s),side) for s in q])
            for i in range(edges.shape[1]):
                axes[0].plot(q,edges[:,i,0],color='royalblue',lw=.9)
            if old is not None:
                old_edges = [lane_edges_kinematics_at(old,float(s),side) for s in q]
                for i in range(min(map(len,old_edges))):
                    axes[0].plot(q,[e[i][0] for e in old_edges],color='gray',lw=.6,ls='--')
            for i,lane in enumerate(lanes):
                source = lane.find("userData[@code='mapforge.source_lane']")
                sid = source.get('value') if source is not None else None
                if sid: source_ids.add(sid)
                center = .5*(edges[:,i]+edges[:,i+1])
                kr = np.array([_edge_world_curvature(*c,*_ref_kappa_at(road,float(s)))
                               for c,s in zip(center,q)])
                axes[1].plot(q,kr,lw=1,label=lane.get('id') if not details else None)
                axes[2].plot(q,center[:,2],lw=1)
                row['lanes'].append({'lane_id':lane.get('id'),'type':lane.get('type'),
                    'source_id':sid,'widths':[dict(w.attrib) for w in lane.findall('width')],
                    'center_start':center[0].tolist(),'center_end':center[-1].tolist(),
                    'kappa_max':float(max(abs(kr)))})
        details.append(row)
        for ax in axes: ax.axvline(s0,color='k',alpha=.15,lw=.7)
    for sid in sorted(source_ids):
        rec=src.lane(sid)
        if rec is None: continue
        for raw in src.lane_boundary_geometries(sid):
            st=project_st(_project(raw,lat0,lon0),road)
            axes[0].plot(*st.T,color='darkorange',lw=.8,alpha=.75)
        st=project_st(_project(rec.geometry,lat0,lon0),road)
        axes[0].plot(*st.T,color='seagreen',lw=.65,ls='--',alpha=.7)
    axes[0].set_title('Raw SHP boundaries (orange), centers (green); written edges (blue), old (gray)')
    axes[0].set_ylabel('transverse t (m)')
    axes[1].set_ylabel('lane center curvature (1/m)')
    axes[2].set_ylabel('center d2t/ds2 (1/m)')
    axes[2].set_xlabel('reference s (m)')
    axes[0].set_xlim(start,end)
    for ax in axes: ax.grid(alpha=.2)
    fig.suptitle(f'{path.parent.name}/{path.stem} road {rid}')
    fig.tight_layout()
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output)
    plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps(details,indent=2),encoding='utf-8')
    for r in details:
        print(r['s0'],r['s1'],[(l['lane_id'],l['source_id'],round(l['kappa_max'],5)) for l in r['lanes']])


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('xodr',type=Path); p.add_argument('road_id')
    p.add_argument('start',type=float); p.add_argument('end',type=float)
    p.add_argument('output',type=Path); p.add_argument('--compare',type=Path)
    a=p.parse_args()
    inspect(a.xodr,a.road_id,a.start,a.end,a.output,a.compare)
