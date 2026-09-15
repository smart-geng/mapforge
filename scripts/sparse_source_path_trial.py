"""Fit an original linked SHP path with 1/3/5 G2 primitives; no XODR export."""
import argparse
import json
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source,_sha256
from mapforge.ops.lane_family_border import _linked_chains
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from spikes.sparse_path_model import fit,Limits,sample,observations
from spikes.straight_transition_model import fit_transition
from pyclothoids import Clothoid


def source_path(road,src,side,lane_id,project):
    sections=road.findall('lanes/laneSection')
    occ,_,chains=_linked_chains(sections,side)
    matching=[c for c in chains if c[-1]==(len(sections)-1,str(lane_id))]
    if len(matching)!=1:raise ValueError('missing or ambiguous source chain; select an explicit path')
    selected=matching[0]
    ids=[];chunks=[];speed=[]
    for key in selected:
        lane=occ[key]
        ud=next((e for e in lane.findall('userData') if e.get('code')=='mapforge.source_lane'),None)
        if ud is None:raise ValueError('path has no explicit original lane identity')
        sid=ud.get('value')
        rec=src.lane(sid)
        if rec.max_speed_kmh<=0:raise ValueError('explicit positive source speed required')
        speed_items=lane.findall('speed')
        if not speed_items:raise ValueError('written lane speed missing')
        for e in speed_items:
            if e.get('unit','m/s') not in ('m/s','km/h'):raise ValueError('unsupported speed unit')
            v=float(e.get('max'))/(3.6 if e.get('unit')=='km/h' else 1.)
            if abs(v*3.6-rec.max_speed_kmh)>1e-6:raise ValueError('written speed differs from original SHP')
            speed.append(v)
        if ids and sid==ids[-1]:continue
        g=project(rec.geometry)
        meta=next(e for e in lane.findall('userData') if e.get('code')=='mapforge.provenance/v1')
        direction=json.loads(meta.get('value')).get('travel_direction')
        if direction not in ('with_s','against_s'):raise ValueError('unknown source direction')
        if direction=='against_s':g=g[::-1]
        if chunks:
            outgoing=src.topo_out if direction=='with_s' else src.topo_in
            if sid not in outgoing.get(ids[-1],[]):raise ValueError('original SHP topology does not link these features')
            if np.linalg.norm(chunks[-1][-1]-g[0])>1e-9:raise ValueError('raw lane paths are disconnected; no fabricated bridge')
            g=g[1:] # One shared endpoint, not deleting a unique observation.
        ids.append(sid);chunks.append(g)
    if not speed or min(speed)<=0:raise ValueError('explicit positive speed required')
    return np.concatenate(chunks),ids,max(speed)


def run(path,output,road_id='10',side='right',lane=-1,speed_frontier=False,protect_straights=False):
    if output.exists():raise FileExistsError('use a new evidence directory')
    digest=_sha256(path)
    families={'IBD_LANE_LINK','IBD_LANE_LINK_MERGE','IBD_LANE_BOUNDARY',
              'IBD_LANE_BOUNDARY_REL','IBD_LANE_TOPO_DETAIL'}
    files=sorted(p for p in (ROOT/'shp_0222-0326').rglob('*') if p.is_file() and p.stem in families)
    files.append(ROOT/'profiles/shp/ibd-smarteditor-v1.yaml')
    hashes={str(p.relative_to(ROOT)):_sha256(p) for p in files}
    root=etree.parse(str(path)).getroot();road=next(r for r in root.findall('road') if r.get('id')==road_id)
    src=shp_source();lat,lon=_origin(root)
    raw,ids,speed=source_path(road,src,side,lane,lambda g:_project(g,lat,lon))
    limits=Limits(speed);output.mkdir(parents=True)
    record={'status':'BLOCKED','input':str(path.resolve()),'input_sha256':digest,
        'source_files_sha256':hashes,
        'projection':{'method':'existing local eqc chart; not absolute CRS verification','lat_0':lat,'lon_0':lon},
        'road':road_id,'side':side,'lane':lane,'source_lane_ids':ids,'raw_xy':raw.tolist(),
        'raw_vertex_count':len(raw),'limits':limits.__dict__,
        'raw_source_features':[{'lane_id':sid,'geometry_lonlat':np.asarray(src.lane(sid).geometry).tolist(),
            'max_speed_kmh':src.lane(sid).max_speed_kmh} for sid in ids],
        'scope':'source path representation experiment, not a lane ribbon or whole-map approval'}
    (output/'input.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    solver=fit_transition if protect_straights else fit
    before=time.monotonic();curves,result=solver(raw,limits,speed_frontier=speed_frontier)
    record.update(result=result,elapsed_seconds=time.monotonic()-before)
    record['source_and_input_unchanged']=(_sha256(path)==digest and
        all(_sha256(ROOT/p)==h for p,h in hashes.items()))
    (output/'report.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    g=road.find('planView/geometry');h=float(g.get('hdg'));origin=raw[0]
    rotation=np.array([[np.cos(h),-np.sin(h)],[np.sin(h),np.cos(h)]])
    chart=lambda p:(np.asarray(p)-origin)@rotation
    fig,axes=plt.subplots(2,1,figsize=(13,8),layout='constrained',dpi=150)
    srcxy=chart(raw);axes[0].plot(*srcxy.T,'o-',color='black',lw=2,ms=3,label='Full original linked LANE_LINK path')
    for trial in result['trials']:
        if 'parameters' not in trial:continue
        cs=[Clothoid.StandardParams(*p) for p in trial['parameters']]
        xy=chart(sample(cs,.1));color={1:'#999999',3:'#426dca',5:'#bb346d'}[len(cs)]
        axes[0].plot(*xy.T,color=color,label=f"{trial.get('model_kind',len(cs))}: {trial['status']} / source max {trial['source_to_curve_max_m']:.3f}m")
        st=0.
        for i,c in enumerate(cs):
            axes[1].plot([st,st+c.length],[c.KappaStart,c.KappaEnd],color=color,lw=2,label=f'{len(cs)} primitives' if i==0 else None)
            st+=c.length
    axes[0].set(ylabel='transverse displacement (m)',xlabel='longitudinal distance in source chart (m)')
    axes[1].set(ylabel='curvature (1/m)',xlabel='actual fitted path arc length (m)')
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=9)
    mode='supported-speed diagnostic' if speed_frontier else 'fixed-speed fit'
    fig.suptitle(f'road {road_id} / {side} {lane}: {mode}\nRequested speed unchanged {speed*3.6:.1f} km/h; BLOCKED for map export')
    fig.savefig(output/'paths.png');plt.close(fig)
    print(json.dumps({'status':record['status'],'path_status':result['status'],
        'elapsed_seconds':record['elapsed_seconds'], 'trials':[{k:v for k,v in t.items() if k not in ('parameters','analytic_g2_joins')} for t in result['trials']]},indent=2))
    if not record['source_and_input_unchanged']:raise RuntimeError('input changed during experiment; result invalid')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('xodr',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--road',default='10');p.add_argument('--side',choices=('left','right'),default='right');p.add_argument('--lane',type=int,default=-1)
    p.add_argument('--speed-frontier',action='store_true',help='diagnostic only; final validation still uses the original requested speed')
    p.add_argument('--protect-straights',action='store_true',help='test line-3spirals-line model with source-supported straight caps')
    a=p.parse_args();run(a.xodr,a.output,a.road,a.side,a.lane,a.speed_frontier,a.protect_straights)
    raise SystemExit(2)
