"""Reproducible, read-only source/port preflight, not a conversion command.

The report explains why a parent block must not be committed independently.
It deliberately writes no XODR, including when a diagnostic relaxation fits.
"""
import argparse
import hashlib
import json
import sys
from dataclasses import asdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source, _sha256
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from mapforge.validate.smoothness import lane_edges_kinematics_at
from spikes.mouth_anchor_review import project_curve
from spikes.road_boundary_family import solve_road
from spikes.shared_boundary_window import SharedBoundaryWindow


def run(path, output, road_id='10', boundary=3, span=30.):
    path=path.resolve(); output=output.resolve()
    if output.exists():
        raise FileExistsError('use a new report directory; previous evidence is immutable')
    root=etree.parse(str(path)).getroot(); digest=_sha256(path)
    src=shp_source(); lat,lon=_origin(root); project=lambda g:_project(g,lat,lon)
    graph=PortDependencies(root); road=graph.roads[road_id]
    ports=graph.boundary_ports(road_id,'end','right',boundary)
    closure=graph.closure(ports)
    model=solve_road(road,src,project,min_span=15.,source_tol=.35,
        source_error_budget='absolute',boundary_association='source-order',physical_graph=True,
        junction_endpoint_mode='free',all_source_vertices=True,model_only=True)
    if isinstance(model,dict): raise ValueError(model)
    _,phase=model.preflight()
    window=SharedBoundaryWindow(road,'right',boundary,span=span,min_span=15.)
    window.gather_source(src,project); _,local=window.source_preflight(.35)
    # Read full original features, including raw vertices outside the current
    # road block. The ordinary block may not account for connector support.
    sids=sorted({u.get('value') for u in road.findall('.//lane/userData') if u.get('code')=='mapforge.source_lane'})
    raw=[]; plot_raw={}
    for sid in sids:
        rec=src.lane(sid); bounds=src.lane_boundary_geometries(sid)
        raw.append({'lane_id':sid,'center_lonlat':np.asarray(rec.geometry).tolist(),
            'boundary_lonlat':[g.tolist() for g in bounds],
            's_width_mm':rec.s_width_mm,'e_width_mm':rec.e_width_mm,
            's_width_known':rec.s_width_known,'e_width_known':rec.e_width_known})
        plot_raw[sid]=(project_curve(rec.geometry,road,project),[project_curve(g,road,project) for g in bounds])
    source_files={}
    for p in sorted((ROOT/'shp_0222-0326').rglob('*')):
        if p.is_file() and p.stem in {'IBD_LANE_LINK','IBD_LANE_LINK_MERGE','IBD_LANE_BOUNDARY',
                                     'IBD_LANE_BOUNDARY_REL','IBD_LANE_TOPO_DETAIL'}:
            source_files[str(p.relative_to(ROOT))]=_sha256(p)
    source_files['profiles/shp/ibd-smarteditor-v1.yaml']=_sha256(ROOT/'profiles/shp/ibd-smarteditor-v1.yaml')
    record={'status':'BLOCKED','kind':'preflight-only-no-xodr-export',
        'input':str(path),'input_sha256':digest,'source_files_sha256':source_files,
        'source_feature_snapshot_sha256':hashlib.sha256(json.dumps(raw,sort_keys=True).encode()).hexdigest(),
        'projection':{'method':'existing local eqc chart; not an absolute CRS verification','lat_0':lat,'lon_0':lon},
        'requested_boundary':{'road':road_id,'contact':'end','side':'right','index':boundary},
        'dependency_closure':{'mutable_ports':[asdict(p) for p in sorted(closure['mutable_ports'])],
            'fixed_ports':[asdict(p) for p in sorted(closure['fixed_ports'])],
            'connectors':sorted(closure['connectors'])},
        'fixed_window':dict(local,lo=window.lo,span=span,variables=window.nvar),
        'whole_road_free_ports':phase,'branch_tie_diagnostics':model.diagnose_branch_ties(),
        'isolated_branch_diagnostics':{f:model.diagnose_branch_ties(f) for f in sorted({
            l['family'] for l in model.equality_labels if l['kind']=='branch-tie'})},
        'physical_events':model.events,
        'family_layout':[{'name':f.key,'chain':f.chain,'breaks':np.unique(f.knots).tolist(),
                          'variables':f.columns.stop-f.columns.start} for f in model.families],
        'source_features':raw,
        'limits':{'source_tolerance_m':.35,'minimum_independent_span_m':15.,
                  'speed_policy':'unchanged; no dynamics approval from this linear preflight'},
        'scope':'chosen straight-frame cubic family, source tubes, width bounds and branch ties only; not global impossibility',
        'untested':['joint connector optimization','world dynamics','whole map','MAP conversion','absolute CRS']}
    fig,axes=plt.subplots(4,1,figsize=(14,13),layout='constrained',dpi=150)
    for ax,side in zip(axes[:2],('right','left')):
        for fi,color in enumerate(('#bb5910','#41826c','#6c70b5','#ba3483')):
            name=f'{side}-{fi}';rows=[r for r in model.source_rows if r[-1]==name]
            if not rows:continue
            for sid in sorted({r[3] for r in rows}):
                part=sorted((r[4],r[1]) for r in rows if r[3]==sid)
                xy=np.array(part);ax.plot(*xy.T,lw=2,color=color)
            ax.plot([],[],lw=2,color=color,label=name+' source edge')
        for si,sec in enumerate(road.findall('lanes/laneSection')):
            a=float(sec.get('s'));b=model.ends[si]
            ss=np.linspace(a+1e-5,b-1e-5,max(3,int((b-a)/.1)))
            edges=np.asarray([lane_edges_kinematics_at(road,s,side) for s in ss])[:,:,0]
            for i in range(edges.shape[1]):ax.plot(ss,edges[:,i],'--',color='#333333',lw=.9,alpha=.65)
        ax.plot([],[],'--',color='#333333',label='Old XODR: not accepted')
        ax.set(ylabel='transverse t (m)',title=f'Original shared edges versus existing XODR ({side} side)')
        ax.legend(ncol=3,fontsize=9)
        for event in model.events:
            if event['side']==side:ax.axvline(event['station_m'],color='#d09b16',ls=':')
    axes[0].axvline(window.lo,color='crimson',ls=':')
    axes[2].axvline(window.lo,color='crimson',ls=':',label='fixed cut')
    for fi,f in enumerate(model.families):
        if fi==0:continue
        breaks=np.unique(f.knots)
        axes[2].plot(breaks,np.full(len(breaks),fi),'o-',ms=5,label=f.key)
        axes[2].text(breaks[-1]+1,fi,f.key,va='center')
    for event in model.events:
        axes[2].axvline(event['station_m'],color='#d09b16',ls=':')
    axes[2].set(ylabel='shared boundary family',title='Independent cubic spans (>=15m); no sample-by-sample knots')
    axes[2].set_xlim(-1,float(road.get('length'))+17)
    branch = next((e for e in model.events if e['side']=='right' and e['kind']=='birth'),None)
    if branch is not None:
        si=next(i for i,v in enumerate(model.starts) if abs(v-branch['station_m'])<1e-6)
        sec=road.findall('lanes/laneSection')[si]
        lids=[branch['new_separator'],branch['collapsed_onto']]
        for j,lid in enumerate(lids):
            lane=next(e for e in sec.findall('right/lane') if e.get('id')==lid)
            sid=next(e.get('value') for e in lane.findall('userData') if e.get('code')=='mapforge.source_lane')
            center,bounds=plot_raw[sid]
            for g in bounds:axes[3].plot(*g.T,'o-',color=['#ba3483','#bb5910'][j],ms=4,lw=2)
            axes[3].plot(*center.T,'--',color=['#ba3483','#bb5910'][j],label='Raw path '+sid[-6:])
        axes[3].axvline(branch['station_m'],ls=':',color='#d09b16',label='Written structural birth')
        axes[3].set_xlim(branch['station_m']-2,branch['station_m']+19)
    axes[3].set(ylabel='transverse t (m)',xlabel='station along existing straight frame (m)',
                title='Raw taper: solid=boundaries; dashed=LANE_LINK (not automatically boundary midpoint)')
    axes[3].legend(fontsize=9)
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('node4 shared-port preflight: BLOCKED\nC0/C1 counterfactuals are diagnostic only; no relaxed XODR exported',fontsize=13)
    output.mkdir(parents=True)
    fig.savefig(output/'preflight.png'); plt.close(fig)
    if _sha256(path)!=digest or any(_sha256(ROOT/p)!=h for p,h in source_files.items()):
        raise RuntimeError('input changed during read-only preflight; discard report')
    (output/'report.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:record[k] for k in ('status','input_sha256','dependency_closure','fixed_window','whole_road_free_ports','branch_tie_diagnostics')},indent=2))
    return False  # No locally feasible block alone constitutes joint success.


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('xodr',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--road',default='10');parser.add_argument('--boundary',type=int,default=3)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.xodr,args.output,args.road,args.boundary) else 2)
