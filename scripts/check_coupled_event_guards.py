"""Frozen real-data seven-road guard replay; never optimize or write a map.

The unchanged r6 XML and one algebraic interface vector are separate records.
All visible and driving surface statistics are diagnostics, not map approval.
"""
import importlib.metadata
import json
from pathlib import Path
import sys

import numpy as np
from shapely.geometry import mapping
from shapely.geometry.base import BaseGeometry

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.prepare_coupled_event_contract import load_contract
from scripts.check_outer_event_written import boundary_poly
from mapforge.repair_web.model import atomic, digest, json_bytes
from mapforge.repair_web.coupled_event_guards import CoupledEventGuards, reference_turn
from mapforge.repair_web.coupled_event_sources import sample_ribbon
from mapforge.repair_web.coupled_event_surface import EventSurface
from mapforge.repair_web.coupled_event_layout import check_parent_records

PRIOR=ROOT/'out/node4-coupled-event-contract-20260917'
DEST=ROOT/'out/node4-coupled-event-guards-20260917'
WEB=[ROOT/'out'/p/'project/project.json' for p in ('node4-local-repair-20260915','node4-repair-ui-v2-20260915')]


def verify(bindings):
    for path,sha in bindings.items():
        if digest(Path(path).read_bytes())!=sha: raise ValueError('Frozen input drift: '+path)


def serial(value):
    if isinstance(value,BaseGeometry): return mapping(value)
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,dict): return {k:serial(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [serial(v) for v in value]
    return value


def draw_polygon(ax,shape,color,*,alpha=1.,edge=None):
    if shape.is_empty: return
    if shape.geom_type=='Polygon':
        xy=np.array(shape.exterior.coords); ax.fill(*xy.T,color=color,alpha=alpha,edgecolor=edge)
        for ring in shape.interiors:
            xy=np.array(ring.coords); ax.fill(*xy.T,color='white')
    elif hasattr(shape,'geoms'):
        for part in shape.geoms: draw_polygon(ax,part,color,alpha=alpha,edge=edge)


def plot(engine,surface,original,reference):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
    bounds=original.bounds; limits=(bounds[0]-4,bounds[2]+4,bounds[1]-4,bounds[3]+4)
    for ax,key,title in zip(axes[0],('visible','driving'),('Visible physical bands incl. auxiliary paving','Driving lanes only (not route coverage verdict)')):
        r=surface[key]; draw_polygon(ax,r['union_geometry'],'#a0a5aa')
        draw_polygon(ax,r['source_missing_geometry'],'#e64b35')
        xy=np.array(original.exterior.coords); ax.plot(*xy.T,color='#18763d',lw=1.2,label='Original SHP junction footprint')
        ax.set(xlim=limits[:2],ylim=limits[2:],aspect='equal',xlabel='Local x (m)',ylabel='Local y (m)')
        ax.set_title(title+'\nUncovered source footprint: %.6f m2'%r['source_junction_missing_m2'],fontsize=10)
        ax.legend(fontsize=8,loc='upper left'); ax.grid(alpha=.15)
    ax=axes[1,0]; p=engine.parent; edge=3
    for t in p.traces:
        if t['edge']==edge:
            xy=t['st']; ax.plot(*xy.T,color='#e8841d',lw=1.5)
    ss=np.linspace(p.births[edge],p.end,1000)
    tt=[boundary_poly(p.road,edge,float(s))[0] for s in ss]
    ax.plot(ss,tt,color='#1759ab',lw=1.4,label='Unchanged r6 boundary 3')
    ax.plot([],[],color='#e8841d',label='Original same-identity SHP boundary')
    ax.axvline(178,color='#333',lw=.7,ls='--')
    target=reference['parent']['target_s178']
    ax.scatter([178,178],[target['original_t'],target['actual_t']],c=['#e8841d','#1759ab'],s=18)
    visible_t=[t['st'][(t['st'][:,0]>=145)&(t['st'][:,0]<=p.end+3),1] for t in p.traces if t['edge']==edge]
    values=np.r_[np.concatenate(visible_t),np.array(tt)[ss>=145]]
    ax.set(xlim=(145,p.end+3),ylim=(float(min(values))-.2,float(max(values))+.2),
           xlabel='Parent road station s (m)',ylabel='Lateral coordinate t (m), expanded scale')
    ax.set_title('Parent road 11: unchanged geometry, not a repair result\ns178 offset error = %.3f cm'%(100*abs(target['error_m'])),fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=.2)
    ax=axes[1,1]; turn=reference_turn(engine.contract.graph.roads['111']); mesh=sample_ribbon(turn)
    old=mesh['points']['right']; row=next(r for r in engine.sources.by_turn['111'] if r['field']=='right' and r['role']=='predecessor')
    raw=row['oriented_full_xy']; ax.plot(*raw.T,color='#e8841d',lw=1.6,label='Complete original incoming boundary')
    ax.plot(*old.T,color='#1759ab',lw=1.4,label='Actual XML turn 111 right edge')
    tip=raw[-1]; ax.scatter(*tip,c='#e8841d',s=22,label='Original via join')
    near=old[mesh['stations']<=12.]; xy=np.vstack([near,tip]); lo=xy.min(axis=0); hi=xy.max(axis=0)
    ax.set(xlim=(lo[0]-1,hi[0]+1),ylim=(lo[1]-1,hi[1]+1),aspect='equal',xlabel='Local x (m)',ylabel='Local y (m)')
    fail=next(r for r in reference['turns']['111']['ordinary_tail_rows'] if r['field']=='right' and r['role']=='predecessor')
    ax.set_title('Turn 111: ordinary tail must be checked separately\nSampled tail deviation %.3f m; existing limit 0.350 m'%fail['statistics']['max_m'],fontsize=10)
    ax.legend(fontsize=8,loc='best'); ax.grid(alpha=.2)
    fig.suptitle('node4 unchanged r6 XML vs original SHP | diagnostic only, NOT accepted',fontsize=14)
    fig.savefig(DEST/'reference-source-surface.png',dpi=140); plt.close(fig)


def main():
    if DEST.exists(): raise ValueError('Do not overwrite frozen guard evidence')
    bindings=json.loads((PRIOR/'binding.json').read_bytes()); verify(bindings)
    web_bindings={str(p):json.loads(p.read_bytes())['binding'] for p in WEB}
    for b in web_bindings.values(): verify(b)
    extra=[*PRIOR.glob('*.json'),*WEB,Path(__file__).resolve(),ROOT/'tests/test_coupled_event_guards.py']
    for module in list(sys.modules.values()):
        name=getattr(module,'__name__',''); path=getattr(module,'__file__',None)
        if path and name.split('.')[0] in ('mapforge','scripts','spikes'): extra.append(Path(path).resolve())
    for path in extra:
        sha=digest(path.read_bytes())
        if str(path) in bindings and bindings[str(path)]!=sha: raise ValueError('Prior binding drift: '+str(path))
        bindings[str(path)]=sha
    verify(bindings)
    contract=load_contract(); engine=CoupledEventGuards(contract)
    reference=engine.evaluate(reference=True)
    diagnostic=engine.evaluate(engine.model.diagnostic)
    snapshot=engine.model.evaluate(engine.model.diagnostic)
    coarse_engine=EventSurface(contract,step=.1); coarse=coarse_engine.evaluate()
    fine_engine=EventSurface(contract,step=.05); fine=fine_engine.evaluate()
    diagnostic_surface=coarse_engine.evaluate(snapshot)
    layout=check_parent_records(contract,snapshot['parent_coefficients'])
    # A resolution comparison is NOT convergence proof or a new tolerance.
    differences={k:abs(fine[k]['source_junction_missing_m2']-coarse[k]['source_junction_missing_m2']) for k in ('visible','driving')}
    report=dict(status='E2_GUARDS_CONNECTED_NOT_TRIAL_ADMITTED',source_inventory=engine.sources.inventory(),
        unchanged_xml_sha256=digest(contract.reference),new_xodr=False,optimizer_calls=0,map_accepted=False,
        short_independent_segments_added=0,web_changed=False,source_speed_or_role_changes=0,
        completed_components=['same-state original source and interval shape checks',
            'separate ordinary-tail guard replay', 'full unchanged-network visible/driving surface diagnostics',
            'in-memory width/offset record stack readback'],
        pending_before_trial=['continuous source/target ownership and error enclosures, not nearest-tip crop pass',
            'full-event shape intent outside old guard domain and surface acceptance policy',
            'complete seven-road actual writer/independent readback/atomic transaction admission',
            'one declared complete solver and non-resettable total budget'],
        original_failure_preservation=dict(parent_source_max_m=reference['parent']['source_max_m'],
            s178_error_m=reference['parent']['target_s178']['error_m'],
            ordinary_tail_failures=[dict(connector=cid,**r) for cid,t in reference['turns'].items() for r in t['ordinary_tail_rows'] if r['sampled_violation']]),
        surface_resolution=dict(coarse_step_m=.1,fine_step_m=.05,absolute_missing_area_delta_m2=differences,
            formal_or_route_certificate=False,driving_footprint_difference_is_not_automatically_missing_road=True),
        packages={k:importlib.metadata.version(k) for k in ('numpy','scipy','shapely','pyclothoids')},
        input_bindings=len(bindings),input_drift=0,web_binding_counts={p:len(b) for p,b in web_bindings.items()})
    verify(bindings)
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json',json_bytes(bindings))
    for name,value in [('reference-guards',reference),('diagnostic-guards',diagnostic),('reference-surface-010',coarse),
                       ('reference-surface-005',fine),('diagnostic-surface',diagnostic_surface),('record-readback',layout)]:
        atomic(DEST/(name+'.json'),json_bytes(serial(value)))
    plot(engine,fine,fine_engine.original,reference)
    for b in web_bindings.values(): verify(b)
    verify(bindings)
    report['artifact_sha256']={p.name:digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()}
    atomic(DEST/'status.json',json_bytes(report))
    print(json.dumps(dict(status=report['status'],input_bindings=len(bindings),
        old_tail_failures=len(report['original_failure_preservation']['ordinary_tail_failures']),
        coarse_visible_missing_m2=coarse['visible']['source_junction_missing_m2'],
        fine_visible_missing_m2=fine['visible']['source_junction_missing_m2'],
        prototype_invalid_bands=len(diagnostic_surface['invalid_bands']),
        record_roundtrip_error_m=layout['maximum_stack_readback_error_m'])))


if __name__=='__main__': main()
