"""Independent source-relative reverse-turn diagnostic from actual XML.

For source-proven single turns only. Never flatten legitimate S-bends or
claim a general fairness certificate from a sign/finite-sampling test.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scripts.gen_all import _sha256,shp_source
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from spikes.measured_connector_caps import composite_sources
from scripts.review_measured_ribbon import target_curves
from scripts.internal_edge_jets import states
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def headings(points):
    delta=np.diff(np.asarray(points,float),axis=0)
    delta=delta[np.linalg.norm(delta,axis=1)>1e-8]
    if len(delta)<2:raise ValueError('complete nondegenerate path required')
    return np.unwrap(np.arctan2(delta[:,1],delta[:,0]))


def reverse_turn(source_heading, target_heading):
    sh=np.unwrap(source_heading);th=np.unwrap(target_heading)
    net=float(sh[-1]-sh[0]);sign=1 if net>=0 else -1
    opposite=lambda h:float(np.degrees(np.maximum(-sign*np.diff(h),0.).sum()))
    raw=opposite(sh);actual=opposite(th)
    proven=bool(abs(np.degrees(net))>=30 and raw<=.5)
    return dict(status=('EXTRA_REVERSE_TURN_DETECTED' if actual>raw+1. else 'NO_EXTRA_REVERSE_TURN_OBSERVED')
                if proven else 'REQUIRES_GENERAL_SHAPE_REVIEW',
        original_net_turn_deg=float(np.degrees(net)),source_reverse_turn_deg=raw,
        written_reverse_turn_deg=actual,extra_reverse_turn_deg=actual-raw,
        source_single_turn_observed=proven,diagnostic_added_budget_deg=1.,
        continuous_certificate=False,production_accepted=False)


def run(directory,output,allow_current_code=False):
    directory=Path(directory).resolve();output=Path(output).resolve()
    trial=json.loads((directory/'report.json').read_text(encoding='utf-8'))
    from scripts.research_code_revision import bind_current_code
    hashes,changes=bind_current_code(trial['source_hashes'])
    if changes and not allow_current_code:raise ValueError('explicit --allow-current-code needed for historical implementation changes')
    unchanged(hashes);path=Path(trial['artifact'])
    if _sha256(path)!=trial['sha256']:raise ValueError('input changed')
    root=ET.parse(path).getroot();src=shp_source();lat,lon=_origin(root)
    records=[];selected=[]
    for row in trial['connectors']:
        road=next(r for r in root.findall('road') if r.get('id')==row['road'])
        source,identity=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
        q=np.linspace(0,float(road.get('length')),max(2,int(float(road.get('length'))/.02)+1))
        jets=np.array([states(road,-1,float(s),False) for s in q])
        actual=target_curves(road,.02)
        fields={}
        for field,index in [('left',0),('right',1),('center',None)]:
            target_heading=jets[:,index,2] if index is not None else headings(actual[field])
            fields[field]=reverse_turn(headings(source[field]),target_heading)
        record=dict(road=row['road'],fields=fields,source_identity=identity)
        records.append(record)
        if any(r['status']=='EXTRA_REVERSE_TURN_DETECTED' for r in fields.values()):
            selected.append((road,source,actual,record))
    output.mkdir(exist_ok=False)
    report=dict(status='REJECTED_SHAPE' if selected else 'LIMITED_SHAPE_CHECK_NOT_ACCEPTANCE',
        artifact=str(path),sha256=_sha256(path),records=records,
        failed_roads=[r[0].get('id') for r in selected],method='actual XML analytic edge headings, 0.02m; raw segment headings',
        scope='single-turn observations only, excludes ordinary roads/surface and general S-bends',
        source_changed=False,production_accepted=False,input_code_revision_changes=changes,
        hashes={str(p):_sha256(p) for p in [Path(__file__),path,directory/'report.json']})
    dump(output/'report.json',report)
    if selected:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        cols=min(len(selected),4);rows=int(np.ceil(len(selected)/cols))
        fig,axes=plt.subplots(rows,cols,figsize=(6*cols,5*rows),squeeze=False,layout='constrained')
        for ax,(road,source,actual,record) in zip(axes.flat,selected):
            for field in source:
                ax.plot(*source[field].T,c='#e78c1a',lw=1.4)
                ax.plot(*actual[field].T,c='#176bbb',lw=1,ls='--' if field=='center' else '-')
            worst=max(v['extra_reverse_turn_deg'] for v in record['fields'].values())
            ax.set_title(f"{road.get('id')}: REJECTED | added reverse turn {worst:.2f} deg")
            ax.set_aspect('equal');ax.grid(alpha=.2)
        for ax in list(axes.flat)[len(selected):]:ax.set_visible(False)
        fig.suptitle('Orange=full original source; blue=written XODR. Fidelity/G2 PASS is NOT a fairness PASS.')
        fig.savefig(output/'reverse-turns.png',dpi=150);plt.close(fig)
    unchanged(hashes);unchanged(report['hashes'])
    print(report['status'],report['failed_roads'],flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output')
    p.add_argument('--allow-current-code',action='store_true');a=p.parse_args()
    run(a.directory,a.output,a.allow_current_code)
