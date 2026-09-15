"""Research transaction: one source-native north state, all incident turns.

Not a simultaneous whole-map optimizer. Parent port hypotheses are explicit;
all twelve turns are recomputed against the SAME immutable parent. Partial
or failed results are diagnostic only and never enter the default converter.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import numpy as np
from scripts.gen_all import _sha256,shp_source
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code
from mapforge.ops.port_dependencies import LanePort,PortDependencies
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from spikes.measured_connector_caps import frames,raw_curves,composite_sources,needs_cap,seed_chain
from spikes.trial_xml import replace_road,write_trial


def install_same_identity_parent(root, component):
    """Match actual endpoint source identities, never apply legacy rank remaps twice."""
    rid=component.get('id');old=next(r for r in root.findall('road') if r.get('id')==rid)
    for cp,index in [('start',0),('end',-1)]:
        def identities(road):
            sec=road.findall('lanes/laneSection')[index];out={}
            for lane in sec.findall('*/lane'):
                sid=lane.find("userData[@code='mapforge.source_lane']")
                if sid is not None:
                    key=sid.get('value')
                    if key in out:raise ValueError('duplicate source identity at parent port')
                    out[key]=int(lane.get('id'))
            return out
        a,b=identities(old),identities(component)
        if not a or a!=b:raise ValueError('source/lane port identities changed; explicit topology migration needed')
    new=copy.deepcopy(component)
    if new.find('link') is not None:raise ValueError('isolated component required')
    if old.find('link') is not None:new.insert(0,copy.deepcopy(old.find('link')))
    replace_road(root,old,new)


def fresh_seed(a,b,previous,old_frames,contact_caps=None):
    """Keep the historical long core when valid; add only required >=6m caps."""
    old_caps=tuple(needs_cap(f) for f in old_frames)
    from mapforge.ops.joint_connector_fit import resolve_contact_caps
    caps=resolve_contact_caps(a,b,contact_caps);count=previous.get('reference_core_count',3)
    q=np.asarray(previous['shape_parameters'],float);old_nr=count+sum(old_caps)
    if len(q)!=old_nr+count-1:raise ValueError('saved primitive count disagrees with actual parent ports')
    if count+sum(caps)<=5:
        old_lengths=q[:old_nr];core=old_lengths[int(old_caps[0]):int(old_caps[0])+count]
        lengths=([old_lengths[0] if old_caps[0] else 6.] if caps[0] else [])+list(core)+([
            old_lengths[-1] if old_caps[1] else 6.] if caps[1] else [])
        seed=np.r_[np.maximum(lengths,6.),q[old_nr:]]
        return seed,count,dict(method='retained-long-core-new-parent-end-caps',old_caps=old_caps,new_caps=caps)
    cls=seed_chain(a,b,6. if caps[0] else 0.,6. if caps[1] else 0.)
    i=int(caps[0]);q=np.r_[np.maximum([c.length for c in cls],6.),cls[i].KappaEnd*20,cls[i+1].KappaEnd*20]
    return q,3,dict(method='three-core-endpoint-seed-five-reference-budget',old_caps=old_caps,new_caps=caps)


def solve_one(job):
    from mapforge.ops.joint_connector_fit import fit_joint
    road=ET.fromstring(job['xml']);rid=road.get('id')
    try:
        candidate,row=fit_joint(road,*job['frames'],job['raw'],job['via'],job['seed'],
            core_count=job['core_count'],fair_world=True,max_iterations=job['max_iterations'],source_tails=job['tails'],
            progress=lambda state:print('PROGRESS',rid,json.dumps(state),flush=True))
        row.update(seed_migration=job['seed_migration'],source_support=job['source_support'])
        return ET.tostring(candidate),row
    except (ValueError,ArithmeticError) as exc:
        return None,dict(road=rid,status='REJECTED',reason=str(exc),geometry_rebuilt=False,
                         source_support=job['source_support'])


def run(component_directory,baseline_directory,output,workers=3,max_iterations=60,prepare_only=False):
    component_directory=Path(component_directory).resolve();baseline_directory=Path(baseline_directory).resolve()
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    cr=json.loads((component_directory/'report.json').read_text(encoding='utf8'))
    previous=json.loads((baseline_directory/'report.json').read_text(encoding='utf8'))
    component_path=Path(cr['artifact']);baseline=Path(previous['artifact'])
    if cr['road']!='10' or _sha256(component_path)!=cr['sha256'] or _sha256(baseline)!=previous['sha256']:
        raise ValueError('exact north component and baseline revision required')
    hashes,changes=bind_current_code({**previous['source_hashes'],**cr['source_hashes']})
    hashes.update({str(p):_sha256(p) for name in ('mapforge','scripts','spikes') for p in (ROOT/name).rglob('*.py')})
    hashes.update({str(p):_sha256(p) for p in (baseline,component_path,component_directory/'report.json',
        component_directory/'shared-state.json',baseline_directory/'report.json')})
    unchanged(hashes);tree=ET.parse(baseline);root=tree.getroot();old=copy.deepcopy(root)
    graph=PortDependencies(root);graph.validate_junction_table()
    parent=graph.roads['10'];mutable={LanePort('10',cp,int(l.get('id'))) for cp,i in [('start',0),('end',-1)]
        for l in parent.findall('lanes/laneSection')[i].findall('*/lane') if l.get('id')!='0'}
    affected=graph.closure(mutable)['connectors']
    if len(affected)!=12:raise ValueError('north component requires exactly twelve explicit incident turns')
    install_same_identity_parent(root,ET.parse(component_path).getroot().find('road'))
    candidate_graph=PortDependencies(root)
    if graph.connections!=candidate_graph.connections:raise ValueError('dependency graph changed')
    candidate_graph.validate_junction_table()
    source=shp_source();lat,lon=_origin(root);project=lambda p:_project(p,lat,lon)
    rows=[];jobs=[];ports=[]
    for rid in sorted(affected,key=int):
        road=next(r for r in root.findall('road') if r.get('id')==rid)
        oldroad=next(r for r in old.findall('road') if r.get('id')==rid)
        raw,support=composite_sources(root,road,source,project)
        via=raw_curves(source,road.find(".//userData[@code='mapforge.source_lane']").get('value'),project)
        oldframes=frames(old,oldroad);newframes=frames(root,road)
        saved=next(r for r in previous['connectors'] if r['road']==rid)
        q,count,migration=fresh_seed(*newframes,saved,oldframes)
        tails=[dict(role=role,source_lane_id=sid,curves=raw_curves(source,sid,project)) for role,sid in (
            ('predecessor',support['source_lane_ids'][0]),('successor',support['source_lane_ids'][-1]))]
        jobs.append(dict(xml=ET.tostring(road),frames=newframes,raw=raw,via=via,source_support=support,
            seed=q,core_count=count,tails=tails,max_iterations=max_iterations,seed_migration=migration))
        ports.append(dict(road=rid,source_identity=support,old_frames=oldframes,new_frames=newframes,seed_migration=migration))
    # This file deliberately says INPUT: its incident turns are NOT connected yet.
    parent_input=output/'unconnected-parent-input.xodr';write_trial(tree,parent_input)
    preparation=dict(status='PARENT_AND_SOURCE_PREPARED_NOT_CONNECTED',affected_roads=sorted(affected,key=int),
        artifact=str(parent_input),sha256=_sha256(parent_input),source_hashes=hashes,ports=ports,
        parent_component=str(component_directory),input=str(baseline),input_sha256=previous['sha256'],
        source_changed=False,production_accepted=False,input_code_revision_changes=changes)
    dump(output/'preparation.json',preparation)
    if prepare_only:unchanged(hashes);return preparation
    print('SAME PARENT / ALL TWELVE',flush=True)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        pending={pool.submit(solve_one,job):ET.fromstring(job['xml']).get('id') for job in jobs}
        for f in as_completed(pending):
            rid=pending[f];xml,row=f.result();rows.append(row)
            if xml is not None:replace_road(root,next(r for r in root.findall('road') if r.get('id')==rid),ET.fromstring(xml))
            dump(output/'connectors-progress.json',sorted(rows,key=lambda r:int(r['road'])))
            print('DEPENDENT',rid,row['status'],row.get('reason',''),flush=True)
    graph.require_complete(mutable,[r['road'] for r in rows])
    for road in old.findall('road'):
        rid=road.get('id')
        if rid in affected or rid=='10':continue
        if ET.tostring(road)!=ET.tostring(next(r for r in root.findall('road') if r.get('id')==rid)):
            raise ValueError('unaffected road changed')
    PortDependencies(root).validate_junction_table()
    path=output/'node4-diagnostic.xodr';write_trial(tree,path)
    updated={r['road']:r for r in rows}
    report=dict(status='REJECTED_PENDING_INDEPENDENT_REVIEW',artifact=str(path),sha256=_sha256(path),
        input=str(baseline),input_sha256=previous['sha256'],source_hashes=hashes,
        component=previous['component'],north_component=str(component_directory),
        changed_parent='10',affected_roads=sorted(affected,key=int),parent_port_dependency_closed=True,
        simultaneous_parent_connector_optimization=False,
        connectors=[updated.get(r['road'],r) for r in previous['connectors']],
        geometry_candidate_roads=[r['road'] for r in rows if r['status']=='GEOMETRY_REVIEW_CANDIDATE'],
        rejected_roads=[r['road'] for r in rows if r['status']!='GEOMETRY_REVIEW_CANDIDATE'],
        source_changed=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',report)
    print('REJECTED DIAGNOSTIC',report['rejected_roads'],flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('component_directory');p.add_argument('baseline_directory');p.add_argument('output')
    p.add_argument('--workers',type=int,default=3);p.add_argument('--max-iterations',type=int,default=60)
    p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
    if not 1<=a.workers<=4 or not 1<=a.max_iterations<=100:raise ValueError('bounded research runtime required')
    run(a.component_directory,a.baseline_directory,a.output,a.workers,a.max_iterations,a.prepare_only)
    raise SystemExit(2)
