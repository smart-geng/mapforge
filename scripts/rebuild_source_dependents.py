"""One fixed north/east parent state, ALL dependent measured turns, review only.

Independent workers solve against the same immutable parent XML. Raw source
tails travel with original TOPO; no turn from a different parent is promoted.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor,as_completed

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
# Each independent solve uses one BLAS thread, avoiding nested oversubscription.
for variable in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[variable]='1'
from scripts.gen_all import shp_source,_sha256
from scripts.build_source_geometry_candidate import dump,restore_source_speeds
from scripts.fit_source_boundary_block import unchanged
from spikes.measured_connector_caps import composite_sources,raw_curves,frames,fit
from spikes.trial_xml import replace_road,write_trial
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def incident_roads(root,parents):
    return sorted((r for r in root.findall('road') if r.get('junction')!='-1' and any(
        e.get('elementType')=='road' and e.get('elementId') in parents for e in r.findall('link/*'))),
        key=lambda r:int(r.get('id')))


def install_component(root,component,ports):
    rid=component.get('id');old=root.find(f"road[@id='{rid}']")
    if old is None:raise ValueError('parent road identity missing')
    new=copy.deepcopy(component)
    if new.find('link') is not None:raise ValueError('expected isolated component')
    new.insert(0,copy.deepcopy(old.find('link')))
    replace_road(root,old,new)
    remap={(r['contact'],int(r['old_lane'])):int(r['new_lane']) for r in ports}
    for road in incident_roads(root,{rid}):
        for role in ('predecessor','successor'):
            link=road.find('link/'+role)
            if link.get('elementId')!=rid:continue
            ll=road.find('lanes/laneSection/right/lane/link/'+role)
            ll.set('id',str(remap[link.get('contactPoint'),int(ll.get('id'))]))
    for connection in root.findall('junction/connection'):
        if connection.get('incomingRoad')!=rid:continue
        road=root.find("road[@id='"+connection.get('connectingRoad')+"']")
        cp=road.find('link/predecessor').get('contactPoint')
        for ll in connection.findall('laneLink'):ll.set('from',str(remap[cp,int(ll.get('from'))]))


def solve_one(job):
    road=ET.fromstring(job['xml'])
    try:
        new,row=fit(road,*job['frames'],job['raw'],adaptive_caps=True,source_warm_start=True,
            raw_via=job['via'],require_dynamics=False,source_cross_section=True,world_g2_cross_section=True)
        return ET.tostring(new),row
    except (ValueError,ArithmeticError) as exc:
        return None,dict(road=road.get('id'),status='REJECTED',reason=str(exc),geometry_rebuilt=False)


def run(component_directory,baseline,output,workers=4):
    component_directory=Path(component_directory).resolve();baseline=Path(baseline).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    component_report=json.loads((component_directory/'report.json').read_text(encoding='utf8'))
    path=Path(component_report['artifact'])
    if _sha256(path)!=component_report['sha256']:raise ValueError('component artifact changed')
    # Historical selected parent is fixed by its exact file, NOT by old code PASS.
    if _sha256(baseline)!='65796005fd40644c296282bd902ba46753a2370219da35fedbdc8f2cc3e370d8':
        raise ValueError('explicit v1.65 retained north state required for this scoped driver')
    original_hashes=component_report['source_hashes'];code_changes=[]
    for name,expected in original_hashes.items():
        current=_sha256(Path(name))
        if current==expected:continue
        if Path(name).suffix!='.py':raise ValueError('immutable source/decision/configuration changed')
        code_changes.append(dict(file=name,previous_sha256=expected,current_sha256=current))
    hashes={name:_sha256(Path(name)) for name in original_hashes}
    hashes.update({str(p):_sha256(p) for d in ('mapforge','scripts','spikes') for p in (ROOT/d).rglob('*.py')})
    hashes.update({str(p):_sha256(p) for p in (baseline,path,component_directory/'report.json',component_directory/'shared-state.json')})
    tree=ET.parse(baseline);root=tree.getroot()
    install_component(root,ET.parse(path).getroot().find('road'),component_report['ports'])
    affected=incident_roads(root,{'10','13'})
    if len(affected)!=20:raise ValueError('node4 north/east dependency union is not the expected 20 turns')
    untouched={r.get('id'):ET.tostring(r) for r in root.findall('road') if r not in affected}
    parent_path=output/'fixed-parent-input.xodr';write_trial(tree,parent_path)
    src=shp_source();lat,lon=_origin(root);jobs=[];rows=[]
    for road in affected:
        rid=road.get('id')
        try:
            speed=restore_source_speeds([road],src)
            sid=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']").get('value')
            via=raw_curves(src,sid,lambda p:_project(p,lat,lon))
            raw,identity=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
            jobs.append(dict(road=rid,xml=ET.tostring(road),raw=raw,via=via,frames=frames(root,road),
                source_support=identity,source_speed_restoration=speed))
        except (ValueError,AttributeError,KeyError) as exc:
            rows.append(dict(road=rid,status='REJECTED',reason=str(exc),geometry_rebuilt=False))
    dump(output/'preparation.json',dict(affected_roads=[r.get('id') for r in affected],
        prepared_roads=[j['road'] for j in jobs],rejections=rows,source_hashes=hashes,
        component_code_changes=code_changes,retained_north_is_historical_not_fresh_source_certificate=True,
        parents_sha256=_sha256(parent_path),source_tails=component_report['compiled']['source_domain'],
        production_accepted=False))
    print('FIXED PARENTS / PREPARED',len(jobs),'REJECTED',rows,flush=True)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures={pool.submit(solve_one,job):job for job in jobs}
        for future in as_completed(futures):
            job=futures[future];xml,row=future.result()
            if xml is not None:
                replace_road(root,root.find("road[@id='"+job['road']+"']"),ET.fromstring(xml))
            row.update(source_support=job['source_support'],source_speed_restoration=job['source_speed_restoration'])
            rows.append(row);dump(output/'connectors-progress.json',rows)
            print('DEPENDENT',job['road'],row['status'],row.get('reason',''),flush=True)
    for rid,data in untouched.items():
        if ET.tostring(root.find(f"road[@id='{rid}']"))!=data:raise ValueError('fixed parent/unaffected road changed')
    path=output/'node4-review.xodr';write_trial(tree,path)
    unchanged(hashes)
    report=dict(status='REJECTED_REVIEW_NOT_DELIVERY',input=str(baseline),input_sha256=_sha256(baseline),
        artifact=str(path),sha256=_sha256(path),component=str(component_directory),source_hashes=hashes,
        connectors=sorted(rows,key=lambda r:int(r['road'])),fixed_parents=['10','13'],
        rebuilt_roads=[r['road'] for r in rows if r.get('geometry_rebuilt',True)],
        geometry_candidate_roads=[r['road'] for r in rows if r['status']=='GEOMETRY_REVIEW_CANDIDATE'],
        source_changed=False,production_accepted=False,aggregate_source_coverage_accepted=False,
        required_review='full original tails+ordinary+turns, world edge jets, consumer and surface inspection')
    dump(output/'report.json',report);print('WRITTEN REVIEW',str(path),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('component_directory');p.add_argument('baseline');p.add_argument('output')
    p.add_argument('--workers',type=int,default=4);a=p.parse_args()
    if not 1<=a.workers<=4:raise ValueError('workers must be 1..4')
    run(a.component_directory,a.baseline,a.output,a.workers)
