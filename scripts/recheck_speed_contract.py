"""Fresh source-to-XODR research run; source limits never tuned to geometry.

Always creates a new directory and never rewrites historical outputs. Speed
integrity, complete-source fidelity and dynamics remain independent gates.
"""
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all, parse_map_xml
from mapforge.ops.map_to_xodr import build_xodr, _source_lane_key, _lane_speed_kmh
from mapforge.ops.shp_to_xodr import build_junction_xodr
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.dynamics_speed import lane_limits
from mapforge.validate.g11 import _audit_d, load_policy
from mapforge.ops.port_dependencies import revision
from scripts.gen_all import CASES, POLICY, shp_source


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def dump(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def check_limits(root, source_limits):
    rows=[];errors=[]
    for road in root.findall('road'):
        if road.get('name')=='junction_paving':continue
        for section in road.findall('lanes/laneSection'):
            for lane in section.findall('left/lane')+section.findall('right/lane'):
                if lane.get('type')!='driving':continue
                ud=lane.find("userData[@code='mapforge.source_lane']")
                sid=ud.get('value') if ud is not None else None
                records=lane_limits(lane)
                expected=source_limits.get(sid)
                reasons=[]
                if sid is not None and sid not in source_limits: reasons.append('unresolved-source-lane')
                if expected is None:
                    if records:reasons.append('invented-limit-without-source')
                elif (not records or records[0]['s_offset_m']!=0.
                      or any(abs(r['max_kmh']-expected)>1e-5 for r in records)):
                    reasons.append('source-limit-not-preserved')
                row=dict(road_id=road.get('id'),section_s_m=float(section.get('s')),lane_id=lane.get('id'),
                         source_lane_id=sid,source_limit_kmh=expected,written_limits=records,reasons=reasons)
                rows.append(row)
                if reasons:errors.append(row)
    return dict(status='FAIL' if errors or not rows else 'PASS',checked_lane_occurrences=len(rows),
                source_bound_lane_occurrences=sum(r['source_lane_id'] is not None for r in rows),
                failures=errors,lanes=rows)


def design_proposal(root):
    # Existing draft fallback of 15 km/h is a RESEARCH proposal, not a new
    # approved speed for all turns. Bind each actual connecting lane explicitly.
    rows=[]
    for road in root.findall('road'):
        if road.get('junction','-1')=='-1' or road.get('name')=='junction_paving':continue
        pred=road.find('link/predecessor');succ=road.find('link/successor')
        for section in road.findall('lanes/laneSection'):
            for lane in section.findall('left/lane')+section.findall('right/lane'):
                if lane.get('type')!='driving':continue
                rows.append(dict(road_id=road.get('id'),lane_id=lane.get('id'),section_s_m=float(section.get('s')),
                    movement_id=f"{pred.get('elementId') if pred is not None else '?'}:{road.get('id')}:{succ.get('elementId') if succ is not None else '?'}",
                    target_speed_kmh=15.,basis='existing draft connector fallback; proposal only, not inferred from curvature'))
    return dict(schema='mapforge/movement-design-speed/v1',artifact_revision=revision(root),
                approval_state='proposed',bindings=rows)


def run(output, pipeline, label):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    xmls=[ROOT/'v2x_map_xml'/name for _,name in CASES]
    source=ROOT/'v2x_map_xml'/dict(CASES)[label]
    nodes=[n for path in xmls for n in parse_map_xml_all(str(path))]
    node=parse_map_xml(str(source))
    files=xmls+list((ROOT/'mapforge').rglob('*.py'))+[Path(__file__).resolve(),POLICY,
          ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml',ROOT/'profiles/shp/ibd-smarteditor-v1.yaml']
    if pipeline=='shp':files+=list((ROOT/'shp_0222-0326').glob('*.*'))
    hashes={str(p.relative_to(ROOT)):sha(p) for p in files if p.is_file()}
    target=output/(label+'.xodr')
    print('CONVERT',pipeline,label,flush=True)
    if pipeline=='map':
        neighbors=[n for n in nodes if (n.region,n.node_id)!=(node.region,node.node_id)]
        stats=build_xodr(node,target,neighbors=neighbors)
        source_limits={_source_lane_key(n,lk,ln):_lane_speed_kmh(ln) for n in nodes for lk in n.links for ln in lk.lanes}
    else:
        src=shp_source();junc,distance=src.find_junction(node.ref_lon,node.ref_lat)
        if junc is None or distance>50:raise ValueError('source junction not resolved')
        stats=build_junction_xodr(src,junc,target)
        root=ET.parse(target).getroot();source_limits={}
        for item in root.findall(".//lane/userData[@code='mapforge.source_lane']"):
            sid=item.get('value');record=src.lane(sid)
            if record is not None:source_limits[sid]=record.max_speed_kmh or None
    root=ET.parse(target).getroot();source_speed=check_limits(root,source_limits)
    print('SPEED_INTEGRITY',source_speed['status'],source_speed['checked_lane_occurrences'],flush=True)
    final=finalize_opendrive_g8(target,stats['source_lane_manifest'],POLICY,
                              raw_map_paths=xmls if pipeline=='map' else None)
    policy=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m']=.02
    stress=_audit_d(root,policy)
    policy['dynamics']['movement_design']=design_proposal(root)
    proposed=_audit_d(root,policy)
    from scripts.esmini_lane_interfaces import check
    from lxml import etree
    consumer=check(target,edges=True)
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    xsd=schema.validate(etree.parse(str(target)))
    print('READBACK',pipeline,label,stress['status'],consumer['status'],xsd,flush=True)
    # Readback visuals use XODR values, not optimizer's in-memory target.
    if pipeline=='shp':
        from scripts.shp_xodr_overlay import render
        render(ROOT/'shp_0222-0326',target,output/'source-overlay.png','ibd-smarteditor-v1')
    else:
        from scripts.freeze_joint_mouth_candidates import source_plot
        source_plot(target,target,output/'source-overlay.png',baseline_title='same candidate (no baseline claim)',
                    candidate_title='fresh default / source limits preserved')
    decision=final['decision'];decision['status']='BLOCKED';decision['candidate_only']=True
    decision['blocked_reasons'].append({'code':'research_run_not_promoted'})
    if source_speed['status']!='PASS':decision['blocked_reasons'].append({'code':'source_speed_integrity_failed'})
    if stress['status']!='PASS':decision['blocked_reasons'].append({'code':'dense_dynamics_failed'})
    if consumer['status']!='PASS':decision['blocked_reasons'].append({'code':'independent_consumer_failed'})
    if not xsd:decision['blocked_reasons'].append({'code':'xsd_failed'})
    final['quality']['delivery_decision']=decision
    for suffix,data in [('.source-speed.json',source_speed),('.dense-stress.json',stress),
                        ('.design-proposal.json',proposed),('.consumer.json',consumer),
                        ('.stats.json',stats),('.delivery-decision.json',decision),
                        ('.quality-report.json',final['quality'])]:dump(target.with_suffix(suffix),data)
    if any(sha(ROOT/p)!=h for p,h in hashes.items()):raise ValueError('source/code changed during run')
    report=dict(schema='mapforge/source-speed-recheck/v1',pipeline=pipeline,case=label,status='BLOCKED',
                xodr_sha256=sha(target),source_speed_integrity=source_speed['status'],
                g8=final['gate']['status'],g11=final['g11']['status'],dense_stress=stress['metrics'],
                design_proposal=proposed['metrics'],design_approved=False,consumer=consumer['status'],xsd=xsd,
                source_and_code_sha256=hashes,geometry_repaired=False,default_promoted=False)
    dump(output/'run.json',report)
    print(json.dumps({k:v for k,v in report.items() if k!='source_and_code_sha256'}),flush=True)
    return 2


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output');p.add_argument('pipeline',choices=['shp','map'])
    p.add_argument('case',choices=dict(CASES));a=p.parse_args()
    raise SystemExit(run(a.output,a.pipeline,a.case))
