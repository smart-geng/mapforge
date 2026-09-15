"""Freeze independent checks for a non-promoted boundary-only experiment."""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

from lxml import etree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.validate.g11 import load_policy,_audit_d
from mapforge.validate.smoothness import surface_continuity
from mapforge.report.decision import finalize_opendrive_g8


def verify(candidate,baseline,manifest_path,road_id):
    root=etree.parse(str(candidate)); old=etree.parse(str(baseline))
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    xsd=schema.validate(root)
    byid={r.get('id'):r for r in root.findall('road')}
    changes=[]; violations=[]
    if set(byid)!={r.get('id') for r in old.findall('road')}:
        violations.append('road identity set changed')
    for before in old.findall('road'):
        rid=before.get('id'); after=byid[rid]
        if etree.tostring(before)!=etree.tostring(after): changes.append(rid)
        if etree.tostring(before.find('planView'))!=etree.tostring(after.find('planView')):
            violations.append(f'{rid}:reference changed')
        for xpath in ('link','.//lane/link','.//lane/speed'):
            if [etree.tostring(x) for x in before.findall(xpath)] != [etree.tostring(x) for x in after.findall(xpath)]:
                violations.append(f'{rid}:{xpath} changed')
        if [x.get('s') for x in before.findall('lanes/laneSection')] != [x.get('s') for x in after.findall('lanes/laneSection')]:
            violations.append(f'{rid}:semantic sections changed')
        def lane_semantics(rd):
            return [(l.get('id'),l.get('type'),[(u.get('code'),u.get('value'))
                    for u in l.findall('userData') if u.get('code')=='mapforge.source_lane'])
                    for l in rd.findall('.//lane')]
        if lane_semantics(before)!=lane_semantics(after): violations.append(f'{rid}:lane/source identities or types changed')
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    final=finalize_opendrive_g8(candidate,manifest,ROOT/'profiles/validation/g8-opendrive-jinfeng-v1.yaml')
    g8,g11=final['gate'],final['g11']
    policy=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m']=.02
    single=etree.Element('OpenDRIVE'); single.append(copy.deepcopy(byid[road_id]))
    dense=_audit_d(single,policy)
    report={'schema':'mapforge/boundary-candidate-verification/v1',
            'candidate_only':True,'production_promoted':False,
            'artifact':str(candidate),'sha256':hashlib.sha256(candidate.read_bytes()).hexdigest(),
            'xsd_15m':'PASS' if xsd else 'FAIL','xsd_errors':str(schema.error_log),
            'changed_roads':changes,'invariant_violations':violations,
            'g8_status':g8['status'],'g8_scope':g8['scope'],'g8_metrics':g8['metrics'],
            'g8_scope_note':'existing comparable manifest only, not all raw source samples',
            'g11_status':g11['status'],'g11_summary':g11['summary'],
            'focused_road_id':road_id,'focused_dense_step_m':.02,'focused_dense_dynamics':dense,
            'junction_surface':surface_continuity(root.getroot()),
            'delivery_status':'BLOCKED','delivery_reasons':['candidate_not_promoted','crs_not_absolutely_verified']}
    if g11['status']!='PASS': report['delivery_reasons'].append('G11_FAIL')
    if g8['status']!='PASS': report['delivery_reasons'].append('G8_NOT_PASS')
    if dense['status']!='PASS': report['delivery_reasons'].append('DENSE_DYNAMICS_FAIL')
    for suffix,value in (('.g8.json',g8),('.g11.json',g11),('.verification.json',report),('.source-lanes.json',manifest)):
        candidate.with_suffix(suffix).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    decision=final['decision']; decision['status']='BLOCKED'; decision['candidate_only']=True
    decision['artifact_sha256']=report['sha256']
    decision['blocked_reasons'].append({'code':'candidate_not_promoted'})
    if dense['status']!='PASS': decision['blocked_reasons'].append({'code':'dense_dynamics_failed'})
    final['quality']['delivery_decision']=decision; final['quality']['candidate_only']=True
    candidate.with_suffix('.quality-report.json').write_text(json.dumps(final['quality'],indent=2,allow_nan=False),encoding='utf-8')
    candidate.with_suffix('.delivery-decision.json').write_text(json.dumps(decision,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('focused_dense_dynamics',)},indent=2,ensure_ascii=True))
    print('DENSE',json.dumps(dense['roads']))


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('candidate',type=Path); p.add_argument('baseline',type=Path)
    p.add_argument('manifest',type=Path); p.add_argument('--road',required=True)
    a=p.parse_args(); verify(a.candidate,a.baseline,a.manifest,a.road)
