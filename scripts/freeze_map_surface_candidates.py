"""Freeze four isolated visual-surface trials with unchanged driving references."""
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from lxml import etree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from spikes.map_corner_surface import build
from spikes.map_connector_widths import patch
from scripts.map_paving_audit import inspect
from scripts.esmini_lane_interfaces import check as consumer
from mapforge.validate.g11 import load_policy,_audit_d
from mapforge.validate.junction_edges import audit as edge_audit
from mapforge.report.decision import finalize_opendrive_g8


def signature(element):
    if element is None:return None
    return (element.tag,tuple(sorted(element.attrib.items())),tuple(signature(c) for c in element))


def main():
    directory=ROOT/'out/map-surface-review-v137';directory.mkdir(parents=True,exist_ok=True)
    policy=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m']=.02
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    summary=[]
    for name in ('node3','NODE5','node13','node16'):
        source=ROOT/'out/map-review-v136'/(name+'.xodr')
        target=directory/(name+'.xodr');before=ET.parse(source).getroot();root=ET.parse(source).getroot()
        surface=build(root);widths=patch(root)
        unchanged=[]
        for old in before.findall('road'):
            if old.get('name')=='junction_paving':continue
            new=root.find(f"road[@id='{old.get('id')}']")
            if old.get('junction')=='-1':assert signature(old)==signature(new)
            for xpath in ('planView','link'):
                assert signature(old.find(xpath))==signature(new.find(xpath))
            for xpath in ('.//lane/link','.//lane/speed'):
                assert [signature(x) for x in old.findall(xpath)]==[signature(x) for x in new.findall(xpath)]
            unchanged.append(old.get('id'))
        assert signature(before.find('junction'))==signature(root.find('junction'))
        ET.indent(root);ET.ElementTree(root).write(target,encoding='utf-8',xml_declaration=True)
        manifest=json.loads(source.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
        final=finalize_opendrive_g8(target,manifest,ROOT/'profiles/validation/g8-opendrive-jinfeng-v1.yaml')
        exact_root=ET.parse(target).getroot()
        dense=_audit_d(exact_root,policy)
        interfaces=consumer(target)
        edges=final['edge_contacts']
        old_edges=edge_audit(before)
        outline=inspect(target,ROOT/'out/preview/map-surface-review-v137'/(name+'-outline.png'))
        result={'case':name,'candidate_only':True,'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
                'source':str(source),'xsd':schema.validate(etree.parse(str(target))),
                'G8':final['gate']['status'],'G11':final['g11']['status'],
                'dense_2cm':dense['status'],'center_interfaces_esmini':interfaces['status'],
                'junction_edge_contacts':edges['status'],
                'unchanged_driving_references_speeds_topology':unchanged,
                'outline_export_comparison':{k:v for k,v in outline.items() if k!='mouths'},
                'surface':surface,'connector_widths':widths,'delivery':'BLOCKED',
                'edge_contact_before':old_edges['maxima'],'edge_contact_after':edges['maxima'],
                'max_jerk_mps3':max(r['lateral_jerk_mps3'] for r in dense['roads'])}
        decision=final['decision'];decision['status']='BLOCKED';decision['candidate_only']=True
        decision['blocked_reasons'] += [{'code':'candidate_not_promoted'},{'code':'sim_auxiliary_only_not_ad_strict'}]
        if edges['status']!='PASS':decision['blocked_reasons'].append({'code':'world_lane_edge_contacts_not_smooth'})
        if dense['status']!='PASS':decision['blocked_reasons'].append({'code':'dense_dynamics_failed'})
        if interfaces['status']!='PASS':decision['blocked_reasons'].append({'code':'independent_interface_failed'})
        final['quality']['delivery_decision']=decision
        for suffix,data in (('.surface-verification.json',result),('.dense-all.json',dense),
                            ('.edge-contacts.json',edges),('.delivery-decision.json',decision),
                            ('.quality-report.json',final['quality'])):
            target.with_suffix(suffix).write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        summary.append(result)
        print('FROZEN',name,result['G8'],result['G11'],result['dense_2cm'],edges['maxima'],flush=True)
    (directory/'review-summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')


def audit_formal():
    """Refresh only sidecars of old formal assets; never regenerate raw XODR."""
    rows=[]
    for folder in ('direct_xodr','m2x'):
        for path in sorted((ROOT/'out'/folder).glob('*.xodr')):
            manifest_path=path.with_suffix('.source-lanes.json')
            if not manifest_path.exists():raise ValueError(f'missing manifest: {path}')
            before=hashlib.sha256(path.read_bytes()).hexdigest()
            result=finalize_opendrive_g8(path,json.loads(manifest_path.read_text(encoding='utf-8')),
                                       ROOT/'profiles/validation/g8-opendrive-jinfeng-v1.yaml')
            assert hashlib.sha256(path.read_bytes()).hexdigest()==before
            row={'pipeline':folder,'case':path.stem,'sha256':before,
                 'G8':result['gate']['status'],'G11':result['g11']['status'],
                 'edge_contacts':result['edge_contacts']['status'],
                 'edge_maxima':result['edge_contacts'].get('maxima'),
                 'delivery':result['decision']['status']}
            rows.append(row);print(json.dumps(row),flush=True)
    (ROOT/'out/g11-edge-contacts-v137.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')


if __name__=='__main__':
    if sys.argv[1:]==['--formal']:audit_formal()
    elif sys.argv[1:]:raise SystemExit('usage: freeze_map_surface_candidates.py [--formal]')
    else:main()
