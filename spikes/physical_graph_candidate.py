"""Refit an existing isolated SHP candidate, retaining real via connectors."""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
from unittest.mock import patch

from lxml import etree
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source,POLICY
from spikes import road_boundary_family as family
from spikes.clarabel_joint_candidate import interior_qp
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.g11 import load_policy,_audit_d
from mapforge.report.decision import finalize_opendrive_g8


def run(source,target,road_ids=None,min_span=15.,geometry_only=False,input_error_envelope=False,
        mouth_mode='preserve'):
    if source.resolve()==target.resolve():raise ValueError('never overwrite input')
    if mouth_mode not in ('preserve','parallel'):raise ValueError('unknown mouth mode')
    tree=etree.parse(str(source));lat,lon=_origin(tree.getroot());src=shp_source()
    manifest=json.loads(source.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
    policy=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');policy['dynamics']['sample_step_m']=.02
    results=[]
    with patch.object(family,'_convex_qp',interior_qp):
        for road in tree.findall('road'):
            rid=road.get('id')
            if road.get('junction')!='-1' or (road_ids and rid not in road_ids):continue
            attempts=[]
            for phase in (None,0.,min_span*.2,min_span*.4,min_span*.6,min_span*.8):
                result=family.solve_road(road,src,lambda g:_project(g,lat,lon),
                    boundary_association='source-order',physical_graph=True,
                    junction_endpoint_mode=mouth_mode,restore_dynamics=True,knot_phase=phase,min_span=min_span,
                    dynamics=not geometry_only,
                    source_error_budget='legacy-envelope' if input_error_envelope else 'absolute')
                attempts.append(dict(result,knot_phase=phase))
                print('ROAD',rid,'PHASE',phase,result['status'],result.get('reason'),flush=True)
                if result['status']=='CANDIDATE':break
            if result['status']=='CANDIDATE':
                sub=etree.Element('OpenDRIVE');sub.append(copy.deepcopy(road));result['dense_2cm']=_audit_d(sub,policy)
            results.append(dict(result,road=rid,attempts=attempts))
            target.parent.mkdir(parents=True,exist_ok=True)
            target.with_suffix('.progress.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    target.parent.mkdir(parents=True,exist_ok=True);tree.write(str(target),encoding='utf-8',xml_declaration=True)
    final=finalize_opendrive_g8(target,manifest,POLICY)
    final['decision']['status']='BLOCKED';final['decision']['blocked_reasons'].append({'code':'physical_boundary_graph_candidate_not_promoted'})
    target.with_suffix('.delivery-decision.json').write_text(json.dumps(final['decision'],indent=2),encoding='utf-8')
    report={'source':str(source),'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
        'roads':results,'minimum_independent_span_requested_m':min_span,
        'candidate_only':True,'diagnostic_geometry_only':geometry_only,
        'junction_endpoint_mode':mouth_mode,
        'measured_connectors_rebuilt':False,
        'source_error_budget':'legacy-envelope' if input_error_envelope else 'absolute',
        'unmodified_rejected_roads_are_not_solutions':True,
        'gates':{k:v['status'] for k,v in final['quality']['gates'].items()}}
    target.with_suffix('.fit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('FINAL',report['gates'],flush=True)
    # A diagnostic or partially solved copy must never look like CLI success.
    return (not geometry_only and bool(results)
            and all(r['status']=='CANDIDATE' and r.get('dense_2cm',{}).get('status')=='PASS' for r in results)
            and all(v=='PASS' for v in report['gates'].values()))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path);p.add_argument('--road',action='append')
    p.add_argument('--span',type=float,default=15.)
    p.add_argument('--diagnostic-geometry-only',action='store_true')
    p.add_argument('--input-error-envelope',action='store_true',help='Legacy diagnostic comparison only; can inherit large input errors')
    p.add_argument('--mouth-mode',choices=('preserve','parallel'),default='preserve',
        help='Preserve end states with unchanged measured connectors; parallel is an uncoupled diagnostic only')
    a=p.parse_args();ok=run(a.source,a.target,a.road,a.span,a.diagnostic_geometry_only,a.input_error_envelope,a.mouth_mode)
    raise SystemExit(0 if ok else 2)
