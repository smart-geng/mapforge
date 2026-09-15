"""Rebuild SHA-bound saved shapes with current strict via and closure checks.

Default: no optimization/restarts. Explicit refit IDs use the SHA-bound shape
as their starting point, with the complete raw via constrained separately.
No shifting whole curves, no rejected-road deletion.
The input batch and formal files remain untouched. The result remains BLOCKED.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.review_measured_ribbon import bound_trial,independent_design_check
from spikes.measured_connector_caps import frames,fit,raw_curves,composite_sources
from scripts.gen_all import shp_source
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.g11 import audit_file
from mapforge.validate.junction_edges import audit as edge_audit
from spikes.trial_xml import replace_road,write_trial


def run(source,target,*,refit_roads=()):
    if source.resolve()==target.resolve():raise ValueError('never overwrite trial batch')
    original=bound_trial(source);tree=ET.parse(source);root=tree.getroot();src=shp_source();lat,lon=_origin(root)
    refit_roads=set(refit_roads)
    unknown=refit_roads-{r['road'] for r in original['roads']}
    if unknown:raise ValueError('unknown refit roads: '+','.join(sorted(unknown)))
    rows=[]
    for old in original['roads']:
        road=root.find("road[@id='%s']"%old['road'])
        row={'road':old['road'],'status':'REJECTED'}
        try:
            sid=road.find(".//lane/userData[@code='mapforge.source_lane']").get('value')
            via=raw_curves(src,sid,lambda p:_project(p,lat,lon))
            raw,info=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
            if 'shape_parameters' not in old:raise ValueError('no cached shape; input rejected road retained')
            new,row=fit(road,*frames(root,road),raw,optimize=old['road'] in refit_roads,adaptive_caps=True,
                        parameters=old['shape_parameters'],raw_via=via)
            row['source_composite']=info;row['design_15kmh_dynamics_2cm']=independent_design_check(new)
            if row['design_15kmh_dynamics_2cm']['status']!='PASS':row['status']='REJECTED'
            replace_road(root,road,new)
        except (ValueError,KeyError) as exc:row['reason']=str(exc)
        row['prior_trial_status']=old['status'];rows.append(row)
        print('ROAD',old['road'],row['status'],flush=True)
        target.parent.mkdir(parents=True,exist_ok=True)
        target.with_suffix('.progress.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    xsd=write_trial(tree,target)
    report={'status':'BLOCKED','candidate_only':True,'source':str(source),'source_sha256':original['sha256'],
            'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'roads':rows,
            'source_support_mode':'source-linked-composite','structural_mouth_retreat_m':0.,
            'source_manifest_not_promoted':True,'ordinary_roads_unchanged':True,
            'explicit_refit_roads':sorted(refit_roads),
            'xsd':xsd,
            'scope':'all saved connector shapes; source-constrained ordinary-mouth parent chain must also pass',
            'g11':audit_file(target,ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml'),'edges':edge_audit(root)}
    target.with_suffix('.ribbon.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print([(r['road'],r['status']) for r in rows]);return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path)
    p.add_argument('--refit-road',action='append',default=[],help='refit only these saved shapes under the unchanged full-source gates')
    a=p.parse_args();run(a.source,a.target,refit_roads=a.refit_road);raise SystemExit(2)
