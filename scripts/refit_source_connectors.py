"""Refit measured connectors in a new review copy; never promote a map."""
import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source,_sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import restore_source_speeds,dump
from spikes.measured_connector_caps import frames,raw_curves,composite_sources,fit
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from spikes.trial_xml import replace_road,write_trial


def run(source,output,road_ids,parent_trial=False,world_g2=False,seed_report=None):
    source=Path(source).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    if parent_trial:
        baseline=json.loads((source.parent/'junction-review.json').read_text(encoding='utf8'))
        checkpoint=json.loads((source.parent/'shared-state.json').read_text(encoding='utf8'))
        baseline['source_hashes']=checkpoint['source_hashes']
        if set(road_ids)!={r['road'] for r in baseline['connectors']}:
            raise ValueError('a different shared parent requires ALL its dependent connectors')
    else:
        baseline=json.loads((source.parent/'review-validation.json').read_text(encoding='utf8'))
    if _sha256(source)!=baseline['sha256']:raise ValueError('baseline file revision mismatch')
    unchanged(baseline['source_hashes']);tree=ET.parse(source);root=tree.getroot();src=shp_source();lat,lon=_origin(root)
    records=[];input_sha=_sha256(source)
    seeds={}
    if seed_report is not None:
        saved=json.loads(Path(seed_report).read_text(encoding='utf8'))
        if input_sha not in (saved.get('sha256'),saved.get('input_sha256')):raise ValueError('seed belongs to a different parent revision')
        seeds={r['road']:r['shape_parameters'] for r in saved['connectors'] if 'shape_parameters' in r}
    parent_before={r.get('id'):ET.tostring(r) for r in root.findall('road') if r.get('id') not in road_ids}
    for rid in road_ids:
        road=next(r for r in root.findall('road') if r.get('id')==rid)
        speed=restore_source_speeds([road],src)
        sid=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']").get('value')
        via=raw_curves(src,sid,lambda p:_project(p,lat,lon));raw,support=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
        a,b=frames(root,road)
        try:
            new,row=fit(road,a,b,raw,adaptive_caps=True,source_warm_start=rid not in seeds,raw_via=via,parameters=seeds.get(rid),
                        require_dynamics=False,source_cross_section=True,world_g2_cross_section=world_g2)
            replace_road(root,road,new)
        except (ValueError,ArithmeticError) as exc:
            row=dict(road=rid,status='REJECTED',reason=str(exc),geometry_rebuilt=False)
        row.update(source_support=support,source_speed_restoration=speed);records.append(row)
        dump(output/'connectors.json',records)
        print('SOURCE RIBBON',rid,row['status'],row.get('reason',''),flush=True)
    assert all(ET.tostring(next(r for r in root.findall('road') if r.get('id')==rid))==xml for rid,xml in parent_before.items())
    path=output/'node4-review.xodr';write_trial(tree,path)
    assert _sha256(source)==input_sha;unchanged(baseline['source_hashes'])
    report=dict(status='REVIEW_NOT_DELIVERY',input=str(source),input_sha256=input_sha,artifact=str(path),
                sha256=_sha256(path),connectors=records,unchanged_other_roads=True,source_changed=False,
                source_hashes=baseline['source_hashes'],production_accepted=False,explicit_parent_trial=parent_trial,
                seed_report=None if seed_report is None else dict(path=str(Path(seed_report).resolve()),sha256=_sha256(Path(seed_report))),
                code_sha256={str(p):_sha256(p) for p in [Path(__file__),ROOT/'mapforge/ops/source_connector_ribbon.py',
                  ROOT/'spikes/measured_connector_caps.py',ROOT/'spikes/connector_cross_section.py']})
    dump(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');p.add_argument('--roads',default='100,104,105,122,123')
    p.add_argument('--parent-trial',action='store_true',help='rebuild ALL connectors of an unpromoted shared-parent experiment')
    p.add_argument('--world-g2',action='store_true',help='world G2 with nonflat transverse slopes; same long basis')
    p.add_argument('--seed-report',help='hash-bound parameters of the same parent revision, not replacement geometry')
    a=p.parse_args();run(a.source,a.output,a.roads.split(','),a.parent_trial,a.world_g2,a.seed_report)
