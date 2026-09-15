"""Bounded source-seed probe on the exact prepared parent; no map promotion."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256,shp_source
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from spikes.measured_connector_caps import frames,raw_curves,composite_sources
from spikes.trial_xml import replace_road,write_trial
from mapforge.ops.long_source_seed import initialize
from mapforge.ops.joint_connector_fit import fit_joint


def run(preparation,output,rid,max_iterations=60):
    preparation=Path(preparation).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    p=json.loads((preparation/'preparation.json').read_text(encoding='utf8'));source=Path(p['artifact'])
    if _sha256(source)!=p['sha256']:raise ValueError('changed parent input')
    hashes,changes=bind_current_code(p['source_hashes'])
    hashes.update({str(f):_sha256(f) for f in (Path(__file__),ROOT/'mapforge/ops/long_source_seed.py',
        preparation/'preparation.json',source)})
    tree=ET.parse(source);root=tree.getroot();road=next(r for r in root.findall('road') if r.get('id')==rid)
    src=shp_source();lat,lon=_origin(root);project=lambda xy:_project(xy,lat,lon)
    raw,support=composite_sources(root,road,src,project)
    via=raw_curves(src,road.find(".//userData[@code='mapforge.source_lane']").get('value'),project)
    a,b=frames(root,road)
    try:q,coeff,seed=initialize(a,b,raw)
    except (ValueError,ArithmeticError) as exc:
        rejection=dict(status='REJECTED_SOURCE_REFERENCE_INITIALIZATION',reason=str(exc),source_hashes=hashes,
            parent_input_sha256=p['sha256'],source_identity=support,road=rid,
            source_changed=False,xodr_generated=False,production_accepted=False)
        unchanged(hashes);dump(output/'rejection.json',rejection)
        print('SEED REJECTED',rid,str(exc),flush=True);return rejection
    dump(output/'seed.json',dict(seed=seed,parameters=q.tolist(),coefficients=coeff.tolist(),
        source_identity=support,source_hashes=hashes))
    print('BOUNDED SEED',rid,seed,flush=True)
    tails=[dict(role=role,source_lane_id=sid,curves=raw_curves(src,sid,project)) for role,sid in (
        ('predecessor',support['source_lane_ids'][0]),('successor',support['source_lane_ids'][-1]))]
    candidate,row=fit_joint(road,a,b,raw,via,q,initial_coefficients=coeff,fair_world=True,
        max_iterations=max_iterations,source_tails=tails,
        progress=lambda state:print('PROGRESS',rid,json.dumps(state),flush=True))
    row.update(source_support=support,bounded_source_seed=seed)
    replace_road(root,road,candidate);path=output/'node4-unconnected-diagnostic.xodr';write_trial(tree,path)
    unchanged(hashes)
    report=dict(status='REJECTED_INCOMPLETE_PARENT_TRANSACTION',artifact=str(path),sha256=_sha256(path),
        input=str(source),input_sha256=p['sha256'],source_hashes=hashes,connectors=[row],
        exact_prepared_parent_sha256=p['sha256'],source_changed=False,production_accepted=False,
        other_incident_roads_not_rebuilt=True,code_revision_changes=changes)
    dump(output/'report.json',report);print('PROBE',rid,row['status'],flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('preparation');p.add_argument('output');p.add_argument('--road',default='122')
    p.add_argument('--max-iterations',type=int,default=60);a=p.parse_args()
    run(a.preparation,a.output,a.road,a.max_iterations);raise SystemExit(2)
