"""One fixed-parent joint solve; full-map independent review remains required."""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source,_sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from spikes.measured_connector_caps import frames,raw_curves,composite_sources
from spikes.trial_xml import replace_road,write_trial
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.ops.joint_connector_fit import fit_joint


def run(source_directory,output,rid,resume_current_code=False,fair_world=False,max_iterations=100,core_count=3):
    source_directory=Path(source_directory).resolve();output=Path(output).resolve()
    info=json.loads((source_directory/'report.json').read_text(encoding='utf8'));source=Path(info['artifact'])
    if _sha256(source)!=info['sha256']:raise ValueError('changed input artifact')
    from scripts.research_code_revision import bind_current_code
    hashes,code_changes=bind_current_code(info['source_hashes'])
    if code_changes and not resume_current_code:raise ValueError('code revision changed; explicit --resume-current-code required')
    unchanged(hashes);tree=ET.parse(source);root=tree.getroot()
    original={r.get('id'):ET.tostring(r) for r in root.findall('road')};src=shp_source();lat,lon=_origin(root)
    road=next(r for r in root.findall('road') if r.get('id')==rid)
    sid=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']").get('value')
    via=raw_curves(src,sid,lambda p:_project(p,lat,lon));raw,support=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
    previous=next(r for r in info['connectors'] if r['road']==rid)
    output.mkdir(parents=True,exist_ok=False)
    code_files=[Path(__file__),ROOT/'scripts/research_code_revision.py',ROOT/'mapforge/ops/joint_connector_fit.py',ROOT/'mapforge/ops/long_connector_chain.py',ROOT/'mapforge/ops/world_curve_fairness.py',ROOT/'spikes/road_boundary_family.py',ROOT/'mapforge/ops/source_connector_ribbon.py',ROOT/'spikes/measured_connector_caps.py']
    code_hashes={str(p):_sha256(p) for p in code_files}
    code_hashes.update({str(p):_sha256(p) for p in [ROOT/'mapforge/ops/fixed_ribbon_sampling.py',ROOT/'mapforge/ops/step_limited_sqp.py']})
    p=ROOT/'mapforge/ops/endpoint_jet_coordinates.py';code_hashes[str(p)]=_sha256(p)
    tails=[dict(role=role,source_lane_id=sid,curves=raw_curves(src,sid,lambda p:_project(p,lat,lon)))
        for role,sid in [('predecessor',support['source_lane_ids'][0]),('successor',support['source_lane_ids'][-1])]]
    q=previous['shape_parameters'];coefficients=previous.get('joint_coefficients');a,b=frames(root,road)
    old_count=previous.get('reference_core_count',3)
    if core_count!=old_count:
        if old_count!=3 or core_count!=4:raise ValueError('only one explicit long-core expansion is supported')
        from mapforge.ops.long_connector_chain import expand_core
        from spikes.measured_connector_caps import needs_cap
        q=expand_core(q,a,b,(needs_cap(a),needs_cap(b)));coefficients=None
    candidate,row=fit_joint(road,a,b,raw,via,q,initial_coefficients=coefficients,
        fair_world=fair_world,max_iterations=max_iterations,core_count=core_count,source_tails=tails,
        progress=lambda state:print('PROGRESS',rid,json.dumps(state),flush=True))
    row['reference_core_expanded']=core_count!=old_count
    row['source_support']=support;replace_road(root,road,candidate)
    assert all(ET.tostring(r)==original[r.get('id')] for r in root.findall('road') if r.get('id')!=rid)
    path=output/'node4-review.xodr';write_trial(tree,path);unchanged(hashes);unchanged(code_hashes)
    report=dict(status='REVIEW_NOT_DELIVERY',artifact=str(path),sha256=_sha256(path),input=str(source),input_sha256=info['sha256'],
        source_hashes=hashes,connectors=[row],source_changed=False,unchanged_other_roads=True,
        production_accepted=False,code_sha256=code_hashes,input_code_revision_changes=code_changes)
    dump(output/'report.json',report);dump(output/'connectors.json',[row]);print('JOINT',rid,row['status'],row['optimizer'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source_directory');p.add_argument('output');p.add_argument('--road',default='122')
    p.add_argument('--resume-current-code',action='store_true',help='log implementation changes, reject any raw/config change')
    p.add_argument('--fair-world',action='store_true');p.add_argument('--max-iterations',type=int,default=100)
    p.add_argument('--core-count',type=int,choices=(3,4),default=3)
    a=p.parse_args();run(a.source_directory,a.output,a.road,a.resume_current_code,a.fair_world,a.max_iterations,a.core_count)
