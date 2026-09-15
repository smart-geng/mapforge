"""Same-state feasibility restoration; no parent/traffic/speed edits."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code
from scripts.review_fixed_parent_replacement import require_fixed_parent
from mapforge.ops.joint_connector_restore import restore
from spikes.measured_connector_caps import frames
from spikes.trial_xml import replace_road,write_trial


def run(directory,output,rid):
    directory=Path(directory).resolve();output=Path(output).resolve()
    original=json.loads((directory/'report.json').read_text(encoding='utf-8'))
    if _sha256(Path(original['artifact']))!=original['sha256']:raise ValueError('changed input')
    hashes,changes=bind_current_code(original['source_hashes'])
    unchanged(original['code_sha256'])
    tree=ET.parse(original['artifact']);before=ET.fromstring(ET.tostring(tree.getroot()));root=tree.getroot()
    road=next(r for r in root.findall('road') if r.get('id')==rid)
    previous=next(r for r in original['connectors'] if r['road']==rid)
    new,row=restore(road,*frames(root,road),previous['shape_parameters'],previous['joint_coefficients'],
        core_count=previous.get('reference_core_count',3))
    replace_road(root,road,new);require_fixed_parent(before,root,{rid})
    output.mkdir(parents=True,exist_ok=False);path=output/'node4-review.xodr';write_trial(tree,path)
    code={str(p):_sha256(p) for p in (Path(__file__),ROOT/'mapforge/ops/joint_connector_restore.py',
        ROOT/'mapforge/ops/joint_connector_fit.py',ROOT/'mapforge/ops/long_connector_chain.py',ROOT/'scripts/research_code_revision.py')}
    hashes.update(code);unchanged(hashes)
    report=dict(status='REVIEW_NOT_DELIVERY',artifact=str(path),sha256=_sha256(path),
        input=original['artifact'],input_sha256=original['sha256'],source_hashes=hashes,code_sha256=code,
        connectors=[row],input_code_revision_changes=changes,production_accepted=False)
    dump(output/'report.json',report);print(row,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');p.add_argument('--road',default='122')
    a=p.parse_args();run(a.directory,a.output,a.road)
