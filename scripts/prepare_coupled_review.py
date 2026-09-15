"""Prepare all-turn independent readers from one actual simultaneous trial.

Only adapts provenance/report layout. Does not rerun a solver, change XML,
reuse geometric PASS verdicts, or replace the retained research revision.
"""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code


def run(directory,baseline,output):
    directory,baseline,output=[Path(p).resolve() for p in (directory,baseline,output)]
    trial=json.loads((directory/'report.json').read_text(encoding='utf8'))
    old=json.loads((baseline/'report.json').read_text(encoding='utf8'))
    saved=json.loads((directory/'joint-state.json').read_text(encoding='utf8'))
    if not trial['artifact'] or not trial['simultaneous_parent_connector_variables']:
        raise ValueError('actual simultaneous XML trial required')
    path=Path(trial['artifact'])
    if _sha256(path)!=trial['sha256'] or Path(trial['input'])!=Path(old['artifact']) or trial['input_sha256']!=old['sha256']:
        raise ValueError('one exact parent/input/map revision required')
    hashes,changes=bind_current_code(trial['source_hashes'])
    unchanged(hashes)
    incoming={r['road']:r for r in trial['connectors']}
    if set(incoming)!=set(trial['affected_roads']) or len(incoming)!=len(trial['connectors']):
        raise ValueError('complete unique dependent trial inventory required')
    if not set(incoming)<=set(r['road'] for r in old['connectors']):
        raise ValueError('trial changed the known physical turn inventory')
    source=Path(trial['north_component'])
    state=json.loads((source/'shared-state.json').read_text(encoding='utf8'))
    if state['model_sha']!=saved['model_sha'] or state['flat_port']:
        raise ValueError('mismatched source model')
    state['coefficients']=saved['parent_coefficients']
    output.mkdir(parents=True,exist_ok=False)
    north=output/'north-state';north.mkdir()
    state['source_hashes']=hashes
    dump(north/'shared-state.json',state)
    dump(north/'report.json',dict(artifact=str(path),sha256=trial['sha256'],production_accepted=False,
        status='SHARED_COEFFICIENT_READBACK_INPUT_NOT_ACCEPTANCE'))
    hashes.update({str(p):_sha256(p) for p in (Path(__file__),directory/'report.json',directory/'joint-state.json',
        baseline/'report.json',north/'shared-state.json',north/'report.json',path)})
    packet=dict(status='REJECTED_PENDING_INDEPENDENT_READBACK',artifact=str(path),sha256=trial['sha256'],
        component=old['component'],north_component=str(north),source_hashes=hashes,
        input=old['artifact'],input_sha256=old['sha256'],
        connectors=[incoming.get(r['road'],dict(road=r['road'],unchanged_from_input=True)) for r in old['connectors']],
        affected_roads=trial['affected_roads'],simultaneous_parent_connector_variables=True,
        no_previous_pass_verdict_reused=True,code_changes_before_independent_readback=changes,
        source_changed=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',packet)
    print('ALL TURN REVIEW PREPARED',len(packet['connectors']),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('baseline');p.add_argument('output')
    a=p.parse_args();run(a.directory,a.baseline,a.output)
