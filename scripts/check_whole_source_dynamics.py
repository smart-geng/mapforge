"""Read node4 originals and evaluate long-curve dynamics in ONE whole state."""
import argparse
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np

from scripts.prepare_whole_source_model import (prepare_roles,raw_models,register_turns,
    ProfileSource,MovableSourceParentState,WholeSourcePortState,WholeSourceGeometryState,
    original_greville_seed,read,dump,unchanged,_sha256)
from mapforge.ops.whole_source_dynamics import WholeSourceDynamics


def run(preparation,review,decision,output):
    preparation,review,decision,output=map(lambda p:Path(p).resolve(),(preparation,review,decision,output))
    output.mkdir(parents=True,exist_ok=False);start=time.monotonic()
    dump(output/'registration-contract.json',dict(status='REGISTERED_WHOLE_STATE_DYNAMIC_CHECK',
        parents=['10','11','12','13','30'],turns=list(map(str,range(100,124))),
        fitting_iterations=0,changed_geometry_parameters=0,geometry_segments_added=0,
        source_speed_policy='unchanged originals; mixed turning limits marked envelope-only',
        limits=dict(ay_mps2=2.5,lateral_rate_mps3=1.),bernstein_max_depth=12,
        minimum_check_span_m=1e-6,declared_geometry_spans_not_final_written_ownership=True,
        no_formal_source_geometry_certificate=True,xodr_writer_enabled=False))
    print('RE-READ ORIGINAL INPUTS AND NINE APPROVALS',flush=True)
    root,scope,domain,contacts,roles,config,hashes,changes=prepare_roles(preparation,review,decision)
    hashes.update({str(p):_sha256(p) for d in ('mapforge','spikes','scripts') for p in (ROOT/d).rglob('*.py')})
    dump(output/'input-binding.json',dict(input_files_sha256=hashes,input_code_revision_changes=changes))
    models,_=raw_models(root,scope,domain,contacts,roles,config)
    parents={rid:MovableSourceParentState(o,m,original_greville_seed(m),source_root=root)
             for rid,(o,m) in models.items()}
    ports=WholeSourcePortState(root,config['whole_junction']['xodr_junction_id'],parents,scope['connectors'])
    initial=ports.evaluate(ports.initial)
    kernels,providers,_=register_turns(ports,initial,ProfileSource(config['source_dir'],config['profile']),scope,domain)
    geometry=WholeSourceGeometryState(ports,kernels,providers)
    saved=read(ROOT/'out/node4-whole-source-model-r4-20260914/whole-coordinate-state.json')['initial']
    if not np.array_equal(geometry.initial,saved):raise ValueError('unregistered initial geometry change')
    dump(output/'coordinate-state.json',dict(vector=geometry.initial.tolist(),unchanged_registered_initial=True))
    print('EVALUATE WHOLE STATE AND CONTINUOUS SPAN DEMAND',flush=True)
    def progress(kind,rid,r):
        print(kind,rid,r['status'],r['counts'],flush=True)
    report=WholeSourceDynamics(geometry,scope['observations']).evaluate(geometry.initial,progress=progress)
    dump(output/'parent-spans.json',report.pop('parent_rows'))
    dump(output/'turn-spans.json',report.pop('turn_rows'))
    report.update(elapsed_seconds=time.monotonic()-start,input_files_sha256=hashes,input_code_revision_changes=changes,
        original_and_approval_hashes_unchanged=True,xodr_generated=False)
    unchanged(hashes);dump(output/'report.json',report)
    print('COMPLETE DEMAND CHECK, NOT MAP',round(report['elapsed_seconds'],2),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for k in ('preparation','review','decision','output'):p.add_argument(k)
    a=p.parse_args();run(a.preparation,a.review,a.decision,a.output)
    raise SystemExit(2)  # This entry never accepts a map.
