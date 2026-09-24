"""Single bounded, source-bound L01 joint geometry/dynamics attempt."""
import argparse
import json
from pathlib import Path
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.repair_web.model import atomic,json_bytes,digest
from mapforge.repair_web.outer_event import OuterEvent,EventScope,WEST_SPLIT
from mapforge.repair_web.outer_event_dynamics import DynamicEvent
from scripts.check_outer_event_written import check,check_dynamics


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--whole-parent-support',action='store_true',help='Single declared wider-support comparison; same reference, births, endpoint locks and source thresholds')
    args=p.parse_args();dest=args.out.resolve()
    if not dest.is_relative_to(ROOT/'out'):p.error('output must be below workspace out/')
    source=ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr'
    packet_path=ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
    decision=ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml'
    run=packet_path.with_name('run.json');packet=json.loads(packet_path.read_text(encoding='utf8'))
    binding=dict(json.loads(run.read_text(encoding='utf8'))['input_files_sha256'])
    for path in [packet_path,decision,run,Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/outer_event.py',ROOT/'mapforge/repair_web/outer_event_dynamics.py',
                 ROOT/'scripts/check_outer_event_written.py',ROOT/'mapforge/ops/continuous_offset_dynamics.py']:
        binding[str(path)]=digest(path.read_bytes())
    def verify():
        for path,sha in binding.items():
            if digest(Path(path).read_bytes())!=sha:raise ValueError('input or code drift: '+path)
    verify();dest.mkdir(exist_ok=False)
    roles={(d['source_lane_id'],d['contact']) for d in yaml.safe_load(decision.read_text(encoding='utf8'))['decisions']
           if d['physical_authority']=='original_left_right_boundaries' and d['lane_path_role']=='movement_path_observation'}
    scope=WEST_SPLIT
    if args.whole_parent_support:
        scope=EventScope('11',(0.,30.,60.,*WEST_SPLIT.knots,211.7074107),WEST_SPLIT.births)
    event=OuterEvent(source.read_bytes(),packet,scope,approved_roles=roles)
    joint=DynamicEvent(event,packet);started=time.monotonic()
    state,report=joint.solve(progress=lambda r:print(json.dumps(r,ensure_ascii=False),flush=True))
    report.update(seconds=time.monotonic()-started,source_sha256=digest(source.read_bytes()),
                  event_scope=scope.__dict__,whole_parent_support=args.whole_parent_support,
                  state_variables=event.nvar,free_variables=event.Z.shape[1],
                  dynamic_spans=len(joint.spans),source_speeds=sorted({s['speed_kmh'] for s in joint.spans}),
                  source_interval_mapping='same-source uniform speed in this event only')
    atomic(dest/'report.json',json_bytes(report));atomic(dest/'binding.json',json_bytes(binding))
    if state is not None:
        data=event.compile(state)
        atomic(dest/'candidate.xodr',data)
        atomic(dest/'written-check.json',json_bytes(check(data,source.read_bytes(),packet,event.scope.__dict__,event.inventory)))
        atomic(dest/'dynamics.json',json_bytes(check_dynamics(data,packet,event.scope.__dict__)))
    verify()
    print({k:v for k,v in report.items() if k not in ('necessary','dynamic_audit')})
    print('necessary', {k:v for k,v in report.get('necessary',{}).items() if k!='witnesses'})


if __name__=='__main__':main()
