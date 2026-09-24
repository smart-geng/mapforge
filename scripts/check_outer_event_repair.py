"""Bounded real-data check for L01; never overwrite an editor session."""
import json
import argparse
import sys
import yaml
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.repair_web.outer_event import OuterEvent
from mapforge.repair_web.model import atomic,json_bytes,parse,digest,complexity
from scripts.internal_edge_jets import audit
from scripts.check_outer_event_written import check,check_dynamics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True,help='New evidence directory; never overwrite an earlier candidate')
    args=parser.parse_args()
    dest=args.out.resolve()
    if not dest.is_relative_to(ROOT/'out'):parser.error('output must be a new child of workspace out/')
    dest.mkdir(exist_ok=False)
    source=ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr'
    packet=ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
    run=json.loads(packet.with_name('run.json').read_text(encoding='utf8'))
    decision=ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml'
    binding={**run['input_files_sha256'],str(packet):digest(packet.read_bytes()),
             str(decision):digest(decision.read_bytes())}
    def verify():
        for path,sha in binding.items():
            if digest(Path(path).read_bytes())!=sha:raise ValueError('Input drift: '+path)
    verify()
    roles={(r['source_lane_id'],r['contact']) for r in yaml.safe_load(decision.read_text(encoding='utf8'))['decisions']
           if r['physical_authority']=='original_left_right_boundaries' and r['lane_path_role']=='movement_path_observation'}
    event=OuterEvent(source.read_bytes(),json.loads(packet.read_text(encoding='utf8')),approved_roles=roles)
    x,report=event.solve()
    atomic(dest/'binding.json',json_bytes(binding))
    atomic(dest/'scope.json',json_bytes({'source_sha256':digest(source.read_bytes()),
        'event':event.scope.__dict__,'boundaries':event.inventory,'movement_observations':event.paths}))
    atomic(dest/'solve.json',json_bytes(report))
    print({k:v for k,v in report.items() if k!='source'})
    print('source max',report['source']['max_m'])
    print('worst',sorted(report['source']['rows'],key=lambda r:r['max_m'],reverse=True)[:5])
    if report['accepted']:
        data=event.compile(x)
        atomic(dest/'candidate.xodr',data)
        parsed=parse(data); written=audit(parsed)
        atomic(dest/'internal-edges.json',json_bytes(written))
        independent=check(data,source.read_bytes(),json.loads(packet.read_text(encoding='utf8')),event.scope.__dict__,event.inventory)
        atomic(dest/'written-check.json',json_bytes(independent))
        print('Independent', {k:v for k,v in independent.items() if not isinstance(v,list)})
        consumer_report={'xodr_sha256':digest(data),'guard':{'changed_roads':['11']}}
        atomic(dest/'consumer-request.json',json_bytes(consumer_report))
        dynamics=check_dynamics(data,json.loads(packet.read_text(encoding='utf8')),event.scope.__dict__)
        atomic(dest/'dynamics.json',json_bytes(dynamics))
        print('Dynamics',dynamics['status'],dynamics['counts'],dynamics['max_observed'])
        atomic(dest/'status.json',json_bytes({'status':'BLOCKED_RESEARCH_CANDIDATE',
            'xodr_sha256':digest(data),'local_geometry':independent['status'],
            'local_dynamics':dynamics['status'],'web_enabled':False,'map_accepted':False}))
        print('SHA',digest(data),'failures',written['failures'],'complexity',complexity(parsed))
    verify()


if __name__=='__main__':main()
