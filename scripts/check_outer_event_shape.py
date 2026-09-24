"""One bounded real static-shape candidate; complete readback, no live UI writes."""
import argparse
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.repair_web.outer_event import OuterEvent
from mapforge.repair_web.outer_event_shape import solve_shape, solve_shape_no_regression, wrong_way_distance
from mapforge.repair_web.model import atomic,json_bytes,digest
from scripts.check_outer_event_written import boundary_poly,check,check_dynamics


def written_shape(data, event):
    """Reads actual XML polynomials, not the solver's state or Bernstein slacks."""
    road=next(r for r in ET.fromstring(data).findall('road') if r.get('id')==event.scope.road)
    cuts={event.start,event.end}
    cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        s=float(sec.get('s'));cuts.add(s)
        cuts.update(s+float(e.get('sOffset')) for e in sec.findall('.//width'))
    rows=[]
    for trace,lo,hi,source in event.source_cells():
        split=[lo]+sorted(s for s in cuts if lo<s<hi)+[hi]
        value=sum(wrong_way_distance(boundary_poly(road,trace['edge'],a), b-a, np.sign(source[1]))
                  for a,b in zip(split,split[1:]))
        rows.append(dict(edge=trace['edge'],key=trace['key'],s=[lo,hi],source_direction=int(np.sign(source[1])),reversal_m=value))
    by_edge={str(edge):sum(r['reversal_m'] for r in rows if r['edge']==edge) for edge in range(event.count+1)}
    total=sum(by_edge.values())
    return dict(xodr_sha256=digest(data),source_direction_reversal_m=total,by_edge=by_edge,rows=rows,
                monotonicity='PASS_IN_REPORTED_SCOPE' if total<=1e-7 else 'RESIDUAL_REVERSALS',
                scope='all original boundary segment intersections with L01 support; not whole map',map_accepted=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--no-regression',action='store_true',help='Exact whole-interval reversal; no transfer to another edge')
    args=p.parse_args();dest=args.out.resolve()
    if dest==ROOT/'out' or not dest.is_relative_to(ROOT/'out'):p.error('new child of out/ required')
    dest.mkdir(exist_ok=False)
    source=ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr'
    packet_path=ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
    prior=ROOT/'out/node4-outer-event-l01-20260915-r3/candidate.xodr'
    decision=ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml'
    run=packet_path.with_name('run.json');packet=json.loads(packet_path.read_text(encoding='utf8'))
    binding=dict(json.loads(run.read_text(encoding='utf8'))['input_files_sha256'])
    for path in [source,packet_path,prior,decision,run,Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/outer_event.py',ROOT/'mapforge/repair_web/outer_event_shape.py',
                 ROOT/'scripts/check_outer_event_written.py']:
        binding[str(path)]=digest(path.read_bytes())
    def verify():
        for path,sha in binding.items():
            if digest(Path(path).read_bytes())!=sha:raise ValueError('input or code drift: '+path)
    verify()
    roles={(d['source_lane_id'],d['contact']) for d in yaml.safe_load(decision.read_text(encoding='utf8'))['decisions']
           if d['physical_authority']=='original_left_right_boundaries' and d['lane_path_role']=='movement_path_observation'}
    event=OuterEvent(source.read_bytes(),packet,approved_roles=roles)
    comparison=dict(baseline=written_shape(source.read_bytes(),event),prior=written_shape(prior.read_bytes(),event))
    progress=lambda r:print(json.dumps(r),flush=True)
    if args.no_regression:
        budgets=[comparison['prior']['by_edge'][str(i)] for i in range(event.count+1)]
        state,report=solve_shape_no_regression(event,budgets,progress=progress)
    else:
        state,report=solve_shape(event,progress=progress)
    atomic(dest/'solve.json',json_bytes(report));atomic(dest/'binding.json',json_bytes(binding))
    atomic(dest/'scope.json',json_bytes(dict(source_sha256=digest(source.read_bytes()),event=event.scope.__dict__,
                                            boundaries=event.inventory,movement_observations=event.paths)))
    if state is not None:
        data=event.compile(state)
        atomic(dest/'candidate.xodr',data)
        atomic(dest/'written-check.json',json_bytes(check(data,source.read_bytes(),packet,event.scope.__dict__,event.inventory)))
        comparison['candidate']=written_shape(data,event)
        atomic(dest/'dynamics.json',json_bytes(check_dynamics(data,packet,event.scope.__dict__)))
        atomic(dest/'consumer-request.json',json_bytes(dict(xodr_sha256=digest(data),guard=dict(changed_roads=['11']))))
    atomic(dest/'shape-comparison.json',json_bytes(comparison))
    verify()
    print(report['status'],{k:(v['source_direction_reversal_m'],v['by_edge']) for k,v in comparison.items()},flush=True)


if __name__=='__main__':main()
