"""One preregistered two-layout trial, original data/whole-shape guards unchanged."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.check_outer_event_control import inputs, SOURCE, REFERENCE, PACKET, DECISION
from mapforge.repair_web.model import atomic, json_bytes, digest
from mapforge.repair_web.outer_event_layout import LayoutFairing, proposals
from mapforge.validate.line_boundary_shape import written_boundary_shape
from scripts.check_outer_event_written import check, check_dynamics
from scripts.check_outer_event_shape import written_shape


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();dest=args.out.resolve()
    if dest==ROOT/'out' or not dest.is_relative_to(ROOT/'out'):parser.error('new workspace out child required')
    dest.mkdir(exist_ok=False)
    plan_path=ROOT/'profiles/repair/node4-l01-long-layout-v1.yaml'
    plan=yaml.safe_load(plan_path.read_text(encoding='utf8'))
    binding=dict(json.loads(PACKET.with_name('run.json').read_text(encoding='utf8'))['input_files_sha256'])
    code=['mapforge/repair_web/'+name+'.py' for name in
          ('model','outer_event','outer_event_control','outer_event_layout','outer_event_fairing','outer_event_shape')]
    code+=['scripts/'+name+'.py' for name in ('check_outer_event_written','check_outer_event_control','check_outer_event_shape','internal_edge_jets')]
    code+=['mapforge/validate/line_boundary_shape.py']
    for path in [SOURCE,REFERENCE,PACKET,DECISION,plan_path,Path(__file__).resolve(),*[ROOT/p for p in code]]:
        binding[str(path)]=digest(path.read_bytes())
    def verify():
        for path,sha in binding.items():
            if digest(Path(path).read_bytes())!=sha:raise ValueError('Input/code drift: '+path)
    verify();event,control=inputs()
    if plan['reference_sha256']!=digest(control.reference) or plan['control']!=asdict(control.point):
        raise ValueError('Original reference/intent differs from registered trial')
    choices=proposals(control)
    if (plan['layout']['alternatives']!=[name for name,_ in choices]
            or plan['budget']!={'maximum_layout_candidates':2,'max_iterations_per_candidate':80,
                               'max_total_iterations':160,'max_total_solve_seconds':90,'stop_on_first_verified_candidate':True}
            or plan['reference_contract']['strain_length_m']!=10.0 or plan['source_envelope_m']!=event.scope.source_tolerance_m):
        raise ValueError('Unregistered layout/solver budget')
    roles={(d['source_lane_id'],d['contact']) for d in yaml.safe_load(DECISION.read_text(encoding='utf8'))['decisions']
           if d['physical_authority']=='original_left_right_boundaries' and d['lane_path_role']=='movement_path_observation'}
    target=float(np.interp(control.point.station,control.trace['st'][:,0],control.trace['st'][:,1]))-control.reference_t
    atomic(dest/'binding.json',json_bytes(binding));atomic(dest/'scope.json',json_bytes(dict(
        event=asdict(event.scope),boundaries=event.inventory,movement_observations=event.paths)))
    started=time.monotonic();used=0;rows=[];accepted=None
    for name,scope in choices:
        remaining=90-(time.monotonic()-started)
        if remaining<=0:break
        trial=dest/name;trial.mkdir()
        fairing=LayoutFairing(control,scope,approved_roles=roles)
        atomic(trial/'preflight.json',json_bytes(fairing.preflight))
        if fairing.unchanged_xml()!=control.reference:raise ValueError('Noop reference drift')
        remaining=90-(time.monotonic()-started)
        if remaining<=0:break
        progress=lambda row:print(json.dumps(dict(layout=name,progress=row)),flush=True)
        state,report=fairing.solve(target,max_iterations=min(80,160-used),max_seconds=remaining,progress=progress)
        used+=report['iterations'];atomic(trial/'solve.json',json_bytes(report))
        rows.append(dict(layout=name,status=report['status'],iterations=report['iterations'],new_xodr=state is not None))
        if state is not None:
            data=fairing.event.compile(state)
            audit=check(data,SOURCE.read_bytes(),event.packet,asdict(scope),fairing.event.inventory)
            if audit['status']!='PASS_LOCAL_EVENT_NOT_MAP':raise ValueError('Final XML rejected')
            atomic(trial/'candidate.xodr',data);atomic(trial/'written-check.json',json_bytes(audit))
            atomic(trial/'world-shape.json',json_bytes(dict(reference=written_boundary_shape(control.reference,asdict(scope)),candidate=written_boundary_shape(data,asdict(scope)))))
            atomic(trial/'shape-comparison.json',json_bytes(dict(reference=written_shape(control.reference,event),candidate=written_shape(data,event))))
            atomic(trial/'dynamics.json',json_bytes(check_dynamics(data,event.packet,asdict(scope))))
            atomic(trial/'consumer-request.json',json_bytes(dict(xodr_sha256=digest(data),guard=dict(changed_roads=['11']))))
            accepted=name;break
    summary=dict(schema='mapforge/long-layout-trial/v1',status='LOCAL_CANDIDATE_NOT_MAP' if accepted else 'LAYOUT_TRIAL_REJECTED',
                 reference_sha256=digest(control.reference),requested_m=target,history=rows,iterations=used,
                 elapsed_seconds=time.monotonic()-started,accepted_layout=accepted,new_xodr=accepted is not None,
                 extra_independent_knots=0,map_accepted=False,web_applied=False)
    verify();atomic(dest/'summary.json',json_bytes(summary));print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
