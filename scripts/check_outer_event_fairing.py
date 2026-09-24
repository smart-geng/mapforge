"""One registered real hard-position/minimal-deformation curvature trial."""
import argparse
import json
from pathlib import Path
import sys
import yaml
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.check_outer_event_control import inputs, SOURCE, REFERENCE, PACKET, DECISION
from mapforge.repair_web.model import atomic, json_bytes, digest
from mapforge.repair_web.outer_event_fairing import CurvatureFairing
from mapforge.validate.line_boundary_shape import written_boundary_shape
from scripts.check_outer_event_written import check, check_dynamics
from scripts.check_outer_event_shape import written_shape


def main():
    p = argparse.ArgumentParser(); p.add_argument('--out', type=Path, required=True)
    p.add_argument('--continuation',action='store_true')
    args = p.parse_args(); dest = args.out.resolve()
    if dest==ROOT/'out' or not dest.is_relative_to(ROOT/'out'): p.error('new workspace out child required')
    dest.mkdir(exist_ok=False)
    plan_path = ROOT/('profiles/repair/node4-l01-curvature-continuation-v1.yaml' if args.continuation else
                     'profiles/repair/node4-l01-curvature-fairing-v1.yaml')
    plan = yaml.safe_load(plan_path.read_text(encoding='utf8'))
    if digest(REFERENCE.read_bytes()) != plan['reference_sha256']: raise ValueError('Reference differs from registered trial')
    binding = dict(json.loads(PACKET.with_name('run.json').read_text(encoding='utf8'))['input_files_sha256'])
    for path in [SOURCE, REFERENCE, PACKET, DECISION, plan_path, Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/outer_event.py', ROOT/'mapforge/repair_web/outer_event_shape.py',
                 ROOT/'mapforge/repair_web/outer_event_control.py', ROOT/'mapforge/repair_web/outer_event_fairing.py',
                 ROOT/'mapforge/validate/line_boundary_shape.py', ROOT/'scripts/check_outer_event_written.py']:
        binding[str(path)]=digest(path.read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes())!=sha: raise ValueError('Input/code drift: '+path)
    verify()
    event, control = inputs(); fairing = CurvatureFairing(control)
    point = control.point
    if (plan['source_boundary'],plan['edge'],plan['station_m']) != (point.source_key,point.edge,point.station):
        raise ValueError('Trial control point drift')
    target = float(np.interp(point.station,control.trace['st'][:,0],control.trace['st'][:,1]))-control.reference_t
    progress=lambda row:print(json.dumps(row),flush=True)
    # Exact reference is the known no-op feasibility control, NOT a second seed.
    values, _, lo, hi = fairing.nonlinear(control.current)
    noop = dict(normalized_violation=float(max(0.,max(lo-values),max(values-hi))),
                reference_sha256=digest(control.reference), map_accepted=False)
    if noop['normalized_violation']>1e-7: raise ValueError('Known-reference calibration failed')
    atomic(dest/'noop-control.json',json_bytes(noop))
    if args.continuation:
        if plan['budget']['fixed_target_fractions']!=[.25,.5,.75,1.]:raise ValueError('Unregistered continuation schedule')
        state,report=fairing.continuation(target,progress=progress)
    else:
        state, report = fairing.solve(target, max_iterations=plan['budget']['max_total_slsqp_iterations'],
                                     max_seconds=plan['budget']['max_wall_seconds'], progress=progress)
    atomic(dest/'solve.json',json_bytes(report)); atomic(dest/'binding.json',json_bytes(binding))
    atomic(dest/'scope.json',json_bytes(dict(source_sha256=digest(SOURCE.read_bytes()),event=event.scope.__dict__,
                                          boundaries=event.inventory,movement_observations=event.paths)))
    if state is not None:
        data=event.compile(state)
        atomic(dest/'candidate.xodr',data)
        atomic(dest/'written-check.json',json_bytes(check(data,SOURCE.read_bytes(),event.packet,event.scope.__dict__,event.inventory)))
        atomic(dest/'world-shape.json',json_bytes(dict(reference=written_boundary_shape(REFERENCE.read_bytes(),event.scope.__dict__),
                                                     candidate=written_boundary_shape(data,event.scope.__dict__))))
        atomic(dest/'shape-comparison.json',json_bytes(dict(reference=written_shape(REFERENCE.read_bytes(),event),candidate=written_shape(data,event))))
        atomic(dest/'dynamics.json',json_bytes(check_dynamics(data,event.packet,event.scope.__dict__)))
        atomic(dest/'consumer-request.json',json_bytes(dict(xodr_sha256=digest(data),guard=dict(changed_roads=['11']))))
    verify();print(json.dumps(dict(status=report['status'],new_xodr=state is not None,requested_m=target)),flush=True)


if __name__=='__main__': main()
