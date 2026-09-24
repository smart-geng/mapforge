"""Prepare a SHA-bound control interval and actual-XML previews in a new folder."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.model import atomic, digest, json_bytes
from mapforge.repair_web.outer_event import OuterEvent
from mapforge.repair_web.outer_event_control import ControlPoint, EventControl
from scripts.check_outer_event_written import check_dynamics
from mapforge.validate.line_boundary_shape import written_boundary_shape

SOURCE = ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr'
REFERENCE = ROOT/'out/node4-outer-event-shape-20260915-r6/candidate.xodr'
PACKET = ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
DECISION = ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml'
# Registered reproduction of the visible third-boundary bulge, NOT a user
# approval to apply this target, or a new source-role decision.
POINT = ControlPoint('IBD_LANE_BOUNDARY:2023041110502726490', 0, 0, 3, 178.)


def inputs():
    packet = json.loads(PACKET.read_text(encoding='utf8'))
    roles = {(d['source_lane_id'], d['contact']) for d in yaml.safe_load(DECISION.read_text(encoding='utf8'))['decisions']
             if d['physical_authority'] == 'original_left_right_boundaries' and d['lane_path_role'] == 'movement_path_observation'}
    event = OuterEvent(SOURCE.read_bytes(), packet, approved_roles=roles)
    return event, EventControl(event, REFERENCE.read_bytes(), POINT)


def main():
    p = argparse.ArgumentParser(); p.add_argument('--out', type=Path, required=True)
    args = p.parse_args(); dest = args.out.resolve()
    if not dest.is_relative_to(ROOT/'out') or dest == ROOT/'out': p.error('new child of workspace out/ required')
    dest.mkdir(exist_ok=False)
    run = PACKET.with_name('run.json')
    binding = dict(json.loads(run.read_text(encoding='utf8'))['input_files_sha256'])
    for path in (SOURCE, REFERENCE, PACKET, DECISION, run, Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/outer_event.py', ROOT/'mapforge/repair_web/outer_event_shape.py',
                 ROOT/'mapforge/repair_web/outer_event_control.py', ROOT/'scripts/check_outer_event_written.py',
                 ROOT/'scripts/check_outer_event_shape.py', ROOT/'scripts/internal_edge_jets.py',
                 ROOT/'mapforge/repair_web/model.py', ROOT/'mapforge/validate/line_boundary_shape.py'):
        binding[str(path)] = digest(path.read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('input/code drift: '+path)
    verify()
    event, control = inputs()
    report = control.prepare_range(progress=lambda r: print(json.dumps(r), flush=True))
    atomic(dest/'range.json', json_bytes(report)); atomic(dest/'binding.json', json_bytes(binding))
    atomic(dest/'control.json', json_bytes(dict(point=asdict(POINT), reference_sha256=digest(REFERENCE.read_bytes()),
                                               witnesses={str(k): v.tolist() for k, v in control.witnesses.items()})))
    atomic(dest/'scope.json', json_bytes(dict(source_sha256=digest(SOURCE.read_bytes()), event=event.scope.__dict__,
                                            boundaries=event.inventory, movement_observations=event.paths)))
    # Explicit source-derived test intent; do not pretend an unattainable target
    # was requested at the returned boundary instead. Preserve the rejection.
    target = report['source_point_delta_m']; lo, hi = report['verified_delta_m']
    atomic(dest/'target-diagnosis.json', json_bytes(control.diagnose_target(target)))
    try:
        data, guard = control.preview(target)
        selection = 'SOURCE_POINT_TARGET_REACHED'
    except ValueError as exc:
        atomic(dest/'source-target-rejection.json', json_bytes(dict(requested_m=target, reason=str(exc))))
        # A separate, explicitly named test of the inside-interval interaction.
        target = (lo if target < 0 else hi)*.5
        data, guard = control.preview(target)
        selection = 'SEPARATE_HALF_RANGE_TEST_NOT_SOURCE_TARGET'
    atomic(dest/'candidate.xodr', data)
    atomic(dest/'preview.json', json_bytes(dict(test=selection, guard=guard)))
    atomic(dest/'world-shape.json', json_bytes(dict(
        reference=written_boundary_shape(REFERENCE.read_bytes(), event.scope.__dict__),
        candidate=written_boundary_shape(data, event.scope.__dict__))))
    atomic(dest/'written-check.json', json_bytes(guard['readback']))
    atomic(dest/'consumer-request.json', json_bytes(dict(xodr_sha256=digest(data), guard=dict(changed_roads=['11']))))
    atomic(dest/'dynamics.json', json_bytes(check_dynamics(data, event.packet, event.scope.__dict__)))
    verify()
    print(json.dumps(dict(status=selection, interval=[lo, hi], requested=target, achieved=guard['achieved_m'],
                          sha256=digest(data), map_accepted=False)), flush=True)


if __name__ == '__main__': main()
