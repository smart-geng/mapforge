"""Run the single registered S2 target once; no alternate output/run reset."""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle, prepare_edit_scope
from mapforge.repair_web.model import atomic, json_bytes, digest
from mapforge.repair_web.outer_event_control import ControlPoint
from mapforge.repair_web.unique_edit import evaluate_unique_edit
from scripts.check_outer_event_control import inputs


def load_prepared(event, control, directory):
    request = json.loads((directory/'request.json').read_bytes())
    typed = EditScopeRequest(request['reference_sha256'], request['road'], tuple(request['edges']),
                            tuple(request['interval']), tuple(ScopeHandle(ControlPoint(**h['point']), tuple(h['components']))
                            for h in request['handles']), tuple(request['acknowledged_dependencies']))
    prepared = prepare_edit_scope(event, control.reference, typed)
    if prepared.report != json.loads((directory/'scope.json').read_bytes()):
        raise ValueError('Rebuilt S1 contract differs from frozen artifact')
    linear = json.loads((directory/'linear-space.json').read_bytes())
    if (linear['contract_sha256'] != prepared.report['contract_sha256']
            or any(not np.array_equal(getattr(prepared, attr), np.asarray(linear[key])) for attr, key in
                   (('reference_state', 'reference_state'), ('space', 'basis'),
                    ('control_matrix', 'control_matrix'), ('frozen_matrix', 'frozen_matrix')))):
        raise ValueError('Stored S1 basis/reference differs from reconstructed state')
    return prepared


def main():
    plan_path = ROOT/'profiles/repair/node4-l01-unique-local-target-v1.yaml'
    plan = yaml.safe_load(plan_path.read_text(encoding='utf8'))
    if (plan['execution_directory'] != 'out/node4-unique-local-target-s2-20260915-r1'
            or plan['nonzero_target_candidates'] != 1 or plan['max_total_iterations'] != 160
            or plan['max_wall_seconds'] != 90 or plan['optimizer'] != 'none_unique_scalar_equality'
            or plan['guard_changes'] != 'none' or plan['target'] != 'original_boundary_at_same_registered_station'
            or plan['guard_policy'] != 'profiles/repair/node4-l01-curvature-fairing-v1.yaml'):
        raise ValueError('Unregistered target policy or budget')
    dest = (ROOT/plan['execution_directory']).resolve()
    if dest.exists():
        raise ValueError('S2 execution slot already exists: inspect it; no retry or new output directory')
    s1 = ROOT/'out/node4-local-edit-scope-s1-20260915-r1'
    binding = json.loads((s1/'binding.json').read_bytes())
    status = json.loads((s1/'status.json').read_bytes())
    for name, sha in status['artifact_sha256'].items():
        path = s1/name
        if digest(path.read_bytes()) != sha: raise ValueError('S1 artifact drift: '+name)
        binding[str(path)] = sha
    for path in (plan_path, ROOT/plan['guard_policy'], s1/'status.json', Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/unique_edit.py', ROOT/'mapforge/validate/line_boundary_shape.py',
                 ROOT/'tests/test_unique_edit.py'):
        binding[str(path)] = digest(path.read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Frozen input/code drift: '+path)
    verify()
    event, control = inputs()
    prepared = load_prepared(event, control, s1)
    if (plan['s1_contract_sha256'] != prepared.report['contract_sha256']
            or plan['reference_sha256'] != control.reference_sha256
            or plan['interval_m'] != prepared.report['request']['interval']
            or (plan['source_boundary'], plan['road'], plan['edge'], plan['station_m']) !=
               (control.point.source_key, event.scope.road, control.point.edge, control.point.station)):
        raise ValueError('S2 cannot replace the S1 scope, reference or source handle')
    target = float(np.interp(control.point.station, control.trace['st'][:, 0], control.trace['st'][:, 1]))-control.reference_t
    # Consume the named slot BEFORE evaluating the one nonzero shape. Even a
    # process failure leaves a started record; a new run cannot reset budget.
    dest.mkdir(exist_ok=False)
    atomic(dest/'binding.json', json_bytes(binding))
    atomic(dest/'registration.json', json_bytes(dict(plan=plan, plan_sha256=digest(plan_path.read_bytes()),
        request=asdict(control.point), target_displacement_m=target, scope_contract_sha256=plan['s1_contract_sha256'],
        state='STARTED_SINGLE_TARGET_SLOT', source_t_m=control.reference_t+target)))
    started = time.monotonic()
    try:
        state, report, data = evaluate_unique_edit(control, prepared, target, max_seconds=90.)
        verify()
        atomic(dest/'evaluation.json', json_bytes(report))
        atomic(dest/'state.json', json_bytes(dict(schema='mapforge/review-coefficient-state/v1',
            status=report['status'], source_scope_sha256=prepared.report['contract_sha256'], coefficients=state.tolist(),
            is_xodr=False, export_allowed=False, formal_certificate=False)))
        if data is not None:
            atomic(dest/'candidate.xodr', data)
        output = dict(status=report['status'], source_max_m=report['source']['max_m'],
                      target_error_m=report['target_error_m'], failures=report['failures'],
                      new_xodr=data is not None, map_accepted=False, web_changed=False,
                      candidate_count=1, optimizer_calls=0, iterations=0, input_bindings=len(binding),
                      elapsed_seconds=time.monotonic()-started,
                      artifact_sha256={p.name: digest(p.read_bytes()) for p in dest.iterdir() if p.is_file()})
        verify(); atomic(dest/'status.json', json_bytes(output))
        print(json.dumps(output, ensure_ascii=False))
    except Exception as exc:
        atomic(dest/'execution-error.json', json_bytes(dict(status='S2_EXECUTION_FAILED_SLOT_RETAINED',
            error_type=type(exc).__name__, error=str(exc), elapsed_seconds=time.monotonic()-started,
            automatic_retry_allowed=False, map_accepted=False)))
        raise


if __name__ == '__main__': main()
