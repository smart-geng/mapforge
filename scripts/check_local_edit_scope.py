"""Exercise S1 admission on frozen node4 inputs. No fit or XODR export."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle, prepare_edit_scope
from mapforge.repair_web.model import atomic, digest, json_bytes
from scripts.check_outer_event_control import inputs, POINT, REFERENCE


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    dest = args.out.resolve()
    if dest == ROOT/'out' or not dest.is_relative_to(ROOT/'out') or dest.exists():
        parser.error('A new workspace out child is required')
    # Reuse the immutable S0 input/code binding, not a moving latest file.
    binding_file = ROOT/'out/node4-edit-structure-review-20260915-r1/binding.json'
    binding = json.loads(binding_file.read_bytes())
    for path in (binding_file, Path(__file__).resolve(), ROOT/'mapforge/repair_web/edit_scope.py',
                 ROOT/'tests/test_edit_scope.py'):
        binding[str(path)] = digest(path.read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha:
                raise ValueError('Frozen input/code drift: '+path)
    verify()
    event, control = inputs()
    request = EditScopeRequest(control.reference_sha256, '11', (3,),
                               (event.scope.knots[5], event.end), (ScopeHandle(POINT),))
    # Contract cases are not geometry candidates or approvals to apply edits.
    cases = [
        ('position', request, 'EDIT_SCOPE_READY_NOT_FEASIBILITY'),
        ('three-spans', replace(request, interval=(event.scope.knots[6], event.end)), 'EDIT_NO_DOF'),
        ('position-and-slope', replace(request, handles=(ScopeHandle(POINT, ('position', 'slope')),)),
         'EDIT_INTENT_NOT_INDEPENDENT'),
        ('unacknowledged-births', replace(request, interval=(event.start, event.end)),
         'EDIT_DEPENDENCY_CONFIRMATION_REQUIRED'),
    ]
    reports, prepared = {}, None
    for name, req, expected in cases:
        result = prepare_edit_scope(event, control.reference, req)
        report = result.report
        if report['status'] != expected or result.replay_noop() != REFERENCE.read_bytes():
            raise ValueError('S1 contract regression: '+name)
        reports[name] = dict(expected=expected, actual=report['status'],
                             linear_free=report['linear_free'], control_rank=report['control_rank'],
                             usable_basis_columns=result.space.shape[1],
                             contract_sha256=report['contract_sha256'], noop_same_bytes=True)
        if name == 'position':
            prepared = result
    verify()
    dest.mkdir(exist_ok=False)
    atomic(dest/'binding.json', json_bytes(binding))
    atomic(dest/'request.json', json_bytes(asdict(request)))
    atomic(dest/'scope.json', json_bytes(prepared.report))
    atomic(dest/'linear-space.json', json_bytes(dict(
        schema='mapforge/local-edit-linear-space/v1', contract_sha256=prepared.report['contract_sha256'],
        reference_sha256=control.reference_sha256,
        reference_state=prepared.reference_state.tolist(), basis=prepared.space.tolist(),
        control_matrix=prepared.control_matrix.tolist(), frozen_matrix=prepared.frozen_matrix.tolist(),
        nonlinear_feasibility='NOT_EVALUATED', export_allowed=False)))
    status = dict(schema='mapforge/s1-contract-check/v1', status='S1_CONTRACT_CHECKS_PASS_NOT_REPAIR',
                  cases=reports, binding_count=len(binding), optimizer_calls=0, new_xodr=False,
                  map_accepted=False, web_changed=False, source_roles_changed=False,
                  artifact_sha256={p.name: digest(p.read_bytes()) for p in dest.iterdir() if p.is_file()})
    verify()
    atomic(dest/'status.json', json_bytes(status))
    print(json.dumps(status, ensure_ascii=False))


if __name__ == '__main__':
    main()
