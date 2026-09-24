"""One registered R2 execution. An existing slot (even failed) cannot be reset."""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle
from mapforge.repair_web.local_refinement import prepare_local_refinement, LO, HI, EDGE, KNOT
from mapforge.repair_web.model import atomic, digest, json_bytes
from mapforge.repair_web.refinement_target import RefinementProblem, solve_once
from scripts.check_outer_event_control import inputs, POINT

R1 = ROOT/'out/node4-local-refinement-writer-r1-20260917'
DEST = ROOT/'out/node4-refinement-target-r2-20260917'
PLAN = ROOT/'profiles/repair/node4-l01-refinement-target-r2-v1.yaml'
CONTRACT = 'e20cf82112389e65aa42a3c49c37f299c5a5b93e4a200cffc57ef3579e373fde'


def validate_plan(plan):
    expected = dict(schema='mapforge/refinement-target-trial/v1',
        reference_sha256='5d1cec2baea21200fbdcf78ca9267bfe7bb94e9cf441539d759370fffdaaea98',
        r1_contract_sha256=CONTRACT, source_boundary=POINT.source_key, record=0, part=0,
        road='11', edge=EDGE, station_m=POINT.station, interval_m=[LO, HI], inserted_knot_m=KNOT,
        target='original_boundary_at_same_registered_station',
        guard_policy='profiles/repair/node4-l01-curvature-fairing-v1.yaml',
        guard_changes='none_thresholds_preserved_cells_rebuilt_for_approved_basis',
        algorithm='one_SLSQP_scalar_full_guards', position='exact_affine_elimination',
        initial='unique_quadratic_minimum_clamped_to_scalar_linear_domain',
        objective='original_integrated_squared_displacement_plus_100_times_squared_slope_change',
        finite_difference_eps=1e-6, ftol=1e-10, max_total_iterations=160,
        max_guard_evaluations=600, max_wall_seconds=90, real_target_candidates=1,
        restarts=0, extra_knots=0, execution_directory=DEST.relative_to(ROOT).as_posix(),
        on_failure='preserve_rejection_no_xml_no_scope_or_guard_change_no_new_execution_slot')
    if any(plan.get(k) != v for k, v in expected.items()):
        raise ValueError('Profile differs from registered R2 contract/budget')


def main():
    if DEST.exists():
        raise ValueError('R2 slot consumed; read evidence, never retry or change output directory')
    plan = yaml.safe_load(PLAN.read_text(encoding='utf8')); validate_plan(plan)
    binding = json.loads((R1/'binding.json').read_bytes())
    prior = json.loads((R1/'status.json').read_bytes())
    if prior['status'] != 'R1_WRITER_VERIFIED_NOT_REPAIR' or prior['contract_sha256'] != CONTRACT:
        raise ValueError('Required R1 compiler evidence missing')
    for name, sha in prior['artifact_sha256'].items(): binding[str(R1/name)] = sha
    for path in (R1/'status.json', PLAN, ROOT/plan['guard_policy'], Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/refinement_target.py', ROOT/'tests/test_refinement_target.py',
                 ROOT/'scripts/inspect_refinement_target.py'):
        sha = digest(path.read_bytes())
        if str(path) in binding and binding[str(path)] != sha:
            raise ValueError('Prior frozen binding changed: '+str(path))
        binding[str(path)] = sha
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Frozen evidence drift: '+path)
    verify()
    event, control = inputs()
    request = EditScopeRequest(control.reference_sha256, '11', (EDGE,), (LO, HI), (ScopeHandle(POINT),))
    model = prepare_local_refinement(event, control.reference, request)
    if model.contract_sha256 != CONTRACT or model._powers != (R1/'basis.json').read_bytes():
        raise ValueError('Rebuilt R1 contract/basis differs')
    target = float(np.interp(POINT.station, control.trace['st'][:, 0], control.trace['st'][:, 1]))-control.reference_t
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(binding))
    atomic(DEST/'registration.json', json_bytes(dict(plan=plan, plan_sha256=digest(PLAN.read_bytes()),
        request=asdict(POINT), target_displacement_m=target, source_t_m=control.reference_t+target,
        state='STARTED_SINGLE_TARGET_SLOT', consumer_validation='R3_ONLY_IF_R2_PASS',
        budget_scope='R2 affine cells, single solve, state guards and actual XML polynomial readback')))
    started = time.monotonic()
    try:
        problem = RefinementProblem(event, control, model)
        # This is the sole nonzero real target evaluation/solve, never called by tests.
        state, report, data = solve_once(problem, target, started=started,
            progress=lambda r: print(json.dumps(r), flush=True))
        domain = report['domain']
        report['linear_domain_witnesses'] = {
            k: {field: value for field, value in problem.linear[index].items() if field != 'D'}
            for k, index in domain.items() if k in ('lower_row', 'upper_row', 'fixed_conflict_row') and index is not None}
        report['model_counts'] = dict(shape_cells=len(problem.spans), source_cells=len(problem.sources),
                                     width_cells=len(problem.widths), linear_rows=len(problem.linear))
        verify()
        atomic(DEST/'evaluation.json', json_bytes(report))
        atomic(DEST/'state.json', json_bytes(dict(schema='mapforge/local-refinement-state/v1',
            status=report['status'], r1_contract_sha256=CONTRACT,
            coefficients=None if state is None else state.tolist(),
            is_xodr=False, export_allowed=False, map_accepted=False)))
        if data is not None: atomic(DEST/'candidate.xodr', data)
        guard = report.get('guard_evaluation') or {}
        summary = dict(status=report['status'], real_target_candidates=1, optimizer_calls=report['optimizer_calls'],
            iterations=report['iterations'], evaluations=report['evaluations'],
            elapsed_seconds=report['elapsed_seconds'], new_xodr=data is not None,
            xodr_sha256=None if data is None else digest(data), target_error_m=report.get('target_error_m'),
            source_max_m=guard.get('source', {}).get('max_m'), failures=guard.get('failures'),
            r1_contract_sha256=CONTRACT, input_bindings=len(binding), input_drift=0,
            automatic_retry_allowed=False, map_accepted=False, web_changed=False,
            xsd='NOT_RUN', esmini='NOT_RUN', operating_dynamics='NOT_EVALUATED',
            absolute_crs='NOT_VALIDATED', legacy_dynamics=prior['legacy_dynamics'],
            artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})
        verify(); atomic(DEST/'status.json', json_bytes(summary))
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    except Exception as exc:
        atomic(DEST/'execution-error.json', json_bytes(dict(status='R2_EXECUTION_FAILED_SLOT_RETAINED',
            error_type=type(exc).__name__, error=str(exc), elapsed_seconds=time.monotonic()-started,
            automatic_retry_allowed=False, map_accepted=False)))
        raise


if __name__ == '__main__': main()
