"""R2 core tests never execute the registered real nonzero target."""
import json
from types import SimpleNamespace
import numpy as np
import pytest

from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle
from mapforge.repair_web.local_refinement import prepare_local_refinement, LO, HI, EDGE
from mapforge.repair_web.refinement_target import (target_line, affine_interval, abs_extremum,
                                                  RefinementProblem, solve_once, BudgetEnded)
from scripts.check_outer_event_control import inputs, POINT, ROOT


def test_position_eliminated_exactly_for_all_remaining_scalar_values():
    h = np.array([2., -3.]); p, n = target_line(h, -.2)
    for value in (-5., 0., 7.):
        assert abs(h@(p+n*value)+.2) < 1e-12


@pytest.mark.parametrize('h,target', [([0., 0.], 0.), ([1.], .1), ([1., float('nan')], 0.),
                                     ([1., 2.], True), ([1., 2.], float('inf')), ([1., 2.], 3.)])
def test_bad_target_or_unresponsive_handle(h, target):
    with pytest.raises(ValueError): target_line(h, target)


def test_scalar_interval_with_positive_negative_and_fixed_rows():
    result = affine_interval([0., 0., 1.], [2., -1., 0.], [-2., -3., 1.], [4., 2., 1.])
    assert result['feasible'] and result['lower'] == -1. and result['upper'] == 2.
    assert not affine_interval([2.], [0.], [-1.], [1.])['feasible']
    assert not affine_interval([0., 0.], [1., 1.], [2., -1.], [3., 1.])['feasible']


def test_source_extremum_detected_between_endpoints():
    error, at = abs_extremum([0., 4., -4., 0.], 1.)
    assert error == 1. and at == .5


@pytest.fixture(scope='module')
def problem():
    event, control = inputs()
    request = EditScopeRequest(control.reference_sha256, '11', (EDGE,), (LO, HI), (ScopeHandle(POINT),))
    return RefinementProblem(event, control, prepare_local_refinement(event, control.reference, request))


def test_zero_reference_full_metrics_match_actual_xml(problem):
    result = problem.evaluate([0., 0.])
    assert not result['failures']
    assert result['source']['max_m'] < .75
    assert min(result['margins']) >= 0
    for edge in range(5):
        for key in ('max_abs_curvature', 'max_abs_curvature_rate', 'curvature_total_variation'):
            assert result['shape_by_edge'][edge][key] == pytest.approx(problem.caps['by_edge'][str(edge)][key], abs=1e-11)
        assert result['reversal_by_edge_m'][edge] == pytest.approx(problem.reversal_caps[str(edge)], abs=1e-10)


def test_baseline_checked_compiler_noop_not_probe_permission(problem):
    data, result = problem.compile_checked(np.zeros(2), 0.)
    assert data is problem.model.reference
    assert result['purpose'] == 'R2_FULL_GUARDS_NOT_MAP'
    assert not result['map_accepted']


def test_target_mismatch_rejected_before_serialization(problem, monkeypatch):
    def forbidden(*args): raise AssertionError('Must not serialize before target guard')
    monkeypatch.setattr(type(problem.model), '_serialize_probe', forbidden)
    with pytest.raises(ValueError): problem.compile_checked(np.zeros(2), .1)


def test_whole_source_or_width_failure_not_encoded(problem, monkeypatch):
    def forbidden(*args): raise AssertionError('Must not serialize a rejected state')
    monkeypatch.setattr(type(problem.model), '_serialize_probe', forbidden)
    state = np.array([10., -10.])
    with pytest.raises(ValueError): problem.compile_checked(state, float(problem.handle@state))


def test_budget_is_checked_before_solver(problem, monkeypatch):
    import mapforge.repair_web.refinement_target as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 91.)
    with pytest.raises(BudgetEnded): solve_once(problem, 0., started=0.)


def test_single_solver_call_on_toy_not_real_target(monkeypatch):
    import mapforge.repair_web.refinement_target as module
    toy = SimpleNamespace(handle=np.array([1., 0.]), H=np.eye(2), a=np.zeros(2),
                          A=np.eye(2), low=np.array([-1., -1.]), high=np.array([1., 1.]))
    toy.evaluate = lambda state: dict(failures=[dict(code='TOY_FAIL')], margins=np.array([-1.]))
    calls = []
    def solver(fun, x, **kwargs):
        calls.append(x); kwargs['callback'](x)
        return SimpleNamespace(x=np.array(x), success=False, nit=1, message='toy refusal')
    monkeypatch.setattr(module, 'minimize', solver)
    state, report, data = solve_once(toy, .2)
    assert len(calls) == 1 and data is None
    assert report['iterations'] == 1 and not report['automatic_retry_allowed']
    assert state[0] == pytest.approx(.2)


def test_registered_result_read_only_when_available():
    path = ROOT/'out/node4-refinement-target-r2-20260917/status.json'
    if not path.exists(): pytest.skip('Single registered real execution has not happened')
    result = json.loads(path.read_bytes())
    assert result['real_target_candidates'] <= 1
    assert result['optimizer_calls'] <= 1 and result['iterations'] <= 160
    assert not result['map_accepted']


def test_preregistered_profile_matches_runner_without_execution():
    import yaml
    from scripts.check_refinement_target import PLAN, validate_plan
    plan = yaml.safe_load(PLAN.read_text(encoding='utf8'))
    validate_plan(plan)
    plan['max_wall_seconds'] = 91
    with pytest.raises(ValueError): validate_plan(plan)


def test_guard_budget_never_exceeds_600_even_with_final_checks(monkeypatch):
    import mapforge.repair_web.refinement_target as module
    toy = SimpleNamespace(handle=np.array([1., 0.]), H=np.eye(2), a=np.zeros(2),
                          A=np.eye(2), low=np.array([-1., -1.]), high=np.array([1., 1.]))
    calls = []
    def guard(state):
        calls.append(state)
        return dict(failures=[dict(code='TOY_FAIL')], margins=np.array([-1.]))
    toy.evaluate = guard
    def solver(fun, x, **kwargs):
        for i in range(601): kwargs['constraints'][0]['fun']([i/1000.])
        raise AssertionError('Must stop before this')
    monkeypatch.setattr(module, 'minimize', solver)
    _, result, data = solve_once(toy, .2)
    assert len(calls) == result['evaluations'] == 600
    assert result['status'] == 'R2_BUDGET_EXHAUSTED' and data is None


def test_empty_affine_domain_is_not_claimed_global_infeasibility():
    toy = SimpleNamespace(handle=np.array([1., 0.]), H=np.eye(2), a=np.zeros(2),
                          A=np.eye(2), low=np.array([-1., -1.]), high=np.array([1., 1.]))
    state, result, data = solve_once(toy, 2.)
    assert state is data is None
    assert result['status'] == 'R2_AFFINE_POLICY_CONFLICT'
    assert result['optimizer_calls'] == result['evaluations'] == 0
