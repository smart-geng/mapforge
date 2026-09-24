from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle, ScopeError, prepare_edit_scope
from mapforge.repair_web.unique_edit import unique_position_state, evaluate_unique_edit, shape_metrics, width_metrics
from scripts.check_outer_event_control import inputs, POINT


def test_scalar_equality_not_soft_fit_and_reference_not_modified():
    x = np.array([2., 5.]); N = np.array([[0.], [2.]]); h = np.array([0., 1.])
    y, u = unique_position_state(x, N, h, 7.)
    assert u == 1. and h@y == 7. and y[0] == 2.
    np.testing.assert_array_equal(x, [2., 5.])


@pytest.mark.parametrize('target', [float('inf'), float('nan'), True, '3'])
def test_bad_target_not_clipped(target):
    with pytest.raises(ScopeError): unique_position_state([0.], [[1.]], [1.], target)


def test_more_than_one_free_direction_not_unique():
    with pytest.raises(ScopeError) as exc:
        unique_position_state([0., 0.], np.eye(2), [1., 0.], 1.)
    assert exc.value.code == 'EDIT_NOT_UNIQUE'


def test_unresponsive_or_nonfinite_matrix_rejected():
    with pytest.raises(ScopeError): unique_position_state([0.], [[0.]], [1.], 1.)
    with pytest.raises(ScopeError): unique_position_state([0.], [[float('nan')]], [1.], 1.)


@pytest.fixture(scope='module')
def context():
    event, control = inputs()
    req = EditScopeRequest(control.reference_sha256, '11', (3,), (event.scope.knots[5], event.end), (ScopeHandle(POINT),))
    prepared = prepare_edit_scope(event, control.reference, req)
    return event, control, prepared


def test_reference_noop_passes_old_guards_and_preserves_original_bytes(context, monkeypatch):
    import mapforge.repair_web.outer_event as module
    event, control, prepared = context
    def forbidden(*a, **kw): raise AssertionError('No optimizer may be called')
    monkeypatch.setattr(module, 'minimize', forbidden)
    monkeypatch.setattr(module, 'linprog', forbidden)
    state, report, data = evaluate_unique_edit(control, prepared, 0.)
    assert report['status'] == 'UNIQUE_LOCAL_CANDIDATE_NOT_MAP'
    assert data is control.reference
    assert not report['failures'] and not report['map_accepted']
    assert report['readback']['frozen_scope_max_m'] < 1e-8
    assert report['iterations'] == report['optimizer_calls'] == 0
    assert width_metrics(event, state)
    assert len(shape_metrics(event, state)) == 5


@pytest.mark.parametrize('changes', ['basis', 'control', 'reference', 'source_identity'])
def test_s1_drift_rejected_before_target_evaluation(context, changes):
    _, control, prepared = context
    if changes == 'basis': bad = replace(prepared, space=prepared.space*2)
    elif changes == 'control': bad = replace(prepared, control_matrix=prepared.control_matrix+1.)
    elif changes == 'reference': bad = replace(prepared, reference=b'wrong')
    else:
        from mapforge.repair_web.model import json_bytes, digest
        report = prepared.report
        report['controls'][0]['source_key'] = 'wrong'
        report.pop('contract_sha256'); report['contract_sha256'] = digest(json_bytes(report))
        bad = replace(prepared, _report_json=json_bytes(report))
    with pytest.raises(ScopeError): evaluate_unique_edit(control, bad, 0.)


@pytest.mark.parametrize('value', [True, 2.1, None, float('nan')])
def test_reject_invalid_displacements(context, value):
    with pytest.raises(ScopeError): evaluate_unique_edit(context[1], context[2], value)


def test_timeout_not_map_acceptance(context):
    _, control, prepared = context
    _, report, data = evaluate_unique_edit(control, prepared, 0., max_seconds=1e-12)
    assert data is None and report['status'] == 'UNIQUE_TARGET_REJECTED'
    assert 'TIME_BUDGET' in [r['code'] for r in report['failures']]


def test_actual_registered_result_is_not_silently_retried():
    # No new shape evaluation: replay the saved verdict if this one-slot
    # execution has already happened. Initial development may precede it.
    import json
    root = Path(__file__).resolve().parents[1]
    path = root/'out/node4-unique-local-target-s2-20260915-r1/evaluation.json'
    if not path.exists(): pytest.skip('registered S2 execution not run yet')
    report = json.loads(path.read_bytes())
    assert report['candidate_count'] == 1 and report['shape_free_after_position'] == 0
    assert report['iterations'] == report['optimizer_calls'] == 0
    assert report['target_error_m'] <= 1e-8
    assert report['status'] in ('UNIQUE_TARGET_REJECTED', 'UNIQUE_LOCAL_CANDIDATE_NOT_MAP')
    assert not report['map_accepted']
    if report['status'] == 'UNIQUE_TARGET_REJECTED':
        assert report['failures'] and not (path.parent/'candidate.xodr').exists()
