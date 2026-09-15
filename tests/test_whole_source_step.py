import copy

import numpy as np
import pytest

from mapforge.ops.source_constraint_envelope import ConstraintStructureError
from mapforge.ops.whole_source_step import WholeSourceStepModel
from tests.test_whole_source_constraints import synthetic_system


def step_model(s,base=None,columns=None):
    base=s.base if base is None else base
    stencil=s.linearize(base,columns)
    radius=np.full(len(s.initial),.05);radius[s.fixed_columns]=0
    return WholeSourceStepModel(s,base,stencil,radius=radius)


def test_two_parent_and_turn_updates_are_atomic_and_match_full(monkeypatch):
    s=synthetic_system(monkeypatch);g=s.geometry
    original_turn=g.evaluate_turn;expected=s.initial.copy()
    expected[[1,10,16]]+=[.001,-.0001,.002]
    seen=[]
    def check(cid,local,parents):
        # Every movement, including one touched through BOTH parents, must
        # see both parents at the new state, and must be evaluated only once.
        for rid,sl in g.ports.slices.items():
            np.testing.assert_array_equal(parents[rid]['coordinate_vector'],expected[sl])
        seen.append(cid)
        return original_turn(cid,local,parents)
    g.evaluate_turn=check
    trial=s.trial_vector(s.base,expected)
    assert sorted(seen)==['100','101','102']
    full=s.evaluate(expected)
    np.testing.assert_array_equal(trial.values,full.values)
    np.testing.assert_array_equal(s.base.vector,s.initial)
    assert not np.array_equal(trial.vector,s.initial)


def test_average_slopes_cannot_hide_opposite_changes_at_min_tie(monkeypatch):
    s=synthetic_system(monkeypatch);v=s.initial.copy();v[6:8]=0.
    base=s.evaluate(v);m=step_model(s,base)
    d=np.zeros(len(v));d[6]=.01;d[7]=-.01
    pred=m.predict(d)
    row=s.slices[('parent','10')].stop-1
    assert pred['central'][row]==pytest.approx(0.)
    assert pred['lower'][row]==pytest.approx(-.01)
    assert pred['upper'][row]==pytest.approx(.01)
    checked=m.verify(d,independent_full=True);report=checked['report']
    assert checked['snapshot'].values[row]==pytest.approx(-.01)
    assert row in report['newly_violated_inequality_rows']
    assert not report['all_modeled_inequalities_feasible']
    assert report['independent_full_max_difference']==0
    assert not report['export_allowed']


def test_actual_nonlinear_check_catches_false_prediction_at_finite_step(monkeypatch):
    s=synthetic_system(monkeypatch);m=step_model(s)
    d=np.zeros(len(s.initial));d[[1,16]]=[.01,.01]
    r=m.verify(d,independent_full=True)['report']
    assert r['complete_free_columns'] and r['independent_full_max_difference']==0
    assert r['maximum_scaled_model_discrepancy']>0
    assert not r['equality_feasibility_certified'] and not r['export_allowed']


def test_independent_oracle_rejects_an_inactive_row_missed_by_working_model(monkeypatch):
    s=synthetic_system(monkeypatch);v=s.initial.copy();v[6]=.001
    base=s.evaluate(v);stencil=s.linearize(base)
    row=s.slices[('parent','10')].stop-1
    # Deliberately bad local approximation: a formerly feasible constraint
    # is predicted flat. The exact full-state check must still evaluate it.
    for name in ('right','left'):
        a=stencil['sided_matrices'][name].tolil();a[row,:]=0
        stencil['sided_matrices'][name]=a.tocsc()
    radius=np.full(len(v),.05);radius[s.fixed_columns]=0
    m=WholeSourceStepModel(s,base,stencil,radius=radius)
    d=np.zeros(len(v));d[6]=-.01
    result=m.verify(d)['report']
    assert row in result['falsely_predicted_feasible_rows']
    assert row in result['newly_violated_inequality_rows']
    assert not result['export_allowed']


def test_missing_derivative_side_and_partial_columns_never_become_zeros(monkeypatch):
    s=synthetic_system(monkeypatch);v=s.initial.copy();v[16]=0
    base=s.evaluate(v);m=step_model(s,base,columns=[16])
    assert not m.stencil['sided_valid']['left'][0]
    d=np.zeros(len(v));d[16]=-.001
    with pytest.raises(ValueError,match='rejected source-domain'):m.predict(d)
    d[16]=.001
    assert not m.predict(d)['complete_free_columns']
    d[1]=.001
    with pytest.raises(ValueError,match='unchecked'):m.predict(d)


def test_stale_linearization_and_excessive_joint_motion_fail_closed(monkeypatch):
    s=synthetic_system(monkeypatch);m=step_model(s)
    changed=s.trial_column(s.base,1,.001)
    with pytest.raises(ConstraintStructureError,match='different whole state'):
        WholeSourceStepModel(s,changed,m.stencil,radius=m.radius)
    d=np.zeros(len(s.initial));d[1]=.1
    with pytest.raises(ValueError,match='radius'):m.verify(d)
    d=np.zeros(len(s.initial));d[4]=1e-6
    with pytest.raises(ValueError):m.verify(d)
    with pytest.raises(ValueError):m.predict(d[:-1])


def test_trial_rejection_cannot_commit_an_earlier_parent_change(monkeypatch):
    s=synthetic_system(monkeypatch)
    before=copy.deepcopy(s.base)
    p=s.geometry.ports.parents['11'];old=p.evaluate_from_snapshot
    def reject(v,state):raise ValueError('registered source family changed')
    p.evaluate_from_snapshot=reject
    v=s.initial.copy();v[[1,9]]+=.01
    with pytest.raises(ValueError,match='family'):s.trial_vector(s.base,v)
    np.testing.assert_array_equal(s.base.vector,before.vector)
    np.testing.assert_array_equal(s.base.values,before.values)
    for rid in before.full['parents']:
        np.testing.assert_array_equal(s.base.full['parents'][rid]['coordinate_vector'],
                                      before.full['parents'][rid]['coordinate_vector'])
    p.evaluate_from_snapshot=old
