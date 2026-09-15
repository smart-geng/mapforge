import copy

import numpy as np
import pytest

from mapforge.ops.source_constraint_envelope import SourceConstraintEnvelope, ConstraintStructureError
from mapforge.ops.source_knot_layout import SourceKnotLayout
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from tests.test_movable_source_layout import block, constant_coefficients, parent


def state(model,x):return dict(model=model,coefficients=x)


@pytest.mark.parametrize('k', [0., 1e-12, -1e-12, .0001, -.0001])
def test_fixed_identities_and_finite_slacks_across_line_arc_transition(k):
    raw,base=block();x=constant_coefficients(base)
    layout=SourceConstraintEnvelope(base,x)
    moved=ArcSourceBoundaryBlock(raw,curvature=k,fixed_layout=SourceKnotLayout.capture(base))
    r=layout.evaluate(state(moved,x))
    assert r['complete_certificate_feasibility_equivalent'] and not r['export_allowed']
    assert len(r['inequalities'])==len(layout.keys)
    assert np.isfinite(r['inequalities']).all()
    assert r['accounting']['all_original_rows_accounted']
    assert sum(w['witness_count'] for w in r['witnesses'])==(
        r['accounting']['expanded_witness_rows']+r['accounting']['normalized_regular_rows'])
    # No disappearing or 1/k -> infinity rows around a perfectly valid Line.
    regular=[w['value'] for w in r['witnesses'] if 'regular-reference-chart' in w['identity']]
    assert regular==pytest.approx([.8-k*2,.8+k*2],abs=1e-10)


def test_complete_rows_equivalent_for_both_feasible_and_bad_coefficients():
    raw,base=block();model=ArcSourceBoundaryBlock(raw,curvature=.00001)
    x=constant_coefficients(model);layout=SourceConstraintEnvelope(model,x)
    rng=np.random.default_rng(4)
    for scale in (0.,.001,.5,100.,10000.):
        v=x+scale*rng.standard_normal(len(x))
        result=layout.evaluate(state(model,v))
        assert (min(result['inequalities'])>=0)==(min(model.C@v-model.lower)>=0)
    broken=copy.deepcopy(model)
    # One bad certificate is not hidden by hundreds of passing neighbours.
    idx=next(i for i,l in enumerate(broken.labels) if l['kind']=='source')
    broken.lower[idx]+=10
    r=layout.evaluate(state(broken,x))
    assert min(r['inequalities'])< -9
    assert any(w['witness'].get('row')==idx for w in r['witnesses'])


def test_variable_certificate_count_keeps_complete_fixed_feature_rows():
    raw,base=block();x=constant_coefficients(base);layout=SourceConstraintEnvelope(base,x)
    moved=ArcSourceBoundaryBlock(raw,heading_delta=.0001,curvature=.0001,
        fixed_layout=SourceKnotLayout.capture(base))
    assert len(moved.C)!=len(base.C)
    a=layout.evaluate(state(base,x));b=layout.evaluate(state(moved,x))
    assert len(a['inequalities'])==len(b['inequalities'])
    assert b['accounting']['original_total']==len(moved.C)
    assert len(base.families)==len(moved.families) and base.nvar==moved.nvar


@pytest.mark.parametrize('fault',['missing_identity','unknown_kind','budget','incomplete_labels'])
def test_no_identity_or_original_budget_can_disappear(fault):
    _,base=block();x=constant_coefficients(base);layout=SourceConstraintEnvelope(base,x)
    m=copy.deepcopy(base)
    if fault=='missing_identity':
        keep=[i for i,l in enumerate(m.labels) if l.get('feature')!='boundary:B:al']
        m.labels=[m.labels[i] for i in keep];m.C=m.C[keep];m.lower=m.lower[keep]
    elif fault=='unknown_kind':m.labels[0]['kind']='ignore-me'
    elif fault=='budget':m.source_tol=.75
    else:m.labels.pop()
    with pytest.raises(ConstraintStructureError):layout.evaluate(state(m,x))


def test_coefficient_only_snapshot_reuse_cannot_freeze_chart_or_knots(monkeypatch):
    p=parent(monkeypatch);v=p.initial.copy();before=p.evaluate(v)
    v[p.coefficient_slice.start]+=.001
    cached=p.evaluate_from_snapshot(v,before);fresh=p.evaluate(v)
    assert not cached['constraints_rebuilt']
    np.testing.assert_allclose(cached['source_inequality_slack'],fresh['source_inequality_slack'],atol=1e-12)
    for k in fresh['jets']:np.testing.assert_allclose(cached['jets'][k],fresh['jets'][k],atol=1e-12)
    v[1]+=.01
    rebuilt=p.evaluate_from_snapshot(v,before)
    assert rebuilt['constraints_rebuilt'] and rebuilt['model'] is not before['model']
    assert np.array_equal(before['coordinate_vector'],p.initial)
