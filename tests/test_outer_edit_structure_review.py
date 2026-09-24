"""Read-only rank checks don't promise a feasible or exportable edited map."""
import numpy as np
import pytest
from scripts.check_outer_event_control import inputs
from scripts.review_outer_edit_structure import capacity,frozen_rows


@pytest.fixture(scope='module')
def context():return inputs()


def test_preserving_every_edge_leaves_no_edit(context):
    event,control=context
    # A selection window wholly before this edge's birth has no free curve.
    result=capacity(event,control.handle,[3],event.start,event.scope.knots[1])
    assert result['free_after_preservation']==0 and result['rank_stable']
    assert not result['target_has_linear_freedom']


def test_three_clamped_cubic_spans_have_no_local_control(context):
    event,control=context
    result=capacity(event,control.handle,[3],event.scope.knots[6],event.end)
    assert result['free_after_preservation']==0 and result['rank_stable']
    assert not result['map_accepted']


def test_four_clamped_spans_provide_one_direction_not_feasibility(context):
    event,control=context
    result=capacity(event,control.handle,[3],event.scope.knots[5],event.end)
    assert result['free_after_preservation']==1 and result['rank_stable']
    assert result['target_has_linear_freedom']
    assert result['target_feasibility']=='NOT_EVALUATED'
    assert result['frozen_polynomial_residual']<1e-8


def test_two_editable_edges_are_distinct_explicit_scope(context):
    event,control=context
    result=capacity(event,control.handle,[3,4],event.scope.knots[5],event.end)
    assert result['free_after_preservation']==2 and result['rank_stable']
    assert result['original_constraint_residual']<1e-8


def test_half_span_pin_is_polynomial_not_one_sample(context):
    event,_=context
    a=event.scope.knots[5];b=event.end
    exact=frozen_rows(event,[3],a,b)@event.Z
    partial=frozen_rows(event,[3],a+.1,b)@event.Z
    # A nonempty frozen portion fixes the entire cubic on that interval.
    assert np.linalg.matrix_rank(partial,tol=1e-8)>np.linalg.matrix_rank(exact,tol=1e-8)


@pytest.mark.parametrize('edges,lo,hi',[([],100,190),([6],100,190),([3],90,190),([3],190,180)])
def test_scope_not_silently_expanded(context,edges,lo,hi):
    with pytest.raises(ValueError):frozen_rows(context[0],edges,lo,hi)
