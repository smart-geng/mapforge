"""Algebraic necessary conditions, not more R2 target trials."""
import numpy as np
import pytest
from scripts.diagnose_refinement_conflict import reversal_lower_support, minimum_abs_curvature


@pytest.mark.parametrize('source_direction', [-1., 0., 1.])
def test_reversal_support_is_lower_bound_at_other_synthetic_states(source_direction):
    from mapforge.repair_web.outer_event_shape import wrong_way_distance
    cp = dict(edge=3, lo=0., hi=2., c=np.array([0., 1., -1., 0.]),
              D=np.array([[0., 0.], [1., 0.], [0., 0.], [0., 0.]]),
              source=[0., source_direction, 0., 0.], key='synthetic', record=0, part=0)
    a, g, _ = reversal_lower_support([cp], np.zeros(2), np.array([1., 0.]), .2)
    for z in (-1., -.3, .2, 1.):
        actual = wrong_way_distance(cp['c']+cp['D']@np.array([z, 0.]), 2., source_direction)
        assert a+g*z <= actual+1e-12
        if z == .2: assert a+g*z == pytest.approx(actual, abs=1e-12)


def test_curvature_minimum_includes_interior_zero():
    result, _ = minimum_abs_curvature([1., .2], [-.2, 1.], -1., 1.)
    assert result['scalar'] == pytest.approx(.2)
    assert result['abs_curvature'] < 1e-14


def test_curvature_minimum_includes_stationary_point_and_endpoints():
    result, rows = minimum_abs_curvature([0., 1.], [1., 0.], -1., 1.)
    assert {r['scalar'] for r in rows} == {-1., 0., 1.}
    assert result['abs_curvature'] == pytest.approx(2**-1.5)
