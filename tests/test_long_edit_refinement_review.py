"""Representation-only checks: never solve or export a nonzero target."""
import numpy as np
import pytest
from scipy.interpolate import BSpline

from scripts.review_long_edit_refinement import insert_long_knot, clamped_capacity, review
from scripts.check_outer_event_control import inputs


@pytest.fixture(scope='module')
def context():
    return inputs()


@pytest.mark.parametrize('station', [float('nan'), float('inf'), True, 0., 30., 10., 12.])
def test_repeated_outside_or_short_knots_rejected(station):
    t = [0.]*4+[10., 20.]+[30.]*4
    bs = BSpline(t, np.arange(6.), 3)
    with pytest.raises(ValueError):
        insert_long_knot(bs, station)


def test_boehm_keeps_cubic_coefficients_and_original_object():
    t = [0.]*4+[15.]+[30.]*4
    bs = BSpline(t, [1., 2., 5., -2., 3.], 3)
    original_t, original_c = bs.t.copy(), bs.c.copy()
    result = insert_long_knot(bs, 22.5)
    for s in (0., 15., 22.5):
        for derivative in range(4):
            np.testing.assert_allclose(result(s, nu=derivative), bs(s, nu=derivative), atol=1e-13)
    np.testing.assert_array_equal(bs.t, original_t)
    np.testing.assert_array_equal(bs.c, original_c)


def test_capacity_is_not_three_independent_controls():
    cuts = [157.6089, 163.6089, 173.3754, 187.5, 194.60370535, 201.7074107]
    result = clamped_capacity(cuts, 178.)
    assert result['free_directions'] == 2
    assert result['free_after_one_position'] == 1
    assert result['position_slope_curvature_jet_ranks'] == [1, 2, 2]
    assert result['endpoint_jet_residual'] < 1e-9


def test_real_reference_structural_review_no_solver_or_compiler(context, monkeypatch):
    event, control = context
    def forbidden(*args, **kwargs):
        raise AssertionError('Design review must not solve, evaluate S2 or compile edits')
    import mapforge.repair_web.outer_event as module
    import mapforge.repair_web.unique_edit as s2
    monkeypatch.setattr(module, 'minimize', forbidden)
    monkeypatch.setattr(module, 'linprog', forbidden)
    monkeypatch.setattr(event, 'compile', forbidden)
    monkeypatch.setattr(s2, 'evaluate_unique_edit', forbidden)
    prior = control.current.copy()
    result = review(event, control)
    assert result['before']['free_directions'] == 1
    assert result['after']['free_directions'] == 2
    assert result['migration']['max_scaled_coefficient_difference_from_xml_m'] < 1e-8
    assert result['minimum_independent_span_m'] >= 6.
    assert result['written_layout']['predicted_extra_width_records'] == 2
    assert result['written_layout']['newly_split_min_record_m'] > 7.
    assert result['excluded_preceding_midpoint']['record_check']['newly_split_min_record_m'] < 1.3
    assert result['target_feasibility'] == 'NOT_EVALUATED'
    assert not result['new_xodr'] and not result['implementation_approved']
    assert result['nonzero_edited_states'] == 0
    np.testing.assert_array_equal(control.current, prior)
