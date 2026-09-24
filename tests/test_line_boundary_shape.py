import numpy as np
import pytest

from mapforge.validate.line_boundary_shape import span_shape, written_boundary_shape
from scripts.check_outer_event_control import inputs, REFERENCE


def test_line_has_zero_curvature_and_variation():
    r = span_shape([2., .1, 0., 0.], 50.)
    assert r['max_abs_curvature'] == r['max_abs_curvature_rate'] == r['curvature_variation'] == 0.


@pytest.mark.parametrize('c,L', [([0., 0., .1, 0.], 10.), ([0., .2, -.03, .001], 22.)])
def test_critical_points_cover_dense_independent_kinematics(c, L):
    from spikes.road_boundary_family import world_kinematics
    s = np.linspace(0., L, 30001)
    jets = np.stack([np.polynomial.polynomial.polyval(s, np.polynomial.polynomial.polyder(c, j))
                     for j in range(4)], axis=-1)
    observed = world_kinematics(jets, 0., 0.)
    r = span_shape(c, L)
    assert r['max_abs_curvature'] >= max(abs(observed[:, 0]))-1e-10
    assert r['max_abs_curvature_rate'] >= max(abs(observed[:, 1]))-1e-10
    assert r['max_abs_curvature'] == pytest.approx(max(abs(observed[:, 0])), abs=1e-7)
    assert r['curvature_variation'] == pytest.approx(sum(abs(np.diff(observed[:, 0]))), abs=1e-7)


@pytest.mark.parametrize('c,L', [([0., 0., 0., float('nan')], 1.), ([0., 0., 0., 0.], 0.)])
def test_invalid_cubic_not_pass(c, L):
    with pytest.raises(ValueError): span_shape(c, L)


def test_all_real_edges_read_actual_xml():
    event, _ = inputs()
    r = written_boundary_shape(REFERENCE.read_bytes(), event.scope.__dict__)
    assert set(r['by_edge']) == {'0', '1', '2', '3', '4'}
    assert all(e['intervals'] > 0 and e['join_jump_sum'] < 1e-8 for e in r['by_edge'].values())
    assert not r['map_accepted'] and not r['formal_certificate']
