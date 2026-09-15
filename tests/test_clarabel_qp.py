import numpy as np
import pytest

pytest.importorskip('clarabel')
from spikes.clarabel_joint_candidate import interior_qp


def test_independent_qp_uses_original_equality_and_inequality_units():
    x, info = interior_qp(np.eye(2), np.array([3., -3.]), np.array([[1., 1.]]), np.array([1.]),
                          np.eye(2), np.zeros(2), np.array([.5, .5]))
    np.testing.assert_allclose(x, [1., 0.], atol=1e-7)
    assert info['original_inequality_residual'] < 1e-6
    assert info['original_equality_residual'] < 1e-6


def test_infeasible_qp_is_not_an_approximate_pass():
    x, info = interior_qp(np.eye(2), np.ones(2), np.array([[1., 1.]]), np.array([1.]),
                          np.eye(2), np.ones(2), np.zeros(2))
    assert x is None


def test_bad_fixed_equalities_do_not_pass():
    x, info = interior_qp(np.eye(1), np.ones(1), np.array([[1.], [1.]]), np.array([0., 2.]),
                          np.eye(1), np.zeros(1), np.zeros(1))
    assert x is None
