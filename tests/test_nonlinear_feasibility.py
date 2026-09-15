import numpy as np
from spikes.nonlinear_feasibility import restore


def test_restoration_keeps_hard_constraints_and_removes_all_elastic_slack():
    # x^2 <= 1 from x=3, while y=2 and x>=.5 remain hard throughout.
    x, report = restore(np.array([3., 2.]), np.array([[0., 1.]]), np.array([2.]),
                        np.array([[1., 0.]]), np.array([.5]),
                        lambda v: np.array([v[0]**2]), lambda v: np.array([[2*v[0], 0.]]))
    assert report['status'] == 'RESTORED'
    assert x[0] >= .5 and x[0]**2 <= 1.+1e-6
    assert abs(x[1]-2.) < 1e-8
    assert all(r.get('hard_residual', 0.) < 1e-6 for r in report['iterations'])


def test_slack_is_not_reported_as_a_feasible_geometry():
    x, report = restore(np.array([3.]), np.array([[1.]]), np.array([3.]),
                        np.array([[1.]]), np.array([0.]),
                        lambda v: v**2, lambda v: np.array([[2*v[0]]]), max_iter=20)
    assert x is None and report['status'] == 'REJECTED'
    assert report['best_exact_ratio'] == 9.


def test_invalid_initial_geometry_does_not_get_elastic_source_constraints():
    x, report = restore(np.array([0.]), np.empty((0, 1)), np.empty(0),
                        np.array([[1.]]), np.array([1.]), lambda v: v, lambda v: np.eye(1))
    assert x is None and report['reason'] == 'initial hard constraints violated'
