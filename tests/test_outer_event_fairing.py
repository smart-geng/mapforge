"""Actual geometry derivatives, exact intent and bounded failure semantics."""
import numpy as np
import pytest

from mapforge.repair_web.outer_event_fairing import CurvatureFairing, world_geometry, critical_parameters
from scripts.check_outer_event_control import inputs


@pytest.fixture(scope='module')
def fairing():
    return CurvatureFairing(inputs()[1])


def test_world_jacobian_matches_central_difference():
    jets = np.array([[.3,.02,-.005], [-.12,-.04,.003], [0.,.05,0.]])
    value, jac = world_geometry(jets)
    for col in range(3):
        d = np.zeros_like(jets); d[:,col]=1e-6
        fd = (world_geometry(jets+d)[0]-world_geometry(jets-d)[0])/2e-6
        assert jac[:,:,col] == pytest.approx(fd, abs=1e-9)
    assert np.isfinite(value).all()


def test_common_state_variation_gradient(fairing):
    # Feasible-subspace perturbations preserve join C2. This check is away
    # from active extrema/tie switches; such switches remain nonsmooth.
    rng = np.random.default_rng(722)
    x = fairing.reference+fairing.event.Z@rng.normal(0,.01,fairing.event.Z.shape[1])
    _, jac = fairing.total_variation(x)
    for _ in range(5):
        d = fairing.event.Z@rng.normal(0,.2,fairing.event.Z.shape[1])
        fd = (fairing.total_variation(x+1e-6*d)[0]-fairing.total_variation(x-1e-6*d)[0])/2e-6
        assert jac@d == pytest.approx(fd, abs=1e-7)


def test_reference_calibrates_continuous_caps(fairing):
    peaks, _ = fairing.continuous(fairing.reference)
    variation, _ = fairing.total_variation(fairing.reference)
    assert peaks == pytest.approx(fairing.caps[:,:2], abs=1e-9)
    assert variation == pytest.approx(fairing.caps[:,2], abs=1e-8)
    value, _, lo, hi = fairing.nonlinear(fairing.reference)
    assert max(0.,max(lo-value),max(value-hi)) < 1e-7


def test_known_interior_curvature_extremum():
    ks, rates = critical_parameters([0.,-.1,.01,0.],20.)
    assert any(abs(u-.25)<1e-8 for u in ks)
    assert ks[0]==rates[0]==0. and ks[-1]==rates[-1]==1.


@pytest.mark.parametrize('value', [True,None,float('nan'),3.])
def test_invalid_target_cannot_start_solver(fairing,value):
    with pytest.raises(ValueError): fairing.solve(value)


def test_tiny_wall_budget_returns_no_candidate(fairing):
    x,report=fairing.solve(-.125,max_seconds=1e-12)
    assert x is None and report['status']=='TIME_BUDGET_EXHAUSTED'
    assert not report['map_accepted'] and report['actual_written_m'] is None


def test_reference_noop_is_position_exact_and_minimum_change(fairing):
    x,report=fairing.solve(0.,max_iterations=5)
    assert x is not None
    assert fairing.control.handle@x-fairing.control.reference_t == pytest.approx(0.,abs=1e-8)
    assert np.linalg.norm(fairing.M@(x-fairing.reference))<1e-7
    assert report['free_with_hard_target']==23 and report['new_curve_knots']==0
    assert not report['map_accepted']


def test_continuation_does_not_publish_partial_or_use_failed_seed(fairing,monkeypatch):
    calls=[]
    def fail(target,**kw):
        calls.append(target)
        return None,dict(status='test-rejected',iterations=40)
    monkeypatch.setattr(fairing,'solve',fail)
    x,report=fairing.continuation(-.12)
    assert x is None and calls==[-.03]
    assert report['iterations']==40 and not report['intermediates_published']
    assert not report['map_accepted'] and not report['exact_final_target_met']


def test_rejected_state_cannot_be_a_continuation_seed(fairing):
    bad=fairing.reference.copy();bad[0]+=.1
    with pytest.raises(ValueError):fairing.solve(-.01,start_state=bad)


def test_minimum_change_objective_is_positive(fairing):
    reduced=fairing.M@fairing.event.Z
    assert min(np.linalg.eigvalsh(reduced.T@reduced))>0
