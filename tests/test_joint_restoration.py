import numpy as np
import pytest

from spikes.nonlinear_feasibility import restore_joint


def equation(x):
    return np.array([x[0]**2-1.,1.-x[0]**2])


def jacobian(x):
    return np.array([[2*x[0]],[-2*x[0]]])


def args(initial=.2):
    return (np.array([initial]),np.empty((0,1)),np.empty(0),np.array([[1.]]),np.array([.1]),
            equation,jacobian,np.eye(1),np.ones(1))


def test_joint_restoration_closes_exact_nonlinear_equation():
    a=args();before=a[0].copy()
    result,report=restore_joint(*a,radii=np.array([.4]))
    assert report['status']=='RESTORED',report
    assert result[0]==pytest.approx(1.,abs=1e-7)
    assert all(i['hard_violation']<=1e-7 for i in report['iterations'] if i['accepted'])
    assert np.array_equal(a[0],before)


def test_nonzero_restoration_slack_never_returns_a_writable_candidate():
    a=list(args());a[3]=np.array([[1.],[-1.]]);a[4]=np.array([.1,-.2])
    result,report=restore_joint(*a,radii=np.array([.4]),max_iter=3)
    assert result is None and report['status']=='REJECTED'
    assert report['best_exact_violation']>.9


def test_fair_qp_failure_does_not_accept_the_lp_vertex(monkeypatch):
    monkeypatch.setattr('spikes.clarabel_joint_candidate.interior_qp',lambda *a:(None,{'status':'test failure'}))
    result,report=restore_joint(*args(),radii=np.array([.4]))
    assert result is None
    assert 'LP vertex not accepted' in report['reason']


def test_inconsistent_hard_source_remains_rejected():
    a=list(args());a[3]=np.array([[1.],[-1.]]);a[4]=np.array([1.,0.])
    result,report=restore_joint(*a,radii=np.array([.4]))
    assert result is None and report['status'] == 'REJECTED'
    assert 'hard-bound' in report['reason']


def test_non_finite_or_stationary_residual_cannot_claim_global_infeasibility():
    a=list(args());a[5]=lambda x:np.array([np.nan])
    result,report=restore_joint(*a,radii=np.array([.4]))
    assert result is None and report['global_infeasibility_claimed'] is False


def test_restoration_option_does_not_allow_independent_road_writes():
    from scripts.rebuild_map_joint import geometry_stage
    with pytest.raises(ValueError,match='requires coupled'):geometry_stage(None,None,restore=True)


def test_nonlinear_source_is_checked_after_every_restoration_step():
    # Linearized x*x <= .25 cannot authorize accepting x=.8 from x=.2.
    result,report=restore_joint(*args(),radii=np.array([.8]),max_iter=5,
        hard_nonlinear=lambda x:np.array([x[0]**2-.25]),
        hard_jacobian=lambda x:np.array([[2*x[0]]]))
    assert result is None
    assert any(i['accepted'] for i in report['iterations'])
    assert all(i['hard_violation']<=1e-7 for i in report['iterations'] if i['accepted'])
