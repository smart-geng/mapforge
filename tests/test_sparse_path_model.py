import numpy as np
import pytest
from pyclothoids import Clothoid

from spikes.sparse_path_model import Limits,chain,fit,observations,sample,verify


def test_shared_curvature_nodes_produce_exact_g2_without_tiny_pieces():
    z=np.r_[1.,2.,.02,[30.,40.,30.],[0.,.5,-.3,0.],0.]
    cc=chain(z,3,np.array([10.,20.]),.4)
    for a,b in zip(cc,cc[1:]):
        assert a.XEnd==b.XStart and a.YEnd==b.YStart
        assert a.ThetaEnd==b.ThetaStart
        assert a.KappaEnd==pytest.approx(b.KappaStart,abs=1e-15)
    limits=Limits(10.,source_m=.01)
    report=verify(cc,sample(cc,.05),limits)
    assert report['status']=='PATH_CANDIDATE'
    assert report['jerk_mps3']==pytest.approx(.2)
    assert report['lengths_m']==[30.,40.,30.]


def test_analytic_dynamics_catches_exactly_fitted_but_unsafe_path():
    c=Clothoid.StandardParams(0.,0.,0.,0.,.002,30.)
    raw=sample([c],.1)
    r=verify([c],raw,Limits(10.,source_m=.01))
    assert r['source_to_curve_max_m']<1e-4
    assert r['status']=='REJECTED' and r['jerk_mps3']==pytest.approx(2.)
    assert r['acceleration_mps2']==pytest.approx(6.)


def test_one_long_straight_is_first_choice_and_raw_array_is_immutable():
    raw=np.array([[10.,-2.],[25.,-2.],[70.,-2.],[110.,-2.]])
    before=raw.copy();curves,result=fit(raw,Limits(60/3.6),counts=(1,))
    assert result['status']=='PATH_CANDIDATE'
    assert len(curves)==1 and curves[0].length==pytest.approx(100.,abs=1e-5)
    assert result['trials'][0]['source_to_curve_max_m']<1e-5
    assert result['trials'][0]['model_kind']=='line'
    assert curves[0].KappaStart==0. and curves[0].dk==0.
    assert len(result['trials'])==1
    np.testing.assert_array_equal(raw,before)


def test_sampling_keeps_every_nonuniform_raw_vertex_and_endpoints():
    raw=np.array([[0.,0.],[2.03,.4],[8.,0.]])
    obs,_=observations(raw,1.)
    for p in raw:assert min(np.linalg.norm(obs-p,axis=1))<1e-12
    with pytest.raises(ValueError):observations(raw,0)
    with pytest.raises(ValueError):observations([[0,0],[0,0],[1,0]],1.)


def test_solver_does_not_fall_back_to_fragment_chain_for_short_path():
    raw=np.array([[0.,0.],[6.,1.],[10.,0.]])
    cc,r=fit(raw,Limits(10.),counts=(3,))
    assert cc is None and r['status']=='REJECTED'
    assert r['trials'][0]['reason']=='insufficient length for span budget'
    with pytest.raises(ValueError):fit(raw,Limits(10.),counts=(1,3,5,9))


def test_endpoint_coverage_prevents_fitting_only_an_interior_piece():
    c=Clothoid.StandardParams(0,0,0,0,0,20)
    r=verify([c],np.array([[-20.,0.],[0.,0.],[20.,0.],[40.,0.]]),Limits(10.))
    assert r['status']=='REJECTED'
    assert r['endpoint_max_m']==pytest.approx(20.)


def test_invalid_limits_are_not_silently_substituted():
    for limits in (Limits(0),Limits(float('nan')),Limits(10,minimum_span_m=-1)):
        with pytest.raises(ValueError):limits.check()


def test_lower_trial_speed_is_never_used_for_acceptance(monkeypatch):
    from types import SimpleNamespace
    import spikes.sparse_path_model as model
    c=Clothoid.StandardParams(0.,0.,0.,0.,.002,30.)
    raw=sample([c],.1)
    chord=np.arctan2(c.YEnd,c.XEnd)
    answer=SimpleNamespace(x=np.array([0.,0.,-chord,30.,0.,6.,5.]),
                           success=True,message='injected lower-speed feasible geometry',nit=1)
    monkeypatch.setattr(model,'minimize',lambda *args,**kwargs:answer)
    cc,result=fit(raw,Limits(10.),counts=(1,),speed_frontier=True)
    assert cc is None and result['status']=='REJECTED'
    report=result['trials'][-1]
    assert report['tube_certified']
    assert report['requested_speed_kmh']==36.
    assert report['supported_speed_kmh']<36.
    assert report['jerk_mps3']==pytest.approx(2.)


def test_verifier_rejects_disconnected_pieces_even_inside_source_tube():
    cc=[Clothoid.StandardParams(0.,0.,0.,0.,0.,10.),
        Clothoid.StandardParams(10.01,0.,0.,0.,0.,10.)]
    report=verify(cc,np.array([[0.,0.],[20.01,0.]]),Limits(10.))
    assert report['source_to_curve_max_m']<.01
    assert not report['g2_connected'] and report['status']=='REJECTED'


def test_verifier_bounds_between_samples_and_keeps_polyline_scope_explicit():
    c=Clothoid.StandardParams(0.,.345,0.,0.,0.,20.)
    report=verify([c],np.array([[0.,0.],[20.,0.]]),Limits(10.))
    assert report['tube_certified']
    assert report['certificate_step_m']<.01
    assert .345 < report['source_tube_upper_bound_m'] < .35
    assert report['primitive_count']==1
