import numpy as np
import pytest

from spikes.road_boundary_family import world_kinematics
from spikes.shared_boundary_dynamics import graph_kinematics_and_gradient,refine_dynamics,verify_refinement
from spikes.source_contact_fit import SourceBoundaryBlock
from tests.test_source_contact_fit import straight_fixture


def test_analytic_derivatives_match_world_formula_and_finite_difference():
    jets=np.array([[1.,.2,.03,-.004],[-2.,-.1,-.01,.003]])
    values,grad=graph_kinematics_and_gradient(jets)
    assert values==pytest.approx(world_kinematics(jets,0.,0.),abs=1e-12)
    for j in range(4):
        a=jets.copy();b=jets.copy();a[:,j]+=1e-6;b[:,j]-=1e-6
        numeric=(graph_kinematics_and_gradient(a)[0]-graph_kinematics_and_gradient(b)[0])/2e-6
        assert grad[:,:,j]==pytest.approx(numeric,abs=1e-9)


def test_straight_shared_state_needs_no_change_but_is_not_map_acceptance():
    model=SourceBoundaryBlock(*straight_fixture(),degree=5);x,_=model.solve()
    new,report=refine_dynamics(model,x,iterations=1)
    assert new==pytest.approx(x)
    assert report['status']=='SAMPLED_TARGET_REACHED' and report['history']==[]
    assert not report['export_allowed'] and not report['full_curve_dynamics_certified']
    assert not report['source_speed_changed'] and not report['source_tolerance_relaxed']
    verify_refinement(model,x,new,report)
    report['final_max_ratio']=1.
    with pytest.raises(ValueError,match='metrics'):verify_refinement(model,x,new,report)


@pytest.mark.parametrize('fault',['initial','speed','limits'])
def test_optimization_may_not_trade_source_or_speed_for_dynamics(fault):
    model=SourceBoundaryBlock(*straight_fixture());x,_=model.solve();kwargs={}
    if fault=='initial':x+=2
    if fault=='speed':model.scope['observations']['a']['source_max_speed_kmh']=None
    if fault=='limits':kwargs['limits']=(3.,2.)
    with pytest.raises(ValueError):refine_dynamics(model,x,**kwargs)


def test_failed_secondary_fairing_never_accepts_an_arbitrary_minimax_lp_vertex(monkeypatch):
    model=SourceBoundaryBlock(*straight_fixture(),degree=5);x,_=model.solve()
    for family in model.families:
        x[family.columns]+=.2*np.sin(np.linspace(0,4*np.pi,family.columns.stop-family.columns.start))
    monkeypatch.setattr('spikes.shared_boundary_dynamics.interior_qp',lambda *a:(None,{'status':'simulated failure'}))
    new,report=refine_dynamics(model,x,iterations=1)
    assert report['initial_max_ratio']>1
    assert new==pytest.approx(x)
    assert report['history'][0]['reason']=='no validated least-unfair tie solution'
    assert not report['history'][0]['accepted']
    verify_refinement(model,x,new,report)
