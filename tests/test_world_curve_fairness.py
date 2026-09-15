import numpy as np
import pytest
from pyclothoids import Clothoid
from scipy.integrate import trapezoid
from mapforge.ops.world_curve_fairness import interval_fairness, ribbon_fairness, source_turn_evidence


def test_constant_offset_arc_has_zero_curvature_variation():
    row=interval_fairness(.03,0.,[2.,0.,0.,0.],20.)
    assert row['curvature_variation_energy']==pytest.approx(0.,abs=1e-25)
    assert row['negative_turn_deg']==pytest.approx(0.,abs=1e-12)
    assert row['positive_turn_deg']==pytest.approx(np.degrees(.6))
    assert row['bending_energy']==pytest.approx((.03/(1-.03*2))**2*20*(1-.03*2))


def test_cubic_world_edge_can_reverse_with_perfectly_straight_reference():
    co=np.array([0.,.3,-.03,.002/3]); L=30.
    row=interval_fairness(0.,0.,co,L,order=32)
    s=np.linspace(0,L,100001); slope=co[1]+2*co[2]*s+3*co[3]*s*s
    heading=np.arctan(slope); diff=np.diff(heading)
    assert row['negative_turn_deg']>20.
    assert row['negative_turn_deg']==pytest.approx(np.degrees(np.maximum(-diff,0).sum()),abs=1e-8)
    assert row['positive_turn_deg']==pytest.approx(np.degrees(np.maximum(diff,0).sum()),abs=1e-8)
    assert row['curvature_variation_energy']>0


def test_spiral_energy_uses_world_arc_length():
    # t=constant: k_world=k/(1-k*t), d/d lane_s=k'/(1-k*t)^3.
    k=.01; dk=.001; t=1.5; L=12.
    row=interval_fairness(k,dk,[t,0,0,0],L,order=32)
    s=np.linspace(0,L,100001); a=1-(k+dk*s)*t
    expected=trapezoid(dk**2/a**5,s)
    assert row['curvature_variation_energy']==pytest.approx(expected,rel=1e-9)


def test_source_s_bend_is_not_forced_to_one_turn_sign():
    s=np.linspace(0,30,121)
    points=np.c_[s,3*np.sin(s/30*2*np.pi)]
    ev=source_turn_evidence({'center':points})['center']
    assert not ev['single_turn']
    assert ev['reverse_turn_deg']>1.


def test_evaluation_split_does_not_add_records_or_change_energy():
    cls=[Clothoid.StandardParams(0,0,0,.01,.001,10)]
    raw={k:np.array([[0.,0.],[5.,2.],[8.,6.]]) for k in ('left','right','center')}
    ev=source_turn_evidence(raw)
    co={'left':np.array([[1.,0.,0.,0.]]),'right':np.array([[-1.,0.,0.,0.]])}
    before={k:v.copy() for k,v in co.items()}
    whole=ribbon_fairness(cls,[0.,10.],co,ev)
    split=ribbon_fairness(cls,[0.,5.,10.],{k:np.repeat(v,2,axis=0) for k,v in co.items()},ev)
    for field in whole:
        assert whole[field]['curvature_variation_energy']==pytest.approx(split[field]['curvature_variation_energy'])
    assert all(np.array_equal(co[k],v) for k,v in before.items())
    assert len(cls)==1


def test_fold_is_not_given_an_angle_certificate():
    row=interval_fairness(1.,0.,[2.,0.,0.,0.],3.)
    assert not row['regular']
    assert row['negative_turn_deg']>=1e5


def test_polynomial_turn_partition_matches_independent_dense_headings():
    rng=np.random.default_rng(17)
    for _ in range(20):
        L=float(rng.uniform(3,12)); k=float(rng.uniform(-.03,.03)); dk=float(rng.uniform(-.001,.001))
        c=np.array([rng.uniform(-2,2),rng.uniform(-.3,.3),rng.uniform(-.04,.04),rng.uniform(-.002,.002)])
        item=interval_fairness(k,dk,c,L,order=32)
        if not item['regular']:continue
        s=np.linspace(0,L,20001);t=c[0]+s*(c[1]+s*(c[2]+s*c[3]));dt=c[1]+s*(2*c[2]+s*3*c[3])
        h=k*s+.5*dk*s*s+np.arctan2(dt,1-(k+dk*s)*t);dh=np.diff(h)
        for label,sign in [('positive',1),('negative',-1)]:
            assert item[label+'_turn_deg']==pytest.approx(np.degrees(np.maximum(sign*dh,0).sum()),abs=2e-6)


@pytest.mark.parametrize('args',[(0.,0.,[1,2,3],4.),(0.,0.,[0,0,0,0],0.),(float('nan'),0.,[0,0,0,0],3.)])
def test_invalid_intervals_rejected(args):
    with pytest.raises(ValueError):interval_fairness(*args)


def test_joint_fairing_is_executed_even_when_the_seed_is_already_feasible(monkeypatch):
    from types import SimpleNamespace
    import xml.etree.ElementTree as ET
    from tests.test_joint_connector_restore import truth
    import mapforge.ops.joint_connector_fit as joint
    road,a,b,raw,seed=truth();before=ET.tostring(road);called=[]
    def optimizer(fun,x,**options):
        called.append(fun(x))
        assert max(abs(options['constraints'][0]['fun'](x)))<1e-6
        assert min(options['constraints'][1]['fun'](x))>=-1e-5
        return SimpleNamespace(x=x,success=True,message='test unchanged feasible seed',nit=0)
    monkeypatch.setattr(joint,'minimize',optimizer)
    candidate,row=joint.fit_joint(road,a,b,raw,raw,seed['shape_parameters'],
        initial_coefficients=seed['joint_coefficients'],fair_world=True,max_iterations=1)
    assert called and ET.tostring(road)==before
    assert row['world_fairing_enabled'] and row['initial_coefficients_supplied']
    assert row['added_reverse_turn_construction_budget_deg']==.5
    assert row['status']=='GEOMETRY_REVIEW_CANDIDATE' and not row['production_accepted']
    assert all(r['additional_reverse_turn_deg']<=.5 for r in row['world_fairness'].values())
    assert row['source_turn_budget_scope']=='all-three-original-fields'
    assert row['endpoint_jets_eliminated']
    assert row['independent_optimization_variables']==row['joint_variables']-12
    assert len(candidate.findall('planView/geometry'))==len(road.findall('planView/geometry'))
