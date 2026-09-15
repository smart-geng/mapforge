from types import SimpleNamespace

import numpy as np
import pytest
from pyclothoids import Clothoid

from mapforge.ops.continuous_offset_dynamics import (
    audit_span, bernstein_controls, split_controls, turn_spans, parent_spans, _world_bounds, Interval)
from mapforge.ops.whole_source_dynamics import source_speed_binding
from spikes.road_boundary_family import world_kinematics


@pytest.mark.parametrize('k', [0.,.001,-.001])
def test_constant_offset_matches_world_radius_and_no_lateral_rate(k):
    r=audit_span([3.,0.,0.,0.],80.,k,0.,60.)
    assert r['status']=='BOUNDED' and r['all_parameter_intervals_accounted']
    assert r['observed_maxima'][0]['value']==pytest.approx((60/3.6)**2*abs(k/(1-3*k)))
    assert r['observed_maxima'][1]['value']==0.
    assert not r['formal_source_certificate'] and not r['export_allowed']


def test_actual_spiral_lane_jerk_uses_lane_not_reference_arclength():
    r=audit_span([3.,0.,0.,0.],30.,.001,1e-5,60.)
    assert r['status']=='BOUNDED'
    assert r['observed_maxima'][1]['value']==pytest.approx((60/3.6)**3*1e-5/(1-3*.0013)**3)


def test_interior_peak_is_not_accepted_from_two_endpoints():
    co=[0.,3.,-.45,.015]
    jets=np.array([[np.polynomial.polynomial.polyval(s,np.polynomial.polynomial.polyder(co,j))
                    for j in range(4)] for s in (0.,20.)])
    assert np.all(abs(world_kinematics(jets,0.,0.))*[(19/3.6)**2,(19/3.6)**3]<[2.5,1.])
    r=audit_span(co,20.,0.,0.,19.)
    assert r['status']=='FAIL'
    assert any(0<x['witness_u']<1 for x in r['interval_leaves'] if x['status']=='FAIL')
    assert r['geometry_segments_added']==0


def test_depth_exhaustion_and_tiny_source_domain_cannot_be_pass():
    r=audit_span([0.,1.,-.15,.005],20.,0.,0.,5.,max_depth=0)
    assert r['status']=='UNKNOWN'  # Samples pass, coarse bound is inconclusive.
    refined=audit_span([0.,1.,-.15,.005],20.,0.,0.,5.)
    assert refined['status']=='BOUNDED' and refined['geometry_segments_added']==0
    tiny=audit_span([0.,0.,0.,0.],1e-9,0.,0.,60.)
    assert tiny['status']=='UNKNOWN' and tiny['all_parameter_intervals_accounted']


def test_nonregular_curve_is_explicit_failure_not_nan_pass():
    r=audit_span([100.,0.,0.,0.],10.,.01,0.,60.)
    assert r['status']=='FAIL'
    assert r['interval_leaves'][0]['reason']=='nonregular-world-lane'


@pytest.mark.parametrize('speed', [0.,-1.,np.nan,np.inf])
def test_missing_source_speed_is_not_filled_in(speed):
    with pytest.raises(ValueError):audit_span([0.,0.,0.,0.],10.,0.,0.,speed)


def test_bound_contains_independent_world_formula_for_random_offset_jets():
    rng=np.random.default_rng(548)
    for _ in range(30):
        jets=np.array([rng.uniform(-5,5),rng.uniform(-.03,.03),rng.uniform(-.001,.001),rng.uniform(-1e-4,1e-4)])
        wiggle=np.array([.1,.002,.0001,1e-5])
        intervals=[Interval(a-b,a+b) for a,b in zip(jets,wiggle)]
        k,kp=.002,1e-5
        _,bk,bdk=_world_bounds(intervals,Interval.point(k),Interval.point(kp))
        samples=jets+rng.uniform(-1,1,(500,4))*wiggle
        values=world_kinematics(samples,k,kp)
        assert np.all(values[:,0]>=bk.lo) and np.all(values[:,0]<=bk.hi)
        assert np.all(values[:,1]>=bdk.lo) and np.all(values[:,1]<=bdk.hi)


def test_bernstein_subdivision_preserves_original_polynomial_no_new_geometry():
    from scipy.interpolate import BPoly
    power=np.array([1.,-.2,.3,-.1]);c,_=bernstein_controls(power)
    a,b=split_controls(c);poly=BPoly(np.c_[a,b],[0.,.5,1.])
    ss=np.linspace(0,1,400)
    np.testing.assert_allclose(poly(ss),np.polynomial.polynomial.polyval(ss,power),atol=1e-14)


def test_turn_reference_and_width_join_both_sides_are_retained():
    c1=Clothoid.StandardParams(0.,0.,0.,0.,.001,6.)
    c2=Clothoid.StandardParams(c1.XEnd,c1.YEnd,c1.ThetaEnd,c1.KappaEnd,-.001,6.)
    co=dict(left=np.array([[2.,0.,0.,.01],[2.64,.48,.12,0.]]),right=np.zeros((2,4)))
    rows=list(turn_spans(dict(result=([c1,c2],[0.,4.,12.],co))))
    assert [(r['start'],r['end']) for r in rows]==[(0.,4.),(4.,6.),(6.,12.)]
    assert [r['sharpness'] for r in rows]==pytest.approx([.001,.001,-.001])
    assert rows[0]['coefficients'][3]==pytest.approx(.005)
    assert rows[1]['coefficients'][3]==0.
    assert rows[2]['curvature']==pytest.approx(.006)


def test_mixed_original_speeds_are_not_misreported_as_real_interval_mapping():
    p=SimpleNamespace(source_ids=['a','b'])
    binding=source_speed_binding(p,{'a':{'source_max_speed_kmh':40},'b':{'source_max_speed_kmh':60}})
    assert binding['evaluation_speed_kmh']==60
    assert not binding['source_interval_mapping_complete']
    assert binding['policy']=='maximum-original-limit-envelope-only'
    assert not binding['approved_movement_design_speed']
    assert not binding['source_speed_changed']
    assert [r['source_speed_kmh'] for r in binding['rows']]==[40.,60.]


def test_parent_midpoint_polynomials_reproduce_shared_original_basis_on_each_open_span():
    from tests.test_movable_source_layout import block
    _,model=block();rng=np.random.default_rng(711)
    x=rng.uniform(-.1,.1,model.nvar)
    before=x.copy();rows=list(parent_spans(dict(model=model,coefficients=x)))
    assert rows and {s for r in rows for s in r['identity']['source_lanes']}=={'a','b'}
    domains={sid:pair for sid,pair in model.lane_pairs.items()}
    for r in rows:
        left,right=domains[r['identity']['source_lane']][:2]
        for u in (.01,.3,.7,.99):
            ds=u*(r['end']-r['start']);s=r['start']+ds
            for j in range(4):
                direct=.5*(model.expression(left,s,j)+model.expression(right,s,j))@x
                rebuilt=np.polynomial.polynomial.polyval(ds,np.polynomial.polynomial.polyder(r['coefficients'],j))
                assert direct==pytest.approx(rebuilt,abs=1e-11)
    np.testing.assert_array_equal(x,before)
