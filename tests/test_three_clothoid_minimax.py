import math
import numpy as np
import pytest
from scipy.optimize import OptimizeResult
from spikes.three_clothoid_minimax import optimize


@pytest.mark.parametrize('end',[(25.,25.,math.pi/2),(14.,10.,math.pi/2),
                                (25.,2.,0.),(25.,-25.,-math.pi/2)])
def test_three_long_primitives_preserve_endpoints_and_do_not_worsen(end):
    cls,report=optimize((0.,0.,0.),end)
    assert len(cls)==3
    assert np.allclose([cls[-1].XEnd,cls[-1].YEnd,cls[-1].ThetaEnd],end,atol=2e-6)
    assert abs(cls[0].KappaStart)<1e-10 and abs(cls[-1].KappaEnd)<1e-10
    for a,b in zip(cls,cls[1:]):
        assert np.allclose([a.XEnd,a.YEnd,a.ThetaEnd,a.KappaEnd],
                           [b.XStart,b.YStart,b.ThetaStart,b.KappaStart],atol=1e-9)
    assert report['dynamic_ratio']<=report['baseline_dynamic_ratio']+1e-7
    assert report['selected']['min_primitive_m']>=5.-1e-7
    assert report['source_speed_changed'] is False
    assert report['global_optimality_claimed'] is False


def test_tight_corner_is_not_accepted_by_lowering_speed():
    _,r=optimize((0.,0.,0.),(10.,10.,math.pi/2))
    assert r['evaluation_speed_kmh']==15.
    assert r['dynamic_ratio']>1.
    assert r['evaluation_status']=='FAIL'


def test_solver_failure_preserves_seed_not_unqualified_parameters(monkeypatch):
    import spikes.three_clothoid_minimax as m
    monkeypatch.setattr(m,'minimize',lambda f,x,**kw:OptimizeResult(
        success=False,x=x*2,message='test failure'))
    cls,r=optimize((0.,0.,0.),(25.,25.,math.pi/2))
    assert r['optimized'] is False
    assert r['dynamic_ratio']==r['baseline_dynamic_ratio']
    assert np.allclose([cls[-1].XEnd,cls[-1].YEnd],[25.,25.])


def test_invalid_speed_is_not_treated_as_missing():
    with pytest.raises(ValueError):optimize((0.,0.,0.),(20.,20.,math.pi/2),speed_kmh=0.)


def test_ordinary_turn_does_not_buy_lower_jerk_with_countersteering():
    cls,r=optimize((0.,0.,0.),(14.,10.,math.pi/2))
    assert r['monotone_turn_required'] is True
    assert r['shape_constraints_satisfied'] is True
    assert min(c.KappaStart for c in cls)>=-1e-9
    assert min(c.KappaEnd for c in cls)>=-1e-9
