import numpy as np
import pytest
from mapforge.ops.long_connector_chain import chain, expand_core, polish_endpoint
from spikes.measured_connector_caps import chain as original
from mapforge.ops.source_connector_ribbon import reference_samples


def frames():
    return dict(pose=(1.,2.,.2),k=0.,dk=0.),dict(pose=(20.,22.,1.2),k=0.,dk=0.)


def test_three_core_is_backward_identical():
    a,b=frames();q=[6.,16.,8.,6.,2.,1.]
    new=chain(q,a,b,(True,False));old=original(np.asarray(q),a,b,(True,False))
    assert [(c.length,c.KappaStart,c.dk,c.XEnd,c.YEnd) for c in new]==[(c.length,c.KappaStart,c.dk,c.XEnd,c.YEnd) for c in old]


def test_one_extra_long_interval_preserves_entire_seed_curve():
    a,b=frames();q=[6.,16.,8.,6.,2.,1.];caps=(True,False)
    old=chain(q,a,b,caps);expanded=expand_core(q,a,b,caps);new=chain(expanded,a,b,caps,4)
    assert len(new)==5 and min(c.length for c in new)>=6
    s=np.linspace(0,sum(c.length for c in old),1001)
    assert np.max(abs(reference_samples(old,s)[0]-reference_samples(new,s)[0]))<1e-11
    # Closure target is frozen to the exact seed for this basis-transfer test.
    b=dict(b,pose=(old[-1].XEnd,old[-1].YEnd,old[-1].ThetaEnd))
    polished,report=polish_endpoint(expanded,a,b,caps,4)
    assert max(abs(np.asarray(report['after'])))<1e-9
    assert np.max(abs(polished-expanded))<1e-8


def test_short_or_excessive_reference_expansion_rejected():
    a,b=frames()
    with pytest.raises(ValueError,match='no core'):expand_core([6.,8.,8.,6.,2.,1.],a,b,(True,False))
    with pytest.raises(ValueError,match='budget'):expand_core([6.,16.,8.,6.,6.,2.,1.],a,b,(True,True))
    with pytest.raises(ValueError,match='at most five'):chain([1.]*9,a,b,(True,True),4)


def test_two_core_with_two_caps_is_a_continuous_four_long_primitive_family():
    a,b=frames();caps=(True,True);q=np.array([6.,9.,8.,6.,-2.])
    cls=chain(q,a,b,caps,2)
    assert len(cls)==4 and min(c.length for c in cls)>=6
    assert cls[0].dk==0 and cls[-1].dk==0
    for u,v in zip(cls[:-1],cls[1:]):
        assert abs(u.KappaEnd-v.KappaStart)<1e-12
        assert abs(u.XEnd-v.XStart)+abs(u.YEnd-v.YStart)<1e-12
        assert abs(u.ThetaEnd-v.ThetaStart)<1e-12
    end=cls[-1];b=dict(b,pose=(end.XEnd,end.YEnd,end.ThetaEnd))
    chosen,report=polish_endpoint(q,a,b,caps,2)
    assert max(abs(np.array(report['after'])))<1e-8 and np.max(abs(chosen-q))<1e-8
    with pytest.raises(ValueError,match='at least three'):chain([8,8,1],a,b,(False,False),2)
