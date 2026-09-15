import numpy as np
import pytest
from mapforge.ops.long_source_seed import initialize
from mapforge.ops.long_connector_chain import chain
from mapforge.ops.source_connector_ribbon import reference_samples


def truth():
    a=dict(pose=(0.,0.,0.),k=0.,dk=0.,edges={
        side:dict(x=0.,y=t,heading=0.,curvature=0.) for side,t in [('left',1.5),('right',-1.5)]})
    q=np.array([10.,18.,12.,1.2,.4]);dummy=dict(pose=(30.,10.,.7),k=0.,dk=0.)
    cc=chain(q,a,dummy,(False,False));e=cc[-1]
    b=dict(pose=(e.XEnd,e.YEnd,e.ThetaEnd),k=0.,dk=0.,edges={side:dict(
        x=e.XEnd-t*np.sin(e.ThetaEnd),y=e.YEnd+t*np.cos(e.ThetaEnd),heading=e.ThetaEnd,curvature=0.)
        for side,t in [('left',1.5),('right',-1.5)]})
    xy,n=reference_samples(cc,np.linspace(0,40,160))
    raw={side:xy+t*n for side,t in [('left',1.5),('right',-1.5),('center',0.)]}
    return a,b,q,raw


def test_bounded_initializer_preserves_positive_long_reference_and_originals():
    a,b,q,raw=truth();copies={k:v.copy() for k,v in raw.items()}
    chosen,coef,report=initialize(a,b,raw,parameters=q+np.array([.5,-.4,.2,.03,-.02]),max_iterations=20)
    assert min(chosen[:3])>=6 and len(chosen)==5 and np.isfinite(coef).all()
    assert report['maximum_scaled_closure_residual']<1e-7
    assert report['feasible_references_retained']>0
    assert not report['production_accepted']
    assert all(np.array_equal(raw[k],v) for k,v in copies.items())


def test_invalid_or_incomplete_source_is_not_a_seed_default():
    a,b,q,raw=truth()
    for broken in ({'center':raw['center']},dict(raw,left=np.array([[0,0],[np.nan,1]]))):
        with pytest.raises(ValueError,match='complete finite'):initialize(a,b,broken,parameters=q)


def test_failed_seed_preserves_which_condition_failed(monkeypatch):
    from types import SimpleNamespace
    import mapforge.ops.long_source_seed as module
    a,b,q,raw=truth();bad=q+np.array([.5,0.,0.,0.,0.])
    def no_progress(fun,x0,**kwargs):
        return SimpleNamespace(x=np.array(x0,copy=True),success=False,message='bounded test stop')
    monkeypatch.setattr(module,'least_squares',no_progress)
    monkeypatch.setattr(module,'minimize',no_progress)
    with pytest.raises(module.ReferenceInitializationError) as error:
        module.initialize(a,b,raw,parameters=bad)
    result=error.value.report
    assert result['status']=='REJECTED_INITIALIZATION_NOT_MAP'
    assert result['bounded_closure']['maximum_scaled_closure_residual']>1e-7
    assert len(result['bounded_closure']['parameters'])==5
    assert result['source_guided']['projected_original_edge_forward_minimum']>.1
    assert not result['global_impossibility_proven'] and not result['production_accepted']
    _,_,open_report=module.initialize(a,b,raw,parameters=bad,allow_open_reference=True)
    assert open_report['status']=='OPEN_REFERENCE_JOINT_INITIALIZATION_ONLY'
    assert not open_report['reference_closed'] and not open_report['production_accepted']
    assert open_report['maximum_scaled_closure_residual']>1e-7
    assert open_report['requires_simultaneous_feasibility_restoration']
    assert open_report['actual_ribbon_forward_minimum']>=.1


def test_local_closure_restoration_uses_three_bounded_free_coordinates():
    from mapforge.ops.long_source_seed import restore_reference_closure
    a,b,q,raw=truth();perturbed=q+np.array([.001,0,0,0,0])
    lo=np.array([6.,6.,6.,-6.,-6.]);hi=np.array([100.,100.,100.,6.,6.])
    candidate,report=restore_reference_closure(perturbed,a,b,(False,False),3,lo,hi)
    assert report['attempted'] and report['improved'] and report['bounded']
    assert report['after_maximum_scaled_residual']<1e-8
    assert len(report['selected_coordinates'])==3 and min(candidate[:3])>=6
    assert np.array_equal(perturbed,q+np.array([.001,0,0,0,0]))


def test_far_closure_is_not_silently_projected_in_a_local_seed_step():
    from mapforge.ops.long_source_seed import restore_reference_closure
    a,b,q,raw=truth();q[0]+=2
    candidate,report=restore_reference_closure(q,a,b,(False,False),3,
        np.array([6.,6.,6.,-6.,-6.]),np.array([100.,100.,100.,6.,6.]))
    assert not report['attempted'] and np.array_equal(candidate,q)


def test_two_core_initializer_and_joint_kernel_keep_long_geometry_and_originals():
    from mapforge.ops.joint_connector_fit import fit_joint
    import xml.etree.ElementTree as ET
    a,_,_,_=truth();q=np.array([6.,10.,12.,6.,1.2]);dummy=dict(pose=(30.,10.,.7),k=0.,dk=0.)
    cls=chain(q,a,dummy,(True,True),2);end=cls[-1]
    b=dict(pose=(end.XEnd,end.YEnd,end.ThetaEnd),k=0.,dk=0.,edges={side:dict(
        x=end.XEnd-t*np.sin(end.ThetaEnd),y=end.YEnd+t*np.cos(end.ThetaEnd),
        heading=end.ThetaEnd,curvature=0.) for side,t in [('left',1.5),('right',-1.5)]})
    xy,n=reference_samples(cls,np.linspace(0,sum(c.length for c in cls),160))
    raw={side:xy+t*n for side,t in [('left',1.5),('right',-1.5),('center',0.)]}
    copies={k:v.copy() for k,v in raw.items()}
    chosen,co,report=initialize(a,b,raw,parameters=q,core_count=2,contact_caps=(True,True),max_iterations=10)
    kernel=fit_joint(ET.Element('road',id='100'),a,b,raw,raw,chosen,initial_coefficients=co,
        core_count=2,contact_caps=(True,True),fair_world=True,_problem_only=True)
    result=kernel['evaluate'](kernel['unpack'](kernel['initial']))
    assert report['maximum_scaled_closure_residual']<1e-7
    assert len(result[0])==4 and min(c.length for c in result[0])>=6
    assert min(np.diff(result[1]))>=3 and max(abs(result[4]))<1e-6
    assert all(np.array_equal(raw[k],v) for k,v in copies.items())
