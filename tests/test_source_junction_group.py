import copy
import numpy as np
import pytest
from mapforge.ops.arc_source_chart import ArcChart
from mapforge.ops.coupled_source_junction import SourceParentState,CoupledSourceJunction
from mapforge.ops.port_dependencies import PortDependencies
from tests.test_coupled_source_junction import real_kernel_fixture,parent_fixture
from scripts.solve_source_junction_group import written_long_seed


def two_parent_problem():
    root,a,kernel=real_kernel_fixture()
    _,stub=parent_fixture(curvature=0.,varying=False)
    model=copy.deepcopy(stub.model);model.road='11';model.axis=ArcChart((55.,35.),np.pi/2,0.)
    compiled=dict(chart_start_m=0.,chart_end_m=20.,lane_ledger=[dict(section=0,lane=-1,
        left='left',right='right',chart_s=[0.,20.])])
    b=SourceParentState(model,stub.x0,compiled)
    return CoupledSourceJunction(PortDependencies(root),{'10':a,'11':b},{'100':kernel}),root


def test_both_parent_jacobians_participate_in_same_real_connector():
    problem,_=two_parent_problem();x=problem.initial.copy()
    x[problem.parent_slices['10'].start]+=.001
    x[problem.parent_slices['11'].start]+=.001
    J=problem.jacobian(x);before=problem.residual(x)
    for rid in ('10','11'):
        direction=np.zeros_like(x);sl=problem.parent_slices[rid]
        direction[sl.start]=.3;direction[sl.start+1]=.01
        numeric=(problem.residual(x+1e-7*direction)-before)/1e-7
        np.testing.assert_allclose(J@direction,numeric,atol=2e-3,rtol=3e-3)
        assert np.linalg.norm(J@direction)>0
    assert not problem.fixed and problem.fixed_source_obstructions()==[]
    assert set(problem.summary(x)['parents'])=={'10','11'}


def test_actual_long_xml_seed_and_short_geometry_rejection():
    from spikes.measured_connector_caps import write_ribbon
    from mapforge.ops.long_connector_chain import chain
    problem,root=two_parent_problem();kernel=problem.kernels['100']
    frames=problem.frames('100',problem.contact_jets(problem.initial,'100'))
    full=kernel['unpack'](kernel['initial'],frames);result=kernel['evaluate'](full,frames)
    actual=write_ribbon(root.find("road[@id='100']"),*result[:3])
    seed=written_long_seed(actual,frames)
    reconstructed=chain(seed['shape_parameters'],*frames,kernel['caps'],seed['reference_core_count'])
    np.testing.assert_allclose([c.length for c in reconstructed],[c.length for c in result[0]],atol=1e-8)
    np.testing.assert_allclose([c.KappaEnd for c in reconstructed],[c.KappaEnd for c in result[0]],atol=1e-10)
    actual.find('planView/geometry').set('length','1.0')
    with pytest.raises(ValueError,match='short or nonfinite'):written_long_seed(actual,frames)


def test_hard_source_step_runs_the_real_two_parent_geometry_kernel():
    from mapforge.ops.source_feasible_gauss_newton import solve,source_constraints
    problem,_=two_parent_problem();initial=problem.initial.copy()
    C,lower=source_constraints(problem)
    final,report=solve(problem,initial,max_iterations=1)
    assert report['method']=='hard-source-bounded-Gauss-Newton-QP-with-backtracking'
    assert max(lower-C@final)<=1e-7
    r0=problem.residual(initial);r1=problem.residual(final)
    assert r1@r1<=r0@r0
    np.testing.assert_array_equal(initial,problem.initial)
    assert not report['production_accepted']
    assert problem.regular_geometry(final)['regular']


def test_two_core_regular_seed_keeps_full_shared_geometry_constraints_and_derivatives():
    from mapforge.ops.long_connector_chain import chain
    from mapforge.ops.regular_ribbon_seed import initialize_regular_ribbon
    from mapforge.ops.source_connector_ribbon import reference_samples
    from mapforge.ops.joint_connector_fit import fit_joint
    old,root=two_parent_problem();a,b=old.frames('100',old.contact_jets(old.initial,'100'))
    q=np.array([6.,18.,18.,6.,1.5]);cls=chain(q,a,b,(True,True),2)
    xy,n=reference_samples(cls,np.linspace(0,sum(c.length for c in cls),100))
    raw={side:xy+t*n for side,t in [('left',1.5),('right',-1.5),('center',0.)]}
    co,_=initialize_regular_ribbon(cls,a,b,raw)
    kernel=fit_joint(root.find("road[@id='100']"),a,b,raw,raw,q,initial_coefficients=co,
        core_count=2,contact_caps=(True,True),fair_world=True,_problem_only=True)
    problem=CoupledSourceJunction(old.graph,old.parents,{'100':kernel});x=problem.initial.copy()
    eq,_,_,_=problem.turn('100',x[problem.turn_slices['100']],problem.contact_jets(x,'100'))
    assert len(eq)==16 and max(abs(eq))>1e-3  # open seed must retain closure/G2 violations
    J=problem.jacobian(x);before=problem.residual(x)
    direction=np.zeros_like(x)
    for sl in problem.parent_slices.values():direction[sl.start]=.3;direction[sl.start+1]=.01
    numeric=(problem.residual(x+1e-7*direction)-before)/1e-7
    np.testing.assert_allclose(J@direction,numeric,atol=2e-3,rtol=3e-3)
    assert problem.regular_geometry(x)['regular'] and not problem.summary(x)['production_accepted']
