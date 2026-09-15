from types import SimpleNamespace
import numpy as np
from scipy.sparse import csr_matrix
import pytest
from mapforge.ops.source_feasible_gauss_newton import qp_step,solve


def test_qp_preserves_source_bound_even_if_unconstrained_fit_would_cross_it():
    step,info=qp_step(csr_matrix(np.eye(2)),np.array([-2.,-3.]),
        np.array([[-1.,0.]]),np.array([-.2]),np.array([[-4.,4.],[-4.,4.]]),np.full(2,4.))
    assert step is not None and info['original_step_violation']<=1e-7
    assert step[0]==pytest.approx(.2,abs=1e-6) and step[1]>2.9


def test_joint_iterations_keep_parent_feasible_while_turn_variable_can_change():
    source=SimpleNamespace(slack0=np.array([.2]),CZ=np.array([[-1.]]))
    problem=SimpleNamespace(initial=np.zeros(2),parents={'a':source},parent_slices={'a':slice(0,1)},
        turn_slices={'c':slice(1,2)},kernels={'c':{'nr':1,'core_count':1}},bounds=np.array([[-4.,4.],[-4.,4.]]),
        residual=lambda x:x-np.array([2.,3.]),jacobian=lambda x:csr_matrix(np.eye(2)),
        summary=lambda x:{'status':'REJECTED'})
    before=problem.initial.copy();x,report=solve(problem,before,max_iterations=4)
    assert x[0]<=.2+1e-7 and x[1]>2.9
    assert all(r.get('source_inequality_violation',0.)<=1e-7 for r in report['history'])
    assert not report['production_accepted'] and np.array_equal(before,problem.initial)
    with pytest.raises(ValueError,match='initial state violates'):solve(problem,np.array([.3,0.]))


def test_equivalent_scaling_preserves_ill_scaled_original_source_constraints():
    J=csr_matrix(np.diag([1e6,1e-2]))
    C=np.array([[-1e8,0.],[0.,-1e-6]])
    lower=np.array([-.2e8,-.4e-6])
    step,info=qp_step(J,np.array([-2e6,-3e-2]),C,lower,np.tile([-4.,4.],(2,1)),np.full(2,4.))
    assert step is not None and info['equivalent_positive_scaling']
    assert max(lower-C@step)<=1e-7
    assert step[0]==pytest.approx(.2,abs=1e-6) and step[1]<=.4
    # Feasible descent, not a high-relative-accuracy optimum for the tiny term.
    before=np.array([-2e6,-3e-2]);after=J@step+before
    assert after@after<before@before


def test_roundoff_equality_remnants_are_not_amplified_into_false_infeasibility():
    C=np.array([[-1.,0.],[-1e-16,1e-16],[1e-16,-1e-16]])
    lower=np.array([-.2,1e-15,1e-15])
    step,info=qp_step(csr_matrix(np.eye(2)),np.array([-2.,-3.]),C,lower,
        np.tile([-4.,4.],(2,1)),np.full(2,4.))
    assert step is not None and info['numerically_null_step_rows']==2
    assert info['numerical_rows_whole_box_violation_bound']<=1e-12
    assert max(lower-C@step)<=1e-7 and step[0]<=.2+1e-8
    # A genuine infeasible source inequality must not receive the same treatment.
    lower[1:]=1e-5
    step,_=qp_step(csr_matrix(np.eye(2)),np.array([-2.,-3.]),C,lower,
        np.tile([-4.,4.],(2,1)),np.full(2,4.))
    assert step is None


def test_residual_reduction_cannot_destroy_preexisting_regular_geometry():
    parent=SimpleNamespace(slack0=np.array([1.]),CZ=np.array([[0.]]))
    problem=SimpleNamespace(initial=np.zeros(2),parents={'p':parent},parent_slices={'p':slice(0,1)},
        turn_slices={'r':slice(1,2)},kernels={'r':dict(nr=1,core_count=1)},bounds=np.tile([-4.,4.],(2,1)),
        residual=lambda x:x-np.array([0.,2.]),jacobian=lambda x:csr_matrix(np.eye(2)),
        summary=lambda x:dict(status='REJECTED'),regular_geometry=lambda x:dict(regular=x[1]<=.3))
    x,report=solve(problem,problem.initial,max_iterations=2)
    assert 0.<x[1]<=.3 and not report['production_accepted']
    with pytest.raises(ValueError,match='regular positive-width'):solve(problem,np.array([0.,.4]))


def test_tiny_coefficient_with_large_positive_margin_is_proven_box_redundant():
    C=np.array([[-1.,0.],[1e-17,-1e-17]])
    lower=np.array([-.2,-.9]);bounds=np.tile([-4.,4.],(2,1))
    step,info=qp_step(csr_matrix(np.eye(2)),np.array([-2.,-3.]),C,lower,bounds,np.full(2,4.))
    assert step is not None and info['box_redundant_step_rows']==1
    assert info['box_redundant_minimum_original_margin']>.89
    assert info['original_constraints_all_rechecked'] and max(lower-C@step)<=1e-7
    assert step[0]<=.2+1e-7 and step[1]>2.9
