import copy

import numpy as np
import pytest
from scipy.interpolate import BSpline

from mapforge.ops.arc_source_chart import ArcChart, ArcSourceTrace
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from spikes.source_contact_fit import SourceBoundaryBlock
from spikes.shared_boundary_dynamics import evaluate
from tests.test_source_contact_fit import straight_fixture


@pytest.mark.parametrize('k',[0.,1e-12,-1e-12,.004,-.004])
def test_exact_arc_chart_roundtrip_including_large_radius(k):
    axis=ArcChart((3.,-7.),.6,k)
    st=np.c_[np.linspace(-20.,100.,30),np.linspace(-8.,9.,30)]
    assert axis.project(axis.world(st))==pytest.approx(st,abs=5e-12)


@pytest.mark.parametrize('k',[0.,.004,-.004])
def test_entire_original_line_is_enclosed_not_its_projected_chord(k):
    axis=ArcChart((0.,0.),0.,k)
    raw=np.array([[0.,-3.],[40.,7.],[100.,7.3]])
    trace=ArcSourceTrace(axis,raw);before=trace.xy.copy()
    for i in range(2):
        a,b=trace.st[i:i+2,0]
        for lo,hi in zip(np.linspace(a,b,20)[:-1],np.linspace(a,b,20)[1:]):
            ss=np.linspace(lo,hi,100);tt=trace.values(i,ss)
            linear=np.interp(ss,[lo,hi],trace.values(i,[lo,hi]))
            assert np.max(abs(tt-linear))<=trace.chord_error_bound(i,lo,hi)+1e-12
            xy=axis.world(np.c_[ss,tt])
            assert np.max(abs((xy-raw[i])@trace.normals[i]))<1e-12
    assert np.array_equal(trace.xy,before)


def test_arc_trace_fails_closed_on_wrapped_or_backtracking_source():
    axis=ArcChart((0.,0.),0.,.01)
    with pytest.raises(ValueError):axis.project([[2.,101.],[4.,103.]])
    with pytest.raises(ValueError):ArcSourceTrace(axis,[[0.,0.],[2.,0.],[1.,0.]])
    with pytest.raises(ValueError):axis.world([[0.,100.]])


@pytest.mark.parametrize('k',[0.,.001,-.001])
def test_shared_arc_cubic_keeps_original_sources_and_exact_long_coefficients(k):
    original=SourceBoundaryBlock(*straight_fixture());before=copy.deepcopy(original.raw)
    model=ArcSourceBoundaryBlock(original,curvature=k)
    x,phase=model.solve()
    assert x is not None,phase
    audit=model.audit(x)
    assert audit['boundary_same_chart_max_m']<=.35+1e-7
    assert not audit['export_allowed'] and not audit['xodr_generated']
    assert model.degree==3 and model.describe()['reference_primitive_count']==1
    for row in model.coefficients(x):
        assert row['length']>=15-1e-7
        f=model.families[row['family']];curve=BSpline(f.knots,x[f.columns],3)
        ds=np.linspace(0.,row['length'],20)
        assert np.polynomial.polynomial.polyval(ds,row['abcd'])==pytest.approx(curve(row['s']+ds),abs=1e-10)
    for key in before:assert np.array_equal(original.raw[key],before[key])
    assert model.source_vertex_indices==original.source_vertex_indices
    for key,indices in model.source_vertex_indices.items():assert sorted(indices)==list(range(len(model.raw[key])))


def test_constant_width_arc_has_correct_offset_curvature_and_gradient():
    # One constant t plus free derivatives, independent finite difference check.
    rows=np.eye(4)[None,:,:];scale=np.ones((1,2));x=np.array([3.,.08,.001,.00002]);k=.004
    v,jac=evaluate(rows,scale,x,(1.,1.),k)
    numeric=np.column_stack([(evaluate(rows,scale,x+np.eye(4)[j]*1e-7,(1.,1.),k)[0].ravel()-
                              evaluate(rows,scale,x-np.eye(4)[j]*1e-7,(1.,1.),k)[0].ravel())/2e-7 for j in range(4)])
    assert jac==pytest.approx(numeric,rel=2e-7,abs=1e-10)
    constant=evaluate(rows,scale,np.array([3.,0.,0.,0.]),(1.,1.),k)[0]
    assert constant[0,0]==pytest.approx(k/(1-3*k))
    assert constant[0,1]==pytest.approx(0.)


def test_no_compilation_of_invalid_shared_state_or_line_only_diagnostic_on_arc():
    model=ArcSourceBoundaryBlock(SourceBoundaryBlock(*straight_fixture()),curvature=.001)
    with pytest.raises(ValueError):model.coefficients(np.full(model.nvar,100.))
    with pytest.raises(ValueError):model.necessary_source_preflight()
    from spikes.source_jet_relaxation import SourceJetRelaxation
    with pytest.raises(ValueError):SourceJetRelaxation(model)


def test_original_vertex_preflight_is_necessary_only_and_has_checked_residuals():
    model=ArcSourceBoundaryBlock(SourceBoundaryBlock(*straight_fixture()),curvature=.001)
    x,r=model.original_vertex_preflight()
    assert x.shape==(model.nvar,) and r['status']=='NECESSARY_VERTICES_ONLY'
    assert not r['export_allowed'] and not r['global_impossibility_proven']
    assert not r['uses_source_interpolation_enclosure']
    assert r['source_boundary_vertices']==sum(len(model.raw[k]) for k in model.owner)
    assert max(r['primal_residual'],r['dual_residual'],r['duality_gap'])<1e-7
    x,sampled=model.sampled_source_preflight()
    assert sampled['status']=='NECESSARY_SAMPLES_ONLY' and not sampled['export_allowed']
    assert sampled['physical_center_observations_included'] and not sampled['dynamics_included']
    assert sampled['width_witnesses']>0 and sampled['original_boundary_and_center_witnesses']>r['source_boundary_vertices']


def test_projected_chord_is_not_sufficient_in_curved_reference_coordinates():
    trace=ArcSourceTrace(ArcChart((0.,0.),0.,.004),[[0.,0.],[100.,0.]])
    a,b=trace.st[:,0];mid=(a+b)/2
    error=abs(trace.values(0,mid)-np.mean(trace.st[:,1]))
    assert error>1  # Simply treating the projected two endpoints as a line is wrong.
    assert error<=trace.chord_error_bound(0,a,b)
    cells=list(trace.cells(a,b,[],step=2.))
    assert max(row[-1] for row in cells)<.01
    assert len(trace.xy)==2  # Certificate density does not become source density.


def test_both_families_use_same_internal_knots_and_never_a_short_endpoint_fix():
    model=ArcSourceBoundaryBlock(SourceBoundaryBlock(*straight_fixture()),curvature=.001)
    internal=[np.unique(f.knots)[1:-1] for f in model.families]
    assert np.array_equal(internal[0],internal[1])
    assert min(min(np.diff(np.unique(f.knots))) for f in model.families)>=15-1e-7
    assert model.describe()['whole_road_compiler_complete'] is False


def test_failed_or_nonfinite_lp_never_becomes_best_axis_trial():
    from scripts.fit_arc_source_block import best_source_trial
    def phase(status,**kw):return dict(phase=dict(status=status,**kw))
    invalid=[{},phase('ERROR'),phase('ERROR',minimum_uniform_constraint_slack_m=0.),
             phase('INFEASIBLE',minimum_uniform_constraint_slack_m=float('nan'))]
    assert best_source_trial(invalid) is None
    rows=invalid+[phase('INFEASIBLE',minimum_uniform_constraint_slack_m=.8),
                  phase('FEASIBLE',minimum_uniform_constraint_slack_m=0.)]
    assert best_source_trial(rows)==5
