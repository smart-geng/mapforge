import numpy as np
import pytest

from spikes.source_jet_relaxation import graph_derivative_bounds,SourceJetRelaxation
from spikes.source_contact_fit import SourceBoundaryBlock
from tests.test_source_contact_fit import straight_fixture


def test_zero_curvature_straight_secant_bounds_true_slope_without_assumption():
    raw=np.array([[0.,2.],[40.,10.]])
    b=graph_derivative_bounds(raw,[10.,12.],0.,0.,.001)
    assert b['slope']==pytest.approx(.2)
    assert b['second']==0
    assert b['third']==pytest.approx(.001*(1+.2**2)**2)


def test_exact_circle_is_inside_necessary_derivative_bounds():
    # Upper graph of a radius-100 circle; k=constant, k_s=0.
    s=np.linspace(-10,10,21);raw=np.c_[s,100-np.sqrt(10000-s*s)]
    b=graph_derivative_bounds(raw,[-3.,4.],.01,.01,0.)
    t=np.linspace(-3,4,100);q=np.sqrt(10000-t*t)
    assert max(abs(t/q))<=b['slope']
    assert max(abs(10000/q**3))<=b['second']
    assert max(abs(30000*t/q**5))<=b['third']


def test_large_heading_circle_needs_the_full_metric_factor_in_third_derivative():
    # A nearly 45-degree tangent exposes a missing (1+p*p)^2 factor that
    # small-angle road cases and zero-curvature tests cannot reliably detect.
    s=np.linspace(69.9,70.1,21);raw=np.c_[s,100-np.sqrt(10000-s*s)]
    b=graph_derivative_bounds(raw,[69.99,70.01],1e-6,.01,0.)
    q=np.sqrt(10000-70.**2);actual=30000*70./q**5
    assert actual>3*b['slope']*.01**2  # The previous incorrect upper bound.
    assert actual<=b['third']
    assert b['third']==pytest.approx(3*b['slope']*.01**2*(1+b['slope']**2)**2)


def test_unbounded_heading_or_bad_source_never_claims_conflict():
    raw=np.array([[0.,0.],[10.,0.]])
    assert graph_derivative_bounds(raw,[0,10],1.,1.,1.)['status']=='UNAVAILABLE'
    with pytest.raises(ValueError):graph_derivative_bounds(raw[::-1],[0,10],.35,.01,.001)


def test_straight_shared_source_relaxation_is_not_a_curve_or_delivery():
    model=SourceBoundaryBlock(*straight_fixture(),degree=5)
    before={k:v.copy() for k,v in model.raw.items()}
    check=SourceJetRelaxation(model,step=5.);x,report=check.solve()
    assert x is not None and report['status']=='RELAXATION_FEASIBLE_ONLY',report
    assert report['minimum_normalized_slack']<1e-7
    assert not report['spline_basis_used'] and not report['export_allowed']
    assert not report['curve_reconstructed'] and not report['global_map_impossibility_proven']
    assert all(np.array_equal(model.raw[k],v) for k,v in before.items())


def test_positive_dual_slack_is_a_conditional_numerical_conflict():
    model=SourceBoundaryBlock(*straight_fixture());check=SourceJetRelaxation(model,5.)
    row=check.row(next(iter(model.owner)),check.family_stations[0][0])
    check.bound(row,-10.,dict(kind='synthetic-conflict'))
    check.bound({k:-v for k,v in row.items()},-10.,dict(kind='synthetic-conflict'))
    _,r=check.solve()
    assert r['status']=='NUMERICAL_CONFLICT' and r['minimum_normalized_slack']>=10-1e-6
    assert r['dual_residual']<1e-6 and r['duality_gap']<1e-6
    assert not r['global_map_impossibility_proven']


def test_restricting_to_long_basis_is_labelled_and_straight_still_passes():
    model=SourceBoundaryBlock(*straight_fixture(),degree=5);check=SourceJetRelaxation(model,5.)
    x,report=check.solve(restrict_basis=True)
    assert x.shape==(model.nvar,) and report['status']=='RELAXATION_FEASIBLE_ONLY'
    assert report['spline_basis_used'] and not report['export_allowed']
    actual=check.basis_mapping()@x
    assert np.max(check.matrix(check.inequalities)@actual-np.asarray(check.rhs))<1e-6


def test_event_basis_has_fewer_or_equal_variables_and_no_short_fallback():
    from spikes.source_event_basis import event_aligned_model
    model=SourceBoundaryBlock(*straight_fixture(),degree=5)
    # Synthetic physical branch event, not a sampling knot.
    model.endpoint_groups.append([('a',31.),('b',31.),('c',31.)])
    altered,report=event_aligned_model(model)
    assert 31. in report['event_stations_m']
    assert report['min_span_m']>=15 and altered.nvar<=model.nvar
    assert all(31. in f.knots for f in altered.families)
    assert all(31. not in f.knots for f in model.families)
    assert not report['direct_xodr_compilation_implemented']
    model.endpoint_groups.append([('a',35.),('b',35.),('c',35.)])
    with pytest.raises(ValueError,match='long-span'):event_aligned_model(model)


def test_solver_success_with_nonfinite_primal_is_never_accepted(monkeypatch):
    import spikes.source_jet_relaxation as module
    real=module.linprog
    def broken(*a,**kw):
        result=real(*a,**kw);result.x[0]=np.nan;return result
    monkeypatch.setattr(module,'linprog',broken)
    check=SourceJetRelaxation(SourceBoundaryBlock(*straight_fixture()),5.)
    x,r=check.solve()
    assert x is None and r['status']=='UNAVAILABLE'


def test_replay_rejects_self_signed_scope_promotion(tmp_path,monkeypatch):
    import json
    import scripts.check_source_graph_feasibility as script
    model=SourceBoundaryBlock(*straight_fixture(),degree=5)
    monkeypatch.setattr(script,'load',lambda *a,**kw:(model,model.roles,{}))
    directory=tmp_path/'evidence'
    script.run('synthetic-source','synthetic-decision',directory,5.)
    assert script.verify(directory)['status']=='DIAGNOSTIC_VERIFIED_NOT_MAP'
    report=json.loads((directory/'report.json').read_text(encoding='utf-8'))
    report['incident_connectors_checked']=True
    script.dump(directory/'report.json',report)
    run=json.loads((directory/'run.json').read_text(encoding='utf-8'))
    run['output_sha256']['report.json']=script.sha(directory/'report.json')
    script.dump(directory/'run.json',run)
    with pytest.raises(ValueError,match='scope or qualification'):script.verify(directory)
