import json

import numpy as np
import pytest

from tests.test_source_transition_domains import oblique_fixture
from spikes.source_jet_relaxation import SourceJetRelaxation
from scripts.check_source_transition_coverage import inspect_jets,context_series


def test_unchecked_gap_can_hide_derivative_spike_but_closed_check_catches_it():
    m=oblique_fixture();check=SourceJetRelaxation(m,2.);x,r=check.solve()
    assert r['status']=='RELAXATION_FEASIBLE_ONLY'
    x=x.copy()
    # Only necessary-test station derivatives are perturbed. No polynomial
    # curve is invented from these deliberately nonintegrable free jets.
    for family in range(len(m.families)):
        x[check.columns[(family,30.,1)]]=100.
    result=inspect_jets(check,x)
    assert result['record_support']['maximum_normalized_violation']<1e-6
    assert result['transition_bands']['maximum_normalized_violation']>1
    assert not result['curve_reconstructed'] and not result['export_allowed']
    x[0]=np.nan
    with pytest.raises(ValueError,match='invalid free'):inspect_jets(check,x)


def test_transition_evidence_replay_rejects_self_signed_acceptance(tmp_path,monkeypatch):
    import scripts.check_source_graph_feasibility as previous_script
    import scripts.check_source_transition_coverage as script
    m=oblique_fixture()
    for module in (previous_script,script):
        monkeypatch.setattr(module,'load',lambda *a,**kw:(m,m.roles,{}))
    previous=tmp_path/'previous';output=tmp_path/'closed'
    previous_script.run('synthetic-source','synthetic-decision',previous,2.,fit_events=True)
    script.run('synthetic-source','synthetic-decision',previous,output)
    assert script.verify(output)['status']=='VERIFIED_COVERAGE_RECHECK_NOT_MAP'
    report=script.read(output/'report.json');report['new_geometry_generated']=True
    script.dump(output/'report.json',report)
    manifest=script.read(output/'run.json');manifest['output_sha256']['report.json']=script.sha(output/'report.json')
    script.dump(output/'run.json',manifest)
    with pytest.raises(ValueError,match='fresh readback differs'):script.verify(output)


def test_context_plot_uses_within_span_sides_and_matches_dense_band_audit():
    from spikes.shared_boundary_dynamics import audit_transition_bands
    m=oblique_fixture();x,_=m.solve();audit=audit_transition_bands(m,x)
    d=audit['coverage']['intervals'][0]
    parts=context_series(m,d,x,d['source_speed_kmh'])
    assert all(np.min(ss)>d['a'] and np.max(ss)<d['b'] for ss,_ in parts)
    values=np.concatenate([v for _,v in parts]);maximum=values.max(axis=0)*[2.5,1.]
    for j,metric in enumerate(('ay_mps2','jerk_mps3')):
        expected=max(r['value'] for r in audit['rows'] if r['metric']==metric)
        assert maximum[j]==pytest.approx(expected,abs=1e-10)
    assert not audit['near_coincident_span_diagnostics']
    assert all(r['numerically_resolved_span'] for r in audit['rows'])
