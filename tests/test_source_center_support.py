import copy

import numpy as np
import pytest

from mapforge.ops.source_center_support import center_support_plan,audit_center_support,source_slice,source_xy
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from spikes.source_contact_fit import SourceBoundaryBlock
from tests.test_source_contact_fit import straight_fixture
from tests.test_source_transition_domains import oblique_fixture


def exterior_fixture(gap=.2):
    m=SourceBoundaryBlock(*straight_fixture(),degree=5)
    m.raw['lane:a'][0,0]-=gap  # synthetic source only; no real file mutation
    m._build_constraints()
    return m


def test_exterior_tail_uses_one_total_euclidean_budget_and_never_extends_geometry():
    m=exterior_fixture();before=copy.deepcopy(m.raw);knots=[f.knots.copy() for f in m.families]
    plan=m.center_support;cap=plan['endpoint_caps'][0]
    assert len(plan['endpoint_caps'])==1 and not plan['unresolved']
    assert cap['a']==pytest.approx(-.2) and cap['b']==pytest.approx(0.)
    w=cap['witnesses'][0]
    assert w['transverse_allowance_m']==pytest.approx(np.sqrt(.35**2-.2**2))
    x,r=m.solve();assert x is not None,r
    assert audit_center_support(m,x,plan)['physical_source_to_center_certified']
    # Lateral error .34 alone passes .35, but sqrt(.34²+.2²) does not.
    wrong=x+.34
    a=audit_center_support(m,wrong,plan)
    assert not a['physical_source_to_center_certified'] and a['maximum_conservative_error_m']>.35
    assert max(m.lower-m.C@wrong)>1e-3
    assert all(np.array_equal(m.raw[k],v) for k,v in before.items())
    assert all(np.array_equal(f.knots,k) for f,k in zip(m.families,knots))
    assert not plan['geometry_added'] and not a['export_allowed'] and not a['dynamics_certified']


def test_endpoint_cap_cannot_hide_excessive_longitudinal_error():
    m=exterior_fixture(.5);plan=m.center_support
    assert plan['unresolved'] and not plan['endpoint_caps']
    assert plan['unresolved'][0]['reason'].startswith('chosen endpoint cannot cover')
    x,_=m.solve();a=audit_center_support(m,x,plan)
    assert not a['physical_source_to_center_certified']


def test_oblique_record_gap_uses_proven_continuation_not_cap_or_nearest_lane():
    m=oblique_fixture();before=m.nvar;plan=center_support_plan(m)
    parts=[p for p in plan['pieces'] if p['host_domain_kind']=='source_transition_band']
    assert [p['source_lane'] for p in parts]==['a','b']
    assert np.array([[p['a'],p['b']] for p in parts])==pytest.approx(np.array([[29.,30.],[30.,31.]]))
    assert not plan['endpoint_caps'] and not plan['unresolved'] and m.nvar==before
    m.contacts['transition_events'][0]['contacts'][0]['to_side']='right'
    with pytest.raises(ValueError):center_support_plan(m)


def test_arc_end_cap_uses_world_frame_and_original_straight_source():
    original=SourceBoundaryBlock(*straight_fixture());original.raw['lane:a'][0,0]-=.2
    original._build_constraints()
    m=ArcSourceBoundaryBlock(original,heading_delta=.005,curvature=.0001)
    cap=m.center_support['endpoint_caps'][0]
    assert abs(cap['host_station'])>1e-4  # not copied from the Line cut at s=0
    for w in cap['witnesses']:
        assert source_xy(m,'lane:a',w['source_station'])==pytest.approx(w['source_xy'])
    x,r=m.solve();assert x is not None,r
    a=audit_center_support(m,x,m.center_support)
    assert a['physical_source_to_center_certified']
    for cap,row in zip(m.center_support['endpoint_caps'],[r for r in a['rows'] if r['kind']=='euclidean_endpoint_cap']):
        # Dense ORIGINAL straight segment, independent of chart interpolation.
        xy=np.array([w['source_xy'] for w in cap['witnesses']])
        pts=xy[0]+np.linspace(0,1,501)[:,None]*(xy[-1]-xy[0])
        assert np.linalg.norm(pts-row['endpoint_xy'],axis=1).max()<=row['conservative_error_m']+1e-10
    with pytest.raises(ValueError):source_slice(m,'lane:a',-100,0)


def test_numerical_contact_sliver_is_checked_not_deleted_or_exported_as_segment():
    m=SourceBoundaryBlock(*straight_fixture(),degree=5);count=m.nvar
    event=m.contacts['transition_events'][0]
    for pair in event['contacts']:
        tip=m.contacts['boundary_endpoints'][pair['to_endpoint']];key=tip['feature']
        i=m.source_vertex_indices[key].index(tip['vertex_index'])
        m.raw[key][i,0]+=2e-9;tip['xy'][0]+=2e-9
    m._build_constraints();plan=m.center_support
    caps=[d for d in plan['endpoint_caps'] if d['kind']=='numerical_contact_cap']
    assert len(caps)==1 and not plan['unresolved']
    assert caps[0]['b']-caps[0]['a']==pytest.approx(2e-9,abs=1e-12)
    assert caps[0]['original_contact_record']==event['topology_record']
    assert m.nvar==count and not plan['geometry_added'] and plan['raw_vertices_removed']==0
    x,r=m.solve();assert x is not None,r
    a=audit_center_support(m,x,plan)
    assert a['physical_source_to_center_certified'] and not a['dynamics_certified']


def test_center_evidence_replay_rejects_self_signed_map_acceptance(tmp_path,monkeypatch):
    import scripts.check_source_graph_feasibility as previous_script
    import scripts.check_source_center_coverage as script
    m=exterior_fixture()
    for module in (previous_script,script):
        monkeypatch.setattr(module,'load',lambda *a,**kw:(copy.deepcopy(m),m.roles,{}))
    previous=tmp_path/'previous';output=tmp_path/'center'
    previous_script.run('synthetic-source','synthetic-decision',previous,2.,fit_events=True)
    report=script.run('synthetic-source','synthetic-decision',previous,output)
    assert not list(output.glob('*.xodr'))
    verified=script.verify(output)
    assert verified['physical_source_to_center_certified'] and not verified['export_allowed']
    report['export_allowed']=True
    script.dump(output/'report.json',report)
    manifest=script.read(output/'run.json');manifest['outputs_sha256']['report.json']=script.sha(output/'report.json')
    script.dump(output/'run.json',manifest)
    with pytest.raises(ValueError,match='fresh source center reconstruction differs'):script.verify(output)
