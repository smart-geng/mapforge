import copy

import numpy as np
import pytest
from pyclothoids import Clothoid

from mapforge.ops.source_path_graph import ordinary_source_paths
from spikes.source_path_jerk_bound import SourcePathJerkBound
from spikes.source_contact_fit import SourceBoundaryBlock
from tests.test_source_contact_fit import straight_fixture


def test_straight_path_has_zero_necessary_jerk_and_no_curve_claim():
    c=SourcePathJerkBound([[0,2],[30,2],[60,2]],60);before=c.original_source.copy();x,r=c.solve()
    assert x is not None and r['status']=='NOT_EXCLUDED_NOT_A_CURVE'
    assert abs(r['minimum_necessary_jerk_ratio'])<1e-8
    assert r['guaranteed_interior_domain_m']==pytest.approx([.35,59.65])
    assert not r['source_endpoints_pinned'] and not r['curve_basis_imposed'] and not r['export_allowed']
    assert np.array_equal(before,c.original_source)


def test_known_analytic_clothoid_is_never_declared_jerk_conflict():
    c=Clothoid.StandardParams(0,0,.1,.003,.0002,20.)
    ss=np.linspace(0,20,41);raw=np.array([[c.X(float(s)),c.Y(float(s))] for s in ss])
    check=SourcePathJerkBound(raw,60);x,r=check.solve()
    assert x is not None,r
    assert r['dual_lower_bound']<=.0002*(60/3.6)**3+1e-6
    assert r['status']=='NOT_EXCLUDED_NOT_A_CURVE'


def test_fast_lateral_shift_conflict_does_not_depend_on_polynomial_degree():
    c=SourcePathJerkBound([[0,0],[40,0],[55,3],[95,3]],60);x,r=c.solve()
    assert x is not None and r['status']=='NECESSARY_JERK_CONFLICT',r
    assert r['dual_lower_bound']>1.1 and r['primal_residual']<1e-6 and r['dual_residual']<1e-6
    assert not r['branch_contacts_imposed'] and not r['general_map_impossibility_proven']


def test_euclidean_tube_superset_does_not_change_original_error_budget():
    raw=np.array([[0,0],[20,20],[40,40]],float);c=SourcePathJerkBound(raw,60)
    assert c.original_tolerance==.35
    assert c.tolerance==pytest.approx(.35*np.sqrt(2))
    # Every displacement in the Euclidean ball lies inside the vertical
    # envelope after matching its x to the ORIGINAL line, not an altered one.
    theta=np.linspace(0,2*np.pi,200);xy=[20,20]+.35*np.c_[np.cos(theta),np.sin(theta)]
    assert max(abs(xy[:,1]-xy[:,0]))<=c.tolerance+1e-10


@pytest.mark.parametrize('fault',['speed','nan','reversed','loose','short'])
def test_invalid_contract_never_succeeds(fault):
    raw=np.array([[0,0],[30,0],[60,0]],float);kw={}
    if fault=='speed':kw['speed_kmh']=0
    if fault=='nan':raw[1,1]=np.nan
    if fault=='reversed':raw=raw[::-1]
    if fault=='loose':kw['tolerance']=1.
    if fault=='short':raw[:,0]*=.01
    with pytest.raises(ValueError):SourcePathJerkBound(raw,**dict({'speed_kmh':60},**kw))


def test_success_status_with_invalid_dual_or_primal_never_proves_conflict(monkeypatch):
    import spikes.source_path_jerk_bound as module
    real=module.linprog
    def broken(*a,**kw):
        result=real(*a,**kw);result.ineqlin.marginals[0]-=1.;return result
    monkeypatch.setattr(module,'linprog',broken)
    x,r=SourcePathJerkBound([[0,0],[40,0],[55,3],[95,3]],60).solve()
    assert x is None and r['status']=='UNAVAILABLE'


def test_source_path_graph_keeps_both_original_endpoint_identities():
    m=SourceBoundaryBlock(*straight_fixture());before=copy.deepcopy(m.raw);p=ordinary_source_paths(m)
    assert p['source_lane_count']==2 and len(p['paths'])==1
    r=p['paths'][0];assert r['source_lanes_in_traffic_order']==['a','b']
    assert len(r['raw_points_st'])==4 and len(r['necessary_anchors_st'])==3
    assert len(r['anchor_source_aliases'][1])==2 and r['source_vertices_removed']==0
    assert all(np.array_equal(m.raw[k],v) for k,v in before.items())
    m.contacts['transition_events'][0]['status']='GUESS'
    with pytest.raises(ValueError,match='unproven'):ordinary_source_paths(m)


def test_close_original_vertices_survive_scaling_and_original_equation_check():
    raw=np.array([[0,0],[40,0],[40.00018,.000036],[55,3],[95,3]],float)
    c=SourcePathJerkBound(raw,60);x,r=c.solve()
    assert np.array_equal(c.original_source,raw)
    assert all(s in c.stations for s in raw[1:-1,0])
    assert r['maximum_row_scale']>1e6
    assert x is not None and r['valid_numerical_certificate'],r
    assert c.verify_certificate(r['certificate'])['valid_numerical_certificate']
    broken=copy.deepcopy(r['certificate']);broken['dual_inequality'][0]-=1.
    assert not c.verify_certificate(broken)['valid_numerical_certificate']
    broken['primal'][0]=np.nan
    with pytest.raises(ValueError):c.verify_certificate(broken)


def test_non_line_coordinate_chart_cannot_be_used_for_world_path_bound():
    m=SourceBoundaryBlock(*straight_fixture());m.reference_curvature=.001
    with pytest.raises(ValueError,match='Line chart'):ordinary_source_paths(m)


def test_source_path_graph_rejects_cycle_and_mixed_source_speed():
    m=SourceBoundaryBlock(*straight_fixture())
    m.scope['observations']['b']['source_max_speed_kmh']=30.
    with pytest.raises(ValueError,match='mixed source speed'):ordinary_source_paths(m)
    m=SourceBoundaryBlock(*straight_fixture())
    m.contacts['transition_events'].append(dict(from_source='b',to_source='a',
        kind='ordinary_continuation',status='SOURCE_C0_CONTACT_PROVEN'))
    with pytest.raises(ValueError,match='cycle'):ordinary_source_paths(m)


def test_replay_rebuilds_source_and_rejects_self_resealed_result(tmp_path,monkeypatch):
    import json
    import scripts.check_source_path_contract as script
    def load(*args,**kwargs):
        model=SourceBoundaryBlock(*straight_fixture(),degree=5)
        return model,model.roles,{}
    monkeypatch.setattr(script,'load',load)
    output=tmp_path/'review'
    report=script.run(tmp_path/'source',tmp_path/'decision',output)
    assert not report['xodr_generated'] and not list(output.glob('*.xodr'))
    fresh=script.verify(output)
    assert fresh['status']=='VERIFIED_SOURCE_CONTRACT_NOT_MAP'
    assert fresh['conflict_paths']==[] and fresh['unavailable_checks']==0
    with pytest.raises(FileExistsError):script.run('source','decision',output)
    # Editing both a report and its manifest is insufficient: source rebuild
    # must independently derive the same interpretation and result.
    report['source_speed_changed']=True
    script.dump(output/'report.json',report)
    record=json.loads((output/'run.json').read_text(encoding='utf-8'))
    record['outputs_sha256']['report.json']=script.sha(output/'report.json')
    script.dump(output/'run.json',record)
    with pytest.raises(ValueError,match='fresh source-bound diagnosis differs'):script.verify(output)
