"""Source contact reuse does not imply written coverage, shape or path acceptance."""
import copy

import numpy as np
import pytest

from mapforge.ops.reconstruction_scope import digest
from mapforge.repair_web.event_source_binding import (
    bind_sources, original_slice, proven_path, continuation_graphs, written_feature, interval_residual,
)
from mapforge.repair_web.model import parse
from mapforge.repair_web.split_event_admission import collect_sources
from scripts.bind_split_event_sources import inputs, verify_raw_contacts


@pytest.fixture(scope='module')
def original():
    return inputs()


@pytest.fixture(scope='module')
def report(original):
    return bind_sources(*original)


def test_real_raw_contacts_are_recompiled_not_trusted_from_pass_text(original):
    verify_raw_contacts(*original[1:4])


def test_real_39_debts_split_into_intervals_and_explicit_endpoint_observations(report):
    assert report['source_relation_debt_count'] == 39
    assert report['binding_counts'] == dict(WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION=35,
        PREBIRTH_ENDPOINT_OBSERVATION=2, EXTERNAL_ENDPOINT_OBSERVATION=2)
    assert report['endpoint_observation_count'] == 4
    assert all(r['binding']['part_identity_retained'] and not r['binding']['source_interval_clipped'] for r in report['receipts'])
    assert not report['old_structural_ledger_rewritten']


def test_35_actual_intervals_cover_original_support_without_nearest_matching(report):
    for row in report['receipts']:
        b = row['binding']
        if b['binding_kind'] != 'WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION': continue
        a, z = sorted([b['source_st'][0][0], b['source_st'][-1][0]])
        assert b['targets'][0]['domain'][0] == a and b['targets'][-1]['domain'][1] == z
        for t in b['targets']:
            assert t['actual_source_feature'] == row['original_debt']['feature'] or t['continuation_evidence']
        for x, y in zip(b['targets'][:-1], b['targets'][1:]): assert x['domain'][1] == y['domain'][0]


def test_prebirth_boundary_is_not_aliased_to_mother_or_created_early(report):
    row = next(r for r in report['receipts'] if r['binding']['binding_kind'] == 'PREBIRTH_ENDPOINT_OBSERVATION'
               and r['original_debt']['kind'] == 'physical_boundary')
    b = row['binding']
    assert b['target_station'] == 131.0543 and not b['birth_station_changed']
    assert b['residual']['max_distance_m'] == pytest.approx(.0305737614384)
    assert not b['residual']['geometric_coverage_proven'] and b['residual']['acceptance'] == 'NOT_ADMITTED'
    assert min(p[0] for p in b['source_st']) < b['target_station']


def test_approved_movement_role_not_reopened_as_physical_center_conflict(report):
    row = next(r for r in report['receipts'] if r['binding']['binding_kind'] == 'PREBIRTH_ENDPOINT_OBSERVATION'
               and r['original_debt']['kind'] == 'movement_path')
    b = row['binding']
    assert b['observation_role'] == 'independent_movement_path' and not b['movement_center_equality_required']
    assert b['residual']['max_distance_m'] == pytest.approx(1.79180778539)
    assert 'diagnostic only' in b['residual']['meaning']
    assert report['source_role_authorizations_added'] == 0


def test_package_external_heads_stay_finite_and_not_dropped_for_small_size(report):
    rows = [r for r in report['receipts'] if r['binding']['binding_kind'] == 'EXTERNAL_ENDPOINT_OBSERVATION']
    assert len(rows) == 2
    assert all(r['binding']['target_station'] == 0 and r['binding']['external_contact_evidence'] for r in rows)
    assert all(min(p[0] for p in r['binding']['source_st']) < 0 for r in rows)
    assert max(r['binding']['residual']['max_distance_m'] for r in rows) == pytest.approx(.004479222409)


def test_no_geometric_acceptance_or_solver_authority_created(report):
    assert report['source_relation_status'] == 'BOUND'
    assert report['source_constraint_status'] == 'READY_FOR_SHAPE_DESIGN'
    assert 'NOT_FULL_INTERVAL_COVERAGE' in report['written_extent_status']
    assert not any(report[k] for k in ('geometry_solver_ran','new_xodr','export_allowed','map_accepted'))
    assert all(r['binding']['residual']['acceptance'] == 'NOT_ADMITTED' for r in report['receipts'])


def test_endpoint_admission_reuses_existing_budget_but_not_for_movement_midpoints(report):
    rows=report['endpoint_checks']
    physical=[r for r in rows if r['status']=='WITHIN_EXISTING_ENDPOINT_BUDGET']
    movements=[r for r in rows if r['status']=='INDEPENDENT_PATH_RETAINED_NOT_GEOMETRY_ACCEPTED']
    assert len(physical)==2 and all(r['tolerance_m']==.35 for r in physical)
    assert len(movements)==2 and all(not r['physical_midpoint_test_applied'] for r in movements)
    approved=next(r for r in movements if r['feature']=='lane:2023041111104128071')
    assert approved['existing_role_decision_replayed']


def test_slice_retains_interior_raw_vertices_and_refuses_extrapolation():
    st = np.array([[0., 0.], [1., 1.], [2., 0.]])
    np.testing.assert_allclose(original_slice(st, [0.,2.,4.], [1.,3.]), [[.5,.5],[1.,1.],[1.5,.5]])
    with pytest.raises(ValueError): original_slice(st, [0.,2.,4.], [-.1,2.])
    with pytest.raises(ValueError): original_slice(st, [0.,2.,4.], [2.,5.])


def test_proven_identity_path_requires_all_contacts_and_handles_cycles():
    graph = {'a':[('b','ab')], 'b':[('a','ba'), ('c','bc')]}
    assert proven_path(graph,'a','c') == ['ab','bc']
    with pytest.raises(ValueError): proven_path(graph, 'a', 'nearby')
    with pytest.raises(ValueError): proven_path({'a':[('b','ab')],'b':[('a','ba')]},'a','c')


def test_zero_birth_must_not_become_an_ordinary_boundary_or_traffic_union():
    contacts = dict(boundary_endpoints={'a':{'feature':'A'}, 'b':{'feature':'B'}}, transition_events=[
        dict(road='11',kind='zero_width_birth',status='SOURCE_C0_CONTACT_PROVEN',from_source='L1',to_source='L2')])
    physical = dict(relations=[dict(endpoints=['a','b'], ordinary_lane_link_supported=False,
                                   witnesses=[dict(road='11',kind='zero_width_birth')])])
    boundaries, paths = continuation_graphs(contacts,physical)
    with pytest.raises(ValueError): proven_path(boundaries, 'A','B')
    with pytest.raises(ValueError): proven_path(paths,'lane:L1','lane:L2')


def test_wrong_node_even_at_same_xy_is_rejected_by_fresh_contact_replay(original):
    _, p, d, c, _ = original
    modified = copy.deepcopy(c)
    key = next(iter(modified['boundary_endpoints']))
    modified['boundary_endpoints'][key]['original_node_id'] = 'not-the-original-node'
    modified['content_sha256'] = digest({k:v for k,v in modified.items() if k!='content_sha256'})
    with pytest.raises(ValueError, match='freshly read'):
        verify_raw_contacts(p,d,modified)


def test_source_or_role_packet_drift_cannot_be_hidden_by_receipt_label(original):
    data,p,d,c,roles = original
    bad = copy.deepcopy(c); bad['previous_unassigned_intervals'] = []
    with pytest.raises(ValueError): bind_sources(data,p,d,bad,roles)
    bad = copy.deepcopy(roles); bad['decision']['decisions'] = []
    with pytest.raises(ValueError): bind_sources(data,p,d,c,bad)


def test_consumer_recomputes_actual_cubic_not_source_or_stored_error(original,report):
    data,p,_,_,_ = original; root=parse(data); road,_=collect_sources(root,p)
    row = next(r for r in report['receipts'] if r['original_debt']['kind']=='physical_boundary'
        and r['binding']['binding_kind']=='WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION')
    st=np.asarray(row['binding']['source_st']); before=interval_residual(road,row['original_debt'],st)['max_lateral_m']
    for offset in road.findall('lanes/laneOffset'): offset.set('a', str(float(offset.get('a'))+2.))
    after=interval_residual(road,row['original_debt'],st)['max_lateral_m']
    assert after > before+1.5


def test_nonzero_solver_is_never_called(original,monkeypatch):
    import scipy.optimize
    def no(*args,**kwargs): raise AssertionError('Source binding must not fit')
    for name in ('minimize','least_squares','root','linprog'): monkeypatch.setattr(scipy.optimize,name,no)
    assert bind_sources(*original)['geometry_solver_ran'] is False
