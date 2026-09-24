"""E1 cannot trim raw parts, hide dependency failures or authorize a solver."""
import copy
import json

import numpy as np
import pytest
import yaml

from mapforge.repair_web.split_event_admission import (
    admit, collect_sources, partition_rows, prefix_to_plane, slope_class, world_delta, checked_payload,
)
from mapforge.repair_web.model import parse
from mapforge.ops.reconstruction_scope import digest as object_digest
from scripts.check_split_event_admission import PACKET, REFERENCE, DECISION, DOMAIN


@pytest.fixture(scope='module')
def source():
    return REFERENCE.read_bytes(), json.loads(PACKET.read_bytes()), json.loads(DOMAIN.read_bytes()), yaml.safe_load(DECISION.read_bytes())


@pytest.fixture(scope='module')
def report(source):
    return admit(*source)


def test_real_existing_ledger_replays_without_reassigning_unowned_fragments(report):
    s = report['source']
    assert s['boundaries'] == 25 and s['movement_paths'] == 19
    assert s['source_ledger_replayed'] and s['original_parts_retained']
    assert len(s['unresolved']) == 39
    assert s['unresolved_length_by_kind_m'] == pytest.approx(dict(physical_boundary=2.313828773088,
                                                               movement_path=1.244474581153))
    assert len(s['external_heads']) == 2  # one boundary + a separate original movement path
    head = next(r for r in s['external_heads'] if r['kind'] == 'physical_boundary')
    assert head['length_m'] == pytest.approx(.00446297463)


def test_five_physical_tails_and_four_paths_have_explicit_consumers(report):
    tails = report['source']['tails']
    assert len(tails) == 9
    assert len([r for r in tails if r['kind'] == 'physical_boundary']) == 5
    assert all(r['consumers'] and all(c.startswith('connector:') for c in r['consumers']) for r in tails)
    middle = next(r for r in tails if r.get('edge') == 3)
    assert middle['consumers'] == ['connector:109', 'connector:110', 'connector:111']
    assert middle['parent_s_range'][1] > 217.7


def test_actual_six_connector_contacts_and_upstream_are_checked(report):
    d = report['dependency']
    assert len(d['edge_contacts']) == 24 and not d['failures']
    assert d['connectors'] == [str(i) for i in range(106, 112)]
    assert all(a['upstream_seam']['pass'] for a in report['anchors'] if a['upstream_seam'])
    assert report['frozen_upstream']['max_m'] == pytest.approx(.159511311534, abs=1e-8)
    assert all(a['derivative_class'] == 'numerical_zero' for a in report['anchors'] if a['s'] > 200)


def test_source_roles_retained_and_no_map_or_trial_approval(report):
    assert {b['source_lane_id'] for b in report['contract']['births']} == {'2023041111104128071', '2023041111104060474'}
    assert report['status'] == 'BLOCKED_E1' and report['blockers']
    assert not report['new_xodr'] and not report['web_changed'] and not report['map_accepted']
    assert report['optimizer_calls'] == 0 and not report['contract']['new_trial_registered']
    assert all(not a['full_anchor_admitted'] for a in report['anchors'])


def test_tails_compared_per_consumer_and_not_against_nearest_unrelated_road(report):
    rows = report['tail_geometry']['rows']
    assert len(rows) == 18  # two physical edges + one path for every explicit turn
    assert all(r['consumer'] in {'connector:'+str(i) for i in range(106,112)} for r in rows)
    assert not report['tail_geometry']['continuous_certificate']
    assert not report['tail_geometry']['acceptance_threshold_added']


@pytest.mark.parametrize('field', ['raw_vertices', 'atoms'])
def test_clipped_original_parts_rejected_even_if_rehashed(source, field):
    data, packet, domain, _ = source
    _, features = collect_sources(parse(data), packet)
    ledger = copy.deepcopy(domain['partition']); key = 'boundary:IBD_LANE_BOUNDARY:2023041110502726490'
    ledger['features'][key]['parts'][0][field].pop()
    with pytest.raises(ValueError, match='vertices|tail'):
        partition_rows(features, ledger)


def test_signed_but_reassigned_ledger_does_not_replay(source):
    data, packet, domain, decision = source
    changed = copy.deepcopy(domain)
    part = changed['partition']['features']['boundary:IBD_LANE_BOUNDARY:2023041110502726490']['parts'][0]
    part['atoms'][-1]['consumers'] = ['connector:106']
    changed['content_sha256'] = object_digest({k:v for k,v in changed.items() if k!='content_sha256'})
    with pytest.raises(ValueError, match='reproduces'):
        admit(data, packet, changed, decision)


def test_missing_role_approval_does_not_get_inferred(source):
    data, packet, domain, decision = source
    decision = copy.deepcopy(decision); decision['decisions'] = []
    with pytest.raises(ValueError, match='approval'):
        admit(data, packet, domain, decision)


def test_xml_or_packet_changes_are_not_new_scope_authorization(source):
    data, packet, domain, decision = source
    with pytest.raises(ValueError, match='Reference changed'):
        admit(data+b'\n', packet, domain, decision)
    changed = copy.deepcopy(packet); changed['mutable_roads'] = []
    with pytest.raises(ValueError, match='signature'):
        checked_payload(changed)


def test_numerical_zero_not_reverse_shape_and_world_checks_still_strict():
    assert slope_class(-1e-14) == 'numerical_zero'
    assert slope_class(-.0088) == 'decreasing'
    assert not world_delta((0,0,0,0), (.011,0,0,0))['pass']
    assert not world_delta((0,0,0,0), (0,0,0,2e-7))['pass']
    with pytest.raises(ValueError): slope_class(float('nan'))


def test_prefix_is_finite_first_crossing_not_extrapolation_or_later_nearest():
    curve = np.array([[0.,0.],[2.,1.],[4.,2.],[0.,3.]])
    result = prefix_to_plane(curve, 3.)
    np.testing.assert_allclose(result, [[0,0],[2,1],[3,1.5]])
    with pytest.raises(ValueError, match='reverses'):
        prefix_to_plane(curve, 5.)
    with pytest.raises(ValueError, match='never reaches'):
        prefix_to_plane(curve[:3], 5.)


def test_admission_never_calls_nonlinear_solver(source, monkeypatch):
    import scipy.optimize
    def forbidden(*args, **kwargs): raise AssertionError('E1 must not solve')
    for name in ('minimize', 'least_squares', 'linprog', 'root'):
        monkeypatch.setattr(scipy.optimize, name, forbidden)
    assert admit(*source)['optimizer_calls'] == 0
