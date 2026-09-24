import copy
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from pyclothoids import Clothoid

from scripts.prepare_coupled_event_contract import load_contract, response_check, SCOPE, ROLES, BOUND
from scripts.bind_split_event_sources import inputs
from mapforge.ops.reconstruction_scope import digest as object_digest
from mapforge.repair_web.coupled_event_contract import CoupledEventContract
from mapforge.repair_web.coupled_event_state import CoupledEventState, EventTurnBlock, long_basis, world_state
from mapforge.repair_web.split_event_admission import START, END
from spikes.connector_cross_section import endpoint_frame


@pytest.fixture(scope='module')
def contract(): return load_contract()


@pytest.fixture(scope='module')
def probe(contract): return response_check(contract)


def test_exact_scope_and_no_map_claim(contract):
    r = contract.describe()
    assert r['replacement_roads'] == ['11','106','107','108','109','110','111']
    assert r['parent_variables'] == 52 and r['parent_affine_freedom'] == 32
    assert r['shared_mouth_jet_rank'] == 10
    assert not r['initial_affine_origin_is_map'] and not r['whole_shape_or_fidelity_feasibility']
    assert not r['map_accepted'] and r['solver_calls'] == 0 and not r['new_xodr']


def test_exact_position_bounds_use_original_sources_and_explicit_sides(contract):
    from shapely.geometry import Point, LineString
    assert len(contract.port_sources) == 24
    for row in contract.port_sources:
        assert row['position_within_budget'] and row['position_budget_m'] == .35
        assert not row['independent_movement_tested']
        assert Point(row['world_point']).distance(LineString(row['complete_original_part_xy'])) == pytest.approx(row['distance_m'], abs=1e-12)
        assert row['source_side_mapping'] == {'left':'right','right':'left'}
    assert max(r['distance_m'] for r in contract.port_sources) == pytest.approx(.3499934337, abs=1e-9)


@pytest.mark.parametrize('edge', range(5))
def test_all_mouth_jets_are_shared_and_positions_fixed(contract, edge):
    p = contract.parent; requested = np.zeros(10); requested[2*edge] = 1e-4
    delta = p.Z@np.linalg.lstsq(contract.jet_matrix@p.Z, requested, rcond=None)[0]
    before = contract.frames(p.origin); after = contract.frames(p.origin+delta)
    expected = contract.describe()['edge_dependents'][str(edge)]; changed = []
    assert contract.jet_matrix@delta == pytest.approx(requested, abs=1e-11)
    for cid, frames in before.items():
        for side in ('left','right'):
            assert [after[cid][0]['edges'][side][k] for k in ('x','y')] == pytest.approx([frames[0]['edges'][side][k] for k in ('x','y')], abs=1e-11)
        if any(abs(after[cid][0]['edges'][side]['heading']-frames[0]['edges'][side]['heading']) > 1e-9 for side in ('left','right')):
            changed.append(cid)
        assert after[cid][1] == frames[1]
    assert changed == expected


def test_births_and_upstream_are_c2_equalities(contract):
    p = contract.parent; x = p.origin+p.Z@np.linspace(-.1,.1,p.Z.shape[1])
    for edge in (0,1,2):
        for d in range(3):
            assert p.row(edge, START, d)@x == pytest.approx(p.old_power(edge, START, True)[d]*math.factorial(d), abs=1e-10)
    for edge, s in p.births.items():
        for d in range(3): assert (p.row(edge,s,d)-p.row(edge-1,s,d))@x == pytest.approx(0.,abs=1e-10)


def test_semantic_short_records_are_reported_not_hidden(contract):
    layout = contract.written_layout()
    assert min(np.diff(contract.parent.scope.knots)) >= 8.33
    assert len(layout['short_semantic_records']) == 5
    assert layout['minimum_width_record_span_m'] == pytest.approx(1.0543)
    assert not layout['short_records_not_identical_to_existing']
    # A mandatory section cut at s130 re-expresses the SAME polynomial, rather
    # than fitting a new independent 1.05m piece before the s131.0543 birth.
    p = contract.parent; x = p.origin+p.Z@np.linspace(-.1,.1,p.Z.shape[1])
    for edge, a, b in ((1,120.,130.),(2,120.,130.),(3,140.,150.)):
        shifted = np.polynomial.Polynomial(p.power(edge,a)@x)(np.polynomial.Polynomial([b-a,1.])).coef
        assert p.power(edge,b)@x == pytest.approx(np.pad(shifted,(0,4-len(shifted))), abs=1e-11)


def test_prebirth_sources_are_not_projected_onto_mother(contract):
    for trace, a, b, _ in contract.parent.source_cells():
        assert a >= contract.parent.births.get(trace['edge'], START)
    r = contract.describe()['source_inventory']
    assert (r['physical_boundary_parts'],r['original_movement_paths']) == (25,19)
    assert r['source_interval_bindings'] == {'WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION':35,
        'PREBIRTH_ENDPOINT_OBSERVATION':2,'EXTERNAL_ENDPOINT_OBSERVATION':2}


def test_seven_road_probe_follows_shared_edge_not_independent_turn_fits(probe):
    assert probe['state_variables'] == 146
    assert probe['requested_actual_error'] < 1e-12
    assert [r['connector'] for r in probe['results'] if r['incoming_frame_changed']] == ['109','110','111']
    assert [r['connector'] for r in probe['results'] if r['transverse_controls_changed']] == ['109','110','111']
    for row in probe['results']:
        assert row['outgoing_frame_unchanged'] and row['minimum_span_m'] >= 6.
        assert row['primitive_count'] == row['transverse_span_count'] == 5
    # This coordinate is deliberately NOT polished into a solved candidate.
    assert max(np.linalg.norm(r['baseline_reference_closure'][:2]) for r in probe['results']) > 5.
    assert not probe['coordinate_probe_is_map'] and not probe['source_fidelity_evaluated']


@pytest.mark.parametrize('lengths', [[6]*4,[6]*4+[5.999],[6]*4+[float('nan')]])
def test_short_or_corrupt_turn_layout_refused(lengths):
    with pytest.raises(ValueError): long_basis(lengths)


def test_exact_straight_ribbon_synthetic_control():
    road = ET.fromstring('<road id="t" length="40"><planView><geometry s="0" x="0" y="0" hdg="0" length="40"><line/></geometry></planView></road>')
    def frame(x):
        return dict(pose=(x,0.,0.),k=0.,dk=0.,edges=dict(left=dict(x=x,y=2.,heading=0.,curvature=0.),
            right=dict(x=x,y=-2.,heading=0.,curvature=0.)),center=dict(x=x,y=0.,heading=0.,curvature=0.))
    block = EventTurnBlock(road, (frame(0.),frame(40.)))
    result = block.evaluate(block.diagnostic,(frame(0.),frame(40.)))
    assert result['reference_closure'] == pytest.approx(np.zeros(4),abs=1e-12)
    for row in result['joins']+result['contacts']: assert row['delta'] == pytest.approx(np.zeros(4),abs=1e-12)
    assert result['minimum_width_m'] == pytest.approx(4.)
    assert result['minimum_forward_factor'] == pytest.approx(1.)


def test_world_curvature_compared_with_independent_coordinate_differentiation():
    ref = Clothoid.StandardParams(1.,-2.,.3,.01,.0004,20.)
    co = np.array([2.,.03,-.0003,.00002]); u = 7.; h = .005
    points = [world_state(ref,co,u+k*h)[:2] for k in (-2,-1,0,1,2)]
    tangent = (points[0]-8*points[1]+8*points[3]-points[4])/(12*h)
    second = (-points[0]+16*points[1]-30*points[2]+16*points[3]-points[4])/(12*h*h)
    k = (tangent[0]*second[1]-tangent[1]*second[0])/np.linalg.norm(tangent)**3
    state = world_state(ref,co,u)
    assert state[2] == pytest.approx(math.atan2(tangent[1],tangent[0]),abs=1e-9)
    assert state[3] == pytest.approx(k,abs=1e-8)


def test_fixed_frames_returned_without_mutable_alias(contract):
    a = contract.frames(contract.parent.origin); a['106'][1]['edges']['left']['x'] = 1e9
    assert contract.frames(contract.parent.origin)['106'][1]['edges']['left']['x'] != 1e9


def test_zero_operation_byte_replay_and_solve_write_blocked(contract):
    data,*_ = inputs(); assert contract.unchanged_xml() == data
    for obj in (contract.parent, CoupledEventState(contract)):
        for method in (obj.solve,obj.compile):
            with pytest.raises(ValueError): method()


@pytest.mark.parametrize('change', ['remove_turn','move_mouth','change_speed','add_role','solve','source_side'])
def test_scope_source_or_authority_changes_rejected(change):
    data,p,d,*_ = inputs(); b = json.loads(BOUND.read_bytes())
    s = yaml.safe_load(SCOPE.read_text(encoding='utf-8')); r = yaml.safe_load(ROLES.read_text(encoding='utf-8'))
    if change == 'remove_turn': s['connectors'].pop()
    if change == 'move_mouth': s['incoming_mouth']['positions_and_cross_section'] = 'free'
    if change == 'change_speed': s['source_speed_changes'] = 1
    if change == 'add_role': r['decisions'].append(copy.deepcopy(r['decisions'][0]))
    if change == 'solve': s['execution']['optimizer_authorized_by_this_file'] = True
    if change == 'source_side':
        p['observations']['2023041111104136051']['boundary_relations'][0]['declared_side'] = 'right'
        p['content_sha256'] = object_digest({k:v for k,v in p.items() if k!='content_sha256'})
    with pytest.raises(ValueError): CoupledEventContract(data,p,d,b,s,r)


def test_no_optimizer_called_for_common_state(monkeypatch):
    import scipy.optimize as opt
    import mapforge.ops.long_connector_chain as lc
    import mapforge.ops.joint_connector_fit as jc
    def forbidden(*a,**kw): raise AssertionError('Unregistered fit')
    for name in ('minimize','least_squares','linprog','root'): monkeypatch.setattr(opt,name,forbidden)
    monkeypatch.setattr(lc,'root',forbidden); monkeypatch.setattr(jc,'fit_joint',forbidden)
    r = response_check(load_contract())
    assert r['optimizer_calls'] == 0 and not r['actual_xml_written']


def test_nonfinite_and_frozen_position_violation_refused(contract):
    x = contract.parent.origin.copy(); x[0] += 1.
    with pytest.raises(ValueError): contract.frames(x)
    model = CoupledEventState(contract); v = model.diagnostic.copy(); v[-1] = float('nan')
    with pytest.raises(ValueError): model.evaluate(v)
