"""Necessary immutable-tail evidence is distinct from interval/map acceptance."""
import copy
import json
import math

import numpy as np
import pytest
from shapely.geometry import Point, LineString

from mapforge.repair_web.event_shape_admission import (
    point_to_original, check_frozen_points, dependency_witnesses, TAIL_BUDGET_M,
)
from mapforge.repair_web.model import parse
from scripts.review_event_shape_admission import load_report, PRIOR
from scripts.bind_split_event_sources import inputs
from scripts.internal_edge_jets import states


@pytest.fixture(scope='module')
def report():
    return load_report()


def test_point_to_finite_original_segments_not_extrapolated_axis():
    raw = [[0., 0.], [1., 0.], [1., 2.]]
    r = point_to_original([2., 1.], raw)
    assert r['distance_m'] == 1 and r['source_segment'] == 1
    assert r['segment_fraction'] == .5
    assert point_to_original([-1., 0.], raw)['distance_m'] == 1


@pytest.mark.parametrize('raw', [[], [[0, 0]], [[0, 0], [0, 0]], [[0, 0], [1, float('nan')]]])
def test_bad_original_parts_rejected(raw):
    with pytest.raises(ValueError): point_to_original([0., 0.], raw)


def test_finite_points_cannot_certify_continuous_pass():
    r = check_frozen_points([[0., .1], [1., .2]], [[0., 0.], [1., 0.]], .35)
    assert r['status'] == 'NO_VIOLATION_FOUND_NOT_PASS'
    assert not r['whole_interval_pass'] and not r['continuous_maximum_proven']


def test_single_immutable_violation_is_sufficient_to_block():
    r = check_frozen_points([[.5, .4]], [[0, 0], [1, 0]], .35)
    assert r['status'] == 'FIXED_POINT_VIOLATES_SOURCE_BOUND'
    assert r['exceeds_m'] == pytest.approx(.05)


def test_real_six_dependencies_all_edges_present(report):
    assert len(report['witnesses']) == 12
    assert report['failed_connectors'] == ['106', '107', '110', '111']
    assert report['failed_edge_consumer_count'] == 5
    assert report['tail_budget_m'] == TAIL_BUDGET_M == .35
    assert report['status'] == 'BLOCKED_FIXED_DEPENDENCY_SOURCE_CONFLICT'


def test_full_source_not_clipped_and_shapely_independent_distances(report):
    for r in report['witnesses']:
        assert r['source_not_cropped'] and not r['independent_movement_tested']
        assert r['original_full_part_st'][0][0] < 200
        w = r['witness']; raw = LineString(r['original_full_part_st'])
        assert Point(w['st']).distance(raw) == pytest.approx(w['distance_m'], abs=1e-12)
        if r['status'] == 'FIXED_POINT_VIOLATES_SOURCE_BOUND':
            # None of these failures is caused by comparison against a cut endpoint.
            assert 0 < w['segment_fraction'] < 1
            assert w['closest_original_st'][0] > report['proposed_edit_domain'][1]


def test_actual_witness_is_direct_primitive_not_mesh_interpolation(report):
    from scipy.integrate import quad
    data, *_ = inputs(); root = parse(data)
    road = root.find("road[@id='111']")
    r = next(r for r in report['witnesses'] if r['consumer'] == 'connector:111' and r['field'] == 'right')
    w = r['witness']; s = w['connector_s_m']
    g = max((g for g in road.findall('planView/geometry') if float(g.get('s')) <= s), key=lambda g: float(g.get('s')))
    spiral = g.find('spiral'); assert spiral is not None
    u = s-float(g.get('s')); h = float(g.get('hdg')); k = float(spiral.get('curvStart'))
    dk = (float(spiral.get('curvEnd'))-k)/float(g.get('length'))
    theta = lambda v: h+k*v+dk*v*v/2
    x = float(g.get('x'))+quad(lambda v: math.cos(theta(v)), 0, u, epsabs=1e-12)[0]
    y = float(g.get('y'))+quad(lambda v: math.sin(theta(v)), 0, u, epsabs=1e-12)[0]
    def poly(items, attr):
        el = max((e for e in items if float(e.get(attr)) <= s), key=lambda e: float(e.get(attr)))
        v = s-float(el.get(attr))
        return sum(float(el.get(key))*v**j for j, key in enumerate('abcd'))
    t = poly(road.findall('lanes/laneOffset'), 's')-poly(road.findall('lanes/laneSection/right/lane/width'), 'sOffset')
    xy = [x-t*math.sin(theta(u)), y+t*math.cos(theta(u))]
    assert xy == pytest.approx(w['xy'], abs=1e-10)
    assert w['distance_m'] == pytest.approx(.5541034189, abs=1e-8)


def test_parent_only_changes_do_not_change_frozen_witness(report):
    data, *_ = inputs(); root = parse(data); changed = copy.deepcopy(root)
    changed.find("road[@id='11']/lanes/laneOffset").set('a', '999')
    for r in report['witnesses']:
        rid = r['consumer'].split(':')[1]; s = r['witness']['connector_s_m']
        before = states(root.find(f"road[@id='{rid}']"), -1, s, False)
        after = states(changed.find(f"road[@id='{rid}']"), -1, s, False)
        assert after == before


def test_no_authorized_mouth_or_source_role_or_map_change(report):
    assert not report['new_xodr'] and not report['map_accepted'] and not report['web_changed']
    assert not report['all_source_coverage_accepted'] and report['solver_calls'] == 0
    proposal = report['next_scope_proposal']
    assert proposal['status'] == 'PROPOSAL_NOT_AUTHORIZED'
    assert not proposal['mouth_movement_authorized'] and proposal['keep_mouth_fixed_first']
    assert proposal['source_role_changes'] == 0


def test_changed_packet_or_xml_refused():
    data, packet, domain, *_ = inputs()
    bound = json.loads((PRIOR/'source-bindings.json').read_bytes())
    with pytest.raises(ValueError, match='revision'):
        dependency_witnesses(data+b' ', packet, domain, bound)
    packet['occurrences'][0]['road'] = 'fake'
    with pytest.raises(ValueError, match='signature'):
        dependency_witnesses(data, packet, domain, bound)


def test_admission_does_not_call_optimizer_or_mesh_sampler(monkeypatch):
    import scipy.optimize
    import mapforge.validate.smoothness as smoothness
    def forbidden(*args, **kwargs): raise AssertionError('Must only evaluate actual immutable points')
    for name in ('minimize', 'linprog', 'least_squares', 'root'):
        monkeypatch.setattr(scipy.optimize, name, forbidden)
    monkeypatch.setattr(smoothness, 'sample_road_ref', forbidden)
    assert load_report()['solver_calls'] == 0
