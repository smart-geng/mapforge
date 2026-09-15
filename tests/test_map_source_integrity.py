import copy
from pathlib import Path

import numpy as np
import pytest

from mapforge.adapters.v2xmap.xml_reader import MapLane, MapLink, MapNode, parse_map_xml
from mapforge.adapters.v2xmap.xml_writer import node_to_xml
from mapforge.ops.map_to_xodr import _georef, _project, _reference_support, build_xodr
from mapforge.validate.g8_model import make_manifest, source_lane
from mapforge.validate.map_source import audit_map_source_manifest


def test_reference_support_preserves_link_and_uses_common_section_offset():
    link = np.array([[0., 0.], [0., 40.]])
    lane = np.array([[3., -200.], [2., -70.], [2., 0.], [2., 40.]])
    before = lane.copy()
    support, meta = _reference_support(link, [(1, lane)])
    np.testing.assert_array_equal(support[-2:], link)
    np.testing.assert_array_equal(lane, before)
    np.testing.assert_allclose(support[0], [1., -200.])
    assert meta['longitudinal_extension_m'] == 200
    assert meta['status'] == 'INFERRED'
    assert meta['source_lane_id'] == 1


@pytest.mark.parametrize('points', [
    [[2., 40.], [2., -200.]],                 # opposite direction
    [[2., -200.], [2., -90.]],                # no common section
    [[2., -200.], [2., -220.], [2., 40.]],   # folds back
    [[50., -200.], [50., 40.]],              # disconnected laterally
])
def test_invalid_prefix_not_silently_used(points):
    link = np.array([[0., 0.], [0., 40.]])
    support, meta = _reference_support(link, [(1, points)])
    np.testing.assert_array_equal(support, link)
    assert meta['prefix_point_count'] == 0
    assert meta['rejected']


@pytest.fixture
def raw_case(tmp_path):
    lane = MapLane(1, 350, None, points=[(106., 29.998), (106., 30.), (106., 30.0004)])
    link = MapLink('north', (500, 2), 350, points=lane.points[1:], lanes=[lane])
    node = MapNode('test', 500, 1, 106., 30., links=[link])
    path = tmp_path/'source.xml'
    path.write_text(node_to_xml(node), encoding='utf-8')
    node = parse_map_xml(str(path))
    points = _project(node.links[0].lanes[0].points, 30., 106.)
    rec = source_lane('map:500:1:from:500:2:north:lane:1', points,
                      owner={'region': 500, 'node': 1, 'upstream': [500, 2], 'link': 'north', 'lane': 1},
                      role='approach', status='TRANSFORMED', support_kind='map-lane-point-list',
                      policy_class='map.point-list-approach', travel_direction='with_s')
    manifest = make_manifest(source_format='map', source_profile='test', lanes=[rec],
                            comparison_crs={'units': 'm', 'origin': {'lon': 106., 'lat': 30.},
                                            'proj_string': _georef(30., 106.)},
                            source_contexts=[{'region': 500, 'node_id': 1, 'role': 'main'}])
    return node, path, manifest


def test_raw_manifest_gate_passes_only_complete_points(raw_case):
    _, path, manifest = raw_case
    result = audit_map_source_manifest(manifest, [path])
    assert result['status'] == 'PASS', result
    clipped = copy.deepcopy(manifest)
    clipped['lanes'][0]['geometry']['coordinates'] = clipped['lanes'][0]['geometry']['coordinates'][1:]
    result = audit_map_source_manifest(clipped, [path])
    assert result['status'] == 'FAIL'
    assert result['failure_reasons'][0]['code'] == 'raw_geometry_changed_or_clipped'


def test_missing_lane_and_reordered_vertices_fail(raw_case):
    _, path, manifest = raw_case
    reordered = copy.deepcopy(manifest)
    reordered['lanes'][0]['geometry']['coordinates'].reverse()
    assert audit_map_source_manifest(reordered, [path])['status'] == 'FAIL'
    manifest['lanes'] = []
    result = audit_map_source_manifest(manifest, [path])
    assert result['failure_reasons'][0]['code'] == 'raw_lane_missing_from_manifest'


def test_raw_input_missing_does_not_reuse_manifest_as_source(raw_case):
    _, _, manifest = raw_case
    assert audit_map_source_manifest(manifest)['status'] == 'UNAVAILABLE'


def test_builder_preserves_raw_points_and_extends_road(raw_case, tmp_path):
    from lxml import etree
    node, path, _ = raw_case
    out = tmp_path/'full.xodr'
    result = build_xodr(node, out)
    manifest = result['source_lane_manifest']
    assert audit_map_source_manifest(manifest, [path])['status'] == 'PASS'
    assert len(manifest['lanes'][0]['geometry']['coordinates']) == 3
    assert float(etree.parse(str(out)).find('road').get('length')) > 265.


def test_unmodeled_link_stays_in_source_manifest(raw_case, tmp_path):
    node, _, _ = raw_case
    node.links[0].points = []
    result = build_xodr(node, tmp_path/'missing.xodr')
    assert result['unmodeled_link_count'] == 1
    assert len(result['source_lane_manifest']['lanes']) == 1
    assert len(result['source_lane_manifest']['lanes'][0]['geometry']['coordinates']) == 3


def test_map_delivery_requires_raw_integrity(build_g8_case, g8_policy):
    from mapforge.report.decision import finalize_opendrive_g8
    out, manifest = build_g8_case()
    manifest['source_format'] = 'map'
    manifest['comparison_crs']['integrity'] = 'verified'
    g8_policy['applicability']['source_formats'].append('map')
    final = finalize_opendrive_g8(out, manifest, g8_policy)
    assert final['gate']['status'] == 'PASS'
    assert final['quality']['gates']['G8-source-integrity']['status'] == 'UNAVAILABLE'
    assert final['decision']['status'] == 'BLOCKED'
    assert 'G8-source-integrity' in final['decision']['required_gates']


def test_simple_coordinate_spine_does_not_crop_lane_domain():
    from spikes.map_full_source_candidate import straight_coordinate_spine
    link = np.array([[0., 0.], [0., 40.]])
    support = np.vstack([[1., -200.], link])
    pv, error, _ = straight_coordinate_spine(link, support)
    assert len(pv.segs) == 1 and pv.segs[0].length == 240.
    assert error == 0
    np.testing.assert_allclose([pv.x0, pv.y0], [0., -200.])
    # A curved measured Link cannot take this shortcut.
    bent = np.array([[0., 0.], [2., 20.], [0., 40.]])
    assert straight_coordinate_spine(bent, np.vstack([[1., -200.], bent])) is None


@pytest.mark.parametrize('reference_left_shift_m', [0., 15.])
def test_real_departure_extends_common_road_without_mirroring(raw_case, tmp_path, monkeypatch,
                                                            reference_left_shift_m):
    from lxml import etree
    from mapforge.adapters.v2xmap.xml_reader import Connection
    from mapforge.validate.lane_fidelity import evaluate_g8
    node, _, _ = raw_case
    incoming = node.links[0].lanes[0]
    incoming.points = node.links[0].points.copy()  # Only 44m of inbound support.
    incoming.connects = [Connection(500, 2, 1, None, None)]
    departure = MapLane(1, 350, None, points=[(105.99992, 30.0004), (105.99992, 29.998)])
    exit_link = MapLink('return', (500, 1), 350, points=departure.points.copy(), lanes=[departure])
    neighbor = MapNode('neighbor', 500, 2, 106., 29.998, links=[exit_link])
    before = copy.deepcopy((node, neighbor))
    if reference_left_shift_m:
        from mapforge.ops import map_to_xodr as converter
        original_fit = converter.fit_leg_refline
        def shifted_axis(*args, **kwargs):
            pv, dev, smooth = original_fit(*args, **kwargs)
            pv.x0 -= reference_left_shift_m
            return pv, dev, smooth
        monkeypatch.setattr(converter, 'fit_leg_refline', shifted_axis)
    out = tmp_path/'real-long.xodr'
    result = build_xodr(node, out, neighbors=[neighbor])
    assert result['exit_real'] == 1 and result['exit_mirror'] == 0
    assert float(etree.parse(str(out)).find('road').get('length')) > 265.
    source = next(x for x in result['source_lane_manifest']['lanes'] if x['role'] == 'departure')
    np.testing.assert_allclose(source['geometry']['coordinates'], _project(departure.points, 30., 106.))
    assert source['travel']['target_direction'] == 'against_s'
    # Physical left is relative to laneOffset; the selected departure must not
    # disappear just because a valid coordinate reference makes its t negative.
    root = etree.parse(str(out))
    assert any(e.get('value') == source['source_lane_id'] for e in
               root.findall("road/lanes/laneSection/left/lane/userData[@code='mapforge.source_lane']"))
    assert result['reference_support_extensions'][0]['source_lane_id'] == source['source_lane_id']
    assert (node, neighbor) == before  # Supporting points were reversed only in a temporary array.
    policy = Path(__file__).resolve().parents[1]/'profiles/validation/g8-opendrive-jinfeng-v1.yaml'
    gate = evaluate_g8(out, result['source_lane_manifest'], policy)
    assert gate['status'] == 'PASS', gate.get('failure_reasons')


def test_raw_neighbor_cannot_be_hidden_by_omitting_context(raw_case, tmp_path):
    from mapforge.adapters.v2xmap.xml_reader import Connection
    node, path, manifest = raw_case
    node.links[0].lanes[0].connects = [Connection(500, 2, 1, None, None)]
    path.write_text(node_to_xml(node), encoding='utf-8')
    lane = MapLane(1, 350, None, points=[(105.99992, 30.0004), (105.99992, 29.998)])
    neighbor = MapNode('neighbor', 500, 2, 106., 29.998,
                       links=[MapLink('return', (500, 1), 350, points=lane.points, lanes=[lane])])
    other = tmp_path/'neighbor.xml'; other.write_text(node_to_xml(neighbor), encoding='utf-8')
    gate = audit_map_source_manifest(manifest, [path, other])
    assert gate['status'] == 'FAIL'
    assert any(x['code'] == 'raw_lane_missing_from_manifest' for x in gate['failure_reasons'])
