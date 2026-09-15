import copy
import json
import xml.etree.ElementTree as ET

import pytest

from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.reconstruction_scope import digest, prepare_scope
from mapforge.ops.source_domain import (original_movements, compare_movements, clipped_intervals,
                                       linear_chart, prepare_source_domain, require_domain_replay)
from tests.test_reconstruction_scope import fixture


def domain_fixture():
    root, src, first = fixture()
    # One two-lane ingress, a one-lane exit, two original movements. Fixtures
    # are fixed independently of later mutations to the XODR under test.
    src.p = {'lane_identity': 'primary-with-supplement-records'}
    src.L = dict(src.L, junction={'file': 'J', 'fields': {
        'id': 'id', 'enter_roads': 'enter', 'leave_roads': 'leave'}},
        lane={'file': 'L', 'fields': {'id': 'id', 'road': 'road'}},
        road_center={'file': 'R', 'interior_values': [1], 'fields': {
            'road': 'road', 'is_junction_interior': 'interior'}},
        topo={'file': 'T', 'fields': {'from': 'a', 'to': 'b'}})
    def row(layer, i, attrs):
        return {'layer': layer, 'record_index': i, 'attributes': attrs, 'parts': ()}
    src.raw = {'junction': [row('J', 0, {'id': 'original-j', 'enter': 'in', 'leave': 'out'})],
               'lane': [], 'topo': [], 'road_center': [row('R', 0, {'road': 'inside', 'interior': 1})]}
    # source identities match the synthetic source but are recorded now, not
    # re-derived from an XODR with deleted or misconnected movements.
    for sid, road in [('10:-1', 'in'), ('10:-2', 'in'), ('11:-1', 'out'),
                      (first+':-1', 'inside'), ('second:-1', 'inside')]:
        src.raw['lane'].append(row('L', len(src.raw['lane']), {'id': sid, 'road': road}))
    for a, bs in src.topo_out.items():
        for b in bs: src.raw['topo'].append(row('T', len(src.raw['topo']), {'a': a, 'b': b}))
    src.layer_raw_records = lambda name: copy.deepcopy(src.raw[name])
    src.fresh_reader = lambda: copy.deepcopy(src)
    return root, src, first


def test_original_inventory_and_existing_graph_are_independent():
    root, src, first = domain_fixture()
    expected = original_movements(src, 'original-j')
    assert len(expected['paths']) == 2 and not expected['issues']
    jid = root.find('junction').get('id')
    assert compare_movements(root, src, expected, jid)['status'] == 'MATCH'
    root.remove(root.find("road[@id='second']"))
    root.find('junction').remove(root.find("junction/connection[@id='second']"))
    assert PortDependencies(root).validate_junction_table()  # old check misses this!
    compared = compare_movements(root, src, original_movements(src, 'original-j'), jid)
    assert compared['expected_count'] == 2 and compared['written_count'] == 1
    assert len(compared['missing']) == 1 and compared['status'] == 'REJECTED'


def test_same_count_and_via_ids_do_not_hide_wrong_parent_lane():
    root, src, first = domain_fixture()
    # Swap the two ingress lane links; both counts and via ID set are unchanged.
    for c in root.findall('junction/connection'):
        val = c.find('laneLink').get('from')
        c.find('laneLink').set('from', '-2' if val == '-1' else '-1')
        lane = root.find("road[@id='"+c.get('connectingRoad')+"']/lanes/laneSection/right/lane")
        lane.find('link/predecessor').set('id', '-2' if val == '-1' else '-1')
    compared = compare_movements(root, src, original_movements(src, 'original-j'), root.find('junction').get('id'))
    assert compared['expected_count'] == compared['written_count'] == 2
    assert compared['status'] == 'REJECTED' and len(compared['missing']) == 2


@pytest.mark.parametrize('fault', ['dead', 'cycle', 'dangling', 'duplicate', 'outside', 'identity'])
def test_original_faults_never_silently_shrink_expected_inventory(fault):
    root, src, first = domain_fixture()
    rows = src.raw['topo']
    mid = next(r for r in rows if r['attributes']['a'] == first+':-1')
    if fault == 'dead': rows.remove(mid)
    elif fault == 'cycle': mid['attributes']['b'] = '10:-1'
    elif fault == 'dangling': mid['attributes']['b'] = 'absent'
    elif fault == 'duplicate': rows.append(copy.deepcopy(mid))
    elif fault == 'outside': src.raw['road_center'][0]['attributes']['interior'] = 0
    elif fault == 'identity': src.raw['lane'].append(copy.deepcopy(src.raw['lane'][0]))
    inv = original_movements(src, 'original-j')
    assert inv['issues']
    assert compare_movements(root, src, inv, root.find('junction').get('id'))['status'] == 'REJECTED'


def test_explicit_classification_identity_and_traversal_completion_required():
    _, src, _ = domain_fixture()
    with pytest.raises(ValueError, match='identity'): original_movements(src, 'wrong')
    with pytest.raises(ValueError, match='budget'): original_movements(src, 'original-j', budget=1)
    del src.L['road_center']['interior_values']
    with pytest.raises(ValueError, match='classification'): original_movements(src, 'original-j')


def test_exact_intervals_keep_vertices_and_reverse_or_recrossing_parts():
    # Clip 3<=x<=7, cuts must fall inside original segments, not sampled indices.
    s, intervals = clipped_intervals([[0, 0], [10, 0]], [(1, 0, 3), (-1, 0, -7)])
    assert s == [0., 10.] and intervals == [[3., 7.]]
    _, reverse = clipped_intervals([[10, 0], [0, 0]], [(1, 0, 3), (-1, 0, -7)])
    assert reverse == [[3., 7.]]
    s, recross = clipped_intervals([[0, 0], [10, 0], [0, 0]], [(1, 0, 3), (-1, 0, -7)])
    assert s == [0., 10., 20.] and recross == [[3., 7.], [13., 17.]]
    s, duplicate = clipped_intervals([[0, 0], [0, 0], [10, 0]])
    assert s == [0., 0., 10.] and duplicate == [[0., 10.]]


def test_branched_multiple_interior_features_match_complete_original_paths():
    root, src, first = domain_fixture()
    # Three source movements; first traverses two interior features after a
    # legal branch. Exported first road names only its principal via feature.
    for row in src.raw['topo']:
        if row['attributes'] == {'a': '10:-1', 'b': first+':-1'}:
            row['attributes']['b'] = 'pre'
    src.raw['lane'].append({'layer': 'L', 'record_index': 9, 'parts': (),
                            'attributes': {'id': 'pre', 'road': 'inside'}})
    for a, b in [('pre', first+':-1'), ('10:-1', 'second:-1')]:
        src.raw['topo'].append({'layer': 'T', 'record_index': len(src.raw['topo']),
                                'parts': (), 'attributes': {'a': a, 'b': b}})
    new = copy.deepcopy(root.find("road[@id='second']")); new.set('id', 'third')
    new.find('lanes/laneSection/right/lane/link/predecessor').set('id', '-1'); root.append(new)
    connection = copy.deepcopy(root.find("junction/connection[@id='second']"))
    connection.set('id', 'third'); connection.set('connectingRoad', 'third')
    connection.find('laneLink').set('from', '-1'); root.find('junction').append(connection)
    inv = original_movements(src, 'original-j')
    result = compare_movements(root, src, inv, root.find('junction').get('id'))
    assert len(inv['paths']) == 3 and not inv['issues']
    assert result['status'] == 'MATCH' and result['resolved_count'] == 3
    p = prepare_scope(root, src, ['10'], {'source_junction_id': 'original-j',
                                        'xodr_junction_id': root.find('junction').get('id')})
    assert p['source_admission'] == 'ADMITTED_FOR_RESEARCH'
    assert any(link.get('source_path') == ['10:-1', 'pre', first+':-1'] for link in p['source_links'])


def test_domain_replay_reopens_original_rows_instead_of_reusing_cache():
    root, src, _ = domain_fixture()
    header = ET.SubElement(root, 'header')
    ET.SubElement(header, 'geoReference').text = '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m'
    old_rows = copy.deepcopy(src.raw)
    src.layer_raw_records = lambda name: copy.deepcopy(old_rows[name])
    def reopen():
        new = copy.deepcopy(src)
        new.layer_raw_records = lambda name: copy.deepcopy(src.raw[name])
        return new
    src.fresh_reader = reopen
    scope = prepare_scope(root, src, ['10'])
    domain = prepare_source_domain(root, src, scope, 'original-j', root.find('junction').get('id'))
    assert require_domain_replay(domain, root, src, scope)
    src.raw['topo'].pop()
    with pytest.raises(ValueError, match='fresh original inputs'):
        require_domain_replay(domain, root, src, scope)


def test_unsupported_curved_chart_must_not_be_approximated_as_straight():
    root, _, _ = domain_fixture(); road = root.find("road[@id='10']")
    assert linear_chart(road)['length'] > 0
    g = road.find('planView/geometry'); g.remove(g.find('line')); ET.SubElement(g, 'arc', curvature='.01')
    with pytest.raises(ValueError, match='exact Line'): linear_chart(road)


def test_full_parts_accounting_and_self_rehashed_omission_is_detected():
    root, src, _ = domain_fixture()
    header = root.find('header')
    if header is None: header = ET.SubElement(root, 'header')
    ET.SubElement(header, 'geoReference').text = '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m'
    scope = prepare_scope(root, src, ['10'])
    packet = prepare_source_domain(root, src, scope, 'original-j', root.find('junction').get('id'))
    before = ET.tostring(root)
    assert require_domain_replay(packet, root, src, scope)
    features = packet['partition']['features']
    assert len(features) == len(scope['observations']) + len(scope['boundaries'])
    for f in features.values():
        for p in f['parts']:
            assert len(p['vertices']) == len(p['raw_vertices'])
            assert sum(a['source_s_m'][1]-a['source_s_m'][0] for a in p['atoms']) == pytest.approx(p['length_m'])
    assert packet['status'] == 'BLOCKED' and not packet['export_allowed']
    assert ET.tostring(root) == before
    # An attacker/bug cannot remove an inconvenient unassigned feature and
    # simply update its content hash to turn it into a replayed report.
    features.pop(next(iter(features)))
    packet['content_sha256'] = digest({k: v for k, v in packet.items() if k != 'content_sha256'})
    with pytest.raises(ValueError, match='fresh original inputs'):
        require_domain_replay(packet, root, src, scope)


def test_multipart_has_separate_local_arclength_no_synthetic_bridge():
    root, src, _ = domain_fixture()
    header = ET.SubElement(root, 'header')
    ET.SubElement(header, 'geoReference').text = '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m'
    old = src.lane_boundary_records
    def with_parts(sid):
        rows = list(copy.deepcopy(old(sid)))
        rows[0]['records'][0]['parts'] = (((0., 1.), (1., 1.)), ((9., 1.), (10., 1.)))
        return tuple(rows)
    src.lane_boundary_records = with_parts
    p = prepare_scope(root, src, ['10'])
    result = prepare_source_domain(root, src, p, 'original-j', root.find('junction').get('id'))
    parts = result['partition']['features']['boundary:B:shared']['parts']
    assert len(parts) == 2 and all(v['source_vertex_s_m'][0] == 0 for v in parts)
    assert sum(len(v['vertices']) for v in parts) == 4


@pytest.mark.parametrize('suffix', ['+proj=utm', '+x_0=1000', '+R=1', '+lat_ts=20'])
def test_projection_changes_cannot_silently_change_source_accounting(suffix):
    root, src, _ = domain_fixture()
    header = ET.SubElement(root, 'header')
    ET.SubElement(header, 'geoReference').text = (
        '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m '+suffix)
    with pytest.raises(ValueError, match='projection'):
        prepare_source_domain(root, src, prepare_scope(root, src, ['10']),
                              'original-j', root.find('junction').get('id'))
    # NaN comparisons can otherwise slip through abs(value-expected)>tol.
    header.find('geoReference').text = '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=nan +units=m'
    with pytest.raises(ValueError, match='projection'):
        prepare_source_domain(root, src, prepare_scope(root, src, ['10']),
                              'original-j', root.find('junction').get('id'))
