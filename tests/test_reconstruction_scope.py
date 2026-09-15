import copy
import json
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.reconstruction_scope import prepare_scope, require_solver_input, digest
from mapforge.ops.reconstruction_scope import source_chain
from tests.test_port_dependencies import network


def shape(points, parts=(0,)):
    return SimpleNamespace(points=points, parts=parts)


def raw_source():
    src = object.__new__(ProfileSource)
    src.L = {'lane': {'file': 'L', 'fields': {'id': 'I'}},
             'boundary': {'file': 'B', 'fields': {'id': 'I'}},
             'lane_boundary_rel': {'file': 'R', 'fields': {'lane': 'L', 'boundary': 'B', 'side': 'S'}}}
    rows = {'L': [({'I': 'x'}, shape([(0, 0), (2, 0)]))],
            'B': [({'I': 'edge'}, shape([(0, 1), (1, 1), (9, 1), (10, 1)], (0, 2)))],
            'R': [({'L': 'x', 'B': 'edge', 'S': 1}, None),
                  ({'L': 'y', 'B': 'edge', 'S': 2}, None),
                  ({'L': 'x', 'B': 'absent', 'S': 99}, None)]}
    src._iter = lambda spec, with_shape=True: iter(rows[spec['file']])
    return src, rows


def test_raw_boundary_api_preserves_identity_unknown_sides_and_multipart():
    src, _ = raw_source()
    x = src.lane_boundary_records('x')
    y = src.lane_boundary_records('y')
    assert x[0]['records'] == y[0]['records']
    assert len(x[0]['records'][0]['parts']) == 2
    assert x[1]['declared_side'] is None and not x[1]['records']
    assert x[0]['relation_index'] == 0 and y[0]['relation_index'] == 1
    x[0]['records'][0]['attributes']['I'] = 'tampered'
    assert src.lane_boundary_records('x')[0]['records'][0]['attributes']['I'] == 'edge'


def test_raw_api_does_not_silently_resolve_duplicate_ids():
    src, rows = raw_source()
    rows['B'].append(({'I': 'edge'}, shape([(0, 2), (1, 2)])))
    rows['L'].append(({'I': 'x'}, shape([(0, 3), (1, 3)])))
    assert len(src.lane_boundary_records('x')[0]['records']) == 2
    assert len(src.lane_raw_records('x')) == 2
    a = src.lane_raw_records('x')
    a[0]['attributes']['I'] = 'edited'
    assert src.lane_raw_records('x')[0]['attributes']['I'] == 'x'
    assert src._raw_parts(SimpleNamespace(points=[(1., 2.)])) == (((1., 2.),),)


class Source:
    L = {'boundary': {'file': 'B'}}
    def __init__(self):
        self.topo_out = {}

    def lane(self, sid):
        return SimpleNamespace(geometry=np.array([[0., 0.], [10., 0.]]),
                               max_speed_kmh=60., s_width_mm=3500, e_width_mm=3500,
                               s_width_known=True, e_width_known=True)

    def lane_raw_records(self, sid):
        return ({'source_lane_id': sid, 'layer': 'L', 'record_index': 0,
                 'geometry_origin': 'field', 'parts': (((0., 0.), (10., 0.)),)},)

    def lane_boundary_records(self, sid):
        return tuple({'boundary_id': bid, 'declared_side': side, 'raw_side': str(i),
                      'relation_index': i, 'relation_layer': 'R', 'source_lane_id': sid,
                      'records': ({'boundary_id': bid, 'layer': 'B', 'record_index': i,
                                   'parts': (((0., 1.), (10., 1.)),)},)}
                     for i, (bid, side) in enumerate([('shared', 'left'), (sid, 'right')]))


def fixture():
    root, first = network()
    src = Source()
    for road in root.findall('road'):
        for si, sec in enumerate(road.findall('lanes/laneSection')):
            for lane in sec.findall('right/lane') + sec.findall('left/lane'):
                for u in list(lane.findall('userData')):
                    lane.remove(u)
                sid = road.get('id') + ':' + lane.get('id')
                ET.SubElement(lane, 'userData', code='mapforge.source_lane', value=sid)
                ET.SubElement(lane, 'userData', code='mapforge.provenance/v1',
                              value=json.dumps({'travel_direction': 'with_s'}))
                for s in lane.findall('speed'):
                    lane.remove(s)
                ET.SubElement(lane, 'speed', sOffset='0', max='60', unit='km/h')
    from mapforge.ops.port_dependencies import PortDependencies
    graph = PortDependencies(root)
    for cid, (a, b) in graph.connections.items():
        src.topo_out.setdefault(a.road + ':' + str(a.lane), []).append(cid + ':-1')
        src.topo_out.setdefault(cid + ':-1', []).append(b.road + ':' + str(b.lane))
    return root, src, first


def test_whole_road_packet_shares_boundary_and_contains_all_turns_and_end_lanes():
    root, src, first = fixture()
    before = ET.tostring(root)
    p = prepare_scope(root, src, ['10'])
    assert p['source_admission'] == 'ADMITTED_FOR_RESEARCH'
    assert p['connectors'] == sorted([first, 'second'])
    assert {v['contact'] for v in p['mutable_ports']} == {'start', 'end'}
    assert len(p['boundary_owners']['B:shared']) == len(p['observations'])
    assert require_solver_input(p, root, src)
    assert p['status'] == 'BLOCKED' and not p['export_allowed']
    assert ET.tostring(root) == before


def test_missing_original_topology_cannot_be_replaced_by_spatial_guess():
    root, src, _ = fixture()
    src.topo_out.clear()
    p = prepare_scope(root, src, ['10'])
    assert any(i['code'] == 'SOURCE_CONNECTOR_TOPOLOGY_MISMATCH' for i in p['issues'])
    with pytest.raises(ValueError, match='admission rejected'):
        require_solver_input(p, root)


@pytest.mark.parametrize('fault', ['missing', 'duplicate', 'unknown_side', 'bad_geometry'])
def test_boundary_faults_preserved_and_rejected(fault):
    root, src, _ = fixture()
    get = src.lane_boundary_records
    def broken(sid):
        rows = list(copy.deepcopy(get(sid)))
        if fault == 'missing': rows[0]['records'] = ()
        if fault == 'duplicate': rows[0]['records'] *= 2
        if fault == 'unknown_side': rows[0]['declared_side'] = None
        if fault == 'bad_geometry': rows[0]['records'][0]['parts'] = ()
        return tuple(rows)
    src.lane_boundary_records = broken
    p = prepare_scope(root, src, ['10'])
    assert p['source_admission'] == 'REJECTED' and p['boundaries']
    with pytest.raises(ValueError, match='admission rejected'):
        require_solver_input(p, root)


def test_speed_disagreement_is_visible_and_never_rewritten():
    root, src, first = fixture()
    road = next(r for r in root.findall('road') if r.get('id') == first)
    road.find('.//lane/speed').set('max', '15')
    before = ET.tostring(root)
    p = prepare_scope(root, src, ['10'])
    assert any(not u['written_source_speed_match'] for u in p['occurrences'])
    assert p['observations'][first + ':-1']['source_max_speed_kmh'] == 60.
    assert ET.tostring(root) == before
    assert not p['export_allowed']


def test_stale_or_modified_packets_cannot_reach_solver():
    root, src, _ = fixture()
    p = prepare_scope(root, src, ['10'])
    other = copy.deepcopy(p); other['connectors'].pop()
    with pytest.raises(ValueError, match='digest'):
        require_solver_input(other, root)
    other['content_sha256'] = digest({k: v for k, v in other.items() if k != 'content_sha256'})
    with pytest.raises(ValueError, match='incomplete'):
        require_solver_input(other, root)
    root.set('changed', 'yes')
    with pytest.raises(ValueError, match='stale'):
        require_solver_input(p, root)


def test_nonordinary_or_empty_selection_is_rejected():
    root, src, first = fixture()
    for ids in ([], ['missing'], [first]):
        with pytest.raises(ValueError):
            prepare_scope(root, src, ids)


def test_source_support_chain_keeps_intermediate_features_without_bounded_search_guess():
    assert source_chain({'a': ['b'], 'b': ['c']}, 'a', 'c') == ['a', 'b', 'c']
    for topo, match in [({'a': ['b', 'd'], 'b': ['c']}, 'branches'),
                        ({'a': ['b'], 'b': ['a']}, 'cycle'),
                        ({'a': ['b'], 'b': ['d'], 'd': ['c']}, 'budget')]:
        with pytest.raises(ValueError, match=match):
            source_chain(topo, 'a', 'c', budget=1 if match == 'budget' else 10000)


def test_primary_supplement_policy_requires_one_record_per_layer_and_preserves_both():
    root, src, first = fixture()
    src.L = dict(src.L, lane={'file': 'L'}, lane_merge={'file': 'LM'})
    old = src.lane_raw_records
    def layered(sid):
        a = old(sid)[0]
        b = dict(a, layer='LM', parts=(((0., 0.), (50., 0.)),))
        return (a, b)
    src.lane_raw_records = layered
    assert prepare_scope(root, src, ['10'])['source_admission'] == 'REJECTED'
    src.p = {'lane_identity': 'primary-with-supplement-records'}
    packet = prepare_scope(root, src, ['10'])
    assert packet['source_admission'] == 'ADMITTED_FOR_RESEARCH'
    for o in packet['observations'].values():
        assert o['identity_resolution']['selected_layer'] == 'L'
        assert len(o['raw_records']) == 2 and len(o['lane_path']) == 2
    src.lane_raw_records = lambda sid: layered(sid) + (old(sid)[0],)
    assert prepare_scope(root, src, ['10'])['source_admission'] == 'REJECTED'


@pytest.mark.parametrize('field', ['mutable_ports', 'fixed_ports', 'replacement_roads'])
def test_rehashed_incomplete_whole_road_packet_is_still_rejected(field):
    root, src, _ = fixture()
    packet = prepare_scope(root, src, ['10'])
    packet[field].pop()
    packet['content_sha256'] = digest({k: v for k, v in packet.items() if k != 'content_sha256'})
    with pytest.raises(ValueError, match='incomplete'):
        require_solver_input(packet, root)


def test_missing_junction_is_not_successful_input_admission():
    root, src, _ = fixture()
    root.remove(root.find('junction'))
    with pytest.raises(ValueError, match='junction table'):
        prepare_scope(root, src, ['10'])


def test_self_rehashed_missing_source_cannot_reach_solver():
    root, src, _ = fixture()
    p = prepare_scope(root, src, ['10'])
    with pytest.raises(ValueError, match='fresh source reader'):
        require_solver_input(p, root)
    p['observations'].pop(next(iter(p['observations'])))
    p['content_sha256'] = digest({k: v for k, v in p.items() if k != 'content_sha256'})
    with pytest.raises(ValueError, match='fresh original inputs'):
        require_solver_input(p, root, src)
