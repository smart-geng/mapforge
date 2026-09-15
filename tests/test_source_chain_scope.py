"""Long-lane provenance must not collapse to a single endpoint source ID."""
import json
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mapforge.ops.reconstruction_scope import _source_ids, _chain_direction, prepare_scope, require_solver_input
from mapforge.ops.source_domain import source_speed_intervals
from tests.test_reconstruction_scope import fixture
from tests.test_exact_source_chart import road


def with_chain():
    root, src, _ = fixture()
    lane = root.find("road[@id='10']/lanes/laneSection/right/lane[@id='-1']")
    ET.SubElement(lane, 'userData', code='mapforge.source_chain/v1', value=json.dumps(['upstream', '10:-1']))
    src.topo_out['upstream'] = ['10:-1']
    return root, src, lane


def test_complete_chain_is_observed_shared_and_replayed_without_source_edits():
    root, src, lane = with_chain(); before = ET.tostring(root)
    p = prepare_scope(root, src, ['10'], identity_mode='source-chain-v1')
    assert _source_ids(lane) == ['upstream', '10:-1']
    assert p['source_admission'] == 'ADMITTED_FOR_RESEARCH' and 'upstream' in p['observations']
    assert {'upstream','10:-1'} <= set(p['boundary_owners']['B:shared'])
    occurrences = [u for u in p['occurrences'] if u.get('source_identity_chain')]
    assert {u['source_lane_id'] for u in occurrences} == {'upstream','10:-1'}
    assert all(u['written_source_speed_match'] is None for u in occurrences)
    assert require_solver_input(p, root, src) and ET.tostring(root) == before
    assert not p['export_allowed']


@pytest.mark.parametrize('value', ['[]','["a","a"]','[1]', '{"ids":[]}', 'bad'])
def test_invalid_chain_cannot_fall_back_to_a_compatible_port_id(value):
    root, src, lane = with_chain()
    lane.find("userData[@code='mapforge.source_chain/v1']").set('value', value)
    with pytest.raises(ValueError): _source_ids(lane)
    p = prepare_scope(root, src, ['10'], identity_mode='source-chain-v1')
    assert p['source_admission'] == 'REJECTED'
    assert any(i['code']=='SOURCE_CHAIN_IDENTITY_INVALID' for i in p['issues'])


def test_chain_topology_and_port_membership_must_match_actual_original_ids():
    root, src, lane = with_chain(); src.topo_out['upstream'] = []
    p = prepare_scope(root, src, ['10'], identity_mode='source-chain-v1')
    assert any(i['code']=='SOURCE_CHAIN_TOPOLOGY_MISMATCH' for i in p['issues'])
    lane.find("userData[@code='mapforge.source_lane']").set('value', 'foreign-port')
    with pytest.raises(ValueError, match='not in'): _source_ids(lane)


def test_legacy_direction_inference_uses_original_order_and_is_explicitly_labelled():
    r = road(0., heading=0.); root = ET.Element('OpenDRIVE')
    ET.SubElement(ET.SubElement(root, 'header'), 'geoReference').text = (
        '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m')
    src = SimpleNamespace(lane=lambda _:SimpleNamespace(geometry=np.array([[0.,0.],[.0001,0.]])))
    direction, evidence = _chain_direction(root, r, ['original'], src)
    assert direction == 'with_s' and evidence['status'] == 'INFERRED'
    assert evidence['rows'][0]['complete_vertex_count'] == 2
    src.lane = lambda _:SimpleNamespace(geometry=np.array([[0.,0.],[.0001,0.],[.00005,0.]]))
    with pytest.raises(ValueError, match='nonmonotone'): _chain_direction(root, r, ['original'], src)


def test_speed_events_compare_only_the_actual_source_interval_not_the_whole_chain():
    root, src, _ = fixture()
    ET.SubElement(ET.SubElement(root, 'header'), 'geoReference').text = (
        '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m')
    r = root.find("road[@id='10']"); r.set('length','100')
    g = r.find('planView/geometry'); g.attrib.update(x='0', y='0', hdg='0', s='0', length='100')
    for child in list(g): g.remove(child)
    ET.SubElement(g,'line')
    meter_degree = 180/(np.pi*6378137.)
    partition = dict(features={'lane:'+sid:dict(parts=[dict(raw_vertices=[[lo*meter_degree,0],[hi*meter_degree,0]])])
                              for sid, lo, hi in [('a',0,40),('b',40,100)]})
    uses = [dict(road='10',section=0,lane=-1,source_lane_id=sid,scope_role='ordinary',
                 written_speeds=[dict(sOffset='0',kmh=40.),dict(sOffset='40',kmh=60.)]) for sid in ['a','b']]
    scope = dict(occurrences=uses, observations={'a':dict(source_max_speed_kmh=40.),'b':dict(source_max_speed_kmh=60.)})
    p = source_speed_intervals(root, scope, partition)
    assert p['counts'] == {'MATCH':2}
    assert [r['overlap_m'] for r in p['rows']] == pytest.approx([40.,60.])
    uses[1]['written_speeds'][1]['kmh'] = 50.
    p = source_speed_intervals(root, scope, partition)
    assert p['counts'] == {'MATCH':1,'MISMATCH':1}
    assert p['rows'][1]['mismatches'][0]['original_overlap_m'] == pytest.approx(60.)


def test_new_compiler_emits_direction_without_changing_speed_or_source_ids(monkeypatch):
    from tests.test_source_cubic_export import fixture as compile_fixture
    from mapforge.ops.source_cubic_export import compile_road
    m,x,old = compile_fixture(monkeypatch)
    result,_,_ = compile_road(m,x,old)
    for side, expected in [('right','with_s'),('left','against_s')]:
        for lane in result.findall('lanes/laneSection/'+side+'/lane'):
            if lane.get('type') != 'driving': continue
            assert _source_ids(lane)
            meta=json.loads(lane.find("userData[@code='mapforge.provenance/v1']").get('value'))
            assert meta['travel_direction']==expected and meta['geometry_acceptance']=='NOT_DELIVERY'
            assert {float(s.get('max')) for s in lane.findall('speed')}=={60/3.6}


def test_partial_model_is_stopped_before_numerical_initialization():
    from mapforge.ops.reconstruction_scope import model_scope_report
    from mapforge.ops.port_dependencies import PortDependencies
    root,_,_=fixture(); jid=root.find('junction').get('id')
    turns=sorted(PortDependencies(root).connections)
    subset=model_scope_report(root,jid,['10'],turns[:1])
    assert subset['missing_parent_models']==['11']
    assert len(subset['missing_connector_models'])==1
    assert subset['status']=='INCOMPLETE_WHOLE_JUNCTION_MODEL'
    complete=model_scope_report(root,jid,['10','11'],turns)
    assert complete['status']=='SCOPE_COMPLETE_NOT_MODEL_FEASIBILITY'
    assert not complete['export_allowed'] and not complete['geometry_solver_ran']
    assert model_scope_report(root,jid,['10','10','11'],turns)['duplicate_models']
