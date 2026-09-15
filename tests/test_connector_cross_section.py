import math
import xml.etree.ElementTree as ET
import pytest
from tests.test_junction_edges import network
from spikes.connector_cross_section import build, endpoint_frame, edge_jet
from mapforge.validate.junction_edges import audit
from mapforge.validate.smoothness import junction_lane_interfaces


def turning_network():
    root = network(vary=True)
    incoming = root.find("road[@id='10']")
    incoming.find('lanes/laneOffset').set('a','1.4')
    incoming.find('lanes/laneOffset').set('b','.005')
    width = incoming.find('lanes/laneSection/right/lane/width')
    width.set('a','2.8'); width.set('b','.01')
    outgoing = root.find("road[@id='11']/planView/geometry")
    outgoing.set('x','55'); outgoing.set('y','35'); outgoing.set('hdg',str(math.pi/2))
    return root


def test_three_clothoid_edge_pass_must_not_be_mistaken_for_center_pass():
    original = turning_network()
    before = ET.tostring(original)
    result, rows = build(original)
    assert ET.tostring(original) == before
    assert len(rows) == 1 and len(rows[0]['primitive_lengths']) == 3
    assert audit(result)['status'] == 'PASS'
    assert max(r['curvature_per_m'] for r in junction_lane_interfaces(result)) > 1e-7


def test_five_primitive_experiment_proves_joint_end_states_but_is_not_three():
    result, rows = build(turning_network(), mode='five')
    assert len(rows[0]['primitive_lengths']) == 5
    assert rows[0]['width_records'] == 7
    assert audit(result)['status'] == 'PASS'
    assert max(r['curvature_per_m'] for r in junction_lane_interfaces(result)) < 1e-9


def test_transverse_jet_inversion_and_reverse_contact():
    root = turning_network()
    road = root.find("road[@id='10']")
    a = endpoint_frame(road, -1, 'end', True)
    b = endpoint_frame(road, -1, 'end', False)
    assert a['pose'][:2] == pytest.approx(b['pose'][:2])
    assert b['k'] == pytest.approx(-a['k'])
    assert b['dk'] == pytest.approx(a['dk'])
    assert a['edges']['left']['x'] == pytest.approx(b['edges']['right']['x'])
    for side in ('left', 'right'):
        jet = edge_jet(a, a['edges'][side], a['k'], a['dk'])
        assert abs(jet[0]) == pytest.approx(1.5)


def test_non_shared_section_and_unknown_mode_reject():
    root = turning_network()
    frame = endpoint_frame(root.find("road[@id='10']"), -1, 'end', True)
    moved = dict(frame['edges']['left'], x=100.)
    with pytest.raises(ValueError, match='non-shared'):
        edge_jet(frame, moved, frame['k'], frame['dk'])
    with pytest.raises(ValueError, match='unknown'):
        build(root, mode='unknown')
