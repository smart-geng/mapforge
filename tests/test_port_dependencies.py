import copy
import xml.etree.ElementTree as ET
import pytest
from mapforge.ops.port_dependencies import LanePort, PortDependencies
from tests.test_connector_cross_section import turning_network


def network():
    root = turning_network()
    parent = next(r for r in root.findall('road') if r.get('id') == '10')
    side = parent.find('lanes/laneSection/right')
    lane = copy.deepcopy(side.find('lane')); lane.set('id', '-2'); side.append(lane)
    connector = next(r for r in root.findall('road') if r.get('junction') != '-1')
    other = copy.deepcopy(connector); other.set('id', 'second')
    other.find('lanes/laneSection/right/lane/link/predecessor').set('id', '-2')
    root.append(other)
    # The second road also needs a real junction-table entry, not only road links.
    connection = copy.deepcopy(root.find('junction/connection'))
    connection.set('id', 'second')
    connection.set('connectingRoad', 'second')
    connection.find('laneLink').set('from', '-2')
    root.find('junction').append(connection)
    return root, connector.get('id')


def test_shared_boundary_closes_both_lanes_and_all_incident_turns():
    root, rid = network(); graph = PortDependencies(root)
    ports = graph.boundary_ports('10', 'end', 'right', 1)
    assert ports == frozenset([LanePort('10', 'end', -1), LanePort('10', 'end', -2)])
    closure = graph.closure(ports)
    assert closure['connectors'] == {rid, 'second'}
    assert all(p.road != '10' for p in closure['fixed_ports'])
    with pytest.raises(ValueError, match='incomplete'):
        graph.require_complete(ports, [rid])
    graph.require_complete(ports, [rid, 'second'])


def test_failed_parent_or_sibling_rolls_back_the_entire_edit():
    root, rid = network(); original = ET.tostring(root); graph = PortDependencies(root)
    ports = graph.boundary_ports('10', 'end', 'right', 1)
    replacements = {k: copy.deepcopy(graph.roads[k]) for k in ['10', rid, 'second']}
    for r in replacements.values(): r.set('name', 'modified')
    for failed in replacements:
        validation = {k: 'PASS' for k in replacements}; validation[failed] = 'FAIL'
        result, applied = graph.commit(root, replacements, ports, validation)
        assert not applied and ET.tostring(result) == original
    assert ET.tostring(root) == original
    result, applied = graph.commit(root, replacements, ports, {k: 'PASS' for k in replacements})
    assert applied and ET.tostring(result) != original
    assert ET.tostring(root) == original


def test_unresolved_reference_is_not_repaired_by_geometry():
    root, rid = network()
    r = next(r for r in root.findall('road') if r.get('id') == rid)
    r.find('link/successor').set('elementId', 'not-present')
    with pytest.raises(ValueError, match='unresolved'):
        PortDependencies(root)


def test_center_boundary_also_affects_opposite_side():
    root, _ = network(); parent = next(r for r in root.findall('road') if r.get('id') == '10')
    sec = parent.find('lanes/laneSection'); left = sec.find('left')
    if left is None: left = ET.SubElement(sec, 'left')
    lane = copy.deepcopy(sec.find('right/lane')); lane.set('id', '1'); left.append(lane)
    ports = PortDependencies(root).boundary_ports('10', 'end', 'right', 0)
    assert {p.lane for p in ports} == {-1, 1}


def test_stale_revision_and_unknown_port_cannot_apply():
    root, rid = network(); graph = PortDependencies(root)
    with pytest.raises(ValueError,match='unknown mutable lane'):
        graph.closure([LanePort('10','end',-100)])
    ports = graph.boundary_ports('10','end','right',1)
    replacements = {k: copy.deepcopy(graph.roads[k]) for k in ['10',rid,'second']}
    root.set('revision','newer')
    with pytest.raises(ValueError,match='stale'):
        graph.commit(root,replacements,ports,{k:'PASS' for k in replacements})


def test_geometry_transaction_cannot_rewire_sibling_turn():
    root,rid=network(); graph=PortDependencies(root)
    ports=graph.boundary_ports('10','end','right',1)
    replacements={k:copy.deepcopy(graph.roads[k]) for k in ['10',rid,'second']}
    replacements['second'].find('lanes/laneSection/right/lane/link/predecessor').set('id','-1')
    before=ET.tostring(root)
    with pytest.raises(ValueError,match='traffic contacts'):
        graph.commit(root,replacements,ports,{k:'PASS' for k in replacements})
    assert ET.tostring(root)==before


def test_junction_table_cannot_be_missing_or_disagree_with_road_contacts():
    root, _ = network()
    assert PortDependencies(root).validate_junction_table()
    removed = copy.deepcopy(root)
    removed.remove(removed.find('junction'))
    with pytest.raises(ValueError, match='junction table'):
        PortDependencies(removed).validate_junction_table()
    root.find('junction/connection/laneLink').set('from', '-999')
    with pytest.raises(ValueError, match='junction table'):
        PortDependencies(root).validate_junction_table()


def test_declaring_one_lane_cannot_move_a_neighbor_even_with_all_pass():
    root, rid = network()
    graph = PortDependencies(root)
    # Claim only the OUTER lane moved, but modify the inner lane's width.
    ports = {LanePort('10', 'end', -2)}
    replacements = {k: copy.deepcopy(graph.roads[k]) for k in ['10', 'second']}
    replacements['10'].find('lanes/laneSection/right/lane/width').set('a', '4')
    before = ET.tostring(root)
    with pytest.raises(ValueError, match='undeclared port changed'):
        graph.commit(root, replacements, ports, {k: 'PASS' for k in replacements})
    assert ET.tostring(root) == before


def test_fixed_far_end_still_checked_when_all_mouth_lanes_are_declared():
    root, rid = network()
    graph = PortDependencies(root)
    ports = graph.boundary_ports('10', 'end', 'right', 1)
    replacements = {k: copy.deepcopy(graph.roads[k]) for k in ['10', rid, 'second']}
    replacements['10'].find('planView/geometry').set('y', '2')
    with pytest.raises(ValueError, match='undeclared port changed'):
        graph.commit(root, replacements, ports, {k: 'PASS' for k in replacements})


def test_geometry_transaction_cannot_hide_a_speed_change_inside_pass():
    root, rid = network()
    graph = PortDependencies(root)
    ports = graph.boundary_ports('10', 'end', 'right', 1)
    replacements = {k: copy.deepcopy(graph.roads[k]) for k in ['10', rid, 'second']}
    lane = replacements[rid].find('lanes/laneSection/right/lane')
    ET.SubElement(lane, 'speed', max='1', unit='m/s', sOffset='0')
    with pytest.raises(ValueError, match='speeds'):
        graph.commit(root, replacements, ports, {k: 'PASS' for k in replacements})
