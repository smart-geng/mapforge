"""World-space lane endpoints, independently checked by Cartesian derivatives."""
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from pyclothoids import Clothoid, SolveG2

from mapforge.adapters.opendrive import writer as W
from mapforge.ops.map_to_xodr import _written_lane_end
from mapforge.validate.smoothness import lane_endpoint_state, junction_lane_interfaces
from mapforge.validate.g11 import _audit_c, load_policy
from pathlib import Path

POLICY = Path(__file__).resolve().parents[1]/'profiles/validation/g11-opendrive-v1.draft.yaml'


def make_road(rid=10):
    road = W.Road(rid)
    road.add_geometry('spiral', 0., 0., .2, 60., .001, .005)
    road.add_offset(0., .4, .015, .0001)
    sec = W.LaneSection(0.)
    sec.right = [W.Lane(-1).add_width(3.5, .006, .00005)]
    # Non-consecutive IDs deliberately ensure identity, not array indexing.
    sec.left = [W.Lane(1).add_width(1.5), W.Lane(3).add_width(3.4, .01)]
    road.sections = [sec]
    return road


@pytest.mark.parametrize('lane_id,reverse', [(-1, False), (3, True)])
def test_written_endpoint_matches_independent_cartesian_derivatives(lane_id, reverse):
    road = make_road()
    c = Clothoid.StandardParams(0., 0., .2, .001, .004/60., 60.)
    def xy(s):
        offset = .4+.015*s+.0001*s*s
        t = offset-(3.5+.006*s+.00005*s*s)/2 if lane_id < 0 else offset+1.5+(3.4+.01*s)/2
        h = .2+.001*s+.5*(.004/60)*s*s
        return np.array([c.X(s)-t*np.sin(h), c.Y(s)+t*np.cos(h)])
    ds = np.linspace(-.02, 0., 9)
    coefficients = np.polynomial.polynomial.polyfit(ds, np.array([xy(60+x) for x in ds]), 4)
    d1, d2 = coefficients[1], 2*coefficients[2]
    expected_h = math.atan2(d1[1], d1[0])+(math.pi if reverse else 0.)
    expected_k = (d1[0]*d2[1]-d1[1]*d2[0])/np.linalg.norm(d1)**3*(-1 if reverse else 1)
    pose, k = _written_lane_end(road, lane_id, reverse=reverse)
    np.testing.assert_allclose(pose[:2], xy(60.), atol=1e-10)
    assert pose[2] == pytest.approx(expected_h, abs=1e-8)
    assert k == pytest.approx(expected_k, abs=1e-7)
    root = ET.Element('OpenDRIVE'); W.XodrDoc('test')._road_el(root, road)
    read = lane_endpoint_state(root.find('road'), lane_id, 'end', forward=not reverse)
    assert read['curvature'] == pytest.approx(expected_k, abs=1e-7)
    assert read['heading'] == pytest.approx(expected_h, abs=1e-8)


def interface_network(correct):
    incoming = make_road()
    outgoing = W.Road(11).add_geometry('line', 100., 30., math.pi, 30.)
    outgoing.sections = [W.LaneSection(0., left=[W.Lane(3).add_width(3.5)])]
    a, ka = _written_lane_end(incoming, -1)
    b, kb = _written_lane_end(outgoing, 3, reverse=True)
    if not correct:
        a = (*a[:2], incoming.end_pose()[2])
        ka = .005  # Old reference-as-lane shortcut.
    connector = W.Road(100, junction=1)
    for c in SolveG2(*a, ka, *b, kb):
        connector.add_geometry('spiral', c.XStart, c.YStart, c.ThetaStart,
                               c.length, c.KappaStart, c.KappaEnd)
    connector.add_offset(0., 1.75)
    lane = W.Lane(-1).add_width(3.5); lane.pred = -1; lane.succ = 3
    connector.sections = [W.LaneSection(0., right=[lane])]
    connector.add_link('predecessor', 'road', 10, 'end')
    connector.add_link('successor', 'road', 11, 'end')
    root = ET.Element('OpenDRIVE'); writer = W.XodrDoc('test')
    for r in (incoming, outgoing, connector): writer._road_el(root, r)
    junction = ET.SubElement(root, 'junction', id='1')
    conn = ET.SubElement(junction, 'connection', id='0', incomingRoad='10',
                         connectingRoad='100', contactPoint='start')
    ET.SubElement(conn, 'laneLink', **{'from':'-1', 'to':'-1'})
    return root


def test_g11_rejects_reference_heading_as_lane_heading():
    root = interface_network(False)
    result = _audit_c(root, load_policy(POLICY))
    codes = {x['code'] for x in result['issues']}
    assert 'road_interface_heading_discontinuity' in codes
    assert 'road_interface_curvature_discontinuity' in codes


def test_g11_accepts_full_world_lane_boundary_conditions():
    root = interface_network(True)
    rows = junction_lane_interfaces(root)
    assert len(rows) == 2 and not any('error' in r for r in rows)
    assert max(r['position_m'] for r in rows) < 1e-6
    assert max(r['heading_deg'] for r in rows) < 1e-5
    assert max(r['curvature_per_m'] for r in rows) < 1e-9
    assert _audit_c(root, load_policy(POLICY))['status'] == 'PASS'


def test_missing_exit_lane_is_not_a_silent_skipped_success():
    root = interface_network(True)
    root.find("road[@id='100']/lanes/laneSection/right/lane/link/successor").set('id', '99')
    result = _audit_c(root, load_policy(POLICY))
    assert result['status'] == 'FAIL'
    assert any(x['code'] == 'unresolved_road_interface' for x in result['issues'])
