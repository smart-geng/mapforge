"""Connector lane edges at reference joins: width slopes chosen so the edges barely jump, lane centre unchanged."""
import numpy as np
import pytest
from lxml import etree

from mapforge.ops import connector_edge_joins as CE
from mapforge.validate.lane_edge_joins import road_edge_jumps
from mapforge.validate.smoothness import lane_edges_kinematics_at

# left turn: clothoid 0 -> 0.15 /m over 5 m, then an arc of 15 m (curvature rate jumps by 0.03 /m^2 at s = 5);
# one right lane narrowing linearly 3.5 -> 2.5 m, its centre on the reference line (offset = width / 2)
L, W0, W1 = 20.0, 3.5, 2.5
B = (W1 - W0) / L


def _road():
    xml = (f'<OpenDRIVE><road id="101" length="{L}" junction="1"><planView>'
           '<geometry s="0" x="0" y="0" hdg="0" length="5"><spiral curvStart="0" curvEnd="0.15"/></geometry>'
           '<geometry s="5" x="4.99" y="0.31" hdg="0.375" length="15"><arc curvature="0.15"/></geometry>'
           f'</planView><lanes><laneOffset s="0" a="{W0 / 2}" b="{B / 2}" c="0" d="0"/>'
           '<laneSection s="0"><center><lane id="0" type="none"/></center><right><lane id="-1" type="driving">'
           f'<width sOffset="0" a="{W0}" b="{B}" c="0" d="0"/></lane></right></laneSection></lanes></road></OpenDRIVE>')
    return etree.fromstring(xml)


def _centre(road, x):
    e = lane_edges_kinematics_at(road, x, "right")
    return [(a + b) / 2 for a, b in zip(e[0], e[1])]


def test_width_slopes_at_the_join_remove_the_edge_jump_and_keep_the_lane_centre():
    root = _road()
    road = root.find("road")
    before = max(j for j, *_ in road_edge_jumps(road))
    assert before > CE.TRIGGER_PER_M
    xs = np.linspace(0.0, L, 81)
    centre_old = [_centre(road, x) for x in xs]
    ends_old = [lane_edges_kinematics_at(road, x, "right") for x in (0.0, L)]
    report = CE.apply(root)
    road = root.find("road")
    assert report["fixed"] == 1
    assert max(j for j, *_ in road_edge_jumps(road)) <= CE.TRIGGER_PER_M
    # the lane centre (the driving path) is unchanged point for point, with its slope and curvature
    assert np.array([_centre(road, x) for x in xs]) == pytest.approx(np.array(centre_old), abs=1e-6)
    # both mouths keep both edges with their slopes and curvatures
    for x, old in zip((0.0, L), ends_old):
        assert np.array(lane_edges_kinematics_at(road, x, "right")) == pytest.approx(np.array(old), abs=1e-6)
    widths = [abs(e[0][0] - e[1][0]) for e in (lane_edges_kinematics_at(road, x, "right") for x in xs)]
    assert min(widths) >= W1 - CE.BULGE_SLACK_M and max(widths) <= W0 + CE.BULGE_SLACK_M


def test_a_connector_without_edge_jumps_is_left_alone():
    root = _road()
    road = root.find("road")
    for el in road.findall("planView/geometry"):
        road.find("planView").remove(el)
    etree.SubElement(road.find("planView"), "geometry", s="0", x="0", y="0", hdg="0", length=str(L)).append(
        etree.Element("line"))
    before = etree.tostring(root)
    report = CE.apply(root)
    assert report["fixed"] == 0 and etree.tostring(root) == before
