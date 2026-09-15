import numpy as np
import pytest

from mapforge.ops.shp_to_xodr import _source_envelope_axis, _topological_connector_source


def test_source_mouth_precedes_earliest_lane_and_keeps_axis():
    axis = np.array([[0., 0.], [40., 0.], [100., 0.]])
    mouths = np.array([[96., -2.], [106., 2.], [99., 5.]])
    result, meta = _source_envelope_axis(axis, mouths)
    np.testing.assert_allclose(result, [[0., 0.], [40., 0.], [93., 0.]])
    assert meta["axis_shift_m"] == pytest.approx(-7.)
    assert len(result) == len(axis)


def test_source_mouth_cannot_delete_seed():
    with pytest.raises(ValueError, match="remove the seed"):
        _source_envelope_axis(np.array([[0., 0.], [10., 0.]]),
                              np.array([[3., -2.]]))


def test_source_chain_reverses_neighbors_without_changing_via():
    via = np.array([[10., 0.], [15., 3.], [20., 10.]])
    incoming = np.array([[10., 0.], [0., 0.]])
    outgoing = np.array([[20., 30.], [20., 10.]])
    full, meta = _topological_connector_source(via, incoming, outgoing)
    np.testing.assert_allclose(full, [[0., 0.], [10., 0.], [15., 3.],
                                      [20., 10.], [20., 30.]])
    assert meta["raw_join_gaps_m"] == [0., 0.]
    assert meta["decision"] == "source-topology-composite"


def test_source_chain_rejects_real_topology_gap():
    full, meta = _topological_connector_source(
        [[10., 0.], [20., 10.]], [[0., 0.], [7., 0.]], [[20., 10.], [20., 30.]])
    assert full is None
    assert meta["raw_join_gaps_m"][0] == 3.


def test_unclipped_route_audit_follows_lane_links_across_sections():
    from mapforge.adapters.opendrive import writer as W
    from scripts.junction_mouth_review import _lane_tail
    rd = W.Road(10)
    rd.add_geometry("line", 0., 0., 0., 20.)
    rd.add_offset(0., 0.)
    rd.add_offset(10., 3.)
    first, last = W.LaneSection(0.), W.LaneSection(10.)
    old = W.Lane(-1).add_width(3.5)
    old.succ = -2
    first.right.append(old)
    last.right.append(W.Lane(-1).add_width(3.))
    continued = W.Lane(-2).add_width(3.5)
    continued.pred = -1
    last.right.append(continued)
    rd.sections = [first, last]
    doc = W.XodrDoc("audit-test")
    import xml.etree.ElementTree as ET
    root = ET.Element("OpenDRIVE")
    el = doc._road_el(root, rd)
    if el is None:
        el = root.find("road")
    points = _lane_tail(el, -2, at_end=True)
    assert points[0, 0] == pytest.approx(20.)
    assert points[-1, 0] == pytest.approx(0.)
    np.testing.assert_allclose(points[:, 1], -1.75)
