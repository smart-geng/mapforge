import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.ops.junction_surface import (
    directional_sweep, bridge_source_tails, source_median_tails, _append_surface_family,
)
from mapforge.validate.smoothness import lane_edges_kinematics_at, road_surface_polygon


def test_directional_sweep_is_local_and_rejects_islands():
    result = directional_sweep(box(0, 0, 10, 3), [2, 0], .8)
    assert result.bounds == pytest.approx((0, 0, 10.8, 3))
    assert result.area == pytest.approx(32.4)
    with pytest.raises(ValueError, match="islands"):
        directional_sweep(Polygon(box(0, 0, 10, 10).exterior,
                                   [box(3, 3, 7, 7).exterior]), [1, 0], 1)


def test_bridge_checks_whole_oblique_end_cap_not_one_touching_corner():
    base = Polygon([(10, 0), (11, 4), (20, 4), (20, 0)])
    tail = box(0, 0, 10, 4)
    mouths = [{"road_id":"10", "pose":[0, 0, 0]}]
    result, issues = bridge_source_tails(base, [{"road_id":"10", "geometry":tail}], mouths)
    assert not issues
    assert len(result) == 1
    assert result[0]["sweep_m"] == pytest.approx(1.25)
    assert result[0]["geometry"].bounds[3] == 4  # no lateral growth


def test_bridge_does_not_hide_large_source_gap():
    result, issues = bridge_source_tails(box(15, 0, 30, 4),
        [{"road_id":"10", "geometry":box(0, 0, 10, 4)}],
        [{"road_id":"10", "pose":[0, 0, 0]}])
    assert not result
    assert issues[0]["reason"] == "end-cap-outside-repair-budget"


def test_median_requires_identity_and_two_supported_carriageways():
    spec = {"road_id":"10", "pose":[0, 0, 0], "median_interval":[-1, 1]}
    tails = [{"road_id":"10", "geometry":p} for p in
             (box(-1, -5, 10.25, -1), box(-1, 1, 10.25, 5))]
    result, issues = source_median_tails(box(10, -6, 20, 6), tails, [spec])
    assert not issues
    assert len(result) == 1
    assert result[0]["geometry"].bounds == pytest.approx((-.5, -1, 10, 1))
    assert not source_median_tails(box(10, -6, 20, 6), tails,
        [{k:v for k,v in spec.items() if k != "median_interval"}])[0]
    assert not source_median_tails(box(10, -6, 20, 6), tails[:1], [spec])[0]


def test_surface_family_has_shared_nonnegative_boundaries_and_no_driving_topology():
    doc = W.XodrDoc("surface-test")
    # Tapered median ending within a continuous pavement. Independent PCHIPs
    # can cross around this termination; the family limiter must prevent that.
    median = Polygon([(0, -1), (0, 1), (14, 1), (15, 0), (14, -1)])
    _append_surface_family(doc, box(0, -5, 20, 5), median,
        {"road_id":"10", "pose":[0, 0, 0], "median_interval":[-1, 1]}, 50, 1, ["a", "b"])
    root = ET.Element("OpenDRIVE")
    doc._road_el(root, doc.roads[0])
    rd = root.find("road")
    assert len(rd.findall("planView/geometry")) == 1
    assert len(rd.findall("lanes/laneSection")) == 1
    assert [l.get("type") for l in rd.findall("lanes/laneSection/left/lane")] == [
        "restricted", "median", "restricted"]
    assert not rd.findall(".//lane/link")
    assert not rd.findall("link")
    for s in np.linspace(0, 20.5, 1001):
        edges = lane_edges_kinematics_at(rd, s, "left")
        assert np.diff([x[0] for x in edges]).min() >= -1e-8
    assert not road_surface_polygon(rd).interiors


def test_auxiliary_writer_rejects_driving_lane():
    with pytest.raises(ValueError, match="routable"):
        W.add_paving_road(W.XodrDoc("test"), np.asarray(box(0,0,20,10).exterior.coords),
                          1, lane_type="driving")
