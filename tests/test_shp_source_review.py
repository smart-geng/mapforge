"""SHP source consistency review (boundary priority): centreline off the boundary midpoint, lane-object steps."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from mapforge.validate import shp_source_review as R

ORIGIN = {"lon": 106.0, "lat": 29.5}


def _deg(xy):
    xy = np.asarray(xy, float)
    r = 6378137.0
    lon = ORIGIN["lon"] + np.degrees(xy[:, 0] / (r * math.cos(math.radians(ORIGIN["lat"]))))
    lat = ORIGIN["lat"] + np.degrees(xy[:, 1] / r)
    return np.column_stack([lon, lat])


class FakeSource:
    def __init__(self):
        self.lanes, self.bnds, self.topo_out = {}, {}, {}

    def add(self, sid, x0, x1, centre_y, left_y, right_y, right_y_start=None):
        self.lanes[sid] = SimpleNamespace(geometry=_deg([(x0, centre_y), (x1, centre_y)]), geometry_source="field",
                                          link_pid="link-" + sid)
        right0 = right_y if right_y_start is None else right_y_start
        self.bnds[sid] = [_deg([(x0, left_y), (x1, left_y)]), _deg([(x0, right0), (x1, right_y)])]

    def lane(self, sid):
        return self.lanes.get(sid)

    def lane_boundary_geometries(self, sid):
        return self.bnds[sid]


def test_centreline_off_the_boundary_midpoint_is_recorded():
    src = FakeSource()
    src.add("A", 0.0, 50.0, 0.4, 1.75, -1.75)
    src.add("B", 0.0, 50.0, 0.05, 1.75, -1.75, )
    rev = R.review(src, ["A", "B"], ORIGIN)
    (f,) = rev["findings"]
    assert f["rule"] == "centreline-off-midpoint" and f["source_lane_id"] == "A"
    assert f["max_m"] == pytest.approx(0.4, abs=0.01) and f["lane_width_m"] == pytest.approx(3.5, abs=0.01)
    assert f["status"] == "RECORDED" and f["resolution"] == "boundary-priority"
    assert rev["open"] == 0 and rev["recorded"] == 1


def test_lane_object_step_is_recorded_and_a_birth_joint_is_not():
    src = FakeSource()
    src.add("A", 0.0, 50.0, 0.0, 1.75, -1.75)
    src.add("B", 50.0, 100.0, 0.2, 1.95, -1.55)
    src.topo_out = {"A": ["B"]}
    rev = R.review(src, ["A", "B"], ORIGIN, {"A": {"roles": ["approach"], "g8_comparable": True}})
    (f,) = rev["findings"]
    assert f["rule"] == "lane-object-step" and (f["source_lane_id"], f["successor"]) == ("A", "B")
    assert f["centreline_lateral_m"] == pytest.approx(0.2, abs=0.01)
    assert f["boundary_lateral_m"] == pytest.approx([0.2, 0.2], abs=0.01)
    assert f["roles"] == ["approach"] and f["g8_comparable"] is True
    assert rev["counts"] == {"lane-object-step/road": 1}
    # a lane born at zero width on A's left edge: a birth, not a step
    born = FakeSource()
    born.add("A", 0.0, 50.0, 0.0, 1.75, -1.75)
    born.add("C", 50.0, 100.0, 3.5, 5.25, 1.75, right_y_start=1.75)
    born.lanes["C"].geometry = _deg([(50.0, 1.75), (100.0, 3.5)])
    born.bnds["C"][0] = _deg([(50.0, 1.75), (100.0, 5.25)])
    born.topo_out = {"A": ["C"]}
    assert [f["rule"] for f in R.review(born, ["A", "C"], ORIGIN)["findings"]] == []


def test_lanes_placed_side_by_side_with_a_gap_are_recorded():
    # two parallel links, 1 m apart, whose lanes the conversion puts next to each other
    src = FakeSource()
    src.add("A", 0.0, 80.0, 0.0, 1.75, -1.75)
    src.add("B", 0.0, 80.0, 4.5, 6.25, 2.75)
    rev = R.review(src, ["A", "B"], ORIGIN, pairs=[("A", "B")])
    (f,) = rev["findings"]
    assert f["rule"] == "shared-boundary-mismatch" and (f["source_lane_id"], f["neighbour"]) == ("A", "B")
    assert f["median_m"] == pytest.approx(1.0, abs=0.01)
    touching = FakeSource()
    touching.add("A", 0.0, 80.0, 0.0, 1.75, -1.75)
    touching.add("B", 0.0, 80.0, 3.5, 5.25, 1.75)
    assert R.review(touching, ["A", "B"], ORIGIN, pairs=[("A", "B")])["findings"] == []


def test_joint_kink_is_recorded_but_a_steady_curve_is_not():
    import math as _m
    src = FakeSource()
    src.add("A", 0.0, 50.0, 0.0, 1.75, -1.75)
    c, s_ = _m.cos(_m.radians(11)), _m.sin(_m.radians(11))
    src.lanes["B"] = SimpleNamespace(geometry=_deg([(50.0, 0.0), (50.0 + 30 * c, 30 * s_)]), geometry_source="field",
                                     link_pid="link-B")
    src.bnds["B"] = [_deg([(50.0 - 1.75 * s_, 1.75 * c), (50.0 + 30 * c - 1.75 * s_, 30 * s_ + 1.75 * c)]),
                     _deg([(50.0 + 1.75 * s_, -1.75 * c), (50.0 + 30 * c + 1.75 * s_, 30 * s_ - 1.75 * c)])]
    src.topo_out = {"A": ["B"]}
    kinks = [f for f in R.review(src, ["A", "B"], ORIGIN)["findings"] if f["rule"] == "lane-object-kink"]
    assert len(kinks) == 1 and kinks[0]["kink_deg"] == pytest.approx(11.0, abs=0.2)
    # a 10 m radius arc split into two lane objects at its middle: no kink
    th = np.arange(-0.5, 0.5001, 0.025)
    arc = np.column_stack([10 * np.sin(th), 10 * (1 - np.cos(th))])
    a, b = arc[arc[:, 0] <= 1e-9], arc[arc[:, 0] >= -1e-9]
    assert abs(_m.degrees(R.joint_heading(b, True) - R.joint_heading(a, False))) < 2.0
