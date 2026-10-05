"""SHP road lanes whose source centreline is off its boundaries: compared on the boundary midpoint (window_midpoint)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from mapforge.ops import window_midpoint as W

ORIGIN = {"lon": 106.0, "lat": 29.5}


def _deg(xy):
    xy = np.asarray(xy, float)
    r = 6378137.0
    return np.column_stack([ORIGIN["lon"] + np.degrees(xy[:, 0] / (r * math.cos(math.radians(ORIGIN["lat"])))),
                            ORIGIN["lat"] + np.degrees(xy[:, 1] / r)])


# a 4 m lane along x (boundaries y = 0 and y = -4, midpoint y = -2) whose centreline swings 0.4 m off in the middle
OFF = [(0.0, -2.0), (20.0, -2.0), (25.0, -2.4), (35.0, -2.4), (40.0, -2.0), (60.0, -2.0)]
SLIGHT = [(0.0, -2.0), (25.0, -2.1), (35.0, -2.1), (60.0, -2.0)]       # 0.1 m off: under the rule's threshold


class FakeSource:
    def __init__(self):
        self.lines = {"R1": OFF, "V1": OFF, "R2": SLIGHT}

    def lane(self, sid):
        return SimpleNamespace(geometry=_deg(self.lines[sid]), geometry_source="field")

    def lane_boundary_geometries(self, sid):
        return [_deg([(0.0, 0.0), (60.0, 0.0)]), _deg([(0.0, -4.0), (60.0, -4.0)])]


def _manifest():
    lanes = [{"source_lane_id": sid, "role": role, "comparison": {"eligible": True},
              "geometry": {"coordinates": [list(p) for p in line]}, "travel": {}}
             for sid, role, line in (("R1", "approach", OFF), ("V1", "junction-via", OFF), ("R2", "departure", SLIGHT))]
    return {"comparison_crs": {"origin": ORIGIN}, "lanes": lanes}


def test_the_off_stretch_of_a_road_lane_is_compared_on_the_midpoint_with_a_smooth_hand_over():
    manifest = _manifest()
    report = W.apply(manifest, FakeSource())
    lanes = {lane["source_lane_id"]: lane for lane in manifest["lanes"]}
    w = np.asarray(lanes["R1"]["geometry"]["coordinates"])
    assert np.abs(w[:, 1] + 2.0).max() < 0.05                         # on the midpoint, hand-over a few cm at most
    mid_stretch = (w[:, 0] >= 23.0) & (w[:, 0] <= 37.0)
    assert np.abs(w[mid_stretch, 1] + 2.0).max() < 1e-9
    assert np.abs(np.diff(w[:, 1])).max() < 0.05                      # no step in the reference
    (adj,) = lanes["R1"]["window_adjustments"]
    assert adj["code"] == W.CODE and adj["rule"] == "centreline-off-midpoint"
    assert adj["source_offset_max_m"] == pytest.approx(0.4, abs=0.01)
    assert adj["window_shift_max_m"] == pytest.approx(0.4, abs=0.01)
    assert lanes["R1"]["travel"]["end"] == pytest.approx([60.0, -2.0])
    assert report["lanes"] == 1 and "manifest_sha256" in manifest


def test_connector_vias_and_slightly_off_lanes_keep_their_centreline():
    manifest = _manifest()
    W.apply(manifest, FakeSource())
    lanes = {lane["source_lane_id"]: lane for lane in manifest["lanes"]}
    for sid, line in (("V1", OFF), ("R2", SLIGHT)):
        assert "window_adjustments" not in lanes[sid]
        assert lanes[sid]["geometry"]["coordinates"] == [list(p) for p in line]
