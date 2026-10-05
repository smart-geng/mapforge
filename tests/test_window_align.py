"""SHP comparison windows aligned to the written lanes at road ends (window_align)."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mapforge.ops import window_align as W

ROOT = ET.fromstring(
    '<OpenDRIVE><road id="10" length="60" junction="-1"><planView><geometry s="0" x="0" y="0" hdg="0" length="60">'
    '<line/></geometry></planView></road></OpenDRIVE>')


def _manifest(windows):
    return {"lanes": [{"source_lane_id": sid, "comparison": {"eligible": True},
                       "geometry": {"coordinates": [list(p) for p in pts]}, "travel": {}}
                      for sid, pts in windows.items()]}


def _run(monkeypatch, windows, targets):
    comps = [{"source_lane_id": sid, "road_id": "10", "points": pts} for sid, pts in targets.items()]
    monkeypatch.setattr("mapforge.validate.lane_fidelity.extract_target_components",
                        lambda root, **kw: {"components": comps})
    manifest = _manifest(windows)
    report = W.apply(ROOT, manifest)
    return {lane["source_lane_id"]: lane for lane in manifest["lanes"]}, report


def test_a_window_past_the_road_end_is_cut_and_one_short_of_its_start_is_continued(monkeypatch):
    lanes, report = _run(monkeypatch,
                         {"A": [(0.1, -1.75), (30.0, -1.75), (60.4, -1.75)]},
                         {"A": [(0.0, -1.75), (30.0, -1.75), (60.0, -1.75), (60.0, -1.75)]})   # (a repeated end sample)
    w = np.asarray(lanes["A"]["geometry"]["coordinates"])
    assert w[0] == pytest.approx([0.0, -1.75], abs=1e-9) and w[-1] == pytest.approx([60.0, -1.75], abs=1e-9)
    assert lanes["A"]["window_adjustments"] == [
        {"code": W.CODE, "end": "start", "road": "10", "contact": "start", "extended_m": 0.1},
        {"code": W.CODE, "end": "end", "road": "10", "contact": "end", "cut_m": 0.4}]
    assert lanes["A"]["travel"]["end"] == pytest.approx([60.0, -1.75])
    assert report["lanes_aligned"] == 1 and report["ends_cut"] == 1 and report["ends_extended"] == 1


def test_a_lane_end_inside_the_road_and_a_change_over_the_limit_are_left_alone(monkeypatch):
    lanes, report = _run(monkeypatch,
                         {"B": [(0.0, 1.75), (41.0, 1.75)],                     # the lane dies at x = 40 (in the road)
                          "C": [(0.0, -5.25), (62.0, -5.25)]},                  # 2 m past the road end
                         {"B": [(0.0, 1.75), (40.0, 1.75)], "C": [(0.0, -5.25), (60.0, -5.25)]})
    assert "window_adjustments" not in lanes["B"] and "window_adjustments" not in lanes["C"]
    assert np.asarray(lanes["B"]["geometry"]["coordinates"])[-1] == pytest.approx([41.0, 1.75])
    assert report["skipped"] == [{"source_lane_id": "C", "end": "end", "road": "10", "contact": "end",
                                  "offset_m": 2.0}]


def test_a_lane_running_against_the_reference_is_aligned_at_both_ends(monkeypatch):
    # a left lane: travels from the road end (x = 60) to its start (x = 0)
    lanes, _ = _run(monkeypatch, {"D": [(60.3, 1.75), (0.05, 1.75)]}, {"D": [(60.0, 1.75), (0.0, 1.75)]})
    w = np.asarray(lanes["D"]["geometry"]["coordinates"])
    assert w[0] == pytest.approx([60.0, 1.75], abs=1e-9) and w[-1] == pytest.approx([0.0, 1.75], abs=1e-9)
    assert [(a["end"], a["contact"], a.get("cut_m"), a.get("extended_m")) for a in lanes["D"]["window_adjustments"]] == [
        ("start", "end", 0.3, None), ("end", "start", None, 0.05)]
