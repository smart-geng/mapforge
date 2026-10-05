"""Raw MAP point-list review: isolated detours and single-point lateral spikes are reported, never repaired."""
import math
from pathlib import Path

import pytest

from mapforge.validate import map_source_review as R

ROOT = Path(__file__).resolve().parents[1]


def _rules(xy):
    return [(f["rule"], f["point_index_0based"]) for f in R.point_findings(xy)]


def test_a_single_point_stepping_out_and_back_is_a_lateral_spike():
    straight = [(0.0, 0.0), (40.0, 0.0), (70.0, 0.0), (80.0, 0.0), (110.0, 0.0)]
    spike = list(straight)
    spike[2] = (70.0, -2.5)            # 2.5 m off a 40 m chord, back on the line at the next point
    assert _rules(straight) == []
    assert _rules(spike) == [("lateral-spike", 2)]


def test_bends_and_lasting_lane_shifts_are_not_spikes():
    bend = [(100.0 * math.sin(a), 100.0 * (1 - math.cos(a))) for a in [k * 0.12 for k in range(8)]]   # 7 deg/point
    shift = [(0.0, 0.0), (40.0, 0.0), (62.0, 2.0), (90.0, 2.0), (130.0, 2.0)]   # 2 m to make room for a bay
    assert _rules(bend) == []
    assert _rules(shift) == []


def test_a_kilometre_detour_is_an_isolated_detour():
    assert _rules([(0.0, 0.0), (1400.0, -800.0), (0.0, 50.0), (0.0, 100.0)]) == [("isolated-detour", 1)]


@pytest.mark.skipif(not (ROOT / "v2x_map_xml").exists(), reason="local MAP samples not installed")
def test_the_seven_raw_sources_have_exactly_the_three_reviewed_points():
    from mapforge.validate.scoreboard import CASES
    found = sorted((label, f["rule"], f["source_lane_id"], f["point_index_0based"])
                   for label, name in CASES for f in R.review_file(ROOT / "v2x_map_xml" / name)["findings"])
    assert found == [("node18", "isolated-detour", "map:500:18:from:500:20:north:lane:1", 1),
                     ("node4", "lateral-spike", "map:500:4:from:500:18:west:lane:2", 5),
                     ("node4", "lateral-spike", "map:500:4:from:500:18:west:lane:3", 5)]


@pytest.mark.skipif(not (ROOT / "v2x_map_xml").exists(), reason="local MAP samples not installed")
def test_every_conversion_reports_the_review_and_what_an_approved_decision_covers(tmp_path):
    from mapforge import pipeline
    from mapforge.ops import map_source_corrections as C
    node4 = ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"
    covered = pipeline._source_review(node4, C.applicable(node4))
    assert covered["open"] == 0
    assert {f["decision"] for f in covered["findings"]} == {"map-node4-west-lane2-lane3-p5-drop-v1"}
    assert pipeline._source_review(node4, [])["open"] == 2          # without the decision both stay open
    derived, record = pipeline.corrected_map_source(node4, tmp_path / "map-node4.xodr")
    assert record["summary"] == {"DROPPED": 2} and derived.exists()
    review = (tmp_path / "map-node4.source-review.json")
    assert review.exists() and '"open": 0' in review.read_text(encoding="utf-8")
