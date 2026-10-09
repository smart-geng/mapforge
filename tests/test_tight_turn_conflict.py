"""Tight-turn source conflicts (user decision 2026-10-09): detection, recording and the graded lane-centre figure."""
import math

import numpy as np
import pytest
import yaml

from mapforge.validate import lane_centre_flare as F
from mapforge.validate import scoreboard as sb
from mapforge.validate import shp_source_review as R
from mapforge.validate import tight_turn_conflict as T


def _arc(radius, turn_deg=90.0, lead=10.0, step=0.25):
    """A straight lead-in, then a circular arc of ``radius`` turning left by ``turn_deg``."""
    pts = [(x, 0.0) for x in np.arange(0.0, lead, step)]
    n = max(2, int(math.radians(turn_deg) * radius / step))
    for a in np.linspace(0.0, math.radians(turn_deg), n):
        pts.append((lead + radius * math.sin(a), radius * (1 - math.cos(a))))
    return pts


def _lane(sid, coords, role="junction-via", eligible=True):
    return {"source_lane_id": sid, "role": role, "comparison": {"eligible": eligible},
            "support": {"reason": "source-topology-gap-bridge"}, "geometry": {"coordinates": coords}}


@pytest.mark.parametrize("radius", [3.0, 4.0, 6.0, 12.0])
def test_tightest_window_radius_of_a_circular_turn(radius):
    shape = T.turn_shape(_arc(radius))
    assert shape["radius_window_min_m"] == pytest.approx(radius, rel=0.03)
    # segment headings: the last chord points half an angular step short of the arc's end tangent
    step_deg = 90.0 / (max(2, int(math.radians(90.0) * radius / 0.25)) - 1)
    assert shape["total_turn_deg"] == pytest.approx(90.0 - step_deg / 2, abs=1e-6)
    assert T.turn_shape([(0, 0), (5, 0), (20, 0)])["radius_window_min_m"] is None


def test_only_comparable_junction_vias_tighter_than_five_metres_are_conflicts():
    manifest = {"lanes": [_lane("tight", _arc(4.0)), _lane("wide", _arc(6.0)),
                          _lane("approach", _arc(4.0), role="junction-approach"),
                          _lane("ineligible", _arc(4.0), eligible=False)]}
    found = T.findings(manifest)
    assert [f["source_lane_id"] for f in found] == ["tight"]
    f = found[0]
    assert f["rule"] == "tight-turn-source-conflict" and f["status"] == "RECORDED"
    assert f["resolution"] == "smooth-connector" and f["radius_window_min_m"] < T.RADIUS_MIN_M


def _case(monkeypatch, via_radius, offset=0.3):
    """Approach A followed exactly; via V written ``offset`` outside its source arc."""
    approach = [(x, -20.0) for x in np.arange(0.0, 40.0, 1.0)]
    via = _arc(via_radius, lead=8.0)
    shifted = [(x, y - offset) for x, y in via]
    manifest = {"lanes": [_lane("A", approach, role="junction-approach"), _lane("V", via)]}
    components = [{"source_lane_id": "A", "points": approach}, {"source_lane_id": "V", "points": shifted}]
    for module in (F, T):
        monkeypatch.setattr(module, "extract_target_components", lambda root, **kw: {"components": components})
    return manifest


def test_without_a_conflict_the_figure_equals_the_flare_free_one(monkeypatch):
    manifest = _case(monkeypatch, via_radius=8.0)
    review = {"findings": []}
    plain, out = F.audit(None, manifest, review), T.audit(None, manifest, review)
    assert out["tight_turn_source_conflicts"] == 0 and out["lane_center_conflict_samples_left_out"] == 0
    for name in ("median", "p95", "max"):
        assert out[f"lane_center_noconflict_{name}_m"] == plain[f"lane_center_noflare_{name}_m"]


def test_a_conflict_lane_is_left_out_whole(monkeypatch):
    manifest = _case(monkeypatch, via_radius=4.0)
    review = {"findings": []}
    plain, out = F.audit(None, manifest, review), T.audit(None, manifest, review)
    assert plain["lane_center_noflare_max_m"] > 0.2
    assert out["tight_turn_source_conflicts"] == 1 and out["lane_center_conflict_lanes_left_out"] == 1
    assert out["lane_center_conflict_samples_left_out"] > 0
    assert out["lane_center_noconflict_max_m"] < 1e-9
    assert T.audit(None, manifest, None) == {}


def test_policy_grades_shp_lane_centres_without_conflicts_with_unchanged_thresholds():
    policy = yaml.safe_load(sb.POLICY.read_bytes())
    assert float(policy["version"].removesuffix("-draft")) >= 0.8   # the 0.8 clause stays in later drafts
    t2 = {c["metric"]: c for c in policy["tiers"]["T2"]["checks"]}
    assert t2["lane_center_noconflict_p95_m"]["value"] == 0.15 and t2["lane_center_noconflict_p95_m"]["applies_to"] == ["shp"]
    assert t2["lane_center_noconflict_max_m"]["value"] == 0.35 and t2["lane_center_noconflict_max_m"]["applies_to"] == ["shp"]
    assert "lane_center_noflare_p95_m" not in t2 and "lane_center_noflare_max_m" not in t2
    assert t2["lane_center_p95_m"]["applies_to"] == ["map"]
    for name in ("lane_center_noconflict_p95_m", "tight_turn_source_conflicts"):
        assert name in sb.METRICS


def test_source_review_records_conflicts_only_when_found(monkeypatch):
    def fake_review(src, lane_ids, origin, meta=None, pairs=None):
        return {"thresholds": {}, "counts": {}, "open": 0, "recorded": 0, "findings": []}
    monkeypatch.setattr(R, "review", fake_review)
    crs = {"id": "local", "origin": {"lon": 0.0, "lat": 0.0}}
    clean = R.review_manifest({"comparison_crs": crs, "lanes": [_lane("wide", _arc(6.0))]}, src=object())
    assert clean == {"thresholds": {}, "counts": {}, "open": 0, "recorded": 0, "findings": [],
                     "comparison_crs": "local"}
    flagged = R.review_manifest({"comparison_crs": crs, "lanes": [_lane("tight", _arc(4.0))]}, src=object())
    assert [f["rule"] for f in flagged["findings"]] == ["tight-turn-source-conflict"]
    assert flagged["counts"] == {"tight-turn-source-conflict/lane": 1} and flagged["recorded"] == 1
    assert flagged["open"] == 0 and flagged["thresholds"]["tight_turn_radius_min_m"] == 5.0
    assert "2026-10-09" in flagged["decision_tight_turns"]
