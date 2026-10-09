"""Unlinked SHP stop lines (user decisions 2026-10-09): nearest match within 2 m, else a recorded source gap."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from mapforge.validate import scoreboard as sb
from mapforge.validate import shp_source_review as R
from mapforge.validate import stopline_match as S

ROOT = Path(__file__).resolve().parents[1]
ORIGIN = {"lon": 106.3, "lat": 29.5}
W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2/candidate.xodr"
G0412 = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3/shp-2023041216222042093.xodr"


def _deg(xy):
    """Local metres back to lon/lat about ORIGIN (inverse of the comparison projection)."""
    xy = np.asarray(xy, float)
    lat0 = math.radians(ORIGIN["lat"])
    return np.column_stack([ORIGIN["lon"] + np.degrees(xy[:, 0] / (S.R_EARTH * math.cos(lat0))),
                            ORIGIN["lat"] + np.degrees(xy[:, 1] / S.R_EARTH)])


class _Source:
    """The two things stopline_match reads from a ProfileSource: the stop-line layer and lane geometry."""

    def __init__(self, stops, lanes):
        self.L = {"stop_line": {"file": "IBD_OBJECT_STOPLINE", "list_sep": ";",
                                "fields": {"id": "OBJECT_PID", "lane_refs": "LANE_REL"}}}
        self._stops, self._lanes = stops, lanes

    def _iter(self, spec):
        for pid, refs, xy in self._stops:
            yield {"OBJECT_PID": pid, "LANE_REL": refs}, SimpleNamespace(points=_deg(xy).tolist())

    def lane(self, pid):
        xy = self._lanes.get(pid)
        return None if xy is None else SimpleNamespace(geometry=_deg(xy))


def _manifest(*lanes):
    return {"comparison_crs": {"id": "local-eqc", "units": "m", "origin": ORIGIN}, "lanes": list(lanes)}


def _lane(sid, window, stop=None, eligible=True):
    return {"source_lane_id": sid, "role": "approach", "comparison": {"eligible": eligible},
            "travel": {"coordinate_order": "with-travel"}, "geometry": {"coordinates": window},
            "stop_line": stop or {"availability": "unavailable", "reason": S.UNLINKED}}


CROSS = lambda x: [(x, -2.0), (x, 2.0)]   # noqa: E731  a stop line across the lane at x


def test_nearest_stop_line_within_two_metres_is_inferred():
    src = _Source([("far", "other", CROSS(60.0)), ("near", "other", CROSS(51.0))], {"A": [(0, 0), (50, 0)]})
    found = S.findings(_manifest(_lane("A", [[10, 0], [40, 0]])), src)
    assert len(found) == 1
    f = found[0]
    assert f["rule"] == S.RULE_MATCH and f["status"] == "INFERRED" and f["stopline_id"] == "near"
    assert f["distance_m"] == pytest.approx(1.0, abs=1e-3) and f["stopline_lane_refs"] == ["other"]


def test_downstream_end_follows_travel_not_digitising_order():
    """Source digitised against travel: the end is still the one past the window's travel end."""
    src = _Source([("near", "x", CROSS(51.0)), ("behind", "x", CROSS(-1.5))], {"A": [(50, 0), (0, 0)]})
    assert S.findings(_manifest(_lane("A", [[10, 0], [40, 0]])), src)[0]["stopline_id"] == "near"
    backwards = _lane("A", [[40, 0], [10, 0]])
    assert S.findings(_manifest(backwards), src)[0]["stopline_id"] == "behind"


def test_none_within_two_metres_is_a_recorded_source_gap():
    src = _Source([("s", "other", CROSS(53.0))], {"A": [(0, 0), (50, 0)]})
    f = S.findings(_manifest(_lane("A", [[10, 0], [40, 0]])), src)[0]
    assert f["rule"] == S.RULE_ABSENT and f["status"] == "RECORDED"
    assert f["nearest_stopline_id"] == "s" and f["nearest_distance_m"] == pytest.approx(3.0, abs=1e-3)


def test_only_unlinked_lanes_are_looked_at():
    linked = _lane("L", [[0, 0], [1, 0]], stop={"availability": "available", "source_id": "x"})
    plain = _lane("P", [[0, 0], [1, 0]], stop={"availability": "not-applicable"})
    assert S.findings(_manifest(linked, plain), src=object()) == []


def test_matched_lanes_carry_the_stop_line_others_stay():
    src = _Source([("near", "x", CROSS(51.0))], {"A": [(0, 0), (50, 0)], "B": [(0, 9), (50, 9)]})
    manifest = _manifest(_lane("A", [[10, 0], [40, 0]]), _lane("B", [[10, 9], [20, 9]]))
    src._stops.append(("far", "y", [(80.0, 7.0), (80.0, 11.0)]))
    found = S.findings(manifest, src)
    out = S.resolved_manifest(manifest, found)
    a, b = out["lanes"]
    assert a["stop_line"]["availability"] == "available" and a["stop_line"]["association"] == "INFERRED"
    assert b["stop_line"] == manifest["lanes"][1]["stop_line"]
    assert manifest["lanes"][0]["stop_line"]["availability"] == "unavailable"   # the input is not modified


@pytest.mark.parametrize("reasons, unmeasurable, absent, errors, expected", [
    (["unavailable:A"], ["A:stopline"], {"A"}, [], ("PASS", ["A"])),
    (["unavailable:A", "B:coverage.source<0.95"], ["A:stopline"], {"A"}, [], ("FAIL", ["A"])),
    (["unavailable:A", "unavailable:B"], ["A:stopline", "B:stopline"], {"A"}, [], ("UNAVAILABLE", ["A"])),
    (["unavailable:A"], ["A", "A:stopline"], {"A"}, [], ("UNAVAILABLE", [])),
    (["unavailable:A"], ["A:stopline"], {"A"}, ["policy_not_active"], ("UNAVAILABLE", ["A"])),
])
def test_only_the_stop_line_requirement_of_absent_lanes_is_left_out(reasons, unmeasurable, absent, errors, expected):
    gate = {"failure_reasons": reasons, "issues": {"unmeasurable_source_ids": unmeasurable}}
    assert S.resolved_status(gate, absent, errors) == expected


def test_policy_grades_the_resolved_g8_status():
    policy = yaml.safe_load(sb.POLICY.read_bytes())
    assert tuple(int(x) for x in policy["version"].removesuffix("-draft").split(".")) >= (0, 10)
    t1 = {c["metric"]: c for c in policy["tiers"]["T1"]["checks"]}
    assert t1["g8_status_stopline_resolved"] == {"metric": "g8_status_stopline_resolved", "op": "eq", "value": "PASS"}
    assert "g8_status" not in t1 and S.MATCH_RADIUS_M == 2.0
    for name in ("g8_status", "g8_status_stopline_resolved", "stopline_matched", "stopline_source_absent"):
        assert name in sb.METRICS


def test_source_review_records_stop_lines_only_when_found(monkeypatch):
    def fake_review(src, lane_ids, origin, meta=None, pairs=None):
        return {"thresholds": {}, "counts": {}, "open": 0, "recorded": 0, "findings": []}
    monkeypatch.setattr(R, "review", fake_review)
    clean = R.review_manifest(_manifest(_lane("A", [[10, 0], [40, 0]], stop={"availability": "not-applicable"})),
                              src=object())
    assert clean == {"thresholds": {}, "counts": {}, "open": 0, "recorded": 0, "findings": [],
                     "comparison_crs": "local-eqc"}
    src = _Source([("near", "x", CROSS(51.0))], {"A": [(0, 0), (50, 0)], "B": [(0, 9), (50, 9)]})
    flagged = R.review_manifest(_manifest(_lane("A", [[10, 0], [40, 0]]), _lane("B", [[10, 9], [20, 9]])), src=src)
    assert [f["rule"] for f in flagged["findings"]] == [S.RULE_MATCH, S.RULE_ABSENT]
    assert flagged["counts"] == {S.RULE_MATCH + "/lane": 1, S.RULE_ABSENT + "/lane": 1}
    assert flagged["recorded"] == 2 and flagged["open"] == 0
    assert flagged["thresholds"]["stopline_match_radius_m"] == 2.0 and "2026-10-09" in flagged["decision_stoplines"]


def _real(xodr):
    from mapforge.adapters.shp.profile_source import ProfileSource
    manifest = json.loads(xodr.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
    g8 = json.loads(xodr.with_suffix(".g8.json").read_text(encoding="utf-8"))
    return manifest, g8, ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")


@pytest.mark.skipif(not (W0621.is_file() and (ROOT / "shp_0222-0326").is_dir()), reason="0621 evidence not restored")
def test_0621_has_no_source_stop_line_for_its_two_approaches():
    manifest, g8, src = _real(W0621)
    out = S.audit(W0621, manifest, g8, src)
    assert g8["status"] == "UNAVAILABLE"
    assert out["stopline_source_absent"] == out["g8_stopline_requirements_left_out"] == 2
    assert out["stopline_matched"] == 0 and out["g8_status_stopline_resolved"] == "PASS"
    near = {f["source_lane_id"]: f["nearest_distance_m"] for f in S.findings(manifest, src) if f["g8_comparable"]}
    assert near == pytest.approx({"2023061413201352024": 119.619, "2023061413450449227": 110.367}, abs=1e-3)


@pytest.mark.skipif(not (G0412.is_file() and (ROOT / "shp_0222-0326").is_dir()), reason="0412 evidence not restored")
def test_0412_matches_the_stop_line_across_its_approach_and_keeps_its_other_failure():
    manifest, g8, src = _real(G0412)
    out = S.audit(G0412, manifest, g8, src)
    assert out["stopline_matched"] == 2 and out["stopline_source_absent"] == 0
    assert out["stopline_match_distance_max_m"] == pytest.approx(0.446, abs=1e-3)
    assert out["stopline_matched_delta_max_m"] < 1.5
    assert out["g8_status_stopline_resolved"] == "FAIL"     # lane 2023041215214931213 still exceeds its thresholds
