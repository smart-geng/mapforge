"""Actual XML extraction and report-binding failures for read-only locations."""
import copy
import json
from pathlib import Path

from lxml import etree
import pytest

from mapforge.validate import g8_model as M
from mapforge.validate.lane_fidelity import evaluate_g8
from mapforge.workbench import fidelity_inspection as F


@pytest.fixture
def case(build_g8_case, g8_policy):
    path, manifest = build_g8_case()
    root = etree.parse(str(path)).getroot()
    manifest["comparison_crs"].update(id="local-eqc", axis_order=["x", "y"],
                                      proj_string=root.findtext("header/geoReference"))
    manifest["lanes"][0]["stop_line"] = {"availability": "unavailable", "reason": "source-stopline-not-linked"}
    manifest["manifest_sha256"] = M.object_sha256({k: v for k, v in manifest.items() if k != "manifest_sha256"})
    g8_policy["classes"]["test.measured"]["require_stopline"] = True
    gate = evaluate_g8(root, manifest, g8_policy)
    return root, manifest, gate, g8_policy


def test_unavailable_gate_still_locates_real_lines_without_claiming_pass(case):
    root, manifest, gate, policy = case
    before = etree.tostring(root), copy.deepcopy(manifest), copy.deepcopy(gate), copy.deepcopy(policy)
    result = F.inspect_fidelity(*case)
    assert result["status"] == "AVAILABLE" and result["gate_status"] == "UNAVAILABLE"
    assert result["summary"] == {"lane_count": 2, "stopline_unavailable_count": 1}
    assert result["whole_map_score_modified"] is result["per_lane_tiers_assigned"] is False
    a = next(row for row in result["lanes"] if row["source_lane_id"] == "A")
    assert a["issues"][0]["code"] == "stopline-unmeasurable"
    assert a["source_geometry"]["coordinates"] == manifest["lanes"][0]["geometry"]["coordinates"]
    assert a["target_geometry"]["coordinates"][0] == [0., -1.75]
    assert a["target_geometry"]["coordinates"][-1] == [100., -1.75]
    assert a["source_to_target"] == gate["per_lane"][0]["source_to_target"]
    assert (etree.tostring(root), manifest, gate, policy) == before
    a["stop_line"]["availability"] = "invented"
    assert manifest["lanes"][0]["stop_line"]["availability"] == "unavailable"


@pytest.mark.parametrize("fault", ["manifest-hash", "frame", "offset", "target", "duplicate", "missing-row",
    "missing-issues", "missing-stopline", "foreign-stopline", "duplicate-stopline", "scope", "nan", "policy",
    "source-geometry", "actual-source-id", "actual-width", "empty"])
def test_incomplete_or_mismatched_report_never_displays_partial_or_zero_success(case, fault):
    root, manifest, gate, policy = case
    if fault == "manifest-hash": manifest["manifest_sha256"] = "0" * 64
    elif fault == "frame": root.find("header/geoReference").text += " +x_0=10"
    elif fault == "offset": etree.SubElement(root.find("header"), "offset", x="0", y="0", z="0", hdg="0")
    elif fault == "target": gate["per_lane"][0]["target"]["lane_id"] = -2
    elif fault == "duplicate": gate["per_lane"].append(copy.deepcopy(gate["per_lane"][0]))
    elif fault == "missing-row": gate["per_lane"].pop()
    elif fault == "missing-issues": del gate["issues"]["missing_source_ids"]
    elif fault == "missing-stopline": gate["issues"]["unmeasurable_source_ids"] = []
    elif fault == "foreign-stopline": gate["issues"]["unmeasurable_source_ids"] = ["X:stopline"]
    elif fault == "duplicate-stopline": gate["issues"]["unmeasurable_source_ids"] *= 2
    elif fault == "scope": del gate["scope"]["excluded_occurrences"]
    elif fault == "nan": gate["per_lane"][0]["source_to_target"]["p95_m"] = float("nan")
    elif fault == "policy": policy["sampling"]["step_m"] = .5
    elif fault == "source-geometry": manifest["lanes"][0]["geometry"]["coordinates"][0][0] = 999
    elif fault == "actual-source-id": root.find(".//userData[@code='mapforge.source_lane']").set("value", "X")
    elif fault == "actual-width": root.find(".//right/lane/width").set("a", "4")
    elif fault == "empty": gate["per_lane"] = []
    result = F.inspect_fidelity(*case)
    assert result["status"] == "UNAVAILABLE", fault
    assert result["lanes"] == [] and result["summary"] is None


def test_display_preserves_support_domain_and_reports_exclusions(case):
    root, manifest, _, policy = case
    lane = root.find(".//right/lane")
    ud = lane.find("userData[@code='mapforge.provenance/v1']")
    provenance = json.loads(ud.get("value"))
    provenance["support_s"] = [20, 60]
    ud.set("value", json.dumps(provenance))
    record = manifest["lanes"][0]
    record["geometry"]["coordinates"] = [[20., -1.75], [60., -1.75]]
    record["geometry"]["geometry_sha256"] = M.geometry_sha256(record["geometry"]["coordinates"])
    manifest["manifest_sha256"] = M.object_sha256({k: v for k, v in manifest.items() if k != "manifest_sha256"})
    gate = evaluate_g8(root, manifest, policy)
    result = F.inspect_fidelity(root, manifest, gate, policy)
    assert result["status"] == "AVAILABLE"
    a = next(row for row in result["lanes"] if row["source_lane_id"] == "A")
    assert a["target_geometry"]["coordinates"][0][0] == 20.
    assert a["target_geometry"]["coordinates"][-1][0] == 60.
    assert result["scope"]["excluded_occurrences"] == 1


@pytest.mark.parametrize("points", [["12", "34"], [["1", 2], [3, 4]], [[True, 2], [3, 4]],
                                    [[0, 0], [0, 0]], [[float("inf"), 2], [3, 4]]])
def test_display_rejects_non_numeric_or_degenerate_points(points):
    with pytest.raises(ValueError):
        F._line(points)


def test_real_frozen_candidate_locations_match_original_report_and_raw_refs():
    from mapforge.workbench.surface_review import RegisteredSurfaceReview
    directory = Path(__file__).resolve().parents[1] / "out/workbench/wb11-source-tracks-20261009-v2"
    if not directory.is_dir():
        pytest.skip("local frozen research evidence is not shipped in the repository")
    before = {p: p.read_bytes() for p in directory.rglob("*") if p.is_file()}
    result = F.inspect_bound_review(RegisteredSurfaceReview(directory))
    assert result["status"] == "AVAILABLE"
    assert result["summary"] == {"lane_count": 18, "stopline_unavailable_count": 2}
    assert result["scope"]["excluded_occurrences"] == 139
    assert result["lanes"][0]["target"]["road_id"] == "100"
    assert result["lanes"][0]["source_to_target"]["p95_m"] == .2939857457642104
    approach = next(row for row in result["lanes"] if row["source_lane_id"] == "2023061413201352024")
    assert approach["primary_source_ref"]["source_ref"]["record_index"] == 2561
    assert {ref["source_ref"]["layer"] for ref in approach["source_refs"]} == {"IBD_LANE_LINK", "IBD_LANE_LINK_MERGE"}
    assert all(p.read_bytes() == data for p, data in before.items())
