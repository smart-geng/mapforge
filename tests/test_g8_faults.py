import copy
import json
import xml.etree.ElementTree as ET

from mapforge.validate.lane_fidelity import evaluate_g8


def test_two_metre_lateral_shift_fails(build_g8_case, g8_policy):
    out, manifest = build_g8_case()
    root = ET.parse(out).getroot()
    root.find("road/lanes/laneOffset").set("a", "2")
    result = evaluate_g8(root, manifest, g8_policy)
    assert result["status"] == "FAIL"
    assert any("source_to_target" in x for x in result["failure_reasons"])


def test_swapping_existing_lane_provenance_fails_without_missing(build_g8_case, g8_policy):
    out, manifest = build_g8_case()
    root = ET.parse(out).getroot()
    uds = root.findall(".//userData[@code='mapforge.source_lane']")
    assert [x.get("value") for x in uds] == ["A", "B"]
    uds[0].set("value", "B")
    uds[1].set("value", "A")
    result = evaluate_g8(root, manifest, g8_policy)
    assert result["status"] == "FAIL"
    assert not result["issues"]["missing_source_ids"]
    assert not result["issues"]["orphan_target_ids"]
    assert any("source_to_target" in x for x in result["failure_reasons"])


def test_trimming_target_end_fails_endpoint_and_coverage(build_g8_case, g8_policy):
    out, manifest = build_g8_case(target_length=80.0, source_length=100.0)
    result = evaluate_g8(out, manifest, g8_policy)
    assert result["status"] == "FAIL"
    reasons = "\n".join(result["failure_reasons"])
    assert "endpoint.end" in reasons
    assert "coverage.source" in reasons
    assert "length_ratio" in reasons


def test_orphan_target_lane_fails(build_g8_case, g8_policy):
    out, manifest = build_g8_case(second_source="fault:orphan")
    result = evaluate_g8(out, manifest, g8_policy)
    assert result["status"] == "FAIL"
    assert result["issues"]["orphan_target_ids"] == ["fault:orphan"]


def test_unprovenanced_positive_width_lane_fails(build_g8_case, g8_policy):
    out, manifest = build_g8_case(second_source=None, second_provenance=False)
    result = evaluate_g8(out, manifest, g8_policy)
    assert result["status"] == "FAIL"
    assert result["issues"]["unprovenanced_targets"]


def test_zero_width_unprovenanced_lane_is_explicit_exclusion(build_g8_case, g8_policy):
    out, manifest = build_g8_case(second_width=0.0, second_source=None,
                                  second_provenance=False)
    result = evaluate_g8(out, manifest, g8_policy)
    assert result["status"] == "PASS", result
    assert any(x["code"] == "zero-width" for x in result["exclusions"])
    json.dumps(result, allow_nan=False)


def test_only_exclusions_cannot_pass(build_g8_case, g8_policy):
    out, manifest = build_g8_case(include_second=False)
    manifest = copy.deepcopy(manifest)
    manifest["lanes"] = []
    root = ET.parse(out).getroot()
    lane = root.find(".//right/lane")
    lane.find("userData[@code='mapforge.source_lane']").set("value", "")
    lane.find("userData[@code='mapforge.provenance/v1']").set(
        "value", json.dumps({"eligibility": "excluded", "role": "departure",
                              "status": "INFERRED", "support_kind": "mirror",
                              "travel_direction": "against_s",
                              "exclusion_code": "mirror-no-source-geometry"}))
    result = evaluate_g8(root, manifest, g8_policy)
    assert result["status"] == "UNAVAILABLE"
    assert "unavailable:no_comparable_source_lanes" in result["failure_reasons"]
