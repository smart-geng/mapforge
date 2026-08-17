import copy
import json

import numpy as np
import pytest

from mapforge.validate.g8_model import (make_manifest, source_lane,
                                         validate_manifest)
from mapforge.validate.lane_fidelity import evaluate_g8


def test_manifest_rejects_duplicate_source_ids():
    lane = source_lane(
        "A", np.array([[0.0, 0.0], [1.0, 0.0]]),
        owner={"format": "synthetic"}, role="approach", status="EXACT",
        support_kind="synthetic", policy_class="test.measured",
        travel_direction="with_s",
    )
    with pytest.raises(ValueError, match="重复 source_lane_id"):
        make_manifest(source_format="synthetic", source_profile="test",
                      comparison_crs={"units": "m"}, lanes=[lane, copy.deepcopy(lane)])


def test_manifest_requires_metric_crs(build_g8_case):
    _out, manifest = build_g8_case()
    manifest["comparison_crs"]["units"] = "degree"
    assert "comparison_crs_not_metric" in validate_manifest(manifest)


def test_baseline_g8_passes_and_is_strict_json(build_g8_case, g8_policy):
    out, manifest = build_g8_case()
    result = evaluate_g8(out, manifest, g8_policy)
    assert result["status"] == "PASS", result
    assert result["scope"]["eligible_source_lanes"] == 2
    assert result["scope"]["matched_source_lanes"] == 2
    assert not result["failure_reasons"]
    json.dumps(result, allow_nan=False)


def test_draft_policy_is_unavailable_but_keeps_metrics(build_g8_case, g8_policy):
    out, manifest = build_g8_case()
    policy = copy.deepcopy(g8_policy)
    policy["lifecycle"] = "draft"
    result = evaluate_g8(out, manifest, policy)
    assert result["status"] == "UNAVAILABLE"
    assert "policy_not_active" in result["failure_reasons"]
    assert result["metrics"]["source_to_target"]["count"] > 0


def test_missing_manifest_is_unavailable(build_g8_case, g8_policy):
    out, _manifest = build_g8_case()
    result = evaluate_g8(out, None, g8_policy)
    assert result["status"] == "UNAVAILABLE"
    assert "manifest_missing" in result["failure_reasons"]
