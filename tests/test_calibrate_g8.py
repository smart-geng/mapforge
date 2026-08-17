import hashlib
import json
from pathlib import Path

import yaml

from mapforge.validate.g8_model import GATE_SCHEMA, load_policy
from scripts.calibrate_g8 import calibrate


CASES = ["node3", "node4", "NODE5", "node13", "node16", "node17", "node18"]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _policy():
    limits = {
        "source_to_target": {"median_max_m": 0.6, "p95_max_m": 1.5, "max_max_m": 3.0},
        "target_to_source": {"median_max_m": 0.6, "p95_max_m": 1.5, "max_max_m": 3.0},
        "endpoint_max_m": 1.5,
        "stopline_delta_max_m": 1.5,
        "coverage_radius_m": 1.5,
        "source_coverage_min": 0.95,
        "target_coverage_min": 0.95,
        "length_ratio": {"min": 0.85, "max": 1.15},
    }
    return {
        "schema": "mapforge/g8-policy/v1",
        "policy_id": "test-g8",
        "version": "1.0-draft",
        "lifecycle": "draft",
        "applicability": {"target_format": "opendrive", "source_formats": ["map"]},
        "classes": {"test.measured": limits},
        "calibration": {
            "note": "fixture",
            "calibration_nodes": CASES[:4],
            "locked_validation_nodes": CASES[4:],
            "engineering_ceiling": {
                "median_max_m": 0.6,
                "p95_max_m": 1.5,
                "max_max_m": 3.0,
                "endpoint_max_m": 1.5,
                "stopline_delta_max_m": 1.5,
                "coverage_min": 0.95,
            },
        },
    }


def _lane(source_lane_id, median=0.0):
    stats = {"median_m": median, "p95_m": median, "max_m": median, "count": 2}
    return {
        "source_lane_id": source_lane_id,
        "policy_class": "test.measured",
        "source_to_target": stats,
        "target_to_source": stats,
        "endpoint": {"travel_start_m": 0.0, "travel_end_m": 0.0},
        "stopline": {"applicable": False, "delta_m": None},
        "coverage": {"source": 1.0, "target": 1.0,
                     "target_source_length_ratio": 1.0},
    }


def _fixture(tmp_path, *, high_median=False, lifecycle="draft"):
    policy = _policy()
    policy["lifecycle"] = lifecycle
    if lifecycle == "active":
        policy["version"] = "1.0"
    policy_path = tmp_path / "policy.yaml"
    policy_path.write_text(yaml.safe_dump(policy, sort_keys=False), encoding="utf-8")
    policy_hash = load_policy(policy_path)["policy_sha256"]
    entries = []
    for case in CASES:
        for pipeline in ("map-to-opendrive", "shp-to-opendrive"):
            stem = f"{case}-{pipeline}"
            artifact = tmp_path / f"{stem}.xodr"
            artifact.write_text("<OpenDRIVE/>", encoding="utf-8")
            median = 0.7 if high_median and case == "node3" and pipeline == "map-to-opendrive" else 0.0
            gate = {
                "schema": GATE_SCHEMA,
                "gate_id": "G8",
                "status": "PASS" if lifecycle == "active" else "UNAVAILABLE",
                "policy": {"sha256": policy_hash},
                "per_lane": [_lane(stem, median)],
                "failure_reasons": [] if lifecycle == "active" else ["policy_not_active"],
            }
            gate_path = tmp_path / f"{stem}.g8.json"
            gate_path.write_text(json.dumps(gate), encoding="utf-8")
            entries.append({
                "case": case,
                "pipeline": pipeline,
                "artifact": str(artifact),
                "artifact_sha256": _sha256(artifact),
                "policy_sha256": policy_hash,
                "sidecars": {"gate_result": str(gate_path)},
            })
    index_path = tmp_path / "index.json"
    index_path.write_text(json.dumps({
        "schema": "mapforge/closed-loop-inputs/v1",
        "suite": "fixture",
        "report_only": True,
        "entries": entries,
        "errors": [],
    }), encoding="utf-8")
    return policy_path, index_path


def test_calibration_writes_separate_active_candidate(tmp_path):
    policy_path, index_path = _fixture(tmp_path)
    report_path = tmp_path / "report.json"
    candidate_path = tmp_path / "candidate.yaml"

    report = calibrate(policy_path, index_path, report_path, candidate_path)

    assert report["status"] == "PASS"
    assert report["mode"] == "calibrate"
    assert report["candidate"]["written"] is True
    assert candidate_path.exists()
    source = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    candidate = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))
    assert source["lifecycle"] == "draft"
    assert candidate["lifecycle"] == "active"
    assert candidate["version"] == "1.0"
    assert candidate["classes"] == source["classes"]
    assert candidate["calibration"]["promotion"]["threshold_fitting"] is False


def test_calibration_ceiling_violation_removes_candidate(tmp_path):
    policy_path, index_path = _fixture(tmp_path, high_median=True)
    report_path = tmp_path / "report.json"
    candidate_path = tmp_path / "candidate.yaml"
    candidate_path.write_text("stale", encoding="utf-8")

    report = calibrate(policy_path, index_path, report_path, candidate_path)

    assert report["status"] == "FAIL"
    assert report["mode"] == "calibrate"
    assert "calibration_engineering_ceiling_exceeded" in report["failure_reasons"]
    assert report["candidate"]["written"] is False
    assert not candidate_path.exists()


def test_active_policy_reverifies_without_touching_candidate(tmp_path):
    """提升后复验：不再报 not_draft，也绝不删除已产出的候选证据。"""
    policy_path, index_path = _fixture(tmp_path, lifecycle="active")
    report_path = tmp_path / "report.json"
    candidate_path = tmp_path / "candidate.yaml"
    candidate_path.write_text("promoted-evidence", encoding="utf-8")

    report = calibrate(policy_path, index_path, report_path, candidate_path)

    assert report["mode"] == "verify"
    assert report["status"] == "PASS"
    assert report["failure_reasons"] == []
    assert report["candidate"]["written"] is False
    assert candidate_path.read_text(encoding="utf-8") == "promoted-evidence"
    assert report["splits"]["locked_validation"]["engineering_ceiling_violations"] == []


def test_active_policy_reverify_still_fails_on_regression(tmp_path):
    """复验不是橡皮图章：现有产物一旦超出冻结上限仍必须 FAIL。"""
    policy_path, index_path = _fixture(tmp_path, high_median=True, lifecycle="active")
    report_path = tmp_path / "report.json"
    candidate_path = tmp_path / "candidate.yaml"
    candidate_path.write_text("promoted-evidence", encoding="utf-8")

    report = calibrate(policy_path, index_path, report_path, candidate_path)

    assert report["mode"] == "verify"
    assert report["status"] == "FAIL"
    assert "calibration_engineering_ceiling_exceeded" in report["failure_reasons"]
    assert candidate_path.exists()
