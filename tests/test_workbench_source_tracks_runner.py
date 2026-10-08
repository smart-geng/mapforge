"""Probe orchestration uses controlled fake children; never compiles real maps."""
from __future__ import annotations

import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from scripts import workbench_run_source_tracks as R


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def report(status="EXPERIMENT_EVALUATED"):
    result = {
        "schema": "mapforge/wb11-source-tracks-probe/v1",
        "status": status,
        "stage": "WHOLE_MAP_SCORE",
        "candidate_accepted": False,
        "formal_release_verified": False,
        "independent_operator_verified": False,
        "default_postprocess_executed": True,
        "whole_map_score_executed": True,
        "code_policy_changed": [],
        "source_integrity_after": {"matches": True, "issues": []},
        "local_support_audit": {"status": "PASS", "candidate_accepted": False, "driving_continuity_proven": False, "source_support_audit": {"passed": True}, "families": [{"passed": True}]},
        "postprocessed_support_audit": {"status": "PASS", "candidate_accepted": False, "driving_continuity_proven": False, "source_support_audit": {"passed": True}, "families": [{"passed": True}]},
    }
    for key in ("local_support_audit", "postprocessed_support_audit"):
        result[key]["material_support_passed"] = True
        result[key]["written_road_mouth_coverage"] = {
            "within_existing_source_cover_tolerance": True,
            "all_exact_contacts_within_numeric_band": False,
            "driving_continuity_proven": False, "route_seams_evaluated": False,
            "rows": [{"road_id": "synthetic-road", "lane_id": -1, "lane_type": "driving",
                      "within_existing_source_cover_tolerance": True}],
        }
    return result


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def refresh_code_bindings(out):
    paths = R._required_code_paths()
    write_json(out / "code-policy-binding.json", {
        p.relative_to(R.ROOT).as_posix(): digest(p) for p in paths
    })
    for name in R.REQUIRED_OUTPUTS:
        if name.startswith("code-snapshots/"):
            relative = name.removeprefix("code-snapshots/")
            (out / name).write_bytes((R.ROOT / relative).read_bytes())


@pytest.fixture
def complete_output(tmp_path, monkeypatch):
    """Tiny byte-bound repository and artifacts, all synthetic; no geometry runs."""
    root = tmp_path / "fixture-repository"
    for name in (
        "mapforge/workbench/source_surface_tracks.py",
        "mapforge/workbench/source_surface_reconstruction.py",
        "mapforge/workbench/source_surface_support.py",
        "scripts/workbench_probe_source_tracks.py",
        "uv.lock", "profiles/shp/ibd-smarteditor-v1.yaml",
        "profiles/validation/static-acceptance-v1.draft.yaml",
        "profiles/validation/g8-opendrive-jinfeng-v1.yaml",
        *R.EXTRA_DEPENDENCIES,
    ):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# synthetic test file: " + name + "\n", encoding="utf-8")
    monkeypatch.setattr(R, "ROOT", root)
    monkeypatch.setattr(R, "PROBE", root / "scripts/workbench_probe_source_tracks.py")
    out = tmp_path / "complete-output"
    for name in R.REQUIRED_OUTPUTS:
        path = out / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}" if name.endswith(".json") else "synthetic evidence " + name,
                        encoding="utf-8")
    refresh_code_bindings(out)
    r = report()
    for field, name in (
        ("source_partial_sha256", "source-generated.partial.xodr"),
        ("unaccepted_surface_sha256", "surface.unaccepted.xodr"),
        ("original_surface_reference_sha256", "original-surface.reference.unaccepted.xodr"),
        ("candidate_sha256", "candidate.xodr"),
    ):
        r[field] = digest(out / name)
    r["source_snapshot_id"] = "synthetic-source-snapshot"
    snapshot = {"snapshot_id": r["source_snapshot_id"], "content_hash": "synthetic-content",
                "junction_id": "synthetic-junction"}
    intent = {"source_snapshot_id": snapshot["snapshot_id"],
              "source_content_hash": snapshot["content_hash"],
              "junction_id": snapshot["junction_id"], "confirmed": True}
    write_json(out / "source-snapshot.json", snapshot)
    write_json(out / "confirmed-probe-intent.json", intent)
    write_json(out / "surface-evidence.json", {
        "written_sha256": r["unaccepted_surface_sha256"], "intent": intent,
        "candidate_accepted": False, "non_paving_unchanged": True,
        "actual_xml_support_audit": r["local_support_audit"],
        "non_paving_subtrees_before": {"road:10": "synthetic-canonical-hash"},
    })
    write_json(out / "original-surface.reference-evidence.json", {
        "written_sha256": r["original_surface_reference_sha256"],
        "non_paving_subtrees_before": {"road:10": "synthetic-canonical-hash"},
    })
    r["preprocess_mouth_comparison"] = {
        "schema": "mapforge/preprocess-mouth-inheritance/v1", "allow_postprocess": True,
        "current_material_support_passed": True, "non_paving_unchanged": True,
        "regressions": [], "final_mouth_gate_unchanged": True,
        "current_mouth_coverage": r["local_support_audit"]["written_road_mouth_coverage"],
        "baseline_mouth_coverage": r["local_support_audit"]["written_road_mouth_coverage"],
        "baseline_origin": "old auxiliary reconstruction of the same freshly generated partial XML",
        "candidate_accepted": False,
    }
    write_json(out / "preprocess-mouth-comparison.json", r["preprocess_mouth_comparison"])
    write_json(out / "postprocessed-support-audit.json", r["postprocessed_support_audit"])
    # Completed experimental evaluation can have FAIL scores and blocked delivery.
    gates = {"G8": {"status": "FAIL"}, "G11": {"status": "FAIL"},
             "G11-edge-contacts": {"status": "PASS"}}
    for key, name in (("G8", "candidate.g8.json"), ("G11", "candidate.g11.json"),
                      ("G11-edge-contacts", "candidate.edge-contacts.json")):
        write_json(out / name, gates[key])
    r["delivery"] = {"schema": "mapforge/delivery-decision/v1", "status": "BLOCKED"}
    write_json(out / "candidate.delivery-decision.json", r["delivery"])
    write_json(out / "candidate.quality-report.json", {
        "schema": "mapforge/opendrive-quality-report/v1", "artifact": str(out / "candidate.xodr"),
        "gates": gates, "delivery_decision": r["delivery"],
    })
    r["tiers"] = {"T1": {"status": "FAIL"}, "T2": {"status": "FAIL"}}
    write_json(out / "scoreboard.json", {
        "schema": "mapforge/scoreboard/v1", "files": 1, "run_dir": str(out),
        "rows": [{"case": intent["junction_id"], "pipeline": "shp", "artifact": "candidate.xodr",
                  "metrics": {"synthetic_metric": 1.0}, "tiers": r["tiers"]}],
        "tier_pass": {"T1": 0, "T2": 0},
    })
    r["local_structure"] = {"xsd": {"passed": True}, "auxiliary_hard_failures": []}
    r["local_consumer"] = {"status": "PASS"}
    write_json(out / "local-structure.json", r["local_structure"])
    write_json(out / "local-consumer.json", r["local_consumer"])
    write_json(out / "report.json", r)
    return out


def change_json(out, name, key, value):
    path = out / name
    data = json.loads(path.read_text(encoding="utf-8"))
    data[key] = value
    write_json(path, data)


def test_complete_bound_evaluation_is_zero_even_when_quality_fails(complete_output):
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 0, result
    assert result["status"] == "EXPERIMENT_COMPLETE"
    assert result["experiment_complete"] is True
    assert result["candidate_accepted"] is False
    assert result["formal_release_verified"] is False
    assert result["problems"] == []
    assert set(result["evidence_bindings"]) == set(R.REQUIRED_OUTPUTS)


@pytest.mark.parametrize("field,value", [
    ("driving_continuity_proven", True),
    ("source_support_audit", {"passed": False}),
    ("families", []), ("families", [{"passed": True}, {"passed": False}]),
])
def test_inconsistent_surface_contract_cannot_pass(complete_output, field, value):
    out = complete_output
    report = json.loads((out / "report.json").read_text("utf8"))
    report["local_support_audit"][field] = value
    evidence = json.loads((out / "surface-evidence.json").read_text("utf8"))
    evidence["actual_xml_support_audit"] = report["local_support_audit"]
    write_json(out / "surface-evidence.json", evidence)
    write_json(out / "report.json", report)
    result = R.assess_output(out, 0)
    assert result["exit_code"] == 1
    assert "source-support phase did not pass" in result["problems"]


@pytest.mark.parametrize("key,value", [
    ("local_structure", {"xsd": {"passed": False}, "auxiliary_hard_failures": []}),
    ("local_structure", {"xsd": {"passed": True}, "auxiliary_hard_failures": [{"code": "degenerate_primitive"}]}),
    ("local_consumer", {"status": "UNAVAILABLE"}),
])
def test_local_structure_or_consumer_failure_is_not_complete(complete_output, key, value):
    out = complete_output
    change_json(out, "report.json", key, value)
    write_json(out / (key.replace("_", "-") + ".json"), value)
    result = R.assess_output(out, 0)
    assert result["exit_code"] == 1
    assert "local structure or consumer readback did not pass" in result["problems"]


@pytest.mark.parametrize("status", sorted(R.REJECTED - {"REJECTED_POSTPROCESS_MOUTHS"}))
@pytest.mark.parametrize("child_exit", [0, 2])
def test_explicit_rejection_is_nonzero(complete_output, status, child_exit):
    change_json(complete_output, "report.json", "status", status)
    result = R.assess_output(complete_output, child_exit)
    assert result["exit_code"] == 2
    assert not result["experiment_complete"]


def test_nonzero_completed_child_is_not_accepted(complete_output):
    assert R.assess_output(complete_output, 2)["exit_code"] == 1


def test_missing_script_binding_is_incomplete(complete_output):
    extra = R.ROOT / "scripts/consumer-helper.py"
    extra.write_text("# previously unbound helper", encoding="utf8")
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert "incomplete code/policy binding" in result["problems"]


def test_changed_candidate_bytes_are_not_complete(complete_output):
    (complete_output / "candidate.xodr").write_bytes(b"different")
    assert R.assess_output(complete_output, 0)["exit_code"] == 1


def test_timeout_keeps_incomplete_status(complete_output):
    result = R.assess_output(complete_output, 0, timed_out=True)
    assert result["exit_code"] == 3
    assert not result["experiment_complete"]


@pytest.mark.parametrize("audit_key", ["local_support_audit", "postprocessed_support_audit"])
@pytest.mark.parametrize("mouth", [None, {},
    {"within_existing_source_cover_tolerance": False, "driving_continuity_proven": False},
    {"within_existing_source_cover_tolerance": True, "driving_continuity_proven": True},
])
def test_missing_or_contradictory_actual_mouth_audit_is_incomplete(complete_output, audit_key, mouth):
    out = complete_output
    r = json.loads((out / "report.json").read_text("utf8"))
    if mouth is None:
        r[audit_key].pop("written_road_mouth_coverage")
    else:
        r[audit_key]["written_road_mouth_coverage"] = mouth
    if audit_key == "local_support_audit":
        evidence = json.loads((out / "surface-evidence.json").read_text("utf8"))
        evidence["actual_xml_support_audit"] = r[audit_key]
        write_json(out / "surface-evidence.json", evidence)
    else:
        write_json(out / "postprocessed-support-audit.json", r[audit_key])
    write_json(out / "report.json", r)
    result = R.assess_output(out, 0)
    assert result["exit_code"] == 1
    assert not result["experiment_complete"]


def persist_audit_change(out, report, *, local=False):
    if local:
        evidence = json.loads((out / "surface-evidence.json").read_text("utf8"))
        evidence["actual_xml_support_audit"] = report["local_support_audit"]
        write_json(out / "surface-evidence.json", evidence)
        write_json(out / "preprocess-mouth-comparison.json", report["preprocess_mouth_comparison"])
    else:
        write_json(out / "postprocessed-support-audit.json", report["postprocessed_support_audit"])
    write_json(out / "report.json", report)


def final_mouth_failure(out):
    report = json.loads((out / "report.json").read_text("utf8"))
    report["status"] = "REJECTED_POSTPROCESS_MOUTHS"
    audit = report["postprocessed_support_audit"]
    audit["status"] = "FAIL"
    mouth = audit["written_road_mouth_coverage"]
    mouth["within_existing_source_cover_tolerance"] = False
    mouth["rows"][0]["within_existing_source_cover_tolerance"] = False
    persist_audit_change(out, report)
    return report


def test_preprocess_inherited_failure_remains_visible_after_final_repair(complete_output):
    out = complete_output
    report = json.loads((out / "report.json").read_text("utf8"))
    audit = report["local_support_audit"]
    audit["status"] = "FAIL"
    mouth = audit["written_road_mouth_coverage"]
    mouth["within_existing_source_cover_tolerance"] = False
    mouth["rows"][0]["within_existing_source_cover_tolerance"] = False
    comparison = report["preprocess_mouth_comparison"]
    comparison["current_mouth_coverage"] = mouth
    comparison["baseline_mouth_coverage"] = mouth
    persist_audit_change(out, report, local=True)
    result = R.assess_output(out, 0)
    assert result["exit_code"] == 0, result
    assert result["experiment_complete"]
    assert result["diagnostic_evaluation_complete"]
    assert json.loads((out / "report.json").read_text("utf8"))["local_support_audit"]["status"] == "FAIL"
    assert result["candidate_accepted"] is False


@pytest.mark.parametrize("child_exit", [0, 2])
def test_final_mouth_failure_keeps_complete_scores_but_rejects_nonzero(complete_output, child_exit):
    final_mouth_failure(complete_output)
    result = R.assess_output(complete_output, child_exit)
    assert result["exit_code"] == 2, result
    assert result["status"] == "REJECTED_POSTPROCESS_MOUTHS"
    assert result["diagnostic_evaluation_complete"] is True
    assert result["experiment_complete"] is False
    assert result["candidate_accepted"] is False
    assert set(result["evidence_bindings"]) == set(R.REQUIRED_OUTPUTS)


@pytest.mark.parametrize("missing", ["scoreboard.json", "candidate.g8.json", "candidate.xodr",
                                     "preprocess-mouth-comparison.json"])
def test_rejected_final_mouth_does_not_hide_missing_diagnostic_evidence(complete_output, missing):
    final_mouth_failure(complete_output)
    (complete_output / missing).unlink()
    result = R.assess_output(complete_output, 2)
    assert result["exit_code"] == 1
    assert result["experiment_complete"] is False
    assert result.get("diagnostic_evaluation_complete") is not True


def test_final_mouth_rejection_label_requires_actual_final_mouth_failure(complete_output):
    change_json(complete_output, "report.json", "status", "REJECTED_POSTPROCESS_MOUTHS")
    result = R.assess_output(complete_output, 2)
    assert result["exit_code"] == 1
    assert result.get("diagnostic_evaluation_complete") is not True


def test_final_source_material_failure_cannot_masquerade_as_mouth_only_failure(complete_output):
    report = final_mouth_failure(complete_output)
    report["postprocessed_support_audit"]["material_support_passed"] = False
    persist_audit_change(complete_output, report)
    result = R.assess_output(complete_output, 2)
    assert result["exit_code"] == 1
    assert result.get("diagnostic_evaluation_complete") is not True


@pytest.mark.parametrize("key,value", [
    ("allow_postprocess", False), ("current_material_support_passed", False),
    ("non_paving_unchanged", False), ("regressions", [{"code": "mouth-gap-expanded-or-shifted"}]),
    ("final_mouth_gate_unchanged", False),
])
def test_preprocess_permission_requires_complete_nonregression_contract(complete_output, key, value):
    out = complete_output
    report = json.loads((out / "report.json").read_text("utf8"))
    report["preprocess_mouth_comparison"][key] = value
    persist_audit_change(out, report, local=True)
    result = R.assess_output(out, 0)
    assert result["exit_code"] == 1
    assert not result["experiment_complete"]


def test_reference_must_share_original_physical_subtrees(complete_output):
    change_json(complete_output, "original-surface.reference-evidence.json",
                "non_paving_subtrees_before", {"road:10": "different-physical-road"})
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1


def test_reference_bytes_bound_even_when_only_final_mouths_fail(complete_output):
    final_mouth_failure(complete_output)
    (complete_output / "original-surface.reference.unaccepted.xodr").write_bytes(b"different reference")
    result = R.assess_output(complete_output, 2)
    assert result["exit_code"] == 1
    assert result.get("diagnostic_evaluation_complete") is not True
