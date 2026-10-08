"""One real whole-map replay, then input rejection and store attachment checks."""
import copy
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from mapforge.workbench.compiler import ROOT
from mapforge.workbench.contracts import digest
from mapforge.workbench.store import ProjectStore
from mapforge.workbench.validation import (ValidationRejected, validate_request,
                                           validation_fingerprints, verify_validation_artifacts)

EVIDENCE = ROOT / "out/workbench/wb05-compiler-20261008/wb05-summary.json"
pytestmark = pytest.mark.skipif(not EVIDENCE.is_file(), reason="requires the actual accepted WB05 candidate evidence")


def copied_accepted_project(base):
    summary = json.loads(EVIDENCE.read_text(encoding="utf8"))
    original = Path(summary["project_directory"])
    # WB05 intentionally ended with a rejected draft. The previous complete
    # snapshot is the actually accepted -15mm candidate, not a fabricated one.
    envelope = json.loads((original / "previous.json").read_text(encoding="utf8"))
    project = envelope["payload"]
    assert project["candidate"]["accepted_epoch"] == project["draft_epoch"]
    store = ProjectStore(base / "projects")
    directory = store.project_path(project["project_id"])
    directory.mkdir()
    (directory / "project.json").write_text(json.dumps(envelope, ensure_ascii=False), encoding="utf8")
    for artifact in project["candidate"]["artifacts"]:
        target = directory / artifact["relative_path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original / artifact["relative_path"], target)
    project = store.load(project["project_id"])
    return store, project, {"project": project, "project_directory": str(directory)}


@pytest.fixture
def accepted(tmp_path):
    return copied_accepted_project(tmp_path)


@pytest.fixture(scope="module")
def real_result(tmp_path_factory):
    store, project, payload = copied_accepted_project(tmp_path_factory.mktemp("wb-validation-real"))
    result = validate_request(payload)
    return store, project, payload, result


def test_actual_whole_candidate_uses_existing_gates_not_tier_release(real_result):
    store, project, payload, result = real_result
    assert result["status"] == "VALIDATED", result
    validation = result["validation"]
    assert set(validation) == {"bundle_id", "candidate_id", "candidate_hash", "context", "checks", "decision"}
    assert validation["candidate_hash"] == digest(project["candidate"])
    assert validation["context"] == project["context"]
    assert validation["decision"] == "BLOCKED"
    board = result["report"]["scoreboard"]
    assert board["files"] == 2
    assert board["tier_pass"] == {"T1": 2, "T2": 2}
    codes = {r["code"] for r in result["report"]["candidate_decision"]["blocked_reasons"]}
    assert "crs_not_absolutely_verified" in codes
    assert "required_gate_not_pass" in codes
    for artifact in result["artifacts"]:
        assert hashlib.sha256((Path(payload["project_directory"]) / artifact["relative_path"]).read_bytes()).hexdigest() == artifact["sha256"]
    assert store.load(project["project_id"])["validation"] is None


def test_real_result_attaches_without_replacing_candidate(real_result):
    store, project, _, result = real_result
    assert result["status"] == "VALIDATED"
    updated = store.attach_validation(project["project_id"], project["revision"], "attach-whole-check", result["validation"])
    assert updated["candidate"] == project["candidate"]
    assert updated["validation"]["decision"] == "BLOCKED"
    assert updated["status"]["validation_stale"] is False
    assert updated["status"]["formal_export_available"] is False


def test_unaccepted_and_stale_candidate_rejected(accepted):
    _, project, payload = accepted
    bad = copy.deepcopy(project)
    bad["candidate"].pop("accepted_epoch")
    result = validate_request({**payload, "project": bad})
    assert result["status"] == "REJECTED" and result["error"]["code"] == "candidate-stale"
    bad = copy.deepcopy(project)
    bad["draft_epoch"] += 1
    result = validate_request({**payload, "project": bad})
    assert result["error"]["code"] == "candidate-stale"


def test_candidate_actual_bytes_mismatch_rejected(accepted):
    _, project, payload = accepted
    artifact = project["candidate"]["artifacts"][0]
    path = Path(payload["project_directory"]) / artifact["relative_path"]
    path.write_bytes(path.read_bytes() + b"\n")
    result = validate_request(payload)
    assert result["status"] == "REJECTED"
    assert result["error"]["code"] == "artifact-hash-mismatch"
    assert result["validation"] is None


def test_compile_evidence_cannot_be_rebound_to_another_candidate(accepted):
    _, project, payload = accepted
    proof = next(a for a in project["candidate"]["artifacts"] if a["relative_path"].endswith("evidence.json"))
    path = Path(payload["project_directory"]) / proof["relative_path"]
    data = json.loads(path.read_text(encoding="utf8"))
    data["candidate_id"] = "different-candidate"
    changed = json.dumps(data).encode()
    path.write_bytes(changed)
    proof["sha256"] = hashlib.sha256(changed).hexdigest()
    result = validate_request(payload)
    assert result["error"]["code"] == "compile-evidence-mismatch"


def test_artifact_path_and_project_path_escape_rejected(accepted):
    _, project, payload = accepted
    bad = copy.deepcopy(project)
    bad["candidate"]["artifacts"][0]["relative_path"] = "artifacts/../../escape.xodr"
    result = validate_request({**payload, "project": bad})
    assert result["error"]["code"] == "artifact-path-escape"
    result = validate_request({**payload, "project_directory": str(Path(payload["project_directory"]).parent)})
    assert result["error"]["code"] == "project-directory-mismatch"


def test_source_drift_and_context_drift_rejected(accepted, tmp_path):
    _, project, payload = accepted
    empty = tmp_path / "empty-source"
    empty.mkdir()
    result = validate_request({**payload, "source_dir": str(empty)})
    assert result["error"]["code"] == "source-drift"
    bad = copy.deepcopy(project)
    bad["context"]["compiler_hash"] = "0" * 64
    bad["candidate"]["context"] = bad["context"]
    result = validate_request({**payload, "project": bad})
    assert result["error"]["code"] == "context-stale"


def test_input_changes_during_computation_are_rejected(accepted, monkeypatch):
    _, project, payload = accepted
    original = Path(payload["project_directory"]) / project["candidate"]["artifacts"][0]["relative_path"]

    def changed_input(*args):
        original.write_bytes(original.read_bytes() + b"\n")
        return {}, {}, []

    # The actual expensive evaluator runs once above. Here only its duration
    # boundary is controlled to test the mandatory post-computation byte check.
    monkeypatch.setattr("mapforge.workbench.validation._evaluate_pair", changed_input)
    result = validate_request(payload)
    assert result["status"] == "REJECTED"
    assert result["error"]["code"] == "validation-input-drift"


@pytest.fixture
def artifact_bundle(tmp_path):
    """Filesystem-only attach contract fixture; no synthetic geometry verdict."""
    directory = tmp_path / ("c" * 32)
    bundle_id = "validation-" + "a" * 24 + "-" + "b" * 8
    output = directory / "artifacts" / bundle_id
    output.mkdir(parents=True)
    artifact = output / "check.json"
    artifact.write_bytes(b"{}")
    source = tmp_path / "input.xodr"
    source.write_bytes(b"filesystem binding fixture only")
    fingerprints = validation_fingerprints()
    files = [{"relative_path": artifact.relative_to(directory).as_posix(),
              "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest()}]
    context = {k: "0" * 64 for k in ("source_hash", "profile_hash", "compiler_hash", "policy_hash")}
    validation = {"bundle_id": bundle_id, "candidate_id": "candidate-fixture", "candidate_hash": "1" * 64,
                  "context": context, "decision": "BLOCKED", "checks": [
                      {"gate": "validation-byte-binding", "status": "PASS",
                       "validator_hash": fingerprints["validator_hash"], "files": files}]}
    report = {"project_id": directory.name, "bundle_id": bundle_id, "candidate_hash": "1" * 64,
              "context": context, "fingerprints": fingerprints,
              "input_files_sha256": {str(source): hashlib.sha256(source.read_bytes()).hexdigest()}}
    saved = {"status": "VALIDATED", "validation": validation, "artifacts": files, "report": report}
    record = output / "validation-result.json"
    record.write_text(json.dumps(saved), encoding="utf8")
    return directory, validation, report, artifact, source, record


def test_attach_helper_checks_real_output_and_input_bytes(artifact_bundle):
    directory, validation, report, artifact, source, _ = artifact_bundle
    assert verify_validation_artifacts(directory, validation) == report
    original = artifact.read_bytes()
    artifact.write_bytes(b"changed output")
    with pytest.raises(ValidationRejected) as caught:
        verify_validation_artifacts(directory, validation)
    assert caught.value.code == "artifact-hash-mismatch"
    artifact.write_bytes(original)
    source.write_bytes(b"changed input after validation")
    with pytest.raises(ValidationRejected) as caught:
        verify_validation_artifacts(directory, validation)
    assert caught.value.code == "validation-input-drift"


def test_attach_helper_cannot_upgrade_saved_decision(artifact_bundle):
    directory, validation, _, _, _, _ = artifact_bundle
    changed = copy.deepcopy(validation)
    changed["decision"] = "DELIVERABLE"
    with pytest.raises(ValidationRejected) as caught:
        verify_validation_artifacts(directory, changed)
    assert caught.value.code == "validation-report-mismatch"


def test_attach_helper_rejects_changed_validator(artifact_bundle, monkeypatch):
    directory, validation, report, _, _, _ = artifact_bundle
    changed = copy.deepcopy(report["fingerprints"])
    changed["validator_hash"] = "f" * 64
    monkeypatch.setattr("mapforge.workbench.validation.validation_fingerprints", lambda: changed)
    with pytest.raises(ValidationRejected) as caught:
        verify_validation_artifacts(directory, validation)
    assert caught.value.code == "validator-stale"
