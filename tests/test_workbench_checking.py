"""Checking coordination uses real files/store and the real validation readback.

Only expensive evaluator output is controlled; these tests do not claim map
quality. Whole-map evaluator integration lives in test_workbench_validation.
"""
import copy
import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from test_workbench_editing import setup, confirmed
from mapforge.workbench import checking, validation
from mapforge.workbench.checking import CheckingService
from mapforge.workbench.contracts import (
    StoreConflict, StoreReadOnly, StoreValidation, canonical_bytes, digest,
)


class CheckedJobs:
    def __init__(self, fingerprints):
        self.jobs, self.payloads = {}, {}
        self.fingerprints = fingerprints

    def start(self, project, operation, payload, request_id):
        identity = hashlib.sha256(request_id.encode()).hexdigest()
        job_id = identity[:32]
        if job_id in self.jobs:
            assert self.payloads[job_id] == payload
            return copy.deepcopy(self.jobs[job_id])
        self.payloads[job_id] = copy.deepcopy(payload)
        bundle_id = "validation-" + identity[:24] + "-" + identity[24:32]
        base = Path(payload["project_directory"])
        output = base / "artifacts" / bundle_id
        output.mkdir(parents=True)
        output_file = output / "scoreboard.json"
        output_file.write_bytes(canonical_bytes({"T1": "PASS", "T2": "PASS"}))
        files = [{"relative_path": output_file.relative_to(base).as_posix(),
                  "sha256": hashlib.sha256(output_file.read_bytes()).hexdigest()}]
        candidate = project["candidate"]
        fingerprints = copy.deepcopy(self.fingerprints)
        package = {"bundle_id": bundle_id, "candidate_id": candidate["candidate_id"],
                   "candidate_hash": digest(candidate), "context": project["context"], "decision": "BLOCKED",
                   "checks": [{"gate": "static-scoreboard", "status": "COMPLETED",
                               "tiers": {"T1": {"status": "PASS"}, "T2": {"status": "PASS"}}},
                              {"gate": "validation-byte-binding", "status": "PASS",
                               "validator_hash": fingerprints["validator_hash"], "files": files}]}
        inputs = {str(base / a["relative_path"]): a["sha256"] for a in candidate["artifacts"]}
        baseline = Path(payload["baseline_path"])
        inputs[str(baseline)] = hashlib.sha256(baseline.read_bytes()).hexdigest()
        report = {"bundle_id": bundle_id, "project_id": project["project_id"],
                  "target_revision": project["revision"], "target_draft_epoch": project["draft_epoch"],
                  "target_content_hash": project["content_hash"], "candidate_hash": digest(candidate),
                  "candidate_sha256": next(a["sha256"] for a in candidate["artifacts"]
                                           if a["relative_path"].endswith(".xodr")),
                  "context": project["context"], "fingerprints": fingerprints,
                  "input_files_sha256": inputs}
        result = {"status": "VALIDATED", "validation": package, "report": report, "artifacts": files}
        (output / "validation-result.json").write_bytes(canonical_bytes(result))
        self.jobs[job_id] = {"job_id": job_id, "project_id": project["project_id"], "operation": operation,
                             "base_revision": project["revision"], "content_hash": project["content_hash"],
                             "state": "succeeded", "historical": False,
                             "archive": {"state": "persisted", "recorded_state": "succeeded"}, "result": result}
        return copy.deepcopy(self.jobs[job_id])

    def get(self, job_id, *, project_id, **kwargs):
        record = self.jobs[job_id]
        if record["project_id"] != project_id:
            raise KeyError("Task does not belong to this project")
        return copy.deepcopy(record)


@pytest.fixture
def checked(setup, monkeypatch):
    e = setup
    _, compiled_id = confirmed(e)
    e.accepted = e.service.accept(e.pid, 2, "accept", compiled_id)
    fingerprints = {"validator_hash": "d" * 64, "implementation_files_sha256": {"validator.py": "d" * 64},
                    "compiler_policy": {"compiler_hash": e.fingerprints["compiler"],
                                        "policy_hash": e.fingerprints["policy"]}}
    monkeypatch.setattr(validation, "validation_fingerprints", lambda: copy.deepcopy(fingerprints))
    monkeypatch.setattr(checking, "verify_source_snapshot", lambda *a: {"matches": True, "issues": []})
    e.check_fingerprints = fingerprints
    e.check_jobs = CheckedJobs(fingerprints)
    e.checker = CheckingService(e.store, e.check_jobs, source_dir=e.service.source_dir,
                                profile_path=e.service.profile_path, baseline_path=e.baseline)
    return e


def started(e, request="check"):
    return e.checker.start(e.pid, 3, request)["job"]["job_id"]


def rewrite_result(e, job_id):
    result = e.check_jobs.jobs[job_id]["result"]
    path = e.store.project_path(e.pid) / "artifacts" / result["validation"]["bundle_id"] / "validation-result.json"
    path.write_bytes(canonical_bytes(result))


def test_start_and_completion_do_not_attach_then_explicit_attach_is_persistent(checked):
    e = checked
    before = (e.store.project_path(e.pid) / "project.json").read_bytes()
    job_id = started(e)
    assert (e.store.project_path(e.pid) / "project.json").read_bytes() == before
    assert e.checker.start(e.pid, 3, "check")["job"]["job_id"] == job_id
    payload = e.check_jobs.payloads[job_id]
    assert payload["source_dir"] == str(e.checker.source_dir)
    assert payload["profile_path"] == str(e.checker.profile_path)
    assert payload["project"]["candidate"] == e.accepted["candidate"]
    saved = e.checker.attach(e.pid, 3, "attach", job_id)
    assert saved["revision"] == 4 and saved["draft_epoch"] == e.accepted["draft_epoch"]
    assert saved["validation"]["decision"] == "BLOCKED"
    assert saved["candidate"] == e.accepted["candidate"]
    assert not saved["status"]["validation_stale"]
    assert not saved["status"]["formal_export_available"]
    assert e.store.load(e.pid)["validation"] == saved["validation"]
    assert e.checker.attach(e.pid, 3, "attach", job_id) == saved
    with pytest.raises(StoreConflict):
        e.checker.attach(e.pid, 4, "different-attach", job_id)


def test_only_current_accepted_candidate_can_start(setup, monkeypatch):
    e = setup
    monkeypatch.setattr(validation, "validation_fingerprints", lambda: {"compiler_policy": {}})
    checker = CheckingService(e.store, e.jobs)
    with pytest.raises(StoreConflict, match="已接受"):
        checker.start(e.pid, 0, "no-candidate")
    assert e.jobs.jobs == {}


@pytest.mark.parametrize("change", ["annotation", "undo_redo", "replacement", "other_validation"])
def test_late_check_cannot_attach_after_other_transactions(checked, change):
    e = checked
    job_id = started(e)
    if change == "annotation":
        current = e.store.commit(e.pid, 3, {"command_id": "note", "type": "annotation", "source_ref": "boundary",
                                          "scope": {"feature_ids": ["boundary"]}, "parameters": {"text": "new"}})
    elif change == "undo_redo":
        e.store.undo(e.pid, 3, "undo")
        current = e.store.redo(e.pid, 4, "redo")
        assert current["content_hash"] == e.accepted["content_hash"]
    elif change == "replacement":
        candidate = {k: v for k, v in e.accepted["candidate"].items()
                     if k not in {"accepted_epoch", "accepted_revision"}}
        current = e.store.accept_candidate(e.pid, 3, "new-candidate-accept", candidate)
    else:
        another_id = started(e, "another-check")
        current = e.checker.attach(e.pid, 3, "another-attach", another_id)
    with pytest.raises(StoreConflict):
        e.checker.attach(e.pid, current["revision"], "late-attach", job_id)


@pytest.mark.parametrize("state", ["running", "cancelled", "failed", "timed_out"])
def test_cancelled_or_unfinished_validation_does_not_attach_buffered_success(checked, state):
    e = checked
    job_id = started(e)
    e.check_jobs.jobs[job_id]["state"] = state
    with pytest.raises(StoreConflict):
        e.checker.attach(e.pid, 3, "attach", job_id)
    assert e.store.load(e.pid)["validation"] is None


@pytest.mark.parametrize("change", ["historical", "archive_failed", "archive_disabled", "old_archive", "rejected"])
def test_historical_unpersisted_and_rejected_results_are_not_attachable(checked, change):
    e = checked
    job_id = started(e)
    job = e.check_jobs.jobs[job_id]
    if change == "historical":
        job["historical"] = True
    elif change.startswith("archive_"):
        job["archive"]["state"] = change.split("_")[1]
    elif change == "old_archive":
        job["archive"]["recorded_state"] = "running"
    else:
        job["result"] = {"status": "REJECTED", "validation": None}
    with pytest.raises((StoreConflict, StoreValidation)):
        e.checker.attach(e.pid, 3, "attach", job_id)


@pytest.mark.parametrize("file_kind", ["candidate", "output", "result", "baseline"])
def test_attach_rechecks_actual_input_and_output_bytes(checked, file_kind):
    e = checked
    job_id = started(e)
    result = e.check_jobs.jobs[job_id]["result"]
    base = e.store.project_path(e.pid)
    if file_kind == "candidate":
        path = base / e.accepted["candidate"]["artifacts"][0]["relative_path"]
    elif file_kind == "output":
        path = base / result["artifacts"][0]["relative_path"]
    elif file_kind == "result":
        path = base / "artifacts" / result["validation"]["bundle_id"] / "validation-result.json"
    else:
        path = e.baseline
    path.write_bytes(path.read_bytes() + b"changed")
    with pytest.raises(StoreValidation):
        e.checker.attach(e.pid, 3, "attach", job_id)
    assert e.store.load(e.pid)["validation"] is None


def test_source_bytes_are_rechecked_after_validation(checked, monkeypatch):
    e = checked
    job_id = started(e)
    monkeypatch.setattr(checking, "verify_source_snapshot", lambda *a: {"matches": False})
    with pytest.raises(StoreReadOnly, match="实际字节"):
        e.checker.attach(e.pid, 3, "attach", job_id)


@pytest.mark.parametrize("change", ["validator", "compiler", "policy"])
def test_changed_validator_compiler_or_policy_invalidates_result(checked, change):
    e = checked
    job_id = started(e)
    if change == "validator":
        e.check_fingerprints["validator_hash"] = "f" * 64
    else:
        e.check_fingerprints["compiler_policy"][change + "_hash"] = "f" * 64
    with pytest.raises(StoreConflict):
        e.checker.attach(e.pid, 3, "attach", job_id)


@pytest.mark.parametrize("field", ["target_revision", "target_draft_epoch", "target_content_hash", "candidate_sha256"])
def test_persisted_report_cannot_change_server_registered_target(checked, field):
    e = checked
    job_id = started(e)
    report = e.check_jobs.jobs[job_id]["result"]["report"]
    report[field] = report[field] + 1 if isinstance(report[field], int) else "f" * 64
    rewrite_result(e, job_id)
    with pytest.raises(StoreConflict, match="目标版本"):
        e.checker.attach(e.pid, 3, "attach", job_id)


def test_tier_passes_never_open_delivery(checked):
    e = checked
    job_id = started(e)
    result = e.check_jobs.jobs[job_id]["result"]
    result["validation"]["decision"] = "DELIVERABLE"
    rewrite_result(e, job_id)
    with pytest.raises(StoreValidation, match="不能开放正式交付"):
        e.checker.attach(e.pid, 3, "attach", job_id)
    assert e.store.load(e.pid)["validation"] is None


def test_new_service_does_not_trust_previous_session_job(checked):
    e = checked
    job_id = started(e)
    restarted = CheckingService(e.store, e.check_jobs, baseline_path=e.baseline)
    with pytest.raises(StoreConflict, match="历史会话"):
        restarted.attach(e.pid, 3, "attach", job_id)


@pytest.mark.parametrize("bad_id", [None, {}, [], True, "../candidate"])
def test_invalid_command_and_job_ids_are_contract_errors(checked, bad_id):
    e = checked
    job_id = started(e)
    with pytest.raises(StoreValidation):
        e.checker.attach(e.pid, 3, bad_id, job_id)
    with pytest.raises(StoreValidation):
        e.checker.attach(e.pid, 3, "attach", bad_id)


def test_invalid_revision_readonly_and_changed_candidate_prevent_start(checked):
    e = checked
    with pytest.raises(StoreValidation):
        e.checker.start(e.pid, True, "bad")
    with pytest.raises(StoreConflict):
        e.checker.start(e.pid, 2, "old")
    e.store.source_validator = lambda snap: [{"code": "drift"}]
    with pytest.raises(StoreReadOnly):
        e.checker.start(e.pid, 3, "readonly")
    e.store.source_validator = None
    path = e.store.project_path(e.pid) / e.accepted["candidate"]["artifacts"][0]["relative_path"]
    path.write_bytes(b"different")
    with pytest.raises(StoreValidation):
        e.checker.start(e.pid, 3, "corrupt")
    assert e.check_jobs.jobs == {}
