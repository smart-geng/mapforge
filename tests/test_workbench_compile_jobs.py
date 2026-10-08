"""Real spawned compiler jobs: business rejection, ownership and persisted results."""
import copy
import hashlib
import json
import os
from pathlib import Path
import time

import pytest

import mapforge.workbench.jobs as jobs_module
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from test_workbench_sources import source


@pytest.fixture
def project(source, tmp_path):
    catalog = SourceCatalog(*source)
    store = ProjectStore(tmp_path / "projects")
    return store, store.create(catalog.snapshot("j1"), "compiler ownership fixture")


def _terminal(manager, job, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        observed = manager.get(job["job_id"], project_id=job["project_id"])
        if observed["state"] != "running":
            return observed
        time.sleep(.01)
    pytest.fail("spawned compiler did not become terminal")


def _wait_marker(path, timeout=15):
    deadline = time.monotonic() + timeout
    while not path.exists():
        assert time.monotonic() < deadline, "test worker did not reach the ownership checkpoint"
        time.sleep(.01)


def _buffered_compile_worker(connection, operation, payload):
    """Only exercises the cancellation race; this is not a geometry result."""
    assert operation == "compile"
    connection.send({"state": "succeeded", "result": {
        "status": "REJECTED", "candidate": None, "accepted": False,
        "error": {"code": "test-checkpoint", "message": "race fixture"}, "delivery": "BLOCKED"}})
    Path(payload["marker"]).write_text("result buffered", encoding="utf-8")
    while True:
        time.sleep(.05)


@pytest.mark.parametrize("payload", [{}, {"draft": {}}])
def test_invalid_draft_is_real_spawn_business_rejection_not_worker_crash(project, payload):
    store, original = project
    manager = JobManager(timeout_s=15)
    try:
        job = manager.start(original, "compile", payload, "invalid-draft")
        assert job["worker_pid"] != os.getpid()
        result = _terminal(manager, job)
        assert result["state"] == "succeeded"
        assert "error" not in result
        assert result["result"]["status"] == "REJECTED"
        assert result["result"]["error"]["code"] == "compile-unavailable"
        assert result["result"]["candidate"] is None
        assert result["result"]["accepted"] is False
        assert result["result"]["delivery"] == "BLOCKED"
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
        assert store.load(original["project_id"]) == original
    finally:
        manager.close()


def test_compile_rejection_archive_reopens_as_read_only_history(project, tmp_path):
    store, original = project
    archive = tmp_path / "jobs"
    manager = JobManager(timeout_s=15, archive_dir=archive)
    payload = {"draft": {}}
    request_id = "do-not-persist-this-raw-request-id"
    try:
        job = manager.start(original, "compile", payload, request_id)
        result = _terminal(manager, job)
        assert result["archive"] == {"state": "persisted", "recorded_state": "succeeded"}
        path = archive / (job["job_id"] + ".json")
        record = json.loads(path.read_text(encoding="utf-8"))["payload"]
        assert record["operation"] == "compile"
        assert record["result"]["status"] == "REJECTED"
        assert record["result"]["candidate"] is None
        assert [event["state"] for event in record["events"]] == ["starting", "running", "succeeded"]
        assert request_id not in path.read_text(encoding="utf-8")
        archived_bytes = path.read_bytes()
        restarted = JobManager(archive_dir=archive)
        try:
            historical = restarted.get(job["job_id"], project_id=original["project_id"],
                                       revision=original["revision"], content_hash=original["content_hash"])
            assert historical["state"] == "succeeded"
            assert historical["historical"] and historical["stale"]
            assert historical["execution_status_known"]
            assert historical["result"] == result["result"]
            again = restarted.start(original, "compile", payload, request_id)
            assert again["job_id"] == job["job_id"] and again["historical"]
            assert "_process" not in restarted._jobs[job["job_id"]]
            with pytest.raises(ValueError, match="read-only"):
                restarted.cancel(job["job_id"], project_id=original["project_id"])
            assert store.load(original["project_id"]) == original
        finally:
            restarted.close()
        assert path.read_bytes() == archived_bytes
    finally:
        manager.close()


def test_real_spawn_validation_rejects_project_without_accepted_candidate_and_archives_history(project, tmp_path):
    store, original = project
    directory = store.project_path(original["project_id"])
    project_bytes = (directory / "project.json").read_bytes()
    payload = {"project": original, "project_directory": str(directory)}
    archive = tmp_path / "validation-jobs"
    manager = JobManager(timeout_s=15, archive_dir=archive)
    try:
        job = manager.start(original, "validate", payload, "no-accepted-candidate")
        assert job["worker_pid"] != os.getpid()
        result = _terminal(manager, job)
        assert result["state"] == "succeeded" and "error" not in result
        assert result["operation"] == "validate"
        assert result["result"]["status"] == "REJECTED"
        assert result["result"]["error"]["code"] == "candidate-stale"
        assert result["result"]["validation"] is None
        assert result["result"]["accepted"] is False
        assert result["archive"] == {"state": "persisted", "recorded_state": "succeeded"}
        assert not (directory / "artifacts").exists()
        assert (directory / "project.json").read_bytes() == project_bytes
        assert store.load(original["project_id"]) == original
        restarted = JobManager(archive_dir=archive)
        try:
            history = restarted.start(original, "validate", payload, "no-accepted-candidate")
            assert history["job_id"] == job["job_id"]
            assert history["historical"] and history["stale"]
            assert history["execution_status_known"]
            assert history["result"] == result["result"]
            assert "_process" not in restarted._jobs[job["job_id"]]
            with pytest.raises(ValueError, match="read-only"):
                restarted.cancel(job["job_id"], project_id=original["project_id"])
            assert (directory / "project.json").read_bytes() == project_bytes
        finally:
            restarted.close()
    finally:
        manager.close()


def test_compile_result_is_stale_after_draft_edit_and_cannot_mutate_project(project):
    store, original = project
    manager = JobManager(timeout_s=15)
    try:
        job = manager.start(original, "compile", {"draft": {}}, "old-draft")
        ref = original["source_snapshot"]["objects"][0]["id"]
        current = store.commit(original["project_id"], original["revision"], {
            "command_id": "newer-note", "type": "annotation", "source_ref": ref,
            "scope": {"feature_ids": [ref]}, "parameters": {"text": "keep this later draft"}})
        _terminal(manager, job)
        observed = manager.get(job["job_id"], project_id=current["project_id"],
                               revision=current["revision"], content_hash=current["content_hash"])
        assert observed["stale"]
        assert observed["base_revision"] == original["revision"]
        assert observed["content_hash"] == original["content_hash"]
        assert store.load(current["project_id"]) == current
        with pytest.raises(KeyError):
            manager.get(job["job_id"], project_id="another-project")
        changed_payload = {"draft": {"revision": 2}}
        with pytest.raises(ValueError, match="different input"):
            manager.start(original, "compile", changed_payload, "old-draft")
    finally:
        manager.close()


def test_compile_cancel_wins_over_buffered_result_and_survives_restart(project, tmp_path, monkeypatch):
    store, original = project
    monkeypatch.setattr(jobs_module, "_worker", _buffered_compile_worker)
    archive = tmp_path / "jobs"
    manager = JobManager(timeout_s=15, archive_dir=archive)
    marker = tmp_path / "buffered"
    payload = {"marker": str(marker)}
    try:
        job = manager.start(original, "compile", payload, "cancel-buffered")
        _wait_marker(marker)
        assert manager._jobs[job["job_id"]]["_pipe"].poll()
        cancelled = manager.cancel(job["job_id"], project_id=original["project_id"])
        assert cancelled["state"] == "cancelled" and "result" not in cancelled
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
        assert cancelled["archive"]["recorded_state"] == "cancelled"
        restarted = JobManager(archive_dir=archive)
        try:
            history = restarted.start(original, "compile", payload, "cancel-buffered")
            assert history["job_id"] == job["job_id"]
            assert history["state"] == "cancelled" and history["historical"]
            assert "result" not in history and "_process" not in restarted._jobs[job["job_id"]]
        finally:
            restarted.close()
        assert store.load(original["project_id"]) == original
    finally:
        manager.close()


def test_live_compile_request_cannot_be_restarted_or_cancelled_by_another_manager(project, tmp_path, monkeypatch):
    _, original = project
    monkeypatch.setattr(jobs_module, "_worker", _buffered_compile_worker)
    archive = tmp_path / "jobs"
    owner = JobManager(timeout_s=15, archive_dir=archive)
    other = JobManager(timeout_s=15, archive_dir=archive)
    marker = tmp_path / "started"
    payload = {"marker": str(marker)}
    try:
        job = owner.start(original, "compile", payload, "live-compile")
        _wait_marker(marker)
        process = owner._jobs[job["job_id"]]["_process"]
        history = other.start(original, "compile", payload, "live-compile")
        assert history["job_id"] == job["job_id"]
        assert history["state"] == "orphaned" and history["recorded_state"] == "running"
        assert not history["execution_status_known"]
        assert history["historical"] and history["stale"]
        with pytest.raises(ValueError, match="read-only"):
            other.cancel(job["job_id"], project_id=original["project_id"])
        other.close()
        assert process.is_alive()
    finally:
        owner.close()
        other.close()


def test_real_node16_spawn_publishes_exact_candidate_without_accepting(tmp_path):
    from mapforge.workbench.compiler import (BASELINE_SHA256, CAPABILITY_ID, DEFAULT_BASELINE,
        INTENT_TYPE, JUNCTION_ID, ROOT, register_verified_baseline)
    from mapforge.workbench.sources import build_source_snapshot

    if not DEFAULT_BASELINE.is_file() or not (ROOT / "shp_0222-0326").is_dir():
        pytest.skip("Real node16 integration requires the registered source and baseline")
    snapshot = build_source_snapshot(ROOT / "shp_0222-0326", ROOT / "profiles/shp/ibd-smarteditor-v1.yaml", JUNCTION_ID)
    registration = register_verified_baseline(snapshot)
    store = ProjectStore(tmp_path / "projects")
    draft = store.create(snapshot, "real node16 spawned compiler")
    context = registration.context(snapshot)
    spec = registration.store_spec()
    draft = store.register_capabilities(draft["project_id"], draft["revision"], "register-real-scope",
                                       [spec], context["compiler_hash"], context["policy_hash"])
    draft = store.commit(draft["project_id"], draft["revision"], {
        "command_id": "minus-15mm", "type": INTENT_TYPE, "source_ref": spec["source_ref"],
        "scope": {"capability_id": CAPABILITY_ID, "feature_ids": spec["feature_ids"]},
        "parameters": {"normal_delta_m": -.015, "baseline_sha256": BASELINE_SHA256}})
    unchanged = copy.deepcopy(draft)
    directory = store.project_path(draft["project_id"])
    project_bytes = (directory / "project.json").read_bytes()
    payload = {"draft": draft, "project_directory": str(directory)}
    archive = tmp_path / "jobs"
    manager = JobManager(timeout_s=120, archive_dir=archive)
    try:
        job = manager.start(draft, "compile", payload, "real-minus-15mm")
        result = _terminal(manager, job, timeout=120)
        assert result["state"] == "succeeded", result
        compiled = result["result"]
        assert compiled["status"] == "COMPILED", compiled
        assert compiled["accepted"] is False and compiled["delivery"] == "BLOCKED"
        candidate = compiled["candidate"]
        assert candidate["target_content_hash"] == draft["content_hash"]
        assert candidate["context"] == draft["context"]
        assert compiled["evidence"]["target_revision"] == draft["revision"]
        assert compiled["evidence"]["target_draft_epoch"] == draft["draft_epoch"]
        xodr = next(item for item in candidate["artifacts"] if item["relative_path"].endswith(".xodr"))
        assert xodr["sha256"] == "78b893993494b46ee5aa1b094f1fd2ad9f40efdad2d2c6c7943b515a5ea6cd86"
        for artifact in candidate["artifacts"]:
            data = (directory / artifact["relative_path"]).read_bytes()
            assert hashlib.sha256(data).hexdigest() == artifact["sha256"]
        assert store.load(draft["project_id"]) == unchanged
        assert (directory / "project.json").read_bytes() == project_bytes
        assert store.load(draft["project_id"])["candidate"] is None
        assert result["archive"]["state"] == "persisted"
        restarted = JobManager(archive_dir=archive)
        try:
            history = restarted.get(job["job_id"], project_id=draft["project_id"])
            assert history["result"] == compiled
            assert history["historical"] and history["stale"]
            assert store.load(draft["project_id"])["candidate"] is None
        finally:
            restarted.close()
    finally:
        manager.close()
