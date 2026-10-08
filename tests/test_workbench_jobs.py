"""Real spawned source checks plus bounded process/race tests; no geometry claims."""
import copy
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
def source_project(source, tmp_path):
    catalog = SourceCatalog(*source)
    store = ProjectStore(tmp_path / "projects")
    project = store.create(catalog.snapshot("j1"), "source fixture")
    payload = {"snapshot": project["source_snapshot"], "source_dir": str(catalog.source_dir),
               "profile_path": str(catalog.profile_path)}
    return catalog, store, project, payload


def terminal(manager, job, *, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = manager.get(job["job_id"], project_id=job["project_id"])
        if state["state"] != "running":
            return state
        time.sleep(.01)
    raise AssertionError("owned background process did not reach terminal state")


def blocking_worker(connection, operation, payload):
    """Test-only worker for process ownership, cancellation and timeout, not results."""
    Path(payload["started_path"]).write_text("started", encoding="utf-8")
    while True:
        time.sleep(.05)


def crash_worker(connection, operation, payload):
    os._exit(19)


def malformed_worker(connection, operation, payload):
    connection.send({"state": "running", "project_id": "wrong-project"})
    connection.close()


def buffered_result_worker(connection, operation, payload):
    connection.send({"state": "succeeded", "result": {"matches": True, "issues": []}})
    Path(payload["started_path"]).write_text("result buffered", encoding="utf-8")
    while True:
        time.sleep(.05)


def wait_started(path, timeout=10):
    deadline = time.monotonic() + timeout
    while not path.exists():
        assert time.monotonic() < deadline, "spawned worker did not start"
        time.sleep(.01)


def test_real_spawn_source_check_success_and_metadata(source_project):
    _, store, project, payload = source_project
    manager = JobManager(timeout_s=10)
    try:
        job = manager.start(project, "source_check", payload, "source-check-1")
        assert manager._jobs[job["job_id"]]["_process"].pid != os.getpid()
        assert job["base_revision"] == project["revision"]
        assert job["content_hash"] == project["content_hash"]
        assert len(job["input_hash"]) == 64
        result = terminal(manager, job)
        assert result["state"] == "succeeded"
        assert result["result"] == {"matches": True, "issues": []}
        assert store.load(project["project_id"])["revision"] == project["revision"]
        assert store.load(project["project_id"])["candidate"] is None
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
    finally:
        manager.close()


def test_real_spawn_source_mismatch_is_a_failed_check_not_worker_failure(source_project):
    catalog, _, project, payload = source_project
    (catalog.source_dir / "vendor-extra.bin").write_bytes(b"changed")
    manager = JobManager(timeout_s=10)
    try:
        result = terminal(manager, manager.start(project, "source_check", payload, "changed"))
        assert result["state"] == "succeeded"  # the check process completed
        assert result["result"]["matches"] is False  # its quality outcome did not pass
        assert result["result"]["issues"][0]["code"] == "source-file-changed"
    finally:
        manager.close()


def test_real_spawn_worker_exception_becomes_terminal_failed(source_project):
    _, _, project, _ = source_project
    manager = JobManager(timeout_s=10)
    try:
        result = terminal(manager, manager.start(project, "source_check", {}, "missing-payload"))
        assert result["state"] == "failed"
        assert "KeyError" in result["error"]
    finally:
        manager.close()


def test_request_dedup_binds_payload_and_revision_and_does_not_restart(source_project):
    _, _, project, payload = source_project
    manager = JobManager(timeout_s=10)
    try:
        first = manager.start(project, "source_check", payload, "request-1")
        result = terminal(manager, first)
        again = manager.start(project, "source_check", payload, "request-1")
        assert again["job_id"] == first["job_id"]
        assert again["state"] == result["state"]
        changed_payload = copy.deepcopy(payload)
        changed_payload["source_dir"] = payload["source_dir"] + "-changed"
        with pytest.raises(ValueError, match="different input"):
            manager.start(project, "source_check", changed_payload, "request-1")
        changed_project = {**project, "revision": project["revision"] + 1}
        with pytest.raises(ValueError, match="different input"):
            manager.start(changed_project, "source_check", payload, "request-1")
    finally:
        manager.close()


def test_late_result_is_stale_and_never_commits_to_changed_project(source_project):
    _, store, project, payload = source_project
    manager = JobManager(timeout_s=10)
    try:
        job = manager.start(project, "source_check", payload, "old-input")
        obj_id = project["source_snapshot"]["objects"][0]["id"]
        changed = store.commit(project["project_id"], 0, {
            "command_id": "note", "type": "annotation", "source_ref": obj_id,
            "scope": {"feature_ids": [obj_id]}, "parameters": {"text": "new draft"}})
        terminal(manager, job)
        observed = manager.get(job["job_id"], project_id=project["project_id"],
                               revision=changed["revision"], content_hash=changed["content_hash"])
        assert observed["stale"] is True
        assert store.load(project["project_id"]) == changed
        with pytest.raises(KeyError):
            manager.get(job["job_id"], project_id="another-project")
        with pytest.raises(KeyError):
            manager.cancel(job["job_id"], project_id="another-project")
    finally:
        manager.close()


def test_actual_process_cancel_is_terminal_and_frees_capacity(source_project, tmp_path, monkeypatch):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", blocking_worker)
    manager = JobManager(max_running=1, timeout_s=10)
    payload = {"started_path": str(tmp_path / "started")}
    try:
        job = manager.start(project, "source_check", payload, "block-1")
        wait_started(Path(payload["started_path"]))
        with pytest.raises(ValueError, match="busy"):
            manager.start(project, "source_check", payload, "over-limit")
        cancelled = manager.cancel(job["job_id"], project_id=project["project_id"])
        assert cancelled["state"] == "cancelled"
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
        assert manager.get(job["job_id"], project_id=project["project_id"])["state"] == "cancelled"
        duplicate = manager.start(project, "source_check", payload, "block-1")
        assert duplicate["job_id"] == job["job_id"] and duplicate["state"] == "cancelled"
        next_job = manager.start(project, "source_check", payload, "block-2")
        assert next_job["job_id"] != job["job_id"]
        manager.close()
        assert manager.get(next_job["job_id"], project_id=project["project_id"])["state"] == "cancelled"
        with pytest.raises(ValueError, match="closed"):
            manager.start(project, "source_check", payload, "closed-manager")
    finally:
        manager.close()


def test_actual_process_timeout_reaps_owned_worker(source_project, tmp_path, monkeypatch):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", blocking_worker)
    manager = JobManager(timeout_s=.2)
    try:
        job = manager.start(project, "source_check", {"started_path": str(tmp_path / "started")}, "timeout")
        result = terminal(manager, job)
        assert result["state"] == "timed_out"
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
    finally:
        manager.close()


def test_cancel_wins_over_result_already_buffered_in_owned_worker_pipe(source_project, tmp_path, monkeypatch):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", buffered_result_worker)
    manager = JobManager(timeout_s=10)
    marker = tmp_path / "result-buffered"
    try:
        job = manager.start(project, "source_check", {"started_path": str(marker)}, "cancel-late")
        wait_started(marker)
        assert manager._jobs[job["job_id"]]["_pipe"].poll()
        result = manager.cancel(job["job_id"], project_id=project["project_id"])
        assert result["state"] == "cancelled"
        observed = manager.get(job["job_id"], project_id=project["project_id"])
        assert observed["state"] == "cancelled" and "result" not in observed
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
    finally:
        manager.close()


@pytest.mark.parametrize("worker", [crash_worker, malformed_worker])
def test_actual_worker_crash_or_invalid_result_is_failed(source_project, monkeypatch, worker):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", worker)
    manager = JobManager(timeout_s=10)
    try:
        job = manager.start(project, "source_check", {}, "bad-worker")
        result = terminal(manager, job)
        assert result["state"] == "failed"
        assert result["project_id"] == project["project_id"]
        assert result["error"]
        assert not manager._jobs[job["job_id"]]["_process"].is_alive()
    finally:
        manager.close()


@pytest.mark.parametrize("options", [{"max_running": 0}, {"max_running": True},
                                    {"timeout_s": 0}, {"timeout_s": float("nan")},
                                    {"timeout_s": float("inf")}])
def test_invalid_worker_limits_rejected(options):
    with pytest.raises(ValueError):
        JobManager(**options)


def test_real_source_result_archived_without_source_payload_or_request_secret(source_project, tmp_path):
    _, store, project, payload = source_project
    archive = tmp_path / "job-archive"
    manager = JobManager(timeout_s=10, archive_dir=archive)
    request_secret = "a-request-id-that-must-not-be-written-raw"
    try:
        job = manager.start(project, "source_check", payload, request_secret)
        result = terminal(manager, job)
        assert result["archive"] == {"state": "persisted", "recorded_state": "succeeded"}
        record = json.loads((archive / (job["job_id"] + ".json")).read_text(encoding="utf-8"))["payload"]
        assert record["project_id"] == project["project_id"]
        assert record["base_revision"] == project["revision"]
        assert record["content_hash"] == project["content_hash"]
        assert record["input_hash"] == result["input_hash"]
        assert record["result"] == {"matches": True, "issues": []}
        assert [event["state"] for event in record["events"]] == ["starting", "running", "succeeded"]
        for path in archive.glob("*.json"):
            text = path.read_text(encoding="utf-8")
            assert request_secret not in text
            assert "source_snapshot" not in text and '"raw_attributes"' not in text
            assert str(payload["source_dir"]).replace("\\", "\\\\") not in text
        restarted = JobManager(archive_dir=archive)
        try:
            history = restarted.get(job["job_id"], project_id=project["project_id"],
                                    revision=project["revision"], content_hash=project["content_hash"])
            assert history["state"] == "succeeded"
            assert history["historical"] and history["stale"]
            assert history["execution_status_known"]
            again = restarted.start(project, "source_check", payload, request_secret)
            assert again["job_id"] == job["job_id"] and again["historical"]
            with pytest.raises(ValueError, match="read-only"):
                restarted.cancel(job["job_id"], project_id=project["project_id"])
            assert store.load(project["project_id"]) == project
        finally:
            restarted.close()
    finally:
        manager.close()


def test_restart_running_record_is_unconfirmed_and_never_takes_ownership(source_project, tmp_path, monkeypatch):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", blocking_worker)
    archive = tmp_path / "archive"
    owner = JobManager(timeout_s=10, archive_dir=archive)
    # Both services start before the request exists: the exclusive request claim
    # must still prevent a duplicate process, not just an in-memory dictionary.
    other = JobManager(timeout_s=10, archive_dir=archive)
    payload = {"started_path": str(tmp_path / "started")}
    try:
        job = owner.start(project, "source_check", payload, "live-request")
        wait_started(Path(payload["started_path"]))
        process = owner._jobs[job["job_id"]]["_process"]
        history = other.start(project, "source_check", payload, "live-request")
        assert history["job_id"] == job["job_id"]
        assert history["state"] == "orphaned" and history["recorded_state"] == "running"
        assert history["historical"] and not history["execution_status_known"]
        assert "_process" not in other._jobs[history["job_id"]]
        with pytest.raises(ValueError, match="read-only"):
            other.cancel(job["job_id"], project_id=project["project_id"])
        other.close()
        assert process.is_alive()  # close did not kill the process another manager owns
        cancelled = owner.cancel(job["job_id"], project_id=project["project_id"])
        assert cancelled["archive"]["recorded_state"] == "cancelled"
        recorded = json.loads((archive / (job["job_id"] + ".json")).read_text(encoding="utf-8"))["payload"]
        assert [event["state"] for event in recorded["events"]][-2:] == ["cancelling", "cancelled"]
        reopened = JobManager(archive_dir=archive)
        try:
            assert reopened.get(job["job_id"], project_id=project["project_id"])["state"] == "cancelled"
        finally:
            reopened.close()
    finally:
        owner.close()
        other.close()


def test_terminal_archive_failure_keeps_atomic_running_record_and_is_explicit(source_project, tmp_path, monkeypatch):
    _, _, project, payload = source_project
    archive = tmp_path / "archive"
    manager = JobManager(timeout_s=10, archive_dir=archive)
    original_replace = os.replace

    def reject_terminal(source, target):
        if Path(target).parent == archive:
            doc = json.loads(Path(source).read_text(encoding="utf-8"))
            if doc["payload"]["state"] == "succeeded":
                raise OSError("injected terminal archive disk full")
        return original_replace(source, target)

    monkeypatch.setattr(os, "replace", reject_terminal)
    try:
        job = manager.start(project, "source_check", payload, "archive-failure")
        result = terminal(manager, job)
        assert result["state"] == "succeeded"  # computation outcome is separate
        assert result["archive"]["state"] == "failed"
        assert result["archive"]["recorded_state"] == "running"
        record = json.loads((archive / (job["job_id"] + ".json")).read_text(encoding="utf-8"))
        assert record["payload"]["state"] == "running"
        assert not list(archive.glob("*.tmp"))
        reopened = JobManager(archive_dir=archive)
        try:
            historical = reopened.get(job["job_id"], project_id=project["project_id"])
            assert historical["state"] == "orphaned" and not historical["execution_status_known"]
        finally:
            reopened.close()
        monkeypatch.setattr(os, "replace", original_replace)
        retry = manager.retry_archive(job["job_id"], project_id=project["project_id"])
        assert retry["archive"]["state"] == "persisted"
        assert retry["archive"]["recorded_state"] == "succeeded"
    finally:
        manager.close()


def test_starting_archive_failure_does_not_launch_or_automatically_repeat(source_project, tmp_path, monkeypatch):
    _, _, project, payload = source_project
    archive = tmp_path / "archive"
    manager = JobManager(timeout_s=10, archive_dir=archive)
    original_replace = os.replace

    def fail_archive(source, target):
        raise OSError("injected starting archive failure")

    monkeypatch.setattr(os, "replace", fail_archive)
    try:
        job = manager.start(project, "source_check", payload, "not-started")
        assert job["state"] == "failed" and job["archive"]["state"] == "failed"
        assert "_process" not in manager._jobs[job["job_id"]]
        assert "worker_pid" not in job
        monkeypatch.setattr(os, "replace", original_replace)
        reopened = JobManager(archive_dir=archive)
        try:
            with pytest.raises(ValueError, match="cannot be restarted"):
                reopened.start(project, "source_check", payload, "not-started")
            assert reopened._jobs == {}
        finally:
            reopened.close()
        assert manager.retry_archive(job["job_id"], project_id=project["project_id"])["archive"]["state"] == "persisted"
    finally:
        manager.close()


def test_corrupt_archive_is_unavailable_and_claim_blocks_duplicate(source_project, tmp_path):
    _, _, project, payload = source_project
    archive = tmp_path / "archive"
    manager = JobManager(timeout_s=10, archive_dir=archive)
    try:
        job = manager.start(project, "source_check", payload, "corrupt-history")
        terminal(manager, job)
        path = archive / (job["job_id"] + ".json")
        path.write_bytes(b'{"truncated":')
        reopened = JobManager(archive_dir=archive)
        try:
            assert reopened.archive_issues[0]["code"] == "archive-unavailable"
            with pytest.raises(KeyError):
                reopened.get(job["job_id"], project_id=project["project_id"])
            with pytest.raises(ValueError, match="cannot be restarted"):
                reopened.start(project, "source_check", payload, "corrupt-history")
            assert path.read_bytes() == b'{"truncated":'
        finally:
            reopened.close()
    finally:
        manager.close()


def test_timeout_terminal_archive_is_readable_after_restart(source_project, tmp_path, monkeypatch):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", blocking_worker)
    archive = tmp_path / "archive"
    manager = JobManager(timeout_s=.2, archive_dir=archive)
    try:
        job = manager.start(project, "source_check", {"started_path": str(tmp_path / "started")}, "timed-out")
        result = terminal(manager, job)
        assert result["state"] == "timed_out" and result["archive"]["recorded_state"] == "timed_out"
        reopened = JobManager(archive_dir=archive)
        try:
            historical = reopened.get(job["job_id"], project_id=project["project_id"])
            assert historical["state"] == "timed_out" and historical["execution_status_known"]
        finally:
            reopened.close()
    finally:
        manager.close()


def test_list_is_project_scoped_detached_and_uses_current_revision_staleness(source_project, monkeypatch):
    catalog, store, project, payload = source_project
    foreign = store.create(catalog.snapshot("j2"), "another project")
    foreign_payload = {**payload, "snapshot": foreign["source_snapshot"]}
    manager = JobManager(max_running=3, timeout_s=10)
    try:
        first = manager.start(project, "source_check", payload, "project-first")
        second = manager.start(project, "source_check", payload, "project-second")
        other = manager.start(foreign, "source_check", foreign_payload, "foreign")
        for job in (first, second, other):
            terminal(manager, job)
        updates = []
        original_update = manager._update

        def observe_update(job):
            updates.append(job["project_id"])
            return original_update(job)

        monkeypatch.setattr(manager, "_update", observe_update)
        rows = manager.list(project_id=project["project_id"], revision=project["revision"],
                            content_hash=project["content_hash"])
        assert [row["job_id"] for row in rows] == [second["job_id"], first["job_id"]]
        assert updates == [project["project_id"], project["project_id"]]
        assert all(row["project_id"] == project["project_id"] and not row["stale"] for row in rows)
        assert all(not any(key.startswith("_") for key in row) for row in rows)
        rows[0]["result"]["issues"].append({"injected": "caller mutation"})
        assert manager.get(second["job_id"], project_id=project["project_id"])["result"]["issues"] == []
        stale = manager.list(project_id=project["project_id"], revision=project["revision"] + 1,
                             content_hash=project["content_hash"])
        assert all(row["stale"] for row in stale)
        changed_content = manager.list(project_id=project["project_id"], revision=project["revision"],
                                       content_hash="0" * 64)
        assert all(row["stale"] for row in changed_content)
        assert manager.list(project_id="no-such-project") == []
        assert store.load(project["project_id"]) == project
        assert store.load(foreign["project_id"]) == foreign
    finally:
        manager.close()


def test_list_restores_cancelled_and_unconfirmed_history_without_process_ownership(source_project, tmp_path, monkeypatch):
    _, _, project, _ = source_project
    monkeypatch.setattr(jobs_module, "_worker", blocking_worker)
    archive = tmp_path / "list-history"
    owner = JobManager(max_running=2, timeout_s=10, archive_dir=archive)
    try:
        first = owner.start(project, "source_check", {"started_path": str(tmp_path / "first")}, "cancelled")
        second = owner.start(project, "source_check", {"started_path": str(tmp_path / "second")}, "still-running")
        wait_started(tmp_path / "first")
        wait_started(tmp_path / "second")
        owner.cancel(first["job_id"], project_id=project["project_id"])
        live = owner._jobs[second["job_id"]]["_process"]
        original_bytes = {path: path.read_bytes() for path in archive.glob("*.json")}
        restarted = JobManager(archive_dir=archive)
        try:
            rows = restarted.list(project_id=project["project_id"], revision=project["revision"],
                                  content_hash=project["content_hash"])
            by_id = {row["job_id"]: row for row in rows}
            assert set(by_id) == {first["job_id"], second["job_id"]}
            assert by_id[first["job_id"]]["state"] == "cancelled"
            assert by_id[first["job_id"]]["execution_status_known"]
            assert by_id[second["job_id"]]["state"] == "orphaned"
            assert not by_id[second["job_id"]]["execution_status_known"]
            assert all(row["historical"] and row["stale"] for row in rows)
            assert all("_process" not in job for job in restarted._jobs.values())
            with pytest.raises(ValueError, match="read-only"):
                restarted.cancel(second["job_id"], project_id=project["project_id"])
            assert all(path.read_bytes() == data for path, data in original_bytes.items())
            assert live.is_alive()
        finally:
            restarted.close()
        assert live.is_alive()  # Listing/closing history never terminates the other manager's worker.
        current = owner.list(project_id=project["project_id"])
        assert {row["state"] for row in current} == {"cancelled", "running"}
    finally:
        owner.close()
