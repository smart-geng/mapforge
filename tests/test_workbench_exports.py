"""Research export uses real stores, files, ZIP readback and job archives.

The expensive geometry evaluator is the controlled checking fixture. These
tests establish export/transaction binding, never map quality or release.
"""
import copy
import hashlib
import io
import json
from pathlib import Path
import zipfile

import pytest

from test_workbench_checking import checked, setup, started
from mapforge.workbench import exports, validation
from mapforge.workbench.contracts import StoreConflict, StoreReadOnly, StoreValidation, canonical_bytes, digest
from mapforge.workbench.jobs import JobManager


@pytest.fixture
def exportable(checked, monkeypatch):
    e = checked
    e.validation_job_id = started(e)
    e.current = e.checker.attach(e.pid, 3, "attach", e.validation_job_id)
    monkeypatch.setattr(exports, "verify_source_snapshot", lambda *a: {"matches": True, "issues": []})
    e.exporter = exports.ResearchExportService(e.store, source_dir=e.service.source_dir,
                                              profile_path=e.service.profile_path)
    e.archiver = JobManager(archive_dir=e.store.root / ".jobs")
    job = copy.deepcopy(e.check_jobs.jobs[e.validation_job_id])
    job.pop("historical", None)
    job.pop("archive", None)
    job.update(input_hash=digest(e.check_jobs.payloads[e.validation_job_id]),
               _request_key=digest([e.pid, "check"]), started_at=1.0, finished_at=2.0,
               worker_pid=None)
    assert e.archiver._archive(job)
    e.archive_path = e.store.root / ".jobs" / (e.validation_job_id + ".json")
    # This is a real durable archive, usable through the production reader.
    assert e.archiver._read_archive(e.validation_job_id)["result"] == job["result"]
    e.validation_result_path = (e.store.project_path(e.pid) / "artifacts" /
                                e.current["validation"]["bundle_id"] / "validation-result.json")
    yield e
    e.archiver.close()


def confirmed_exports(e):
    folder = e.store.project_path(e.pid) / ".research-exports"
    return sorted(folder.glob("research-*")) if folder.exists() else []


def note(e):
    return e.store.commit(e.pid, e.store.load(e.pid)["revision"],
                          {"command_id": "later-note", "type": "annotation", "source_ref": "boundary",
                           "scope": {"feature_ids": ["boundary"]}, "parameters": {"text": "later"}})


def rewrite_archive(e, mutate):
    envelope = json.loads(e.archive_path.read_text(encoding="utf8"))
    mutate(envelope["payload"])
    e.archive_path.write_bytes(JobManager._envelope(envelope["payload"]))


def test_blocked_research_zip_is_byte_bound_marked_and_does_not_change_project(exportable):
    e = exportable
    original = (e.store.project_path(e.pid) / "project.json").read_bytes()
    receipt = e.exporter.create(e.pid, 4)
    downloaded, data = e.exporter.download(e.pid, receipt["export_id"])
    assert receipt == downloaded
    assert receipt["purpose"] == "RESEARCH_ONLY" and receipt["formal_delivery"] is False
    assert receipt["decision"] == "BLOCKED"
    assert receipt["sha256"] == hashlib.sha256(data).hexdigest()
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        manifest = json.loads(archive.read("research-manifest.json"))
        project = json.loads(archive.read("research-project.json"))
        assert manifest["purpose"] == project["purpose"] == "RESEARCH_ONLY"
        assert manifest["formal_delivery"] is project["formal_delivery"] is False
        assert manifest["validation"] == e.current["validation"]
        assert "不是正式交付包" in archive.read("README-研究用途.txt").decode("utf8")
        for artifact in e.current["candidate"]["artifacts"]:
            actual = (e.store.project_path(e.pid) / artifact["relative_path"]).read_bytes()
            assert archive.read(artifact["relative_path"]) == actual
        for item in manifest["files"]:
            payload = archive.read(item["path"])
            assert hashlib.sha256(payload).hexdigest() == item["sha256"]
            assert len(payload) == item["size_bytes"]
    assert (e.store.project_path(e.pid) / "project.json").read_bytes() == original
    assert not e.store.load(e.pid)["status"]["formal_export_available"]


def test_duplicate_export_is_idempotent_and_leaves_one_complete_directory(exportable):
    e = exportable
    first = e.exporter.create(e.pid, 4)
    second = e.exporter.create(e.pid, 4)
    assert first == second
    assert len(confirmed_exports(e)) == 1
    folder = e.store.project_path(e.pid) / ".research-exports"
    assert not list(folder.glob(".*.tmp"))


@pytest.mark.parametrize("revision", [None, True, -1, "4", 3])
def test_invalid_and_old_revisions_do_not_publish(exportable, revision):
    e = exportable
    with pytest.raises((StoreConflict, StoreValidation)):
        e.exporter.create(e.pid, revision)
    assert confirmed_exports(e) == []


def test_changed_validator_invalidates_export_and_existing_download(exportable):
    e = exportable
    receipt = e.exporter.create(e.pid, 4)
    e.check_fingerprints["validator_hash"] = "f" * 64
    for operation in (lambda: e.exporter.create(e.pid, 4),
                      lambda: e.exporter.download(e.pid, receipt["export_id"])):
        with pytest.raises((StoreValidation, validation.ValidationRejected)):
            operation()


def test_changed_source_disables_export_and_download(exportable, monkeypatch):
    e = exportable
    receipt = e.exporter.create(e.pid, 4)
    monkeypatch.setattr(exports, "verify_source_snapshot", lambda *a: {"matches": False})
    with pytest.raises(StoreReadOnly):
        e.exporter.create(e.pid, 4)
    with pytest.raises(StoreReadOnly):
        e.exporter.download(e.pid, receipt["export_id"])


@pytest.mark.parametrize("artifact", ["candidate", "scoreboard", "baseline"])
def test_actual_artifact_drift_cannot_be_packaged(exportable, artifact):
    e = exportable
    if artifact == "candidate":
        path = e.store.project_path(e.pid) / e.current["candidate"]["artifacts"][0]["relative_path"]
    elif artifact == "baseline":
        path = e.baseline
    else:
        path = e.validation_result_path.with_name("scoreboard.json")
    path.write_bytes(path.read_bytes() + b"changed")
    with pytest.raises((StoreValidation, validation.ValidationRejected)):
        e.exporter.create(e.pid, 4)
    assert confirmed_exports(e) == []


@pytest.mark.parametrize("field", ["target_revision", "target_draft_epoch", "target_content_hash", "candidate_sha256",
                                   "input_files_sha256", "unbound_added_field"])
def test_result_report_tampering_is_rejected_by_job_result_binding(exportable, field):
    e = exportable
    result = json.loads(e.validation_result_path.read_text(encoding="utf8"))
    if field == "input_files_sha256":
        result["report"][field] = {str(e.baseline): hashlib.sha256(e.baseline.read_bytes()).hexdigest()}
    elif field == "unbound_added_field":
        result["report"][field] = "Changed after successful validation"
    else:
        value = result["report"][field]
        result["report"][field] = value + 1 if isinstance(value, int) else "f" * 64
    e.validation_result_path.write_bytes(canonical_bytes(result))
    with pytest.raises((StoreConflict, StoreValidation, validation.ValidationRejected)):
        e.exporter.create(e.pid, 4)
    assert confirmed_exports(e) == []


@pytest.mark.parametrize("problem", ["missing", "checksum", "wrong_schema", "cancelled", "running", "wrong_project",
                                     "wrong_result", "wrong_revision", "malformed_result"])
def test_successful_matching_validation_archive_is_required(exportable, problem):
    e = exportable
    if problem == "missing":
        e.archive_path.unlink()
    elif problem == "checksum":
        envelope = json.loads(e.archive_path.read_text(encoding="utf8"))
        envelope["sha256"] = "0" * 64
        e.archive_path.write_bytes(canonical_bytes(envelope))
    elif problem == "wrong_schema":
        envelope = json.loads(e.archive_path.read_text(encoding="utf8"))
        envelope["schema"] = "unrelated-envelope-v1"
        e.archive_path.write_bytes(canonical_bytes(envelope))
    elif problem in {"cancelled", "running"}:
        rewrite_archive(e, lambda p: p.update(state=problem))
    elif problem == "wrong_project":
        rewrite_archive(e, lambda p: p.update(project_id="f" * 32))
    elif problem == "wrong_revision":
        rewrite_archive(e, lambda p: p.update(base_revision=999))
    elif problem == "malformed_result":
        rewrite_archive(e, lambda p: p.update(result=[]))
    else:
        rewrite_archive(e, lambda p: p["result"]["report"].update(target_revision=999))
    with pytest.raises((StoreConflict, StoreValidation, validation.ValidationRejected)):
        e.exporter.create(e.pid, 4)
    assert confirmed_exports(e) == []


def test_disk_write_failure_leaves_no_completed_or_staging_package(exportable, monkeypatch):
    e = exportable
    def disk_full(_):
        raise OSError("simulated no disk space")
    monkeypatch.setattr(exports.os, "fsync", disk_full)
    with pytest.raises(OSError, match="disk space"):
        e.exporter.create(e.pid, 4)
    assert confirmed_exports(e) == []
    folder = e.store.project_path(e.pid) / ".research-exports"
    assert not list(folder.glob(".*.tmp"))
    assert e.store.load(e.pid)["revision"] == 4


@pytest.mark.parametrize("moment", ["assembling", "after_recheck_before_publication"])
def test_project_change_during_packaging_prevents_publication(exportable, monkeypatch, moment):
    e = exportable
    if moment == "assembling":
        original = e.exporter._zip
        def changing(files):
            result = original(files)
            note(e)
            return result
        monkeypatch.setattr(e.exporter, "_zip", changing)
    else:
        original = e.exporter._folder
        changed = False
        def changing(project_id):
            nonlocal changed
            result = original(project_id)
            if not changed:
                changed = True
                note(e)
            return result
        monkeypatch.setattr(e.exporter, "_folder", changing)
    with pytest.raises(StoreConflict):
        e.exporter.create(e.pid, 4)
    assert confirmed_exports(e) == []
    folder = e.store.project_path(e.pid) / ".research-exports"
    assert not folder.exists() or not list(folder.glob(".*.tmp"))


def test_stale_export_cannot_be_downloaded(exportable):
    e = exportable
    receipt = e.exporter.create(e.pid, 4)
    note(e)
    with pytest.raises(StoreConflict):
        e.exporter.download(e.pid, receipt["export_id"])


def test_project_change_while_reading_existing_zip_cannot_return_stale_download(exportable, monkeypatch):
    e = exportable
    receipt = e.exporter.create(e.pid, 4)
    original = e.exporter._package
    def changing(project_id, export_id):
        result = original(project_id, export_id)
        note(e)
        return result
    monkeypatch.setattr(e.exporter, "_package", changing)
    with pytest.raises(StoreConflict):
        e.exporter.download(e.pid, receipt["export_id"])


@pytest.mark.parametrize("part", ["receipt", "zip"])
def test_corrupt_published_package_is_not_downloaded(exportable, part):
    e = exportable
    receipt = e.exporter.create(e.pid, 4)
    folder = confirmed_exports(e)[0]
    path = folder / ("receipt.json" if part == "receipt" else "research.zip")
    path.write_bytes(path.read_bytes() + b"corrupt")
    with pytest.raises(StoreValidation):
        e.exporter.download(e.pid, receipt["export_id"])


@pytest.mark.parametrize("relative", ["../candidate.xodr", "artifacts/../candidate.xodr",
                                      "artifacts/C:/outside", "artifacts\\candidate.xodr",
                                      "/artifacts/candidate.xodr", "artifacts//candidate.xodr"])
def test_artifact_traversal_paths_are_rejected(exportable, relative):
    e = exportable
    with pytest.raises(StoreValidation):
        e.exporter._read_artifact(e.pid, {"relative_path": relative, "sha256": "a" * 64})


def test_linked_candidate_is_not_packaged(exportable, tmp_path):
    e = exportable
    path = e.store.project_path(e.pid) / e.current["candidate"]["artifacts"][0]["relative_path"]
    outside = tmp_path / "external-candidate.xodr"
    outside.write_bytes(path.read_bytes())
    path.unlink()
    try:
        path.symlink_to(outside)
    except OSError:
        pytest.skip("Windows symlink creation unavailable in this test context")
    with pytest.raises((StoreValidation, validation.ValidationRejected)):
        e.exporter.create(e.pid, 4)
