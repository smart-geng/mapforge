"""Independent transfer tests using real tiny SHP, Store and JobManager archives.

Stored synthetic candidate bytes exercise history only, never quality acceptance.
"""
import hashlib
import io
import json
from pathlib import Path
import stat
import threading
import time
from types import SimpleNamespace
import uuid
import warnings
import zipfile

import pytest

from mapforge.workbench import project_transfer as M
from mapforge.workbench.contracts import canonical_bytes
from mapforge.workbench.store import ProjectStore
from test_workbench_project_relocation import case, _job, _never_bound, _request_index, _sha, _tree


def _request():
    return uuid.uuid4().hex


def _wait(service, job, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = service.get(job["job_id"])
        if current["state"] in {"succeeded", "failed", "cancelled"}:
            return current
        time.sleep(.01)
    pytest.fail(f"transfer did not finish: {current}")


@pytest.fixture
def transfer(case):
    source = M.TransferService(case.store, case.old_raw, case.old_profile)
    target_store = ProjectStore(case.destination)
    target = M.TransferService(target_store, case.raw, case.profile)
    result = SimpleNamespace(case=case, source=source, target=target, target_store=target_store)
    try:
        yield result
    finally:
        source.close()
        target.close()


def _export(t):
    job = _wait(t.source, t.source.start_export(t.case.project["project_id"],
                                               t.case.project["revision"], _request()))
    assert job["state"] == "succeeded", job
    name, data = t.source.download(job["job_id"])
    assert name.endswith(".zip")
    return job, data


def _upload(service, data, *, sha256=None):
    upload = service.create_upload("测试工程.mapforge-project.zip", len(data), sha256 or _sha(data))
    for offset in range(0, len(data), 16384):
        upload = service.append_upload(upload["upload_id"], offset, data[offset:offset + 16384])
    return upload


def _import(t, data):
    upload = _upload(t.target, data)
    return _wait(t.target, t.target.start_import(upload["upload_id"], _request()))


def _unpublished(t):
    assert not (t.target_store.root / t.case.project["project_id"]).exists()
    assert t.target_store.list() == []


def _rewrite(data, transform, *, compression=zipfile.ZIP_STORED):
    """Mutate transport bytes; upload size/hash remain correct for the mutated ZIP."""
    with zipfile.ZipFile(io.BytesIO(data)) as package:
        entries = [(info, package.read(info)) for info in package.infolist()]
    entries = transform(entries)
    stream = io.BytesIO()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        with zipfile.ZipFile(stream, "w", compression=compression) as package:
            for info, content in entries:
                if isinstance(info, str):
                    info = zipfile.ZipInfo(info)
                info.compress_type = compression
                package.writestr(info, content)
    return stream.getvalue()


def test_real_package_retains_drafts_and_history_but_reopens_with_stale_evidence(transfer):
    t, c = transfer, transfer.case
    project_before = _tree(c.directory)
    raw_before = _tree(c.old_raw), _tree(c.raw), c.old_profile.read_bytes(), c.profile.read_bytes()
    other = _job(c.store.root, c.project, job_id="9" * 32, state="running", project_id="f" * 32)
    jobs_before = _tree(c.store.root / ".jobs")
    job, data = _export(t)
    with zipfile.ZipFile(io.BytesIO(data)) as package:
        assert all(info.compress_type == zipfile.ZIP_STORED for info in package.infolist())
        names = package.namelist()
        assert "jobs/" + c.job.name in names and "jobs/" + c.claim.name in names
        assert "jobs/" + other.name not in names
        assert "jobs/" + _request_index(other).name not in names
        assert not any(name.endswith((".shp", ".dbf", ".shx")) for name in names)
        manifest = json.loads(package.read("manifest.json"))
        assert manifest["schema"] == "mapforge/project-transfer/v1"
    imported = _import(t, data)
    assert imported["state"] == "succeeded", imported
    moved = ProjectStore(c.destination).load(c.project["project_id"])
    for key in ("project_id", "name", "intents", "timeline", "cursor", "actions", "candidate",
                "validation", "candidate_history", "validation_history", "context", "content_hash"):
        assert moved[key] == c.project[key], key
    assert moved["revision"] == c.project["revision"] + 1
    assert moved["draft_epoch"] == c.project["draft_epoch"] + 1
    assert moved["capabilities"] == {}
    assert moved["status"]["candidate_stale"] and moved["status"]["validation_stale"]
    assert moved["status"]["can_redo"]
    assert not moved["status"]["formal_export_available"]
    assert moved["source_snapshot"]["locator"] == {
        "source_dir": str(c.raw.resolve()), "profile_path": str(c.profile.resolve())}
    assert {k: v for k, v in moved["source_snapshot"].items() if k != "locator"} == {
        k: v for k, v in c.snapshot.items() if k != "locator"}
    target = t.target_store.project_path(c.project["project_id"])
    for name, content in project_before.items():
        if name != "project.json":
            assert (target / name).read_bytes() == content, name
    assert (target / "unrecognized-empty/nested-empty").is_dir()
    assert any(p.read_bytes() == project_before["project.json"]
               for p in target.glob("relocation-history/**/project.original.json"))
    saved_jobs = {p.name: p.read_bytes() for p in target.glob("relocation-history/**/jobs/*.json")}
    assert saved_jobs == {c.job.name: jobs_before[c.job.name], c.claim.name: jobs_before[c.claim.name]}
    assert not (c.destination / ".jobs").exists()
    assert _tree(c.directory) == project_before
    assert _tree(c.store.root / ".jobs") == jobs_before
    assert (_tree(c.old_raw), _tree(c.raw), c.old_profile.read_bytes(), c.profile.read_bytes()) == raw_before


@pytest.mark.parametrize("bind_after_notes", [False, True], ids=["never-bound", "previous-unbound"])
def test_project_saved_before_editing_was_enabled_round_trips(tmp_path, bind_after_notes):
    c = _never_bound(tmp_path, bind_after_notes=bind_after_notes)
    pid = c.project["project_id"]
    target_store = ProjectStore(tmp_path / "目标 工程库")
    source = M.TransferService(c.store, c.raw, c.profile)
    target = M.TransferService(target_store, c.raw, c.profile)
    try:
        job = _wait(source, source.start_export(pid, c.project["revision"], _request()))
        assert job["state"] == "succeeded", job
        _, data = source.download(job["job_id"])
        upload = _upload(target, data)
        imported = _wait(target, target.start_import(upload["upload_id"], _request()))
        assert imported["state"] == "succeeded", imported
        assert imported["result"]["candidate_stale"] is False and imported["result"]["validation_stale"] is False
        moved = target_store.load(pid)
        for key in ("intents", "timeline", "cursor", "context", "content_hash", "candidate"):
            assert moved[key] == c.project[key], key
        assert moved["revision"] == c.project["revision"] + 1
    finally:
        source.close()
        target.close()


@pytest.mark.parametrize("fault", ["source-byte", "profile-byte", "active-job", "bad-history"])
def test_untrusted_or_busy_input_fails_with_queryable_error_and_no_publication(transfer, fault):
    t, c = transfer, transfer.case
    if fault == "active-job":
        _job(c.store.root, c.project, job_id="2" * 32, state="running")
    elif fault == "bad-history":
        (c.directory / "artifacts/candidate-1.xodr").write_bytes(b"history was damaged")
    if fault in {"active-job", "bad-history"}:
        job = _wait(t.source, t.source.start_export(c.project["project_id"], c.project["revision"], _request()))
        service = t.source
    else:
        _, data = _export(t)
        if fault == "source-byte":
            (c.raw / "vendor-extra.bin").write_bytes(b"different source bytes")
        else:
            c.profile.write_bytes(c.profile.read_bytes() + b"\n# changed profile\n")
        job = _import(t, data)
        service = t.target
    assert job["state"] == "failed", job
    assert job["error"]
    assert service.get(job["job_id"]) == job
    with pytest.raises(M.TransferRejected):
        service.download(job["job_id"])
    _unpublished(t)


def test_same_request_is_idempotent_but_changed_request_payload_is_rejected(transfer):
    t, c = transfer, transfer.case
    key = _request()
    first = t.source.start_export(c.project["project_id"], c.project["revision"], key)
    again = t.source.start_export(c.project["project_id"], c.project["revision"], key)
    assert again["job_id"] == first["job_id"]
    assert _wait(t.source, first)["state"] == "succeeded"
    assert t.source.start_export(c.project["project_id"], c.project["revision"], key)["job_id"] == first["job_id"]
    with pytest.raises(M.TransferRejected):
        t.source.start_export(c.project["project_id"], c.project["revision"] + 1, key)


def test_import_replay_and_existing_project_never_overwrite(transfer):
    t = transfer
    _, data = _export(t)
    upload = _upload(t.target, data)
    key = _request()
    first = _wait(t.target, t.target.start_import(upload["upload_id"], key))
    assert first["state"] == "succeeded", first
    before = _tree(t.target_store.project_path(t.case.project["project_id"]))
    assert t.target.start_import(upload["upload_id"], key)["job_id"] == first["job_id"]
    second = _import(t, data)
    assert second["state"] == "failed", second
    assert second["error"]
    assert _tree(t.target_store.project_path(t.case.project["project_id"])) == before


@pytest.mark.parametrize("fault", ["size-short", "hash", "offset", "overflow"])
def test_upload_is_contiguous_and_bound_to_declared_size_and_hash(transfer, fault):
    s = transfer.target
    data = b"small upload data"
    upload = s.create_upload("test.mapforge-project.zip", len(data), _sha(data if fault != "hash" else b"wrong"))
    if fault in {"offset", "overflow"}:
        with pytest.raises(M.TransferRejected):
            s.append_upload(upload["upload_id"], 1 if fault == "offset" else 0,
                            data if fault == "offset" else data + b"x")
    else:
        if fault == "size-short":
            s.append_upload(upload["upload_id"], 0, data[:-1])
        else:
            # Implementations may reject as soon as the final chunk proves the mismatch.
            try:
                s.append_upload(upload["upload_id"], 0, data)
            except M.TransferRejected:
                _unpublished(transfer)
                return
        try:
            job = s.start_import(upload["upload_id"], _request())
        except M.TransferRejected:
            pass
        else:
            assert _wait(s, job)["state"] == "failed"
    _unpublished(transfer)


@pytest.mark.parametrize("fault", ["extra", "traversal", "absolute", "backslash", "drive", "duplicate",
                                    "case-collision", "symlink", "compressed", "manifest-schema", "hash"])
def test_malformed_package_is_rejected_before_any_project_is_openable(transfer, fault):
    t = transfer
    _, data = _export(t)
    def change(entries):
        if fault == "compressed":
            return entries
        if fault in {"manifest-schema", "hash"}:
            changed = []
            for info, content in entries:
                if info.filename == "manifest.json":
                    manifest = json.loads(content)
                    if fault == "manifest-schema":
                        manifest["schema"] = "mapforge/project-transfer/v999"
                    else:
                        manifest["files"][0]["sha256"] = "0" * 64
                    content = canonical_bytes(manifest)
                changed.append((info, content))
            return changed
        names = {"extra": "unknown-root.txt", "traversal": "../escape.txt", "absolute": "/escape.txt",
                 "backslash": "project\\escape.txt", "drive": "C:/escape.txt"}
        if fault in names:
            return entries + [(names[fault], b"must not be extracted")]
        if fault == "duplicate":
            return entries + [entries[0]]
        if fault == "case-collision":
            return entries + [(entries[0][0].filename.upper(), entries[0][1])]
        info = zipfile.ZipInfo("project/" + t.case.project["project_id"] + "/link")
        info.create_system = 3
        info.external_attr = (stat.S_IFLNK | 0o777) << 16
        return entries + [(info, b"../../escape")]
    malicious = _rewrite(data, change, compression=zipfile.ZIP_DEFLATED if fault == "compressed" else zipfile.ZIP_STORED)
    job = _import(t, malicious)
    assert job["state"] == "failed", job
    assert job["error"]
    _unpublished(t)
    assert not (t.target_store.root.parent / "escape.txt").exists()


def test_new_service_session_cannot_download_or_resume_old_jobs(transfer):
    t = transfer
    job, _ = _export(t)
    restarted = M.TransferService(t.case.store, t.case.old_raw, t.case.old_profile)
    try:
        with pytest.raises(M.TransferRejected):
            restarted.get(job["job_id"])
        with pytest.raises(M.TransferRejected):
            restarted.download(job["job_id"])
    finally:
        restarted.close()


def test_export_version_changes_and_later_saved_edits_disable_download(transfer):
    t, c = transfer, transfer.case
    stale = _wait(t.source, t.source.start_export(c.project["project_id"], c.project["revision"] - 1, _request()))
    assert stale["state"] == "failed" and stale["error"]
    job, _ = _export(t)
    ref = c.snapshot["objects"][0]["id"]
    t.case.store.commit(c.project["project_id"], c.project["revision"], {
        "command_id": "edit-after-export", "type": "annotation", "source_ref": ref,
        "scope": {"feature_ids": [ref]}, "parameters": {"text": "导出后新增待办"}})
    with pytest.raises(M.TransferRejected):
        t.source.download(job["job_id"])
    _unpublished(t)


def test_export_rechecks_input_after_writing_zip(transfer, monkeypatch):
    t, c = transfer, transfer.case
    original = M._write_package
    def mutate_after_copy(*args, **kwargs):
        result = original(*args, **kwargs)
        (c.directory / "unrecognized-note.bin").write_bytes(b"changed during export")
        return result
    monkeypatch.setattr(M, "_write_package", mutate_after_copy)
    job = _wait(t.source, t.source.start_export(c.project["project_id"], c.project["revision"], _request()))
    assert job["state"] == "failed" and job["error"]
    with pytest.raises(M.TransferRejected):
        t.source.download(job["job_id"])


def test_sealed_upload_is_rehashed_before_import(transfer):
    t = transfer
    _, data = _export(t)
    upload = _upload(t.target, data)
    assert upload["state"] == "sealed"
    path = t.target.root / "uploads" / upload["upload_id"] / "upload.zip"
    altered = bytearray(data)
    altered[-1] ^= 1
    path.write_bytes(altered)
    job = _wait(t.target, t.target.start_import(upload["upload_id"], _request()))
    assert job["state"] == "failed" and job["error"]
    _unpublished(t)


def test_publication_failure_keeps_saved_failure_without_visible_project(transfer, monkeypatch):
    t = transfer
    _, data = _export(t)
    def blocked_publish(*args):
        raise OSError("controlled publication failure")
    monkeypatch.setattr(t.target, "_publish_import", blocked_publish)
    job = _import(t, data)
    assert job["state"] == "failed" and "controlled publication failure" in str(job["error"])
    _unpublished(t)
    records = list(t.target.root.glob("jobs/" + job["job_id"] + "/record.json"))
    assert records and b"failed" in records[0].read_bytes()


def test_close_while_worker_prepares_prevents_later_publication(transfer, monkeypatch):
    t = transfer
    _, data = _export(t)
    original = M.relocation.relocate_project
    prepared, release = threading.Event(), threading.Event()
    def pause_after_private_relocation(*args, **kwargs):
        result = original(*args, **kwargs)
        prepared.set()
        assert release.wait(10), "test did not release worker"
        return result
    monkeypatch.setattr(M.relocation, "relocate_project", pause_after_private_relocation)
    upload = _upload(t.target, data)
    job = t.target.start_import(upload["upload_id"], _request())
    try:
        assert prepared.wait(10)
        assert t.target.get(job["job_id"])["state"] == "running"
        _unpublished(t)
        t.target.close()
    finally:
        release.set()
    final = _wait(t.target, job)
    assert final["state"] == "cancelled", final
    assert final["result"] is None
    _unpublished(t)


@pytest.mark.parametrize("fault", ["declared-traversal", "declared-other-root", "declared-link",
                                    "declared-case-collision", "trailing-dot", "device-name",
                                    "file-parent", "empty-parent", "bool-size", "missing-member",
                                    "unknown-field"])
def test_manifest_cannot_authorize_unsafe_or_ambiguous_members(transfer, fault):
    t = transfer
    _, data = _export(t)
    prefix = "project/" + t.case.project["project_id"] + "/"
    def change(entries):
        manifest = json.loads(next(content for info, content in entries if info.filename == "manifest.json"))
        additions = []
        if fault == "bool-size":
            manifest["files"][0]["size"] = True
        elif fault == "unknown-field":
            manifest["client_crs_approved"] = True
        elif fault == "missing-member":
            missing = manifest["files"][0]["path"]
            entries = [(i, b) for i, b in entries if i.filename != missing]
        else:
            name = {"declared-traversal": prefix + "../escape.txt", "declared-other-root": "raw/source.txt",
                    "declared-link": prefix + "link", "declared-case-collision": prefix + "PROJECT.JSON",
                    "trailing-dot": prefix + "unsafe.", "device-name": prefix + "CON.txt",
                    "file-parent": prefix + "project.json/child", "empty-parent": prefix + "empty/child"}[fault]
            content = b"controlled malicious entry"
            manifest["files"].append({"path": name, "size": len(content), "sha256": _sha(content)})
            if fault == "empty-parent":
                manifest["empty_dirs"].append(prefix + "empty")
            info = zipfile.ZipInfo(name)
            if fault == "declared-link":
                info.create_system = 3
                info.external_attr = (stat.S_IFLNK | 0o777) << 16
            additions.append((info, content))
        return [(i, canonical_bytes(manifest) if i.filename == "manifest.json" else b)
                for i, b in entries] + additions
    job = _import(t, _rewrite(data, change))
    assert job["state"] == "failed" and job["error"], job
    _unpublished(t)


@pytest.mark.parametrize("budget", ["package", "manifest", "entry", "total", "count"])
def test_package_budgets_fail_before_extraction(transfer, monkeypatch, tmp_path, budget):
    _, data = _export(transfer)
    path = tmp_path / "valid.zip"
    path.write_bytes(data)
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        entries = archive.infolist()
        manifest_size = len(archive.read("manifest.json"))
        largest = max(info.file_size for info in entries if info.filename != "manifest.json")
        total = sum(info.file_size for info in entries)
    if budget == "package":
        monkeypatch.setattr(M, "MAX_PACKAGE_BYTES", len(data) - 1)
    elif budget == "manifest":
        monkeypatch.setattr(M, "MAX_MANIFEST_BYTES", manifest_size - 1)
    elif budget == "entry":
        monkeypatch.setattr(M.relocation, "MAX_FILE_BYTES", largest - 1)
    elif budget == "total":
        monkeypatch.setattr(M.relocation, "MAX_TOTAL_BYTES", total - 1)
    else:
        monkeypatch.setattr(M.relocation, "MAX_FILES", len(entries) - 2)
    destination = tmp_path / "must-not-extract"
    with pytest.raises(M.TransferRejected):
        M._read_package(path, destination)
    assert not destination.exists()
