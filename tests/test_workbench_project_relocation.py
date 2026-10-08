"""Independent relocation transactions on real, tiny SHP/source workspaces.

No converter, native consumer, or 0621 research result is simulated as passing.
The candidate bytes below only exercise storage and immutable history bindings.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest
import shapefile
import yaml

from mapforge.workbench import project_relocation as M
from mapforge.workbench.contracts import canonical_bytes, content_hash, digest
from mapforge.workbench.exports import ResearchExportService
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ENVELOPE, ProjectStore, StoreConflict


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _tree(path):
    return {p.relative_to(path).as_posix(): p.read_bytes()
            for p in sorted(path.rglob("*")) if p.is_file()}


def _layer(root, name, fields, rows):
    with shapefile.Writer(str(root / name), shapeType=shapefile.POLYLINE) as writer:
        for field in fields:
            writer.field(field, "C", size=80)
        for attrs, parts in rows:
            writer.line(parts)
            writer.record(*attrs)


def _source(root, *, vendor_size=None):
    raw = root / "原始 SHP"
    raw.mkdir(parents=True)
    _layer(raw, "junction", ["PID", "ENTER", "LEAVE"], [
        (["synthetic-j1", "r1", "r2"], [[[0, 0], [20, 0], [20, 20], [0, 0]]]),
    ])
    _layer(raw, "road", ["PID"], [
        (["r1"], [[[0, 0], [0, 20]]]), (["r2"], [[[20, 0], [20, 20]]]),
    ])
    _layer(raw, "lane", ["PID", "ROAD", "path"], [
        (["same-business-id", "r1", "F:/original/vendor-value"],
         [[[0, 0], [0, 5]], [[0, 10], [0, 15]]]),
        (["same-business-id", "r1", "unrecognized:keep-exactly"], [[[1, 0], [1, 5]]]),
    ])
    vendor = b"unknown source bytes\x00\xff" if vendor_size is None else b"v" * vendor_size
    (raw / "vendor-extra.bin").write_bytes(vendor)
    profile = root / "源 profile.yaml"
    profile.write_text(yaml.safe_dump({
        "profile": "small-independent-relocation-fixture", "units": {"length": "mystery"},
        "layers": {
            "junction": {"file": "junction", "fields": {
                "id": "PID", "enter_roads": "ENTER", "leave_roads": "LEAVE"}},
            "road": {"file": "road", "fields": {"id": "PID"}},
            "lane": {"file": "lane", "fields": {"id": "PID", "road": "ROAD"}},
        },
    }), encoding="utf8")
    return raw, profile


def _job(workspace, project, *, job_id="1" * 32, state="succeeded", project_id=None):
    """Use the real claim/archive writers without launching a fake quality worker."""
    pid = project_id or project["project_id"]
    key = digest({"project_id": pid, "request_id": job_id})
    payload = {"synthetic_storage_only": True}
    signature = digest({"operation": "validate", "payload": payload, "project_id": pid,
                        "revision": project["revision"], "content_hash": project["content_hash"]})
    job = {"job_id": job_id, "project_id": pid, "operation": "validate",
           "base_revision": project["revision"], "content_hash": project["content_hash"],
           "input_hash": signature, "_request_key": key, "state": "starting", "started_at": 1.0}
    manager = JobManager(archive_dir=workspace / ".jobs")
    try:
        previous = manager._claim_request(job)
        assert previous is None or previous["job_id"] == job_id
        assert manager._archive(job)
        job.update(state=state, result=payload, error=None)
        if state in {"succeeded", "failed", "cancelled", "timed_out"}:
            job["finished_at"] = 2.0
        assert manager._archive(job)
        assert job["archive"]["state"] == "persisted"
    finally:
        manager.close()
    return workspace / ".jobs" / (job_id + ".json")


def _request_index(job_path):
    record = json.loads(job_path.read_bytes())["payload"]
    return job_path.parent / ("request-" + record["request_key"] + ".json")


def _save_payload(path, project):
    path.write_bytes(canonical_bytes({"schema": ENVELOPE, "payload": project, "sha256": digest(project)}))


@pytest.fixture
def case(tmp_path, request):
    old_raw, old_profile = _source(tmp_path / "旧 原料", **getattr(request, "param", {}))
    fresh = tmp_path / "新 原料"
    fresh.mkdir()
    raw, profile = fresh / old_raw.name, fresh / old_profile.name
    shutil.copytree(old_raw, raw)
    shutil.copyfile(old_profile, profile)
    snapshot = SourceCatalog(old_raw, old_profile).snapshot("synthetic-j1")
    store = ProjectStore(tmp_path / "旧 工程库")
    project = store.create(snapshot, "含历史与撤回待办的迁移工程")
    pid = project["project_id"]
    source_ref = snapshot["objects"][0]["id"]
    spec = {"capability_id": "synthetic-stored-capability", "type": "shared_boundary_c2_normal_delta",
            "source_ref": source_ref, "feature_ids": [source_ref], "baseline_sha256": "c" * 64,
            "normal_delta_m": {"min": -.1, "max": .1}}
    project = store.register_capabilities(pid, project["revision"], "register", [spec], "a" * 64, "b" * 64)
    for index in (1, 2):
        command = {"command_id": f"note-{index}", "type": "annotation", "source_ref": source_ref,
                   "scope": {"feature_ids": [source_ref]},
                   "parameters": {"text": f"迁移后仍保留的待办 {index}", "status": "unresolved"}}
        project = store.commit(pid, project["revision"], command)
    project = store.undo(pid, project["revision"], "keep-redo")
    directory = store.project_path(pid)
    (directory / "artifacts").mkdir()
    for index in (1, 2):
        data = f"synthetic storage candidate {index}; NOT geometry acceptance".encode()
        relative = f"artifacts/candidate-{index}.xodr"
        (directory / relative).write_bytes(data)
        candidate = {"candidate_id": f"candidate-{index}", "target_content_hash": project["content_hash"],
                     "context": project["context"], "artifacts": [{"relative_path": relative, "sha256": _sha(data)}],
                     "affected_ids": [source_ref], "checks": [{"id": "synthetic-compile", "status": "PASS"}]}
        project = store.accept_candidate(pid, project["revision"], f"accept-{index}", candidate)
        validation = {"bundle_id": f"check-{index}", "candidate_id": candidate["candidate_id"],
                      "candidate_hash": digest(project["candidate"]), "context": project["context"],
                      "checks": [{"id": "CRS", "status": "UNAVAILABLE"}], "decision": "BLOCKED"}
        project = store.attach_validation(pid, project["revision"], f"check-{index}", validation)
    extra = directory / ".research-exports" / "historical"
    extra.mkdir(parents=True)
    (extra / "research.zip").write_bytes(b"historical opaque bytes, not a current export")
    (directory / "unrecognized-note.bin").write_bytes(b"PASSTHROUGH\r\nunchanged\n")
    (directory / "unrecognized-empty" / "nested-empty").mkdir(parents=True)
    job = _job(store.root, project)
    return SimpleNamespace(store=store, directory=directory, project=project, snapshot=snapshot,
                           old_raw=old_raw, old_profile=old_profile, raw=raw, profile=profile,
                           destination=tmp_path / "迁移 后的中文工程库", job=job, claim=_request_index(job))


def _relocate(case):
    return M.relocate_project(case.directory, case.destination, case.raw, case.profile)


def _assert_not_published(case):
    assert not (case.destination / case.project["project_id"]).exists()
    if case.destination.exists():
        assert not any(p.is_dir() and len(p.name) == 32 and all(c in "0123456789abcdef" for c in p.name)
                       for p in case.destination.iterdir())


def test_real_sources_move_with_complete_history_but_no_current_candidate_or_check(case):
    project_before = _tree(case.directory)
    source_before = _tree(case.old_raw), case.old_profile.read_bytes()
    target_source_before = _tree(case.raw), case.profile.read_bytes()
    job_before = case.job.read_bytes()
    claims_and_jobs_before = _tree(case.store.root / ".jobs")
    result = _relocate(case)
    assert result["status"] == "RELOCATED_RECOMPUTE_REQUIRED"
    assert result["accepted"] is False and result["formal_delivery"] is False
    target = Path(result["project_directory"])
    assert target == case.destination / case.project["project_id"]
    moved = ProjectStore(case.destination).load(case.project["project_id"])
    old = case.project
    assert moved["revision"] == old["revision"] + 1
    assert moved["draft_epoch"] == old["draft_epoch"] + 1
    assert moved["content_hash"] == old["content_hash"]
    assert moved["context"] == old["context"], "same fingerprints must not make old results current"
    for key in ("project_id", "name", "intents", "timeline", "cursor", "actions", "candidate",
                "validation", "candidate_history", "validation_history"):
        assert moved[key] == old[key], key
    assert moved["capabilities"] == {} and old["capabilities"]
    assert moved["journal"][:-1] == old["journal"]
    assert moved["journal"][-1]["kind"] == "relocation"
    assert moved["status"]["candidate_stale"] and moved["status"]["validation_stale"]
    assert moved["status"]["can_redo"]
    assert moved["status"]["formal_export_available"] is False
    assert {k: v for k, v in moved["source_snapshot"].items() if k != "locator"} == {
        k: v for k, v in case.snapshot.items() if k != "locator"}
    assert moved["source_snapshot"]["locator"] == {
        "source_dir": str(case.raw.resolve()), "profile_path": str(case.profile.resolve())}
    for name, data in project_before.items():
        if name != "project.json":
            assert (target / name).read_bytes() == data, name
    assert (target / "unrecognized-empty" / "nested-empty").is_dir()
    assert list((target / "unrecognized-empty" / "nested-empty").iterdir()) == []
    history_files = list(case.destination.glob("**/relocation-history/**/project.original.json"))
    assert any(path.read_bytes() == project_before["project.json"] for path in history_files)
    history_jobs = list(case.destination.glob("**/relocation-history/**/jobs/*.json"))
    assert {path.name: path.read_bytes() for path in history_jobs} == claims_and_jobs_before
    assert result["receipt"]["historical_jobs"] == {name: _sha(data) for name, data in claims_and_jobs_before.items()}
    assert not (case.destination / ".jobs").exists()
    assert _tree(case.directory) == project_before
    assert (_tree(case.old_raw), case.old_profile.read_bytes()) == source_before
    assert (_tree(case.raw), case.profile.read_bytes()) == target_source_before
    assert case.job.read_bytes() == job_before
    assert _tree(case.store.root / ".jobs") == claims_and_jobs_before
    exports = ResearchExportService(ProjectStore(case.destination), source_dir=case.raw, profile_path=case.profile)
    with pytest.raises(StoreConflict):
        exports.create(moved["project_id"], moved["revision"])
    with pytest.raises(StoreConflict):
        exports.download(moved["project_id"], "research-" + "f" * 32)


def test_relocation_does_not_restore_or_copy_other_projects_jobs(case):
    other = _job(case.store.root, case.project, job_id="2" * 32, state="running", project_id="f" * 32)
    other_bytes = other.read_bytes()
    other_claim = _request_index(other)
    other_claim_bytes = other_claim.read_bytes()
    _relocate(case)
    saved = {p.name: p.read_bytes() for p in case.destination.glob("**/jobs/*.json")}
    assert set(saved) == {case.job.name, case.claim.name}
    assert other.name not in saved and other_claim.name not in saved
    assert other.read_bytes() == other_bytes
    assert other_claim.read_bytes() == other_claim_bytes
    manager = JobManager(archive_dir=case.destination / ".jobs")
    try:
        assert manager._jobs == {}
    finally:
        manager.close()


def test_second_relocation_retains_the_first_history_without_reviving_acceptance(case, tmp_path):
    first = _relocate(case)
    first_directory = Path(first["project_directory"])
    first_bytes = _tree(first_directory)
    second_workspace = tmp_path / "再迁移 中文 工程库"
    second = M.relocate_project(first_directory, second_workspace, case.raw, case.profile)
    second_directory = Path(second["project_directory"])
    assert second["project"]["revision"] == first["project"]["revision"] + 1
    assert second["project"]["draft_epoch"] == first["project"]["draft_epoch"] + 1
    assert second["project"]["status"]["candidate_stale"]
    assert second["project"]["status"]["validation_stale"]
    for name, data in first_bytes.items():
        if name != "project.json":
            assert (second_directory / name).read_bytes() == data, name
    assert _tree(first_directory) == first_bytes
    assert not (second_workspace / ".jobs").exists()


@pytest.mark.parametrize("state", ["starting", "running", "cancelling"])
def test_nonterminal_related_job_blocks_migration_without_publishing(case, state):
    _job(case.store.root, case.project, job_id="3" * 32, state=state)
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before


@pytest.mark.parametrize("damage", ["malformed-index", "orphan-index", "missing-index", "input-hash",
                                     "project-id", "request-key", "unsafe-job-id", "duplicate-job-request-key"])
def test_request_claim_must_be_complete_and_match_exactly_one_job(case, damage):
    if damage == "malformed-index":
        case.claim.write_bytes(b"{broken")
    elif damage == "orphan-index":
        case.job.unlink()
    elif damage == "missing-index":
        case.claim.unlink()
    elif damage == "duplicate-job-request-key":
        record = json.loads(case.job.read_bytes())["payload"]
        record["job_id"] = "4" * 32
        (case.job.parent / (record["job_id"] + ".json")).write_bytes(JobManager._envelope(record))
    else:
        claim = json.loads(case.claim.read_bytes())["payload"]
        field, value = {
            "input-hash": ("input_hash", "0" * 64),
            "project-id": ("project_id", "f" * 32),
            "request-key": ("request_key", "0" * 64),
            "unsafe-job-id": ("job_id", "../not-a-job"),
        }[damage]
        claim[field] = value
        case.claim.write_bytes(JobManager._envelope(claim))
    before = _tree(case.directory)
    archived = _tree(case.store.root / ".jobs")
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before
    assert _tree(case.store.root / ".jobs") == archived


@pytest.mark.parametrize("change", ["unknown-source-bytes", "profile-bytes", "source-file-added", "source-file-missing"])
def test_relocated_source_must_have_exact_original_manifest(case, change):
    if change == "unknown-source-bytes":
        (case.raw / "vendor-extra.bin").write_bytes(b"different")
    elif change == "profile-bytes":
        case.profile.write_bytes(case.profile.read_bytes() + b"\n# changed profile bytes\n")
    elif change == "source-file-added":
        (case.raw / "new-unknown-file.bin").write_bytes(b"new")
    else:
        (case.raw / "vendor-extra.bin").unlink()
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before


@pytest.mark.parametrize("change", ["raw-path", "point", "record-index", "unsupported-nested-locator", "boolean-type"])
def test_rehashing_a_forged_source_snapshot_does_not_authorize_its_semantics(case, change):
    path = case.directory / "project.json"
    project = json.loads(path.read_bytes())["payload"]
    snapshot = project["source_snapshot"]
    obj = next(o for o in snapshot["objects"] if o["role"] == "lane")
    if change == "raw-path":
        obj["raw_attributes"]["path"] = "replacement is not a transport locator"
    elif change == "point":
        obj["points"][0][0] += 10
    elif change == "record-index":
        obj["source_ref"]["record_index"] += 1
    elif change == "boolean-type":
        assert snapshot["capabilities"]["compile"] is False
        snapshot["capabilities"]["compile"] = 0
    else:
        obj["locator"] = {"path": "must remain source identity"}
    snapshot["content_hash"] = digest({k: v for k, v in snapshot.items() if k not in {"locator", "content_hash"}})
    project["content_hash"] = content_hash(snapshot, project["intents"])
    project["context"]["source_hash"] = digest({k: v for k, v in snapshot.items() if k != "locator"})
    _save_payload(path, project)
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before


@pytest.mark.parametrize("part", ["current-envelope", "previous-envelope", "candidate-history-artifact", "job-envelope"])
def test_corrupt_bound_history_cannot_be_silently_dropped_or_repaired(case, part):
    if part == "current-envelope":
        (case.directory / "project.json").write_bytes(b"{broken")
    elif part == "previous-envelope":
        (case.directory / "previous.json").write_bytes(b"{broken")
    elif part == "candidate-history-artifact":
        (case.directory / "artifacts/candidate-1.xodr").write_bytes(b"changed historical artifact")
    else:
        case.job.write_bytes(b"{broken")
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before


@pytest.mark.parametrize("damage", ["validation-hash", "historical-candidate-id", "historical-context",
                                     "empty-candidate", "empty-validation"])
def test_valid_envelope_does_not_make_broken_history_relationships_valid(case, damage):
    path = case.directory / "project.json"
    project = json.loads(path.read_bytes())["payload"]
    if damage == "validation-hash":
        project["validation"]["candidate_hash"] = "0" * 64
    elif damage == "historical-candidate-id":
        project["validation_history"][0]["candidate_id"] = "nonexistent-candidate"
    elif damage == "historical-context":
        project["validation_history"][0]["context"]["compiler_hash"] = "f" * 64
    elif damage == "empty-candidate":
        project["candidate"] = {}
    else:
        project["validation"] = {}
    _save_payload(path, project)
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before


def test_existing_destination_is_not_merged_or_overwritten(case):
    case.destination.mkdir()
    marker = case.destination / "keep.txt"
    marker.write_bytes(b"preexisting user data")
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    assert _tree(case.destination) == {"keep.txt": b"preexisting user data"}


@pytest.mark.parametrize("input_name", ["directory", "raw"])
def test_destination_cannot_be_inside_any_input(case, input_name):
    root = getattr(case, input_name)
    case.destination = root / "nested-destination"
    before = _tree(root)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    assert not case.destination.exists()
    assert _tree(root) == before


def test_destination_cannot_replace_the_profile_file(case):
    case.destination = case.profile
    before = case.profile.read_bytes()
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    assert case.profile.read_bytes() == before


@pytest.mark.parametrize("state", ["failed", "cancelled", "timed_out"])
def test_terminal_failures_are_copied_as_history_without_restarting_jobs(case, state):
    _job(case.store.root, case.project, state=state)
    before = case.job.read_bytes()
    result = _relocate(case)
    history = Path(result["project_directory"]) / result["receipt"]["history_directory"]
    assert (history / "jobs" / case.job.name).read_bytes() == before
    assert not (case.destination / ".jobs").exists()
    assert result["project"]["status"]["candidate_stale"]


@pytest.mark.parametrize("relative", ["artifacts/../../outside.xodr", "C:/outside.xodr", "artifacts\\outside.xodr"])
def test_history_artifact_paths_cannot_escape_the_project(case, relative):
    path = case.directory / "project.json"
    project = json.loads(path.read_bytes())["payload"]
    project["candidate_history"][0]["artifacts"][0]["relative_path"] = relative
    _save_payload(path, project)
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before


@pytest.mark.parametrize("changed", ["project", "source", "profile", "job", "request-index", "staged-artifact"])
def test_observed_drift_after_copy_cannot_publish_a_current_project(case, monkeypatch, changed):
    original = M._copy_tree
    changed_path = []
    def corrupt_after_copy(source, target, manifest):
        original(source, target, manifest)
        if changed == "project":
            path = case.directory / "unrecognized-note.bin"
        elif changed == "source":
            path = case.raw / "vendor-extra.bin"
        elif changed == "profile":
            path = case.profile
        elif changed == "job":
            path = case.job
        elif changed == "request-index":
            path = case.claim
        else:
            path = Path(target) / "artifacts/candidate-1.xodr"
        path.write_bytes(path.read_bytes() + b"\ncontrolled concurrent change")
        changed_path.append(path)
    monkeypatch.setattr(M, "_copy_tree", corrupt_after_copy)
    with pytest.raises(M.RelocationRejected) as failure:
        _relocate(case)
    assert changed_path
    _assert_not_published(case)
    assert changed_path[0].read_bytes().endswith(b"controlled concurrent change")
    assert ProjectStore(case.destination).list() == []
    pending = Path(failure.value.pending_directory)
    assert (pending / "failure.json").is_file()


def test_failed_atomic_publish_leaves_only_unopenable_pending_evidence(case, monkeypatch):
    def deny(pending, final):
        assert Path(pending).is_dir() and not Path(final).exists()
        raise OSError("controlled publication failure")
    monkeypatch.setattr(M, "_publish", deny)
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected) as failure:
        _relocate(case)
    _assert_not_published(case)
    assert _tree(case.directory) == before
    assert ProjectStore(case.destination).list() == []
    pending = Path(failure.value.pending_directory)
    assert pending.is_relative_to(case.destination)
    assert (pending / "failure.json").is_file()
    assert "controlled publication failure" in (pending / "failure.json").read_text(encoding="utf8")


def test_destination_race_at_publish_preserves_the_other_owners_directory(case, monkeypatch):
    original = M._publish
    def occupy(pending, final):
        final = Path(final)
        final.mkdir()
        (final / "keep.txt").write_bytes(b"other owner's data")
        return original(pending, final)
    monkeypatch.setattr(M, "_publish", occupy)
    before = _tree(case.directory)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    final = case.destination / case.project["project_id"]
    assert _tree(final) == {"keep.txt": b"other owner's data"}
    assert _tree(case.directory) == before


class _BoundedReader:
    def __init__(self, stream, reads, after_read=None):
        self.stream, self.reads, self.after_read = stream, reads, after_read

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return self.stream.__exit__(*args)

    def __getattr__(self, name):
        return getattr(self.stream, name)

    def read(self, size=-1):
        assert 0 < size <= 1024 * 1024, "source hashing must request bounded chunks"
        self.reads.append(size)
        result = self.stream.read(size)
        if self.after_read is not None:
            action, self.after_read = self.after_read, None
            action()
        return result


@pytest.mark.parametrize("case", [{"vendor_size": 192 * 1024}], indirect=True)
def test_source_larger_than_parser_cap_is_streamed_under_source_tree_budget(case, monkeypatch):
    # Low caps exercise the same boundary without allocating hundreds of MB.
    parse_cap, source_budget = 64 * 1024, 2 * 1024 * 1024
    monkeypatch.setattr(M, "MAX_FILE_BYTES", parse_cap)
    monkeypatch.setattr(M, "MAX_TOTAL_BYTES", source_budget)
    vendor = case.raw / "vendor-extra.bin"
    assert parse_cap < vendor.stat().st_size < source_budget
    read_bytes, open_file = Path.read_bytes, Path.open
    stream_reads = []
    def no_whole_read(path):
        assert path != vendor, "large source file must not be read_bytes()"
        return read_bytes(path)
    def bounded_open(path, *args, **kwargs):
        stream = open_file(path, *args, **kwargs)
        mode = args[0] if args else kwargs.get("mode", "r")
        return _BoundedReader(stream, stream_reads) if path == vendor and mode == "rb" else stream
    monkeypatch.setattr(Path, "read_bytes", no_whole_read)
    monkeypatch.setattr(Path, "open", bounded_open)
    result = _relocate(case)
    assert result["status"] == "RELOCATED_RECOMPUTE_REQUIRED"
    assert result["receipt"]["source_files"]["vendor-extra.bin"] == {
        "size": 192 * 1024, "sha256": _sha(b"v" * (192 * 1024))}
    assert stream_reads and all(size <= 1024 * 1024 for size in stream_reads)
    assert result["project"]["status"]["candidate_stale"]
    assert M.MAX_FILE_BYTES == parse_cap, "source allowance must not relax parser limit"


def test_larger_source_allowance_does_not_relax_project_or_json_read_limit(case, monkeypatch):
    parse_cap = 64 * 1024
    monkeypatch.setattr(M, "MAX_FILE_BYTES", parse_cap)
    oversized = case.directory / "unrecognized-note.bin"
    original = b"j" * (parse_cap + 1)
    oversized.write_bytes(original)
    with pytest.raises(M.RelocationRejected):
        M._read(oversized)
    with pytest.raises(M.RelocationRejected):
        _relocate(case)
    _assert_not_published(case)
    assert oversized.read_bytes() == original


@pytest.mark.parametrize("budget, allowed", [(512, True), (511, False)])
def test_streamed_source_manifest_enforces_total_budget_including_prior_files(tmp_path, monkeypatch, budget, allowed):
    folder = tmp_path / "source-budget"
    folder.mkdir()
    (folder / "a.bin").write_bytes(b"a" * 256)
    (folder / "b.bin").write_bytes(b"b" * 256)
    monkeypatch.setattr(M, "MAX_FILE_BYTES", 64)
    monkeypatch.setattr(M, "MAX_TOTAL_BYTES", budget)
    if allowed:
        manifest = M._inventory(folder, max_file_bytes=budget)
        assert manifest == {name: {"size": 256, "sha256": _sha(value * 256)}
                            for name, value in (("a.bin", b"a"), ("b.bin", b"b"))}
    else:
        with pytest.raises(M.RelocationRejected):
            M._inventory(folder, max_file_bytes=budget)
    # The same source-sized files are still refused by a project inventory.
    with pytest.raises(M.RelocationRejected):
        M._inventory(folder)


@pytest.mark.parametrize("change", ["growth-over-budget", "truncation"])
def test_streaming_detects_actual_size_drift_without_trusting_initial_stat(tmp_path, monkeypatch, change):
    path = tmp_path / "changing-source.bin"
    path.write_bytes(b"a" * 32)
    reads = []
    open_file = Path.open
    def mutate():
        with open_file(path, "ab" if change == "growth-over-budget" else "wb") as stream:
            stream.write(b"b" * 64 if change == "growth-over-budget" else b"b")
    def controlled_open(item, *args, **kwargs):
        stream = open_file(item, *args, **kwargs)
        mode = args[0] if args else kwargs.get("mode", "r")
        return _BoundedReader(stream, reads, mutate) if item == path and mode == "rb" else stream
    monkeypatch.setattr(Path, "open", controlled_open)
    with pytest.raises(M.RelocationRejected):
        M._stream_binding(path, 64)
    assert reads


@pytest.mark.skipif(os.name != "nt", reason="Windows junction contract")
def test_windows_junction_project_child_is_rejected_without_traversal(case, tmp_path):
    outside = tmp_path / "unrelated"
    outside.mkdir()
    marker = outside / "keep.txt"
    marker.write_bytes(b"do not traverse or copy")
    junction = case.directory / "linked-assets"
    process = subprocess.run(["cmd.exe", "/c", "mklink", "/J", str(junction), str(outside)],
                             capture_output=True, shell=False)
    if process.returncode:
        pytest.skip("Host cannot create junction: " + process.stderr.decode(errors="replace"))
    try:
        with pytest.raises(M.RelocationRejected):
            _relocate(case)
        _assert_not_published(case)
        assert marker.read_bytes() == b"do not traverse or copy"
    finally:
        junction.rmdir()  # Remove only the fixture junction, never its target.
