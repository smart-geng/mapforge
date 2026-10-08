"""Server acceptance tests; synthetic workers exercise coordination, not geometry quality."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from mapforge.workbench import compiler, editing
from mapforge.workbench.contracts import (
    StoreConflict, StoreReadOnly, StoreValidation, canonical_bytes, context_for,
)
from mapforge.workbench.editing import EditingService
from mapforge.workbench.store import ProjectStore


class CompletedJobs:
    """Controlled worker replies; every referenced artifact is a real file."""
    def __init__(self):
        self.jobs = {}
        self.payloads = {}

    def start(self, project, operation, payload, request_id):
        job_id = hashlib.sha256(request_id.encode()).hexdigest()[:32]
        if job_id in self.jobs:
            assert self.payloads[job_id] == payload
            return copy.deepcopy(self.jobs[job_id])
        self.payloads[job_id] = copy.deepcopy(payload)
        draft = payload["draft"]
        data = b"candidate actual file bytes"
        identity = "candidate-" + job_id
        evidence = {"candidate_id": identity, "candidate_sha256": hashlib.sha256(data).hexdigest(),
                    "target_revision": draft["revision"], "target_draft_epoch": draft["draft_epoch"],
                    "target_content_hash": draft["content_hash"], "context": draft["context"]}
        candidate = {"candidate_id": identity, "target_content_hash": draft["content_hash"],
                     "context": draft["context"], "artifacts": [], "affected_ids": ["boundary"], "checks": []}
        for name, raw in (("candidate.xodr", data), ("evidence.json", canonical_bytes(evidence))):
            relative = "artifacts/" + identity + "/" + name
            path = Path(payload["project_directory"]) / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
            candidate["artifacts"].append({"relative_path": relative, "sha256": hashlib.sha256(raw).hexdigest()})
        self.jobs[job_id] = {"job_id": job_id, "project_id": project["project_id"], "operation": operation,
                             "base_revision": project["revision"], "content_hash": project["content_hash"],
                             "state": "succeeded", "historical": False,
                             "archive": {"state": "persisted", "recorded_state": "succeeded"},
                             "result": {"status": "COMPILED", "candidate": candidate, "evidence": evidence}}
        return copy.deepcopy(self.jobs[job_id])

    def get(self, job_id, *, project_id, **kwargs):
        if self.jobs[job_id]["project_id"] != project_id:
            raise KeyError("different project")
        return copy.deepcopy(self.jobs[job_id])


@pytest.fixture
def setup(tmp_path, monkeypatch):
    raw = b"registered baseline"
    manifest = b"registered source manifest"
    baseline = tmp_path / "baseline.xodr"
    baseline.write_bytes(raw)
    baseline.with_suffix(".source-lanes.json").write_bytes(manifest)
    baseline_hash = hashlib.sha256(raw).hexdigest()
    monkeypatch.setattr(compiler, "BASELINE_SHA256", baseline_hash)
    monkeypatch.setattr(compiler, "SOURCE_MANIFEST_SHA256", hashlib.sha256(manifest).hexdigest())
    fingerprints = {"compiler": "a" * 64, "policy": "b" * 64}
    spec = {"capability_id": compiler.CAPABILITY_ID, "type": compiler.INTENT_TYPE, "source_ref": "boundary",
            "feature_ids": ["boundary"], "baseline_sha256": baseline_hash,
            "normal_delta_m": {"min": -.1, "max": .1}}
    registration = SimpleNamespace(
        context=lambda snap: context_for(snap, fingerprints["compiler"], fingerprints["policy"]),
        store_spec=lambda: copy.deepcopy(spec), capability=lambda: {**spec, "control": {"continuous": True}},
        prepared=SimpleNamespace(data=raw),
    )
    calls = []

    def register(snapshot, **kwargs):
        calls.append(kwargs)
        return registration

    monkeypatch.setattr(compiler, "register_verified_baseline", register)
    monkeypatch.setattr(editing, "verify_source_snapshot", lambda *a: {"matches": True, "issues": []})
    snapshot = {"snapshot_id": "dataset", "junction_id": compiler.JUNCTION_ID,
                "profile": {"sha256": "c" * 64}, "source_files": [],
                "locator": {"source_dir": "untrusted-client-path", "profile_path": "untrusted-profile"},
                "objects": [{"id": "boundary", "source_ref": {"snapshot_id": "dataset"}}]}
    store, jobs = ProjectStore(tmp_path / "projects"), CompletedJobs()
    service = EditingService(store, jobs, baseline_path=baseline,
                             source_dir=tmp_path / "source", profile_path=tmp_path / "profile.yaml")
    project = store.create(snapshot)
    command = {"command_id": "edit-1", "type": compiler.INTENT_TYPE, "source_ref": "boundary",
               "scope": {"capability_id": compiler.CAPABILITY_ID, "feature_ids": ["boundary"]},
               "parameters": {"normal_delta_m": -.015, "baseline_sha256": baseline_hash}}
    return SimpleNamespace(service=service, store=store, jobs=jobs, project=project,
                           pid=project["project_id"], command=command, calls=calls,
                           fingerprints=fingerprints, registration=registration, baseline=baseline)


def enabled(env):
    return env.service.enable(env.pid, 0, "enable")["project"]


def previewed(env):
    project = enabled(env)
    result = env.service.preview(env.pid, project["revision"], env.command, "preview-request")
    return result


def confirmed(env):
    preview = previewed(env)
    project = env.store.commit(env.pid, 1, preview["preview"]["command"])
    return project, preview["job"]["job_id"]


def test_describe_is_read_only_cached_and_uses_only_configured_paths(setup):
    e = setup
    path = e.store.project_path(e.pid) / "project.json"
    original = path.read_bytes()
    first = e.service.describe(e.project)
    assert first["supported"] and not first["enabled"]
    for _ in range(3):
        assert e.service.describe(e.project) == first
    assert path.read_bytes() == original
    assert len(e.calls) == 1
    assert e.calls[0]["source_dir"] == e.service.source_dir
    assert "untrusted-client-path" not in str(e.calls)
    other = copy.deepcopy(e.project)
    other["source_snapshot"]["junction_id"] = "unregistered"
    assert not e.service.describe(other)["supported"]
    assert len(e.calls) == 1


def test_preview_does_not_save_and_only_exact_commit_can_accept(setup):
    e = setup
    job = previewed(e)
    current = e.store.load(e.pid)
    assert current["revision"] == current["draft_epoch"] == 1
    assert current["intents"] == [] and current["candidate"] is None
    with pytest.raises(StoreConflict, match="尚未确认"):
        e.service.accept(e.pid, 1, "accept", job["job"]["job_id"])
    committed = e.store.commit(e.pid, 1, job["preview"]["command"])
    assert committed["content_hash"] == job["preview"]["target_content_hash"]
    accepted = e.service.accept(e.pid, 2, "accept", job["job"]["job_id"])
    assert accepted["revision"] == 3 and accepted["draft_epoch"] == 2
    assert not accepted["status"]["candidate_stale"]
    assert not accepted["status"]["formal_export_available"]
    assert e.store.load(e.pid)["candidate"] == accepted["candidate"]


def test_confirmed_draft_compile_accepts_without_an_extra_edit(setup):
    e = setup
    project = enabled(e)
    job = e.service.compile(e.pid, project["revision"], "compile")["job"]
    accepted = e.service.accept(e.pid, 1, "accept", job["job_id"])
    assert accepted["intents"] == [] and accepted["draft_epoch"] == 1
    assert accepted["candidate"]["accepted_epoch"] == 1


def test_enable_and_accept_retry_are_idempotent_but_new_accept_is_stale(setup):
    e = setup
    first = enabled(e)
    assert e.service.enable(e.pid, 0, "enable")["project"] == first
    preview = e.service.preview(e.pid, 1, e.command, "preview")
    e.store.commit(e.pid, 1, preview["preview"]["command"])
    job_id = preview["job"]["job_id"]
    accepted = e.service.accept(e.pid, 2, "accept", job_id)
    assert e.service.accept(e.pid, 2, "accept", job_id) == accepted
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, 3, "different-accept", job_id)
    assert e.store.load(e.pid)["revision"] == 3


@pytest.mark.parametrize("change", ["annotation", "undo_redo", "context"])
def test_late_result_cannot_accept_after_any_draft_change(setup, change):
    e = setup
    project, job_id = confirmed(e)
    if change == "annotation":
        project = e.store.commit(e.pid, 2, {"command_id": "note", "type": "annotation", "source_ref": "boundary",
                                          "scope": {"feature_ids": ["boundary"]}, "parameters": {"text": "changed"}})
    elif change == "undo_redo":
        original_hash = project["content_hash"]
        project = e.store.undo(e.pid, 2, "undo")
        project = e.store.redo(e.pid, 3, "redo")
        assert project["content_hash"] == original_hash  # Same bytes do not revive the old epoch.
    else:
        project = e.store.set_context(e.pid, 2, "context", "f" * 64, "b" * 64)
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, project["revision"], "accept", job_id)
    assert e.store.load(e.pid)["candidate"] is None


@pytest.mark.parametrize("state", ["running", "cancelled", "failed", "timed_out"])
def test_unsuccessful_or_cancelled_job_never_accepts_buffered_result(setup, state):
    e = setup
    project, job_id = confirmed(e)
    e.jobs.jobs[job_id]["state"] = state
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, project["revision"], "accept", job_id)
    assert e.store.load(e.pid)["candidate"] is None


@pytest.mark.parametrize("change", ["historical", "archive_failed", "archive_disabled", "archive_old", "rejected"])
def test_unusable_results_cannot_replace_last_candidate(setup, change):
    e = setup
    project, job_id = confirmed(e)
    accepted = e.service.accept(e.pid, 2, "first-accept", job_id)
    next_job = e.service.compile(e.pid, accepted["revision"], "next")["job"]
    record = e.jobs.jobs[next_job["job_id"]]
    if change == "historical":
        record["historical"] = True
    elif change == "archive_old":
        record["archive"]["recorded_state"] = "running"
    elif change.startswith("archive_"):
        record["archive"]["state"] = change.split("_")[1]
    else:
        record["result"] = {"status": "REJECTED", "candidate": None, "error": {"message": "残差退步"}}
    with pytest.raises((StoreConflict, StoreValidation)):
        e.service.accept(e.pid, accepted["revision"], "next-accept", next_job["job_id"])
    after = e.store.load(e.pid)
    assert after["candidate"] == accepted["candidate"] and after["revision"] == accepted["revision"]


@pytest.mark.parametrize("target", ["candidate.xodr", "evidence.json"])
def test_accept_reads_actual_artifact_bytes_not_claimed_hash(setup, target):
    e = setup
    project, job_id = confirmed(e)
    artifact = next(a for a in e.jobs.jobs[job_id]["result"]["candidate"]["artifacts"]
                    if a["relative_path"].endswith(target))
    path = e.store.project_path(e.pid) / artifact["relative_path"]
    path.write_bytes(path.read_bytes() + b"changed")
    with pytest.raises(StoreValidation, match="实际哈希"):
        e.service.accept(e.pid, project["revision"], "accept", job_id)
    assert e.store.load(e.pid)["candidate"] is None


def test_result_evidence_cannot_claim_different_target_epoch(setup):
    e = setup
    project, job_id = confirmed(e)
    e.jobs.jobs[job_id]["result"]["evidence"]["target_draft_epoch"] += 1
    with pytest.raises(StoreConflict, match="目标不一致"):
        e.service.accept(e.pid, project["revision"], "accept", job_id)


def test_source_bytes_and_runtime_rechecked_at_acceptance(setup, monkeypatch):
    e = setup
    project, job_id = confirmed(e)
    monkeypatch.setattr(editing, "verify_source_snapshot", lambda *a: {"matches": False, "issues": ["changed"]})
    with pytest.raises(StoreReadOnly, match="实际字节"):
        e.service.accept(e.pid, project["revision"], "accept", job_id)
    monkeypatch.setattr(editing, "verify_source_snapshot", lambda *a: {"matches": True, "issues": []})
    e.fingerprints["compiler"] = "e" * 64
    with pytest.raises(StoreConflict, match="过期"):
        e.service.accept(e.pid, project["revision"], "accept", job_id)


def test_new_service_cannot_accept_old_session_jobs(setup):
    e = setup
    project, job_id = confirmed(e)
    restarted = EditingService(e.store, e.jobs, baseline_path=e.baseline)
    with pytest.raises(StoreConflict, match="历史会话"):
        restarted.accept(e.pid, project["revision"], "accept", job_id)


def test_repeated_preview_reuses_job_but_different_edit_cannot_use_its_result(setup):
    e = setup
    first = previewed(e)
    second = e.service.preview(e.pid, 1, e.command, "preview-request")
    assert first == second and len(e.jobs.jobs) == 1
    changed = copy.deepcopy(e.command)
    changed["command_id"] = "different-command"
    committed = e.store.commit(e.pid, 1, changed)
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, committed["revision"], "accept", first["job"]["job_id"])


def test_readonly_and_boolean_revisions_never_start_jobs(setup):
    e = setup
    enabled(e)
    with pytest.raises(StoreValidation):
        e.service.compile(e.pid, True, "bad-version")
    e.store.source_validator = lambda snap: [{"code": "drift"}]
    with pytest.raises(StoreReadOnly):
        e.service.preview(e.pid, 1, e.command, "readonly")
    assert e.jobs.jobs == {}


@pytest.mark.parametrize("bad_id", [None, {}, [], True, "../artifact"])
def test_malformed_transaction_ids_fail_as_contract_errors(setup, bad_id):
    e = setup
    with pytest.raises(StoreValidation):
        e.service.enable(e.pid, 0, bad_id)
    project, job_id = confirmed(e)
    with pytest.raises(StoreValidation):
        e.service.accept(e.pid, project["revision"], bad_id, job_id)
    with pytest.raises(StoreValidation):
        e.service.accept(e.pid, project["revision"], "accept", bad_id)


def test_local_plot_samples_actual_verified_xodr_bytes():
    baseline = compiler.DEFAULT_BASELINE
    selected = compiler.ROOT / "out/workbench/wb04-boundary-probe-20261008/case-02/target-03.xodr"
    if not baseline.is_file() or not selected.is_file():
        pytest.skip("Actual registered WB04 baseline/candidate assets required")
    registration = SimpleNamespace(prepared=SimpleNamespace(knots=compiler.KNOTS))
    before = EditingService._lines(baseline.read_bytes(), registration)
    after = EditingService._lines(selected.read_bytes(), registration)
    assert before["sha256"] == compiler.BASELINE_SHA256
    assert after["sha256"] == "78b893993494b46ee5aa1b094f1fd2ad9f40efdad2d2c6c7943b515a5ea6cd86"
    assert before["lines"][0]["id"] == "fixed_inner"
    # Sampling grids include written cuts, so interpolate-free comparisons use
    # their exact common stations, including the real control station at 92 m.
    def at(plot, line_id, station):
        index = plot["stations_m"].index(station)
        return next(x for x in plot["lines"] if x["id"] == line_id)["points"][index]
    for station in (84., 92., 100.):
        for edge in ("fixed_inner", "fixed_outer"):
            assert at(before, edge, station) == pytest.approx(at(after, edge, station), abs=1e-10)
    a, b = at(before, "shared", 92.), at(after, "shared", 92.)
    assert ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** .5 == pytest.approx(.015, abs=1e-9)
