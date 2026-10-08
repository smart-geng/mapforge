"""Real Store/EditingService timing with synthetic workers and actual files.

Compiler geometry/provenance verification is tested separately. Here the costly
registered compiler and diagnosis are isolated to exercise session ownership,
immutable result binding and user-command ordering through the real Store.
"""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from mapforge.workbench import editing
from mapforge.workbench import surface_editing as S
from mapforge.workbench.contracts import (StoreConflict, StoreReadOnly, StoreValidation,
                                         canonical_bytes, context_for, digest)
from mapforge.workbench.store import ProjectStore


class CompletedJobs:
    def __init__(self):
        self.jobs, self.payloads = {}, {}

    def start(self, project, operation, payload, request_id):
        assert operation == "compile_surface" and "baseline_path" not in payload
        job_id = hashlib.sha256(request_id.encode()).hexdigest()[:32]
        if job_id in self.jobs:
            assert self.payloads[job_id] == payload
            return copy.deepcopy(self.jobs[job_id])
        self.payloads[job_id] = copy.deepcopy(payload)
        draft = payload["draft"]
        prefix = "artifacts/" + job_id
        data = b"actual synthetic candidate"
        evidence = {"candidate_id": "surface-" + job_id, "candidate_sha256": hashlib.sha256(data).hexdigest(),
            "primary_artifact": prefix + "/run/candidate.xodr", "target_revision": draft["revision"],
            "target_draft_epoch": draft["draft_epoch"], "target_content_hash": draft["content_hash"],
            "context": copy.deepcopy(draft["context"]), "accepted": False, "delivery": "BLOCKED"}
        candidate = {"candidate_id": evidence["candidate_id"], "target_content_hash": draft["content_hash"],
            "context": copy.deepcopy(draft["context"]), "artifacts": [],
            "affected_ids": [obj["id"] for obj in draft["source_snapshot"]["objects"]],
            "checks": [{"gate": "production-delivery", "status": "BLOCKED"}]}
        for relative, raw in ((evidence["primary_artifact"], data),
                              (prefix + "/run/source-generated.partial.xodr", b"failed partial"),
                              (prefix + "/evidence.json", canonical_bytes(evidence))):
            path = Path(payload["project_directory"]) / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
            candidate["artifacts"].append({"relative_path": relative, "sha256": hashlib.sha256(raw).hexdigest()})
        record = {"job_id": job_id, "project_id": project["project_id"], "operation": operation,
            "base_revision": project["revision"], "content_hash": project["content_hash"],
            "state": "succeeded", "historical": False,
            "archive": {"state": "persisted", "recorded_state": "succeeded"},
            "result": {"status": "COMPILED", "candidate": candidate, "evidence": evidence}}
        self.jobs[job_id] = record
        return copy.deepcopy(record)

    def get(self, job_id, *, project_id, **kwargs):
        if self.jobs[job_id]["project_id"] != project_id:
            raise KeyError("wrong project")
        return copy.deepcopy(self.jobs[job_id])


@pytest.fixture
def setup(tmp_path, monkeypatch):
    snapshot = {"snapshot_id": "synthetic-surface-dataset", "junction_id": S.compiler.P.JUNCTION_ID,
        "profile": {"sha256": "a" * 64}, "source_files": [],
        "objects": [{"id": name, "source_ref": {"snapshot_id": "synthetic-surface-dataset"}}
                    for name in ("junction", "lane", "boundary")],
        "locator": {"source_dir": "untrusted-client-source"}}
    snapshot["content_hash"] = digest({k: v for k, v in snapshot.items() if k != "locator"})
    fingerprints = {"compiler": "b" * 64, "policy": "c" * 64}
    features = sorted(obj["id"] for obj in snapshot["objects"])
    spec = {"capability_id": S.compiler.CAPABILITY_ID, "type": S.compiler.INTENT_TYPE,
        "source_ref": "junction", "feature_ids": features,
        "source_content_hash": snapshot["content_hash"], "operation_hash": "d" * 64}
    def command(cid):
        return {"command_id": cid, "type": S.compiler.INTENT_TYPE, "source_ref": "junction",
            "scope": {"capability_id": S.compiler.CAPABILITY_ID, "feature_ids": features},
            "parameters": {k: spec[k] for k in ("source_content_hash", "operation_hash")}}
    registration = SimpleNamespace(snapshot_content_hash=snapshot["content_hash"],
        context=lambda snap: context_for(snap, fingerprints["compiler"], fingerprints["policy"]),
        store_spec=lambda: copy.deepcopy(spec), command_template=command,
        capability=lambda: {**copy.deepcopy(spec), "actions": ["withdraw", "complete", "represent"]})
    calls, verifier_calls, diagnosis_calls = [], [], []
    drift = {"source": False}
    def register(snap, **kwargs):
        calls.append(kwargs)
        if drift["source"]:
            raise S.compiler.SurfaceCompilerRejected("source-drift", "source changed")
        return registration
    monkeypatch.setattr(S.compiler, "register_source", register)
    monkeypatch.setattr(editing, "verify_source_snapshot", lambda *a: {"matches": not drift["source"]})
    def verify(directory, candidate):
        verifier_calls.append(copy.deepcopy(candidate))
        paths = {}
        for item in candidate["artifacts"]:
            path = Path(directory) / item["relative_path"]
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != item["sha256"]:
                raise ValueError("artifact changed")
            paths[item["relative_path"]] = path
        proof = next(path for path in paths.values() if path.name == "evidence.json")
        evidence = json.loads(proof.read_bytes())
        primary = paths[evidence["primary_artifact"]]
        return {"primary_path": str(primary), "evidence": evidence, "run_directory": str(primary.parent)}
    monkeypatch.setattr(S.compiler, "verify_candidate_artifacts", verify)
    def review(directory):
        diagnosis_calls.append(directory)
        report = {"schema": "mapforge/surface-diagnostics/v1", "status": "DIAGNOSED",
            "candidate_accepted": False, "issues": [], "layers": [{"id": "actual", "geometry": {"type": "Polygon", "coordinates": []}}],
            "context": {"candidate_accepted": False, "meaning": "old external context",
                        "frame": {"kind": "local-eqc", "unit": "m", "absolute_crs_status": "unverified"}}}
        return SimpleNamespace(diagnose=lambda: {"available": True, "report": copy.deepcopy(report)})
    monkeypatch.setattr(S, "RegisteredSurfaceReview", review)
    store, jobs = ProjectStore(tmp_path / "projects"), CompletedJobs()
    service = S.SurfaceEditingService(store, jobs, source_dir=tmp_path / "source", profile_path=tmp_path / "profile.yaml")
    project = store.create(snapshot)
    return SimpleNamespace(store=store, jobs=jobs, service=service, project=project, pid=project["project_id"],
        command=command("surface-command"), registration=registration, calls=calls, verifier_calls=verifier_calls,
        diagnosis_calls=diagnosis_calls, drift=drift, fingerprints=fingerprints)


def enable(env):
    return env.service.enable(env.pid, 0, "enable")["project"]


def preview(env):
    project = enable(env)
    return env.service.preview(env.pid, project["revision"], env.command, "preview")


def confirm(env):
    result = preview(env)
    project = env.store.commit(env.pid, 1, result["preview"]["command"])
    return project, result["job"]["job_id"]


def accept(env):
    project, job_id = confirm(env)
    return env.service.accept(env.pid, project["revision"], "accept", job_id)


def test_descriptor_template_is_readonly_cached_and_source_scoped(setup):
    e = setup
    before = (e.store.project_path(e.pid) / "project.json").read_bytes()
    result = e.service.describe(e.project)
    assert result["supported"] and not result["enabled"]
    assert "command_id" not in result["command_template"]
    assert result["command_template"]["scope"]["feature_ids"] == ["boundary", "junction", "lane"]
    assert e.service.describe(e.project) == result and len(e.calls) == 1
    assert "untrusted-client-source" not in str(e.calls)
    assert (e.store.project_path(e.pid) / "project.json").read_bytes() == before
    unsupported = copy.deepcopy(e.project)
    unsupported["source_snapshot"]["junction_id"] = "other"
    assert not e.service.describe(unsupported)["supported"]
    assert len(e.calls) == 1


def test_empty_draft_has_no_complete_baseline_candidate_or_compile_job(setup):
    project = enable(setup)
    descriptor = setup.service.describe(project)
    assert descriptor["can_preview"] and not descriptor["can_compile"]
    assert not setup.service.geometry(setup.pid)["available"]
    assert not setup.diagnosis_calls
    with pytest.raises(StoreValidation):
        setup.service.compile(setup.pid, project["revision"], "empty")
    assert setup.jobs.jobs == {}


def test_preview_does_not_persist_and_accept_requires_exact_command_commit(setup):
    e = setup
    result = preview(e)
    job_id = result["job"]["job_id"]
    unchanged = e.store.load(e.pid)
    assert unchanged["intents"] == [] and unchanged["candidate"] is None
    assert result["job"]["operation"] == "compile_surface"
    assert "baseline_path" not in e.jobs.payloads[job_id]
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, 1, "accept", job_id)
    view = e.service.geometry(e.pid, job_id)
    assert view["available"] and view["preview"] and not view["candidate_accepted"]
    assert "本工程预览目标" in view["report"]["context"]["meaning"]
    committed = e.store.commit(e.pid, 1, result["preview"]["command"])
    accepted = e.service.accept(e.pid, committed["revision"], "accept", job_id)
    assert accepted["revision"] == 3 and not accepted["status"]["candidate_stale"]
    assert not accepted["status"]["formal_export_available"]
    assert e.calls[-1]["source_dir"] == e.service.source_dir


def test_saved_operation_cannot_be_added_twice_and_can_be_recompiled(setup):
    project, _ = confirm(setup)
    descriptor = setup.service.describe(project)
    assert not descriptor["can_preview"] and descriptor["can_compile"]
    with pytest.raises(StoreConflict, match="重复追加"):
        setup.service.preview(setup.pid, 2, {**setup.command, "command_id": "again"}, "duplicate")
    job = setup.service.compile(setup.pid, 2, "saved")["job"]
    accepted = setup.service.accept(setup.pid, 2, "accept-saved", job["job_id"])
    assert len(accepted["intents"]) == 1


def test_saved_draft_does_not_hide_outdated_registration_reason(setup):
    project, _ = confirm(setup)
    setup.fingerprints["compiler"] = "e" * 64
    descriptor = setup.service.describe(project)
    assert not descriptor["enabled"] and not descriptor["can_compile"]
    assert "过期" in descriptor["reason"] and "可重新编译" not in descriptor["reason"]


def test_accepted_candidate_geometry_survives_new_service_but_historical_job_does_not(setup):
    e = setup
    project = accept(e)
    e.service = S.SurfaceEditingService(e.store, e.jobs, source_dir=e.service.source_dir, profile_path=e.service.profile_path)
    view = e.service.geometry(e.pid)
    assert view["available"] and view["candidate_accepted"] and not view["preview"]
    assert view["report"]["context"]["candidate_accepted"] is True
    assert view["report"]["candidate_accepted"] is False  # immutable local proof, not Store action
    assert view["report"]["context"]["project_id"] == e.pid
    assert "external" not in view["report"]["context"]["meaning"]
    old_job = next(iter(e.jobs.jobs))
    assert not e.service.geometry(e.pid, old_job)["available"]
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, project["revision"], "old-again", old_job)


@pytest.mark.parametrize("change", ["cancelled", "running", "failed", "timed_out", "historical", "archive-failed", "archive-old", "wrong-operation"])
def test_cancelled_or_unowned_results_never_render_or_accept(setup, change):
    project, job_id = confirm(setup)
    job = setup.jobs.jobs[job_id]
    if change == "historical": job["historical"] = True
    elif change == "archive-failed": job["archive"]["state"] = "failed"
    elif change == "archive-old": job["archive"]["recorded_state"] = "running"
    elif change == "wrong-operation": job["operation"] = "compile"
    else: job["state"] = change
    assert not setup.service.geometry(setup.pid, job_id)["available"]
    with pytest.raises(StoreConflict):
        setup.service.accept(setup.pid, project["revision"], "accept", job_id)
    assert setup.store.load(setup.pid)["candidate"] is None


@pytest.mark.parametrize("change", ["different-command", "annotation", "undo-redo", "context"])
def test_late_preview_cannot_accept_after_changed_transaction_even_same_content(setup, change):
    e = setup
    result = preview(e)
    if change == "different-command":
        project = e.store.commit(e.pid, 1, {**e.command, "command_id": "different-confirmation"})
    else:
        project = e.store.commit(e.pid, 1, result["preview"]["command"])
        if change == "annotation":
            project = e.store.commit(e.pid, 2, {"command_id": "note", "type": "annotation", "source_ref": "junction",
                "scope": {"feature_ids": ["junction"]}, "parameters": {"text": "changed"}})
        elif change == "undo-redo":
            expected = project["content_hash"]
            project = e.store.undo(e.pid, 2, "undo")
            project = e.store.redo(e.pid, 3, "redo")
            assert project["content_hash"] == expected
        else:
            project = e.store.set_context(e.pid, 2, "changed-context", "e" * 64, "c" * 64)
    with pytest.raises(StoreConflict):
        e.service.accept(e.pid, project["revision"], "accept", result["job"]["job_id"])


def test_idempotent_same_accept_request_does_not_create_another_revision(setup):
    project, job_id = confirm(setup)
    result = setup.service.accept(setup.pid, project["revision"], "accept", job_id)
    assert setup.service.accept(setup.pid, project["revision"], "accept", job_id) == result
    with pytest.raises(StoreConflict):
        setup.service.accept(setup.pid, result["revision"], "accept-again", job_id)


@pytest.mark.parametrize("change", ["artifact", "source", "compiler", "result-evidence"])
def test_changed_artifacts_source_or_context_refuse_acceptance(setup, change):
    project, job_id = confirm(setup)
    if change == "artifact":
        candidate = setup.jobs.jobs[job_id]["result"]["candidate"]
        (setup.store.project_path(setup.pid) / candidate["artifacts"][0]["relative_path"]).write_bytes(b"changed")
    elif change == "source": setup.drift["source"] = True
    elif change == "compiler": setup.fingerprints["compiler"] = "f" * 64
    else: setup.jobs.jobs[job_id]["result"]["evidence"]["candidate_sha256"] = "f" * 64
    with pytest.raises((StoreConflict, StoreReadOnly, StoreValidation)):
        setup.service.accept(setup.pid, project["revision"], "accept", job_id)
    assert setup.store.load(setup.pid)["candidate"] is None


def test_stale_accepted_geometry_is_cleared_after_undo_redo(setup):
    project = accept(setup)
    old_hash = project["content_hash"]
    project = setup.store.undo(setup.pid, project["revision"], "undo")
    assert not setup.service.geometry(setup.pid)["available"]
    project = setup.store.redo(setup.pid, project["revision"], "redo")
    assert project["content_hash"] == old_hash
    view = setup.service.geometry(setup.pid)
    assert not view["available"] and view["candidate_stale"] and "report" not in view


def test_source_readonly_clears_previously_accepted_geometry(setup):
    accept(setup)
    setup.store.source_validator = lambda snapshot: [{"code": "source-drift"}]
    view = setup.service.geometry(setup.pid)
    assert not view["available"] and "report" not in view


def test_geometry_rechecks_project_after_diagnosis_and_discards_racing_result(setup, monkeypatch):
    project = accept(setup)
    def review(directory):
        def diagnose():
            setup.store.undo(setup.pid, project["revision"], "racing-undo")
            return {"available": True, "report": {"layers": [], "issues": [], "context": {}}}
        return SimpleNamespace(diagnose=diagnose)
    monkeypatch.setattr(S, "RegisteredSurfaceReview", review)
    assert not setup.service.geometry(setup.pid)["available"]


def test_source_registration_failure_never_displays_external_review(setup, monkeypatch):
    project = accept(setup)
    setup.drift["source"] = True
    view = setup.service.geometry(setup.pid)
    assert not view["available"] and "report" not in view


def test_invalid_or_annotation_only_preview_is_rejected_without_job(setup):
    project = enable(setup)
    with pytest.raises(StoreValidation):
        setup.service.preview(setup.pid, project["revision"], {"type": "annotation"}, "note")
    assert not setup.jobs.jobs
