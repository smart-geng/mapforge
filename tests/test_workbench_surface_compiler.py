"""Surface compiler transaction/binding tests, never a product acceptance run.

The expensive source runner is replaced only in adapter tests. Source snapshot
contracts, draft checks, path/file hashing and independent verification stay real.
The runner's own existing suites test real evaluator/evidence rejection paths.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from mapforge.workbench import surface_compiler as M
from mapforge.workbench.contracts import canonical_bytes, content_hash, digest


@pytest.fixture
def setup(tmp_path, monkeypatch):
    profile = {"name": "synthetic-profile", "sha256": "a" * 64}
    dataset = digest({"files": [], "profile_sha256": profile["sha256"]})
    objects = []
    for index in range(210):
        junction = index == 209
        ref = {"snapshot_id": dataset, "layer": "IBD_OBJECT_INTERSECTION_SURFACE" if junction else "IBD_LANE_BOUNDARY",
               "record_index": 33 if junction else index, "part_index": 0}
        objects.append({"id": "src:" + digest([dataset, ref["layer"], ref["record_index"], 0]),
            "source_ref": ref, "role": "junction" if junction else "boundary",
            "business_id": M.P.JUNCTION_ID if junction else str(index),
            "points": [[0, 0], [1, 0], [1, 1], [0, 0]] if junction else []})
    snapshot = {"schema": "mapforge/workbench-source/v1", "snapshot_id": dataset, "source_files": [],
                "profile": profile, "junction_id": M.P.JUNCTION_ID, "objects": objects,
                "locator": {"source_dir": "not-an-input-path"}}
    snapshot["content_hash"] = M._source_content(snapshot)
    # Distinct synthetic registration; this never authorizes it in production.
    monkeypatch.setattr(M.P, "SNAPSHOT_ID", dataset)
    monkeypatch.setattr(M.P, "SNAPSHOT_CONTENT_HASH", snapshot["content_hash"])
    monkeypatch.setattr(M, "verify_source_snapshot", lambda *a, **kw: {"matches": True, "issues": []})
    fingerprints = {"compiler_hash": "b" * 64, "policy_hash": "c" * 64,
                    "code_files_sha256": {"synthetic.py": "d" * 64},
                    "policy_files_sha256": {}, "runtime": {"python": "3.11.16"}}
    monkeypatch.setattr(M, "compiler_fingerprints", lambda: deepcopy(fingerprints))
    registration = M.register_source(snapshot)
    command = registration.command_template("surface-command")
    draft = {"source_snapshot": snapshot, "revision": 2, "draft_epoch": 1, "intents": [command],
             "content_hash": content_hash(snapshot, [command]), "context": registration.context(snapshot)}
    project = tmp_path / "project"
    project.mkdir()
    payload = {"draft": draft, "project_directory": str(project)}
    return SimpleNamespace(snapshot=snapshot, registration=registration, draft=draft, command=command,
                           payload=payload, project=project, fingerprints=fingerprints)


def rehash(draft):
    draft["content_hash"] = content_hash(draft["source_snapshot"], draft["intents"])


def install_runner(setup, monkeypatch):
    calls = []
    assessment = {"status": "EXPERIMENT_COMPLETE", "experiment_complete": True,
                  "exit_code": 0, "problems": [], "evidence_bindings": {}}

    def run(directory):
        calls.append(directory)
        assert directory.parent.is_dir() and not directory.exists()
        directory.mkdir()
        files = {"source-snapshot.json": canonical_bytes(setup.snapshot),
            "candidate.xodr": b"<OpenDRIVE/>", "source-generated.partial.xodr": b"<OpenDRIVE/>",
            "original-surface.reference.unaccepted.xodr": b"<OpenDRIVE/>",
            "candidate.source-lanes.json": canonical_bytes({"source": "fresh"}),
            "scoreboard.json": canonical_bytes({"rows": [{"tiers": {"T1": {"status": "FAIL"}, "T2": {"status": "FAIL"}}}]}),
            "candidate.delivery-decision.json": canonical_bytes({"status": "BLOCKED"}),
            "runner-result.json": canonical_bytes(assessment),
            "worker.log": b"synthetic runner\n"}
        for name, data in files.items():
            (directory / name).write_bytes(data)
        return deepcopy(assessment)

    def verified(directory, snapshot):
        assert snapshot["content_hash"] == setup.snapshot["content_hash"]
        def read():
            return {p.relative_to(directory).as_posix(): p.read_bytes()
                    for p in directory.rglob("*") if p.is_file()}
        return SimpleNamespace(read_bound=read), deepcopy(assessment), read()

    monkeypatch.setattr(M.runner, "run_experiment", run)
    monkeypatch.setattr(M, "_verified_run", verified)
    return calls


def compile_success(setup, monkeypatch):
    calls = install_runner(setup, monkeypatch)
    result = M.compile_request(setup.payload)
    assert result["status"] == "COMPILED", result
    assert len(calls) == 1
    return result


def rewrite_evidence(setup, candidate, mutation):
    entry = next(item for item in candidate["artifacts"] if item["relative_path"].endswith("/evidence.json"))
    path = setup.project / entry["relative_path"]
    evidence = json.loads(path.read_bytes())
    mutation(evidence)
    path.write_bytes(canonical_bytes(evidence))
    entry["sha256"] = M._file_hash(path)


def test_registration_binds_unique_real_polygon_and_complete_dependency_set(setup):
    registration = setup.registration
    assert registration.source_ref == setup.snapshot["objects"][-1]["id"]
    assert len(registration.feature_ids) == 210
    assert registration.feature_ids == tuple(sorted(x["id"] for x in setup.snapshot["objects"]))
    assert set(registration.store_spec()) == {"capability_id", "type", "source_ref", "feature_ids",
                                            "source_content_hash", "operation_hash"}
    assert registration.operation_hash == digest(M.R.make_intent(setup.snapshot))
    assert registration.capability()["human_confirmation_verified"] is False
    assert registration.capability()["input_complete_xodr_required"] is False
    assert "base_revision" not in registration.command_template("preview")


@pytest.mark.parametrize("change", ["point", "junction", "profile", "remove-object", "duplicate-object", "polygon-role", "record-index"])
def test_self_consistent_but_forged_snapshot_cannot_register(setup, change):
    snapshot = deepcopy(setup.snapshot)
    if change == "point": snapshot["objects"][-1]["points"][0][0] = 9
    elif change == "junction": snapshot["junction_id"] = "other"
    elif change == "profile": snapshot["profile"]["sha256"] = "f" * 64
    elif change == "remove-object": snapshot["objects"].pop()
    elif change == "duplicate-object": snapshot["objects"][0] = deepcopy(snapshot["objects"][1])
    elif change == "polygon-role": snapshot["objects"][-1]["role"] = "annotation"
    else: snapshot["objects"][0]["source_ref"]["record_index"] = 900
    snapshot["content_hash"] = M._source_content(snapshot)
    with pytest.raises(ValueError):
        M.register_source(snapshot)


def test_locator_is_not_permission_to_change_runner_source(setup, tmp_path):
    snapshot = deepcopy(setup.snapshot)
    snapshot["locator"] = {"source_dir": str(tmp_path / "unrelated")}
    assert M.register_source(snapshot).source_dir == str((M.ROOT / "shp_0222-0326").resolve())
    with pytest.raises(M.SurfaceCompilerRejected, match="路径必须"):
        M.register_source(snapshot, source_dir=tmp_path)
    with pytest.raises(M.SurfaceCompilerRejected):
        M.register_source(snapshot, profile_path=tmp_path / "other.yaml")


def test_actual_source_drift_prevents_registration(setup, monkeypatch):
    monkeypatch.setattr(M, "verify_source_snapshot", lambda *a, **kw: {"matches": False})
    with pytest.raises(M.SurfaceCompilerRejected) as error:
        M.register_source(setup.snapshot)
    assert error.value.code == "source-drift"


@pytest.mark.parametrize("change", ["empty", "annotation-only", "duplicate-command", "duplicate-operation", "short-scope", "duplicate-scope",
                                   "wrong-target", "operation-hash", "source-hash", "extra-param", "unsupported-type",
                                   "context", "content", "bool-revision", "negative-epoch", "zero-epoch", "old-candidate"])
def test_invalid_or_unconfirmed_target_never_starts_fresh_runner(setup, monkeypatch, change):
    payload = deepcopy(setup.payload)
    draft = payload["draft"]
    command = draft["intents"][0]
    if change == "empty": draft["intents"] = []
    elif change == "annotation-only":
        draft["intents"] = [{"command_id": "note", "type": "annotation", "source_ref": command["source_ref"],
                            "scope": {"feature_ids": [command["source_ref"]]}, "parameters": {"text": "note", "status": "unresolved"}}]
    elif change == "duplicate-command": draft["intents"].append(deepcopy(command))
    elif change == "duplicate-operation": draft["intents"].append({**deepcopy(command), "command_id": "second"})
    elif change == "short-scope": command["scope"]["feature_ids"].pop()
    elif change == "duplicate-scope": command["scope"]["feature_ids"][0] = command["scope"]["feature_ids"][1]
    elif change == "wrong-target": command["source_ref"] = draft["source_snapshot"]["objects"][0]["id"]
    elif change == "operation-hash": command["parameters"]["operation_hash"] = "f" * 64
    elif change == "source-hash": command["parameters"]["source_content_hash"] = "f" * 64
    elif change == "extra-param": command["parameters"]["padding_m"] = .05
    elif change == "unsupported-type": command["type"] = "shared_boundary_c2_normal_delta"
    elif change == "context": draft["context"]["compiler_hash"] = "f" * 64
    elif change == "bool-revision": draft["revision"] = True
    elif change == "negative-epoch": draft["draft_epoch"] = -1
    elif change == "zero-epoch": draft["draft_epoch"] = 0
    elif change == "old-candidate": payload["candidate"] = "old.xodr"
    rehash(draft)
    if change == "content": draft["content_hash"] = "f" * 64
    monkeypatch.setattr(M.runner, "run_experiment", lambda *a: pytest.fail("runner must not start"))
    result = M.compile_request(payload)
    assert result["status"] == "REJECTED" and result["candidate"] is None
    assert not (setup.project / "artifacts").exists()


def test_fresh_run_outputs_complete_primary_and_partial_evidence_without_acceptance(setup, monkeypatch):
    result = compile_success(setup, monkeypatch)
    candidate = result["candidate"]
    assert set(candidate) == M._CANDIDATE_FIELDS
    assert not result["accepted"] and result["delivery"] == "BLOCKED"
    assert result["evidence"]["geometry_command_ids"] == ["surface-command"]
    assert result["evidence"]["human_confirmation_verified"] is False
    assert len([p for p in candidate["artifacts"] if p["relative_path"].endswith(".xodr")]) == 3
    verified = M.verify_candidate_artifacts(setup.project, candidate)
    assert verified["primary_artifact"] == result["evidence"]["primary_artifact"]
    assert Path(verified["primary_path"]).name == "candidate.xodr"
    assert candidate["checks"][2]["tiers"]["T1"]["status"] == "FAIL"


def test_annotation_changes_are_preserved_in_target_but_not_geometry_commands(setup, monkeypatch):
    ref = setup.command["source_ref"]
    setup.draft["intents"].append({"command_id": "note", "type": "annotation", "source_ref": ref,
        "scope": {"feature_ids": [ref]}, "parameters": {"text": "保留源空区", "status": "unresolved"}})
    rehash(setup.draft)
    result = compile_success(setup, monkeypatch)
    assert len(result["evidence"]["draft"]["intents"]) == 2
    assert result["evidence"]["geometry_command_ids"] == ["surface-command"]


def test_repeated_evaluation_creates_new_run_without_overwriting_old_evidence(setup, monkeypatch):
    calls = install_runner(setup, monkeypatch)
    first = M.compile_request(setup.payload)
    first_path = Path(M.verify_candidate_artifacts(setup.project, first["candidate"])["primary_path"])
    before = first_path.read_bytes()
    second = M.compile_request(setup.payload)
    assert first["status"] == second["status"] == "COMPILED"
    assert len(calls) == 2 and calls[0] != calls[1]
    assert first["candidate"]["candidate_id"] != second["candidate"]["candidate_id"]
    assert first_path.read_bytes() == before


@pytest.mark.parametrize("change", ["manifest-removed", "manifest-duplicate", "byte-change", "missing-file", "extra-run-file",
                                   "escape", "wrong-primary", "false-score", "missing-commands", "wrong-candidate-id",
                                   "outer-context", "false-fresh", "true-input-xodr", "true-source-change", "false-validation", "human-claimed"])
def test_independent_verification_rejects_forgery_even_when_evidence_hash_is_updated(setup, monkeypatch, change):
    result = compile_success(setup, monkeypatch)
    candidate = deepcopy(result["candidate"])
    verified = M.verify_candidate_artifacts(setup.project, candidate)
    if change == "manifest-removed": candidate["artifacts"].pop(0)
    elif change == "manifest-duplicate": candidate["artifacts"].append(deepcopy(candidate["artifacts"][0]))
    elif change == "byte-change": Path(verified["primary_path"]).write_bytes(b"changed")
    elif change == "missing-file": (Path(verified["run_directory"]) / "candidate.source-lanes.json").unlink()
    elif change == "extra-run-file": (Path(verified["run_directory"]) / "extra.json").write_bytes(b"{}")
    elif change == "escape": candidate["artifacts"][0]["relative_path"] = "artifacts/../secret"
    elif change == "wrong-primary": rewrite_evidence(setup, candidate, lambda e: e.update(primary_artifact=e["primary_artifact"].replace("candidate.xodr", "source-generated.partial.xodr")))
    elif change == "false-score": candidate["checks"][2]["tiers"]["T1"]["status"] = "PASS"
    elif change == "missing-commands": rewrite_evidence(setup, candidate, lambda e: e.update(geometry_command_ids=[]))
    elif change == "wrong-candidate-id":
        candidate["candidate_id"] = "surface-" + "f" * 32
        rewrite_evidence(setup, candidate, lambda e: e.update(candidate_id=candidate["candidate_id"]))
    elif change == "outer-context":
        candidate["context"]["compiler_hash"] = "e" * 64
        rewrite_evidence(setup, candidate, lambda e: e.update(context=candidate["context"]))
    elif change == "false-fresh": rewrite_evidence(setup, candidate, lambda e: e.update(fresh_source_generation=False))
    elif change == "true-input-xodr": rewrite_evidence(setup, candidate, lambda e: e.update(input_complete_xodr=True))
    elif change == "true-source-change": rewrite_evidence(setup, candidate, lambda e: e.update(source_originals_changed=True))
    elif change == "false-validation": rewrite_evidence(setup, candidate, lambda e: e.update(whole_map_validation="NOT_RUN"))
    else: rewrite_evidence(setup, candidate, lambda e: e.update(human_confirmation_verified=True))
    with pytest.raises(ValueError):
        M.verify_candidate_artifacts(setup.project, candidate)


@pytest.mark.parametrize("metadata,passes", [({"accepted_revision": 3, "accepted_epoch": 1}, True),
    ({"accepted_revision": True, "accepted_epoch": 1}, False),
    ({"accepted_revision": 2, "accepted_epoch": 1}, False),
    ({"accepted_revision": 3, "accepted_epoch": 2}, False),
    ({"accepted_revision": 3}, False), ({"accepted_revision": -1, "accepted_epoch": 1}, False)])
def test_persisted_acceptance_metadata_is_supported_but_not_inferred_as_quality(setup, monkeypatch, metadata, passes):
    result = compile_success(setup, monkeypatch)
    candidate = {**result["candidate"], **metadata}
    if passes:
        assert M.verify_candidate_artifacts(setup.project, candidate)["evidence"]["delivery"] == "BLOCKED"
    else:
        with pytest.raises(ValueError):
            M.verify_candidate_artifacts(setup.project, candidate)


def test_failed_fresh_execution_retains_run_and_has_no_candidate(setup, monkeypatch):
    def fail(directory):
        directory.mkdir()
        (directory / "report.json").write_bytes(b'{"status":"FAILED"}')
        return {"status": "CHILD_FAILED", "experiment_complete": False, "exit_code": 1, "problems": ["failed"]}
    monkeypatch.setattr(M.runner, "run_experiment", fail)
    result = M.compile_request(setup.payload)
    assert result["status"] == "REJECTED" and result["candidate"] is None
    assert (Path(result["retained_run_directory"]) / "report.json").exists()


def test_source_or_code_drift_after_fresh_run_refuses_publication(setup, monkeypatch):
    install_runner(setup, monkeypatch)
    original = M.runner.run_experiment
    def change(directory):
        result = original(directory)
        setup.fingerprints["compiler_hash"] = "9" * 64
        return result
    monkeypatch.setattr(M.runner, "run_experiment", change)
    result = M.compile_request(setup.payload)
    assert result["status"] == "REJECTED" and result["candidate"] is None
    assert result["error"]["code"] == "inputs-changed-during-compile"


def test_nonexistent_or_linked_project_cannot_publish(setup, monkeypatch):
    monkeypatch.setattr(M.runner, "run_experiment", lambda *a: pytest.fail("no worker"))
    setup.payload["project_directory"] = str(setup.project / "missing")
    result = M.compile_request(setup.payload)
    assert result["status"] == "REJECTED" and result["error"]["code"] == "invalid-project-directory"


@pytest.mark.parametrize("failure", ["source", "dependencies", "runner-code", "assessment", "diagnosis", "receipt-bindings"])
def test_strict_run_verifier_does_not_trust_exit_zero_or_report_labels(setup, monkeypatch, tmp_path, failure):
    out = tmp_path / "run"
    out.mkdir()
    dependencies = {name: {"path": name, "sha256": "e" * 64, "size": 1} for name in M.runner.EXTRA_DEPENDENCIES}
    file_binding = {"sha256": "f" * 64, "size": 1}
    receipt = {"extra_dependency_integrity_after": {"matches": True, "issues": [], "bindings": dependencies},
               "binding": {"extra_dependencies": dependencies, "runner": file_binding, "worker": file_binding}}
    assessment = {"status": "EXPERIMENT_COMPLETE", "experiment_complete": True, "exit_code": 0,
                  "problems": [], "evidence_bindings": {"candidate.xodr": {"sha256": "a" * 64}}}
    review = SimpleNamespace(matches=lambda snapshot: failure != "source",
        read_bound=lambda: {"runner-result.json": canonical_bytes(receipt)},
        diagnose=lambda: {"available": failure != "diagnosis"},
        bindings={"candidate.xodr": {"sha256": "a" * 64}})
    if failure == "dependencies": receipt["extra_dependency_integrity_after"]["matches"] = False
    elif failure == "runner-code": receipt["binding"]["runner"] = {"sha256": "9" * 64, "size": 1}
    elif failure == "assessment": assessment["status"] = "INCOMPLETE_EVIDENCE"
    elif failure == "receipt-bindings": review.bindings["candidate.xodr"]["sha256"] = "b" * 64
    monkeypatch.setattr(M, "RegisteredSurfaceReview", lambda _: review)
    monkeypatch.setattr(M.runner, "_extra_dependency_binding", lambda name: dependencies[name])
    monkeypatch.setattr(M.runner, "_file_binding", lambda path: file_binding)
    monkeypatch.setattr(M.runner, "assess_output", lambda *a, **kw: assessment)
    with pytest.raises(M.SurfaceCompilerRejected):
        M._verified_run(out, setup.snapshot)
