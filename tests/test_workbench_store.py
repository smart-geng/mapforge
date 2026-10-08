"""Persistence guarantees for confirmed source-workspace notes, not geometry acceptance."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from mapforge.workbench.contracts import digest
from mapforge.workbench.store import (
    ProjectStore, StoreConflict, StoreReadOnly, StoreValidation,
)


@pytest.fixture
def snapshot():
    return {
        "schema": "source-v1", "snapshot_id": "dataset-1", "junction_id": "j1",
        "source_files": [{"relative_path": "lane.shp", "size": 1, "sha256": "1" * 64}],
        "profile": {"id": "profile-1", "sha256": "2" * 64},
        "frame": {"units": "unknown", "absolute_crs_status": "unverified"},
        "objects": [
            {"id": "obj-0", "source_ref": {"snapshot_id": "dataset-1", "layer": "lane",
                                             "record_index": 0, "part_index": 0},
             "raw_attributes": {"PID": "duplicate", "unknown": [1, 2]}, "points": [[1, 2], [3, 4]]},
            {"id": "obj-1", "source_ref": {"snapshot_id": "dataset-1", "layer": "lane",
                                             "record_index": 1, "part_index": 0},
             "raw_attributes": {"PID": "duplicate"}, "points": [[3, 4], [5, 6]]},
        ],
        "issues": [{"code": "conversion-failed", "message": "no XODR"}],
        "capabilities": {"annotation": True, "geometry_edit": False},
    }


@pytest.fixture
def store(tmp_path):
    return ProjectStore(tmp_path / "projects")


def note(command_id="note-1", text="核对缺失边界", **kwargs):
    return {"command_id": command_id, "type": "annotation", "source_ref": "obj-0",
            "scope": {"feature_ids": ["obj-0"]},
            "parameters": {"text": text, "status": "unresolved"}, **kwargs}


def test_failed_source_and_unknown_fields_round_trip_without_compile(store, snapshot):
    project = store.create(snapshot, "未转出的源工程")
    result = store.commit(project["project_id"], 0, note())
    reopened = ProjectStore(store.root).load(project["project_id"])
    assert reopened["source_snapshot"] == snapshot
    assert reopened["intents"][0]["parameters"]["status"] == "unresolved"
    assert reopened["content_hash"] == result["content_hash"]
    assert reopened["candidate"] is None
    assert not reopened["status"]["formal_export_available"]
    assert len(store.list()) == 1
    snapshot["objects"].clear()
    assert len(store.load(project["project_id"])["source_snapshot"]["objects"]) == 2


def test_cancelled_preview_has_no_write_or_hidden_history(store, snapshot):
    project = store.create(snapshot)
    path = store.project_path(project["project_id"]) / "project.json"
    original = path.read_bytes()
    preview = store.preview(project["project_id"], 0, note())
    assert preview["compiled"] is False
    assert path.read_bytes() == original
    assert not path.with_name("previous.json").exists()
    committed = store.commit(project["project_id"], 0, note())
    assert committed["content_hash"] == preview["target_content_hash"]
    assert len(committed["journal"]) == 1


def test_duplicate_submission_is_idempotent_but_reusing_id_is_not(store, snapshot):
    project = store.create(snapshot)
    first = store.commit(project["project_id"], 0, note())
    assert store.commit(project["project_id"], 0, note()) == first
    with pytest.raises(StoreConflict, match="同一 command_id"):
        store.commit(project["project_id"], 0, note(text="另一个请求"))
    assert store.load(project["project_id"])["revision"] == 1


def test_multi_window_conflict_preserves_first_confirmed_note(store, snapshot):
    project = store.create(snapshot)
    second = ProjectStore(store.root)
    original = second.load(project["project_id"])
    store.commit(project["project_id"], 0, note())
    with pytest.raises(StoreConflict, match="工程已更新"):
        second.commit(project["project_id"], original["revision"], note("note-2"))
    assert len(second.load(project["project_id"])["intents"]) == 1


def test_undo_redo_hash_and_new_branch_with_monotonic_revision(store, snapshot):
    base = store.create(snapshot)
    pid = base["project_id"]
    a = store.commit(pid, 0, note())
    b = store.undo(pid, 1, "undo-1")
    assert b["content_hash"] == base["content_hash"]
    assert b["revision"] > a["revision"]
    assert b["status"]["can_redo"]
    c = store.redo(pid, 2, "redo-1")
    assert c["content_hash"] == a["content_hash"]
    assert store.redo(pid, 2, "redo-1") == c
    d = store.undo(pid, 3, "undo-2")
    e = store.commit(pid, d["revision"], note("branch-note"))
    assert not e["status"]["can_redo"]
    assert e["intents"][0]["command_id"] == "branch-note"
    with pytest.raises(StoreConflict):
        store.redo(pid, e["revision"], "redo-branch")


@pytest.mark.parametrize("change", [
    {"type": "boundary_offset"},
    {"type": "python", "expression": "open('source','w')"},
    {"source_ref": "other-source-object"},
    {"scope": {"feature_ids": ["obj-1"]}},
    {"scope": {"feature_ids": ["obj-0", "obj-0"]}},
    {"scope": {"feature_ids": ["obj-0", "other-object"]}},
    {"parameters": {"text": "", "status": "unresolved"}},
    {"parameters": {"text": "repair", "status": "compiled"}},
    {"parameters": {"text": "repair", "offset_m": float("nan")}},
    {"parameters": {"text": "repair", "offset_m": float("inf")}},
    {"command_id": "../escape"},
])
def test_invalid_intents_rejected_without_any_transaction(store, snapshot, change):
    project = store.create(snapshot)
    pid = project["project_id"]
    command = note()
    command.update(change)
    with pytest.raises(StoreValidation):
        store.commit(pid, 0, command)
    assert store.load(pid) == project


def test_same_dataset_different_junction_has_different_content_hash(store, snapshot):
    other = copy.deepcopy(snapshot)
    other["junction_id"] = "j2"
    a, b = store.create(snapshot), store.create(other)
    assert a["content_hash"] != b["content_hash"]
    assert a["context"]["source_hash"] != b["context"]["source_hash"]


def test_source_relocation_excludes_only_top_level_locator(store, snapshot):
    snapshot["locator"] = {"source_dir": "F:/source", "profile_path": "F:/profile.yaml"}
    snapshot["objects"][0]["raw_attributes"]["path"] = "original business value"
    moved = copy.deepcopy(snapshot)
    moved["locator"] = {"source_dir": "D:/source", "profile_path": "D:/profile.yaml"}
    first, second = store.create(snapshot), store.create(moved)
    assert first["content_hash"] == second["content_hash"]
    assert first["context"] == second["context"]
    moved["objects"][0]["raw_attributes"]["path"] = "different business value"
    assert store.create(moved)["content_hash"] != first["content_hash"]


def geometry_spec(**changes):
    return {"capability_id": "server-boundary-v1", "type": "shared_boundary_c2_normal_delta",
            "source_ref": "obj-0", "feature_ids": ["obj-1", "obj-0"],
            "baseline_sha256": "c" * 64, "normal_delta_m": {"min": -.1, "max": .1}, **changes}


def geometry_command(command_id="move-1", delta=-.015):
    return {"command_id": command_id, "type": "shared_boundary_c2_normal_delta", "source_ref": "obj-0",
            "scope": {"capability_id": "server-boundary-v1", "feature_ids": ["obj-0", "obj-1"]},
            "parameters": {"normal_delta_m": delta, "baseline_sha256": "c" * 64}}


def registered_geometry(store, snapshot):
    project = store.create(snapshot)
    return store.register_capabilities(project["project_id"], 0, "verified-server-registration",
                                       [geometry_spec()], "a" * 64, "b" * 64)


def test_geometry_draft_saves_without_compiler_and_undo_redo_reopens(store, snapshot, monkeypatch):
    # Structural persistence must not attempt a geometry import or computation.
    monkeypatch.setitem(sys.modules, "mapforge.workbench.compiler", None)
    project = registered_geometry(store, snapshot)
    pid = project["project_id"]
    intent = geometry_command(delta=.03791)  # continuous value, not a preset
    preview = store.preview(pid, project["revision"], intent)
    assert store.load(pid)["intents"] == []
    saved = store.commit(pid, project["revision"], intent)
    assert saved["content_hash"] == preview["target_content_hash"]
    assert saved["intents"][0]["parameters"]["normal_delta_m"] == .03791
    assert saved["candidate"] is None
    assert ProjectStore(store.root).load(pid)["intents"] == saved["intents"]
    undone = store.undo(pid, saved["revision"], "undo-move")
    assert undone["intents"] == []
    redone = store.redo(pid, undone["revision"], "redo-move")
    assert redone["content_hash"] == saved["content_hash"]
    assert ProjectStore(store.root).load(pid)["capabilities"] == project["capabilities"]
    assert not redone["status"]["formal_export_available"]


@pytest.mark.parametrize("delta", [-.1, 0, .1, -.015, .03791])
def test_registered_geometry_range_is_continuous_and_inclusive(store, snapshot, delta):
    project = registered_geometry(store, snapshot)
    result = store.commit(project["project_id"], project["revision"], geometry_command(delta=delta))
    assert result["intents"][0]["parameters"]["normal_delta_m"] == delta


@pytest.mark.parametrize("delta", [-.10001, .10001, float("nan"), float("inf"), True, "0.02", None])
def test_invalid_geometry_values_cannot_enter_draft(store, snapshot, delta):
    project = registered_geometry(store, snapshot)
    with pytest.raises(StoreValidation):
        store.commit(project["project_id"], project["revision"], geometry_command(delta=delta))
    assert store.load(project["project_id"]) == project


def test_geometry_needs_registered_capability_not_source_snapshot_capability_flag(store, snapshot):
    snapshot["capabilities"] = {"geometry_edit": True, "capability_id": "server-boundary-v1"}
    project = store.create(snapshot)
    with pytest.raises(StoreValidation, match="没有服务器登记"):
        store.commit(project["project_id"], 0, geometry_command())
    assert store.load(project["project_id"])["intents"] == []


@pytest.mark.parametrize("mutate", [
    lambda c: c["scope"].update(capability_id="unregistered-id"),
    lambda c: c["scope"].update(feature_ids=["obj-0"]),
    lambda c: c.update(source_ref="obj-1"),
    lambda c: c["parameters"].update(knots=[1, 2, 3]),
    lambda c: c["parameters"].update(station_m=5),
])
def test_geometry_cannot_change_registered_scope_or_server_fixed_controls(store, snapshot, mutate):
    project = registered_geometry(store, snapshot)
    intent = geometry_command()
    mutate(intent)
    with pytest.raises(StoreValidation):
        store.commit(project["project_id"], project["revision"], intent)
    assert store.load(project["project_id"])["intents"] == []


def test_stale_baseline_or_compiler_context_rejected_but_existing_draft_preserved(store, snapshot):
    project = registered_geometry(store, snapshot)
    pid = project["project_id"]
    stale = geometry_command()
    stale["parameters"]["baseline_sha256"] = "d" * 64
    with pytest.raises(StoreConflict, match="基线"):
        store.commit(pid, project["revision"], stale)
    saved = store.commit(pid, project["revision"], geometry_command())
    changed = store.set_context(pid, saved["revision"], "new-compiler", "d" * 64, "b" * 64)
    with pytest.raises(StoreConflict, match="过期"):
        store.commit(pid, changed["revision"], geometry_command("after-context-change"))
    assert store.load(pid)["intents"] == saved["intents"]
    refreshed = store.register_capabilities(pid, changed["revision"], "verified-again", [geometry_spec()],
                                            "d" * 64, "b" * 64)
    result = store.commit(pid, refreshed["revision"], geometry_command("new-intent", .023))
    assert len(result["intents"]) == 2  # absolute baseline intent semantics belong to the compiler


def test_capability_ids_cannot_silently_rebind_and_registration_is_atomic(store, snapshot):
    project = registered_geometry(store, snapshot)
    pid = project["project_id"]
    altered = geometry_spec(baseline_sha256="d" * 64)
    with pytest.raises(StoreConflict, match="不得原地改变"):
        store.register_capabilities(pid, project["revision"], "rebind", [altered], "a" * 64, "b" * 64)
    invalid = geometry_spec(capability_id="missing-scope", feature_ids=["obj-0", "foreign-object"])
    with pytest.raises(StoreValidation):
        store.register_capabilities(pid, project["revision"], "invalid-registration", [invalid], "a" * 64, "b" * 64)
    assert store.load(pid) == project
    # A genuinely new server capability can coexist, using a new immutable ID.
    second = geometry_spec(capability_id="server-boundary-v2", baseline_sha256="d" * 64)
    changed = store.register_capabilities(pid, project["revision"], "new-capability", [second], "a" * 64, "b" * 64)
    assert set(changed["capabilities"]) == {"server-boundary-v1", "server-boundary-v2"}


def test_duplicate_raw_business_id_is_allowed_but_duplicate_identity_is_not(store, snapshot):
    store.create(snapshot)
    snapshot["objects"][1]["id"] = "obj-0"
    with pytest.raises(StoreValidation, match="id 重复"):
        store.create(snapshot)


def test_foreign_snapshot_ref_is_rejected(store, snapshot):
    snapshot["objects"][0]["source_ref"]["snapshot_id"] = "new-source"
    with pytest.raises(StoreValidation, match="此快照"):
        store.create(snapshot)


def test_disk_replace_failure_keeps_previous_complete_transaction(store, snapshot, monkeypatch):
    base = store.create(snapshot)
    project = store.commit(base["project_id"], 0, note())
    path = store.project_path(project["project_id"]) / "project.json"
    before = path.read_bytes()
    original_replace = os.replace

    def fail_current(source, target):
        if Path(target).name == "project.json":
            raise OSError("injected disk full or interrupted replace")
        return original_replace(source, target)

    monkeypatch.setattr(os, "replace", fail_current)
    with pytest.raises(OSError, match="injected"):
        store.commit(project["project_id"], 1, note("next"))
    assert path.read_bytes() == before
    assert store.load(project["project_id"]) == project
    assert not list(path.parent.glob("*.tmp"))
    assert path.with_name("previous.json").read_bytes() == before


def test_backup_failure_never_replaces_current(store, snapshot, monkeypatch):
    base = store.create(snapshot)
    project = store.commit(base["project_id"], 0, note())
    path = store.project_path(project["project_id"]) / "project.json"
    before = path.read_bytes()
    original_replace = os.replace

    def fail_backup(source, target):
        if Path(target).name == "previous.json":
            raise OSError("injected backup failure")
        return original_replace(source, target)

    monkeypatch.setattr(os, "replace", fail_backup)
    with pytest.raises(OSError, match="backup failure"):
        store.commit(project["project_id"], 1, note("next"))
    assert path.read_bytes() == before
    assert store.load(project["project_id"]) == project


@pytest.mark.parametrize("version", [True, -1, 1.0, "1"])
def test_versions_are_strict_integers(store, snapshot, version):
    project = store.create(snapshot)
    with pytest.raises(StoreValidation):
        store.commit(project["project_id"], version, note())
    with pytest.raises(StoreValidation):
        store.preview(project["project_id"], version, note())


def test_corruption_loads_read_only_backup_and_requires_explicit_recovery(store, snapshot):
    base = store.create(snapshot)
    pid = base["project_id"]
    store.commit(pid, 0, note())
    store.commit(pid, 1, note("note-2"))
    path = store.project_path(pid) / "project.json"
    path.write_bytes(b'{"truncated":')
    recovered_view = store.load(pid)
    assert recovered_view["revision"] == 1
    assert recovered_view["status"]["read_only"]
    assert recovered_view["status"]["recovery"]["required"]
    assert path.read_bytes() == b'{"truncated":'
    with pytest.raises(StoreReadOnly):
        store.commit(pid, 1, note("write-denied"))
    restored = store.recover(pid)
    assert not restored["status"]["read_only"]
    assert restored["revision"] > 2
    assert restored["intents"] == recovered_view["intents"]
    assert list(path.parent.glob("corrupt-*.json"))[0].read_bytes() == b'{"truncated":'
    with pytest.raises(StoreConflict):
        store.commit(pid, 2, note("pre-crash-window"))
    assert store.commit(pid, restored["revision"], note("after-recovery"))["revision"] > restored["revision"]


def test_checksum_change_without_valid_backup_cannot_be_opened(store, snapshot):
    project = store.create(snapshot)
    path = store.project_path(project["project_id"]) / "project.json"
    doc = json.loads(path.read_text(encoding="utf-8"))
    doc["payload"]["source_snapshot"]["junction_id"] = "tampered"
    path.write_text(json.dumps(doc), encoding="utf-8")
    with pytest.raises(StoreReadOnly, match="均无法读取"):
        store.load(project["project_id"])


def test_source_drift_callback_blocks_all_mutations_and_preserves_bound_snapshot(store, snapshot):
    project = store.create(snapshot)
    drifted = ProjectStore(store.root, source_validator=lambda old: [{"code": "source-hash-changed"}])
    loaded = drifted.load(project["project_id"])
    assert loaded["status"]["read_only"]
    assert loaded["source_snapshot"] == snapshot
    with pytest.raises(StoreReadOnly):
        drifted.commit(project["project_id"], 0, note())
    with pytest.raises(StoreReadOnly):
        drifted.preview(project["project_id"], 0, note())


@pytest.mark.parametrize("project_id", ["../outside", "C:/Windows", "../" + "a" * 32, "A" * 32, ""])
def test_project_paths_are_not_user_supplied_filesystem_paths(store, project_id):
    with pytest.raises(StoreValidation):
        store.load(project_id)


def ready_candidate(store, snapshot):
    project = store.create(snapshot)
    pid = project["project_id"]
    project = store.set_context(pid, 0, "engine-v1", "a" * 64, "b" * 64)
    directory = store.project_path(pid) / "artifacts"
    directory.mkdir()
    data = b"an actual immutable candidate file; not a geometry quality claim"
    (directory / "candidate.xodr").write_bytes(data)
    candidate = {"candidate_id": "candidate-1", "target_content_hash": project["content_hash"],
                 "context": project["context"], "artifacts": [{"relative_path": "artifacts/candidate.xodr",
                                                               "sha256": hashlib.sha256(data).hexdigest()}],
                 "affected_ids": [], "checks": [{"id": "compile", "status": "PASS"}]}
    return project, candidate


def test_candidate_validation_stale_after_edit_undo_and_policy_change(store, snapshot):
    project, candidate = ready_candidate(store, snapshot)
    pid = project["project_id"]
    accepted = store.accept_candidate(pid, project["revision"], "accept-1", candidate)
    assert not accepted["status"]["candidate_stale"]
    bundle = {"bundle_id": "check-1", "candidate_id": "candidate-1",
              "candidate_hash": digest(accepted["candidate"]), "context": accepted["context"],
              "checks": [{"id": "CRS", "status": "UNAVAILABLE"}], "decision": "BLOCKED"}
    checked = store.attach_validation(pid, accepted["revision"], "check-1", bundle)
    assert not checked["status"]["candidate_stale"]
    assert not checked["status"]["validation_stale"]
    assert not checked["status"]["formal_export_available"]
    edited = store.commit(pid, checked["revision"], note())
    assert edited["status"]["candidate_stale"] and edited["status"]["validation_stale"]
    undone = store.undo(pid, edited["revision"], "undo-note")
    assert undone["content_hash"] == accepted["content_hash"]
    assert undone["status"]["candidate_stale"]  # matching content alone is not reacceptance
    accepted_again = store.accept_candidate(pid, undone["revision"], "accept-2", candidate)
    assert not accepted_again["status"]["candidate_stale"]
    assert accepted_again["status"]["validation_stale"]
    changed = store.set_context(pid, accepted_again["revision"], "policy-v2", "a" * 64, "c" * 64)
    assert changed["status"]["candidate_stale"]
    with pytest.raises(StoreConflict, match="过期"):
        store.accept_candidate(pid, changed["revision"], "late-result", candidate)


def test_artifact_missing_changed_or_outside_project_is_rejected(store, snapshot):
    project, candidate = ready_candidate(store, snapshot)
    pid = project["project_id"]
    (store.project_path(pid) / "artifacts/candidate.xodr").write_bytes(b"changed")
    with pytest.raises(StoreValidation, match="哈希"):
        store.accept_candidate(pid, project["revision"], "accept-changed", candidate)
    candidate["artifacts"][0]["relative_path"] = "artifacts/../../outside.xodr"
    with pytest.raises(StoreValidation, match="路径"):
        store.accept_candidate(pid, project["revision"], "accept-outside", candidate)


def test_source_store_can_move_without_dependency_on_old_project_path(store, snapshot, tmp_path):
    project = store.create(snapshot)
    pid = project["project_id"]
    confirmed = store.commit(pid, 0, note())
    moved = tmp_path / "moved"
    store.root.rename(moved)
    assert ProjectStore(moved).load(pid) == confirmed


def test_concurrent_processes_cannot_both_commit_same_base_revision(store, snapshot, tmp_path):
    project = store.create(snapshot)
    gate = tmp_path / "gate"
    code = '''import json,sys,time
from pathlib import Path
from mapforge.workbench.store import ProjectStore,StoreConflict
root,pid,gate,number=sys.argv[1:]
Path(gate+number).write_text("ready")
deadline=time.monotonic()+8
while not Path(gate).exists():
    if time.monotonic()>deadline: raise RuntimeError("gate timeout")
    time.sleep(.01)
command={"command_id":"process-"+number,"type":"annotation","source_ref":"obj-0","scope":{"feature_ids":["obj-0"]},"parameters":{"text":"process "+number}}
try:
    ProjectStore(root).commit(pid,0,command)
    print("committed")
except StoreConflict:
    print("conflict")
'''
    processes = [subprocess.Popen([sys.executable, "-c", code, str(store.root), project["project_id"],
                                   str(gate), str(index)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  text=True, cwd=Path(__file__).resolve().parents[1]) for index in (1, 2)]
    try:
        deadline = time.monotonic() + 8
        while not all(Path(str(gate) + str(i)).exists() for i in (1, 2)):
            assert time.monotonic() < deadline, "child processes did not become ready"
            time.sleep(.01)
        gate.write_text("go")
        outputs = [process.communicate(timeout=10) for process in processes]
        assert all(process.returncode == 0 for process in processes), outputs
        assert sorted(stdout.strip() for stdout, _ in outputs) == ["committed", "conflict"]
        assert store.load(project["project_id"])["revision"] == 1
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
