"""Strict structural storage contracts, without running a geometry compiler.

Synthetic source objects here exercise transaction and scope invariants only;
they never prove that a real map can be reconstructed or accepted for release.
"""
import copy
import hashlib
import sys

import pytest

from mapforge.workbench.contracts import (StoreConflict, StoreValidation,
                                         context_for, digest, validate_capability,
                                         validate_command)
from mapforge.workbench.store import ProjectStore
from mapforge.workbench.surface_contracts import CAPABILITY_ID, INTENT_TYPE, JUNCTION_ID


@pytest.fixture
def snapshot():
    value = {"schema": "source-contract-fixture-v1", "snapshot_id": "test-dataset",
             "junction_id": JUNCTION_ID, "profile": {"sha256": "2" * 64},
             "source_files": [{"relative_path": "source.shp", "sha256": "1" * 64}],
             "objects": [
                 {"id": name, "source_ref": {"snapshot_id": "test-dataset", "layer": layer,
                                              "record_index": index, "part_index": 0},
                  "points": [[index, 0], [index, 1]], "raw_attributes": {"PID": name}}
                 for index, (name, layer) in enumerate([
                     ("source-surface", "IBD_OBJECT_INTERSECTION_SURFACE"),
                     ("source-lane", "IBD_LANE"), ("source-boundary", "IBD_LANE_BOUNDARY")])],
             "locator": {"source_dir": "F:/source", "profile_path": "F:/profile.yaml"}}
    value["content_hash"] = digest({key: item for key, item in value.items() if key != "locator"})
    return value


def spec(snapshot):
    return {"capability_id": CAPABILITY_ID, "type": INTENT_TYPE, "source_ref": "source-surface",
            "feature_ids": [item["id"] for item in snapshot["objects"]],
            "source_content_hash": snapshot["content_hash"], "operation_hash": "3" * 64}


def command(snapshot, command_id="surface-rebuild-1"):
    capability = spec(snapshot)
    return {"command_id": command_id, "type": INTENT_TYPE, "source_ref": capability["source_ref"],
            "scope": {"capability_id": CAPABILITY_ID, "feature_ids": capability["feature_ids"]},
            "parameters": {name: capability[name] for name in ("source_content_hash", "operation_hash")}}


def registered(tmp_path, snapshot):
    store = ProjectStore(tmp_path / "projects")
    project = store.create(snapshot, "结构合同测试")
    project = store.register_capabilities(project["project_id"], 0, "register-surface",
                                          [spec(snapshot)], "a" * 64, "b" * 64)
    return store, project


def test_register_preview_confirm_undo_redo_and_reopen_do_not_compile(tmp_path, snapshot, monkeypatch):
    for module in ("compiler", "surface_compiler", "source_surface_reconstruction", "source_surface_tracks"):
        monkeypatch.setitem(sys.modules, "mapforge.workbench." + module, None)
    store, initial = registered(tmp_path, snapshot)
    pid = initial["project_id"]
    path = store.project_path(pid) / "project.json"
    before = path.read_bytes()
    preview = store.preview(pid, initial["revision"], command(snapshot))
    assert path.read_bytes() == before
    assert preview["compiled"] is False
    assert preview["intent"]["scope"]["feature_ids"] == sorted(spec(snapshot)["feature_ids"])
    saved = store.commit(pid, initial["revision"], command(snapshot))
    assert saved["content_hash"] == preview["target_content_hash"]
    assert saved["candidate"] is None and saved["validation"] is None
    assert saved["draft_epoch"] == initial["draft_epoch"] + 1
    assert store.commit(pid, initial["revision"], command(snapshot)) == saved
    undone = store.undo(pid, saved["revision"], "undo-surface")
    assert undone["intents"] == [] and undone["content_hash"] == initial["content_hash"]
    redone = store.redo(pid, undone["revision"], "redo-surface")
    assert redone["intents"] == saved["intents"] and redone["content_hash"] == saved["content_hash"]
    assert redone["draft_epoch"] > saved["draft_epoch"]
    reopened = ProjectStore(store.root).load(pid)
    assert reopened == redone
    assert reopened["capabilities"] == initial["capabilities"]
    assert not reopened["status"]["formal_export_available"]


def test_scope_order_is_normalized_without_mutating_request(snapshot):
    context = context_for(snapshot, "a" * 64, "b" * 64)
    raw = spec(snapshot)
    original = copy.deepcopy(raw)
    capability = validate_capability(raw, snapshot, context)
    assert raw == original
    requested = command(snapshot)
    requested["scope"]["feature_ids"].reverse()
    before = copy.deepcopy(requested)
    value = validate_command(requested, snapshot, 0, capabilities={CAPABILITY_ID: capability}, context=context)
    assert requested == before
    assert value["scope"]["feature_ids"] == sorted(raw["feature_ids"])
    assert value["parameters"] == requested["parameters"]


def test_capability_flag_does_not_grant_registration(tmp_path, snapshot):
    snapshot["capabilities"] = {"geometry_edit": True, "surface_rebuild": True}
    snapshot["content_hash"] = digest({key: item for key, item in snapshot.items()
                                      if key not in {"locator", "content_hash"}})
    store = ProjectStore(tmp_path)
    project = store.create(snapshot)
    with pytest.raises(StoreValidation, match="没有服务器登记"):
        store.commit(project["project_id"], 0, command(snapshot))
    assert store.load(project["project_id"]) == project


@pytest.mark.parametrize("mutate", [
    lambda value: value.update(path="old/candidate.xodr"),
    lambda value: value.update(confirmed=True),
    lambda value: value.update(candidate_accepted=True),
    lambda value: value.update(command_id="../escape"),
    lambda value: value.update(base_revision=True),
    lambda value: value.update(source_ref="source-lane"),
    lambda value: value["scope"].update(capability_id="another-capability"),
    lambda value: value["scope"].update(feature_ids=["source-surface"]),
    lambda value: value["scope"]["feature_ids"].append("source-surface"),
    lambda value: value["scope"]["feature_ids"].append("another-source"),
    lambda value: value["scope"].update(road_ids=[12]),
    lambda value: value["parameters"].update(padding_m=0),
    lambda value: value["parameters"].update(source_content_hash="0" * 64),
    lambda value: value["parameters"].update(operation_hash="0" * 64),
    lambda value: value["parameters"].update(operation_hash="A" * 64),
    lambda value: value["parameters"].pop("operation_hash"),
    lambda value: value.update(parameters=None),
])
def test_invalid_command_is_rejected_before_any_persistence(tmp_path, snapshot, mutate):
    store, project = registered(tmp_path, snapshot)
    pid = project["project_id"]
    before = (store.project_path(pid) / "project.json").read_bytes()
    request = command(snapshot)
    mutate(request)
    for method in (store.preview, store.commit):
        with pytest.raises((StoreValidation, StoreConflict)):
            method(pid, project["revision"], request)
        assert (store.project_path(pid) / "project.json").read_bytes() == before


@pytest.mark.parametrize("mutate", [
    lambda value: value.update(capability_id="different-surface-v1"),
    lambda value: value.update(feature_ids=["source-surface"]),
    lambda value: value["feature_ids"].append("source-surface"),
    lambda value: value.update(source_ref="outside"),
    lambda value: value.update(source_content_hash="0" * 64),
    lambda value: value.update(operation_hash="invalid"),
    lambda value: value.update(padding_m=0),
    lambda value: value.pop("operation_hash"),
])
def test_invalid_server_registration_cannot_create_a_partial_capability(tmp_path, snapshot, mutate):
    store = ProjectStore(tmp_path)
    project = store.create(snapshot)
    value = spec(snapshot)
    mutate(value)
    with pytest.raises((StoreValidation, StoreConflict)):
        store.register_capabilities(project["project_id"], 0, "bad-registration", [value], "a" * 64, "b" * 64)
    assert store.load(project["project_id"]) == project


def test_stale_context_rejects_old_registration_after_runtime_change(tmp_path, snapshot):
    store, project = registered(tmp_path, snapshot)
    current = store.set_context(project["project_id"], project["revision"], "runtime-change", "c" * 64, "b" * 64)
    with pytest.raises(StoreConflict, match="指纹已过期"):
        store.commit(current["project_id"], current["revision"], command(snapshot))
    assert store.load(current["project_id"]) == current


@pytest.mark.parametrize("field", ["source_hash", "profile_hash"])
def test_foreign_source_context_cannot_register(snapshot, field):
    context = context_for(snapshot, "a" * 64, "b" * 64)
    context[field] = "0" * 64
    with pytest.raises(StoreConflict, match="不属于当前源快照"):
        validate_capability(spec(snapshot), snapshot, context)


def test_changed_source_without_new_content_checksum_cannot_register(snapshot):
    snapshot["objects"][0]["points"][0][0] += 1
    with pytest.raises(StoreConflict, match="实际内容"):
        validate_capability(spec(snapshot), snapshot, context_for(snapshot, "a" * 64, "b" * 64))


def test_same_dataset_different_junction_cannot_register(snapshot):
    snapshot["junction_id"] = "another-junction"
    snapshot["content_hash"] = digest({key: item for key, item in snapshot.items()
                                      if key not in {"locator", "content_hash"}})
    with pytest.raises(StoreValidation, match="0621"):
        validate_capability(spec(snapshot), snapshot, context_for(snapshot, "a" * 64, "b" * 64))


def test_location_changes_do_not_change_content_or_capability(snapshot):
    moved = copy.deepcopy(snapshot)
    moved["locator"] = {"source_dir": "D:/新位置", "profile_path": "D:/新位置/profile.yaml"}
    first = validate_capability(spec(snapshot), snapshot, context_for(snapshot, "a" * 64, "b" * 64))
    second = validate_capability(spec(moved), moved, context_for(moved, "a" * 64, "b" * 64))
    assert first == second


def test_altered_registered_spec_hash_is_not_trusted(snapshot):
    context = context_for(snapshot, "a" * 64, "b" * 64)
    capability = validate_capability(spec(snapshot), snapshot, context)
    capability["spec_hash"] = "0" * 64
    with pytest.raises(StoreConflict, match="登记哈希"):
        validate_command(command(snapshot), snapshot, 0, capabilities={CAPABILITY_ID: capability}, context=context)


def test_scope_covers_210_objects_without_narrowing_to_visible_layers(tmp_path, snapshot):
    for index in range(3, 210):
        snapshot["objects"].append({"id": f"object-{index}", "source_ref": {
            "snapshot_id": snapshot["snapshot_id"], "layer": "unrecognized-preserved", "record_index": index}})
    snapshot["content_hash"] = digest({key: item for key, item in snapshot.items()
                                      if key not in {"locator", "content_hash"}})
    store, project = registered(tmp_path, snapshot)
    saved = store.commit(project["project_id"], project["revision"], command(snapshot))
    assert len(saved["intents"][0]["scope"]["feature_ids"]) == 210


def test_undo_redo_never_reactivate_previously_accepted_candidate(tmp_path, snapshot):
    store, project = registered(tmp_path, snapshot)
    pid = project["project_id"]
    saved = store.commit(pid, project["revision"], command(snapshot))
    artifact = store.project_path(pid) / "artifacts" / "contract-fixture.txt"
    artifact.parent.mkdir()
    artifact.write_bytes(b"not geometry: candidate epoch contract fixture")
    candidate = {"candidate_id": "contract-only", "target_content_hash": saved["content_hash"],
                 "context": saved["context"], "affected_ids": ["source-surface"], "checks": [],
                 "artifacts": [{"relative_path": "artifacts/contract-fixture.txt",
                                "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest()}]}
    accepted = store.accept_candidate(pid, saved["revision"], "accept-contract-fixture", candidate)
    assert not accepted["status"]["candidate_stale"]
    undone = store.undo(pid, accepted["revision"], "undo-after-accept")
    redone = store.redo(pid, undone["revision"], "redo-after-accept")
    assert redone["content_hash"] == accepted["content_hash"]
    assert redone["status"]["candidate_stale"]
    assert redone["candidate"] == accepted["candidate"]
    assert ProjectStore(store.root).load(pid)["status"]["candidate_stale"]
    assert not redone["status"]["formal_export_available"]
