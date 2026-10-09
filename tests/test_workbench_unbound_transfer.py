"""Uncompiled real source drafts are portable without pretending to be checked."""
import copy

import pytest

from mapforge.workbench.contracts import StoreValidation, context_for
from mapforge.workbench.project_context import validate_project_context
from mapforge.workbench.project_relocation import relocate_project
from mapforge.workbench.store import ProjectStore
from test_workbench_project_relocation import case, _tree
from test_workbench_transfer_api import _client, _package, _upload, _import


def _fresh(c, tmp_path, *, bind=False):
    store = ProjectStore(tmp_path / "未编译源工程")
    project = store.create(c.snapshot, "只有原件和待办，没有启用编辑")
    ref = c.snapshot["objects"][0]["id"]
    for n in (1, 2):
        project = store.commit(project["project_id"], project["revision"], {
            "command_id": f"note-{n}", "type": "annotation", "source_ref": ref,
            "scope": {"feature_ids": [ref]},
            "parameters": {"text": f"真正未编译的待办 {n}", "status": "unresolved"}})
    project = store.undo(project["project_id"], project["revision"], "keep-redo")
    if bind:
        project = store.set_context(project["project_id"], project["revision"], "bind-now", "a"*64, "b"*64)
    return store, project


@pytest.mark.parametrize("bind", [False, True])
def test_real_uncompiled_current_or_previous_round_trip_over_http(case, tmp_path, bind):
    store, project = _fresh(case, tmp_path, bind=bind)
    before = _tree(store.project_path(project["project_id"]))
    target_store = ProjectStore(tmp_path / "导入后的源工程")
    with _client(store, case.old_raw, case.old_profile) as source, \
            _client(target_store, case.raw, case.profile) as target:
        _, response = _package(source, project)
        result = _import(target, _upload(target, response.content))
        assert result["state"] == "succeeded", result
        assert result["result"]["formal_delivery"] is False
        reopened = target.get("/api/projects/" + project["project_id"]).json()
        assert reopened["intents"] == project["intents"]
        assert reopened["timeline"] == project["timeline"]
        assert reopened["context"] == project["context"]
        assert reopened["status"]["can_redo"]
        assert reopened["candidate"] is None and reopened["validation"] is None
        assert reopened["status"]["formal_export_available"] is False
    assert _tree(store.project_path(project["project_id"])) == before
    for name, data in before.items():
        if name != "project.json":
            assert (target_store.project_path(project["project_id"]) / name).read_bytes() == data


def test_direct_relocation_of_newly_created_project_needs_no_compiler(case, tmp_path):
    store = ProjectStore(tmp_path / "从未编辑")
    project = store.create(case.snapshot, "新建源工程")
    assert project["context"] == context_for(case.snapshot)
    result = relocate_project(store.project_path(project["project_id"]), tmp_path / "新工程库",
                              case.raw, case.profile)
    assert result["status"] == "RELOCATED_RECOMPUTE_REQUIRED"
    assert result["project"]["context"]["compiler_hash"] == ""
    assert result["project"]["context"]["policy_hash"] == ""
    assert result["formal_delivery"] is False


@pytest.mark.parametrize("key,value", [("compiler_hash", "a"*64), ("policy_hash", "b"*64),
                                      ("compiler_hash", None), ("policy_hash", "invalid"),
                                      ("profile_hash", "c"*64), ("source_hash", "d"*64)])
def test_partial_or_wrong_empty_context_is_not_a_source_draft(case, tmp_path, key, value):
    store, project = _fresh(case, tmp_path)
    project["context"][key] = value
    with pytest.raises(StoreValidation):
        validate_project_context(project)


@pytest.mark.parametrize("key", ["candidate", "validation", "candidate_history", "validation_history", "capabilities"])
def test_compiled_or_capable_project_cannot_claim_empty_context(case, key):
    project = copy.deepcopy(case.project)
    project["context"] = context_for(project["source_snapshot"])
    for field in ("candidate", "validation"):
        project[field] = None
    for field in ("candidate_history", "validation_history"):
        project[field] = []
    project["capabilities"] = {}
    project[key] = {} if key in {"candidate", "validation"} else ["stored"] if key.endswith("history") else {"registered": {}}
    with pytest.raises(StoreValidation):
        validate_project_context(project)


def test_empty_context_does_not_relax_candidate_or_check_validation(case):
    from mapforge.workbench.contracts import validate_context
    with pytest.raises(StoreValidation):
        validate_context(context_for(case.snapshot))

