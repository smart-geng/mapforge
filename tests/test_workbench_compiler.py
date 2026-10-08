"""WB05 real source/real file replay and draft rejection/retention semantics.

The bounded registered asset is required for these integration checks. They do
not substitute synthetic maps when the source dataset or baseline is absent.
"""
import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from mapforge.workbench.compiler import (
    BASELINE_SHA256, CAPABILITY_ID, DEFAULT_BASELINE, INTENT_TYPE, JUNCTION_ID,
    ROOT, CompilerRejected, compile_draft, compile_request,
    register_verified_baseline,
)
from mapforge.workbench.contracts import canonical_bytes, content_hash, digest
from mapforge.workbench.sources import build_source_snapshot
from mapforge.workbench.store import ProjectStore

SELECTED_SHA256 = "78b893993494b46ee5aa1b094f1fd2ad9f40efdad2d2c6c7943b515a5ea6cd86"
pytestmark = pytest.mark.skipif(not DEFAULT_BASELINE.is_file() or not (ROOT / "shp_0222-0326").is_dir(),
                                reason="WB05 integration requires the registered real source and XODR")


@pytest.fixture(scope="module")
def registered():
    snapshot = build_source_snapshot(ROOT / "shp_0222-0326", ROOT / "profiles/shp/ibd-smarteditor-v1.yaml", JUNCTION_ID)
    return snapshot, register_verified_baseline(snapshot)


def command(registration, delta=-.015, command_id="geometry-1"):
    spec = registration.store_spec()
    return {"command_id": command_id, "type": INTENT_TYPE, "source_ref": spec["source_ref"],
            "scope": {"capability_id": CAPABILITY_ID, "feature_ids": spec["feature_ids"]},
            "parameters": {"normal_delta_m": delta, "baseline_sha256": BASELINE_SHA256}}


def draft(registered, intents=None):
    snapshot, registration = registered
    intents = [] if intents is None else intents
    return {"source_snapshot": copy.deepcopy(snapshot), "revision": 1, "draft_epoch": 1,
            "intents": copy.deepcopy(intents), "content_hash": content_hash(snapshot, intents),
            "context": registration.context(snapshot)}


def test_registered_identity_and_scope_come_from_real_source(registered):
    snapshot, registration = registered
    capability = registration.capability()
    objects = {obj["id"]: obj for obj in snapshot["objects"]}
    target = objects[capability["source_ref"]]
    assert target["business_id"] == "2023061410304612847"
    assert target["source_ref"]["layer"] == "IBD_LANE_BOUNDARY"
    assert len(registration.feature_ids) == 9
    assert len(registration.affected_ids) == 3
    assert capability["coordinate_evidence"]["absolute_crs_verified"] is False
    assert capability["control"]["continuous"] is True
    assert capability["scope_s_m"] == [76.2690602264151, 110.]


def test_zero_bytes_and_actual_registered_target_bytes(registered, tmp_path):
    _, registration = registered
    before_source = Path(registration.baseline_path).read_bytes()
    zero = compile_draft(draft(registered), registration, tmp_path)
    assert zero.artifact_bytes == before_source
    assert zero.candidate["affected_ids"] == []
    actual_draft = draft(registered, [command(registration)])
    unchanged = canonical_bytes(actual_draft)
    result = compile_draft(actual_draft, registration, tmp_path)
    assert hashlib.sha256(result.artifact_bytes).hexdigest() == SELECTED_SHA256
    assert canonical_bytes(actual_draft) == unchanged
    assert Path(registration.baseline_path).read_bytes() == before_source
    assert result.candidate["target_content_hash"] == actual_draft["content_hash"]
    assert result.candidate["context"] == actual_draft["context"]
    assert result.evidence["local_checks"]["outside_coefficient_error"] == 0
    assert result.evidence["local_checks"]["delta_C2_jet_max_error"] < 1e-10
    assert result.evidence["whole_map_validation"] == "NOT_RUN"
    for artifact in result.candidate["artifacts"]:
        data = (tmp_path / artifact["relative_path"]).read_bytes()
        assert hashlib.sha256(data).hexdigest() == artifact["sha256"]


@pytest.mark.parametrize("field", ["source_hash", "profile_hash", "compiler_hash", "policy_hash"])
def test_stale_context_cannot_publish(registered, tmp_path, field):
    doc = draft(registered, [command(registered[1])])
    doc["context"][field] = "0" * 64
    with pytest.raises(CompilerRejected, match="指纹"):
        compile_draft(doc, registered[1], tmp_path)
    assert list(tmp_path.iterdir()) == []


def test_draft_hash_and_exact_dependency_scope_checked(registered, tmp_path):
    doc = draft(registered, [command(registered[1])])
    doc["intents"][0]["parameters"]["normal_delta_m"] = -.01
    with pytest.raises(CompilerRejected) as caught:
        compile_draft(doc, registered[1], tmp_path)
    assert caught.value.code == "draft-hash-mismatch"
    doc["intents"][0]["scope"]["feature_ids"].pop()
    doc["content_hash"] = content_hash(doc["source_snapshot"], doc["intents"])
    with pytest.raises(CompilerRejected) as caught:
        compile_draft(doc, registered[1], tmp_path)
    assert caught.value.code == "scope-mismatch"
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("target", [.101, -.101, True, "-0.015"])
def test_invalid_finite_targets_rejected(registered, tmp_path, target):
    doc = draft(registered, [command(registered[1], target)])
    with pytest.raises(CompilerRejected) as caught:
        compile_draft(doc, registered[1], tmp_path)
    assert caught.value.code == "target-out-of-range"


def test_nonfinite_request_is_structured_rejection(registered, tmp_path):
    doc = draft(registered)
    doc["intents"] = [command(registered[1], float("nan"))]
    result = compile_request({"draft": doc, "project_directory": str(tmp_path)})
    assert result["status"] == "REJECTED" and result["candidate"] is None
    assert not (tmp_path / "artifacts").exists()


def test_target_identity_survives_baseline_relocation(registered, tmp_path):
    snapshot, registration = registered
    moved = tmp_path / "different-name.xodr"
    shutil.copyfile(DEFAULT_BASELINE, moved)
    shutil.copyfile(DEFAULT_BASELINE.with_suffix(".source-lanes.json"), moved.with_suffix(".source-lanes.json"))
    relocated = register_verified_baseline(snapshot, baseline_path=moved)
    assert relocated.store_spec() == registration.store_spec()
    assert relocated.source_ref == registration.source_ref
    moved.write_bytes(b"changed baseline")
    with pytest.raises(CompilerRejected) as caught:
        compile_draft(draft((snapshot, relocated)), relocated, tmp_path)
    assert caught.value.code == "baseline-drift"


def test_source_objects_checked_against_original_rows(registered):
    snapshot, registration = registered
    changed = copy.deepcopy(snapshot)
    target = next(o for o in changed["objects"] if o["id"] == registration.source_ref)
    target["points"][0][0] += .000001
    changed["content_hash"] = digest({k: v for k, v in changed.items() if k not in {"locator", "content_hash"}})
    with pytest.raises(CompilerRejected) as caught:
        register_verified_baseline(changed)
    assert caught.value.code == "source-object-mismatch"


def test_missing_original_source_rejected_without_editing_it(registered, tmp_path):
    snapshot, registration = registered
    missing = tmp_path / "missing-source"
    missing.mkdir()
    altered_locator = replace(registration, source_dir=str(missing))
    with pytest.raises(CompilerRejected) as caught:
        compile_draft(draft(registered), altered_locator, tmp_path)
    assert caught.value.code == "source-drift"


def test_rejected_real_target_saved_but_last_candidate_retained(registered, tmp_path):
    snapshot, registration = registered
    store = ProjectStore(tmp_path / "projects")
    project = store.create(snapshot)
    context = registration.context(snapshot)
    project = store.register_capabilities(project["project_id"], project["revision"], "register",
                                           [registration.store_spec()], context["compiler_hash"], context["policy_hash"])
    project = store.commit(project["project_id"], project["revision"], command(registration))
    result = compile_draft(project, registration, store.project_path(project["project_id"]))
    project = store.accept_candidate(project["project_id"], project["revision"], "accept-good", result.candidate)
    previous = copy.deepcopy(project["candidate"])
    project = store.commit(project["project_id"], project["revision"], command(registration, +.015, "unsafe-direction"))
    # This fixed negative safety case is intentionally opposite to the measured
    # residual, not a search for another positive probe target.
    reopened = store.load(project["project_id"])
    assert reopened["intents"][-1]["parameters"]["normal_delta_m"] == .015
    with pytest.raises(CompilerRejected) as caught:
        compile_draft(reopened, registration, store.project_path(project["project_id"]))
    assert caught.value.code == "source-residual-regression"
    after = store.load(project["project_id"])
    assert after["candidate"] == previous
    assert after["status"]["candidate_stale"] is True
    for artifact in previous["artifacts"]:
        data = (store.project_path(project["project_id"]) / artifact["relative_path"]).read_bytes()
        assert hashlib.sha256(data).hexdigest() == artifact["sha256"]


def test_latest_intent_is_absolute_not_accumulated(registered, tmp_path):
    _, registration = registered
    intents = [command(registration, -.015, "first"), command(registration, 0., "reset")]
    result = compile_draft(draft(registered, intents), registration, tmp_path)
    assert result.artifact_bytes == DEFAULT_BASELINE.read_bytes()
    assert result.evidence["geometry_command_ids"] == ["first", "reset"]
