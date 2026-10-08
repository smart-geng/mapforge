"""Single-candidate validation contract tests, not geometry acceptance.

The source registration and expensive metrics are synthetic. Actual byte/path
guards, finalizer assembly, original tier policy, owned-worker request protocol,
package verifier and service attachment contract remain in use. The final test
runs a real Windows Job bootstrap to check its request-directory behavior.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

from mapforge.workbench import surface_validation as M
from mapforge.workbench.contracts import canonical_bytes, content_hash, context_for, digest, StoreConflict
from mapforge.workbench.surface_checking import SurfaceCheckingService
from mapforge.validate import scoreboard as sb


@pytest.fixture
def setup(tmp_path, monkeypatch):
    project_id = "a" * 32
    directory = tmp_path / project_id
    run = directory / "artifacts" / ("b" * 32) / "run"
    run.mkdir(parents=True)
    snapshot = {"snapshot_id": "c" * 64, "content_hash": "d" * 64,
                "profile": {"sha256": "e" * 64}, "junction_id": "2023062115120840003", "objects": []}
    fingerprints = {"validator_hash": "f" * 64, "implementation_files_sha256": {"synthetic": "f" * 64},
                    "compiler_policy": {"compiler_hash": "1" * 64, "policy_hash": "2" * 64}}
    context = context_for(snapshot, "1" * 64, "2" * 64)
    project = {"project_id": project_id, "source_snapshot": snapshot, "revision": 3, "draft_epoch": 1,
               "intents": [], "content_hash": content_hash(snapshot, []), "context": context,
               "status": {"read_only": False, "candidate_stale": False}, "actions": {}}
    original = {"candidate.xodr": b"<?xml version='1.0'?><OpenDRIVE/>",
                "candidate.source-lanes.json": canonical_bytes({"source_format": "shp", "lanes": []}),
                "candidate.source-review.json": canonical_bytes({"schema": "synthetic-source-review", "source": "current"}),
                # A deliberately unrelated old board must never become the new check.
                "scoreboard.json": canonical_bytes({"old": True, "rows": [{"metrics": {"paving_holes_gt1cm2": 999}}]})}
    for name, data in original.items():
        (run / name).write_bytes(data)
    candidate = {"candidate_id": "candidate-surface-test", "target_content_hash": project["content_hash"],
                 "context": deepcopy(context), "accepted_revision": 3, "accepted_epoch": 1,
                 "artifacts": [{"relative_path": (run / name).relative_to(directory).as_posix(), "sha256": M._sha(data)}
                               for name, data in original.items()]}
    project["candidate"] = candidate
    state = SimpleNamespace(directory=directory, run=run, project=project, fingerprints=fingerprints,
                            snapshot=snapshot, original=original, calls=[], source_valid=True)

    def register(snapshot, *a):
        if not state.source_valid:
            raise ValueError("source drift")
        return SimpleNamespace(context=lambda s: context, source_dir=tmp_path, profile_path=tmp_path / "profile.yaml")

    monkeypatch.setattr(M.compiler, "register_source", register)
    monkeypatch.setattr(M.compiler, "verify_candidate_artifacts", lambda directory, candidate: {
        "primary_path": run / "candidate.xodr", "source_snapshot": deepcopy(snapshot)})
    monkeypatch.setattr(M, "validation_fingerprints", lambda: deepcopy(fingerprints))
    # Use the original finalizer while replacing only costly gate evaluators.
    from mapforge.report import decision
    from mapforge.validate import g11, junction_edges
    from mapforge.adapters.shp import profile_source
    from mapforge.workbench import consumer_metrics
    monkeypatch.setattr(decision, "evaluate_g8", lambda *a, **kw: {"gate_id": "G8", "status": "PASS", "exclusions": []})
    monkeypatch.setattr(g11, "audit_file", lambda *a, **kw: {"gate_id": "G11", "status": "FAIL"})
    monkeypatch.setattr(junction_edges, "audit", lambda *a, **kw: {"status": "PASS"})
    monkeypatch.setattr(profile_source, "ProfileSource", lambda *a, **kw: object())
    policy = yaml.safe_load(sb.POLICY.read_bytes())
    metrics = {check["metric"]: check["value"] for tier in policy["tiers"].values() for check in tier["checks"]
               if not check.get("applies_to") or "shp" in check["applies_to"]}
    metrics.update(paving_holes_gt1cm2=2, boundary_inside_p95_m=0.118)
    state.metrics = metrics

    def evaluate(candidate, pipeline, **kwargs):
        state.calls.append((candidate, pipeline, candidate.read_bytes()))
        return deepcopy(state.metrics), {"schema": consumer_metrics.SCHEMA, "status": "CHECKED", "synthetic": True}

    monkeypatch.setattr(consumer_metrics, "evaluate", evaluate)

    def child(command, timeout):
        assert command[:4] == [sys.executable, "-B", "-m", "mapforge.workbench.surface_validation"]
        assert timeout == 600
        M._worker_request(Path(command[-3]), command[-1])
        return {"exit_code": 0, "timed_out": False,
                "tree_control": "Windows Job Object, launch gated before descendants", "stdout": b"", "stderr": b""}

    monkeypatch.setattr(M, "_run_child", child)
    state.payload = {"project": project, "project_directory": str(directory)}
    return state


def validated(setup):
    result = M.validate_request(setup.payload)
    assert result["status"] == "VALIDATED", result
    return result


def output(setup, result):
    return setup.directory / "artifacts" / result["validation"]["bundle_id"]


def rewrite(path, mutate):
    value = json.loads(path.read_bytes())
    mutate(value)
    path.write_bytes(canonical_bytes(value))


def test_fresh_single_candidate_retains_failures_and_missing_baseline(setup):
    before = deepcopy(setup.project)
    result = validated(setup)
    assert setup.project == before
    assert len(setup.calls) == 1
    path, pipeline, data = setup.calls[0]
    assert path.parent == output(setup, result) and pipeline == "shp"
    assert data == setup.original["candidate.xodr"]
    assert {p.name: p.read_bytes() for p in setup.run.iterdir()} == setup.original
    report = M.verify_validation_artifacts(setup.directory, result["validation"])
    assert report == result["report"]
    assert report["baseline_decision"] == M.BASELINE
    assert report["changed_metrics"] is None
    assert report["scoreboard"]["files"] == 1
    row = report["scoreboard"]["rows"][0]
    assert row["metrics"]["paving_holes_gt1cm2"] == 2
    assert row["tiers"]["T1"]["status"] == row["tiers"]["T2"]["status"] == "FAIL"
    assert result["validation"]["decision"] == "BLOCKED"
    assert report["formal_delivery"] is result["accepted"] is False
    assert len(result["artifacts"]) == len(M._OUTPUTS)
    assert set(result["validation"]) == M._PACKAGE_FIELDS


def test_repeat_checks_use_new_independent_directory(setup):
    first, second = validated(setup), validated(setup)
    assert len(setup.calls) == 2
    assert first["validation"]["bundle_id"] != second["validation"]["bundle_id"]
    assert M.verify_validation_artifacts(setup.directory, first["validation"]) == first["report"]


@pytest.mark.parametrize("mutation", [
    lambda p: p.update(baseline_path="historical.xodr"),
    lambda p: p["project"]["candidate"].pop("accepted_revision"),
    lambda p: p["project"]["candidate"].update(accepted_epoch=0),
    lambda p: p["project"]["candidate"].update(accepted_revision=True),
    lambda p: p["project"]["status"].update(candidate_stale=True),
    lambda p: p["project"]["status"].update(read_only=True),
    lambda p: p["project"].update(content_hash="0" * 64),
    lambda p: p["project"].update(project_id="0" * 32),
    lambda p: p["project"]["context"].update(compiler_hash="0" * 64),
    lambda p: p["project"]["candidate"]["artifacts"].append(deepcopy(p["project"]["candidate"]["artifacts"][0])),
    lambda p: p["project"]["candidate"]["artifacts"].pop(2),
])
def test_rejects_ineligible_input_before_evaluation(setup, mutation):
    mutation(setup.payload)
    result = M.validate_request(setup.payload)
    assert result["status"] == "REJECTED" and result["validation"] is None
    assert not setup.calls


@pytest.mark.parametrize("name", sorted(M._OUTPUTS) + ["validation-result.json"])
def test_each_output_is_bound_to_recorded_bytes(setup, name):
    result = validated(setup)
    path = output(setup, result) / name
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises((ValueError, KeyError, TypeError)):
        M.verify_validation_artifacts(setup.directory, result["validation"])


@pytest.mark.parametrize("part", ["candidate.xodr", "candidate.source-lanes.json", "candidate.source-review.json", "scoreboard.json"])
def test_original_candidate_inputs_are_rechecked(setup, part):
    result = validated(setup)
    (setup.run / part).write_bytes(b"changed")
    with pytest.raises(ValueError):
        M.verify_validation_artifacts(setup.directory, result["validation"])


@pytest.mark.parametrize("kind", ["source", "fingerprints", "candidate"])
def test_drift_during_evaluation_returns_no_package(setup, monkeypatch, kind):
    evaluate = M._evaluate_single
    def drifting(out, inputs):
        evaluate(out, inputs)
        if kind == "source":
            setup.source_valid = False
        elif kind == "fingerprints":
            setup.fingerprints["validator_hash"] = "0" * 64
        else:
            (setup.run / "candidate.xodr").write_bytes(b"changed")
    monkeypatch.setattr(M, "_evaluate_single", drifting)
    result = M.validate_request(setup.payload)
    assert result["status"] == "REJECTED" and result["validation"] is None


@pytest.mark.parametrize("mutation", [
    lambda board: board.update(files=2),
    lambda board: board["rows"].append(deepcopy(board["rows"][0])),
    lambda board: board["rows"][0].update(case="another-junction"),
    lambda board: board["rows"][0].update(pipeline="map"),
    lambda board: board["rows"][0].update(artifact="old-baseline.xodr"),
    lambda board: board["rows"][0]["tiers"]["T1"].update(status="PASS"),
])
def test_wrong_or_forged_board_rejected_even_before_manifest_built(setup, monkeypatch, mutation):
    evaluate = M._evaluate_single
    def changed(out, inputs):
        evaluate(out, inputs)
        rewrite(out / "scoreboard.json", mutation)
        board = json.loads((out / "scoreboard.json").read_bytes())
        (out / "scoreboard.md").write_bytes(sb.markdown(board).encode())
    monkeypatch.setattr(M, "_evaluate_single", changed)
    result = M.validate_request(setup.payload)
    assert result["status"] == "REJECTED" and result["validation"] is None


@pytest.mark.parametrize("mutation", [
    lambda proc: proc.update(exit_code=1),
    lambda proc: proc.update(timed_out=True),
    lambda proc: proc.update(tree_control="unowned process"),
])
def test_child_failure_or_unowned_process_is_not_validation(setup, monkeypatch, mutation):
    child = M._run_child
    def changed(command, timeout):
        proc = child(command, timeout)
        mutation(proc)
        return proc
    monkeypatch.setattr(M, "_run_child", changed)
    result = M.validate_request(setup.payload)
    assert result["status"] == "REJECTED" and result["validation"] is None


def test_missing_metric_is_unavailable_and_stays_blocked(setup):
    setup.metrics.pop("xsd_valid")
    result = validated(setup)
    assert result["report"]["scoreboard"]["rows"][0]["tiers"]["T1"]["status"] == "UNAVAILABLE"
    assert result["validation"]["decision"] == "BLOCKED"


def test_checking_service_sends_current_candidate_without_legacy_baseline(setup):
    calls = []
    store = SimpleNamespace(load=lambda pid: deepcopy(setup.project), project_path=lambda pid: setup.directory)
    def start(project, operation, payload, request_id):
        calls.append((project, operation, payload, request_id))
        return {"job_id": "owned-validate-job"}
    service = SurfaceCheckingService(store, SimpleNamespace(start=start))
    result = service.start(setup.project["project_id"], 3, "request-validate")
    assert result["job"]["job_id"] == "owned-validate-job"
    assert calls[0][1] == "validate" and "baseline_path" not in calls[0][2]
    assert calls[0][2]["project"] == setup.project
    assert service.validation_module is M
    assert service._proofs["owned-validate-job"]["candidate_sha256"] == M._sha(setup.original["candidate.xodr"])
    with pytest.raises(StoreConflict):
        service.start(setup.project["project_id"], 2, "request-stale")


@pytest.mark.parametrize("changed", [None, "historical", "revision", "report", "candidate"])
def test_checking_attaches_only_current_owned_surface_result(setup, monkeypatch, changed):
    from mapforge.workbench import checking
    from mapforge.workbench.contracts import StoreValidation
    result = validated(setup)
    job = {"job_id": "job-surface-validate", "operation": "validate", "state": "succeeded",
           "base_revision": 3, "content_hash": setup.project["content_hash"],
           "archive": {"state": "persisted", "recorded_state": "succeeded"}, "result": result}
    attached = []
    def attach(*args):
        attached.append(args)
        return {"attached": True}
    store = SimpleNamespace(load=lambda pid: deepcopy(setup.project), project_path=lambda pid: setup.directory,
                            attach_validation=attach)
    jobs = SimpleNamespace(start=lambda *a: deepcopy(job), get=lambda *a, **kw: deepcopy(job))
    monkeypatch.setattr(checking, "verify_source_snapshot", lambda *a: {"matches": True})
    service = SurfaceCheckingService(store, jobs)
    service.start(setup.project["project_id"], 3, "request-owned")
    if changed == "historical":
        job["historical"] = True
    elif changed == "revision":
        setup.project["revision"] = 4
    elif changed == "report":
        job["result"]["report"]["candidate_sha256"] = "0" * 64
    elif changed == "candidate":
        (setup.run / "candidate.xodr").write_bytes(b"changed")
    if changed:
        with pytest.raises((StoreConflict, StoreValidation)):
            service.attach(setup.project["project_id"], setup.project["revision"], "attach-owned", job["job_id"])
        assert not attached
    else:
        assert service.attach(setup.project["project_id"], 3, "attach-owned", job["job_id"]) == {"attached": True}
        assert attached[0][-1] == result["validation"]


@pytest.mark.parametrize("change", ["extra", "missing", "copy", "manifest", "quality"])
def test_cross_binding_rejects_bad_outputs_before_manifest_is_created(setup, monkeypatch, change):
    evaluate = M._evaluate_single
    def changed(out, inputs):
        evaluate(out, inputs)
        if change == "extra":
            (out / "unlisted.txt").write_bytes(b"extra")
        elif change == "missing":
            (out / "candidate.g11.json").unlink()
        elif change == "copy":
            (out / "candidate.xodr").write_bytes(b"different candidate")
        elif change == "manifest":
            rewrite(out / "candidate.source-lanes.json", lambda v: v.update(lanes=["wrong-source"]))
        elif change == "quality":
            rewrite(out / "candidate.quality-report.json", lambda v: v["gates"]["G11"].update(status="PASS"))
    monkeypatch.setattr(M, "_evaluate_single", changed)
    result = M.validate_request(setup.payload)
    assert result["status"] == "REJECTED" and result["validation"] is None


@pytest.mark.skipif(os.name != "nt", reason="real Windows Job bootstrap")
def test_real_owned_bootstrap_does_not_precreate_validation_logs(tmp_path):
    from scripts.workbench_run_source_tracks import _run_child
    request = tmp_path / "request.json"
    request.write_bytes(b"{}")
    program = ("from pathlib import Path; import sys; "
               "p=Path(sys.argv[1]); assert {x.name for x in p.iterdir()}=={'request.json'}; "
               "print('request-only')")
    proc = _run_child([sys.executable, "-c", program, str(tmp_path)], 10)
    assert proc["exit_code"] == 0 and proc["timed_out"] is False
    assert proc["tree_control"] == "Windows Job Object, launch gated before descendants"
    assert b"request-only" in proc["stdout"]
    assert {p.name for p in tmp_path.iterdir()} == {"request.json"}
