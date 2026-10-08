"""Probe orchestration uses controlled fake children; never compiles real maps."""
from __future__ import annotations

import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from scripts import workbench_run_source_cells as R


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def report(status="EXPERIMENT_EVALUATED"):
    return {
        "schema": "mapforge/wb11-source-cells-probe/v1",
        "status": status,
        "stage": "WHOLE_MAP_SCORE",
        "candidate_accepted": False,
        "formal_release_verified": False,
        "independent_operator_verified": False,
        "default_postprocess_executed": True,
        "whole_map_score_executed": True,
        "code_policy_changed": [],
        "source_integrity_after": {"matches": True, "issues": []},
        "local_support_audit": {"status": "PASS", "candidate_accepted": False},
        "postprocessed_support_audit": {"status": "PASS", "candidate_accepted": False},
    }


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def refresh_code_bindings(out):
    paths = R._required_code_paths()
    write_json(out / "code-policy-binding.json", {
        p.relative_to(R.ROOT).as_posix(): digest(p) for p in paths
    })
    for name in R.REQUIRED_OUTPUTS:
        if name.startswith("code-snapshots/"):
            relative = name.removeprefix("code-snapshots/")
            (out / name).write_bytes((R.ROOT / relative).read_bytes())


@pytest.fixture
def complete_output(tmp_path, monkeypatch):
    """Tiny byte-bound repository and artifacts, all synthetic; no geometry runs."""
    root = tmp_path / "fixture-repository"
    for name in (
        "mapforge/workbench/source_surface_cells.py",
        "mapforge/workbench/source_surface_rebuild.py",
        "scripts/workbench_probe_source_cells.py",
        "uv.lock", "profiles/shp/ibd-smarteditor-v1.yaml",
        "profiles/validation/static-acceptance-v1.draft.yaml",
        "profiles/validation/g8-opendrive-jinfeng-v1.yaml",
        *R.EXTRA_DEPENDENCIES,
    ):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# synthetic test file: " + name + "\n", encoding="utf-8")
    monkeypatch.setattr(R, "ROOT", root)
    monkeypatch.setattr(R, "PROBE", root / "scripts/workbench_probe_source_cells.py")
    out = tmp_path / "complete-output"
    for name in R.REQUIRED_OUTPUTS:
        path = out / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}" if name.endswith(".json") else "synthetic evidence " + name,
                        encoding="utf-8")
    refresh_code_bindings(out)
    r = report()
    for field, name in (
        ("source_partial_sha256", "source-generated.partial.xodr"),
        ("unaccepted_surface_sha256", "surface.unaccepted.xodr"),
        ("candidate_sha256", "candidate.xodr"),
    ):
        r[field] = digest(out / name)
    r["source_snapshot_id"] = "synthetic-source-snapshot"
    snapshot = {"snapshot_id": r["source_snapshot_id"], "content_hash": "synthetic-content",
                "junction_id": "synthetic-junction"}
    intent = {"source_snapshot_id": snapshot["snapshot_id"],
              "source_content_hash": snapshot["content_hash"],
              "junction_id": snapshot["junction_id"], "confirmed": True}
    write_json(out / "source-snapshot.json", snapshot)
    write_json(out / "confirmed-probe-intent.json", intent)
    write_json(out / "surface-evidence.json", {
        "written_sha256": r["unaccepted_surface_sha256"], "intent": intent,
        "candidate_accepted": False, "non_paving_unchanged": True,
        "actual_xml_support_audit": r["local_support_audit"],
    })
    write_json(out / "postprocessed-support-audit.json", r["postprocessed_support_audit"])
    # Completed experimental evaluation can have FAIL scores and blocked delivery.
    gates = {"G8": {"status": "FAIL"}, "G11": {"status": "FAIL"},
             "G11-edge-contacts": {"status": "PASS"}}
    for key, name in (("G8", "candidate.g8.json"), ("G11", "candidate.g11.json"),
                      ("G11-edge-contacts", "candidate.edge-contacts.json")):
        write_json(out / name, gates[key])
    r["delivery"] = {"schema": "mapforge/delivery-decision/v1", "status": "BLOCKED"}
    write_json(out / "candidate.delivery-decision.json", r["delivery"])
    write_json(out / "candidate.quality-report.json", {
        "schema": "mapforge/opendrive-quality-report/v1", "artifact": str(out / "candidate.xodr"),
        "gates": gates, "delivery_decision": r["delivery"],
    })
    r["tiers"] = {"T1": {"status": "FAIL"}, "T2": {"status": "FAIL"}}
    write_json(out / "scoreboard.json", {
        "schema": "mapforge/scoreboard/v1", "files": 1, "run_dir": str(out),
        "rows": [{"case": intent["junction_id"], "pipeline": "shp", "artifact": "candidate.xodr",
                  "metrics": {"synthetic_metric": 1.0}, "tiers": r["tiers"]}],
        "tier_pass": {"T1": 0, "T2": 0},
    })
    write_json(out / "report.json", r)
    return out


def change_json(out, name, key, value):
    path = out / name
    data = json.loads(path.read_text(encoding="utf-8"))
    data[key] = value
    write_json(path, data)


def test_complete_bound_evaluation_is_zero_even_when_quality_fails(complete_output):
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 0, result
    assert result["status"] == "EXPERIMENT_COMPLETE"
    assert result["experiment_complete"] is True
    assert result["candidate_accepted"] is False
    assert result["formal_release_verified"] is False
    assert result["problems"] == []
    assert set(result["evidence_bindings"]) == set(R.REQUIRED_OUTPUTS)


@pytest.mark.parametrize("child_exit", [1, 2, 7, -1])
def test_complete_evidence_cannot_mask_nonzero_child_exit(complete_output, child_exit):
    result = R.assess_output(complete_output, child_exit)
    assert result["exit_code"] == 1
    assert result["status"] == "CHILD_FAILED"


@pytest.mark.parametrize("field,value", [
    ("schema", "unknown/v1"), ("stage", "DEFAULT_POSTPROCESS"),
    ("candidate_accepted", True), ("formal_release_verified", True),
    ("independent_operator_verified", True), ("default_postprocess_executed", False),
    ("whole_map_score_executed", False), ("code_policy_changed", ["unexpected.py"]),
    ("source_integrity_after", {"matches": False, "issues": [{"code": "source-file-changed"}]}),
    ("source_integrity_after", {"matches": True, "issues": [{"code": "unresolved"}]}),
    ("source_snapshot_id", "different-input"),
])
def test_inconsistent_terminal_claim_fails_complete_fixture(complete_output, field, value):
    change_json(complete_output, "report.json", field, value)
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert result["problems"]


@pytest.mark.parametrize("name", [
    "source-generated.partial.xodr", "surface.unaccepted.xodr", "candidate.xodr",
])
def test_each_xodr_hash_is_checked(complete_output, name):
    path = complete_output / name
    path.write_bytes(path.read_bytes() + b"changed")
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert any("hash mismatch" in problem for problem in result["problems"])


@pytest.mark.parametrize("name", [
    "candidate.quality-report.json", "candidate.g11.json", "scoreboard.json",
    "code-snapshots/mapforge/workbench/source_surface_cells.py",
])
def test_missing_required_evidence_fails(complete_output, name):
    (complete_output / name).unlink()
    assert R.assess_output(complete_output, 0)["exit_code"] == 1


@pytest.mark.parametrize("name,field,value", [
    ("confirmed-probe-intent.json", "confirmed", False),
    ("surface-evidence.json", "non_paving_unchanged", False),
    ("postprocessed-support-audit.json", "status", "FAIL"),
    ("candidate.g11.json", "status", "PASS"),
    ("candidate.delivery-decision.json", "status", "ACCEPTED"),
    ("candidate.quality-report.json", "artifact", "another.xodr"),
    ("scoreboard.json", "tier_pass", {"T1": 1, "T2": 1}),
    ("scoreboard.json", "run_dir", "another-run"),
])
def test_cross_document_mismatch_fails(complete_output, name, field, value):
    change_json(complete_output, name, field, value)
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert result["problems"]


def test_live_code_drift_is_not_hidden_by_unchanged_report_claim(complete_output):
    path = R.ROOT / "mapforge/workbench/source_surface_cells.py"
    path.write_bytes(path.read_bytes() + b"# changed after worker check\n")
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert any("binding changed" in problem for problem in result["problems"])


def test_missing_code_binding_and_altered_snapshot_fail(complete_output):
    (complete_output / "code-snapshots/mapforge/workbench/source_surface_cells.py").write_bytes(b"other")
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert any("snapshot mismatch" in problem for problem in result["problems"])
    write_json(complete_output / "code-policy-binding.json", {})
    result = R.assess_output(complete_output, 0)
    assert result["exit_code"] == 1
    assert any("incomplete code/policy" in problem for problem in result["problems"])


@pytest.mark.parametrize("content", [None, "{broken", "[]", "null", '"not an object"',
                                     '{"status":"FAILED","status":"EXPERIMENT_EVALUATED"}',
                                     '{"invalid":NaN}'])
def test_missing_invalid_or_nonobject_report_fails_closed(tmp_path, content):
    if content is not None:
        (tmp_path / "report.json").write_text(content, encoding="utf-8")
    result = R.assess_output(tmp_path, 0)
    assert result["exit_code"] == 1
    assert result["problems"]


@pytest.mark.parametrize("state", ["RUNNING", "PASS", "COMPLETED", "FAILED", ""])
def test_nonterminal_or_failure_report_never_becomes_process_success(tmp_path, state):
    write_json(tmp_path / "report.json", report(state))
    result = R.assess_output(tmp_path, 0)
    assert result["exit_code"] == 1
    assert result["report_status"] == state


def test_evaluated_word_without_bound_artifacts_is_not_success(tmp_path):
    write_json(tmp_path / "report.json", report())
    result = R.assess_output(tmp_path, 0)
    assert result["exit_code"] == 1
    assert result["problems"]


def test_timeout_takes_precedence_over_child_zero_and_report_state(tmp_path):
    write_json(tmp_path / "report.json", report())
    result = R.assess_output(tmp_path, 0, timed_out=True)
    assert result["exit_code"] == 3


def fake_probe(tmp_path, body):
    path = tmp_path / "controlled_probe.py"
    path.write_text(
        "import argparse,json,os,subprocess,sys,time\n"
        "from pathlib import Path\n"
        "parser=argparse.ArgumentParser()\n"
        "parser.add_argument('--out',type=Path,required=True)\n"
        "args=parser.parse_args()\n"
        "args.out.mkdir(parents=True,exist_ok=True)\n" + body,
        encoding="utf-8",
    )
    return path


def test_old_probe_reporting_failure_but_exiting_zero_is_failure(tmp_path, monkeypatch):
    payload = report("FAILED")
    fake = fake_probe(tmp_path, "(args.out/'report.json').write_text(" +
                      repr(json.dumps(payload)) + ",encoding='utf-8')\n")
    monkeypatch.setattr(R, "PROBE", fake)
    result = R.run_experiment(tmp_path / "run", timeout_s=15)
    assert result["exit_code"] == 1
    assert result["report_status"] == "FAILED"


@pytest.mark.parametrize("state", [
    "REJECTED_LOCAL_SOURCE_SUPPORT", "REJECTED_POSTPROCESS_SOURCE_SUPPORT", "REJECTED_INPUT_DRIFT",
])
def test_explicit_probe_rejection_with_child_zero_returns_two(tmp_path, monkeypatch, state):
    payload = report(state)
    payload["local_support_audit"] = {"status": "FAIL", "candidate_accepted": False}
    fake = fake_probe(tmp_path, "(args.out/'report.json').write_text(" +
                      repr(json.dumps(payload)) + ",encoding='utf-8')\n")
    monkeypatch.setattr(R, "PROBE", fake)
    result = R.run_experiment(tmp_path / "run", timeout_s=15)
    assert result["exit_code"] == 2
    assert result["status"] == "REJECTED"
    assert result["report_status"] == state
    assert result["experiment_complete"] is False


def prepare_complete_fake_worker(complete_output, tmp_path, after_copy=""):
    fake = fake_probe(
        tmp_path,
        "import shutil\n"
        "shutil.copytree(" + repr(str(complete_output)) + ",args.out,dirs_exist_ok=True)\n"
        "for name,key,value in [('candidate.quality-report.json','artifact',str(args.out/'candidate.xodr')),"
        "('scoreboard.json','run_dir',str(args.out))]:\n"
        "    path=args.out/name\n"
        "    value_json=json.loads(path.read_text(encoding='utf-8'))\n"
        "    value_json[key]=value\n"
        "    path.write_text(json.dumps(value_json),encoding='utf-8')\n" + after_copy,
    )
    # Bind the controlled worker under the same relative identity as the probe.
    R.PROBE.write_bytes(fake.read_bytes())
    refresh_code_bindings(complete_output)


def test_complete_fake_worker_run_records_bindings_without_acceptance(complete_output, tmp_path):
    prepare_complete_fake_worker(complete_output, tmp_path)
    out = tmp_path / "successful-fake-run"
    result = R.run_experiment(out, timeout_s=15)
    assert result["exit_code"] == 0, result
    assert result["candidate_accepted"] is False
    assert result["binding"]["worker"]["sha256"] == digest(R.PROBE)
    assert result["binding"]["child_command_sha256"]
    assert result["binding"]["runner"]["sha256"]
    assert set(result["binding"]["extra_dependencies"]) == set(R.EXTRA_DEPENDENCIES)
    assert result["extra_dependency_integrity_after"]["matches"] is True
    assert result["extra_dependency_integrity_after"]["issues"] == []
    old_binding = json.loads((out / "code-policy-binding.json").read_text(encoding="utf-8"))
    assert set(R.EXTRA_DEPENDENCIES).isdisjoint(old_binding), "frozen probe contract was expanded"
    persisted = json.loads(Path(result["runner_result_path"]).read_text(encoding="utf-8"))
    assert persisted == result


@pytest.mark.parametrize("relative", R.EXTRA_DEPENDENCIES)
def test_missing_external_scoring_dependency_blocks_before_child(
        complete_output, tmp_path, monkeypatch, relative):
    (R.ROOT / relative).unlink()
    launched = []
    monkeypatch.setattr(R, "_run_child", lambda *args: launched.append(args))
    result = R.run_experiment(tmp_path / "missing-dependency", timeout_s=15)
    assert result["status"] == "DEPENDENCY_UNAVAILABLE"
    assert result["exit_code"] == 1
    assert launched == []
    assert result["extra_dependency_integrity_after"]["matches"] is False
    assert any(relative in problem for problem in result["problems"])


@pytest.mark.parametrize("relative", R.EXTRA_DEPENDENCIES)
def test_external_scoring_dependency_drift_overrides_complete_fake_evaluation(
        complete_output, tmp_path, relative):
    changed = R.ROOT / relative
    prepare_complete_fake_worker(
        complete_output, tmp_path,
        "changed=Path(" + repr(str(changed)) + ")\n"
        "changed.write_bytes(changed.read_bytes()+b'changed during controlled worker')\n",
    )
    out = tmp_path / "dependency-drift"
    result = R.run_experiment(out, timeout_s=15)
    assert result["report_status"] == "EXPERIMENT_EVALUATED"
    assert result["status"] == "BINDING_CHANGED"
    assert result["exit_code"] == 1
    assert result["experiment_complete"] is False
    integrity = result["extra_dependency_integrity_after"]
    assert integrity["matches"] is False
    assert integrity["issues"] == [{"path": relative, "code": "dependency-changed"}]
    # These newly bound inputs belong to the wrapper, not the frozen report.
    assert R.assess_output(out, 0)["exit_code"] == 0


def test_external_dependency_removed_during_fake_worker_fails(complete_output, tmp_path):
    relative = "esmini/bin/esminiRMLib.dll"
    prepare_complete_fake_worker(complete_output, tmp_path,
                                 "Path(" + repr(str(R.ROOT / relative)) + ").unlink()\n")
    result = R.run_experiment(tmp_path / "dependency-removed", timeout_s=15)
    assert result["exit_code"] == 1
    assert result["status"] == "BINDING_CHANGED"
    issue = result["extra_dependency_integrity_after"]["issues"][0]
    assert issue["path"] == relative and issue["code"] == "dependency-unavailable"


def test_fake_worker_code_mutation_is_reported_as_binding_change(tmp_path, monkeypatch):
    fake = fake_probe(tmp_path, "Path(__file__).write_text('# changed during execution\\n')\n")
    monkeypatch.setattr(R, "PROBE", fake)
    result = R.run_experiment(tmp_path / "mutation-run", timeout_s=15)
    assert result["exit_code"] == 1
    assert result["status"] == "BINDING_CHANGED"
    assert any("worker changed" in problem for problem in result["problems"])


def test_existing_output_is_preserved_and_never_runs_probe(tmp_path, monkeypatch):
    marker = tmp_path / "unexpected-child.txt"
    fake = fake_probe(tmp_path, "Path(" + repr(str(marker)) + ").write_text('started')\n")
    monkeypatch.setattr(R, "PROBE", fake)
    out = tmp_path / "existing"
    out.mkdir()
    retained = out / "old-evidence.bin"
    retained.write_bytes(b"existing evidence must not change\x00")
    result = R.run_experiment(out, timeout_s=15)
    assert result["exit_code"] != 0
    assert retained.read_bytes() == b"existing evidence must not change\x00"
    assert not marker.exists()


def windows_pid_has_exited(pid):
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.WaitForSingleObject.restype = wintypes.DWORD
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    handle = kernel.OpenProcess(0x00100000, False, pid)  # SYNCHRONIZE only
    if not handle:
        assert ctypes.get_last_error() == 87  # ERROR_INVALID_PARAMETER: no such PID
        return True
    try:
        return kernel.WaitForSingleObject(handle, 2000) == 0
    finally:
        kernel.CloseHandle(handle)


def cleanup_owned_fake_pid(pid):
    """Test failure must not leave its deliberately infinite fake worker behind."""
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.WaitForSingleObject.restype = wintypes.DWORD
    kernel.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
    kernel.TerminateProcess.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    handle = kernel.OpenProcess(0x00100001, False, pid)
    if handle:
        try:
            if kernel.WaitForSingleObject(handle, 0) == 258:
                kernel.TerminateProcess(handle, 99)
                kernel.WaitForSingleObject(handle, 2000)
        finally:
            kernel.CloseHandle(handle)


@pytest.mark.skipif(os.name != "nt", reason="Windows Job Object ownership contract")
def test_timeout_reaps_owned_children_and_grandchildren_only(tmp_path, monkeypatch):
    descendant = tmp_path / "controlled_descendant.py"
    descendant.write_text(
        "import os,subprocess,sys,time\nfrom pathlib import Path\n"
        "out=Path(sys.argv[1]); depth=int(sys.argv[2])\n"
        "(out/('descendant-'+str(depth)+'.pid')).write_text(str(os.getpid()))\n"
        "if depth: subprocess.Popen([sys.executable,__file__,str(out),str(depth-1)])\n"
        "while True: time.sleep(1)\n",
        encoding="utf-8",
    )
    fake = fake_probe(
        tmp_path,
        "(args.out/'probe.pid').write_text(str(os.getpid()))\n"
        "subprocess.Popen([sys.executable," + repr(str(descendant)) + ",str(args.out),'1'])\n"
        "while True: time.sleep(1)\n",
    )
    monkeypatch.setattr(R, "PROBE", fake)
    sentinel = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"],
        creationflags=subprocess.CREATE_NO_WINDOW,
    )
    out = tmp_path / "timeout-run"
    try:
        result = R.run_experiment(out, timeout_s=4)
        assert result["exit_code"] == 3
        for name in ("probe.pid", "descendant-1.pid", "descendant-0.pid"):
            pid = int((out / name).read_text())
            assert windows_pid_has_exited(pid), (name, pid)
        assert sentinel.poll() is None, "unrelated process was terminated"
    finally:
        for name in ("probe.pid", "descendant-1.pid", "descendant-0.pid"):
            path = out / name
            if path.exists():
                cleanup_owned_fake_pid(int(path.read_text()))
        sentinel.terminate()
        sentinel.wait(timeout=5)
