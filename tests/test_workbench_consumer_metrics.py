"""Transport-only opt-in, with no real geometry or native consumer execution."""
import copy
import hashlib
import json
from pathlib import Path

import pytest

from mapforge.workbench import consumer_metrics as C


def binding(path):
    data = path.read_bytes()
    return {"path": str(path.resolve()), "size": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}


@pytest.fixture
def controlled(tmp_path, monkeypatch):
    directory = tmp_path / "中文 空格工程"
    directory.mkdir()
    paths = {"input": directory / "候选.xodr"}
    paths.update({name: directory / name for name in
                  ("dll", "checker", "adapter", "scoreboard", "metrics_adapter")})
    for name, path in paths.items():
        path.write_bytes(("fixture " + name).encode())
    original = {"esmini_pass": False, "esmini_gap_max_cm": None,
                "xsd_pass": True, "geometry_error": 0.000031,
                "unavailable_metric": None, "loss_count": 3,
                "unexpected_metric": {"status": "FAIL", "values": [1, 2]}}
    bindings = {name: binding(paths[name]) for name in ("input", "dll", "checker", "adapter")}
    line = ("候选.xodr: 换乘缝隙 max 进侧 0.0cm / 出侧 0.1cm（6 对）；"
            "行驶穿越 5 次，跳变 0，借道弃计 0，灭车道并线 0  -> PASS")
    consumer = {
        "schema": "mapforge/esmini-portable-check/v1", "status": "CHECKED",
        "pass": True, "gap_max_cm": 0.1, "line": line,
        "evidence": {"before": copy.deepcopy(bindings), "after": copy.deepcopy(bindings),
                     "unchanged": True, "loader": "RM_InitWithString", "exact_bytes": True,
                     "full_check_executed": True, "worker_exit_code": 0,
                     "stdout": line + "\n", "stderr": ""},
    }
    calls = []

    def scoreboard(path, pipeline, **kwargs):
        calls.append(("scoreboard", path, pipeline, kwargs))
        return original

    def check(path):
        calls.append(("consumer", path))
        return consumer

    monkeypatch.setattr(C, "_binding_paths", lambda path: paths)
    monkeypatch.setattr(C.sb, "evaluate", scoreboard)
    monkeypatch.setattr(C, "_check_file", check)
    return paths, original, consumer, calls


def run(controlled):
    return C.evaluate(controlled[0]["input"], "shp")


def test_only_two_metrics_change_without_mutating_original(controlled):
    paths, original, consumer, calls = controlled
    saved = copy.deepcopy(original)
    source, schema = object(), object()
    metrics, evidence = C.evaluate(paths["input"], "shp", schema=schema, shp_source=source)
    assert metrics["esmini_pass"] is True and metrics["esmini_gap_max_cm"] == 0.1
    assert original == saved
    for name in set(original) - {"esmini_pass", "esmini_gap_max_cm"}:
        assert metrics[name] is original[name]
    assert set(metrics) == set(original)
    assert evidence["status"] == "CHECKED" and evidence["unchanged"] is True
    assert evidence["legacy_consumer_metrics"] == {
        "esmini_pass": {"present": True, "value": False},
        "esmini_gap_max_cm": {"present": True, "value": None},
    }
    assert evidence["consumer_result"] == consumer
    assert evidence["before"] == evidence["after"]
    assert calls == [("scoreboard", paths["input"], "shp", {"schema": schema, "shp_source": source}),
                     ("consumer", paths["input"])]
    json.dumps(evidence, allow_nan=False)


def test_completed_geometry_failure_keeps_actual_gap(controlled):
    consumer = controlled[2]
    consumer.update({"pass": False, "gap_max_cm": 21.3,
                     "line": consumer["line"].replace("0.1cm", "21.3cm").replace("PASS", "CHECK")})
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] == 21.3
    assert evidence["status"] == "CHECKED" and not evidence["problems"]


@pytest.mark.parametrize("key,value", [
    ("status", "FAILED"), ("status", "TIMED_OUT"), ("status", "REJECTED_INPUT"),
    ("schema", "mapforge/esmini-portable-check/v2"), ("pass", 1),
    ("gap_max_cm", None), ("gap_max_cm", True), ("gap_max_cm", -1),
    ("gap_max_cm", float("nan")), ("gap_max_cm", float("inf")), ("gap_max_cm", 0.2),
    ("line", "loaded successfully -> PASS"),
    ("line", "other.xodr: 换乘缝隙 max 进侧 0.0cm / 出侧 0.1cm -> PASS"),
    ("line", "候选.xodr: 换乘缝隙 max 进侧 0.0cm / 出侧 0.1cm -> CHECK"),
    ("line", "候选.xodr: 换乘缝隙 max 进侧 0..0cm / 出侧 0.1cm -> PASS"),
    ("evidence", None),
])
def test_invalid_return_cannot_retain_old_pass(controlled, key, value):
    controlled[1].update(esmini_pass=True, esmini_gap_max_cm=0.0)
    controlled[2][key] = value
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] is None
    assert metrics["unexpected_metric"] is controlled[1]["unexpected_metric"]
    assert evidence["status"] == "INVALID" and evidence["problems"]
    assert evidence["legacy_consumer_metrics"]["esmini_pass"]["value"] is True
    json.dumps(evidence, allow_nan=False)


@pytest.mark.parametrize("key", ["pass", "gap_max_cm", "line", "status", "schema", "evidence"])
def test_missing_result_field_is_not_success(controlled, key):
    del controlled[2][key]
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] is None
    assert evidence["consumer_result"] == controlled[2]


@pytest.mark.parametrize("key,value", [
    ("full_check_executed", False), ("full_check_executed", 1), ("exact_bytes", False),
    ("unchanged", False), ("loader", "RM_Init"), ("worker_exit_code", 1),
    ("worker_exit_code", False), ("stdout", None), ("stderr", None),
])
def test_loading_alone_and_incomplete_process_evidence_are_not_checks(controlled, key, value):
    controlled[2]["evidence"][key] = value
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] is None
    assert evidence["status"] == "INVALID"


@pytest.mark.parametrize("phase", ["before", "after"])
@pytest.mark.parametrize("name", ["input", "dll", "checker", "adapter"])
def test_reported_binding_must_match_actual_dependency(controlled, phase, name):
    controlled[2]["evidence"][phase][name]["sha256"] = "0" * 64
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] is None
    assert evidence["unchanged"] is True  # Files were stable, returned binding was false.


def test_same_bytes_at_another_input_path_are_not_the_bound_input(controlled):
    paths, _, consumer, _ = controlled
    duplicate = paths["input"].with_name("其他.xodr")
    duplicate.write_bytes(paths["input"].read_bytes())
    for phase in ("before", "after"):
        consumer["evidence"][phase]["input"] = binding(duplicate)
    assert run(controlled)[0]["esmini_pass"] is False


@pytest.mark.parametrize("name", ["input", "dll", "checker", "adapter", "scoreboard", "metrics_adapter"])
def test_disk_drift_during_consumer_invalidates_result(controlled, monkeypatch, name):
    def drifting_check(path):
        controlled[0][name].write_bytes(b"changed during consumer")
        return controlled[2]
    monkeypatch.setattr(C, "_check_file", drifting_check)
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] is None
    assert evidence["unchanged"] is False
    assert evidence["before"][name] != evidence["after"][name]


def test_input_changed_between_original_scoring_and_consumer_is_not_combined(controlled, monkeypatch):
    paths, original, consumer, _ = controlled
    def original_check(*args, **kwargs):
        paths["input"].write_bytes(b"a different candidate")
        for phase in ("before", "after"):
            consumer["evidence"][phase]["input"] = binding(paths["input"])
        return original
    monkeypatch.setattr(C.sb, "evaluate", original_check)
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and evidence["unchanged"] is False


def test_missing_dependency_is_not_stable_evidence(controlled):
    controlled[0]["dll"].unlink()
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and evidence["unchanged"] is False
    assert evidence["before"]["dll"] is None and evidence["problems"]


def test_consumer_exception_retains_original_metrics_and_diagnostic(controlled, monkeypatch):
    def broken(path):
        raise OSError("native worker could not start")
    monkeypatch.setattr(C, "_check_file", broken)
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and metrics["esmini_gap_max_cm"] is None
    assert evidence["status"] == "FAILED"
    assert evidence["error"] == {"type": "OSError", "message": "native worker could not start"}
    assert metrics["geometry_error"] is controlled[1]["geometry_error"]


@pytest.mark.parametrize("value", [None, True, [], "PASS"])
def test_non_object_consumer_return_is_failure(controlled, monkeypatch, value):
    monkeypatch.setattr(C, "_check_file", lambda path: value)
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is False and evidence["consumer_result"] == value


def test_original_scorer_exception_propagates_without_running_consumer(controlled, monkeypatch):
    def broken(*args, **kwargs):
        raise ValueError("original XML invalid")
    monkeypatch.setattr(C.sb, "evaluate", broken)
    with pytest.raises(ValueError, match="original XML invalid"):
        run(controlled)
    assert controlled[3] == []


def test_legacy_missing_is_distinct_from_explicit_none(controlled):
    del controlled[1]["esmini_gap_max_cm"]
    metrics, evidence = run(controlled)
    assert metrics["esmini_pass"] is True
    assert evidence["legacy_consumer_metrics"]["esmini_gap_max_cm"] == {"present": False, "value": None}


def test_default_dispatch_uses_portable_function_without_native_call(tmp_path, monkeypatch):
    from scripts import workbench_esmini_portable as portable
    sentinel = {"status": "controlled dispatch"}
    called = []
    def check(path):
        called.append(path)
        return sentinel
    monkeypatch.setattr(portable, "check_file", check)
    path = tmp_path / "候选.xodr"
    assert C._check_file(path) is sentinel
    assert called == [path]
