"""Fresh single-candidate checks for the accepted 0621 surface operation.

The owned daemon worker holds the existing Windows Job Object directly. Only
its gated child evaluates G8/G11 and the original scoreboard; the old compile
score is never used as this check's result. A completed check stays BLOCKED.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from uuid import uuid4

import yaml

from scripts.workbench_run_source_tracks import _run_child
from . import surface_compiler as compiler
from .contracts import canonical_bytes, content_hash, digest, require_json, sha256
from .validation import ValidationRejected

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "mapforge/workbench-surface-whole-validation/v1"
REQUEST_SCHEMA = "mapforge/workbench-surface-validation-request/v1"
WORKER_SCHEMA = "mapforge/workbench-surface-validation-worker/v1"
BASELINE = {"status": "UNAVAILABLE", "reason": "unavailable-original-generation-failed",
            "complete_xodr_available": False, "metrics": None, "tiers": None}
_BUNDLE = r"validation-[0-9a-f]{24}-[0-9a-f]{8}"
_OUTPUTS = frozenset({"request.json", "worker-result.json", "process.json", "worker.stdout.txt",
    "worker.stderr.txt", "candidate.xodr", "candidate.source-lanes.json", "candidate.source-review.json",
    "candidate.g8.json", "candidate.g11.json", "candidate.edge-contacts.json",
    "candidate.quality-report.json", "candidate.delivery-decision.json", "scoreboard.json", "scoreboard.md"})
_PACKAGE_FIELDS = {"bundle_id", "candidate_id", "candidate_hash", "context", "checks", "decision"}
MAX_FILE_BYTES = 64 * 1024 * 1024


def _reject(code, message):
    raise ValidationRejected(code, message)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _read(path):
    path = Path(path)
    if (not path.is_file() or path.stat().st_size > MAX_FILE_BYTES
            or any(p.is_symlink() for p in (path, *path.parents))):
        _reject("validation-file-unavailable", "检查文件缺失、过大或使用链接")
    data = path.read_bytes()
    if len(data) > MAX_FILE_BYTES:
        _reject("validation-file-unavailable", "检查文件过大")
    return data


def _json(data):
    value = compiler.runner._json(data)
    require_json(value)
    return value


def _write_new(path, data):
    compiler._write_new(Path(path), data)
    if _read(path) != data:
        _reject("validation-readback-failed", "新检查产物实际读回不一致")


def _artifact(directory, item):
    if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
        _reject("invalid-artifact", "检查产物清单格式无效")
    sha256(item["sha256"], "artifact.sha256")
    path = compiler._relative_file(directory, item["relative_path"])
    data = _read(path)
    if _sha(data) != item["sha256"]:
        _reject("artifact-hash-mismatch", "实际产物字节与清单不一致")
    return path, data


def validation_fingerprints():
    from mapforge.validate import scoreboard as sb
    names = {"mapforge/workbench/surface_validation.py", "mapforge/workbench/surface_checking.py",
             "mapforge/workbench/consumer_metrics.py", "mapforge/report/decision.py",
             "scripts/workbench_esmini_portable.py", "scripts/esmini_rm_check.py",
             "scripts/workbench_run_source_cells.py", "scripts/workbench_run_source_tracks.py",
             "OpenDRIVE_1.5M.xsd", "esmini/bin/esminiRMLib.dll"}
    names.update(path.relative_to(ROOT).as_posix() for path in (ROOT / "mapforge/validate").glob("*.py"))
    files = {name: _sha(_read(ROOT / name)) for name in sorted(names)}
    files[Path(sb.POLICY).relative_to(ROOT).as_posix()] = _sha(_read(sb.POLICY))
    policy = compiler.compiler_fingerprints()
    return {"validator_hash": digest({"files": files, "compiler_policy": policy}),
            "implementation_files_sha256": files, "compiler_policy": policy}


def _load_inputs(payload):
    require_json(payload)
    if (not isinstance(payload, dict) or set(payload) - {"project", "project_directory", "source_dir", "profile_path"}
            or not {"project", "project_directory"} <= set(payload)):
        _reject("invalid-validation-request", "单候选检查不接受基线、旧检查包或额外路径")
    project = payload["project"]
    if not isinstance(project, dict):
        _reject("invalid-project", "缺少完整工程")
    directory = compiler._directory(payload["project_directory"])
    if directory.name != project.get("project_id") or not re.fullmatch(r"[0-9a-f]{32}", directory.name):
        _reject("project-directory-mismatch", "工程目录与工程身份不一致")
    snapshot, candidate = project["source_snapshot"], project.get("candidate")
    if (not isinstance(candidate, dict) or type(candidate.get("accepted_revision")) is not int
            or type(candidate.get("accepted_epoch")) is not int
            or type(project.get("revision")) is not int or type(project.get("draft_epoch")) is not int
            or not 0 <= candidate["accepted_revision"] <= project["revision"]
            or candidate["accepted_epoch"] != project["draft_epoch"]
            or candidate.get("target_content_hash") != project.get("content_hash")
            or project.get("content_hash") != content_hash(snapshot, project["intents"])
            or candidate.get("context") != project.get("context")
            or project.get("status", {}).get("read_only")
            or project.get("status", {}).get("candidate_stale")):
        _reject("candidate-stale", "单候选检查要求当前草稿已接受且未失效的实际候选")
    registration = compiler.register_source(snapshot, payload.get("source_dir"), payload.get("profile_path"))
    if registration.context(snapshot) != project["context"]:
        _reject("context-stale", "来源、实现或策略指纹已改变")
    verified = compiler.verify_candidate_artifacts(directory, candidate)
    if (verified["source_snapshot"]["content_hash"] != snapshot["content_hash"]
            or verified["source_snapshot"]["snapshot_id"] != snapshot["snapshot_id"]):
        _reject("candidate-source-mismatch", "候选来源不属于当前工程")
    primary = Path(verified["primary_path"])
    manifest, review = primary.with_suffix(".source-lanes.json"), primary.with_suffix(".source-review.json")
    inputs = {}
    for item in candidate["artifacts"]:
        path, data = _artifact(directory, item)
        if item["relative_path"] in inputs:
            _reject("duplicate-candidate-artifact", "候选清单含重复产物")
        inputs[item["relative_path"]] = _sha(data)
    if any(p.relative_to(directory).as_posix() not in inputs for p in (primary, manifest, review)):
        _reject("missing-source-sidecars", "主候选、来源清单和源复核必须同时绑定")
    return {"project": project, "directory": directory, "registration": registration,
            "primary": primary, "manifest": manifest, "review": review,
            "input_files_sha256": inputs, "candidate_sha256": inputs[primary.relative_to(directory).as_posix()]}


def _same_inputs(before, payload, fingerprints):
    after = _load_inputs(payload)
    if (after["input_files_sha256"] != before["input_files_sha256"]
            or after["candidate_sha256"] != before["candidate_sha256"]
            or validation_fingerprints() != fingerprints):
        _reject("validation-input-drift", "检查期间输入、实现或策略发生变化")
    return after


def _evaluate_single(output, inputs):
    """Run real existing evaluators once on the same accepted XML bytes."""
    from lxml import etree
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.report.decision import finalize_opendrive_g8
    from mapforge.validate import scoreboard as sb
    from mapforge.validate.g8_model import json_safe
    from .consumer_metrics import evaluate

    candidate = output / "candidate.xodr"
    _write_new(candidate, _read(inputs["primary"]))
    _write_new(candidate.with_suffix(".source-review.json"), _read(inputs["review"]))
    manifest = _json(_read(inputs["manifest"]))
    # The original finalizer writes the same manifest plus G8/G11/contact reports.
    finalize_opendrive_g8(candidate, manifest, ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml")
    registration = inputs["registration"]
    source = ProfileSource(registration.source_dir, registration.profile_path)
    schema = etree.XMLSchema(etree.parse(str(sb.XSD)))
    policy = yaml.safe_load(_read(sb.POLICY))
    metrics, transport = evaluate(candidate, "shp", schema=schema, shp_source=source)
    metrics = json_safe(metrics)
    row = {"case": inputs["project"]["source_snapshot"]["junction_id"], "pipeline": "shp",
           "artifact": "candidate.xodr", "metrics": metrics, "tiers": sb.apply_tiers(metrics, "shp", policy),
           "consumer_transport": transport}
    board = json_safe({"schema": "mapforge/scoreboard/v1", "run_dir": str(output), "files": 1,
        "policy": {key: policy[key] for key in ("id", "version", "lifecycle")}, "rows": [row],
        "tier_pass": {tier: int(row["tiers"][tier]["status"] == "PASS") for tier in policy["tiers"]}})
    _write_new(output / "scoreboard.json", canonical_bytes(board))
    _write_new(output / "scoreboard.md", sb.markdown(board).encode("utf8"))


def _request_inputs(path, expected_sha=None):
    path = Path(path)
    raw = _read(path)
    if expected_sha is not None and _sha(raw) != expected_sha:
        _reject("request-hash-mismatch", "后台检查请求字节已改变")
    request = _json(raw)
    if set(request) != {"schema", "payload", "bundle_id", "fingerprints", "input_files_sha256", "candidate_sha256"}:
        _reject("invalid-worker-request", "后台检查请求字段不完整")
    if request["schema"] != REQUEST_SCHEMA or not re.fullmatch(_BUNDLE, request["bundle_id"]):
        _reject("invalid-worker-request", "后台检查请求类型或包身份无效")
    inputs = _load_inputs(request["payload"])
    output = inputs["directory"] / "artifacts" / request["bundle_id"]
    if path.resolve() != output / "request.json" or path.name != "request.json":
        _reject("worker-request-scope", "后台检查请求必须位于自己的工程检查包内")
    if (request["fingerprints"] != validation_fingerprints()
            or request["input_files_sha256"] != inputs["input_files_sha256"]
            or request["candidate_sha256"] != inputs["candidate_sha256"]):
        _reject("worker-input-mismatch", "后台检查请求与当前实际输入不一致")
    expected_prefix = "validation-" + digest({"candidate": digest(inputs["project"]["candidate"]),
        "validator": request["fingerprints"]["validator_hash"]})[:24] + "-"
    if not request["bundle_id"].startswith(expected_prefix):
        _reject("validation-bundle-binding-mismatch", "检查包身份不属于此候选与检查器")
    return request, inputs, output, _sha(raw)


def _worker_request(path, expected_sha):
    request, inputs, output, request_sha = _request_inputs(path, expected_sha)
    if {p.name for p in output.iterdir()} != {"request.json"}:
        _reject("validation-output-not-new", "后台重评只允许新的空检查包")
    _evaluate_single(output, inputs)
    _same_inputs(inputs, request["payload"], request["fingerprints"])
    if _sha(_read(path)) != request_sha:
        _reject("request-drift", "检查期间请求发生变化")
    value = {"schema": WORKER_SCHEMA, "status": "EVALUATED", "request_sha256": request_sha,
             "candidate_sha256": inputs["candidate_sha256"], "fingerprints": request["fingerprints"],
             "input_files_sha256": inputs["input_files_sha256"], "source_unchanged": True,
             "baseline": BASELINE, "accepted": False, "formal_delivery": False}
    _write_new(output / "worker-result.json", canonical_bytes(value))


def _evaluation(output, request, inputs, request_sha):
    """Cross-check persisted evaluator outputs, never infer success from exit 0."""
    from mapforge.validate import scoreboard as sb
    worker = _json(_read(output / "worker-result.json"))
    expected_worker = {"schema": WORKER_SCHEMA, "status": "EVALUATED", "request_sha256": request_sha,
        "candidate_sha256": inputs["candidate_sha256"], "fingerprints": request["fingerprints"],
        "input_files_sha256": inputs["input_files_sha256"], "source_unchanged": True,
        "baseline": BASELINE, "accepted": False, "formal_delivery": False}
    if worker != expected_worker:
        _reject("worker-evidence-mismatch", "后台检查报告与候选、来源或请求不一致")
    if (_read(output / "candidate.xodr") != _read(inputs["primary"])
            or _read(output / "candidate.source-review.json") != _read(inputs["review"])
            or _json(_read(output / "candidate.source-lanes.json")) != _json(_read(inputs["manifest"]))):
        _reject("candidate-copy-mismatch", "整图检查没有使用原实际候选及其来源报告")
    board = _json(_read(output / "scoreboard.json"))
    policy = yaml.safe_load(_read(sb.POLICY))
    rows = board.get("rows")
    if (board.get("schema") != "mapforge/scoreboard/v1" or type(board.get("files")) is not int
            or board["files"] != 1 or board.get("run_dir") != str(output)
            or board.get("policy") != {key: policy[key] for key in ("id", "version", "lifecycle")}
            or not isinstance(rows, list) or len(rows) != 1 or not isinstance(rows[0], dict)):
        _reject("invalid-single-scoreboard", "检查必须仅包含本次候选的完整单行评分")
    row = rows[0]
    if (row.get("case") != inputs["project"]["source_snapshot"]["junction_id"]
            or row.get("pipeline") != "shp" or row.get("artifact") != "candidate.xodr"
            or row.get("error") or not isinstance(row.get("metrics"), dict) or not row["metrics"]
            or not isinstance(row.get("consumer_transport"), dict)
            or row.get("tiers") != sb.apply_tiers(row["metrics"], "shp", policy)
            or board.get("tier_pass") != {tier: int(row["tiers"][tier]["status"] == "PASS") for tier in policy["tiers"]}
            or _read(output / "scoreboard.md") != sb.markdown(board).encode("utf8")):
        _reject("scoreboard-binding-mismatch", "评分身份、原阈值分级或摘要不一致")
    quality = _json(_read(output / "candidate.quality-report.json"))
    decision = _json(_read(output / "candidate.delivery-decision.json"))
    gates = quality.get("gates")
    if (quality.get("schema") != "mapforge/opendrive-quality-report/v1"
            or quality.get("artifact") != str(output / "candidate.xodr")
            or not isinstance(gates, dict) or set(gates) != {"G8", "G11", "G11-edge-contacts"}
            or quality.get("delivery_decision") != decision
            or decision.get("schema") != "mapforge/delivery-decision/v1"
            or decision.get("status") not in {"BLOCKED", "REVIEW_REQUIRED", "DELIVERABLE"}):
        _reject("quality-decision-mismatch", "质量报告与实际候选或交付裁决不一致")
    for gate, name in (("G8", "candidate.g8.json"), ("G11", "candidate.g11.json"),
                       ("G11-edge-contacts", "candidate.edge-contacts.json")):
        actual = _json(_read(output / name))
        if actual != gates[gate] or actual.get("status") not in {"PASS", "FAIL", "UNAVAILABLE", "NOT_RUN"}:
            _reject("gate-binding-mismatch", "单项门禁与完整质量报告不一致")
    return board, quality, decision


def _file_list(directory, output):
    entries = list(output.iterdir())
    if any(not p.is_file() or p.is_symlink() for p in entries):
        _reject("validation-extra-directory", "检查包不得含额外目录或链接")
    names = {p.name for p in entries}
    if names - {"validation-result.json"} != _OUTPUTS:
        _reject("incomplete-validation-artifacts", "检查产物集合不完整或有未声明产物")
    return [{"relative_path": p.relative_to(directory).as_posix(), "sha256": _sha(_read(p))}
            for p in sorted(entries) if p.name != "validation-result.json"]


def _checks(quality, board, decision, fingerprints, files):
    return [*[{"gate": gate, "status": value["status"]} for gate, value in quality["gates"].items()],
        {"gate": "static-scoreboard", "status": "COMPLETED", "tiers": board["rows"][0]["tiers"],
         "errors": [], "meaning": "fresh single-candidate draft tiers; not production delivery"},
        {"gate": "baseline-comparison", **BASELINE},
        {"gate": "delivery-decision", "status": decision["status"],
         "blocked_reasons": decision.get("blocked_reasons", []), "review_reasons": decision.get("review_reasons", [])},
        {"gate": "research-only", "status": "BLOCKED", "formal_delivery": False},
        {"gate": "validation-byte-binding", "status": "PASS", "validator_hash": fingerprints["validator_hash"],
         "files": files}]


def _report(request, inputs, output, board, decision):
    project = inputs["project"]
    return {"schema": SCHEMA, "bundle_id": request["bundle_id"], "project_id": project["project_id"],
        "target_revision": project["revision"], "target_draft_epoch": project["draft_epoch"],
        "target_content_hash": project["content_hash"], "candidate_hash": digest(project["candidate"]),
        "candidate_sha256": inputs["candidate_sha256"], "context": project["context"],
        "source_hash": project["context"]["source_hash"], "source_snapshot_id": project["source_snapshot"]["snapshot_id"],
        "source_content_hash": project["source_snapshot"]["content_hash"],
        "primary_artifact": inputs["primary"].relative_to(inputs["directory"]).as_posix(),
        "input_files_sha256": inputs["input_files_sha256"], "input_path_basis": "project-relative",
        "fingerprints": request["fingerprints"], "baseline_decision": BASELINE,
        "candidate_decision": decision, "scoreboard": board, "changed_metrics": None,
        "comparison_status": "unavailable-original-generation-failed", "output_directory": str(output),
        "accepted": False, "formal_delivery": False}


def _process_proof(output):
    process = _json(_read(output / "process.json"))
    if (type(process.get("exit_code")) is not int or process["exit_code"] != 0
            or process.get("timed_out") is not False
            or process.get("tree_control") != "Windows Job Object, launch gated before descendants"):
        _reject("evaluation-process-incomplete", "整图检查子进程未正常完成受归属的运行")
    return process


def validate_request(payload):
    """Owned-worker entry: produce immutable evidence, never attach or release."""
    try:
        if os.name != "nt":
            _reject("unsupported-platform", "本次完整检查要求 Windows 原生消费者")
        inputs = _load_inputs(payload)
        fingerprints = validation_fingerprints()
        candidate = inputs["project"]["candidate"]
        bundle_id = "validation-" + digest({"candidate": digest(candidate),
            "validator": fingerprints["validator_hash"]})[:24] + "-" + uuid4().hex[:8]
        output = inputs["directory"] / "artifacts" / bundle_id
        if output.parent.is_symlink() or not output.resolve().is_relative_to(inputs["directory"]):
            _reject("artifact-path-escape", "检查目录越界或使用链接")
        output.mkdir(exist_ok=False)
        request = {"schema": REQUEST_SCHEMA, "payload": copy.deepcopy(payload), "bundle_id": bundle_id,
                   "fingerprints": fingerprints, "input_files_sha256": inputs["input_files_sha256"],
                   "candidate_sha256": inputs["candidate_sha256"]}
        request_path = output / "request.json"
        data = canonical_bytes(request)
        _write_new(request_path, data)
        command = [sys.executable, "-B", "-m", "mapforge.workbench.surface_validation", "--worker-request",
                   str(request_path), "--request-sha256", _sha(data)]
        process = _run_child(command, 600)
        _write_new(output / "worker.stdout.txt", process["stdout"])
        _write_new(output / "worker.stderr.txt", process["stderr"])
        _write_new(output / "process.json", canonical_bytes({k: v for k, v in process.items()
                                                            if k not in {"stdout", "stderr"}}))
        _process_proof(output)
        _same_inputs(inputs, payload, fingerprints)
        if _read(request_path) != data:
            _reject("request-drift", "检查期间请求发生变化")
        board, quality, decision = _evaluation(output, request, inputs, _sha(data))
        files = _file_list(inputs["directory"], output)
        checks = _checks(quality, board, decision, fingerprints, files)
        package = {"bundle_id": bundle_id, "candidate_id": candidate["candidate_id"],
                   "candidate_hash": digest(candidate), "context": inputs["project"]["context"],
                   "checks": checks, "decision": "BLOCKED"}
        report = _report(request, inputs, output, board, decision)
        result = {"status": "VALIDATED", "validation": package, "report": report,
                  "artifacts": files, "accepted": False}
        _write_new(output / "validation-result.json", canonical_bytes(result))
        verify_validation_artifacts(inputs["directory"], package)
        return result
    except Exception as exc:
        return {"status": "REJECTED", "validation": None, "accepted": False,
                "error": {"code": getattr(exc, "code", "surface-validation-unavailable"),
                          "message": f"{type(exc).__name__}: {exc}"}}


def verify_validation_artifacts(project_directory, validation):
    """Verify all inputs/outputs and cross-bind current fingerprints and reports.

    Attachment additionally requires the owned job result; exports additionally
    compare validation-result.json to the sealed successful job archive.
    """
    require_json(validation)
    if (not isinstance(validation, dict) or set(validation) != _PACKAGE_FIELDS
            or validation.get("decision") != "BLOCKED"
            or not isinstance(validation.get("bundle_id"), str)
            or not re.fullmatch(_BUNDLE, validation["bundle_id"])):
        _reject("invalid-validation-record", "单候选检查包必须完整且保持 BLOCKED")
    directory = compiler._directory(project_directory)
    output = directory / "artifacts" / validation["bundle_id"]
    request, inputs, actual_output, request_sha = _request_inputs(output / "request.json")
    if actual_output != output:
        _reject("validation-output-scope", "检查包不属于当前工程")
    _process_proof(output)
    board, quality, decision = _evaluation(output, request, inputs, request_sha)
    files = _file_list(directory, output)
    checks = _checks(quality, board, decision, request["fingerprints"], files)
    candidate = inputs["project"]["candidate"]
    expected = {"bundle_id": request["bundle_id"], "candidate_id": candidate["candidate_id"],
                "candidate_hash": digest(candidate), "context": inputs["project"]["context"],
                "checks": checks, "decision": "BLOCKED"}
    if validation != expected:
        _reject("validation-binding-mismatch", "检查包未完整绑定当前实际文件、原检查结论与候选")
    report = _report(request, inputs, output, board, decision)
    result_bytes = _read(output / "validation-result.json")
    result = _json(result_bytes)
    expected_result = {"status": "VALIDATED", "validation": expected, "report": report,
                       "artifacts": files, "accepted": False}
    if result != expected_result or result_bytes != canonical_bytes(expected_result):
        _reject("validation-report-mismatch", "检查结果文件与完整包内容不一致")
    _same_inputs(inputs, request["payload"], request["fingerprints"])
    if (_file_list(directory, output) != files or _sha(_read(output / "request.json")) != request_sha
            or _read(output / "validation-result.json") != result_bytes):
        _reject("validation-output-drift", "复核期间检查证据发生变化")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Internal owned surface-validation worker")
    parser.add_argument("--worker-request", type=Path, required=True)
    parser.add_argument("--request-sha256", required=True)
    args = parser.parse_args(argv)
    try:
        sha256(args.request_sha256, "request_sha256")
        _worker_request(args.worker_request, args.request_sha256)
        return 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
