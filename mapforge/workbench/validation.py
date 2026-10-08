"""Whole-file validation of an accepted source-bound workbench candidate.

Only orchestration is new: G8/G11/contact decisions and draft tiers use the
existing evaluators. A completed check is not release, and T1/T2 never determine
the delivery decision. The caller owns job cancellation/staleness and attaching
the returned six-field validation record to ProjectStore.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
from uuid import uuid4

import yaml

from .compiler import (CAPABILITY_ID, ROOT, SOURCE_MANIFEST_SHA256,
                       compiler_fingerprints, register_verified_baseline)
from .contracts import canonical_bytes, content_hash, digest
from .sources import verify_source_snapshot

SOURCE_REVIEW_SHA256 = "d52311f01af70671b281d1dcdd5fecfb05cac55516434f4e66f5938318a91faf"


class ValidationRejected(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _file_hash(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validation_fingerprints():
    # These implementation hashes are separate from the already registered
    # compiler context. A new validator cannot silently inherit old checks.
    names = {"mapforge/workbench/validation.py", "mapforge/report/decision.py",
             "scripts/esmini_rm_check.py", "OpenDRIVE_1.5M.xsd"}
    names.update(p.relative_to(ROOT).as_posix() for p in (ROOT / "mapforge/validate").glob("*.py"))
    files = {name: _file_hash(ROOT / name) for name in sorted(names)}
    dll = ROOT / "esmini/bin/esminiRMLib.dll"
    files["esmini/bin/esminiRMLib.dll"] = _file_hash(dll) if dll.is_file() else None
    return {"validator_hash": digest(files), "implementation_files_sha256": files,
            "compiler_policy": compiler_fingerprints()}


def _artifact_path(directory, item):
    if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
        raise ValidationRejected("invalid-artifact", "候选产物记录格式无效")
    relative = item["relative_path"]
    if (not isinstance(relative, str) or "\\" in relative or ":" in relative
            or not relative.startswith("artifacts/")
            or any(part in {"", ".", ".."} for part in relative.split("/"))):
        raise ValidationRejected("artifact-path-escape", "候选路径必须位于工程 artifacts/ 内")
    path = directory.joinpath(*PurePosixPath(relative).parts)
    if (not path.resolve().is_relative_to(directory)
            or any(p.is_symlink() for p in [path, *path.parents] if p != directory.parent)):
        raise ValidationRejected("artifact-path-escape", "候选路径越界或包含符号链接")
    if not path.is_file() or _file_hash(path) != item["sha256"]:
        raise ValidationRejected("artifact-hash-mismatch", "实际候选产物与登记哈希不一致")
    return path


def _load_inputs(payload):
    project = payload["project"]
    snapshot, candidate = project["source_snapshot"], project.get("candidate")
    directory_arg = Path(payload["project_directory"])
    directory = directory_arg.resolve()
    if (directory_arg.is_symlink() or not directory.is_dir()
            or directory.name != project.get("project_id")):
        raise ValidationRejected("project-directory-mismatch", "服务器工程路径与工程身份不一致")
    if (not isinstance(candidate, dict) or "accepted_epoch" not in candidate
            or "accepted_revision" not in candidate
            or candidate["accepted_epoch"] != project.get("draft_epoch")
            or candidate["target_content_hash"] != project.get("content_hash")
            or project.get("content_hash") != content_hash(snapshot, project["intents"])
            or candidate["context"] != project.get("context")):
        raise ValidationRejected("candidate-stale", "必须检查当前草稿已接受且未失效的候选")
    registration = register_verified_baseline(snapshot, baseline_path=payload.get("baseline_path"),
                                                source_dir=payload.get("source_dir"),
                                                profile_path=payload.get("profile_path"))
    if registration.context(snapshot) != project["context"]:
        raise ValidationRejected("context-stale", "源/Profile/编译器/策略指纹已失效")
    artifacts = candidate.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValidationRejected("missing-artifacts", "候选没有实际产物")
    paths = [_artifact_path(directory, item) for item in artifacts]
    xodrs = [path for path in paths if path.suffix == ".xodr"]
    proofs = [path for path in paths if path.name == "evidence.json"]
    if len(xodrs) != 1 or len(proofs) != 1 or len(paths) != len(set(paths)):
        raise ValidationRejected("ambiguous-candidate", "候选须有唯一实际 XODR 与编译证据")
    evidence = json.loads(proofs[0].read_text(encoding="utf8"))
    if (evidence.get("schema") != "mapforge/workbench-compiled-draft/v1"
            or evidence.get("capability_id") != CAPABILITY_ID
            or evidence.get("candidate_id") != candidate["candidate_id"]
            or evidence.get("candidate_sha256") != _file_hash(xodrs[0])
            or evidence.get("target_content_hash") != project["content_hash"]
            or evidence.get("target_draft_epoch") != candidate["accepted_epoch"]
            or evidence.get("context") != project["context"]
            or evidence.get("local_checks", {}).get("source_binding_sha256")
               != registration.prepared.capability()["source_binding_sha256"]):
        raise ValidationRejected("compile-evidence-mismatch", "编译证据未绑定当前实际候选/草稿/来源")
    baseline = Path(registration.baseline_path)
    review = baseline.with_suffix(".source-review.json")
    if not review.is_file() or _file_hash(review) != SOURCE_REVIEW_SHA256:
        raise ValidationRejected("source-review-mismatch", "基线源复核报告缺失或改变")
    return project, registration, directory, xodrs[0], paths, review


def _write(path, data):
    temporary = path.with_name("." + path.name + "." + uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    if path.read_bytes() != data:
        raise ValidationRejected("validation-readback-failed", "检查产物实际读回不一致")


def _evaluate_pair(directory, registration, candidate_data, review_data):
    """No threshold or geometric policy is defined here."""
    from lxml import etree
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.report.decision import finalize_opendrive_g8
    from mapforge.validate import scoreboard as sb
    from mapforge.validate.g8_model import json_safe

    source = ProfileSource(registration.source_dir, registration.profile_path)
    baseline = Path(registration.baseline_path)
    manifest_bytes = baseline.with_suffix(".source-lanes.json").read_bytes()
    if _sha(manifest_bytes) != SOURCE_MANIFEST_SHA256:
        raise ValidationRejected("source-manifest-drift", "检查期间基线来源报告发生变化")
    manifest = json.loads(manifest_bytes)
    policy = yaml.safe_load(sb.POLICY.read_text(encoding="utf8"))
    schema = etree.XMLSchema(etree.parse(str(sb.XSD)))
    rows, decisions = [], {}
    for label, data in (("baseline", registration.prepared.data), ("candidate", candidate_data)):
        path = directory / (label + ".xodr")
        _write(path, data)
        _write(path.with_suffix(".source-review.json"), review_data)
        final = finalize_opendrive_g8(path, manifest,
                                      ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml")
        decisions[label] = final["decision"]
        row = {"case": label, "pipeline": "shp", "artifact": path.name}
        try:
            metrics = sb.evaluate(path, "shp", schema=schema, shp_source=source)
            row.update(metrics=metrics, tiers=sb.apply_tiers(metrics, "shp", policy))
        except Exception as exc:
            row.update(error=f"{type(exc).__name__}: {exc}", tiers={})
        rows.append(row)
    board = {"schema": "mapforge/scoreboard/v1", "run_dir": str(directory), "files": 2,
             "policy": {k: policy[k] for k in ("id", "version", "lifecycle")}, "rows": rows,
             "tier_pass": {tier: sum(row.get("tiers", {}).get(tier, {}).get("status") == "PASS"
                                       for row in rows) for tier in policy["tiers"]}}
    board = json_safe(board)
    _write(directory / "scoreboard.json", canonical_bytes(board))
    _write(directory / "scoreboard.md", sb.markdown(board).encode("utf8"))
    changes = []
    before, after = rows[0].get("metrics", {}), rows[1].get("metrics", {})
    for key in sorted(set(before) | set(after)):
        if before.get(key) != after.get(key):
            changes.append({"metric": key, "before": before.get(key), "after": after.get(key),
                            "meaning": list(sb.METRICS.get(key, ()))})
    return board, decisions, json_safe(changes)


def validate_request(payload: dict) -> dict:
    """JSON worker entry point. Never attach validation or mutate a candidate."""
    try:
        project, registration, directory, candidate_path, original_paths, review = _load_inputs(payload)
        fingerprints = validation_fingerprints()
        candidate = project["candidate"]
        candidate_hash = digest(candidate)
        candidate_data = candidate_path.read_bytes()
        input_hashes = {str(path): _file_hash(path) for path in original_paths}
        input_hashes[str(Path(registration.baseline_path))] = _file_hash(registration.baseline_path)
        input_hashes[str(review)] = _file_hash(review)
        manifest_path = Path(registration.baseline_path).with_suffix(".source-lanes.json")
        input_hashes[str(manifest_path)] = _file_hash(manifest_path)
        bundle_id = "validation-" + digest({"candidate": candidate_hash, "validator": fingerprints["validator_hash"]})[:24] + "-" + uuid4().hex[:8]
        artifacts_root = directory / "artifacts"
        if artifacts_root.is_symlink() or not artifacts_root.resolve().is_relative_to(directory):
            raise ValidationRejected("artifact-path-escape", "检查目录不得越出工程或使用符号链接")
        output = artifacts_root / bundle_id
        output.mkdir(parents=True, exist_ok=False)
        board, decisions, changes = _evaluate_pair(output, registration, candidate_data, review.read_bytes())
        # Changes while computing invalidate the result. Live draft changes are
        # checked separately by jobs/store against target revision/content/epoch.
        if (any(not Path(path).is_file() or _file_hash(path) != sha for path, sha in input_hashes.items())
                or registration.context(project["source_snapshot"]) != project["context"]
                or validation_fingerprints() != fingerprints
                or not verify_source_snapshot(project["source_snapshot"], registration.source_dir,
                                               registration.profile_path)["matches"]):
            raise ValidationRejected("validation-input-drift", "计算期间输入、策略或实现字节发生变化")
        if (output / "candidate.xodr").read_bytes() != candidate_data:
            raise ValidationRejected("candidate-copy-mismatch", "检查对象不是输入的实际候选字节")
        decision = decisions["candidate"]
        errors = [row["error"] for row in board["rows"] if row.get("error")]
        verdict = {"REVIEW_REQUIRED": "REVIEW", "BLOCKED": "BLOCKED", "DELIVERABLE": "DELIVERABLE"}.get(decision["status"])
        if verdict is None:
            raise ValidationRejected("unknown-delivery-decision", "交付裁决状态未知")
        if errors:
            verdict = "BLOCKED"
        files = [{"relative_path": path.relative_to(directory).as_posix(), "sha256": _file_hash(path)}
                 for path in sorted(output.iterdir()) if path.is_file()]
        quality = json.loads((output / "candidate.quality-report.json").read_text(encoding="utf8"))
        checks = [{"gate": gate, "status": value["status"]} for gate, value in quality["gates"].items()]
        checks.extend([
            {"gate": "static-scoreboard", "status": "UNAVAILABLE" if errors else "COMPLETED",
             "tiers": board["rows"][1].get("tiers", {}), "errors": errors,
             "meaning": "draft static tiers, not production delivery"},
            {"gate": "delivery-decision", "status": decision["status"],
             "blocked_reasons": decision.get("blocked_reasons", []),
             "review_reasons": decision.get("review_reasons", [])},
            {"gate": "validation-byte-binding", "status": "PASS",
             "validator_hash": fingerprints["validator_hash"], "files": files},
        ])
        validation = {"bundle_id": bundle_id, "candidate_id": candidate["candidate_id"],
                      "candidate_hash": candidate_hash, "context": project["context"],
                      "checks": checks, "decision": verdict}
        report = {"schema": "mapforge/workbench-whole-validation/v1", "bundle_id": bundle_id,
                  "project_id": project["project_id"], "target_revision": project["revision"],
                  "target_draft_epoch": project["draft_epoch"], "target_content_hash": project["content_hash"],
                  "candidate_hash": candidate_hash, "candidate_sha256": _sha(candidate_data),
                  "source_hash": project["context"]["source_hash"], "context": project["context"],
                  "input_files_sha256": input_hashes, "fingerprints": fingerprints,
                  "baseline_decision": decisions["baseline"], "candidate_decision": decision,
                  "scoreboard": board, "changed_metrics": changes,
                  "output_directory": str(output), "accepted": False}
        result = {"status": "VALIDATED", "validation": validation, "report": report,
                  "artifacts": files, "accepted": False}
        _write(output / "validation-result.json", canonical_bytes(result))
        return result
    except Exception as exc:
        return {"status": "REJECTED", "validation": None, "accepted": False,
                "error": {"code": getattr(exc, "code", "validation-unavailable"),
                          "message": f"{type(exc).__name__}: {exc}"}}


def verify_validation_artifacts(project_directory, validation: dict) -> dict:
    """Attach-time readback for a server-owned job result; returns its report.

    The service must additionally compare this report with the owned job and
    current draft epoch/content/candidate, and recheck the full source snapshot.
    This function rechecks all candidate/baseline input files and output files.
    """
    required = {"bundle_id", "candidate_id", "candidate_hash", "context", "checks", "decision"}
    if not isinstance(validation, dict) or set(validation) != required:
        raise ValidationRejected("invalid-validation-record", "检查包字段不完整")
    bundle_id = validation["bundle_id"]
    if not isinstance(bundle_id, str) or not re.fullmatch(r"validation-[0-9a-f]{24}-[0-9a-f]{8}", bundle_id):
        raise ValidationRejected("invalid-validation-bundle", "检查包身份无效")
    base_arg = Path(project_directory)
    base = base_arg.resolve()
    if base_arg.is_symlink() or not base.is_dir():
        raise ValidationRejected("project-directory-mismatch", "工程目录无效")
    bindings = [c for c in validation["checks"] if isinstance(c, dict)
                and c.get("gate") == "validation-byte-binding"]
    if len(bindings) != 1 or bindings[0].get("status") != "PASS":
        raise ValidationRejected("validation-binding-missing", "检查包缺少唯一字节绑定")
    fingerprints = validation_fingerprints()
    if bindings[0].get("validator_hash") != fingerprints["validator_hash"]:
        raise ValidationRejected("validator-stale", "检查器实现已改变，请重新检查")
    files = bindings[0].get("files")
    if not isinstance(files, list) or not files:
        raise ValidationRejected("validation-artifacts-missing", "检查包没有实际产物")
    prefix = f"artifacts/{bundle_id}/"
    for item in files:
        if not isinstance(item, dict) or not str(item.get("relative_path", "")).startswith(prefix):
            raise ValidationRejected("validation-artifact-scope", "检查产物不属于当前包")
        _artifact_path(base, item)
    result_path = base / "artifacts" / bundle_id / "validation-result.json"
    if (not result_path.resolve().is_relative_to(base)
            or any(p.is_symlink() for p in [result_path, *result_path.parents] if p != base.parent)):
        raise ValidationRejected("artifact-path-escape", "检查记录路径越界或使用符号链接")
    try:
        result = json.loads(result_path.read_text(encoding="utf8"))
    except (OSError, ValueError) as exc:
        raise ValidationRejected("validation-record-unavailable", "检查记录缺失或损坏") from exc
    report = result.get("report", {})
    if (result.get("status") != "VALIDATED" or result.get("validation") != validation
            or result.get("artifacts") != files or report.get("bundle_id") != bundle_id
            or report.get("project_id") != base.name
            or report.get("candidate_hash") != validation["candidate_hash"]
            or report.get("context") != validation["context"]
            or report.get("fingerprints") != fingerprints):
        raise ValidationRejected("validation-report-mismatch", "实际检查记录与提交的检查包不一致")
    inputs = report.get("input_files_sha256")
    if not isinstance(inputs, dict) or not inputs:
        raise ValidationRejected("validation-input-binding-missing", "检查记录缺少输入字节绑定")
    for path, sha in inputs.items():
        if not Path(path).is_file() or _file_hash(path) != sha:
            raise ValidationRejected("validation-input-drift", "检查完成后输入字节已改变")
    return report
