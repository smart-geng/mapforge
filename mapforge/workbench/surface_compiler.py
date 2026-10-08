"""Fresh-source compiler adapter for the registered 0621 auxiliary operation.

This module does not mutate drafts or accept candidates. A COMPILED response
means a newly generated research candidate has complete, verified evidence;
the existing FAIL scores and BLOCKED delivery decision remain visible. The
research runner's automatic ``confirmed`` flag is not a user transaction.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tomllib
from uuid import uuid4

from scripts import workbench_run_source_tracks as runner
from . import source_surface_partition as P
from . import source_surface_reconstruction as R
from .contracts import (canonical_bytes, content_hash, context_for, digest, identifier,
                        require_json, sha256, validate_command, validate_context, validate_snapshot)
from .sources import verify_source_snapshot
from .surface_review import RegisteredSurfaceReview

ROOT = Path(__file__).resolve().parents[2]
CAPABILITY_ID = "0621-source-supported-surface-v1"
INTENT_TYPE = "source_supported_surface_tracks_v2"
SCHEMA = "mapforge/workbench-surface-compiled-draft/v1"
SOURCE_OBJECT_COUNT = 210
_DRAFT_FIELDS = {"source_snapshot", "revision", "draft_epoch", "intents", "content_hash", "context"}
_CANDIDATE_FIELDS = {"candidate_id", "target_content_hash", "context", "artifacts", "affected_ids", "checks"}


class SurfaceCompilerRejected(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _reject(code, message):
    raise SurfaceCompilerRejected(code, message)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _file_hash(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _source_content(snapshot):
    return digest({k: v for k, v in snapshot.items() if k not in {"locator", "content_hash"}})


def _paths(source_dir=None, profile_path=None):
    source = Path(source_dir or ROOT / "shp_0222-0326")
    profile = Path(profile_path or ROOT / "profiles/shp/ibd-smarteditor-v1.yaml")
    for path in (source, profile):
        if any(p.is_symlink() for p in (path, *path.parents)):
            _reject("source-path-link", "原料与 Profile 路径不得使用链接")
    source, profile = source.resolve(), profile.resolve()
    if (source != (ROOT / "shp_0222-0326").resolve()
            or profile != (ROOT / "profiles/shp/ibd-smarteditor-v1.yaml").resolve()
            or ROOT.resolve() != runner.ROOT.resolve()):
        _reject("runner-source-path-mismatch", "登记路径必须与实际固定研究运行器的原料和 Profile 完全一致")
    return source, profile


def _runtime_versions():
    """Verify installed core distribution versions against both pinned inputs.

    This is a version check, not a byte audit of installed wheels. Runtime
    distribution provenance remains a separate installation/release check.
    """
    project = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    lock = tomllib.loads((ROOT / "uv.lock").read_text(encoding="utf-8"))
    normalize = lambda name: re.sub(r"[-_.]+", "-", name).lower()
    locked = {}
    for package in lock.get("package", []):
        locked.setdefault(normalize(package["name"]), set()).add(package.get("version"))
    requirements = project.get("project", {}).get("dependencies")
    if not isinstance(requirements, list) or not requirements:
        _reject("runtime-dependencies-unavailable", "缺少已固定的核心依赖清单")
    versions = {}
    for requirement in requirements:
        matched = re.fullmatch(r"([A-Za-z0-9_.-]+)==([^\s;]+)", requirement)
        if matched is None:
            _reject("runtime-dependency-not-pinned", "核心依赖必须逐项固定版本")
        name, expected = matched.groups()
        key = normalize(name)
        if key in versions or locked.get(key) != {expected}:
            _reject("runtime-lock-mismatch", "核心依赖声明与 uv.lock 不一致: " + name)
        try:
            actual = metadata.version(name)
        except metadata.PackageNotFoundError as exc:
            raise SurfaceCompilerRejected("runtime-package-missing", "缺少当前核心依赖: " + name) from exc
        if actual != expected:
            _reject("runtime-version-mismatch", "当前依赖版本与锁定值不一致: " + name)
        versions[key] = actual
    return {"python": sys.version.split()[0], "implementation": sys.implementation.name,
            "core_distributions": versions, "installed_package_bytes_verified": False}


def compiler_fingerprints():
    """Bind repository inputs and verified installed versions, not node16's context."""
    paths = {*runner._required_code_paths(), *(ROOT / name for name in runner.EXTRA_DEPENDENCIES),
             Path(__file__).resolve(), ROOT / "pyproject.toml"}
    files = {}
    for path in sorted(paths):
        if (not path.is_file() or any(p.is_symlink() for p in (path, *path.parents))
                or not path.resolve().is_relative_to(ROOT.resolve())):
            _reject("compiler-dependency-unavailable", "编译依赖缺失或路径不受支持: " + str(path))
        files[path.relative_to(ROOT).as_posix()] = _file_hash(path)
    policies = {name: sha for name, sha in files.items()
                if name.startswith("profiles/") or name.endswith(".xsd")}
    runtime = _runtime_versions()
    return {"compiler_hash": digest({"files": files, "runtime": runtime}),
            "policy_hash": digest(policies), "code_files_sha256": files,
            "policy_files_sha256": policies, "runtime": runtime}


def _check_snapshot(snapshot):
    validate_snapshot(snapshot)
    if (snapshot.get("schema") != "mapforge/workbench-source/v1"
            or snapshot.get("content_hash") != _source_content(snapshot)
            or len(snapshot["objects"]) != SOURCE_OBJECT_COUNT):
        _reject("source-snapshot-mismatch", "源快照内容或完整依赖对象集合不匹配")
    # The existing source-bound operation checks dataset, complete snapshot and
    # junction identity against its registered constants before any conversion.
    intent = R.make_intent(snapshot)
    profile_hash = snapshot.get("profile", {}).get("sha256")
    sha256(profile_hash, "profile.sha256")
    if digest({"files": snapshot.get("source_files"), "profile_sha256": profile_hash}) != snapshot["snapshot_id"]:
        _reject("source-file-manifest-mismatch", "原始文件清单与源数据集身份不一致")
    for obj in snapshot["objects"]:
        ref = obj["source_ref"]
        expected = "src:" + digest([snapshot["snapshot_id"], ref.get("layer"),
                                     ref.get("record_index"), ref.get("part_index")])
        if obj["id"] != expected:
            _reject("source-object-identity-mismatch", "源对象记录身份与内容索引不一致")
    polygons = [obj for obj in snapshot["objects"]
                if obj["source_ref"].get("layer") == "IBD_OBJECT_INTERSECTION_SURFACE"
                and obj.get("role") == "junction" and obj.get("business_id") == snapshot["junction_id"]
                and obj["source_ref"].get("part_index") == 0
                and isinstance(obj.get("points"), list) and len(obj["points"]) >= 4]
    if len(polygons) != 1:
        _reject("source-junction-polygon-ambiguous", "必须有唯一实际路口面原始记录作为操作目标")
    return intent, polygons[0]["id"], tuple(sorted(obj["id"] for obj in snapshot["objects"]))


@dataclass(frozen=True)
class RegisteredSurfaceCapability:
    snapshot_content_hash: str
    source_ref: str
    feature_ids: tuple[str, ...]
    operation_hash: str
    operation_json: bytes
    source_dir: str
    profile_path: str

    def context(self, snapshot):
        operation, target, dependencies = _check_snapshot(snapshot)
        if (snapshot["content_hash"] != self.snapshot_content_hash or target != self.source_ref
                or dependencies != self.feature_ids or digest(operation) != self.operation_hash):
            _reject("registration-source-changed", "操作登记不属于该源快照")
        _paths(self.source_dir, self.profile_path)
        fingerprints = compiler_fingerprints()
        return context_for(snapshot, fingerprints["compiler_hash"], fingerprints["policy_hash"])

    def store_spec(self):
        return {"capability_id": CAPABILITY_ID, "type": INTENT_TYPE, "source_ref": self.source_ref,
                "feature_ids": list(self.feature_ids), "source_content_hash": self.snapshot_content_hash,
                "operation_hash": self.operation_hash}

    def command_template(self, command_id):
        identifier(command_id, "command_id")
        return {"command_id": command_id, "type": INTENT_TYPE, "source_ref": self.source_ref,
                "scope": {"capability_id": CAPABILITY_ID, "feature_ids": list(self.feature_ids)},
                "parameters": {"operation_hash": self.operation_hash,
                               "source_content_hash": self.snapshot_content_hash}}

    def capability(self):
        return {**self.store_spec(), "intent_type": INTENT_TYPE,
            "scope": {"capability_id": CAPABILITY_ID, "feature_ids": list(self.feature_ids)},
            "operation": json.loads(self.operation_json),
            "actions": ["撤回 road 12 离去侧无来源支撑的扫掠", "依两条原始内边界补全分隔带",
                        "按真实父子来源面重新表达整个路口辅助铺面"],
            "dependency_scope": "all 210 source objects; originals remain unchanged",
            "coordinate_space": "fresh-source-derived local metres; absolute CRS unverified",
            "input_complete_xodr_required": False, "preview_requires_fresh_source_generation": True,
            "human_confirmation_verified": False, "automatic_probe_flag_is_user_confirmation": False,
            "formal_export_available": False}


def register_source(snapshot, source_dir=None, profile_path=None):
    operation, target, dependencies = _check_snapshot(snapshot)
    source, profile = _paths(source_dir, profile_path)
    if sys.version_info[:3] != (3, 11, 16):
        _reject("unsupported-runtime", "该操作要求 Python 3.11.16")
    if verify_source_snapshot(snapshot, source, profile).get("matches") is not True:
        _reject("source-drift", "原件或 Profile 已变化，不能登记编译能力")
    return RegisteredSurfaceCapability(snapshot["content_hash"], target, dependencies, digest(operation),
                                       canonical_bytes(operation), str(source), str(profile))


def _validate_draft(draft, registration):
    require_json(draft, "draft")
    if not isinstance(draft, dict) or set(draft) != _DRAFT_FIELDS:
        _reject("invalid-draft-fields", "编译草稿必须完整绑定六项输入，不能附带旧候选或路径")
    for key in ("revision", "draft_epoch"):
        if type(draft[key]) is not int or draft[key] < 0:
            _reject("invalid-draft-version", "草稿版本和 epoch 必须是非负整数")
    if not isinstance(draft["intents"], list):
        _reject("invalid-intents", "草稿意图必须为列表")
    if content_hash(draft["source_snapshot"], draft["intents"]) != draft["content_hash"]:
        _reject("draft-hash-mismatch", "草稿内容与目标哈希不一致")
    validate_context(draft["context"])
    if registration.context(draft["source_snapshot"]) != draft["context"]:
        _reject("stale-context", "源、Profile、实现或策略指纹已失效")
    seen, geometric = set(), []
    for command in draft["intents"]:
        if not isinstance(command, dict):
            _reject("invalid-intent", "意图必须为对象")
        command_id = identifier(command.get("command_id"), "command_id")
        if command_id in seen:
            _reject("duplicate-command", "同一事务命令不能重复编译")
        seen.add(command_id)
        if command.get("type") == "annotation":
            if validate_command(command, draft["source_snapshot"], draft["revision"]) != command:
                _reject("noncanonical-annotation", "编译需要已规范化的待办意图")
        elif command.get("type") == INTENT_TYPE:
            if command != registration.command_template(command_id):
                _reject("operation-scope-mismatch", "操作参数、来源或完整依赖范围与登记不一致")
            geometric.append(command_id)
        else:
            _reject("unsupported-intent", "草稿含未登记的几何操作")
    if len(geometric) != 1:
        _reject("exactly-one-surface-operation-required", "该失败源工程要求且仅允许一次明确的辅助面重建意图")
    if draft["revision"] < 1 or draft["draft_epoch"] < 1:
        _reject("missing-target-transaction-version", "操作预览目标或草稿必须具有非零版本和 epoch")
    return geometric


def _directory(path):
    raw = Path(path)
    if any(p.is_symlink() for p in (raw, *raw.parents)) or not raw.is_dir():
        _reject("invalid-project-directory", "工程目录不存在或使用链接")
    return raw.resolve()


def _relative_file(base, relative):
    if (not isinstance(relative, str) or "\\" in relative or ":" in relative
            or not relative.startswith("artifacts/") or PurePosixPath(relative).is_absolute()
            or any(part in {"", ".", ".."} for part in relative.split("/"))):
        _reject("artifact-path-escape", "候选产物必须位于工程 artifacts 内")
    path = base.joinpath(*relative.split("/"))
    if (not path.is_file() or not path.resolve().is_relative_to(base)
            or any(p.is_symlink() for p in (path, *path.parents))):
        _reject("artifact-unavailable", "候选产物缺失、越界或使用链接")
    return path


def _run_files(directory):
    files = []
    for path in sorted(directory.rglob("*")):
        if path.is_symlink() or not path.resolve().is_relative_to(directory):
            _reject("run-path-escape", "运行证据目录含链接或越界路径")
        if path.is_file():
            files.append(path)
    return files


def _verified_run(run_directory, snapshot):
    """Reassess actual reports and bytes, including current code/dependencies."""
    review = RegisteredSurfaceReview(run_directory)
    if not review.matches(snapshot):
        _reject("run-source-mismatch", "新运行使用的来源不属于当前草稿")
    data = review.read_bound()
    receipt = runner._json(data["runner-result.json"])
    integrity = receipt.get("extra_dependency_integrity_after", {})
    if integrity.get("matches") is not True or integrity.get("issues") != []:
        _reject("run-dependency-integrity-missing", "运行缺少完整外部依赖字节复核")
    binding = receipt.get("binding", {})
    expected_extra = {name: runner._extra_dependency_binding(name) for name in runner.EXTRA_DEPENDENCIES}
    if (binding.get("extra_dependencies") != expected_extra or integrity.get("bindings") != expected_extra
            or binding.get("runner") != runner._file_binding(Path(runner.__file__))
            or binding.get("worker") != runner._file_binding(runner.PROBE)):
        _reject("run-code-dependency-drift", "运行代码或消费者依赖已变化")
    assessment = runner.assess_output(run_directory, 0)
    if (assessment.get("status") != "EXPERIMENT_COMPLETE" or assessment.get("experiment_complete") is not True
            or assessment.get("exit_code") != 0 or assessment.get("problems") != []):
        _reject("run-incomplete-or-rejected", "新运行未通过完整证据核验: " + str(assessment.get("problems")))
    for name, value in assessment["evidence_bindings"].items():
        if review.bindings.get(name, {}).get("sha256") != value["sha256"]:
            _reject("run-assessment-binding-mismatch", "重评与运行回执字节不一致")
    diagnostic = review.diagnose()
    if diagnostic.get("available") is not True:
        _reject("run-actual-surface-unverified", "实际 XML 来源支持无法重新验证")
    review.read_bound()
    return review, assessment, data


def _checks(data):
    board = runner._json(data["scoreboard.json"])
    delivery = runner._json(data["candidate.delivery-decision.json"])
    return [{"gate": "local-source-supported-surface", "status": "PASS", "scope": CAPABILITY_ID},
            {"gate": "fresh-source-generation", "status": "COMPLETED", "input_complete_xodr": False},
            {"gate": "whole-map-validation", "status": "COMPLETED", "tiers": board["rows"][0]["tiers"]},
            {"gate": "production-delivery", "status": "BLOCKED", "reported_decision": delivery["status"]}]


def _write_new(path, data):
    temporary = path.with_name("." + path.name + "." + uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            _reject("artifact-conflict", "不覆盖已有编译证据")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _candidate_id(draft, prefix, candidate_sha):
    return "surface-" + digest({"target": draft["content_hash"], "context": draft["context"],
        "revision": draft["revision"], "epoch": draft["draft_epoch"], "run": prefix,
        "artifact_sha256": candidate_sha, "capability": CAPABILITY_ID})[:32]


def verify_candidate_artifacts(project_directory, candidate):
    """Verify the explicit primary candidate, all run evidence and live inputs.

    This is not permission to accept an arbitrary client candidate. The caller
    must additionally bind this exact result to its owned job and current draft.
    """
    require_json(candidate, "candidate")
    accepted_fields = {"accepted_revision", "accepted_epoch"}
    if (not isinstance(candidate, dict)
            or set(candidate) not in (_CANDIDATE_FIELDS, _CANDIDATE_FIELDS | accepted_fields)):
        _reject("invalid-candidate-fields", "候选必须符合六字段存储合同")
    if accepted_fields <= set(candidate):
        if any(type(candidate[k]) is not int or candidate[k] < 0 for k in accepted_fields):
            _reject("invalid-accepted-metadata", "候选接纳版本元数据必须是非负整数")
    identifier(candidate["candidate_id"], "candidate_id")
    sha256(candidate["target_content_hash"], "target_content_hash")
    validate_context(candidate["context"])
    base = _directory(project_directory)
    artifacts = candidate["artifacts"]
    if not isinstance(artifacts, list) or not artifacts:
        _reject("missing-artifact-manifest", "候选必须列出完整运行及编译证据")
    paths = {}
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
            _reject("invalid-artifact-manifest", "产物清单字段不匹配")
        relative = item["relative_path"]
        path = _relative_file(base, relative)
        sha256(item["sha256"], "artifact.sha256")
        if relative in paths or _file_hash(path) != item["sha256"]:
            _reject("artifact-hash-or-identity-mismatch", "产物重复或实际字节已改变")
        paths[relative] = path
    proofs = [p for p in paths.values() if p.name == "evidence.json"]
    if len(proofs) != 1:
        _reject("compile-evidence-not-unique", "候选需要唯一编译证据")
    proof_path = proofs[0]
    evidence = runner._json(proof_path.read_bytes())
    prefix = proof_path.parent.relative_to(base).as_posix()
    if not re.fullmatch(r"artifacts/[0-9a-f]{32}", prefix):
        _reject("candidate-directory-scope", "编译产物不属于独立运行目录")
    run_directory = proof_path.parent / "run"
    primary = prefix + "/run/candidate.xodr"
    if (evidence.get("schema") != SCHEMA or evidence.get("capability_id") != CAPABILITY_ID
            or evidence.get("candidate_id") != candidate["candidate_id"]
            or evidence.get("primary_artifact") != primary or primary not in paths
            or evidence.get("target_content_hash") != candidate["target_content_hash"]
            or evidence.get("context") != candidate["context"]
            or evidence.get("candidate_sha256") != _file_hash(paths[primary])
            or evidence.get("human_confirmation_verified") is not False
            or evidence.get("fresh_source_generation") is not True
            or evidence.get("input_complete_xodr") is not False
            or evidence.get("source_originals_changed") is not False
            or evidence.get("whole_map_validation") != "COMPLETED"
            or evidence.get("accepted") is not False or evidence.get("delivery") != "BLOCKED"):
        _reject("compile-evidence-mismatch", "编译证据未绑定当前实际候选或错误宣称了确认/交付")
    run_files = _run_files(run_directory)
    expected_paths = {p.relative_to(base).as_posix() for p in run_files} | {prefix + "/evidence.json"}
    if set(paths) != expected_paths:
        _reject("incomplete-run-artifact-manifest", "候选清单没有完整且唯一绑定全部运行产物")
    run_bindings = {p.relative_to(run_directory).as_posix(): _file_hash(p) for p in run_files}
    if evidence.get("run_artifacts_sha256") != run_bindings:
        _reject("run-artifact-manifest-mismatch", "运行证据清单与编译证明不一致")
    snapshot = runner._json((run_directory / "source-snapshot.json").read_bytes())
    registration = register_source(snapshot)
    draft = evidence.get("draft")
    command_ids = _validate_draft(draft, registration)
    if (draft["source_snapshot"]["content_hash"] != snapshot["content_hash"]
            or evidence.get("context") != draft["context"]
            or candidate["candidate_id"] != _candidate_id(draft, prefix, evidence["candidate_sha256"])
            or evidence.get("target_revision") != draft["revision"]
            or evidence.get("target_draft_epoch") != draft["draft_epoch"]
            or evidence.get("target_content_hash") != draft["content_hash"]
            or evidence.get("geometry_command_ids") != command_ids
            or evidence.get("source_ref") != registration.source_ref
            or evidence.get("feature_ids") != list(registration.feature_ids)
            or candidate["affected_ids"] != list(registration.feature_ids)
            or evidence.get("operation_hash") != registration.operation_hash
            or evidence.get("fingerprints") != compiler_fingerprints()):
        _reject("compile-input-binding-mismatch", "编译目标、完整作用域、来源或实现证据不匹配")
    if accepted_fields <= set(candidate):
        if (candidate["accepted_epoch"] != draft["draft_epoch"]
                or candidate["accepted_revision"] <= draft["revision"]):
            _reject("accepted-metadata-target-mismatch", "接纳版本元数据不对应本次编译目标")
    review, assessment, data = _verified_run(run_directory, snapshot)
    if candidate["checks"] != _checks(data):
        _reject("candidate-checks-mismatch", "候选检查摘要不等于实际运行结果")
    if evidence.get("runner_receipt_sha256") != _sha(data["runner-result.json"]):
        _reject("runner-receipt-mismatch", "编译证据未绑定严格运行回执")
    if verify_source_snapshot(snapshot, registration.source_dir, registration.profile_path).get("matches") is not True:
        _reject("source-drift", "核验期间源数据发生变化")
    if evidence["fingerprints"] != compiler_fingerprints():
        _reject("compiler-drift", "核验期间实现或策略发生变化")
    review.read_bound()
    for item in artifacts:
        if _file_hash(_relative_file(base, item["relative_path"])) != item["sha256"]:
            _reject("artifact-drift", "核验期间候选证据发生变化")
    return {"primary_path": str(paths[primary]), "primary_artifact": primary,
            "evidence": evidence, "run_directory": str(run_directory),
            "runner_assessment": assessment, "source_snapshot": snapshot}


def compile_request(payload):
    """Run the existing strict fresh-source runner inside the owned job worker."""
    output = None
    try:
        require_json(payload, "compile_request")
        if (not isinstance(payload, dict) or set(payload) - {"draft", "project_directory", "source_dir", "profile_path"}
                or not {"draft", "project_directory"} <= set(payload)):
            _reject("invalid-compile-request", "编译请求不接受旧候选、旧运行或客户端选定的入口")
        draft = payload["draft"]
        if not isinstance(draft, dict):
            _reject("invalid-draft", "缺少编译草稿")
        registration = register_source(draft.get("source_snapshot"), payload.get("source_dir"), payload.get("profile_path"))
        command_ids = _validate_draft(draft, registration)
        fingerprints = compiler_fingerprints()
        directory = _directory(payload["project_directory"])
        artifacts_root = directory / "artifacts"
        if artifacts_root.is_symlink() or not artifacts_root.resolve().is_relative_to(directory):
            _reject("artifact-path-escape", "产物目录越界或使用链接")
        artifacts_root.mkdir(exist_ok=True)
        output = artifacts_root / uuid4().hex
        output.mkdir(exist_ok=False)
        run_directory = output / "run"  # Must NOT exist: runner proves a fresh execution.
        result = runner.run_experiment(run_directory)
        if (result.get("status") != "EXPERIMENT_COMPLETE" or result.get("experiment_complete") is not True
                or result.get("exit_code") != 0 or result.get("problems") != []):
            _reject("fresh-source-run-rejected", "新生成未形成证据完整的局部合格候选: " + str(result.get("status")))
        receipt = runner._json((run_directory / "runner-result.json").read_bytes())
        if result != receipt:
            _reject("runner-result-mismatch", "返回结果与持久运行回执不一致")
        review, assessment, data = _verified_run(run_directory, draft["source_snapshot"])
        if (fingerprints != compiler_fingerprints()
                or registration.context(draft["source_snapshot"]) != draft["context"]
                or verify_source_snapshot(draft["source_snapshot"], registration.source_dir,
                                          registration.profile_path).get("matches") is not True):
            _reject("inputs-changed-during-compile", "计算期间来源、实现或策略发生变化")
        prefix = output.relative_to(directory).as_posix()
        primary = prefix + "/run/candidate.xodr"
        candidate_sha = _sha(data["candidate.xodr"])
        candidate_id = _candidate_id(draft, prefix, candidate_sha)
        run_files = _run_files(run_directory)
        evidence = {"schema": SCHEMA, "candidate_id": candidate_id, "capability_id": CAPABILITY_ID,
            "primary_artifact": primary, "candidate_sha256": candidate_sha,
            "target_content_hash": draft["content_hash"], "target_revision": draft["revision"],
            "target_draft_epoch": draft["draft_epoch"], "context": draft["context"],
            "source_ref": registration.source_ref, "feature_ids": list(registration.feature_ids),
            "operation_hash": registration.operation_hash, "geometry_command_ids": command_ids,
            "draft": draft, "fingerprints": fingerprints,
            "run_artifacts_sha256": {p.relative_to(run_directory).as_posix(): _file_hash(p) for p in run_files},
            "runner_receipt_sha256": _sha(data["runner-result.json"]),
            "fresh_source_generation": True, "input_complete_xodr": False,
            "source_originals_changed": False, "whole_map_validation": "COMPLETED",
            "human_confirmation_verified": False,
            "transaction_scope": "draft or preview target only; service must separately verify user commit and job ownership",
            "accepted": False, "delivery": "BLOCKED"}
        proof_path = output / "evidence.json"
        _write_new(proof_path, canonical_bytes(evidence))
        artifacts = [{"relative_path": p.relative_to(directory).as_posix(), "sha256": _file_hash(p)}
                     for p in [*run_files, proof_path]]
        candidate = {"candidate_id": candidate_id, "target_content_hash": draft["content_hash"],
            "context": draft["context"], "artifacts": artifacts,
            "affected_ids": list(registration.feature_ids), "checks": _checks(data)}
        verify_candidate_artifacts(directory, candidate)
        return {"status": "COMPILED", "candidate": candidate, "evidence": evidence,
                "delivery": "BLOCKED", "accepted": False}
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        return {"status": "REJECTED", "candidate": None, "accepted": False, "delivery": "BLOCKED",
                "error": {"code": getattr(exc, "code", "surface-compile-unavailable"), "message": str(exc)},
                "retained_run_directory": str(output / "run") if output is not None else None}
