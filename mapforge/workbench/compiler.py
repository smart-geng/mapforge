"""Source-bound draft -> actual local candidate for the one verified capability.

Registration is restricted to WB-04's node16 road12 shared boundary. Controls
are continuous, but every candidate must pass the same analytic invariants and
source/center guards. This adapter never changes the draft, accepts a result,
or grants production release. Store/jobs own acceptance and stale-result rules.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from uuid import uuid4

import numpy
import scipy

from .boundary_probe import (BoundaryEditRejected, PreparedBoundaryEdit,
                             build_source_binding, compile_boundary_edit,
                             prepare_boundary_edit)
from .contracts import canonical_bytes, content_hash, context_for, digest, validate_snapshot
from .sources import verify_source_snapshot

ROOT = Path(__file__).resolve().parents[2]
CAPABILITY_ID = "node16-road12-minus1-minus2-v1"
INTENT_TYPE = "shared_boundary_c2_normal_delta"
JUNCTION_ID = "2023061509381992034"
BASELINE_SHA256 = "712e06cfb32bb8788d34441caae3d851a17cc4cc46dbfa9c30dd1dbaa2c571ae"
SOURCE_MANIFEST_SHA256 = "effb1c6f9226bd977d8792ecb09fa79e3ae9f9dbbec208b2755c0b59e5555cc1"
DATASET_SHA256 = "c7e9b7866dad49d00f46f27a96ed60d852572e031bd102be9ee2d5716820ab71"
PROFILE_SHA256 = "7309c3b4029ab49cb5e5cfb3ab613bbb98484ddfb3e20aaf974c401b1c9469fe"
KNOTS = (76.2690602264151, 84.0, 92.0, 100.0, 110.0)
DEFAULT_BASELINE = ROOT / "out/scoreboard/20261008-safe-mouth-recovery-v3/shp-node16.xodr"
_CODE_FILES = (
    "mapforge/workbench/compiler.py", "mapforge/workbench/boundary_probe.py",
    "mapforge/workbench/sources.py", "mapforge/workbench/contracts.py",
    "mapforge/repair_web/model.py", "mapforge/adapters/shp/profile_source.py",
    "scripts/internal_edge_jets.py", "mapforge/validate/shp_boundary_fidelity.py",
    "mapforge/validate/g11.py", "mapforge/validate/smoothness.py", "uv.lock",
)
_POLICY_FILES = ("profiles/validation/static-acceptance-v1.draft.yaml",
                 "profiles/validation/g8-opendrive-jinfeng-v1.yaml",
                 "profiles/validation/g11-opendrive-v1.draft.yaml")


class CompilerRejected(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _hash_file(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def compiler_fingerprints() -> dict:
    """Content fingerprints include the reused interpreters and locked runtime."""
    code = {name: _hash_file(ROOT / name) for name in _CODE_FILES}
    policies = {name: _hash_file(ROOT / name) for name in _POLICY_FILES}
    runtime = {"python": sys.version.split()[0], "numpy": numpy.__version__, "scipy": scipy.__version__}
    return {"compiler_hash": digest({"files": code, "runtime": runtime}),
            "policy_hash": digest(policies), "code_files_sha256": code,
            "policy_files_sha256": policies, "runtime": runtime}


def _check_snapshot(snapshot):
    validate_snapshot(snapshot)
    if (snapshot.get("snapshot_id") != DATASET_SHA256
            or snapshot.get("junction_id") != JUNCTION_ID
            or snapshot.get("profile", {}).get("sha256") != PROFILE_SHA256):
        raise CompilerRejected("unsupported-source", "该源/路口/Profile 没有已登记的修补能力")
    if digest({"files": snapshot.get("source_files"), "profile_sha256": PROFILE_SHA256}) != DATASET_SHA256:
        raise CompilerRejected("source-manifest-mismatch", "源文件清单与登记的内容身份不一致")
    if snapshot.get("content_hash") != digest({k: v for k, v in snapshot.items()
                                               if k not in {"locator", "content_hash"}}):
        raise CompilerRejected("snapshot-corrupt", "源快照内容校验失败")


def _match_objects(snapshot, binding):
    """Match original layer/row/part AND contents, never a business-ID guess."""
    objects = {o["id"]: o for o in snapshot["objects"]}
    dependencies, affected = set(), set()
    target = None

    def require(row, *, layer=None, index=None, attributes=None, geometry=True):
        row_layer = layer or row["layer"]
        row_index = row["record_index"] if index is None else index
        attrs = row["attributes"] if attributes is None else attributes
        parts = row.get("parts", []) if geometry else []
        ids = []
        for part in list(range(len(parts))) or [None]:
            key = "src:" + digest([snapshot["snapshot_id"], row_layer, row_index, part])
            obj = objects.get(key)
            expected_ref = {"snapshot_id": snapshot["snapshot_id"], "layer": row_layer,
                            "record_index": row_index, "part_index": part}
            expected_points = [] if part is None else parts[part]
            if (obj is None or obj.get("source_ref") != expected_ref
                    or canonical_bytes(obj.get("raw_attributes")) != canonical_bytes(attrs)
                    or canonical_bytes(obj.get("points")) != canonical_bytes(json.loads(json.dumps(expected_points)))):
                raise CompilerRejected("source-object-mismatch", "源对象身份或原始内容不匹配: " + key)
            dependencies.add(key)
            ids.append(key)
        return ids

    for section in binding["source_rows"]:
        for lane_records in section["lane_records"]:
            for row in lane_records:
                ids = require(row)
                if section["editable_section"]:
                    affected.update(ids)
        for rels in section["relations"]:
            for relation in rels:
                require({}, layer=relation["relation_layer"], index=relation["relation_index"],
                        attributes=relation["attributes"], geometry=False)
                for row in relation["records"]:
                    ids = require(row)
                    if section["editable_section"] and row["boundary_id"] == section["shared_boundary_id"]:
                        affected.update(ids)
                        target = target or ids[0]
    if not target:
        raise CompilerRejected("no-source-boundary", "没有可编辑的源共享边界对象")
    return target, tuple(sorted(dependencies)), tuple(sorted(affected))


@dataclass(frozen=True)
class RegisteredBoundaryCapability:
    prepared: PreparedBoundaryEdit
    snapshot_hash: str
    source_ref: str
    feature_ids: tuple[str, ...]
    affected_ids: tuple[str, ...]
    baseline_path: str
    source_dir: str
    profile_path: str
    coordinate_evidence_json: bytes

    def context(self, snapshot):
        _check_snapshot(snapshot)
        if digest(snapshot) != self.snapshot_hash:
            raise CompilerRejected("snapshot-changed", "能力属于另一份源快照")
        fingerprints = compiler_fingerprints()
        return context_for(snapshot, fingerprints["compiler_hash"], fingerprints["policy_hash"])

    def capability(self) -> dict:
        from scripts.internal_edge_jets import states
        from mapforge.repair_web.model import parse
        road = parse(self.prepared.data).find("road[@id='12']")
        anchor = 92.0
        edge = states(road, -1, anchor, False)[1]
        heading = float(road.find("planView/geometry").get("hdg"))
        return {**self.prepared.capability(), "capability_id": CAPABILITY_ID,
                "source_ref": self.source_ref, "feature_ids": list(self.feature_ids),
                "affected_ids": list(self.affected_ids), "intent_type": INTENT_TYPE,
                "scope": {"capability_id": CAPABILITY_ID, "feature_ids": list(self.feature_ids)},
                "control": {"parameter": "normal_delta_m", "semantics": "absolute-from-registered-baseline",
                            "min": -0.1, "max": 0.1, "continuous": True, "anchor_s_m": anchor,
                            "anchor_xy_m": list(edge[:2]),
                            "normal_xy": [-math.sin(heading), math.cos(heading)]},
                "coordinate_space": "existing-xodr-local-metres",
                "coordinate_evidence": json.loads(self.coordinate_evidence_json),
                "supported_scope": "one registered object/range; not all normal roads",
                "acceptance": "each target must pass local and same-source residual guards",
                "formal_export_available": False}

    def store_spec(self) -> dict:
        """Minimal capability contract for server-side draft registration."""
        return {"capability_id": CAPABILITY_ID, "type": INTENT_TYPE,
                "source_ref": self.source_ref, "feature_ids": list(self.feature_ids),
                "baseline_sha256": BASELINE_SHA256,
                "normal_delta_m": {"min": -0.1, "max": 0.1}}


def register_verified_baseline(snapshot: dict, *, baseline_path=None,
                               source_dir=None, profile_path=None) -> RegisteredBoundaryCapability:
    """Server registration. Client paths, IDs or a claimed capability are not authority."""
    _check_snapshot(snapshot)
    baseline = Path(baseline_path or DEFAULT_BASELINE).resolve()
    source_dir = Path(source_dir or snapshot["locator"]["source_dir"]).resolve()
    profile_path = Path(profile_path or snapshot["locator"]["profile_path"]).resolve()
    try:
        data = baseline.read_bytes()
        if _sha(data) != BASELINE_SHA256:
            raise CompilerRejected("baseline-mismatch", "基线内容不属于已验证能力")
        manifest_bytes = baseline.with_suffix(".source-lanes.json").read_bytes()
        if _sha(manifest_bytes) != SOURCE_MANIFEST_SHA256:
            raise CompilerRejected("baseline-source-manifest-mismatch", "基线来源报告内容不匹配")
        check = verify_source_snapshot(snapshot, source_dir, profile_path)
        if not check["matches"]:
            raise CompilerRejected("source-drift", "原件/Profile 已改变或缺失，不能继续编译")
        binding = build_source_binding(data, source_dir, profile_path, "12", -1, list(KNOTS),
                                       include_neighbor_context=False)
        prepared = prepare_boundary_edit(data, binding, "12", -1, list(KNOTS))
    except CompilerRejected:
        raise
    except (OSError, ValueError, KeyError) as exc:
        raise CompilerRejected("registration-unavailable", str(exc)) from exc
    target, features, affected = _match_objects(snapshot, binding)
    source_manifest = json.loads(manifest_bytes)
    comparison_crs = source_manifest.get("comparison_crs", {})
    if (comparison_crs.get("units") != "m" or not comparison_crs.get("proj_string")
            or comparison_crs.get("integrity") != "internally-consistent"):
        raise CompilerRejected("local-coordinate-evidence-missing", "缺少现有 XODR 来源报告的米制比较依据")
    coordinate_evidence = {"baseline_sha256": BASELINE_SHA256,
                           "source_manifest_sha256": SOURCE_MANIFEST_SHA256,
                           "source_comparison_crs": comparison_crs,
                           "local_projection": binding["projection"],
                           "source_frame_modified": False, "absolute_crs_verified": False,
                           "production_authority": False}
    return RegisteredBoundaryCapability(prepared, digest(snapshot), target, features, affected,
                                         str(baseline), str(source_dir), str(profile_path),
                                         canonical_bytes(coordinate_evidence))


def _target(draft, registration):
    value = 0.0
    command_ids = []
    for intent in draft["intents"]:
        if not isinstance(intent, dict):
            raise CompilerRejected("invalid-intent", "草稿意图必须是对象")
        if intent.get("type") == "annotation":
            # Annotation validation belongs to the draft contract; it never
            # writes geometry. Its exact content still participates in hash.
            continue
        if intent.get("type") != INTENT_TYPE:
            raise CompilerRejected("unsupported-intent", "草稿含尚未支持的几何意图")
        if (set(intent) - {"command_id", "base_revision", "type", "source_ref", "scope", "parameters"}
                or intent.get("source_ref") != registration.source_ref
                or not isinstance(intent.get("scope"), dict)
                or set(intent["scope"]) != {"feature_ids", "capability_id"}
                or intent["scope"]["capability_id"] != CAPABILITY_ID
                or sorted(intent["scope"]["feature_ids"]) != list(registration.feature_ids)
                or len(intent["scope"]["feature_ids"]) != len(registration.feature_ids)):
            raise CompilerRejected("scope-mismatch", "编辑意图与服务器登记的完整身份/作用域不一致")
        params = intent.get("parameters")
        if (not isinstance(params, dict) or set(params) != {"normal_delta_m", "baseline_sha256"}
                or params["baseline_sha256"] != BASELINE_SHA256):
            raise CompilerRejected("target-baseline-mismatch", "控制参数或基线指纹不匹配")
        value = params["normal_delta_m"]
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value) or abs(value) > 0.1):
            raise CompilerRejected("target-out-of-range", "法向目标需为 ±0.1m 内有限连续数值")
        if not isinstance(intent.get("command_id"), str) or not intent["command_id"]:
            raise CompilerRejected("invalid-command-id", "几何意图缺少命令身份")
        command_ids.append(intent["command_id"])
    return float(value), command_ids


def _quality_guard(evidence, delta):
    if delta == 0:
        return
    for key, before in evidence["source_before"]["items"].items():
        after = evidence["source_after"]["items"][key]
        if any(after[metric] > before[metric] + 1e-9 for metric in ("max_m", "p95_m", "mean_m")):
            raise CompilerRejected("source-residual-regression", "目标使真实共享边界或相邻中心残差退步: " + key)
    if not all(evidence["source_after"]["items"]["shared"][metric]
               < evidence["source_before"]["items"]["shared"][metric] - 1e-6
               for metric in ("max_m", "p95_m", "mean_m")):
        raise CompilerRejected("no-verified-improvement", "非零目标未达到预登记的真实残差改善条件")


def _atomic_artifact(project_directory: Path, relative: str, data: bytes):
    base = project_directory.resolve()
    if not base.is_dir() or project_directory.is_symlink():
        raise CompilerRejected("invalid-project-directory", "工程目录不存在或为符号链接")
    target = base.joinpath(*relative.split("/"))
    if not target.resolve().is_relative_to(base):
        raise CompilerRejected("artifact-path-escape", "产物路径越出工程")
    parents = [target, target.parent, target.parent.parent]
    if any(path.is_symlink() for path in parents):
        raise CompilerRejected("artifact-symlink", "产物路径不得使用符号链接")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if target.read_bytes() != data:
            raise CompilerRejected("artifact-conflict", "同身份产物已经存在且内容不同")
    else:
        temporary = target.with_name("." + target.name + "." + uuid4().hex + ".tmp")
        try:
            with temporary.open("xb") as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        finally:
            if temporary.exists():
                temporary.unlink()
    if target.read_bytes() != data:
        raise CompilerRejected("artifact-readback-failed", "实际产物读回字节不一致")
    return {"relative_path": relative, "sha256": _sha(data)}


@dataclass(frozen=True)
class CompiledDraft:
    artifact_bytes: bytes
    evidence: dict
    candidate: dict

    def as_job_result(self):
        return {"status": "COMPILED", "candidate": self.candidate, "evidence": self.evidence,
                "delivery": "BLOCKED", "accepted": False}


def compile_draft(draft: dict, registration: RegisteredBoundaryCapability,
                  project_directory: str | Path) -> CompiledDraft:
    """Compile and persist immutable artifacts. Never alter a project snapshot."""
    snapshot = draft.get("source_snapshot")
    _check_snapshot(snapshot)
    if not isinstance(draft.get("intents"), list):
        raise CompilerRejected("invalid-draft", "草稿缺少意图列表")
    try:
        actual_content = content_hash(snapshot, draft["intents"])
    except ValueError as exc:
        raise CompilerRejected("invalid-draft", str(exc)) from exc
    if draft.get("content_hash") != actual_content:
        raise CompilerRejected("draft-hash-mismatch", "草稿内容已变化或校验失败")
    context = registration.context(snapshot)
    if draft.get("context") != context:
        raise CompilerRejected("stale-context", "源/Profile/编译器/策略指纹已过期")
    if _hash_file(registration.baseline_path) != BASELINE_SHA256:
        raise CompilerRejected("baseline-drift", "登记的基线文件已改变")
    if _hash_file(Path(registration.baseline_path).with_suffix(".source-lanes.json")) != SOURCE_MANIFEST_SHA256:
        raise CompilerRejected("baseline-source-drift", "登记的来源报告已改变")
    source_check = verify_source_snapshot(snapshot, registration.source_dir, registration.profile_path)
    if not source_check["matches"]:
        raise CompilerRejected("source-drift", "原件/Profile 已改变；保留草稿与最后有效候选")
    delta, command_ids = _target(draft, registration)
    try:
        data, local = compile_boundary_edit(registration.prepared, delta)
    except BoundaryEditRejected as exc:
        raise CompilerRejected("local-constraint-rejected", str(exc)) from exc
    _quality_guard(local, delta)
    candidate_id = "boundary-" + digest({"target": actual_content, "context": context,
                                        "revision": draft.get("revision"), "draft_epoch": draft.get("draft_epoch"),
                                        "artifact_sha256": _sha(data), "capability": CAPABILITY_ID})[:32]
    evidence = {"schema": "mapforge/workbench-compiled-draft/v1", "candidate_id": candidate_id,
                "target_content_hash": actual_content, "target_revision": draft.get("revision"),
                "target_draft_epoch": draft.get("draft_epoch"), "context": context,
                "capability_id": CAPABILITY_ID, "baseline_sha256": BASELINE_SHA256,
                "candidate_sha256": _sha(data), "source_ref": registration.source_ref,
                "feature_ids": list(registration.feature_ids), "affected_ids": list(registration.affected_ids),
                "geometry_command_ids": command_ids, "normal_delta_m": delta,
                "coordinate_evidence": json.loads(registration.coordinate_evidence_json),
                "fingerprints": compiler_fingerprints(), "local_checks": local,
                "delivery": "BLOCKED", "whole_map_validation": "NOT_RUN"}
    # Recheck immutable inputs after computation, before publishing artifacts.
    if (context != registration.context(snapshot)
            or not verify_source_snapshot(snapshot, registration.source_dir, registration.profile_path)["matches"]):
        raise CompilerRejected("inputs-changed-during-compile", "计算期间输入或实现指纹发生变化")
    prefix = f"artifacts/{candidate_id}"
    artifacts = [_atomic_artifact(Path(project_directory), prefix + "/candidate.xodr", data),
                 _atomic_artifact(Path(project_directory), prefix + "/evidence.json", canonical_bytes(evidence))]
    candidate = {"candidate_id": candidate_id, "target_content_hash": actual_content, "context": context,
                 "artifacts": artifacts, "affected_ids": list(registration.affected_ids) if delta else [],
                 "checks": [{"gate": "local-shared-boundary", "status": "PASS", "scope": CAPABILITY_ID},
                            {"gate": "same-source-residual", "status": "PASS" if delta else "UNCHANGED"},
                            {"gate": "whole-map-validation", "status": "NOT_RUN"},
                            {"gate": "production-delivery", "status": "BLOCKED"}]}
    return CompiledDraft(data, evidence, candidate)


def compile_request(payload: dict) -> dict:
    """JSON worker adapter. A rejection contains no candidate to accidentally accept."""
    try:
        draft = payload["draft"]
        registration = register_verified_baseline(
            draft["source_snapshot"], baseline_path=payload.get("baseline_path"),
            source_dir=payload.get("source_dir"), profile_path=payload.get("profile_path"))
        return compile_draft(draft, registration, payload["project_directory"]).as_job_result()
    except CompilerRejected as exc:
        return {"status": "REJECTED", "error": {"code": exc.code, "message": str(exc)},
                "candidate": None, "accepted": False, "delivery": "BLOCKED"}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {"status": "REJECTED", "error": {"code": "compile-unavailable", "message": str(exc)},
                "candidate": None, "accepted": False, "delivery": "BLOCKED"}
