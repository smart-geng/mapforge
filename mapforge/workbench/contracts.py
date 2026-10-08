"""JSON contracts for the source workspace; no geometry compiler is invoked here."""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re
from typing import Any


class StoreError(ValueError):
    """An explainable project operation failure."""


class StoreValidation(StoreError):
    pass


class StoreConflict(StoreError):
    pass


class StoreNotFound(StoreError):
    pass


class StoreReadOnly(StoreError):
    pass


def require_json(value: Any, path: str = "value") -> None:
    """Reject non-JSON values, including NaN/Infinity and non-string keys."""
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise StoreValidation(f"{path}: 数值必须有限")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            require_json(item, f"{path}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise StoreValidation(f"{path}: JSON 对象键必须是字符串")
            require_json(item, f"{path}.{key}")
        return
    raise StoreValidation(f"{path}: 不支持的 JSON 类型 {type(value).__name__}")


def canonical_bytes(value: Any) -> bytes:
    require_json(value)
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def digest(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def identifier(value: Any, label: str = "ID") -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value):
        raise StoreValidation(f"{label}: 需要 1～128 位字母、数字或 _ . : -")
    return value


def sha256(value: Any, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise StoreValidation(f"{label}: 需要小写 SHA-256")
    return value


def validate_snapshot(snapshot: Any) -> dict:
    require_json(snapshot, "source_snapshot")
    if not isinstance(snapshot, dict) or not isinstance(snapshot.get("snapshot_id"), str):
        raise StoreValidation("源快照缺少 snapshot_id")
    if not snapshot["snapshot_id"] or not isinstance(snapshot.get("objects"), list):
        raise StoreValidation("源快照必须包含身份及 objects")
    seen = set()
    for obj in snapshot["objects"]:
        if not isinstance(obj, dict) or not isinstance(obj.get("id"), str) or not obj["id"]:
            raise StoreValidation("源对象缺少稳定 id")
        if obj["id"] in seen:
            raise StoreValidation("源对象 id 重复；业务 ID 重复应使用不同记录身份保留")
        seen.add(obj["id"])
        ref = obj.get("source_ref")
        if not isinstance(ref, dict) or ref.get("snapshot_id") != snapshot["snapshot_id"]:
            raise StoreValidation("源对象引用不属于此快照")
    return copy.deepcopy(snapshot)


def source_hash(snapshot: dict) -> str:
    # Bind the whole selected snapshot, not just its dataset snapshot_id: two
    # junctions may share dataset/profile hashes but have different objects.
    # A locator identifies the current file location, not the immutable source
    # content. Only this top-level transport field is excluded; business fields
    # called "path" or nested locators remain part of the source identity.
    return digest({key: value for key, value in snapshot.items() if key != "locator"})


def content_hash(snapshot: dict, intents: list[dict]) -> str:
    return digest({"source_hash": source_hash(snapshot), "intents": intents})


def validate_command(command: Any, snapshot: dict, base_revision: int, *,
                     capabilities: dict | None = None, context: dict | None = None) -> dict:
    require_json(command, "command")
    if not isinstance(command, dict):
        raise StoreValidation("命令必须为 JSON 对象")
    extra = set(command) - {"command_id", "base_revision", "type", "source_ref", "scope", "parameters"}
    if extra:
        raise StoreValidation(f"未支持的命令字段: {', '.join(sorted(extra))}")
    identifier(command.get("command_id"), "command_id")
    if "base_revision" in command:
        version = command["base_revision"]
        if isinstance(version, bool) or not isinstance(version, int) or version < 0:
            raise StoreValidation("命令 base_revision 必须为非负整数")
        if version != base_revision:
            raise StoreConflict("命令版本与请求版本不一致")
    kind = command.get("type")
    if kind not in ("annotation", "shared_boundary_c2_normal_delta"):
        raise StoreValidation("尚未开放该编辑能力")
    ids = {obj["id"] for obj in snapshot["objects"]}
    ref = command.get("source_ref")
    if not isinstance(ref, str) or ref not in ids:
        raise StoreValidation("源对象引用不属于当前工程")
    scope = command.get("scope")
    scope_keys = {"feature_ids"} if kind == "annotation" else {"feature_ids", "capability_id"}
    if not isinstance(scope, dict) or set(scope) != scope_keys:
        raise StoreValidation("scope 字段不符合已注册操作的作用域合同")
    features = scope["feature_ids"]
    if (not isinstance(features, list) or not 1 <= len(features) <= 256
            or any(not isinstance(x, str) or x not in ids for x in features)
            or len(features) != len(set(features)) or ref not in features):
        raise StoreValidation("作用域必须包含目标，且全部引用当前工程中的唯一源对象")
    params = command.get("parameters")
    if kind == "shared_boundary_c2_normal_delta":
        capability_id = identifier(scope.get("capability_id"), "capability_id")
        capability = (capabilities or {}).get(capability_id)
        if capability is None:
            raise StoreValidation("此工程没有服务器登记的修形能力")
        if capability.get("context") != context:
            raise StoreConflict("修形能力指纹已过期；需要服务器重新核验登记")
        if (capability.get("type") != kind or capability.get("source_ref") != ref
                or sorted(features) != capability.get("feature_ids")):
            raise StoreValidation("意图目标或作用域与已登记能力不一致")
        if not isinstance(params, dict) or set(params) != {"normal_delta_m", "baseline_sha256"}:
            raise StoreValidation("修形参数只支持 normal_delta_m 与 baseline_sha256")
        baseline = sha256(params["baseline_sha256"], "baseline_sha256")
        if baseline != capability["baseline_sha256"]:
            raise StoreConflict("修形意图的基线哈希已过期")
        value = params["normal_delta_m"]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise StoreValidation("normal_delta_m 必须为有限数值，单位为米")
        limits = capability["normal_delta_m"]
        if not limits["min"] <= value <= limits["max"]:
            raise StoreValidation("normal_delta_m 超出服务器登记的允许范围")
        return {"command_id": command["command_id"], "type": kind, "source_ref": ref,
                "scope": {"feature_ids": sorted(features), "capability_id": capability_id},
                "parameters": {"normal_delta_m": float(value), "baseline_sha256": baseline}}
    if not isinstance(params, dict) or set(params) - {"text", "status"}:
        raise StoreValidation("待办参数仅支持 text 和 status")
    text = params.get("text")
    if not isinstance(text, str) or not text.strip() or len(text) > 10000:
        raise StoreValidation("待办文字需为 1～10000 字符")
    status = params.get("status", "unresolved")
    if status not in ("unresolved", "resolved"):
        raise StoreValidation("待办状态必须为 unresolved 或 resolved")
    return {"command_id": command["command_id"], "type": "annotation", "source_ref": ref,
            "scope": {"feature_ids": sorted(features)},
            "parameters": {"text": text, "status": status}}


def validate_capability(spec: Any, snapshot: dict, context: dict) -> dict:
    """Validate server-supplied capability metadata; registration is not an HTTP edit."""
    require_json(spec, "capability")
    required = {"capability_id", "type", "source_ref", "feature_ids", "baseline_sha256", "normal_delta_m"}
    if not isinstance(spec, dict) or set(spec) != required:
        raise StoreValidation("服务器能力登记字段不完整或包含未知字段")
    identifier(spec["capability_id"], "capability_id")
    if spec["type"] != "shared_boundary_c2_normal_delta":
        raise StoreValidation("尚未支持该服务器能力类型")
    ids = {obj["id"] for obj in snapshot["objects"]}
    features = spec["feature_ids"]
    if (not isinstance(spec["source_ref"], str) or spec["source_ref"] not in ids
            or not isinstance(features, list) or not 1 <= len(features) <= 256
            or any(not isinstance(x, str) or x not in ids for x in features)
            or len(set(features)) != len(features) or spec["source_ref"] not in features):
        raise StoreValidation("能力作用域必须绑定当前工程中的唯一源对象")
    sha256(spec["baseline_sha256"], "baseline_sha256")
    limits = spec["normal_delta_m"]
    if not isinstance(limits, dict) or set(limits) != {"min", "max"}:
        raise StoreValidation("能力必须登记 normal_delta_m 的 min/max")
    if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in limits.values()) or not -0.1 <= limits["min"] < limits["max"] <= 0.1
            or not limits["min"] <= 0 <= limits["max"]):
        raise StoreValidation("当前修形能力允许范围须包含零且位于 ±0.1 米内")
    normalized = copy.deepcopy(spec)
    normalized["feature_ids"] = sorted(features)
    normalized["normal_delta_m"] = {key: float(value) for key, value in limits.items()}
    normalized["spec_hash"] = digest(normalized)
    normalized["context"] = validate_context(context)
    return normalized


def context_for(snapshot: dict, compiler_hash: str = "", policy_hash: str = "") -> dict:
    return {"source_hash": source_hash(snapshot), "profile_hash": digest(snapshot.get("profile")),
            "compiler_hash": compiler_hash, "policy_hash": policy_hash}


def validate_context(value: Any) -> dict:
    if not isinstance(value, dict) or set(value) != {
            "source_hash", "profile_hash", "compiler_hash", "policy_hash"}:
        raise StoreValidation("候选必须绑定源、Profile、编译器和策略四项指纹")
    for key, item in value.items():
        sha256(item, key)
    return copy.deepcopy(value)
