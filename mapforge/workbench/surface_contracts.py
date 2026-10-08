"""Structural contracts for the fixed, source-bound auxiliary-surface operation.

No compiler, source reader or geometry code is imported. Server registration
proves that an operation is available; these checks prevent a stored draft from
changing that registration's source, operation, context or complete scope.
"""
from __future__ import annotations

import copy

from .contracts import (StoreConflict, StoreValidation, context_for, digest,
                        identifier, require_json, sha256, validate_context,
                        validate_snapshot)

CAPABILITY_ID = "0621-source-supported-surface-v1"
INTENT_TYPE = "source_supported_surface_tracks_v2"
JUNCTION_ID = "2023062110304177600"
_SPEC_FIELDS = {"capability_id", "type", "source_ref", "feature_ids",
                "source_content_hash", "operation_hash"}


def _snapshot_binding(snapshot, context):
    validate_snapshot(snapshot)
    if snapshot.get("junction_id") != JUNCTION_ID:
        raise StoreValidation("此辅助铺面能力仅属于已登记的 0621 源路口")
    source_content = sha256(snapshot.get("content_hash"), "source_content_hash")
    if source_content != digest({key: value for key, value in snapshot.items()
                                 if key not in {"locator", "content_hash"}}):
        raise StoreConflict("源快照实际内容与 source_content_hash 不一致")
    current = validate_context(context)
    if current != context_for(snapshot, current["compiler_hash"], current["policy_hash"]):
        raise StoreConflict("辅助铺面上下文不属于当前源快照或 Profile")
    return source_content, current


def validate_capability(spec, snapshot, context):
    """Normalize server-owned metadata, never register client geometry claims."""
    require_json(spec, "capability")
    if not isinstance(spec, dict) or set(spec) != _SPEC_FIELDS:
        raise StoreValidation("辅助铺面能力登记字段不完整或包含未知字段")
    identifier(spec["capability_id"], "capability_id")
    if spec["capability_id"] != CAPABILITY_ID or spec["type"] != INTENT_TYPE:
        raise StoreValidation("尚未支持该辅助铺面能力身份或类型")
    source_content, current = _snapshot_binding(snapshot, context)
    features = spec["feature_ids"]
    ids = {item["id"] for item in snapshot["objects"]}
    if (not isinstance(features, list) or not 1 <= len(features) <= 256
            or any(not isinstance(item, str) or item not in ids for item in features)
            or len(set(features)) != len(features) or set(features) != ids
            or not isinstance(spec["source_ref"], str) or spec["source_ref"] not in ids):
        raise StoreValidation("辅助铺面作用域必须绑定当前路口的全部唯一源对象及目标")
    if sha256(spec["source_content_hash"], "source_content_hash") != source_content:
        raise StoreConflict("辅助铺面能力的源内容哈希已过期")
    sha256(spec["operation_hash"], "operation_hash")
    normalized = copy.deepcopy(spec)
    normalized["feature_ids"] = sorted(features)
    normalized["spec_hash"] = digest(normalized)
    normalized["context"] = current
    return normalized


def validate_command(command, snapshot, *, capabilities, context):
    """Validate the surface branch after contracts' common command checks.

    The shared dispatcher already checks command identity/base revision, JSON,
    source membership and the exact scope envelope for every geometry command.
    This branch requires the entire registered source scope and fixed hashes.
    """
    scope, params = command["scope"], command.get("parameters")
    capability_id = identifier(scope["capability_id"], "capability_id")
    capability = (capabilities or {}).get(capability_id)
    if capability is None:
        raise StoreValidation("此工程没有服务器登记的辅助铺面能力")
    if not isinstance(capability, dict):
        raise StoreValidation("辅助铺面能力登记格式无效")
    if capability.get("context") != context:
        raise StoreConflict("辅助铺面能力指纹已过期；需要服务器重新核验登记")
    if set(capability) != _SPEC_FIELDS | {"spec_hash", "context"}:
        raise StoreValidation("辅助铺面能力登记字段不完整或包含未知字段")
    normalized = validate_capability({key: capability[key] for key in _SPEC_FIELDS}, snapshot, context)
    if normalized != capability:
        raise StoreConflict("辅助铺面能力规格或登记哈希已变化")
    if (capability_id != CAPABILITY_ID or command["type"] != INTENT_TYPE
            or command["source_ref"] != capability["source_ref"]
            or sorted(scope["feature_ids"]) != capability["feature_ids"]):
        raise StoreValidation("辅助铺面意图目标或完整作用域与已登记能力不一致")
    if not isinstance(params, dict) or set(params) != {"source_content_hash", "operation_hash"}:
        raise StoreValidation("辅助铺面参数仅支持 source_content_hash 与 operation_hash")
    for name in ("source_content_hash", "operation_hash"):
        if sha256(params[name], name) != capability[name]:
            raise StoreConflict("辅助铺面意图的来源或操作哈希已过期")
    return {"command_id": command["command_id"], "type": INTENT_TYPE,
            "source_ref": command["source_ref"],
            "scope": {"capability_id": capability_id, "feature_ids": sorted(scope["feature_ids"])},
            "parameters": {key: params[key] for key in ("source_content_hash", "operation_hash")}}
