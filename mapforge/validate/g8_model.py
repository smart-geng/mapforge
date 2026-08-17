# -*- coding: utf-8 -*-
"""G8 来源车道 manifest 与版本化 policy 的轻量契约。"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

MANIFEST_SCHEMA = "mapforge/source-lane-geometry-manifest/v1"
GATE_SCHEMA = "mapforge/gate-result/v1"
POLICY_SCHEMA = "mapforge/g8-policy/v1"

CONVERSION_STATES = {
    "EXACT", "TRANSFORMED", "APPROXIMATED", "INFERRED",
    "EXTENSION", "PASSTHROUGH", "DROPPED", "FAILED",
}
GATE_STATES = {"PASS", "FAIL", "NOT_RUN", "UNAVAILABLE"}


def json_safe(value: Any):
    """递归转换为严格 JSON 值；NaN/Inf 变为 null。"""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def canonical_json(value: Any) -> str:
    return json.dumps(json_safe(value), ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)


def object_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def geometry_sha256(coordinates) -> str:
    pts = np.asarray(coordinates, float)
    return object_sha256(np.round(pts, 9).tolist())


def source_lane(
    source_lane_id: str,
    coordinates,
    *,
    owner: dict,
    role: str,
    status: str,
    support_kind: str,
    policy_class: str,
    travel_direction: str,
    eligible: bool = True,
    stop_line: dict | None = None,
    legacy_source_key: str | None = None,
) -> dict:
    """创建一条按行车方向排列的来源车道记录。"""
    if status not in CONVERSION_STATES:
        raise ValueError(f"未知转换状态: {status}")
    if travel_direction not in ("with_s", "against_s"):
        raise ValueError(f"未知行车方向: {travel_direction}")
    pts = np.asarray(coordinates, float)
    if pts.ndim != 2 or (pts.shape[1] if pts.ndim == 2 else 0) != 2:
        pts = np.zeros((0, 2), float)
    rec = {
        "source_lane_id": str(source_lane_id),
        "owner": json_safe(owner),
        "role": role,
        "status": status,
        "support_kind": support_kind,
        "policy_class": policy_class,
        "comparison": {"eligible": bool(eligible)},
        "geometry": {
            "type": "LineString",
            "coordinates": pts.tolist(),
            "geometry_sha256": geometry_sha256(pts),
        },
        "travel": {
            "coordinate_order": "with-travel",
            "target_direction": travel_direction,
            "start": pts[0].tolist() if len(pts) else None,
            "end": pts[-1].tolist() if len(pts) else None,
        },
        "stop_line": json_safe(stop_line) if stop_line else {"availability": "not-applicable"},
    }
    if legacy_source_key is not None:
        rec["legacy_source_key"] = legacy_source_key
    return rec


def make_manifest(
    *,
    source_format: str,
    source_profile: str,
    comparison_crs: dict,
    lanes: list[dict],
    source_contexts: list[dict] | None = None,
) -> dict:
    """创建 manifest，并在写入前拒绝重复来源键。"""
    ids = [str(x.get("source_lane_id", "")) for x in lanes]
    dup = sorted({x for x in ids if ids.count(x) > 1})
    if dup:
        raise ValueError(f"重复 source_lane_id: {dup}")
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "source_format": source_format,
        "source_profile": source_profile,
        "comparison_crs": json_safe(comparison_crs),
        "source_contexts": json_safe(source_contexts or []),
        "lanes": json_safe(lanes),
    }
    manifest["manifest_sha256"] = object_sha256(manifest)
    return manifest


def validate_manifest(manifest: dict | None) -> list[str]:
    errors = []
    if not isinstance(manifest, dict):
        return ["manifest_missing"]
    if manifest.get("schema") != MANIFEST_SCHEMA:
        errors.append("manifest_schema_unsupported")
    crs = manifest.get("comparison_crs") or {}
    if crs.get("units") != "m":
        errors.append("comparison_crs_not_metric")
    lanes = manifest.get("lanes")
    if not isinstance(lanes, list):
        errors.append("manifest_lanes_missing")
        return errors
    ids = [str(x.get("source_lane_id", "")) for x in lanes]
    if any(not x for x in ids):
        errors.append("source_lane_id_empty")
    if len(ids) != len(set(ids)):
        errors.append("duplicate_source_ids")
    for lane in lanes:
        if lane.get("status") not in CONVERSION_STATES:
            errors.append(f"invalid_conversion_state:{lane.get('source_lane_id')}")
        g = ((lane.get("geometry") or {}).get("coordinates") or [])
        if lane.get("comparison", {}).get("eligible") and len(g) < 2:
            errors.append(f"unmeasurable:{lane.get('source_lane_id')}")
    return sorted(set(errors))


def load_policy(ref: str | Path | dict | None) -> dict | None:
    if ref is None:
        return None
    if isinstance(ref, dict):
        policy = json_safe(ref)
    else:
        path = Path(ref)
        policy = yaml.safe_load(path.read_text(encoding="utf-8"))
        policy["_path"] = str(path)
    data = {k: v for k, v in policy.items() if k not in ("_path", "policy_sha256")}
    policy["policy_sha256"] = object_sha256(data)
    return policy


def validate_policy(policy: dict | None) -> list[str]:
    if not isinstance(policy, dict):
        return ["policy_missing"]
    errors = []
    if policy.get("schema") != POLICY_SCHEMA:
        errors.append("policy_schema_unsupported")
    if policy.get("lifecycle") != "active":
        errors.append("policy_not_active")
    if not policy.get("classes"):
        errors.append("policy_classes_missing")
    app = policy.get("applicability") or {}
    if app.get("target_format") != "opendrive":
        errors.append("policy_not_applicable")
    return errors


def write_json(path: str | Path, value: Any) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(json_safe(value), ensure_ascii=False, indent=2,
                              allow_nan=False) + "\n", encoding="utf-8")
    return out
