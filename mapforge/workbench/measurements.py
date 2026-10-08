"""Read-only source measurements with an explicit, evidence-bound local frame.

The registered CRS audit only licenses numerical inspection on the source's
declared ellipsoid. It cannot verify a physical datum, approve an edit, or grant
production authority. Missing audit evidence leaves raw coordinate measurement
available, with its original unit and the reason metric inspection is unavailable.
"""
from __future__ import annotations

import hashlib
import copy
import json
import math
from pathlib import Path
from typing import Any

from pyproj import Transformer

SCHEMA = "mapforge/workbench-measurement/v1"
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_AUDIT_PATH = ROOT / "out/workbench/20261008-crs-audit/report.json"
AUDIT_SHA256 = "fb7bc19596648a83e78f7800e4e890f1fea1e3745dbe8b84598840735a3f3103"
_POLYLINE_TYPES = {3, 5, 13, 15, 23, 25}


class MeasurementRejected(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _snapshot(snapshot: dict) -> None:
    if not isinstance(snapshot, dict) or snapshot.get("schema") != "mapforge/workbench-source/v1":
        raise MeasurementRejected("invalid-source-snapshot", "测量要求已登记的源工程快照")
    try:
        expected = _digest({k: v for k, v in snapshot.items() if k not in {"content_hash", "locator"}})
    except (TypeError, ValueError, OverflowError) as exc:
        raise MeasurementRejected("invalid-source-snapshot", "源工程快照不能规范序列化") from exc
    if expected != snapshot.get("content_hash"):
        raise MeasurementRejected("source-snapshot-changed", "源工程快照内容与登记哈希不符，请重新打开工程")
    if not isinstance(snapshot.get("frame"), dict) or snapshot["frame"].get("kind") != "raw":
        raise MeasurementRejected("unsupported-source-frame", "测量当前只支持原始坐标框架")


def _point(point: Any) -> list[float]:
    if (not isinstance(point, (list, tuple)) or len(point) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point)):
        raise MeasurementRejected("invalid-point", "测量点必须为两个有限数值，顺序与源坐标一致")
    try:
        result = [float(v) for v in point]
    except OverflowError as exc:
        raise MeasurementRejected("invalid-point", "测量点超出数值范围") from exc
    if not all(math.isfinite(v) for v in result):
        raise MeasurementRejected("invalid-point", "测量点必须为有限数值")
    return result


def _raw_frame(snapshot: dict, code: str, reason: str) -> dict:
    unit = snapshot["frame"].get("coordinate_unit", "unknown")
    known_unit = unit if isinstance(unit, str) and unit else "unknown"
    return {
        "mode": "raw-coordinate", "unit": known_unit,
        "display_label": f"原始坐标长度（{known_unit}）",
        "metric_available": False, "reason_code": code, "reason": reason,
        "pipeline": None, "audit_sha256": None,
        "assumptions": ["仅计算原始二维坐标中的线段长度；不等于经过测绘核验的地面距离。",
                        "宽度字段单位不用于缩放几何点列；不使用未登记的 EPSG 或默认椭球。"],
    }


def _frame(snapshot: dict, audit_path: str | Path | None) -> dict:
    if audit_path is None:
        return _raw_frame(snapshot, "audit-not-selected", "未选择登记的局部坐标审核依据")
    try:
        data = Path(audit_path).read_bytes()
    except OSError:
        return _raw_frame(snapshot, "audit-unavailable", "登记的局部坐标审核报告缺失或不可读取")
    if hashlib.sha256(data).hexdigest() != AUDIT_SHA256:
        return _raw_frame(snapshot, "audit-binding-mismatch", "审核报告字节与登记依据不符")
    # The allowlisted report fixes the pipeline, input axis order and source
    # declaration. Client-provided pipelines and CRS identifiers are never used.
    audit = json.loads(data)
    raw_files = [{"relative_path": f["path"].removeprefix("shp_0222-0326/"),
                  "sha256": f["sha256"], "size": f["size"]}
                 for f in audit["source_files"] if f["path"].startswith("shp_0222-0326/")]
    source_files = snapshot.get("source_files", [])
    if (not raw_files or not isinstance(source_files, list)
            or sorted(raw_files, key=lambda f: f["relative_path"]) != source_files):
        return _raw_frame(snapshot, "audit-source-mismatch", "当前源文件清单不属于已审核的数据集")
    frame = snapshot["frame"]
    if frame.get("coordinate_unit") != "degree" or frame.get("transform") != "identity":
        return _raw_frame(snapshot, "audit-frame-mismatch", "当前原始坐标单位或变换与审核依据不同")
    # Every displayed layer's declared unit is hash-bound to an audited PRJ.
    audited_prj = {f["file"]: f["sha256"] for f in audit["declarations"]["prj_files"]}
    declarations = frame.get("unit_sources", [])
    if (not declarations or any(f.get("unit") != "degree"
                               or audited_prj.get(f.get("file")) != f.get("sha256") for f in declarations)):
        return _raw_frame(snapshot, "audit-declaration-mismatch", "源坐标声明与审核登记不一致")
    projection = audit["projection"]
    if (projection.get("status") != "PASS" or projection.get("scope") != "numerical-transform-only"
            or projection.get("input_order") != "longitude,latitude"
            or projection.get("input_unit") != "degree" or projection.get("output_unit") != "m"
            or projection.get("network_enabled") is not False):
        return _raw_frame(snapshot, "audit-transform-unavailable", "登记审核未提供可用的离线局部计算框架")
    bounds = snapshot.get("bounds")
    if (not isinstance(bounds, list) or len(bounds) != 4
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in bounds)
            or not (-180 <= bounds[0] <= bounds[2] <= 180 and -90 < bounds[1] <= bounds[3] < 90)):
        return _raw_frame(snapshot, "invalid-source-bounds", "当前源工程没有有效的经纬度测量范围")
    return {
        "mode": "declared-local-plane", "unit": "m",
        "display_label": "按源声明计算的局部距离（m）", "metric_available": True,
        "reason_code": None, "reason": None,
        "input_order": "longitude,latitude", "input_unit": "degree",
        "output_order": "east,north", "origin": projection["origin"],
        "pipeline": projection["pipeline"], "audit_sha256": AUDIT_SHA256,
        "scope_bounds": list(bounds),
        "assumptions": ["按已登记 PRJ 声明的椭球与经纬度次序计算局部 AEQD 平面长度。",
                        "仅覆盖本源工程包围框；不改原坐标，不作基准纠偏，不计高程。",
                        "数值往返审核不证明测绘精度或绝对地理位置正确。"],
    }


def _capability(snapshot: dict, audit_path: str | Path | None) -> dict:
    frame = _frame(snapshot, audit_path)
    basis = {"source_snapshot_id": snapshot["snapshot_id"],
             "source_content_hash": snapshot["content_hash"], **frame}
    return {"schema": SCHEMA, **basis, "frame_id": "measure:" + _digest(basis),
            "absolute_crs_verified": False, "production_authority": False,
            "metric_edit_allowed": False, "read_only": True,
            "height_included": False}


def measurement_capability(snapshot: dict, *, audit_path: str | Path | None = DEFAULT_AUDIT_PATH) -> dict:
    """Describe the exact units/assumptions the UI must show before measuring.

    ``audit_path`` is a server deployment location only; HTTP callers must not
    select it. Relocation is allowed because the reviewed bytes are pinned.
    """
    _snapshot(snapshot)
    return _capability(snapshot, audit_path)


def _measure(snapshot: dict, points: list[list[float]], *, audit_path: str | Path | None,
             operation: str, identity: dict) -> dict:
    frame = _capability(snapshot, audit_path)
    try:
        raw_length = math.fsum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    except (ValueError, OverflowError) as exc:
        raise MeasurementRejected("measurement-overflow", "测量长度超出可表示范围") from exc
    measured = points
    if frame["metric_available"]:
        bounds = frame["scope_bounds"]
        if any(not (bounds[0] <= p[0] <= bounds[2] and bounds[1] <= p[1] <= bounds[3]) for p in points):
            # A click outside the bound project is still viewable in raw units;
            # do not silently extrapolate the approved inspection scope.
            raw = _raw_frame(snapshot, "points-outside-source-scope", "测量点越出当前源工程登记范围，返回原始坐标长度")
            basis = {"source_snapshot_id": snapshot["snapshot_id"],
                     "source_content_hash": snapshot["content_hash"], **raw}
            frame = {**frame, **raw, "frame_id": "measure:" + _digest(basis)}
            for key in ("origin", "input_order", "input_unit", "output_order", "scope_bounds"):
                frame.pop(key, None)
        else:
            try:
                projection = Transformer.from_pipeline(frame["pipeline"])
                if projection.is_network_enabled:
                    raise ValueError("online transformation is not allowed")
                x, y = projection.transform([p[0] for p in points], [p[1] for p in points], errcheck=True)
                measured = [list(p) for p in zip(x, y)]
            except Exception as exc:
                raise MeasurementRejected("measurement-transform-failed", "登记的离线局部坐标计算失败") from exc
    try:
        length = math.fsum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(measured, measured[1:]))
    except (ValueError, OverflowError) as exc:
        raise MeasurementRejected("measurement-overflow", "测量长度超出可表示范围") from exc
    if not math.isfinite(length) or not math.isfinite(raw_length):
        raise MeasurementRejected("measurement-overflow", "测量长度超出可表示范围")
    return {"schema": SCHEMA, "operation": operation, "length": length,
            "unit": frame["unit"], "display_label": frame["display_label"],
            "point_count": len(points), "segment_count": len(points) - 1,
            "raw_length": raw_length, "raw_unit": snapshot["frame"].get("coordinate_unit", "unknown"),
            "frame": frame, "identity": copy.deepcopy(identity),
            "absolute_crs_verified": False, "production_authority": False,
            "read_only": True, "height_included": False}


def measure_points(snapshot: dict, points: list, *, audit_path: str | Path | None = DEFAULT_AUDIT_PATH) -> dict:
    """Measure the 2-D segment between two clicks in original source axes."""
    _snapshot(snapshot)
    if not isinstance(points, (list, tuple)) or len(points) != 2:
        raise MeasurementRejected("two-points-required", "两点测距必须恰好提供两个源坐标点")
    original = [_point(point) for point in points]
    return _measure(snapshot, original, audit_path=audit_path, operation="point-distance",
                    identity={"points": original})


def measure_object(snapshot: dict, object_id: str, *, audit_path: str | Path | None = DEFAULT_AUDIT_PATH) -> dict:
    """Sum consecutive original vertices of exactly one selected source part.

    No resampling, cross-part join, polygon closure, or inferred point is added.
    Z/M fields remain preserved in the source and do not enter this 2-D length.
    """
    _snapshot(snapshot)
    if not isinstance(object_id, str) or not object_id:
        raise MeasurementRejected("invalid-object-id", "测量要求一个有效的原对象标识")
    objects = [obj for obj in snapshot.get("objects", []) if obj.get("id") == object_id]
    if len(objects) != 1:
        raise MeasurementRejected("source-object-unavailable", "原对象不存在或身份不唯一")
    obj = objects[0]
    if obj.get("shape_type") not in _POLYLINE_TYPES:
        raise MeasurementRejected("source-object-not-line", "当前原对象没有可测量的线或环点列")
    points = [_point(point) for point in obj.get("points", [])]
    if len(points) < 2:
        raise MeasurementRejected("source-object-empty-line", "原对象没有至少两个线点")
    return _measure(snapshot, points, audit_path=audit_path, operation="source-polyline-length",
                    identity={"object_id": object_id, "source_ref": obj.get("source_ref"),
                              "points_sha256": _digest(points), "closure_added": False,
                              "parts_joined": False})
