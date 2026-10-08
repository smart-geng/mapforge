"""Read-only, source-first projects. No XODR generation or geometry inference.

Business identifiers are attributes, never row identity. Unknown layers remain
in the bound file manifest; the displayed subset is selected by declared source
relations, not proximity or interpreted/derived lanes.
"""
from __future__ import annotations

import copy
from array import array
import datetime
import hashlib
import json
import math
import struct
from pathlib import Path
from typing import Any

import shapefile
import yaml

from mapforge.adapters.shp.profile_source import ProfileSource

SCHEMA = "mapforge/workbench-source/v1"
_ROOT = Path(__file__).resolve().parents[2]
_LINEAR_UNITS = {"m": 1.0, "cm": 0.01, "mm": 0.001}


def _json_value(value: Any) -> Any:
    if isinstance(value, (datetime.date, datetime.datetime)):
        return {"type": "date", "value": value.isoformat()}
    if isinstance(value, bytes):
        return {"type": "bytes", "hex": value.hex()}
    if isinstance(value, float) and not math.isfinite(value):
        return {"type": "nonfinite", "value": str(value)}
    if isinstance(value, dict):
        return {str(k): _json_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, array)):
        return [_json_value(v) for v in value]
    return value


def _digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _file_hash(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _profile_path(ref: str | Path) -> Path:
    path = Path(ref)
    if not path.is_file():
        path = _ROOT / "profiles" / "shp" / f"{ref}.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"SHP Profile 不存在：{ref}")
    return path.resolve()


def _inside(root: Path, relative: str) -> Path:
    target = (root / relative).resolve()
    if not target.is_relative_to(root):
        raise ValueError(f"源路径不能越出登记目录：{relative}")
    return target


def _manifest(root: Path) -> list[dict]:
    entries = []
    for path in sorted(root.rglob("*"), key=lambda p: p.relative_to(root).as_posix()):
        if not path.is_file():
            continue
        _inside(root, path.relative_to(root).as_posix())
        before = path.stat()
        digest = _file_hash(path)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError(f"读取时源文件变化：{path.name}")
        entries.append({"relative_path": path.relative_to(root).as_posix(),
                        "size": after.st_size, "sha256": digest})
    return entries


def _sid(value: Any) -> str:
    return "" if value is None else str(value).strip()


def _issue(code: str, message: str, *, objects: list[str] | None = None,
           severity: str = "warning", **details: Any) -> dict:
    item = {"code": code, "severity": severity, "message": message,
            "object_ids": sorted(objects or []), "details": details}
    return {"id": "issue:" + _digest(item)[:24], **item}


class SourceCatalog:
    """Immutable-in-use raw catalog shared by multiple selected junctions.

    This deliberately does not construct ProfileSource: its conversion validator
    supplies a default length unit and rejects unknown units. Inspection must
    still work in that situation. Only its lossless part-splitting helper is
    reused. Read errors are visible, and the complete original file set remains
    hash-bound even for unmapped layers.
    """

    def __init__(self, source_dir: str | Path, profile_path: str | Path):
        self.source_dir = Path(source_dir).resolve()
        if not self.source_dir.is_dir():
            raise NotADirectoryError(str(self.source_dir))
        self.profile_path = _profile_path(profile_path)
        self.profile_hash = _file_hash(self.profile_path)
        self.profile = yaml.safe_load(self.profile_path.read_text(encoding="utf-8-sig"))
        if not isinstance(self.profile, dict) or not isinstance(self.profile.get("layers"), dict):
            raise ValueError("Profile 必须声明 layers 对象")
        self._file_stats = {p.relative_to(self.source_dir).as_posix():
                            (p.stat().st_size, p.stat().st_mtime_ns)
                            for p in self.source_dir.rglob("*") if p.is_file()}
        self.files = _manifest(self.source_dir)
        self.snapshot_id = _digest({"files": self.files, "profile_sha256": self.profile_hash})
        self.layers = self.profile["layers"]
        self.rows: dict[str, list[dict]] = {}
        self.load_issues: list[dict] = []
        for role, spec in sorted(self.layers.items()):
            self.rows[role] = self._load(role, spec)
        self.assert_unchanged()

    def _stat(self, relative: str) -> tuple[int, int]:
        st = _inside(self.source_dir, relative).stat()
        return st.st_size, st.st_mtime_ns

    def assert_unchanged(self) -> None:
        """Cheap per-open guard. Persisted reopen uses full hash verification."""
        current = {p.relative_to(self.source_dir).as_posix()
                   for p in self.source_dir.rglob("*") if p.is_file()}
        if current != set(self._file_stats):
            raise ValueError("源目录文件集合已变化，请重新导入")
        if any(self._stat(name) != st for name, st in self._file_stats.items()):
            raise ValueError("源内容已变化，请重新导入")
        if _file_hash(self.profile_path) != self.profile_hash:
            raise ValueError("Profile 已变化，请重新导入")

    def _load(self, role: str, spec: dict) -> list[dict]:
        if not isinstance(spec, dict) or not isinstance(spec.get("file"), str):
            self.load_issues.append(_issue("invalid-layer-profile", "图层缺少文件声明", role=role))
            return []
        path = _inside(self.source_dir, spec["file"])
        layer = path.relative_to(self.source_dir).as_posix()
        records = []
        try:
            with shapefile.Reader(str(path), encoding=self.profile.get("encoding", "utf-8")) as reader:
                field_names = [field[0] for field in reader.fields[1:]]
                for index, sr in enumerate(reader.iterShapeRecords()):
                    attributes = _json_value(dict(zip(field_names, sr.record)))
                    parts = _json_value(ProfileSource._raw_parts(sr.shape))
                    if any(not isinstance(v, (int, float)) or not math.isfinite(v)
                           for part in parts for point in part for v in point):
                        raise ValueError("几何含非有限坐标")
                    records.append({"layer": layer, "record_index": index,
                                    "raw_attributes": attributes, "parts": parts,
                                    "shape_type": sr.shape.shapeType,
                                    "raw_z": _json_value(getattr(sr.shape, "z", [])),
                                    "raw_m": _json_value(getattr(sr.shape, "m", [])),
                                    "field_schema": _json_value(reader.fields[1:])})
        except (OSError, ValueError, shapefile.ShapefileException, UnicodeError, struct.error, EOFError) as exc:
            # Never present a partially decoded table as a complete table.
            self.load_issues.append(_issue("source-layer-unreadable", "源图层无法完整读取",
                                           severity="error", role=role, layer=layer,
                                           error=str(exc), decoded_rows=len(records)))
            return []
        return records

    def _value(self, role: str, row: dict, field: str) -> str:
        name = self.layers[role].get("fields", {}).get(field)
        return _sid(row["raw_attributes"].get(name)) if name else ""

    def _ids(self, role: str, field: str = "id") -> dict[str, list[dict]]:
        values: dict[str, list[dict]] = {}
        for row in self.rows.get(role, []):
            ident = self._value(role, row, field)
            if ident:
                values.setdefault(ident, []).append(row)
        return values

    def _split(self, role: str, value: str) -> list[str]:
        return [v.strip() for v in value.split(self.layers[role].get("list_sep", ";")) if v.strip()]

    def _object_id(self, row: dict, part: int | None) -> str:
        return "src:" + _digest([self.snapshot_id, row["layer"], row["record_index"], part])

    def _row_ids(self, row: dict) -> list[str]:
        return [self._object_id(row, i) for i in range(len(row["parts"]))] or [self._object_id(row, None)]

    def junctions(self) -> list[dict]:
        return [{"id": self._value("junction", row, "id"),
                 "name": self._value("junction", row, "name"),
                 "record_index": row["record_index"]} for row in self.rows.get("junction", [])]

    def list_junctions(self) -> list[dict]:
        return self.junctions()

    def snapshot(self, junction_id: str, *, frame: dict | None = None) -> dict:
        self.assert_unchanged()
        selected_junctions = self._ids("junction").get(str(junction_id), [])
        if not selected_junctions:
            raise ValueError(f"源路口不存在或无法读取：{junction_id}")
        selected: dict[str, list[dict]] = {"junction": selected_junctions}
        road_ids = {rid for row in selected_junctions for field in ("enter_roads", "leave_roads")
                    for rid in self._split("junction", self._value("junction", row, field))}
        selected["road"] = [row for row in self.rows.get("road", [])
                            if self._value("road", row, "id") in road_ids]
        selected["road_center"] = [row for row in self.rows.get("road_center", [])
                                   if self._value("road_center", row, "road") in road_ids]
        lane_roles = ("lane", "lane_merge")
        primary_lane_ids = {self._value(role, row, "id") for role in lane_roles
                            for row in self.rows.get(role, [])
                            if self._value(role, row, "road") in road_ids}
        primary_lane_ids.discard("")
        # One declared topology hop includes via records but cannot expand to
        # the entire city's lane graph. Scope is explicit in the result.
        selected["topo"] = [row for row in self.rows.get("topo", [])
                            if any(self._value("topo", row, side) in primary_lane_ids
                                   for side in ("from", "to"))]
        lane_ids = primary_lane_ids | {self._value("topo", row, side)
                                      for row in selected["topo"] for side in ("from", "to")}
        lane_ids.discard("")
        # Expansion chooses objects, not a licence to omit their original
        # relations. Include the induced graph among already-selected lanes,
        # without taking another topology hop into the surrounding network.
        selected["topo"] = [row for row in self.rows.get("topo", [])
                            if any(self._value("topo", row, side) in primary_lane_ids
                                   for side in ("from", "to"))
                            or all(self._value("topo", row, side) in lane_ids
                                   for side in ("from", "to"))]
        for role in lane_roles:
            selected[role] = [row for row in self.rows.get(role, [])
                              if self._value(role, row, "id") in lane_ids]
        selected["lane_boundary_rel"] = [row for row in self.rows.get("lane_boundary_rel", [])
                                          if self._value("lane_boundary_rel", row, "lane") in lane_ids]
        boundary_ids = {self._value("lane_boundary_rel", row, "boundary")
                        for row in selected["lane_boundary_rel"]}
        selected["boundary"] = [row for row in self.rows.get("boundary", [])
                                if self._value("boundary", row, "id") in boundary_ids]
        selected["stop_line"] = [row for row in self.rows.get("stop_line", [])
                                 if set(self._split("stop_line", self._value("stop_line", row, "lane_refs"))) & lane_ids]
        for rows in selected.values():
            rows.sort(key=lambda row: (row["layer"], row["record_index"]))
        objects: dict[str, dict] = {}
        for role, rows in sorted(selected.items()):
            for row in rows:
                for part_index in list(range(len(row["parts"]))) or [None]:
                    ident = self._object_id(row, part_index)
                    if ident in objects:
                        objects[ident]["roles"].append(role)
                        continue
                    source_ref = {"snapshot_id": self.snapshot_id, "layer": row["layer"],
                                  "record_index": row["record_index"], "part_index": part_index}
                    objects[ident] = {"id": ident, "source_ref": source_ref, "role": role,
                                      "roles": [role], "business_id": self._value(role, row, "id"),
                                      "raw_attributes": row["raw_attributes"],
                                      "points": [] if part_index is None else row["parts"][part_index],
                                      "shape_type": row["shape_type"], "raw_z": row["raw_z"],
                                      "raw_m": row["raw_m"], "field_schema": row["field_schema"],
                                      "status": "PASSTHROUGH"}
        issues = copy.deepcopy(self.load_issues)
        references = []
        indexes = {role: self._ids(role) for role in self.rows}

        def reference(role: str, row: dict, field: str, target: str, target_roles: tuple[str, ...]) -> None:
            targets = [r for tr in target_roles for r in indexes.get(tr, {}).get(target, [])]
            target_object_ids = sorted({oid for r in targets for oid in self._row_ids(r)})
            source_object_ids = self._row_ids(row)
            status = "missing" if not targets else "resolved" if len(targets) == 1 else "multiple-records"
            references.append({"source_object_ids": source_object_ids, "field": field,
                               "target_business_id": target, "target_roles": list(target_roles),
                               "target_object_ids": target_object_ids, "status": status,
                               "in_view": bool(target_object_ids) and all(x in objects for x in target_object_ids),
                               "target_scope": ("missing" if not targets else "inside-view"
                                                if all(x in objects for x in target_object_ids) else "outside-view")})
            if status == "missing":
                issues.append(_issue("missing-source-reference", "原始引用没有对应记录",
                                     objects=source_object_ids, role=role, field=field,
                                     target=target, target_roles=list(target_roles)))

        for role, rows in sorted(selected.items()):
            by_id: dict[str, list[dict]] = {}
            for row in rows:
                sid = self._value(role, row, "id")
                if sid:
                    by_id.setdefault(sid, []).append(row)
                if role == "junction":
                    for field in ("enter_roads", "leave_roads"):
                        for target in self._split(role, self._value(role, row, field)):
                            reference(role, row, field, target, ("road",))
                elif role in lane_roles:
                    reference(role, row, "road", self._value(role, row, "road"), ("road",))
                elif role == "road_center":
                    reference(role, row, "road", self._value(role, row, "road"), ("road",))
                elif role == "topo":
                    for field in ("from", "to"):
                        reference(role, row, field, self._value(role, row, field), lane_roles)
                elif role == "lane_boundary_rel":
                    reference(role, row, "lane", self._value(role, row, "lane"), lane_roles)
                    reference(role, row, "boundary", self._value(role, row, "boundary"), ("boundary",))
                    side = self._value(role, row, "side")
                    side_values = self.layers[role].get("side_values", {})
                    if side not in {_sid(v) for v in side_values.values()}:
                        issues.append(_issue("unknown-boundary-side", "原 SIDE 未被 Profile 声明，未推断物理左右",
                                             objects=self._row_ids(row), raw_side=side))
                elif role == "stop_line":
                    for target in self._split(role, self._value(role, row, "lane_refs")):
                        reference(role, row, "lane_refs", target, lane_roles)
            for sid, duplicates in by_id.items():
                if len(duplicates) > 1:
                    issues.append(_issue("duplicate-business-id", "同层业务 ID 重复，原记录分别保留",
                                         objects=[oid for r in duplicates for oid in self._row_ids(r)],
                                         role=role, business_id=sid))
        project_frame = self._frame(frame)
        if not project_frame["metric_edit_allowed"]:
            issues.append(_issue("metric-edit-unavailable", "当前坐标/单位依据不满足米制编辑条件，源查看仍可用",
                                 frame_status=project_frame["status"]))
        issues.append(_issue("absolute-crs-unverified", "绝对 CRS 未完成核验，不能生产交付", severity="error"))
        points = [p for obj in objects.values() for p in obj["points"]]
        bounds = ([min(p[0] for p in points), min(p[1] for p in points),
                   max(p[0] for p in points), max(p[1] for p in points)] if points else None)
        known_stems = {spec.get("file") for spec in self.layers.values() if isinstance(spec, dict)}
        unmapped = [f["relative_path"] for f in self.files
                    if Path(f["relative_path"]).with_suffix("").as_posix() not in known_stems]
        result = {"schema": SCHEMA, "snapshot_id": self.snapshot_id,
                  "source_hash": self.snapshot_id, "source_files": self.files,
                  "profile": {"name": self.profile.get("profile", "unnamed"),
                              "sha256": self.profile_hash},
                  "locator": {"source_dir": str(self.source_dir), "profile_path": str(self.profile_path)},
                  "junction_id": str(junction_id), "frame": project_frame,
                  "objects": list(objects.values()), "issues": sorted(issues, key=lambda i: i["id"]),
                  "references": references, "bounds": bounds,
                  "scope": {"selection": "declared-road-membership-and-one-topology-hop",
                            "topology_relations": "primary-incident-and-all-relations-among-selected-lanes",
                            "road_ids": sorted(road_ids), "lane_ids": sorted(lane_ids),
                            "complete_raw_catalog": False,
                            "unmapped_files": unmapped,
                            "unmapped_status": "PASSTHROUGH"},
                  "capabilities": {"view_source": True, "save_draft": True,
                                   "metric_edit": project_frame["metric_edit_allowed"],
                                   "compile": False, "production_export": False}}
        result["content_hash"] = _digest({k: v for k, v in result.items() if k != "locator"})
        return copy.deepcopy(result)

    def _frame(self, explicit: dict | None) -> dict:
        # Unit of scalar width fields is not the unit of geometry coordinates.
        # WKT tells us declared units, never proves absolute georeferencing.
        declarations = []
        units = set()
        from pyproj import CRS
        from pyproj.exceptions import CRSError
        for file in self.files:
            if Path(file["relative_path"]).suffix.lower() != ".prj":
                continue
            path = _inside(self.source_dir, file["relative_path"])
            try:
                crs = CRS.from_wkt(path.read_text(encoding="utf-8-sig"))
                unit = "degree" if crs.is_geographic else crs.axis_info[0].unit_name
                unit = {"metre": "m", "meter": "m"}.get(unit, unit)
                units.add(unit)
                declarations.append({"file": file["relative_path"], "sha256": file["sha256"], "unit": unit})
            except (ValueError, OSError, UnicodeError, CRSError):
                declarations.append({"file": file["relative_path"], "sha256": file["sha256"], "unit": "unknown"})
                units.add("unknown")
        unit = next(iter(units)) if len(units) == 1 else "unknown"
        version_evidence = []
        # IBD's metadata enumerates coordinate units separately from scalar
        # widths. It is a declaration, not an absolute control-point survey.
        if self.profile.get("profile") == "ibd-smarteditor-v1" and (self.source_dir / "IBD_VERSION.dbf").is_file():
            try:
                with shapefile.Reader(dbf=str(self.source_dir / "IBD_VERSION.dbf"),
                                      encoding=self.profile.get("encoding", "utf-8")) as reader:
                    fields = [f[0] for f in reader.fields[1:]]
                    for index, rec in enumerate(reader.iterRecords()):
                        attrs = _json_value(dict(zip(fields, rec)))
                        version_evidence.append({"file": "IBD_VERSION.dbf", "record_index": index,
                                                 "raw_attributes": attrs})
                version_units = {_sid(r["raw_attributes"].get("COORD_UNIT")) for r in version_evidence}
                if version_units == {"2"}:
                    unit = "degree" if unit in ("unknown", "degree") else "conflicting"
            except (OSError, ValueError, shapefile.ShapefileException, UnicodeError, struct.error, EOFError) as exc:
                version_evidence = [{"file": "IBD_VERSION.dbf", "error": str(exc)}]
        explicit = copy.deepcopy(explicit or {})
        if explicit:
            if explicit.get("kind", "raw") != "raw":
                raise ValueError("首轮源适配器只接受 raw 坐标框架，投影变换尚未接入")
            if not explicit.get("evidence"):
                raise ValueError("显式坐标单位必须提供 evidence")
            proposed = explicit.get("coordinate_unit", unit)
            if unit != "unknown" and proposed != unit:
                raise ValueError("显式单位与源 PRJ 声明不一致")
            unit = proposed
        return {"kind": "raw", "coordinate_unit": unit,
                "attribute_length_unit": (self.profile.get("units") or {}).get("length", "unknown"),
                "origin": [0.0, 0.0], "transform": "identity", "proj_pipeline": None,
                "status": "declared-unverified" if unit != "unknown" else "unknown",
                "absolute_crs_status": "unverified", "crs_declaration": self.profile.get("crs"),
                "unit_sources": declarations, "explicit_evidence": explicit.get("evidence"),
                "version_evidence": version_evidence,
                "metres_per_unit": _LINEAR_UNITS.get(unit),
                "metric_edit_allowed": False}


def build_source_snapshot(source_dir: str | Path, profile_path: str | Path,
                          junction_id: str, *, frame: dict | None = None) -> dict:
    return SourceCatalog(source_dir, profile_path).snapshot(junction_id, frame=frame)


def verify_source_snapshot(snapshot: dict, source_dir: str | Path,
                           profile_path: str | Path | None = None) -> dict:
    """Full byte check, allowing relocation but never silently rebinding bytes."""
    root = Path(source_dir).resolve()
    issues = []
    expected = {entry["relative_path"]: entry for entry in snapshot["source_files"]}
    if not root.is_dir():
        return {"matches": False, "issues": [{"code": "source-directory-missing", "path": str(root)}]}
    current = {entry["relative_path"]: entry for entry in _manifest(root)}
    for name in sorted(set(expected) | set(current)):
        if name not in current:
            issues.append({"code": "source-file-missing", "relative_path": name})
        elif name not in expected:
            issues.append({"code": "source-file-added", "relative_path": name})
        elif current[name] != expected[name]:
            issues.append({"code": "source-file-changed", "relative_path": name})
    path = Path(profile_path) if profile_path is not None else Path(snapshot["locator"]["profile_path"])
    if not path.is_file():
        issues.append({"code": "profile-missing"})
    elif _file_hash(path) != snapshot["profile"]["sha256"]:
        issues.append({"code": "profile-changed"})
    return {"matches": not issues, "issues": issues}
