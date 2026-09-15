# -*- coding: utf-8 -*-
"""冻结 v1.31 金凤 OpenDRIVE 基线的哈希与结构清单。

只读取 ``out/closed-loop-inputs.json`` 指向的正式产物，不重新生成，也不修改
任何源数据。输出用于 v1.33/G11 的回归对照：每条 road 的 planView primitive、
laneOffset、laneSection、width/border 复杂度和 provenance 状态均被锁定。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INDEX = ROOT / "out/closed-loop-inputs.json"
DEFAULT_REPORT = ROOT / "out/closed-loop-report.json"
DEFAULT_OUTPUT = ROOT / "out/v131-opendrive-baseline.json"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _primitive(el: ET.Element) -> dict:
    out = {
        "s": float(el.get("s", "0")),
        "x": float(el.get("x", "0")),
        "y": float(el.get("y", "0")),
        "hdg": float(el.get("hdg", "0")),
        "length_m": float(el.get("length", "0")),
    }
    for tag in ("line", "arc", "spiral", "poly3", "paramPoly3"):
        child = el.find(tag)
        if child is None:
            continue
        out["type"] = tag
        out["parameters"] = dict(sorted(child.attrib.items()))
        return out
    out["type"] = "unknown"
    out["parameters"] = {}
    return out


def _road_provenance(road: ET.Element) -> dict:
    statuses: Counter[str] = Counter()
    support_kinds: Counter[str] = Counter()
    roles: Counter[str] = Counter()
    invalid = 0
    for ud in road.findall(".//lane/userData"):
        if ud.get("code") != "mapforge.provenance/v1":
            continue
        try:
            value = json.loads(ud.get("value", "{}"))
        except (TypeError, ValueError):
            invalid += 1
            continue
        statuses[str(value.get("status", "UNKNOWN"))] += 1
        support_kinds[str(value.get("support_kind", "UNKNOWN"))] += 1
        roles[str(value.get("role", "UNKNOWN"))] += 1
    return {
        "statuses": dict(sorted(statuses.items())),
        "support_kinds": dict(sorted(support_kinds.items())),
        "roles": dict(sorted(roles.items())),
        "invalid_records": invalid,
    }


def _section_snapshot(section: ET.Element) -> dict:
    lanes = []
    for side in ("left", "center", "right"):
        for lane in section.findall(f"{side}/lane"):
            lanes.append({
                "side": side,
                "id": int(lane.get("id", "0")),
                "type": lane.get("type", "none"),
                "width_records": len(lane.findall("width")),
                "border_records": len(lane.findall("border")),
                "speed_records": len(lane.findall("speed")),
            })
    return {"s": float(section.get("s", "0")), "lanes": lanes}


def _road_snapshot(road: ET.Element) -> dict:
    primitives = [_primitive(g) for g in road.findall("planView/geometry")]
    sections = [_section_snapshot(s) for s in road.findall("lanes/laneSection")]
    return {
        "id": road.get("id"),
        "name": road.get("name", ""),
        "junction": road.get("junction", "-1"),
        "length_m": float(road.get("length", "0")),
        "planview": primitives,
        "planview_count": len(primitives),
        "lane_offset_count": len(road.findall("lanes/laneOffset")),
        "lane_section_count": len(sections),
        "width_record_count": sum(
            lane["width_records"] for sec in sections for lane in sec["lanes"]),
        "border_record_count": sum(
            lane["border_records"] for sec in sections for lane in sec["lanes"]),
        "sections": sections,
        "provenance": _road_provenance(road),
    }


def freeze(index_path: Path, report_path: Path, output_path: Path) -> dict:
    index = json.loads(index_path.read_text(encoding="utf-8"))
    entries = []
    aggregate_types: Counter[str] = Counter()
    aggregate = {
        "files": 0, "roads": 0, "planview_primitives": 0,
        "lane_offsets": 0, "lane_sections": 0,
        "width_records": 0, "border_records": 0,
    }
    for entry in index.get("entries", []):
        artifact = ROOT / entry["artifact"]
        if not artifact.exists():
            raise FileNotFoundError(artifact)
        actual_hash = _sha256(artifact)
        root = ET.parse(artifact).getroot()
        roads = [_road_snapshot(r) for r in root.findall("road")]
        for road in roads:
            aggregate["roads"] += 1
            aggregate["planview_primitives"] += road["planview_count"]
            aggregate["lane_offsets"] += road["lane_offset_count"]
            aggregate["lane_sections"] += road["lane_section_count"]
            aggregate["width_records"] += road["width_record_count"]
            aggregate["border_records"] += road["border_record_count"]
            aggregate_types.update(p["type"] for p in road["planview"])
        aggregate["files"] += 1
        entries.append({
            "case": entry["case"],
            "pipeline": entry["pipeline"],
            "artifact": entry["artifact"],
            "artifact_size": artifact.stat().st_size,
            "artifact_sha256": actual_hash,
            "index_artifact_sha256": entry.get("artifact_sha256"),
            "hash_matches_index": actual_hash == entry.get("artifact_sha256"),
            "roads": roads,
        })
    aggregate["primitive_types"] = dict(sorted(aggregate_types.items()))
    result = {
        "schema": "mapforge/opendrive-baseline/v1",
        "baseline_id": "v1.31-jinfeng-opendrive-14",
        "source_index": str(index_path.relative_to(ROOT)),
        "source_index_sha256": _sha256(index_path),
        "closed_loop_report": (
            str(report_path.relative_to(ROOT)) if report_path.exists() else None),
        "closed_loop_report_sha256": (
            _sha256(report_path) if report_path.exists() else None),
        "aggregate": aggregate,
        "entries": entries,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=Path, default=DEFAULT_INDEX)
    ap.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = ap.parse_args()
    result = freeze(args.index, args.report, args.output)
    mismatches = [
        x["artifact"] for x in result["entries"] if not x["hash_matches_index"]]
    print(json.dumps(result["aggregate"], ensure_ascii=False, indent=2))
    print(f"baseline: {args.output}")
    if mismatches:
        print("hash mismatch:", ", ".join(mismatches))
        return 1
    print("artifact hashes: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
