# -*- coding: utf-8 -*-
"""机械汇总金凤 G8 校准证据；只写候选文件，不覆盖源 policy。"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Callable

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.validate.g8_model import (GATE_SCHEMA, POLICY_SCHEMA, load_policy,
                                        object_sha256, write_json)  # noqa: E402

REPORT_SCHEMA = "mapforge/g8-calibration-report/v1"
DEFAULT_POLICY = ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"
DEFAULT_INDEX = ROOT / "out/closed-loop-inputs.json"
DEFAULT_REPORT = ROOT / "out/g8-calibration-report.json"
DEFAULT_CANDIDATE = ROOT / "out/g8-opendrive-jinfeng-v1.candidate.yaml"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _rooted(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else ROOT / path


def _number(value: Any) -> float | None:
    return float(value) if isinstance(value, (int, float)) else None


def _extreme(rows: list[dict], getter: Callable[[dict], Any], *, minimum=False) -> dict:
    values = [(value, row) for row in rows if (value := _number(getter(row))) is not None]
    if not values:
        return {"value": None, "case": None, "pipeline": None, "source_lane_id": None}
    value, row = (min if minimum else max)(values, key=lambda item: item[0])
    return {"value": value, "case": row["case"], "pipeline": row["pipeline"],
            "source_lane_id": row["source_lane_id"]}


def _class_summary(rows: list[dict]) -> dict:
    def metric(direction, name):
        return lambda row: (row["lane"].get(direction) or {}).get(name)

    def endpoint(name):
        return lambda row: (row["lane"].get("endpoint") or {}).get(name)

    def coverage(name):
        return lambda row: (row["lane"].get("coverage") or {}).get(name)

    stop_rows = [row for row in rows if (row["lane"].get("stopline") or {}).get("applicable")]
    return {
        "lanes": len(rows),
        "source_to_target": {
            "median_max_m": _extreme(rows, metric("source_to_target", "median_m")),
            "p95_max_m": _extreme(rows, metric("source_to_target", "p95_m")),
            "max_max_m": _extreme(rows, metric("source_to_target", "max_m")),
        },
        "target_to_source": {
            "median_max_m": _extreme(rows, metric("target_to_source", "median_m")),
            "p95_max_m": _extreme(rows, metric("target_to_source", "p95_m")),
            "max_max_m": _extreme(rows, metric("target_to_source", "max_m")),
        },
        "endpoint": {
            "travel_start_max_m": _extreme(rows, endpoint("travel_start_m")),
            "travel_end_max_m": _extreme(rows, endpoint("travel_end_m")),
        },
        "stopline": {
            "applicable_lanes": len(stop_rows),
            "delta_max_m": _extreme(stop_rows, lambda row:
                                     (row["lane"].get("stopline") or {}).get("delta_m")),
        },
        "coverage": {
            "source_min": _extreme(rows, coverage("source"), minimum=True),
            "target_min": _extreme(rows, coverage("target"), minimum=True),
        },
        "length_ratio": {
            "min": _extreme(rows, coverage("target_source_length_ratio"), minimum=True),
            "max": _extreme(rows, coverage("target_source_length_ratio")),
        },
    }


def _ceiling_violations(row: dict, ceiling: dict) -> list[dict]:
    lane = row["lane"]
    checks = []
    for direction in ("source_to_target", "target_to_source"):
        stats = lane.get(direction) or {}
        for metric, limit_key in (("median_m", "median_max_m"),
                                  ("p95_m", "p95_max_m"),
                                  ("max_m", "max_max_m")):
            checks.append((f"{direction}.{metric}", stats.get(metric),
                           ceiling.get(limit_key), "max"))
    endpoint = lane.get("endpoint") or {}
    checks.extend([
        ("endpoint.travel_start_m", endpoint.get("travel_start_m"),
         ceiling.get("endpoint_max_m"), "max"),
        ("endpoint.travel_end_m", endpoint.get("travel_end_m"),
         ceiling.get("endpoint_max_m"), "max"),
    ])
    stopline = lane.get("stopline") or {}
    if stopline.get("applicable"):
        checks.append(("stopline.delta_m", stopline.get("delta_m"),
                       ceiling.get("stopline_delta_max_m"), "max"))
    coverage = lane.get("coverage") or {}
    checks.extend([
        ("coverage.source", coverage.get("source"), ceiling.get("coverage_min"), "min"),
        ("coverage.target", coverage.get("target"), ceiling.get("coverage_min"), "min"),
    ])
    failures = []
    for metric, value, limit, kind in checks:
        value, limit = _number(value), _number(limit)
        if value is None or limit is None:
            continue
        failed = value > limit + 1e-9 if kind == "max" else value < limit - 1e-9
        if failed:
            failures.append({"case": row["case"], "pipeline": row["pipeline"],
                             "source_lane_id": row["source_lane_id"],
                             "policy_class": row["policy_class"], "metric": metric,
                             "value": value, "limit": limit, "limit_kind": kind})
    return failures


def _split_report(name: str, nodes: list[str], files: list[dict], ceiling: dict) -> dict:
    rows = [row for file in files for row in file["lane_rows"]]
    by_class: dict[str, list[dict]] = {}
    for row in rows:
        by_class.setdefault(row["policy_class"], []).append(row)
    case_names = sorted(set(nodes), key=lambda value: value.lower())
    by_case = {}
    for case in case_names:
        case_files = [file for file in files if file["case"] == case]
        case_rows = [row for file in case_files for row in file["lane_rows"]]
        by_case[case] = {
            "files": len(case_files),
            "pipelines": sorted(file["pipeline"] for file in case_files),
            "lanes": len(case_rows),
            "classes": {class_id: len([row for row in case_rows
                                       if row["policy_class"] == class_id])
                        for class_id in sorted({row["policy_class"] for row in case_rows})},
            "failure_reasons": [reason for file in case_files
                                for reason in file["non_lifecycle_failure_reasons"]],
        }
    ceiling_failures = [failure for row in rows
                        for failure in _ceiling_violations(row, ceiling)]
    gate_failures = [{"case": file["case"], "pipeline": file["pipeline"],
                      "reason": reason}
                     for file in files for reason in file["non_lifecycle_failure_reasons"]]
    expected_files = len(nodes) * 2
    expected_pipelines = {"map-to-opendrive", "shp-to-opendrive"}
    matrix_ok = (len(files) == expected_files
                 and all({file["pipeline"] for file in files if file["case"] == case}
                         == expected_pipelines for case in nodes))
    failure_reasons = []
    if not matrix_ok:
        failure_reasons.append(f"{name}_matrix_incomplete")
    if gate_failures:
        failure_reasons.append(f"{name}_gate_failures")
    if ceiling_failures:
        failure_reasons.append(f"{name}_engineering_ceiling_exceeded")
    return {
        "nodes": nodes,
        "expected_files": expected_files,
        "files": len(files),
        "matrix_ok": matrix_ok,
        "lanes": len(rows),
        "by_case": by_case,
        "by_class": {class_id: _class_summary(by_class[class_id])
                     for class_id in sorted(by_class)},
        "gate_failures": gate_failures,
        "engineering_ceiling_violations": ceiling_failures,
        "failure_reasons": failure_reasons,
    }


def _candidate(policy: dict, index_path: Path, source_policy_sha256: str) -> dict:
    candidate = copy.deepcopy({k: v for k, v in policy.items()
                               if k not in ("_path", "policy_sha256")})
    version = str(candidate.get("version", "1.0"))
    if version.endswith("-draft"):
        version = version[:-6]
    candidate["version"] = version
    candidate["lifecycle"] = "active"
    calibration = candidate.setdefault("calibration", {})
    calibration["promotion"] = {
        "method": "unchanged-pre-registered-limits",
        "threshold_fitting": False,
        "engineering_ceiling_relaxation": False,
        "source_policy_sha256": source_policy_sha256,
        "input_index_sha256": _sha256(index_path),
    }
    return candidate


def calibrate(policy_path: Path, index_path: Path, report_path: Path,
              candidate_path: Path) -> dict:
    failure_reasons = []
    if not policy_path.exists():
        failure_reasons.append("policy_missing")
        policy = None
    else:
        policy = load_policy(policy_path)
    if not index_path.exists():
        failure_reasons.append("input_index_missing")
        index = {}
    else:
        index = json.loads(index_path.read_text(encoding="utf-8"))

    lifecycle = (policy or {}).get("lifecycle")
    mode = "verify" if lifecycle == "active" else "calibrate"
    if not policy or policy.get("schema") != POLICY_SCHEMA:
        failure_reasons.append("policy_schema_unsupported")
    if policy and lifecycle not in ("draft", "active"):
        failure_reasons.append("source_policy_lifecycle_unsupported")
    if index.get("schema") != "mapforge/closed-loop-inputs/v1":
        failure_reasons.append("input_index_schema_unsupported")
    if index.get("errors"):
        failure_reasons.append("input_index_has_errors")

    calibration = (policy or {}).get("calibration") or {}
    calibration_nodes = [str(x) for x in calibration.get("calibration_nodes", [])]
    locked_nodes = [str(x) for x in calibration.get("locked_validation_nodes", [])]
    if not calibration_nodes or not locked_nodes or set(calibration_nodes) & set(locked_nodes):
        failure_reasons.append("invalid_split_protocol")
    policy_sha = (policy or {}).get("policy_sha256")
    files = []
    for entry in index.get("entries", []):
        artifact = _rooted(entry.get("artifact", ""))
        gate_ref = (entry.get("sidecars") or {}).get("gate_result")
        gate_path = _rooted(gate_ref) if gate_ref else None
        entry_errors = []
        if not artifact.is_file():
            entry_errors.append("artifact_missing")
        elif entry.get("artifact_sha256") != _sha256(artifact):
            entry_errors.append("artifact_hash_mismatch")
        if gate_path is None or not gate_path.is_file():
            entry_errors.append("gate_result_missing")
            gate = {}
        else:
            gate = json.loads(gate_path.read_text(encoding="utf-8"))
        if gate.get("schema") != GATE_SCHEMA:
            entry_errors.append("gate_result_schema_unsupported")
        if (gate.get("policy") or {}).get("sha256") != policy_sha:
            entry_errors.append("gate_policy_hash_mismatch")
        if entry.get("policy_sha256") != policy_sha:
            entry_errors.append("index_policy_hash_mismatch")
        if entry_errors:
            failure_reasons.extend(f"{entry.get('case')}:{entry.get('pipeline')}:{error}"
                                   for error in entry_errors)
        non_lifecycle = [reason for reason in gate.get("failure_reasons", [])
                         if reason != "policy_not_active"]
        lane_rows = [{"case": entry.get("case"), "pipeline": entry.get("pipeline"),
                      "source_lane_id": lane.get("source_lane_id"),
                      "policy_class": lane.get("policy_class"), "lane": lane}
                     for lane in gate.get("per_lane", [])]
        files.append({"case": entry.get("case"), "pipeline": entry.get("pipeline"),
                      "artifact": entry.get("artifact"), "gate_result": gate_ref,
                      "gate_status": gate.get("status"),
                      "non_lifecycle_failure_reasons": non_lifecycle,
                      "lane_rows": lane_rows})

    known_nodes = set(calibration_nodes) | set(locked_nodes)
    unknown_cases = sorted({file["case"] for file in files if file["case"] not in known_nodes})
    if unknown_cases:
        failure_reasons.append("input_contains_unassigned_cases")
    ceiling = calibration.get("engineering_ceiling") or {}
    splits = {
        "calibration": _split_report(
            "calibration", calibration_nodes,
            [file for file in files if file["case"] in calibration_nodes], ceiling),
        "locked_validation": _split_report(
            "locked_validation", locked_nodes,
            [file for file in files if file["case"] in locked_nodes], ceiling),
    }
    failure_reasons.extend(reason for split in splits.values()
                           for reason in split["failure_reasons"])
    if len(files) != 14:
        failure_reasons.append("input_matrix_not_14_files")
    failure_reasons = sorted(set(failure_reasons))

    candidate_info = {"written": False, "path": str(candidate_path), "sha256": None}
    if mode == "calibrate":
        if not failure_reasons and policy:
            candidate = _candidate(policy, index_path, policy_sha)
            candidate_path.parent.mkdir(parents=True, exist_ok=True)
            candidate_path.write_text(yaml.safe_dump(candidate, allow_unicode=True,
                                                     sort_keys=False), encoding="utf-8")
            candidate_info.update({"written": True, "sha256": object_sha256(candidate)})
        else:
            # 只有校准路线才清理自己产出的候选；已提升 policy 的复验绝不删证据。
            candidate_path.unlink(missing_ok=True)
    else:
        candidate_info["skipped_reason"] = "policy_already_active"

    report = {
        "schema": REPORT_SCHEMA,
        "mode": mode,
        "status": "PASS" if not failure_reasons else "FAIL",
        "policy": {"path": str(policy_path),
                   "id": (policy or {}).get("policy_id"),
                   "version": (policy or {}).get("version"),
                   "lifecycle": (policy or {}).get("lifecycle"),
                   "sha256": policy_sha},
        "input": {"path": str(index_path), "sha256": _sha256(index_path)
                  if index_path.exists() else None,
                  "suite": index.get("suite"), "entries": len(files),
                  "report_only": index.get("report_only"),
                  "unknown_cases": unknown_cases},
        "protocol": {
            "calibration_nodes": calibration_nodes,
            "locked_validation_nodes": locked_nodes,
            "historical_observation_note": calibration.get("note"),
            "acceptance": "unchanged pre-registered class limits and engineering ceiling",
            "threshold_fitting": False,
        },
        "splits": splits,
        "candidate": candidate_info,
        "failure_reasons": failure_reasons,
    }
    write_json(report_path, report)
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--index", type=Path, default=DEFAULT_INDEX)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    args = parser.parse_args(argv)
    report = calibrate(_rooted(args.policy), _rooted(args.index),
                       _rooted(args.report), _rooted(args.candidate))
    cal = report["splits"]["calibration"]
    locked = report["splits"]["locked_validation"]
    print(f"G8 {report['mode']}: {report['status']}")
    print(f"  calibration: {cal['files']} files, {cal['lanes']} lanes, "
          f"{len(cal['engineering_ceiling_violations'])} ceiling violations")
    print(f"  locked-validation: {locked['files']} files, {locked['lanes']} lanes, "
          f"{len(locked['engineering_ceiling_violations'])} ceiling violations")
    if report["candidate"]["written"]:
        print(f"  candidate: {report['candidate']['path']}")
    elif report["mode"] == "verify" and report["status"] == "PASS":
        print(f"  active policy {report['policy']['version']} "
              f"({report['policy']['sha256'][:12]}…) 仍被当前 14 文件满足")
    for reason in report["failure_reasons"]:
        print(f"  FAIL: {reason}")
    print(f"  report: {_rooted(args.report)}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
