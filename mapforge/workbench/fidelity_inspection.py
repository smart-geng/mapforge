"""Read-only locations for an artifact-bound G8 report, never new gate results.

The caller authenticates the source, candidate and reports. This module checks
their identities/frame again and extracts only the actual G8 comparison domains.
It does not infer stoplines, change comparison windows or assign per-lane tiers.
"""
from __future__ import annotations

import copy
import math
from numbers import Real
from pathlib import Path

from lxml import etree

from mapforge.validate import g8_model as model
from mapforge.validate.lane_fidelity import evaluate_g8, extract_target_components
from .fidelity_replay import matches_fidelity_replay

SCHEMA = "mapforge/workbench-fidelity-inspection/v1"
POLICY = Path(__file__).resolve().parents[2] / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"
TARGET_FIELDS = ("road_id", "side", "lane_id", "section_first", "section_last")


def _line(points):
    result = []
    for point in points:
        if len(point) != 2 or any(not isinstance(v, Real) or isinstance(v, bool)
                                  or not math.isfinite(float(v)) for v in point):
            raise ValueError("来源或候选定位坐标无效")
        result.append([float(v) for v in point])
    if len(result) < 2 or len({tuple(p) for p in result}) < 2:
        raise ValueError("来源或候选定位缺少有效比较区间")
    return {"type": "LineString", "coordinates": result}


def _stats(value):
    if (not isinstance(value, dict) or type(value.get("count")) is not int or value["count"] < 1
            or any(type(value.get(k)) not in (int, float) or not math.isfinite(value[k]) or value[k] < 0
                   for k in ("p95_m", "max_m", "median_m"))):
        raise ValueError("保真统计缺失或无效，不能当作零误差")
    return copy.deepcopy(value)


def inspect_fidelity(root, manifest, gate, policy):
    """Return complete comparison lines ranked by existing P95, not bad segments.

    UNAVAILABLE removes every location; partial/ambiguous matches are never
    silently omitted. G8's own UNAVAILABLE still permits valid diagnostic lines.
    """
    unavailable = {"schema": SCHEMA, "status": "UNAVAILABLE", "reason": "来源保真定位未完成",
                   "gate_status": None, "scope": None, "summary": None, "lanes": [],
                   "whole_map_score_modified": False, "per_lane_tiers_assigned": False}
    try:
        if model.validate_manifest(manifest):
            raise ValueError("来源车道清单不完整或身份不唯一")
        expected = model.object_sha256({k: v for k, v in manifest.items() if k != "manifest_sha256"})
        if manifest.get("manifest_sha256") != expected:
            raise ValueError("来源清单内容哈希不一致")
        crs = manifest["comparison_crs"]
        refs = root.findall("header/geoReference")
        if (root.tag != "OpenDRIVE" or len(refs) != 1 or root.find("header/offset") is not None
                or crs.get("id") != "local-eqc" or crs.get("axis_order") != ["x", "y"]
                or not isinstance(crs.get("proj_string"), str)
                or crs["proj_string"].split() != (refs[0].text or "").split()):
            raise ValueError("来源与实际候选的局部米制坐标框架不一致")
        policy = model.load_policy(policy)
        if model.validate_policy(policy):
            raise ValueError("保真策略缺失或不受支持")
        if (gate.get("schema") != "mapforge/gate-result/v1" or gate.get("gate_id") != "G8"
                or gate.get("status") not in {"PASS", "FAIL", "REVIEW", "UNAVAILABLE"}
                or gate.get("policy", {}).get("sha256") != policy["policy_sha256"]):
            raise ValueError("原保真报告或策略身份不匹配")
        # Replay the unchanged evaluator solely to authenticate the report we
        # display. Do not reimplement its stopline/coverage/exclusion semantics
        # here: a missing issue or forged zero must never disappear in the UI.
        if not matches_fidelity_replay(gate, evaluate_g8(root, manifest, policy)):
            raise ValueError("原保真报告与实际候选、来源及策略的回读结果不一致")
        rows, scope, issues = gate.get("per_lane"), gate.get("scope"), gate.get("issues")
        if (not isinstance(rows, list) or not rows or not isinstance(scope, dict)
                or not isinstance(issues, dict) or scope.get("matched_source_lanes") != len(rows)):
            raise ValueError("原保真报告的逐车道结果或范围不完整")
        if any(issues.get(k) for k in ("missing_source_ids", "orphan_target_ids", "one_source_many_targets",
                                      "unprovenanced_targets", "duplicate_target_occurrences", "metadata_errors")):
            raise ValueError("来源或候选关联不完整，暂不能可靠定位全部比较对象")
        sampling = policy["sampling"]
        actual = extract_target_components(root, ds=float(sampling["step_m"]),
            zero_width_epsilon_m=float(sampling["zero_width_epsilon_m"]))
        if any(actual[k] for k in ("unprovenanced", "duplicate_occurrences", "metadata_errors")):
            raise ValueError("实际候选存在不明确的车道身份")
        sources = {lane["source_lane_id"]: lane for lane in manifest["lanes"]
                   if lane.get("comparison", {}).get("eligible") is True}
        by_id = {}
        for component in actual["components"]:
            by_id.setdefault(component["source_lane_id"], []).append(component)
        ids = [row["source_lane_id"] for row in rows]
        if (len(set(ids)) != len(ids) or set(ids) != set(sources) or set(ids) != set(by_id)
                or scope.get("eligible_source_lanes") != len(sources)
                or scope.get("target_components") != len(actual["components"])):
            raise ValueError("实际源、目标与报告没有完整一一对应")
        missing = issues.get("unmeasurable_source_ids")
        if not isinstance(missing, list):
            raise ValueError("原保真报告缺少不可测对象清单")
        lanes = []
        for row in rows:
            sid = row["source_lane_id"]
            source, components = sources[sid], by_id[sid]
            if len(components) != 1:
                raise ValueError("同一来源对应多个目标，不能自行选择定位对象")
            component = components[0]
            target = {k: component[k] for k in TARGET_FIELDS}
            if (row.get("target") != target or row.get("policy_class") != source.get("policy_class")
                    or component.get("policy_class") != source.get("policy_class")):
                raise ValueError("实际候选与报告的道路、车道或比较区间不匹配")
            source_geometry = source["geometry"]
            if (source_geometry.get("type") != "LineString"
                    or source_geometry.get("geometry_sha256") != model.geometry_sha256(source_geometry["coordinates"])):
                raise ValueError("来源比较线身份不匹配")
            stop_line = copy.deepcopy(source.get("stop_line"))
            if not isinstance(stop_line, dict) or not isinstance(stop_line.get("availability"), str):
                raise ValueError("来源停止线状态缺失")
            lane_issues = []
            if sid + ":stopline" in missing:
                lane_issues.append({"code": "stopline-unmeasurable", "reason": stop_line.get("reason"),
                    "message": "来源停止线不可测；需核对原始对象及显式关联，不能按邻近位置补绑。"})
            lanes.append({"id": "fidelity:" + sid, "source_lane_id": sid, "target": target,
                "policy_class": row["policy_class"], "source_to_target": _stats(row.get("source_to_target")),
                "target_to_source": _stats(row.get("target_to_source")), "stop_line": stop_line,
                "source_geometry": _line(source_geometry["coordinates"]),
                "target_geometry": _line(component["points"]), "issues": lane_issues})
        lanes.sort(key=lambda item: (-item["source_to_target"]["p95_m"], item["source_lane_id"]))
        return {**unavailable, "status": "AVAILABLE", "reason": "只读定位，沿用原保真报告数值及比较范围",
            "gate_status": gate["status"], "scope": copy.deepcopy(scope),
            "summary": {"lane_count": len(lanes),
                        "stopline_unavailable_count": sum(bool(row["issues"]) for row in lanes)},
            "lanes": lanes, "issues": copy.deepcopy(issues),
            "geometry_scope": "完整比较区间，并非精确超限区间；不含原 G8 排除域",
            "metrics_basis": "原 G8 采样统计；单车道数值不替代整图评分"}
    except (OSError, ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError) as exc:
        return {**unavailable, "reason": "保真定位不可用：" + str(exc)}


def inspect_bound_review(review):
    """Read only the caller's authenticated run; never load HTTP-supplied paths."""
    from .surface_review import _json

    data = review.read_bound()
    parser = etree.XMLParser(resolve_entities=False, no_network=True, load_dtd=False)
    root = etree.fromstring(data["candidate.xodr"], parser)
    if root.getroottree().docinfo.doctype:
        raise ValueError("定位 XML 不允许文档类型声明")
    result = inspect_fidelity(root, _json(data["candidate.source-lanes.json"]),
                              _json(data["candidate.g8.json"]), POLICY)
    if result["status"] == "AVAILABLE":
        snapshot = _json(data["source-snapshot.json"])
        for lane in result["lanes"]:
            refs = [{k: copy.deepcopy(obj[k]) for k in ("id", "role", "source_ref")}
                    for obj in snapshot["objects"] if obj.get("business_id") == lane["source_lane_id"]
                    and obj.get("role") in {"lane", "lane_merge"}]
            primary = [ref for ref in refs if ref["role"] == "lane"
                       and ref["source_ref"].get("layer") == "IBD_LANE_LINK"]
            if len(primary) != 1:
                result = {**result, "status": "UNAVAILABLE", "lanes": [], "summary": None,
                          "reason": "来源主车道记录不唯一或缺失，不能可靠定位原件"}
                break
            lane["source_refs"] = sorted(refs, key=lambda item: (item["role"] != "lane", item["id"]))
            lane["primary_source_ref"] = copy.deepcopy(primary[0])
    review.read_bound()
    return result
