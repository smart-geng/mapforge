# -*- coding: utf-8 -*-
"""目标格式无关的交付裁决，以及 OpenDRIVE G8 收口 sidecar。"""
from __future__ import annotations

from pathlib import Path

from mapforge.validate.g8_model import write_json
from mapforge.validate.lane_fidelity import evaluate_g8

DECISION_SCHEMA = "mapforge/delivery-decision/v1"


def delivery_decision(gate_results, required_gates, *, blockers=None, review_reasons=None):
    """required gate 只有 PASS 才允许交付；其他红线与人工复核原因独立保留。"""
    if isinstance(gate_results, dict):
        gates = {str(k): v for k, v in gate_results.items()}
    else:
        gates = {str(x.get("gate_id")): x for x in (gate_results or [])}
    blocked = list(blockers or [])
    reviews = list(review_reasons or [])
    for gate_id in sorted(set(required_gates)):
        gate = gates.get(gate_id)
        status = gate.get("status") if gate else "NOT_RUN"
        if status != "PASS":
            blocked.append({"code": "required_gate_not_pass",
                            "gate_id": gate_id, "gate_status": status})
    status = "BLOCKED" if blocked else "REVIEW_REQUIRED" if reviews else "DELIVERABLE"
    return {
        "schema": DECISION_SCHEMA,
        "status": status,
        "required_gates": sorted(set(required_gates)),
        "blocked_reasons": blocked,
        "review_reasons": reviews,
    }


def _sidecar(path: Path, suffix: str) -> Path:
    return path.with_suffix(suffix)


def finalize_opendrive_g8(xodr_path, source_manifest, policy, *, connect_mode="data"):
    """评估一次 G8，并把同一结果写入质量报告和交付裁决。"""
    xodr = Path(xodr_path)
    gate = evaluate_g8(xodr, source_manifest, policy)
    exclusions = sorted({x.get("code") for x in gate.get("exclusions", []) if x.get("code")})
    policy_obj = policy if isinstance(policy, dict) else None
    if policy_obj is None:
        from mapforge.validate.g8_model import load_policy
        policy_obj = load_policy(policy) or {}
    review_codes = set(policy_obj.get("review_exclusions") or [])
    reviews = [{"code": "g8_exclusion_requires_review", "exclusion_code": code}
               for code in exclusions if code in review_codes]
    if connect_mode == "full":
        reviews.append({"code": "connect_mode_full", "message": "全连接仅供仿真/审查"})
    blockers = []
    crs = (source_manifest or {}).get("comparison_crs") or {}
    if crs.get("integrity") != "verified":
        blockers.append({"code": "crs_not_absolutely_verified",
                         "integrity": crs.get("integrity", "missing")})
    decision = delivery_decision({"G8": gate}, {"G8"}, blockers=blockers,
                                 review_reasons=reviews)
    quality = {
        "schema": "mapforge/opendrive-quality-report/v1",
        "artifact": str(xodr),
        "gates": {"G8": gate},
        "gate_summary": {
            "passed": 1 if gate["status"] == "PASS" else 0,
            "failed": 1 if gate["status"] == "FAIL" else 0,
            "not_run": 1 if gate["status"] == "NOT_RUN" else 0,
            "unavailable": 1 if gate["status"] == "UNAVAILABLE" else 0,
        },
        "delivery_decision": decision,
    }
    files = {
        "source_manifest": _sidecar(xodr, ".source-lanes.json"),
        "gate_result": _sidecar(xodr, ".g8.json"),
        "quality_report": _sidecar(xodr, ".quality-report.json"),
        "delivery_decision": _sidecar(xodr, ".delivery-decision.json"),
    }
    write_json(files["source_manifest"], source_manifest)
    write_json(files["gate_result"], gate)
    write_json(files["quality_report"], quality)
    write_json(files["delivery_decision"], decision)
    return {"gate": gate, "quality": quality, "decision": decision,
            "files": {k: str(v) for k, v in files.items()}}
