# -*- coding: utf-8 -*-
"""目标格式无关的交付裁决，以及 OpenDRIVE G8/G11 收口 sidecar。"""
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


def finalize_opendrive_g8(xodr_path, source_manifest, policy, *, connect_mode="data",
                         g11_policy=None, g11_baseline=None, raw_map_paths=None):
    """G8/G11及真实内外缘接触检查都必须进入质量和交付裁决。"""
    xodr = Path(xodr_path)
    gate = evaluate_g8(xodr, source_manifest, policy)
    from mapforge.validate.g11 import audit_file
    g11_policy=g11_policy or Path(__file__).resolve().parents[2]/'profiles/validation/g11-opendrive-v1.draft.yaml'
    try:
        g11=audit_file(xodr,g11_policy,g11_baseline)
    except Exception as exc:
        g11={'gate_id':'G11','status':'UNAVAILABLE',
             'failure_reasons':[f'{type(exc).__name__}: {exc}']}
    from mapforge.validate.junction_edges import audit as edge_audit
    import xml.etree.ElementTree as ET
    try:
        edge_contacts=edge_audit(ET.parse(xodr).getroot())
        edge_contacts['gate_id']='G11-edge-contacts'
    except Exception as exc:
        edge_contacts={'gate_id':'G11-edge-contacts','status':'UNAVAILABLE',
                       'failure_reasons':[f'{type(exc).__name__}: {exc}']}
    gates={'G8':gate,'G11':g11,'G11-edge-contacts':edge_contacts}
    if (source_manifest or {}).get('source_format') == 'map':
        from mapforge.validate.map_source import audit_map_source_manifest
        gates['G8-source-integrity'] = audit_map_source_manifest(source_manifest, raw_map_paths)
        from mapforge.validate.map_width import audit as width_audit
        try:gates['MAP-explicit-width']=width_audit(ET.parse(xodr).getroot(),source_manifest,raw_map_paths)
        except Exception as exc:
            gates['MAP-explicit-width']={'gate_id':'MAP-explicit-width','status':'UNAVAILABLE',
                                         'failure_reasons':[f'{type(exc).__name__}: {exc}']}
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
    decision = delivery_decision(gates, set(gates), blockers=blockers,
                                 review_reasons=reviews)
    quality = {
        "schema": "mapforge/opendrive-quality-report/v1",
        "artifact": str(xodr),
        "gates": gates,
        "gate_summary": {
            key:sum(g['status']==status for g in gates.values())
            for key,status in [('passed','PASS'),('failed','FAIL'),
                               ('not_run','NOT_RUN'),('unavailable','UNAVAILABLE')]
        },
        "delivery_decision": decision,
    }
    files = {
        "source_manifest": _sidecar(xodr, ".source-lanes.json"),
        "gate_result": _sidecar(xodr, ".g8.json"),
        "g11_result": _sidecar(xodr, ".g11.json"),
        "edge_contacts_result": _sidecar(xodr, ".edge-contacts.json"),
        "quality_report": _sidecar(xodr, ".quality-report.json"),
        "delivery_decision": _sidecar(xodr, ".delivery-decision.json"),
    }
    write_json(files["source_manifest"], source_manifest)
    write_json(files["gate_result"], gate)
    write_json(files["g11_result"], g11)
    write_json(files["edge_contacts_result"], edge_contacts)
    if 'G8-source-integrity' in gates:
        files['source_integrity_result'] = _sidecar(xodr, '.source-integrity.json')
        write_json(files['source_integrity_result'], gates['G8-source-integrity'])
        files['source_width_result'] = _sidecar(xodr, '.source-width.json')
        write_json(files['source_width_result'], gates['MAP-explicit-width'])
    write_json(files["quality_report"], quality)
    write_json(files["delivery_decision"], decision)
    return {"gate": gate, "g11":g11, "edge_contacts":edge_contacts,
            "quality": quality, "decision": decision,
            "files": {k: str(v) for k, v in files.items()}}
