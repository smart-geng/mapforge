"""Scoreboard tier logic, draft-policy guardrails and tolerance-aware replay comparison."""
from pathlib import Path

import pytest
import yaml

from mapforge.validate.replay import equivalent
from mapforge.validate.scoreboard import POLICY, apply_tiers

ROOT = Path(__file__).resolve().parents[1]


def test_equivalent_tolerates_float_ulp_but_not_real_changes():
    stored = {"a": [1.7671845769635228, 2], "b": "x", "ok": True, "content_sha256": "old"}
    assert equivalent({"a": [1.7671845769635226, 2], "b": "x", "ok": True, "content_sha256": "new"}, stored)
    assert equivalent({"a": (1.7671845769635226, 2), "b": "x", "ok": True}, stored)  # tuple vs JSON list
    assert not equivalent({"a": [1.7671845769635226 + 1e-6, 2], "b": "x", "ok": True}, stored)
    assert not equivalent({"a": [1.7671845769635228, 3], "b": "x", "ok": True}, stored)
    assert not equivalent({"a": [1.7671845769635228, 2], "b": "y", "ok": True}, stored)
    assert not equivalent({"a": [1.7671845769635228, 2], "b": "x", "ok": 1}, stored)
    assert not equivalent({"a": [1.7671845769635228, 2], "b": "x"}, stored)


def _policy():
    return {"tiers": {
        "T1": {"checks": [{"metric": "xsd_valid", "op": "eq", "value": True},
                          {"metric": "gap_m", "op": "le", "value": 0.01}]},
        "T2": {"requires": "T1", "checks": [{"metric": "boundary_p95_m", "op": "le", "value": 0.1,
                                             "applies_to": ["shp"]},
                                            {"metric": "seg_min_m", "op": "ge", "value": 5.0}]},
    }}


def test_tiers_report_values_and_never_pass_missing_metrics():
    t = apply_tiers({"xsd_valid": True, "gap_m": 0.0, "boundary_p95_m": 0.05, "seg_min_m": 6.0}, "shp", _policy())
    assert t["T1"]["status"] == "PASS" and t["T2"]["status"] == "PASS"
    t = apply_tiers({"xsd_valid": True, "gap_m": 0.02, "boundary_p95_m": 0.05, "seg_min_m": 6.0}, "shp", _policy())
    assert t["T1"]["status"] == "FAIL" and t["T1"]["failed"][0]["actual"] == 0.02
    assert t["T2"]["status"] == "BLOCKED_BY_T1"
    t = apply_tiers({"xsd_valid": True, "gap_m": 0.0, "seg_min_m": 6.0}, "shp", _policy())
    assert t["T2"]["status"] == "UNAVAILABLE" and t["T2"]["unavailable"] == ["boundary_p95_m"]
    # SHP-only check does not apply to the MAP pipeline
    t = apply_tiers({"xsd_valid": True, "gap_m": 0.0, "seg_min_m": 6.0}, "map", _policy())
    assert t["T2"]["status"] == "PASS"


def test_static_policy_stays_a_draft_without_dynamics():
    policy = yaml.safe_load(POLICY.read_text(encoding="utf-8"))
    assert policy["lifecycle"] == "draft" and policy["version"].endswith("-draft")
    metrics = {c["metric"] for tier in policy["tiers"].values() for c in tier["checks"]}
    # 2026-09-15 decision: policy-speed dynamics are a scenario group, never a static tier check.
    assert not {m for m in metrics if m.startswith("policy_speed_")}
    assert policy["tiers"]["T2"]["requires"] == "T1"
