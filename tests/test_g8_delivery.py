import copy
import json
from pathlib import Path

from mapforge.report.decision import delivery_decision, finalize_opendrive_g8


def _gate(status):
    return {"gate_id": "G8", "status": status}


def test_required_g8_accepts_only_pass():
    assert delivery_decision({"G8": _gate("PASS")}, {"G8"})["status"] == "DELIVERABLE"
    for status in ("FAIL", "NOT_RUN", "UNAVAILABLE"):
        result = delivery_decision({"G8": _gate(status)}, {"G8"})
        assert result["status"] == "BLOCKED"
        assert result["blocked_reasons"][0]["gate_status"] == status
    missing = delivery_decision({}, {"G8"})
    assert missing["status"] == "BLOCKED"
    assert missing["blocked_reasons"][0]["gate_status"] == "NOT_RUN"


def test_blocker_wins_over_review_reason():
    result = delivery_decision({"G8": _gate("PASS")}, {"G8"},
                               blockers=[{"code": "crs"}],
                               review_reasons=[{"code": "full"}])
    assert result["status"] == "BLOCKED"
    assert result["review_reasons"] == [{"code": "full"}]


def test_finalize_writes_one_gate_result_to_all_sidecars(build_g8_case, g8_policy):
    out, manifest = build_g8_case()
    manifest = copy.deepcopy(manifest)
    manifest["comparison_crs"]["integrity"] = "verified"
    result = finalize_opendrive_g8(out, manifest, g8_policy)
    assert result["gate"]["status"] == "PASS"
    assert result["decision"]["status"] == "DELIVERABLE"
    for path in result["files"].values():
        assert Path(path).exists()
        json.loads(Path(path).read_text(encoding="utf-8"))
    quality = json.loads(Path(result["files"]["quality_report"]).read_text(encoding="utf-8"))
    gate = json.loads(Path(result["files"]["gate_result"]).read_text(encoding="utf-8"))
    assert quality["gates"]["G8"] == gate


def test_finalize_keeps_crs_blocked_and_full_review(build_g8_case, g8_policy):
    out, manifest = build_g8_case()
    manifest = copy.deepcopy(manifest)
    manifest["comparison_crs"]["integrity"] = "internally-consistent"
    result = finalize_opendrive_g8(out, manifest, g8_policy, connect_mode="full")
    assert result["gate"]["status"] == "PASS"
    assert result["decision"]["status"] == "BLOCKED"
    assert any(x["code"] == "crs_not_absolutely_verified"
               for x in result["decision"]["blocked_reasons"])
    assert any(x["code"] == "connect_mode_full"
               for x in result["decision"]["review_reasons"])
