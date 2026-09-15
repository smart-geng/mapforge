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
    assert quality['gates']['G11']==result['g11']
    assert result['decision']['required_gates']==['G11','G11-edge-contacts','G8']
    assert quality['gates']['G11-edge-contacts']==result['edge_contacts']


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


def test_g11_failure_blocks_delivery_even_if_g8_and_crs_pass(build_g8_case,g8_policy,monkeypatch):
    from mapforge.validate import g11
    out,manifest=build_g8_case(); manifest=copy.deepcopy(manifest)
    manifest['comparison_crs']['integrity']='verified'
    monkeypatch.setattr(g11,'audit_file',lambda *_args,**_kwargs:{'gate_id':'G11','status':'FAIL'})
    result=finalize_opendrive_g8(out,manifest,g8_policy)
    assert result['gate']['status']=='PASS'
    assert result['decision']['status']=='BLOCKED'
    assert any(x.get('gate_id')=='G11' for x in result['decision']['blocked_reasons'])


def test_g11_exception_is_unavailable_not_deliverable(build_g8_case,g8_policy,monkeypatch):
    from mapforge.validate import g11
    out,manifest=build_g8_case(); manifest=copy.deepcopy(manifest)
    manifest['comparison_crs']['integrity']='verified'
    def fail(*_args,**_kwargs): raise ValueError('invalid geometry')
    monkeypatch.setattr(g11,'audit_file',fail)
    result=finalize_opendrive_g8(out,manifest,g8_policy)
    assert result['g11']['status']=='UNAVAILABLE'
    assert result['decision']['status']=='BLOCKED'


def test_edge_contact_failure_blocks_even_if_centers_and_crs_pass(build_g8_case,g8_policy,monkeypatch):
    from mapforge.validate import junction_edges
    out,manifest=build_g8_case(); manifest=copy.deepcopy(manifest)
    manifest['comparison_crs']['integrity']='verified'
    monkeypatch.setattr(junction_edges,'audit',lambda *_:{'status':'FAIL','count':4,'failed_count':2})
    result=finalize_opendrive_g8(out,manifest,g8_policy)
    assert result['gate']['status']=='PASS' and result['g11']['status']=='PASS'
    assert result['decision']['status']=='BLOCKED'
    assert any(x.get('gate_id')=='G11-edge-contacts' for x in result['decision']['blocked_reasons'])
