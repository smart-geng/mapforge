"""Registration, source identity and read-only API boundaries, not geometry proof."""
import copy
import hashlib
import json
from pathlib import Path

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench import surface_review as review
from mapforge.workbench.app import create_app
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from test_workbench_sources import source

TOKEN = "surface-review-session-token-long-enough"


def put(path, value):
    path.write_bytes(value if isinstance(value, bytes) else json.dumps(value).encode())


def make_run(directory, snapshot):
    directory.mkdir()
    intent = {"confirmed": True, "source_snapshot_id": snapshot["snapshot_id"],
              "source_content_hash": snapshot["content_hash"], "junction_id": snapshot["junction_id"]}
    tiers = {tier: {"status": "FAIL"} for tier in ("T1", "T2")}
    decision = {"schema": "mapforge/delivery-decision/v1", "status": "BLOCKED"}
    xml = b'<OpenDRIVE><header><geoReference>+proj=eqc +units=m +lon_0=106 +lat_0=29</geoReference></header></OpenDRIVE>'
    docs = {"candidate.xodr": xml, "source-snapshot.json": snapshot,
        "confirmed-probe-intent.json": intent, "surface-evidence.json": {"intent": intent},
        "report.json": {"status": "EXPERIMENT_EVALUATED", "candidate_accepted": False,
            "formal_release_verified": False, "candidate_sha256": review._sha(xml),
            "source_snapshot_id": snapshot["snapshot_id"], "tiers": tiers, "delivery": decision},
        "scoreboard.json": {"schema": "mapforge/scoreboard/v1", "files": 1,
            "rows": [{"case": snapshot["junction_id"], "pipeline": "shp", "artifact": "candidate.xodr", "tiers": tiers}],
            "tier_pass": {"T1": 0, "T2": 0}},
        "candidate.delivery-decision.json": decision,
        "code-policy-binding.json": {"historical": True}}
    for name in review.REQUIRED:
        docs.setdefault(name, b"{}")
    for name, doc in docs.items():
        (directory / name).parent.mkdir(parents=True, exist_ok=True)
        put(directory / name, doc)
    receipt = {"schema": "mapforge/wb11-source-tracks-runner/v1", "status": "EXPERIMENT_COMPLETE",
        "report_status": "EXPERIMENT_EVALUATED", "exit_code": 0, "experiment_complete": True,
        "candidate_accepted": False, "formal_release_verified": False, "problems": [],
        "evidence_bindings": {name: {"sha256": review._sha((directory/name).read_bytes()),
                             "size": (directory/name).stat().st_size} for name in docs}}
    put(directory / "runner-result.json", receipt)
    return directory


def refresh_receipt(directory, name):
    path = directory / "runner-result.json"
    receipt = json.loads(path.read_bytes())
    data = (directory / name).read_bytes()
    receipt["evidence_bindings"][name] = {"sha256": review._sha(data), "size": len(data)}
    put(path, receipt)


@pytest.fixture
def setup(source, tmp_path, monkeypatch):
    catalog = SourceCatalog(*source)
    snapshot = catalog.snapshot("j1")
    run = make_run(tmp_path / "run", snapshot)
    # Deliberately stub only geometric classification; covered by core tests.
    monkeypatch.setattr(review, "_diagnose", lambda root, evidence: {
        "schema": "mapforge/surface-diagnostics/v1", "status": "DIAGNOSED", "issues": [], "layers": [],
        "source_proof": {"snapshot_id": snapshot["snapshot_id"], "content_hash": snapshot["content_hash"]}})
    return catalog, snapshot, run


def project(snapshot):
    return {"source_snapshot": snapshot, "status": {}, "candidate": None, "intents": []}


def test_read_only_source_bound_review_preserves_project_and_original_failure(setup):
    catalog, snapshot, run = setup
    service = review.SurfaceReviewService(catalog, [run])
    state = project(snapshot); before = copy.deepcopy(state)
    result = service.describe(state)
    assert result["available"] and result["report"]["status"] == "DIAGNOSED"
    context = result["report"]["context"]
    assert context["candidate_accepted"] is False and context["formal_release_verified"] is False
    assert context["historical_delivery_decision"] == "BLOCKED"
    assert context["historical_score_tiers"][0]["T1"]["status"] == "FAIL"
    assert context["frame"]["origin"] == [106, 29]
    assert context["frame"]["absolute_crs_status"] == "unverified"
    assert state == before


def test_other_junction_or_changed_content_never_receives_old_layers(setup):
    catalog, snapshot, run = setup
    service = review.SurfaceReviewService(catalog, [run])
    other = copy.deepcopy(snapshot); other["junction_id"] = "another"
    other["content_hash"] = review._snapshot_hash(other)
    assert not service.describe(project(other))["available"]
    other = copy.deepcopy(snapshot); other["objects"][0]["business_id"] = "forged"
    assert not service.describe(project(other))["available"]


@pytest.mark.parametrize("name", ["candidate.xodr", "surface-evidence.json", "scoreboard.json", "runner-result.json"])
def test_any_registered_byte_drift_disables_review(setup, name):
    catalog, snapshot, run = setup
    service = review.SurfaceReviewService(catalog, [run])
    with (run/name).open("ab") as stream:
        stream.write(b" ")
    assert not service.describe(project(snapshot))["available"]


def test_source_drift_and_read_only_state_disable_review(setup):
    catalog, snapshot, run = setup
    service = review.SurfaceReviewService(catalog, [run])
    state = project(snapshot); state["status"]["read_only"] = True
    assert not service.describe(state)["available"]
    source_file = catalog.source_dir / snapshot["source_files"][0]["relative_path"]
    with source_file.open("ab") as stream:
        stream.write(b"drift")
    assert not service.describe(project(snapshot))["available"]


def test_drift_during_computation_discards_result(setup, monkeypatch):
    catalog, snapshot, run = setup
    service = review.SurfaceReviewService(catalog, [run])
    def changed(root, evidence):
        with (run/"candidate.xodr").open("ab") as stream:
            stream.write(b" ")
        return {"status": "DIAGNOSED"}
    monkeypatch.setattr(review, "_diagnose", changed)
    assert not service.describe(project(snapshot))["available"]


@pytest.mark.parametrize("name,value", [("candidate_accepted", True), ("formal_release_verified", True),
    ("experiment_complete", False), ("status", "INCOMPLETE_EVIDENCE"), ("problems", ["missing"]), ("exit_code", 2), ("exit_code", False)])
def test_incomplete_or_mislabelled_receipt_not_registered(setup, name, value):
    _, _, run = setup
    path=run/"runner-result.json"; receipt=json.loads(path.read_bytes()); receipt[name]=value; put(path,receipt)
    with pytest.raises(ValueError):
        review.RegisteredSurfaceReview(run)


@pytest.mark.parametrize("name", ["../outside.json", "/absolute.json", "folder\\file.json", "C:/data.json"])
def test_registration_cannot_read_outside_its_directory(setup, name):
    _, _, run=setup
    path=run/"runner-result.json"; receipt=json.loads(path.read_bytes())
    receipt["evidence_bindings"][name] = {"size": 0, "sha256": hashlib.sha256(b"").hexdigest()};put(path,receipt)
    with pytest.raises(ValueError):
        review.RegisteredSurfaceReview(run)


def test_duplicate_source_registration_rejected(setup):
    catalog, _, run = setup
    with pytest.raises(ValueError, match="多个"):
        review.SurfaceReviewService(catalog, [run, run])


def test_mismatched_source_intent_rejected_even_if_record_hash_rewritten(setup):
    _, _, run = setup
    path=run/"confirmed-probe-intent.json"; intent=json.loads(path.read_bytes());intent["junction_id"]="other";put(path,intent)
    refresh_receipt(run,path.name)
    with pytest.raises(ValueError, match="绑定"):
        review.RegisteredSurfaceReview(run)


@pytest.mark.parametrize("xml", [b'<!DOCTYPE OpenDRIVE [<!ENTITY test SYSTEM "file:///private">]><OpenDRIVE>&test;</OpenDRIVE>',
    b'<OpenDRIVE><header><geoReference>+proj=longlat +units=degrees</geoReference></header></OpenDRIVE>'])
def test_unsupported_frame_or_dtd_cannot_be_drawn(setup, xml):
    catalog, snapshot, run=setup
    put(run/"candidate.xodr",xml);refresh_receipt(run,"candidate.xodr")
    p=run/"report.json"; d=json.loads(p.read_bytes());d["candidate_sha256"]=review._sha(xml);put(p,d);refresh_receipt(run,p.name)
    assert not review.SurfaceReviewService(catalog,[run]).describe(project(snapshot))["available"]


def test_unavailable_core_result_is_not_an_empty_success(setup, monkeypatch):
    catalog,snapshot,run=setup
    monkeypatch.setattr(review,"_diagnose",lambda root,evidence:{"status":"UNAVAILABLE","reason":"unsupported"})
    assert review.SurfaceReviewService(catalog,[run]).describe(project(snapshot)) == {"available":False,"reason":"unsupported"}


@pytest.mark.parametrize("name", sorted(review.REQUIRED))
def test_every_runner_required_artifact_must_be_present_in_binding_and_disk(setup, name):
    _,_,run=setup
    path=run/"runner-result.json"; receipt=json.loads(path.read_bytes())
    del receipt["evidence_bindings"][name]; (run/name).unlink(); put(path,receipt)
    with pytest.raises(ValueError, match="不完整"):
        review.RegisteredSurfaceReview(run)


def test_core_proof_must_match_registered_source(setup, monkeypatch):
    catalog,snapshot,run=setup
    monkeypatch.setattr(review,"_diagnose",lambda root,evidence:{"status":"DIAGNOSED",
        "source_proof":{"snapshot_id":"other","content_hash":snapshot["content_hash"]}})
    result=review.SurfaceReviewService(catalog,[run]).describe(project(snapshot))
    assert not result["available"] and "不属于" in result["reason"]


@pytest.mark.parametrize("field,value", [("case","another-junction"), ("artifact","other.xodr"),
    ("pipeline","map"), ("tiers", {"T1":{"status":"PASS"},"T2":{"status":"PASS"}})])
def test_spliced_score_cannot_be_displayed_as_this_research_candidate(setup, field, value):
    _,_,run=setup;path=run/"scoreboard.json";board=json.loads(path.read_bytes())
    board["rows"][0][field]=value;put(path,board);refresh_receipt(run,path.name)
    with pytest.raises(ValueError, match="历史评分"):
        review.RegisteredSurfaceReview(run)


def test_spliced_delivery_cannot_replace_original_decision(setup):
    _,_,run=setup;path=run/"candidate.delivery-decision.json";decision=json.loads(path.read_bytes())
    decision["status"]="DELIVERABLE";put(path,decision);refresh_receipt(run,path.name)
    with pytest.raises(ValueError, match="历史交付"):
        review.RegisteredSurfaceReview(run)


def test_http_is_authenticated_read_only_and_does_not_register_client_paths(setup, tmp_path):
    catalog,snapshot,run=setup
    store=ProjectStore(tmp_path/"projects"); state=store.create(snapshot,"诊断源工程")
    directory=store.root/state["project_id"]; before={p.name:p.read_bytes() for p in directory.iterdir() if p.is_file()}
    app=create_app(store,catalog,TOKEN,18978,surface_reviews=[run])
    with TestClient(app,base_url="http://127.0.0.1:18978") as client:
        endpoint=f"/api/projects/{state['project_id']}/surface-diagnostics"
        assert client.get(endpoint).status_code==403
        headers={"Authorization":"Bearer "+TOKEN}
        assert client.get(endpoint,headers=headers).json()["available"]
        assert client.post(endpoint,json={"path":"C:/private"},headers={**headers,"X-Mapforge-CSRF":TOKEN}).status_code==404
        assert client.get("/assets/diagnostics.js").status_code==200
    assert {p.name:p.read_bytes() for p in directory.iterdir() if p.is_file()}==before
    assert store.load(state["project_id"])["candidate"] is None
