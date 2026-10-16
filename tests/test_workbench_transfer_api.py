"""Project-package HTTP contract between two real workbench sessions.

Uses the tiny real SHP/Store/job-history fixture of the relocation tests. These
tests prove transport, permission and publication behavior, never map quality.
"""
import hashlib
import time
import uuid

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench import project_transfer
from mapforge.workbench.app import create_app
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from test_workbench_project_relocation import _never_bound, case

TOKEN = "transfer-api-test-session-token-with-32-characters"
PORT = 19881
BASE_URL = f"http://127.0.0.1:{PORT}"
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}
RAW = {"Content-Type": "application/octet-stream"}


def _client(store, raw, profile):
    app = create_app(store, SourceCatalog(raw, profile), TOKEN, PORT)
    return TestClient(app, base_url=BASE_URL, headers=HEADERS)


@pytest.fixture
def pair(case):
    with _client(case.store, case.old_raw, case.old_profile) as source, \
            _client(ProjectStore(case.destination), case.raw, case.profile) as target:
        yield source, target, case


def _wait(client, job, timeout=15):
    deadline = time.monotonic() + timeout
    while job["state"] == "running" and time.monotonic() < deadline:
        time.sleep(.01)
        response = client.get(f"/api/transfers/jobs/{job['job_id']}")
        assert response.status_code == 200, response.text
        job = response.json()
    assert job["state"] != "running", job
    return job


def _export(client, project):
    response = client.post("/api/transfers/exports", json={
        "project_id": project["project_id"], "base_revision": project["revision"], "request_id": uuid.uuid4().hex})
    assert response.status_code == 200, response.text
    job = _wait(client, response.json())
    assert job["state"] == "succeeded", job
    return job


def _package(client, project):
    job = _export(client, project)
    response = client.get(f"/api/transfers/jobs/{job['job_id']}/package")
    assert response.status_code == 200, response.text
    return job, response


def _upload(client, data, *, sha256=None, chunk=16384):
    response = client.post("/api/transfers/uploads", json={
        "name": "迁移 工程.mapforge-project.zip", "size": len(data),
        "sha256": sha256 or hashlib.sha256(data).hexdigest()})
    assert response.status_code == 200, response.text
    upload = response.json()
    for offset in range(0, len(data), chunk):
        response = client.put(f"/api/transfers/uploads/{upload['upload_id']}/chunks", params={"offset": offset},
                              content=data[offset:offset + chunk], headers=RAW)
        assert response.status_code == 200, response.text
        upload = response.json()
    return upload


def _import(client, upload):
    response = client.post("/api/transfers/imports", json={"upload_id": upload["upload_id"],
                                                           "request_id": uuid.uuid4().hex})
    assert response.status_code == 200, response.text
    return _wait(client, response.json())


def test_page_round_trip_keeps_drafts_marks_old_results_stale_and_never_opens_automatically(pair):
    source, target, c = pair
    before = (c.directory / "project.json").read_bytes()
    assert not (c.store.root / ".transfers").exists()
    described = source.get("/api/transfers").json()
    assert described["source_included"] is False and described["formal_delivery"] is False
    assert described["recompute_required"] is True
    assert described["chunk_bytes"] == project_transfer.MAX_CHUNK_BYTES
    job, response = _package(source, c.project)
    data = response.content
    assert job["result"]["source_included"] is False and job["result"]["formal_delivery"] is False
    assert response.headers["Content-Type"] == "application/zip"
    assert response.headers["X-Content-SHA256"] == job["result"]["sha256"] == hashlib.sha256(data).hexdigest()
    assert int(response.headers["Content-Length"]) == job["result"]["size"] == len(data)
    assert response.headers["X-Mapforge-Purpose"] == "SAVED_PROJECT_REQUIRES_IDENTICAL_SOURCE"
    assert response.headers["Content-Disposition"] == f'attachment; filename="{job["result"]["filename"]}"'
    assert response.headers["Cache-Control"] == "no-store"
    assert (c.directory / "project.json").read_bytes() == before

    assert target.get("/api/projects").json() == []
    upload = _upload(target, data)
    assert upload["state"] == "sealed" and upload["received_size"] == len(data)
    imported = _import(target, upload)
    assert imported["state"] == "succeeded", imported
    result = imported["result"]
    assert result["project_id"] == c.project["project_id"]
    assert result["candidate_stale"] is True and result["validation_stale"] is True
    assert result["recompute_required"] is True and result["formal_delivery"] is False
    # Published only after success; the page lists it, but opening stays explicit.
    assert [p["project_id"] for p in target.get("/api/projects").json()] == [c.project["project_id"]]
    opened = target.get(f"/api/projects/{c.project['project_id']}")
    assert opened.status_code == 200, opened.text
    project = opened.json()
    assert project["intents"] == c.project["intents"]
    assert project["status"]["can_redo"] is True
    assert project["status"]["candidate_stale"] is True and project["status"]["validation_stale"] is True
    assert project["status"]["formal_export_available"] is False


def test_to_do_only_project_exports_and_imports_through_the_page_api(tmp_path):
    c = _never_bound(tmp_path)
    with _client(c.store, c.raw, c.profile) as source, \
            _client(ProjectStore(tmp_path / "目标 工程库"), c.raw, c.profile) as target:
        _, response = _package(source, c.project)
        imported = _import(target, _upload(target, response.content))
        assert imported["state"] == "succeeded", imported
        assert imported["result"]["candidate_stale"] is False
        project = target.get(f"/api/projects/{c.project['project_id']}").json()
        assert project["intents"] == c.project["intents"]
        assert project["candidate"] is None


def test_same_id_import_is_rejected_with_explanation_and_does_not_overwrite(pair):
    source, target, c = pair
    _, response = _package(source, c.project)
    assert _import(target, _upload(target, response.content))["state"] == "succeeded"
    published = c.destination / c.project["project_id"] / "project.json"
    before = published.read_bytes()
    second = _import(target, _upload(target, response.content))
    assert second["state"] == "failed"
    assert second["error"]["code"] == "project-already-exists"
    assert "不重命名或覆盖" in second["error"]["message"]
    assert published.read_bytes() == before
    assert len(target.get("/api/projects").json()) == 1


def test_download_after_a_later_saved_edit_is_refused(pair):
    source, _, c = pair
    job = _export(source, c.project)
    oid = c.snapshot["objects"][0]["id"]
    saved = source.post(f"/api/projects/{c.project['project_id']}/commands", json={
        "base_revision": c.project["revision"], "command": {
            "command_id": "after-export", "type": "annotation", "source_ref": oid,
            "scope": {"feature_ids": [oid]}, "parameters": {"text": "导出后的新待办", "status": "unresolved"}}})
    assert saved.status_code == 200, saved.text
    response = source.get(f"/api/transfers/jobs/{job['job_id']}/package")
    assert response.status_code == 409
    assert response.json()["code"] == "project-revision-conflict"
    assert "请刷新后导出" in response.json()["detail"]


def test_upload_failures_are_queryable_and_cannot_be_imported(pair):
    source, target, c = pair
    _, response = _package(source, c.project)
    data = response.content
    upload = _upload(target, data, sha256="0" * 64)
    assert upload["state"] == "failed" and upload["error"]["code"] == "upload-hash-mismatch"
    assert target.get(f"/api/transfers/uploads/{upload['upload_id']}").json()["state"] == "failed"
    failed = _import(target, upload)
    assert failed["state"] == "failed" and failed["error"]["code"] == "upload-not-complete"
    assert target.get("/api/projects").json() == []
    partial = target.post("/api/transfers/uploads", json={
        "name": "a.mapforge-project.zip", "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}).json()
    skipped = target.put(f"/api/transfers/uploads/{partial['upload_id']}/chunks", params={"offset": 1},
                         content=data[1:10], headers=RAW)
    assert skipped.status_code == 422 and skipped.json()["code"] == "invalid-upload-chunk"


@pytest.mark.parametrize("path, body, code", [
    ("/api/transfers/exports", {"project_id": "0" * 32, "base_revision": 1, "request_id": "r", "path": "C:/x"},
     "invalid-transfer-request"),
    ("/api/transfers/exports", {"project_id": "0" * 32, "base_revision": True, "request_id": "r"},
     "invalid-project-request"),
    ("/api/transfers/exports", {"project_id": "../x", "base_revision": 1, "request_id": "r"},
     "invalid-project-request"),
    ("/api/transfers/uploads", {"name": "a.mapforge-project.zip", "size": 1, "sha256": "0" * 64,
                                "destination": "../"}, "invalid-transfer-request"),
    ("/api/transfers/uploads", {"name": "../a.mapforge-project.zip", "size": 1, "sha256": "0" * 64},
     "invalid-upload"),
    ("/api/transfers/uploads", {"name": "a.zip", "size": 1, "sha256": "0" * 64}, "invalid-upload"),
    ("/api/transfers/imports", {"upload_id": "0" * 32, "request_id": "r", "project_id": "0" * 32},
     "invalid-transfer-request"),
    ("/api/transfers/imports", {"upload_id": "0" * 32, "request_id": "r"}, "unknown-upload"),
])
def test_requests_cross_http_only_as_declared_identities(pair, path, body, code):
    response = pair[1].post(path, json=body)
    assert response.status_code in (404, 422), response.text
    assert response.json()["code"] == code


def test_unknown_or_foreign_session_records_are_not_found(pair):
    source, target, c = pair
    job = _export(source, c.project)
    for path in [f"/api/transfers/jobs/{job['job_id']}", f"/api/transfers/jobs/{job['job_id']}/package",
                 f"/api/transfers/uploads/{'0' * 32}"]:
        response = target.get(path)
        assert response.status_code == 404, path
        assert response.json()["code"] in {"unknown-transfer", "export-unavailable", "unknown-upload"}


@pytest.mark.parametrize("change, status", [
    ({"X-Mapforge-CSRF": "wrong"}, 403),
    ({"Authorization": "Bearer wrong"}, 403),
    ({"Origin": "http://evil.example"}, 403),
    ({"Content-Type": "application/json"}, 415),
    ({"Content-Type": "text/plain"}, 415),
])
def test_chunk_route_keeps_session_and_type_guards(pair, change, status):
    _, target, _ = pair
    upload = target.post("/api/transfers/uploads", json={
        "name": "a.mapforge-project.zip", "size": 4, "sha256": "0" * 64}).json()
    response = target.put(f"/api/transfers/uploads/{upload['upload_id']}/chunks", params={"offset": 0},
                          content=b"abcd", headers={**RAW, **change})
    assert response.status_code == status
    assert target.get(f"/api/transfers/uploads/{upload['upload_id']}").json()["received_size"] == 0


def test_raw_bytes_are_limited_to_the_chunk_route_and_its_size(pair):
    _, target, _ = pair
    upload = target.post("/api/transfers/uploads", json={
        "name": "a.mapforge-project.zip", "size": project_transfer.MAX_CHUNK_BYTES + 1, "sha256": "0" * 64}).json()
    chunk = f"/api/transfers/uploads/{upload['upload_id']}/chunks"
    too_large = target.put(chunk, params={"offset": 0}, content=b"x" * (project_transfer.MAX_CHUNK_BYTES + 1),
                           headers=RAW)
    assert too_large.status_code == 413
    assert target.get(f"/api/transfers/uploads/{upload['upload_id']}").json()["received_size"] == 0
    exact = target.put(chunk, params={"offset": 0}, content=b"x" * project_transfer.MAX_CHUNK_BYTES, headers=RAW)
    assert exact.status_code == 200, exact.text
    assert exact.json()["received_size"] == project_transfer.MAX_CHUNK_BYTES
    for method, path in [("post", "/api/transfers/uploads"), ("post", "/api/transfers/imports"),
                         ("put", f"/api/transfers/uploads/{upload['upload_id']}")]:
        response = getattr(target, method)(path, content=b"abcd", headers=RAW)
        assert response.status_code == 415, path
        assert response.json()["detail"] == "JSON only"
    oversized_json = target.post("/api/transfers/exports", content=b" " * 65537,
                                 headers={"Content-Type": "application/json"})
    assert oversized_json.status_code == 413


def test_closed_session_rejects_further_transfers(case):
    client = _client(case.store, case.old_raw, case.old_profile)
    with client:
        job = _export(client, case.project)
    with client:
        response = client.get(f"/api/transfers/jobs/{job['job_id']}")
    assert response.status_code == 409 and response.json()["code"] == "session-closed"
