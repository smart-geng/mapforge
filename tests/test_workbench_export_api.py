"""Research-package HTTP contract over the real exporter/store/ZIP fixture.

The inherited fixture controls source verification and expensive quality output;
these tests prove transport and permission behavior, not geometric acceptance.
"""
import hashlib
import io
import json
from types import SimpleNamespace
import zipfile

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench.app import create_app
from mapforge.workbench.contracts import StoreConflict, StoreReadOnly, StoreValidation
from test_workbench_exports import exportable, checked, setup

TOKEN = "research-export-api-test-session-token-32-characters"
PORT = 19878
BASE_URL = f"http://127.0.0.1:{PORT}"
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}


@pytest.fixture
def client(exportable):
    e = exportable
    catalog = SimpleNamespace(source_dir=e.service.source_dir, profile_path=e.service.profile_path,
                              snapshot_id=e.current["source_snapshot"]["snapshot_id"],
                              assert_unchanged=lambda: None, list_junctions=lambda: [])
    app = create_app(e.store, catalog, TOKEN, PORT, jobs=e.archiver,
                     editing=e.service, checking=e.checker, exports=e.exporter)
    with TestClient(app, base_url=BASE_URL, headers=HEADERS) as session:
        yield session


def _url(e):
    return f"/api/projects/{e.pid}/research-exports"


def _create(client, e):
    response = client.post(_url(e), json={"base_revision": e.current["revision"]})
    assert response.status_code == 200, response.text
    return response.json()


def test_create_download_transports_exact_zip_with_research_headers(client, exportable):
    e = exportable
    before = (e.store.project_path(e.pid) / "project.json").read_bytes()
    receipt = _create(client, e)
    assert receipt["project_id"] == e.pid
    assert receipt["revision"] == e.current["revision"]
    assert receipt["purpose"] == "RESEARCH_ONLY"
    assert receipt["decision"] == "BLOCKED"
    assert not receipt["formal_delivery"]
    downloaded = client.get(_url(e) + "/" + receipt["export_id"])
    assert downloaded.status_code == 200, downloaded.text
    _, original_bytes = e.exporter.download(e.pid, receipt["export_id"])
    assert downloaded.content == original_bytes
    assert downloaded.headers["Content-Type"] == "application/zip"
    assert downloaded.headers["Content-Disposition"] == f'attachment; filename="{receipt["file_name"]}"'
    assert "RESEARCH_ONLY.zip" in downloaded.headers["Content-Disposition"]
    assert downloaded.headers["X-Mapforge-Purpose"] == "RESEARCH_ONLY"
    assert downloaded.headers["X-Content-SHA256"] == receipt["sha256"] == hashlib.sha256(downloaded.content).hexdigest()
    assert int(downloaded.headers["Content-Length"]) == receipt["size_bytes"] == len(downloaded.content)
    assert downloaded.headers["Cache-Control"] == "no-store"
    assert downloaded.headers["X-Content-Type-Options"] == "nosniff"
    with zipfile.ZipFile(io.BytesIO(downloaded.content)) as archive:
        manifest = json.loads(archive.read("research-manifest.json"))
        assert manifest["validation"]["decision"] == "BLOCKED"
        assert manifest["purpose"] == "RESEARCH_ONLY"
        assert manifest["formal_delivery"] is False
    assert (e.store.project_path(e.pid) / "project.json").read_bytes() == before
    assert _create(client, e) == receipt


@pytest.mark.parametrize("body", [
    {}, {"base_revision": 4, "force": True}, {"base_revision": 4, "decision": "DELIVERABLE"},
    {"base_revision": 4, "destination": "C:/other"}, {"base_revision": 4, "path": "../outside.zip"},
    {"base_revision": 4, "candidate": {}}, {"base_revision": 4, "format": "formal"},
    {"base_revision": 4, "crs_verified": True}, {"base_revision": 4, "export_id": "client-chosen"},
])
def test_extra_fields_or_missing_revision_are_rejected_before_exporter(client, exportable, monkeypatch, body):
    called = []
    monkeypatch.setattr(exportable.exporter, "create", lambda *args: called.append(args))
    response = client.post(_url(exportable), json=body)
    assert response.status_code == 422, response.text
    assert "仅接受当前 base_revision" in response.json()["detail"]
    assert called == []


@pytest.mark.parametrize("revision,status", [(None, 422), (True, 422), (-1, 422), ("4", 422), (3, 409), (5, 409)])
def test_revision_value_validation_and_stale_version_reach_http_error(client, exportable, revision, status):
    e = exportable
    response = client.post(_url(e), json={"base_revision": revision})
    assert response.status_code == status, response.text
    assert not (e.store.project_path(e.pid) / ".research-exports").exists()
    assert e.store.load(e.pid)["revision"] == 4


def test_formal_export_remains_blocked_after_research_download(client, exportable):
    e = exportable
    receipt = _create(client, e)
    assert client.get(_url(e) + "/" + receipt["export_id"]).status_code == 200
    response = client.post(f"/api/projects/{e.pid}/export", json={
        "base_revision": 4, "force": True, "purpose": "DELIVERABLE", "crs_verified": True})
    assert response.status_code == 409
    assert "未开放" in response.json()["detail"]
    assert not e.store.load(e.pid)["status"]["formal_export_available"]


@pytest.mark.parametrize("operation", ["create", "download"])
@pytest.mark.parametrize("headers", [
    {"Authorization": ""}, {"Authorization": "Bearer other-session"},
    {"Origin": "https://attacker.example"}, {"Host": "attacker.example"},
    {"Host": f"localhost:{PORT}"},
])
def test_common_session_host_and_origin_checks_run_before_exporter(client, exportable, monkeypatch, operation, headers):
    e = exportable
    called = []
    monkeypatch.setattr(e.exporter, operation, lambda *args: called.append(args))
    if operation == "create":
        response = client.post(_url(e), json={"base_revision": 4}, headers=headers)
    else:
        response = client.get(_url(e) + "/research-" + "a" * 32, headers=headers)
    assert response.status_code == 403, response.text
    assert called == []


def test_create_requires_csrf_json_and_body_limit_but_download_does_not_require_csrf(client, exportable):
    e = exportable
    assert client.post(_url(e), json={"base_revision": 4}, headers={"X-Mapforge-CSRF": ""}).status_code == 403
    assert client.post(_url(e), content="{}", headers={"Content-Type": "text/plain"}).status_code == 415
    body = json.dumps({"base_revision": 4, "padding": "x" * 65536})
    assert client.post(_url(e), content=body, headers={"Content-Type": "application/json", "Content-Length": "0"}).status_code == 413
    receipt = _create(client, e)
    assert client.get(_url(e) + "/" + receipt["export_id"], headers={"X-Mapforge-CSRF": ""}).status_code == 200


@pytest.mark.parametrize("operation", ["create", "download"])
@pytest.mark.parametrize("error,status", [
    (StoreConflict("工程已变化"), 409), (StoreReadOnly("源文件改变"), 409),
    (StoreValidation("研究包不完整"), 422), (OSError("test disk unavailable"), 507),
])
def test_service_failure_returns_error_without_zip_or_success_receipt(client, exportable, monkeypatch, operation, error, status):
    e = exportable
    def failed(*args):
        raise error
    monkeypatch.setattr(e.exporter, operation, failed)
    if operation == "create":
        response = client.post(_url(e), json={"base_revision": 4})
    else:
        response = client.get(_url(e) + "/research-" + "a" * 32)
    assert response.status_code == status, response.text
    assert response.headers["Content-Type"].startswith("application/json")
    assert "detail" in response.json()
    assert "Content-Disposition" not in response.headers
    assert "X-Content-SHA256" not in response.headers
    assert "X-Mapforge-Purpose" not in response.headers


def test_download_of_wrong_or_cross_project_export_cannot_return_archive(client, exportable):
    e = exportable
    receipt = _create(client, e)
    wrong = client.get(_url(e) + "/research-" + "a" * 32)
    assert wrong.status_code == 422
    other = e.store.create(e.current["source_snapshot"])
    response = client.get(f"/api/projects/{other['project_id']}/research-exports/{receipt['export_id']}")
    assert response.status_code == 409
    assert response.headers["Content-Type"].startswith("application/json")
    assert "Content-Disposition" not in response.headers


@pytest.mark.parametrize("export_id", ["outside.zip", "%2e%2e", "C%3Aprivate", "..%2Foutside.zip"])
def test_download_takes_registered_export_identity_not_file_path(client, exportable, export_id):
    response = client.get(_url(exportable) + "/" + export_id)
    assert response.status_code in {404, 422}, response.text
    assert "Content-Disposition" not in response.headers


def test_later_saved_edit_disables_download_of_previously_created_package(client, exportable):
    e = exportable
    receipt = _create(client, e)
    e.store.commit(e.pid, 4, {"command_id": "later", "type": "annotation", "source_ref": "boundary",
                              "scope": {"feature_ids": ["boundary"]}, "parameters": {"text": "later"}})
    response = client.get(_url(e) + "/" + receipt["export_id"])
    assert response.status_code == 409
    assert "Content-Disposition" not in response.headers
