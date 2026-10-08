"""HTTP integration of real raw SHP snapshots, draft storage and owned jobs."""
import copy
import json
import os
from pathlib import Path
import shutil
import time

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench.app import create_app
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from test_workbench_sources import source

TOKEN = "a-test-session-token-with-at-least-32-characters"
PORT = 19873
BASE_URL = f"http://127.0.0.1:{PORT}"
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}


@pytest.fixture
def workspace(source, tmp_path):
    catalog = SourceCatalog(*source)
    store = ProjectStore(tmp_path / "projects")
    jobs = JobManager(timeout_s=10)
    app = create_app(store, catalog, TOKEN, PORT, jobs=jobs)
    with TestClient(app, base_url=BASE_URL, headers=HEADERS) as client:
        yield client, catalog, store, jobs


def new_project(client, junction_id="j1", name="源工程"):
    response = client.post("/api/projects", json={"junction_id": junction_id, "name": name})
    assert response.status_code == 200, response.text
    return response.json()


def command(project, command_id="annotation-1", text="人工待核查"):
    oid = project["source_snapshot"]["objects"][0]["id"]
    return {"base_revision": project["revision"], "command": {
        "command_id": command_id, "type": "annotation", "source_ref": oid,
        "scope": {"feature_ids": [oid]}, "parameters": {"text": text, "status": "unresolved"}}}


def wait_job(client, project, job, timeout=10):
    deadline = time.monotonic() + timeout
    path = f"/api/projects/{project['project_id']}/jobs/{job['job_id']}"
    while time.monotonic() < deadline:
        response = client.get(path)
        assert response.status_code == 200, response.text
        result = response.json()
        if result["state"] != "running":
            return result
        time.sleep(.01)
    raise AssertionError("owned API task did not reach a terminal state")


def test_create_open_confirm_undo_redo_reopen_and_originals_unchanged(workspace):
    client, catalog, store, _ = workspace
    originals = {p: p.read_bytes() for p in catalog.source_dir.iterdir() if p.is_file()}
    project = new_project(client)
    pid = project["project_id"]
    assert project["source_snapshot"]["objects"]
    assert project["candidate"] is None
    assert not project["status"]["formal_export_available"]
    submitted = command(project)
    confirmed = client.post(f"/api/projects/{pid}/commands", json=submitted)
    assert confirmed.status_code == 200
    confirmed = confirmed.json()
    assert confirmed["revision"] == 1
    assert confirmed["intents"][0]["parameters"]["status"] == "unresolved"
    assert client.post(f"/api/projects/{pid}/commands", json=submitted).json() == confirmed
    undo = client.post(f"/api/projects/{pid}/undo", json={"base_revision": 1, "command_id": "undo-1"})
    assert undo.status_code == 200
    assert undo.json()["intents"] == []
    assert undo.json()["content_hash"] == project["content_hash"]
    redo = client.post(f"/api/projects/{pid}/redo", json={"base_revision": 2, "command_id": "redo-1"})
    assert redo.status_code == 200
    assert redo.json()["content_hash"] == confirmed["content_hash"]
    assert client.get(f"/api/projects/{pid}").json() == redo.json()
    assert ProjectStore(store.root).load(pid) == redo.json()
    assert client.get("/api/projects").json()[0]["project_id"] == pid
    assert all(p.read_bytes() == data for p, data in originals.items())


def test_two_windows_and_cross_project_commands_cannot_overwrite(workspace):
    client, _, store, _ = workspace
    first = new_project(client)
    second = new_project(client, "j2")
    pid = first["project_id"]
    assert client.post(f"/api/projects/{pid}/commands", json=command(first)).status_code == 200
    assert client.post(f"/api/projects/{pid}/commands", json=command(first, "old-window")).status_code == 409
    foreign = command(second)
    foreign["base_revision"] = 1
    foreign["command"]["command_id"] = "foreign-object"
    assert client.post(f"/api/projects/{pid}/commands", json=foreign).status_code == 422
    assert len(store.load(pid)["intents"]) == 1
    assert store.load(second["project_id"])["intents"] == []


@pytest.mark.parametrize("headers", [{"Host": "attacker.example"},
                                     {"Host": f"localhost:{PORT}"},
                                     {"Origin": "https://attacker.example"},
                                     {"Authorization": ""},
                                     {"Authorization": "Bearer wrong-session"}])
def test_wrong_host_origin_or_session_rejected(workspace, headers):
    client, _, store, _ = workspace
    response = client.get("/api/catalog", headers=headers)
    assert response.status_code == 403
    assert store.list() == []


@pytest.mark.parametrize("headers", [{"X-Mapforge-CSRF": ""},
                                     {"X-Mapforge-CSRF": "wrong"}])
def test_mutations_require_separate_csrf_header(workspace, headers):
    client, _, store, _ = workspace
    response = client.post("/api/projects", headers=headers, json={"junction_id": "j1", "name": "x"})
    assert response.status_code == 403
    assert store.list() == []


@pytest.mark.parametrize("header", [b"authorization", b"x-mapforge-csrf"])
def test_non_ascii_auth_headers_are_rejected_without_server_error(workspace, header):
    client, _, _, _ = workspace
    headers = [(header, b"\xff")]
    response = client.post("/api/projects", headers=headers, json={"junction_id": "j1", "name": "x"})
    assert response.status_code == 403


def test_json_body_length_limit_uses_actual_bytes_not_just_header(workspace):
    client, _, store, _ = workspace
    assert client.post("/api/projects", content="{}", headers={"Content-Type": "text/plain"}).status_code == 415
    assert client.post("/api/projects", content="{}", headers={"Content-Type": "application/json", "Content-Length": "garbage"}).status_code == 400
    for size in ("-1", "65537"):
        assert client.post("/api/projects", content="{}", headers={"Content-Type": "application/json", "Content-Length": size}).status_code == 413
    body = json.dumps({"junction_id": "j1", "name": "x", "padding": "x" * 65536})
    assert client.post("/api/projects", content=body, headers={"Content-Type": "application/json", "Content-Length": "0"}).status_code == 413
    assert client.post("/api/projects", content=(part.encode() for part in [body[:50000], body[50000:]]),
                       headers={"Content-Type": "application/json"}).status_code == 413
    assert client.post("/api/projects", content='{"incomplete":', headers={"Content-Type": "application/json"}).status_code == 422
    assert store.list() == []


def test_static_page_has_no_token_leak_and_uses_security_headers(workspace):
    client, _, _, _ = workspace
    response = client.get("/")
    assert response.status_code == 200
    assert TOKEN not in response.text
    assert response.headers["Cache-Control"] == "no-store"
    assert response.headers["X-Content-Type-Options"] == "nosniff"
    assert "frame-ancestors 'none'" in response.headers["Content-Security-Policy"]
    assert client.get("/assets/app.js").status_code == 200
    assert client.get("/assets/style.css").status_code == 200
    assert client.get("/assets/store.py").status_code == 404
    assert client.get("/docs").status_code == 404
    assert client.get("/openapi.json").status_code == 404


@pytest.mark.parametrize("override", [{"type": "boundary_offset"},
                                      {"source_ref": "unregistered-object"},
                                      {"parameters": {"text": "", "status": "unresolved"}}])
def test_unsupported_or_invalid_edits_do_not_create_hidden_history(workspace, override):
    client, _, store, _ = workspace
    project = new_project(client)
    body = command(project)
    body["command"].update(override)
    response = client.post(f"/api/projects/{project['project_id']}/commands", json=body)
    assert response.status_code == 422
    assert store.load(project["project_id"])["journal"] == []


def test_formal_export_cannot_be_forced_by_client_fields(workspace):
    client, _, _, _ = workspace
    project = new_project(client)
    response = client.post(f"/api/projects/{project['project_id']}/export", json={
        "force": True, "decision": "DELIVERABLE", "crs_verified": True, "checks": "PASS"})
    assert response.status_code == 409
    assert "未开放" in response.json()["detail"]
    catalog = client.get("/api/catalog").json()
    assert catalog["capabilities"]["geometry_edit"] == "registered-scope-only"
    assert not catalog["capabilities"]["formal_export"]


def test_disk_failure_returns_507_and_does_not_claim_saved(workspace, monkeypatch):
    client, _, store, _ = workspace
    project = new_project(client)

    def failed_write(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(store, "_persist", failed_write)
    response = client.post(f"/api/projects/{project['project_id']}/commands", json=command(project))
    assert response.status_code == 507
    assert "未确认保存" in response.json()["detail"]
    assert store.load(project["project_id"])["revision"] == 0


def test_full_reopen_hash_check_catches_same_size_same_mtime_drift(workspace):
    client, catalog, _, _ = workspace
    project = new_project(client)
    target = catalog.source_dir / "vendor-extra.bin"
    stat = target.stat()
    original = target.read_bytes()
    target.write_bytes(b"X" + original[1:])
    os.utime(target, ns=(stat.st_atime_ns, stat.st_mtime_ns))
    catalog.assert_unchanged()  # stat-only quick check intentionally cannot see this
    opened = client.get(f"/api/projects/{project['project_id']}")
    assert opened.status_code == 200
    assert opened.json()["status"]["read_only"]
    assert any(issue["code"] == "source-file-changed" for issue in opened.json()["status"]["source_issues"])
    assert client.post(f"/api/projects/{project['project_id']}/commands", json=command(project)).status_code == 409


def test_real_source_check_job_is_project_scoped_and_old_result_marked_stale(workspace):
    client, _, store, _ = workspace
    project = new_project(client)
    second = new_project(client, "j2")
    path = f"/api/projects/{project['project_id']}"
    started = client.post(path + "/source-check", json={"base_revision": 0, "request_id": "source-check"})
    assert started.status_code == 200
    job = started.json()
    assert client.get(f"/api/projects/{second['project_id']}/jobs/{job['job_id']}").status_code == 404
    assert client.post(f"/api/projects/{second['project_id']}/jobs/{job['job_id']}/cancel", json={}).status_code == 404
    confirmed = client.post(path + "/commands", json=command(project)).json()
    result = wait_job(client, project, job)
    assert result["state"] == "succeeded" and result["result"]["matches"] is True
    assert result["stale"] is True
    assert store.load(project["project_id"])["intents"] == confirmed["intents"]
    assert store.load(second["project_id"])["revision"] == 0


def test_historical_success_cannot_clear_later_verified_source_drift(workspace):
    client, catalog, _, _ = workspace
    project = new_project(client)
    path = f"/api/projects/{project['project_id']}"
    started = client.post(path + "/source-check", json={"base_revision": 0, "request_id": "old-success"}).json()
    assert wait_job(client, project, started)["result"]["matches"] is True
    target = catalog.source_dir / "vendor-extra.bin"
    stat = target.stat()
    target.write_bytes(b"X" + target.read_bytes()[1:])
    os.utime(target, ns=(stat.st_atime_ns, stat.st_mtime_ns))
    assert client.get(path).json()["status"]["read_only"]
    assert client.get(path + f"/jobs/{started['job_id']}").status_code == 200
    response = client.post(path + "/commands", json=command(project))
    assert response.status_code == 409


@pytest.mark.parametrize("revision", [True, False, 0.0, -1, None, "0"])
def test_source_check_revision_is_strict_integer(workspace, revision):
    client, _, _, _ = workspace
    project = new_project(client)
    response = client.post(f"/api/projects/{project['project_id']}/source-check",
                           json={"base_revision": revision, "request_id": "request"})
    assert response.status_code == 422


def test_old_drift_result_cannot_overrule_a_later_clean_reopen(workspace):
    client, catalog, _, _ = workspace
    project = new_project(client)
    path = f"/api/projects/{project['project_id']}"
    target = catalog.source_dir / "vendor-extra.bin"
    original, stat = target.read_bytes(), target.stat()
    target.write_bytes(b"X" + original[1:])
    os.utime(target, ns=(stat.st_atime_ns, stat.st_mtime_ns))
    started = client.post(path + "/source-check", json={
        "base_revision": 0, "request_id": "old-drift"}).json()
    assert wait_job(client, project, started)["result"]["matches"] is False
    assert client.post(path + "/commands", json=command(project)).status_code == 409
    target.write_bytes(original)
    os.utime(target, ns=(stat.st_atime_ns, stat.st_mtime_ns))
    assert client.get(path).json()["status"]["read_only"] is False
    assert client.get(path + f"/jobs/{started['job_id']}").status_code == 200
    assert client.post(path + "/commands", json=command(project)).status_code == 200


def test_source_check_stale_revision_is_conflict(workspace):
    client, _, _, _ = workspace
    project = new_project(client)
    response = client.post(f"/api/projects/{project['project_id']}/source-check",
                           json={"base_revision": 1, "request_id": "request"})
    assert response.status_code == 409


def test_real_source_schema_relocation_preserves_canonical_draft_identity(source, tmp_path):
    catalog = SourceCatalog(*source)
    first_snapshot = catalog.snapshot("j1")
    moved_source = tmp_path / "moved-source"
    shutil.copytree(catalog.source_dir, moved_source)
    moved_profile = tmp_path / "moved-profile.yaml"
    shutil.copy2(catalog.profile_path, moved_profile)
    moved_snapshot = SourceCatalog(moved_source, moved_profile).snapshot("j1")
    assert first_snapshot["locator"] != moved_snapshot["locator"]
    store = ProjectStore(tmp_path / "projects")
    first, moved = store.create(first_snapshot), store.create(moved_snapshot)
    assert first["content_hash"] == moved["content_hash"]
    assert first["context"] == moved["context"]
    other_junction = store.create(catalog.snapshot("j2"))
    assert other_junction["content_hash"] != first["content_hash"]


def test_new_service_session_reopens_saved_project_but_rejects_old_token_and_job(workspace):
    client, catalog, store, _ = workspace
    project = new_project(client)
    pid = project["project_id"]
    assert client.post(f"/api/projects/{pid}/commands", json=command(project)).status_code == 200
    job = client.post(f"/api/projects/{pid}/source-check",
                      json={"base_revision": 1, "request_id": "before-reopen"}).json()
    wait_job(client, project, job)
    next_token = "new-service-session-with-new-random-secret"
    new_store = ProjectStore(store.root)
    next_app = create_app(new_store, catalog, next_token, PORT)
    with TestClient(next_app, base_url=BASE_URL, headers={
            "Authorization": "Bearer " + next_token, "X-Mapforge-CSRF": next_token}) as reopened:
        assert reopened.get(f"/api/projects/{pid}", headers={"Authorization": "Bearer " + TOKEN}).status_code == 403
        saved = reopened.get(f"/api/projects/{pid}")
        assert saved.status_code == 200 and len(saved.json()["intents"]) == 1
        assert reopened.get(f"/api/projects/{pid}/jobs/{job['job_id']}").status_code == 404
