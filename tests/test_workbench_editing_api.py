"""Editing HTTP boundary and real store coordination; no geometry-quality claims.

The registered test service uses controlled compiler replies backed by actual
artifact files. Source-unsupported checks use the genuine raw-source adapter.
"""
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench.app import create_app
from mapforge.workbench.editing import EditingService
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from test_workbench_editing import setup as editing_setup
from test_workbench_sources import source

TOKEN = "editing-api-test-token-with-at-least-32-characters"
PORT = 19874
BASE_URL = f"http://127.0.0.1:{PORT}"
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}


@pytest.fixture
def api(editing_setup):
    env = editing_setup
    # No processes are owned by the controlled worker; the source declaration
    # here exercises HTTP/store coordination only, not original-source hashing.
    env.jobs.close = lambda: None
    catalog = SimpleNamespace(source_dir=env.service.source_dir, profile_path=env.service.profile_path,
                              snapshot_id=env.project["source_snapshot"]["snapshot_id"],
                              assert_unchanged=lambda: None)
    app = create_app(env.store, catalog, TOKEN, PORT, jobs=env.jobs, editing=env.service)
    with TestClient(app, base_url=BASE_URL, headers=HEADERS) as client:
        env.client = client
        env.url = f"/api/projects/{env.pid}"
        yield env


def enable(env):
    response = env.client.post(env.url + "/editing/enable", json={"base_revision": 0, "command_id": "enable-http"})
    assert response.status_code == 200, response.text
    return response.json()["project"]


def test_http_preview_confirm_accept_uses_server_paths_and_preserves_preview_draft(api):
    e = api
    path = e.store.project_path(e.pid) / "project.json"
    original = path.read_bytes()
    status = e.client.get(e.url + "/editing")
    assert status.status_code == 200
    assert status.json()["supported"] and not status.json()["enabled"]
    assert path.read_bytes() == original
    project = enable(e)
    saved = path.read_bytes()
    response = e.client.post(e.url + "/editing/preview", json={
        "base_revision": project["revision"], "command": e.command, "request_id": "preview-http"})
    assert response.status_code == 200, response.text
    result = response.json()
    assert result["preview"]["target_revision"] == project["revision"] + 1
    assert result["job"]["base_revision"] == project["revision"]
    assert result["job"]["content_hash"] == project["content_hash"]
    assert path.read_bytes() == saved
    assert e.store.load(e.pid)["candidate"] is None
    payload = e.jobs.payloads[result["job"]["job_id"]]
    assert payload["source_dir"] == str(e.service.source_dir)
    assert payload["profile_path"] == str(e.service.profile_path)
    assert payload["baseline_path"] == str(e.service.baseline_path)
    assert payload["project_directory"] == str(e.store.project_path(e.pid))
    assert "untrusted-client-path" not in str(e.calls)
    assert payload["draft"]["revision"] == result["preview"]["target_revision"]
    assert payload["draft"]["content_hash"] == result["preview"]["target_content_hash"]
    job_id = result["job"]["job_id"]
    observed = e.client.get(e.url + "/jobs/" + job_id)
    assert observed.status_code == 200 and observed.json()["state"] == "succeeded"
    assert path.read_bytes() == saved  # Polling never confirms or accepts an edit.
    body = {"base_revision": project["revision"], "command_id": "accept-http", "job_id": job_id}
    assert e.client.post(e.url + "/editing/accept", json=body).status_code == 409
    confirmed = e.client.post(e.url + "/commands", json={
        "base_revision": project["revision"], "command": result["preview"]["command"]})
    assert confirmed.status_code == 200
    body["base_revision"] = confirmed.json()["revision"]
    accepted = e.client.post(e.url + "/editing/accept", json=body)
    assert accepted.status_code == 200, accepted.text
    assert accepted.json()["candidate"] is not None
    assert not accepted.json()["status"]["candidate_stale"]
    assert not accepted.json()["status"]["formal_export_available"]
    accepted_bytes = path.read_bytes()
    retry = e.client.post(e.url + "/editing/accept", json=body)
    assert retry.status_code == 200 and retry.json() == accepted.json()
    assert path.read_bytes() == accepted_bytes
    assert e.client.post(e.url + "/export", json={}).status_code == 409


def test_http_compile_uses_current_saved_revision_without_creating_an_edit(api):
    e = api
    project = enable(e)
    response = e.client.post(e.url + "/editing/compile", json={
        "base_revision": project["revision"], "request_id": "compile-http"})
    assert response.status_code == 200, response.text
    job = response.json()["job"]
    draft = e.jobs.payloads[job["job_id"]]["draft"]
    assert draft["revision"] == project["revision"]
    assert draft["content_hash"] == project["content_hash"]
    assert e.store.load(e.pid) == project
    assert e.store.load(e.pid)["candidate"] is None


@pytest.mark.parametrize("revision", [None, True, False, -1, "0", 0.0])
def test_invalid_revision_rejected_before_registration_or_job_start(api, revision):
    e = api
    response = e.client.post(e.url + "/editing/enable", json={"base_revision": revision, "command_id": "enable"})
    assert response.status_code == 422, response.text
    assert not e.calls and not e.jobs.jobs
    assert e.store.load(e.pid) == e.project


def test_stale_http_revision_cannot_enqueue_or_replace_candidate(api):
    e = api
    current = enable(e)
    response = e.client.post(e.url + "/editing/preview", json={
        "base_revision": 0, "command": e.command, "request_id": "old-tab"})
    assert response.status_code == 409
    assert not e.jobs.jobs
    assert e.store.load(e.pid) == current


@pytest.mark.parametrize("value", [None, {}, [], True])
def test_non_string_command_and_job_ids_are_validation_errors_not_server_errors(api, value):
    e = api
    response = e.client.post(e.url + "/editing/enable", json={"base_revision": 0, "command_id": value})
    assert response.status_code == 422
    assert not e.calls and e.store.load(e.pid) == e.project
    current = enable(e)
    response = e.client.post(e.url + "/editing/accept", json={
        "base_revision": current["revision"], "command_id": "accept-http", "job_id": value})
    assert response.status_code == 422
    assert not e.jobs.jobs
    assert e.store.load(e.pid) == current


@pytest.mark.parametrize("operation,body,extra", [
    ("enable", {"base_revision": 0, "command_id": "e"}, {"baseline_path": "C:/client.xodr"}),
    ("enable", {"base_revision": 0, "command_id": "e"}, {"capabilities": [{"arbitrary": True}]}),
    ("preview", {"base_revision": 0, "command": {}, "request_id": "p"}, {"source_dir": "C:/client-source"}),
    ("preview", {"base_revision": 0, "command": {}, "request_id": "p"}, {"profile_path": "C:/client.yaml"}),
    ("compile", {"base_revision": 0, "request_id": "c"}, {"draft": {"content_hash": "claimed"}}),
    ("compile", {"base_revision": 0, "request_id": "c"}, {"project_directory": "C:/outside"}),
    ("accept", {"base_revision": 0, "command_id": "a", "job_id": "j"}, {"candidate": {"candidate_id": "forged"}}),
    ("accept", {"base_revision": 0, "command_id": "a", "job_id": "j"}, {"checks": "PASS", "force": True}),
])
def test_client_paths_capabilities_candidates_and_claimed_checks_are_rejected(api, operation, body, extra):
    e = api
    response = e.client.post(e.url + "/editing/" + operation, json={**body, **extra})
    assert response.status_code == 422, response.text
    assert not e.calls and not e.jobs.jobs
    assert e.store.load(e.pid) == e.project


@pytest.mark.parametrize("operation", ["enable", "preview", "compile", "accept"])
def test_get_cannot_invoke_editing_mutations(api, operation):
    e = api
    response = e.client.get(e.url + "/editing/" + operation, params={"base_revision": 0})
    assert response.status_code == 405
    assert not e.calls and not e.jobs.jobs
    assert e.store.load(e.pid) == e.project


def test_unknown_or_wrong_method_operation_cannot_fall_through_to_source_action(api):
    e = api
    for operation in ("geometry", "export", "source-check", "unknown"):
        response = e.client.post(e.url + "/editing/" + operation, json={"base_revision": 0})
        assert response.status_code == 404, response.text
    assert not e.calls and not e.jobs.jobs
    assert e.store.load(e.pid) == e.project


@pytest.mark.parametrize("headers", [
    {"Authorization": ""}, {"Authorization": "Bearer another-session"},
    {"X-Mapforge-CSRF": ""}, {"Origin": "https://foreign.example"},
    {"Host": "foreign.example"}, {"Host": f"localhost:{PORT}"},
])
def test_editing_mutations_require_local_host_origin_session_and_csrf(api, headers):
    e = api
    response = e.client.post(e.url + "/editing/enable", headers=headers,
                             json={"base_revision": 0, "command_id": "enable"})
    assert response.status_code == 403
    assert not e.calls and not e.jobs.jobs
    assert e.store.load(e.pid) == e.project


def test_get_geometry_delegates_only_project_and_job_identity_without_mutation(api, monkeypatch):
    e = api
    calls = []

    def geometry(project_id, *, job_id=None):
        calls.append((project_id, job_id))
        return {"geometry_adapter_called": True, "formal_export_available": False}

    monkeypatch.setattr(e.service, "geometry", geometry)
    path = e.store.project_path(e.pid) / "project.json"
    original = path.read_bytes()
    response = e.client.get(e.url + "/editing/geometry", params={"job_id": "registered-job"})
    assert response.status_code == 200
    assert calls == [(e.pid, "registered-job")]
    assert response.json()["formal_export_available"] is False
    assert path.read_bytes() == original
    assert not e.jobs.jobs


@pytest.mark.parametrize("operation,body,arguments", [
    ("start", {"base_revision": 7, "request_id": "check-request"}, (7, "check-request")),
    ("attach", {"base_revision": 9, "command_id": "attach-report", "job_id": "check-job"},
     (9, "attach-report", "check-job")),
])
def test_checking_routes_forward_only_identities_and_reject_client_report_or_path(api, monkeypatch, operation, body, arguments):
    from mapforge.workbench.checking import CheckingService

    e = api
    calls = []

    def invoked(self, *args):
        calls.append(args)
        return {"delegated": operation}

    monkeypatch.setattr(CheckingService, operation, invoked)
    path = e.store.project_path(e.pid) / "project.json"
    original = path.read_bytes()
    url = e.url + "/checking/" + operation
    response = e.client.post(url, json=body)
    assert response.status_code == 200 and response.json() == {"delegated": operation}
    assert calls == [(e.pid, *arguments)]
    assert e.client.get(url).status_code == 405
    rejected = e.client.post(url, json={**body, "validation": {"decision": "DELIVERABLE"},
                                       "project_directory": "C:/client-selected-directory"})
    assert rejected.status_code == 422
    assert calls == [(e.pid, *arguments)]
    assert path.read_bytes() == original and not e.jobs.jobs


def test_real_source_without_registered_geometry_scope_stays_read_only_for_editing(source, tmp_path):
    catalog = SourceCatalog(*source)
    store = ProjectStore(tmp_path / "projects")
    project = store.create(catalog.snapshot("j1"), "unsupported source")
    jobs = JobManager(timeout_s=10)
    service = EditingService(store, jobs, source_dir=catalog.source_dir, profile_path=catalog.profile_path)
    app = create_app(store, catalog, TOKEN, PORT, jobs=jobs, editing=service)
    url = f"/api/projects/{project['project_id']}/editing"
    with TestClient(app, base_url=BASE_URL, headers=HEADERS) as client:
        status = client.get(url)
        assert status.status_code == 200
        assert status.json()["supported"] is False and status.json()["capability"] is None
        assert client.post(url + "/enable", json={"base_revision": 0, "command_id": "enable"}).status_code == 422
        assert client.post(url + "/compile", json={"base_revision": 0, "request_id": "compile"}).status_code == 422
        assert client.get(url + "/geometry").status_code == 422
        assert not jobs._jobs
        assert store.load(project["project_id"]) == project
