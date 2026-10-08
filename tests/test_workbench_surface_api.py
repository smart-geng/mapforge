"""HTTP identities and validator selection; no geometric acceptance claims."""
import copy
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench.app import create_app
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from mapforge.workbench import surface_routing as routing, validation_state
from test_workbench_sources import source

TOKEN = "surface-http-test-token-with-at-least-32-characters"
PORT = 19881
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}


@pytest.fixture
def api(source, tmp_path):
    catalog = SourceCatalog(*source)
    store = ProjectStore(tmp_path / "projects")
    project = store.create(catalog.snapshot("j1"))
    calls = []
    service = SimpleNamespace()
    for name in ("enable", "preview", "compile", "accept"):
        def call(*args, _name=name):
            calls.append((_name, args))
            return {"operation": _name}
        setattr(service, name, call)
    service.describe = lambda value: {"supported": False, "project_id": value["project_id"]}
    service.geometry = lambda pid, job_id=None: {"available": False, "project_id": pid, "job_id": job_id}
    jobs = JobManager(timeout_s=10)
    app = create_app(store, catalog, TOKEN, PORT, jobs=jobs, surface_editing=service)
    with TestClient(app, base_url=f"http://127.0.0.1:{PORT}", headers=HEADERS) as client:
        yield SimpleNamespace(client=client, store=store, project=project, calls=calls,
                              url=f"/api/projects/{project['project_id']}/surface-rebuild")


OPERATIONS = [
    ("enable", {"base_revision": 0, "command_id": "enable"}, (0, "enable")),
    ("preview", {"base_revision": 1, "command": {"command_id": "intent"}, "request_id": "preview"},
     (1, {"command_id": "intent"}, "preview")),
    ("compile", {"base_revision": 2, "request_id": "compile"}, (2, "compile")),
    ("accept", {"base_revision": 3, "command_id": "accept", "job_id": "job"}, (3, "accept", "job")),
]


@pytest.mark.parametrize("operation,body,arguments", OPERATIONS)
def test_http_passes_only_project_and_transaction_identities(api, operation, body, arguments):
    response = api.client.post(api.url + "/" + operation, json=body)
    assert response.status_code == 200, response.text
    assert api.calls == [(operation, (api.project["project_id"], *arguments))]
    assert api.store.load(api.project["project_id"]) == api.project


@pytest.mark.parametrize("operation,body,arguments", OPERATIONS)
@pytest.mark.parametrize("extra", [{"source_dir": "C:/outside"}, {"candidate": {}},
                                    {"checks": "PASS"}, {"baseline_path": "old.xodr"}])
def test_client_paths_candidates_or_claimed_checks_are_rejected(api, operation, body, arguments, extra):
    assert api.client.post(api.url + "/" + operation, json={**body, **extra}).status_code == 422
    assert not api.calls


@pytest.mark.parametrize("operation,body,arguments", OPERATIONS)
def test_missing_fields_and_get_cannot_start_mutations(api, operation, body, arguments):
    for field in body:
        missing = {k: v for k, v in body.items() if k != field}
        assert api.client.post(api.url + "/" + operation, json=missing).status_code == 422
    assert api.client.get(api.url + "/" + operation).status_code == 405
    assert not api.calls


@pytest.mark.parametrize("headers", [{"Authorization": ""}, {"X-Mapforge-CSRF": ""},
                                     {"Origin": "https://foreign.example"}, {"Host": "foreign.example"}])
def test_surface_mutations_share_loopback_session_csrf_boundary(api, headers):
    response = api.client.post(api.url + "/enable", headers=headers,
                               json={"base_revision": 0, "command_id": "e"})
    assert response.status_code == 403
    assert not api.calls


def test_reads_do_not_mutate_project_and_unknown_operation_does_not_fall_through(api):
    assert api.client.get(api.url).json()["supported"] is False
    geometry = api.client.get(api.url + "/geometry", params={"job_id": "owned-job"}).json()
    assert geometry == {"available": False, "project_id": api.project["project_id"], "job_id": "owned-job"}
    assert api.client.post(api.url + "/export", json={}).status_code == 404
    assert not api.calls
    assert api.store.load(api.project["project_id"]) == api.project


@pytest.mark.parametrize("value", [None, {}, {"source_snapshot": None}, {"source_snapshot": {}},
                                    {"source_snapshot": {"junction_id": "another"}}])
def test_routing_never_guesses_surface_source(value):
    assert routing.is_surface_project(value) is False


def test_surface_freshness_uses_its_validator_and_never_old_node16(monkeypatch):
    from mapforge.workbench import surface_validation
    def forbidden():
        pytest.fail("A surface candidate must not use node16 validator fingerprints")
    monkeypatch.setattr(validation_state, "_current_fingerprints", forbidden)
    fingerprints = {"validator_hash": "a" * 64,
                    "implementation_files_sha256": {"surface.py": "a" * 64},
                    "compiler_policy": {"compiler_hash": "b" * 64, "policy_hash": "c" * 64}}
    monkeypatch.setattr(surface_validation, "validation_fingerprints", lambda: fingerprints)
    project = {"source_snapshot": {"junction_id": routing.JUNCTION_ID},
               "context": copy.deepcopy(fingerprints["compiler_policy"]),
               "status": {"validation_stale": False},
               "validation": {"checks": [{"gate": "validation-byte-binding", "status": "PASS",
                                             "validator_hash": "a" * 64}]}}
    original = copy.deepcopy(project)
    assert not validation_state.project_view(project)["status"]["validation_stale"]
    fingerprints["validator_hash"] = "d" * 64
    assert validation_state.project_view(project)["status"]["validation_stale"]
    assert project == original
