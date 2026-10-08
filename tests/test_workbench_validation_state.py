"""HTTP freshness only: fixture checks are not geometry/quality acceptance."""
import copy
import hashlib
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from test_workbench_sources import source
from mapforge.workbench import validation_state as state
from mapforge.workbench.app import create_app
from mapforge.workbench.contracts import digest
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore

CURRENT = "d" * 64
TOKEN = "display-freshness-test-token-0123456789"
PORT = 19879
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}


@pytest.fixture
def workspace(source, tmp_path, monkeypatch):
    catalog = SourceCatalog(*source)
    store = ProjectStore(tmp_path / "projects")
    project = store.create(catalog.snapshot("j1"))
    pid = project["project_id"]
    project = store.set_context(pid, 0, "fixture-context", "a" * 64, "b" * 64)
    artifact = store.project_path(pid) / "artifacts" / "fixture.xodr"
    artifact.parent.mkdir()
    artifact.write_bytes(b"fixture bytes: no geometry validation claimed")
    candidate = {"candidate_id": "candidate-fixture", "target_content_hash": project["content_hash"],
                 "context": project["context"], "artifacts": [{"relative_path": "artifacts/fixture.xodr",
                  "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest()}],
                 "affected_ids": [], "checks": []}
    project = store.accept_candidate(pid, 1, "accept-fixture", candidate)
    package = {"bundle_id": "validation-fixture", "candidate_id": candidate["candidate_id"],
               "candidate_hash": digest(project["candidate"]), "context": project["context"],
               "decision": "BLOCKED", "checks": [{"gate": "validation-byte-binding", "status": "PASS",
                                                    "validator_hash": CURRENT}]}
    project = store.attach_validation(pid, 2, "attach-fixture", package)
    fingerprints = {"validator_hash": CURRENT, "implementation_files_sha256": {"fixture.py": CURRENT},
                    "compiler_policy": {"compiler_hash": "a" * 64, "policy_hash": "b" * 64}}
    monkeypatch.setattr(state, "_current_fingerprints", lambda: copy.deepcopy(fingerprints))
    jobs = JobManager(timeout_s=10)
    app = create_app(store, catalog, TOKEN, PORT, jobs=jobs)
    with TestClient(app, base_url=f"http://127.0.0.1:{PORT}", headers=HEADERS) as client:
        yield SimpleNamespace(client=client, store=store, catalog=catalog, pid=pid,
                              project=project, fingerprints=fingerprints, jobs=jobs)


def disk_bytes(workspace):
    return {str(p): p.read_bytes() for p in workspace.store.root.rglob("*") if p.is_file()}


def get_project(workspace):
    response = workspace.client.get(f"/api/projects/{workspace.pid}")
    assert response.status_code == 200
    return response.json()


def codes(project):
    return {r["code"] for r in project["status"]["validation_stale_reasons"]}


def test_api_validator_upgrade_marks_saved_check_stale_without_disk_mutation(workspace):
    w = workspace
    before = disk_bytes(w)
    assert not get_project(w)["status"]["validation_stale"]
    w.fingerprints["validator_hash"] = "e" * 64
    changed = get_project(w)
    assert changed["status"]["validation_stale"] is True
    assert codes(changed) == {"validator-stale"}
    assert changed["validation"] == w.project["validation"]
    assert changed["candidate"] == w.project["candidate"]
    assert changed["source_snapshot"] == w.project["source_snapshot"]
    assert changed["revision"] == w.project["revision"]
    assert disk_bytes(w) == before
    # Runtime state is not smuggled into the persistent store format.
    assert w.store.load(w.pid) == w.project


def test_api_fresh_check_remains_current_without_disk_mutation(workspace):
    before = disk_bytes(workspace)
    project = get_project(workspace)
    assert project["status"]["validation_stale"] is False
    assert project["status"]["validation_stale_reasons"] == []
    assert project["validation"]["decision"] == "BLOCKED"
    assert project["status"]["formal_export_available"] is False
    assert disk_bytes(workspace) == before


@pytest.mark.parametrize("key", ["compiler_hash", "policy_hash"])
def test_api_compiler_or_policy_upgrade_is_stale_even_if_validator_hash_unchanged(workspace, key):
    w = workspace
    before = disk_bytes(w)
    w.fingerprints["compiler_policy"][key] = "f" * 64
    result = get_project(w)
    assert result["status"]["validation_stale"] is True
    assert codes(result) == {"compiler-policy-stale"}
    assert result["context"] == w.project["context"]
    assert result["validation"] == w.project["validation"]
    assert disk_bytes(w) == before


@pytest.mark.parametrize("compiler_policy", [None, {}, {"compiler_hash": "a" * 64},
                                             {"compiler_hash": "a" * 64, "policy_hash": "bad"}])
def test_api_missing_compiler_policy_fails_closed(workspace, compiler_policy):
    workspace.fingerprints["compiler_policy"] = compiler_policy
    result = get_project(workspace)
    assert result["status"]["validation_stale"] is True
    assert codes(result) == {"validator-fingerprint-unavailable"}


@pytest.mark.parametrize("checks", [
    [], [{"gate": "historical-check", "status": "PASS"}],
    [{"gate": "validation-byte-binding", "status": "PASS"}],
    [{"gate": "validation-byte-binding", "status": "PASS", "validator_hash": "invalid"}],
    [{"gate": "validation-byte-binding", "status": "FAIL", "validator_hash": CURRENT}],
    [{"gate": "validation-byte-binding", "status": "PASS", "validator_hash": CURRENT}] * 2,
])
def test_api_missing_or_ambiguous_binding_is_stale(workspace, checks):
    w = workspace
    package = copy.deepcopy(w.project["validation"])
    package["checks"] = checks or [{"gate": "old-unbound-check", "status": "PASS"}]
    w.store.attach_validation(w.pid, 3, "attach-old-binding", package)
    before = disk_bytes(w)
    result = get_project(w)
    assert result["status"]["validation_stale"] is True
    assert codes(result) == {"validator-binding-missing"}
    assert disk_bytes(w) == before


@pytest.mark.parametrize("fingerprints", [None, {}, {"validator_hash": CURRENT},
    {"validator_hash": CURRENT, "implementation_files_sha256": {"dll": None}}])
def test_api_unavailable_fingerprint_fails_closed(workspace, monkeypatch, fingerprints):
    monkeypatch.setattr(state, "_current_fingerprints", lambda: fingerprints)
    project = get_project(workspace)
    assert project["status"]["validation_stale"] is True
    assert codes(project) == {"validator-fingerprint-unavailable"}


def test_api_fingerprint_read_error_marks_history_without_http_failure(workspace, monkeypatch):
    def unavailable():
        raise OSError("missing live DLL")
    monkeypatch.setattr(state, "_current_fingerprints", unavailable)
    project = get_project(workspace)
    assert codes(project) == {"validator-fingerprint-unavailable"}
    assert project["status"]["validation_stale"] is True


def test_existing_staleness_and_reasons_are_never_cleared(workspace):
    project = copy.deepcopy(workspace.project)
    project["status"]["validation_stale"] = True
    project["status"]["read_only"] = True
    project["status"]["source_issues"] = [{"code": "source-drift"}]
    project["status"]["validation_stale_reasons"] = [{"code": "previous-stale"}]
    original = copy.deepcopy(project)
    result = state.project_view(project)
    assert result["status"] == original["status"]
    result["validation"]["checks"].clear()
    assert project == original


def test_project_without_validation_avoids_live_hash_reads(monkeypatch):
    def forbidden():
        pytest.fail("No check is saved, so live fingerprints are unnecessary")
    monkeypatch.setattr(state, "_current_fingerprints", forbidden)
    project = {"validation": None, "status": {"validation_stale": False}}
    assert state.project_view(project) == project


def test_command_undo_redo_responses_preserve_existing_stale_status(workspace):
    w = workspace
    obj = w.project["source_snapshot"]["objects"][0]["id"]
    command = {"command_id": "note", "type": "annotation", "source_ref": obj,
               "scope": {"feature_ids": [obj]}, "parameters": {"text": "note"}}
    result = w.client.post(f"/api/projects/{w.pid}/commands",
                           json={"base_revision": 3, "command": command})
    assert result.status_code == 200
    assert "project-validation-stale" in codes(result.json())
    for revision, action in [(4, "undo"), (5, "redo")]:
        result = w.client.post(f"/api/projects/{w.pid}/{action}",
                               json={"base_revision": revision, "command_id": action})
        assert result.status_code == 200
        assert result.json()["status"]["validation_stale"] is True
        assert "project-validation-stale" in codes(result.json())


@pytest.mark.parametrize("route", ["editing/enable", "editing/accept", "checking/attach"])
def test_service_project_responses_include_runtime_staleness(workspace, route):
    w = workspace
    w.fingerprints["validator_hash"] = "e" * 64
    saved = copy.deepcopy(w.project)
    editing = SimpleNamespace(enable=lambda *a: {"project": saved, "capability": {"unchanged": True}},
                              accept=lambda *a: saved)
    checking = SimpleNamespace(attach=lambda *a: saved)
    app = create_app(w.store, w.catalog, TOKEN, PORT, jobs=w.jobs, editing=editing, checking=checking)
    before = disk_bytes(w)
    with TestClient(app, base_url=f"http://127.0.0.1:{PORT}", headers=HEADERS) as client:
        response = client.post(f"/api/projects/{w.pid}/{route}", json={"base_revision": 2})
    assert response.status_code == 200
    value = response.json()
    if route.endswith("enable"):
        assert value["capability"] == {"unchanged": True}
        value = value["project"]
    assert value["status"]["validation_stale"] is True
    assert codes(value) == {"validator-stale"}
    assert saved == w.project
    assert disk_bytes(w) == before
