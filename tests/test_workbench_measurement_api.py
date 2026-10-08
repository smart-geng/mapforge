"""HTTP measurements use raw SHP parts without mutating the saved project."""
import hashlib
import json

from fastapi.testclient import TestClient
import pytest

from mapforge.workbench.app import create_app
from mapforge.workbench.jobs import JobManager
from mapforge.workbench.sources import SourceCatalog
from mapforge.workbench.store import ProjectStore
from test_workbench_sources import source

TOKEN = "measurement-api-fixture-session-token-32-characters"
PORT = 19877
BASE_URL = f"http://127.0.0.1:{PORT}"
HEADERS = {"Authorization": "Bearer " + TOKEN, "X-Mapforge-CSRF": TOKEN}


@pytest.fixture
def workspace(source, tmp_path, request):
    raw, profile = source
    if getattr(request, "param", "degree") == "degree":
        (raw / "lane.prj").write_text(
            'GEOGCS["fixture declaration",DATUM["D_WGS_1984",'
            'SPHEROID["WGS84",6378137,298.257223563]],PRIMEM["Greenwich",0],'
            'UNIT["Degree",0.017453292519943295]]', encoding="utf-8")
    catalog = SourceCatalog(raw, profile)
    store = ProjectStore(tmp_path / "projects")
    app = create_app(store, catalog, TOKEN, PORT, jobs=JobManager(timeout_s=10))
    with TestClient(app, base_url=BASE_URL, headers=HEADERS) as client:
        response = client.post("/api/projects", json={"junction_id": "j1", "name": "量距测试工程"})
        assert response.status_code == 200, response.text
        yield client, catalog, store, response.json()


def _url(project):
    return f"/api/projects/{project['project_id']}/measurement"


def _original_part(project, part_index=0):
    return next(obj for obj in project["source_snapshot"]["objects"]
                if obj["role"] == "lane" and obj["business_id"] == "l1"
                and obj["source_ref"]["record_index"] == 0
                and obj["source_ref"]["part_index"] == part_index)


def _files(root):
    return {path.relative_to(root).as_posix(): path.read_bytes()
            for path in root.rglob("*") if path.is_file()}


def test_capability_and_raw_point_measurement_show_units_and_do_not_authorize_crs(workspace):
    client, _, store, project = workspace
    capability = client.get(_url(project))
    assert capability.status_code == 200
    frame = capability.json()
    assert frame["mode"] == "raw-coordinate"
    assert frame["unit"] == "degree"
    assert frame["display_label"] == "原始坐标长度（degree）"
    assert frame["reason_code"] in {"audit-source-mismatch", "audit-unavailable"}
    assert frame["assumptions"] and frame["pipeline"] is None
    assert not frame["metric_available"]
    assert not frame["absolute_crs_verified"]
    assert not frame["production_authority"]
    assert not frame["metric_edit_allowed"]
    response = client.post(_url(project), json={"points": [[0, 0], [3, 4]]})
    assert response.status_code == 200, response.text
    result = response.json()
    assert result["length"] == 5
    assert result["unit"] == "degree"
    assert result["operation"] == "point-distance"
    assert result["identity"]["points"] == [[0, 0], [3, 4]]
    assert result["frame"]["frame_id"] == frame["frame_id"]
    assert not result["absolute_crs_verified"] and not result["production_authority"]
    assert store.load(project["project_id"])["source_snapshot"]["frame"]["absolute_crs_status"] == "unverified"
    assert response.headers["Cache-Control"] == "no-store"


@pytest.mark.parametrize("workspace", ["unknown"], indirect=True)
def test_unknown_geometry_unit_is_not_inferred_from_scalar_profile(workspace):
    client, _, _, project = workspace
    result = client.post(_url(project), json={"points": [[0, 0], [3, 4]]}).json()
    assert result["unit"] == "unknown"
    assert result["raw_unit"] == "unknown"
    assert result["length"] == 5
    assert result["frame"]["mode"] == "raw-coordinate"
    assert project["source_snapshot"]["frame"]["attribute_length_unit"] == "mystery"


def test_multipart_geometry_measures_one_original_part_and_retains_row_identity(workspace):
    client, _, _, project = workspace
    measurements = []
    for part_index in (0, 1):
        obj = _original_part(project, part_index)
        response = client.post(_url(project), json={"object_id": obj["id"]})
        assert response.status_code == 200, response.text
        result = response.json()
        assert result["operation"] == "source-polyline-length"
        assert result["length"] == 5  # Each source part is 5 units; do not bridge the gap.
        assert result["point_count"] == 2 and result["segment_count"] == 1
        assert result["identity"]["object_id"] == obj["id"]
        assert result["identity"]["source_ref"] == obj["source_ref"]
        assert not result["identity"]["parts_joined"]
        assert not result["identity"]["closure_added"]
        measurements.append(result)
    assert measurements[0]["identity"]["object_id"] != measurements[1]["identity"]["object_id"]
    assert measurements[0]["identity"]["source_ref"]["record_index"] == measurements[1]["identity"]["source_ref"]["record_index"]


def test_measurement_keeps_existing_intents_candidate_history_and_all_original_bytes(workspace):
    client, catalog, store, project = workspace
    pid = project["project_id"]
    obj = _original_part(project)
    note = {"command_id": "measurement-test-note", "type": "annotation", "source_ref": obj["id"],
            "scope": {"feature_ids": [obj["id"]]}, "parameters": {"text": "待核对", "status": "unresolved"}}
    project = store.commit(pid, project["revision"], note)
    project = store.set_context(pid, project["revision"], "test-context", "a" * 64, "b" * 64)
    artifact = store.project_path(pid) / "artifacts" / "measurement-test-candidate.xodr"
    artifact.parent.mkdir()
    data = b"storage fixture only; not a geometric output or quality claim"
    artifact.write_bytes(data)
    candidate = {"candidate_id": "measurement-test-candidate", "target_content_hash": project["content_hash"],
                 "context": project["context"], "artifacts": [{"relative_path": "artifacts/" + artifact.name,
                                                               "sha256": hashlib.sha256(data).hexdigest()}],
                 "affected_ids": [], "checks": [{"id": "compile", "status": "PASS"}]}
    before = store.accept_candidate(pid, project["revision"], "accept-test-fixture", candidate)
    disk_before = _files(store.root)
    source_before = _files(catalog.source_dir)
    assert client.get(_url(project)).status_code == 200
    assert client.post(_url(project), json={"object_id": obj["id"]}).status_code == 200
    assert client.post(_url(project), json={"points": [[0, 0], [3, 4]]}).status_code == 200
    assert store.load(pid) == before
    assert ProjectStore(store.root).load(pid) == before
    assert _files(store.root) == disk_before
    assert _files(catalog.source_dir) == source_before


@pytest.mark.parametrize("extra", [
    {"audit_path": "C:/private/report.json"}, {"source_dir": "C:/private"},
    {"path": "../outside.shp"}, {"epsg": 4326}, {"pipeline": "+proj=longlat"},
    {"absolute_crs_verified": True}, {"unit": "m"}, {"production_authority": True},
])
def test_client_cannot_select_paths_projection_units_or_approval(workspace, extra):
    client, _, store, project = workspace
    before = store.load(project["project_id"])
    response = client.post(_url(project), json={"points": [[0, 0], [3, 4]], **extra})
    assert response.status_code == 422
    assert "不能指定投影或文件路径" in response.json()["detail"]
    assert store.load(project["project_id"]) == before


@pytest.mark.parametrize("body", [
    {}, {"object_id": "x", "points": [[0, 0], [1, 1]]}, {"points": []},
    {"points": [[0, 0]]}, {"points": [[0, 0]] * 3}, {"points": [[True, 0], [1, 1]]},
    {"points": [["0", 0], [1, 1]]}, {"points": [[0, 0, 0], [1, 1]]},
    {"object_id": "../outside.shp"}, {"object_id": None}, {"object_id": 1},
])
def test_invalid_body_or_points_return_422_without_saved_changes(workspace, body):
    client, _, store, project = workspace
    before = store.load(project["project_id"])
    response = client.post(_url(project), json=body)
    assert response.status_code == 422, response.text
    assert store.load(project["project_id"]) == before


@pytest.mark.parametrize("bad_number", ["NaN", "Infinity", "-Infinity", "1e999"])
def test_nonfinite_json_points_are_rejected_without_server_error(workspace, bad_number):
    client, _, _, project = workspace
    response = client.post(_url(project), content='{"points":[[' + bad_number + ',0],[1,1]]}',
                           headers={"Content-Type": "application/json"})
    assert response.status_code == 422


def test_foreign_source_identity_and_non_geometry_rows_are_rejected(workspace):
    client, _, store, project = workspace
    other = client.post("/api/projects", json={"junction_id": "j2", "name": "另一工程"}).json()
    foreign = next(obj for obj in other["source_snapshot"]["objects"] if obj["role"] == "lane")
    response = client.post(_url(project), json={"object_id": foreign["id"]})
    assert response.status_code == 422
    null_row = next(obj for obj in project["source_snapshot"]["objects"] if obj["shape_type"] == 0)
    assert client.post(_url(project), json={"object_id": null_row["id"]}).status_code == 422
    assert store.load(project["project_id"])["revision"] == 0


@pytest.mark.parametrize("method", ["get", "post"])
@pytest.mark.parametrize("headers", [
    {"Host": "attacker.example"}, {"Host": f"localhost:{PORT}"},
    {"Origin": "https://attacker.example"}, {"Authorization": ""},
    {"Authorization": "Bearer wrong-session"},
])
def test_measurement_uses_common_host_origin_and_session_permissions(workspace, method, headers):
    client, _, store, project = workspace
    kwargs = {"json": {"points": [[0, 0], [1, 1]]}} if method == "post" else {}
    response = client.request(method, _url(project), headers=headers, **kwargs)
    assert response.status_code == 403
    assert store.load(project["project_id"])["revision"] == 0


def test_measurement_post_uses_common_csrf_json_and_actual_body_size_limits(workspace):
    client, _, store, project = workspace
    url = _url(project)
    assert client.get(url, headers={"X-Mapforge-CSRF": ""}).status_code == 200
    assert client.post(url, json={"points": [[0, 0], [1, 1]]}, headers={"X-Mapforge-CSRF": ""}).status_code == 403
    assert client.post(url, content="{}", headers={"Content-Type": "text/plain"}).status_code == 415
    body = json.dumps({"points": [[0, 0], [1, 1]], "padding": "x" * 65536})
    assert client.post(url, content=body, headers={"Content-Type": "application/json", "Content-Length": "0"}).status_code == 413
    assert store.load(project["project_id"])["revision"] == 0


def test_missing_project_measurement_returns_404(workspace):
    client, _, _, _ = workspace
    url = "/api/projects/" + "a" * 32 + "/measurement"
    assert client.get(url).status_code == 404
    assert client.post(url, json={"points": [[0, 0], [1, 1]]}).status_code == 404
