"""Source measurement stays read-only and never promotes declarations to CRS truth."""
import copy
import hashlib
import json

import pytest
from pyproj import Geod

from mapforge.workbench import measurements as module
from mapforge.workbench.measurements import (
    MeasurementRejected, measure_object, measure_points, measurement_capability,
)


def _bind(snapshot):
    snapshot["content_hash"] = module._digest({k: v for k, v in snapshot.items()
                                             if k not in {"content_hash", "locator"}})
    return snapshot


@pytest.fixture
def source():
    return _bind({
        "schema": "mapforge/workbench-source/v1", "snapshot_id": "dataset-fixture",
        "locator": {"source_dir": "ignored-path"},
        "source_files": [{"relative_path": "lane.prj", "size": 1, "sha256": "a" * 64},
                         {"relative_path": "lane.shp", "size": 2, "sha256": "b" * 64}],
        "frame": {"kind": "raw", "coordinate_unit": "degree", "transform": "identity",
                  "attribute_length_unit": "mm", "metric_edit_allowed": False,
                  "absolute_crs_status": "unverified",
                  "unit_sources": [{"file": "lane.prj", "unit": "degree", "sha256": "a" * 64}]},
        "bounds": [106.31, 29.51, 106.33, 29.53],
        "objects": [{"id": "line:part0", "source_ref": {"part_index": 0, "record_index": 1},
                     "shape_type": 13, "points": [[106.32, 29.52], [106.3202, 29.5201], [106.3203, 29.5203]],
                     "raw_z": [0, 99, 50], "raw_m": [0, 0, 0]},
                    {"id": "line:part1", "source_ref": {"part_index": 1, "record_index": 1},
                     "shape_type": 3, "points": [[106.32, 29.52], [106.3203, 29.5203]]}],
    })


@pytest.fixture
def audit(source, tmp_path, monkeypatch):
    # The unit fixture is explicitly registered in this test only. Production
    # accepts only the pinned real report, never a hash supplied by an HTTP user.
    report = {
        "source_files": [{"path": "shp_0222-0326/" + f["relative_path"],
                          "sha256": f["sha256"], "size": f["size"]} for f in source["source_files"]],
        "declarations": {"prj_files": [{"file": "lane.prj", "sha256": "a" * 64}]},
        "projection": {
            "status": "PASS", "scope": "numerical-transform-only", "input_order": "longitude,latitude",
            "input_unit": "degree", "output_unit": "m", "network_enabled": False,
            "origin": [106.32, 29.52],
            "pipeline": "proj=pipeline step proj=unitconvert xy_in=deg xy_out=rad step proj=aeqd lon_0=106.32 lat_0=29.52 a=6378137 rf=298.257223563",
        },
    }
    data = json.dumps(report).encode()
    path = tmp_path / "audit.json"
    path.write_bytes(data)
    monkeypatch.setattr(module, "AUDIT_SHA256", hashlib.sha256(data).hexdigest())
    return path


def test_numeric_axis_check_is_not_absolute_crs_or_edit_authority(source, audit):
    baseline = copy.deepcopy(source)
    geod = Geod(a=6378137, rf=298.257223563)
    east_lon, east_lat, _ = geod.fwd(106.32, 29.52, 90, 100)
    result = measure_points(source, [[106.32, 29.52], [east_lon, east_lat]], audit_path=audit)
    assert result["length"] == pytest.approx(100, abs=1e-7)
    assert result["frame"]["mode"] == "declared-local-plane"
    assert result["unit"] == "m"
    assert result["display_label"] == "按源声明计算的局部距离（m）"
    assert not result["absolute_crs_verified"] and not result["production_authority"]
    assert not result["frame"]["metric_edit_allowed"]
    assert source == baseline
    result["frame"]["scope_bounds"][0] = 0
    assert source == baseline


@pytest.mark.parametrize("option,reason", [(None, "audit-not-selected"), ("absent", "audit-unavailable")])
def test_without_bound_audit_degree_stays_raw_and_mm_is_not_applied(source, option, reason):
    result = measure_points(source, [[106.32, 29.52], [106.321, 29.52]], audit_path=option)
    assert result["unit"] == "degree"
    assert result["length"] == pytest.approx(0.001)
    assert result["frame"]["reason_code"] == reason
    assert result["frame"]["pipeline"] is None
    assert not result["frame"]["metric_available"]


def test_missing_unit_stays_unknown_even_if_profile_width_is_mm(source):
    source["frame"]["coordinate_unit"] = "unknown"
    _bind(source)
    result = measure_points(source, [[0, 0], [3, 4]], audit_path=None)
    assert result["length"] == 5
    assert result["unit"] == "unknown"
    assert not result["frame"]["metric_available"]


def test_audit_modification_is_not_silently_trusted(source, audit):
    audit.write_bytes(audit.read_bytes() + b"\n")
    result = measurement_capability(source, audit_path=audit)
    assert result["mode"] == "raw-coordinate"
    assert result["reason_code"] == "audit-binding-mismatch"


@pytest.mark.parametrize("change,expected", [
    ("source", "audit-source-mismatch"), ("frame", "audit-frame-mismatch"),
    ("declaration", "audit-declaration-mismatch"), ("bounds", "invalid-source-bounds"),
])
def test_mismatching_source_context_does_not_reuse_audit(source, audit, change, expected):
    if change == "source":
        source["source_files"][0]["sha256"] = "c" * 64
    elif change == "frame":
        source["frame"]["coordinate_unit"] = "m"
    elif change == "declaration":
        source["frame"]["unit_sources"][0]["sha256"] = "d" * 64
    else:
        source["bounds"] = [0, 0, 200, 100]
    _bind(source)
    assert measurement_capability(source, audit_path=audit)["reason_code"] == expected


def test_drifted_geometry_rejected_but_locator_relocation_does_not_change_frame(source, audit):
    frame = measurement_capability(source, audit_path=audit)
    source["locator"]["source_dir"] = "relocated"
    assert measurement_capability(source, audit_path=audit) == frame
    source["objects"][0]["points"][0][0] += 0.0001
    with pytest.raises(MeasurementRejected) as error:
        measure_object(source, "line:part0", audit_path=audit)
    assert error.value.code == "source-snapshot-changed"


def test_click_outside_source_scope_explicitly_uses_raw_coordinate_units(source, audit):
    result = measure_points(source, [[106.32, 29.52], [0, 0]], audit_path=audit)
    assert result["unit"] == "degree"
    assert result["frame"]["reason_code"] == "points-outside-source-scope"
    assert result["frame"]["pipeline"] is None
    assert "origin" not in result["frame"]


def test_polyline_sums_original_segments_without_z_or_cross_part_join(source, audit):
    baseline = copy.deepcopy(source)
    result = measure_object(source, "line:part0", audit_path=audit)
    points = source["objects"][0]["points"]
    pair_lengths = [measure_points(source, [a, b], audit_path=audit)["length"] for a, b in zip(points, points[1:])]
    assert result["length"] == pytest.approx(sum(pair_lengths))
    chord = measure_object(source, "line:part1", audit_path=audit)
    assert chord["length"] < result["length"]
    assert result["segment_count"] == 2
    assert result["identity"]["source_ref"]["part_index"] == 0
    assert not result["identity"]["parts_joined"]
    assert not result["identity"]["closure_added"]
    assert not result["height_included"]
    result["identity"]["source_ref"]["part_index"] = 100
    assert source == baseline


@pytest.mark.parametrize("points", [[], [[0, 0]], [[0, 0]] * 3, [[True, 0], [1, 2]],
                                     [[float("nan"), 0], [1, 2]], [[0, 0, 0], [1, 2]],
                                     [["0", 0], [1, 2]], [[10**400, 0], [1, 2]]])
def test_invalid_clicks_are_rejected(source, points):
    with pytest.raises(MeasurementRejected):
        measure_points(source, points, audit_path=None)


def test_line_identity_and_geometry_kind_are_checked(source):
    with pytest.raises(MeasurementRejected, match="不存在"):
        measure_object(source, "not-present", audit_path=None)
    source["objects"][0]["shape_type"] = 1
    _bind(source)
    with pytest.raises(MeasurementRejected, match="没有可测量"):
        measure_object(source, "line:part0", audit_path=None)


def test_large_coordinate_difference_is_not_reported_as_infinite_length(source):
    with pytest.raises(MeasurementRejected) as error:
        measure_points(source, [[-1e308, 0], [1e308, 0]], audit_path=None)
    assert error.value.code == "measurement-overflow"
