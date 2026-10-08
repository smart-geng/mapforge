"""Evidence tiers must not collapse numerical consistency into CRS approval."""
from pathlib import Path

import numpy as np
import pytest
import shapefile

from mapforge.workbench.crs_audit import (
    _declarations, _lane_segments, _raw_map_points, audit_crs,
    compare_internal, control_input_contract, projection_check,
)


def _source(tmp_path):
    source = tmp_path / "shp_0222-0326"
    source.mkdir()
    with shapefile.Writer(str(source / "IBD_LANE_LINK"), shapeType=shapefile.POLYLINE) as writer:
        writer.field("LANE_PID", "C")
        writer.line([[[106.3, 29.5], [106.301, 29.5]], [[106.302, 29.5], [106.303, 29.5]]])
        writer.record("raw-lane")
    with shapefile.Writer(str(source / "IBD_VERSION"), shapeType=shapefile.NULL) as writer:
        writer.field("COORD_SYS", "C")
        writer.field("COORD_UNIT", "N")
        writer.null()
        writer.record("84", 2)
    (source / "IBD_LANE_LINK.prj").write_text('GEOGCS["custom",DATUM["D_WGS_1984",SPHEROID["custom",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]')
    maps = tmp_path / "v2x_map_xml"
    maps.mkdir()
    (maps / "map-fixture.xml").write_text('''<MessageFrame><mapFrame><nodes><Node><id><id>1</id></id><refPos><long>1063000000</long><lat>295000000</lat></refPos><inLinks><Link><name>west</name><lanes><Lane><laneID>1</laneID><points><RoadPoint><posOffset><offsetLL><position-LatLon><lon>1063005000</lon><lat>295000000</lat></position-LatLon></offsetLL></posOffset></RoadPoint></points></Lane></lanes></Link></inLinks></Node></nodes></mapFrame></MessageFrame>''')
    return source, maps


def test_projection_axis_units_and_inverse_are_only_numerical_evidence():
    points = np.array([[106.3, 29.5], [106.302, 29.501], [106.299, 29.499]])
    result = projection_check(points, 106.3, 29.5)
    assert result["status"] == "PASS"
    assert result["roundtrip_max_deg"] < 1e-9
    assert result["axis_test"]["east_100m_xy"][0] == pytest.approx(100, abs=1e-5)
    assert result["axis_test"]["north_100m_xy"][1] == pytest.approx(100, abs=1e-5)
    assert result["absolute_crs_verified"] is False
    assert "unitconvert" in result["pipeline"]
    assert result["network_enabled"] is False


def test_axis_swap_invalid_coordinates_and_empty_data_do_not_pass():
    assert projection_check(np.array([[29.5, 106.3]]), 106.3, 29.5)["status"] == "FAILED"
    assert projection_check(np.array([[np.nan, 29.5]]), 106.3, 29.5)["status"] == "FAILED"
    assert projection_check(np.empty((0, 2)), 106.3, 29.5)["status"] == "FAILED"


def test_multipart_does_not_invent_bridge_and_raw_map_points_keep_integer_identity(tmp_path):
    source, maps = _source(tmp_path)
    segments, refs = _lane_segments(source)
    assert segments.shape == (2, 2, 2)
    assert [r["part_index"] for r in refs] == [0, 1]
    raw = _raw_map_points(maps / "map-fixture.xml")
    assert raw["points"][0]["longitude_e7"] == 1063005000
    assert raw["points"][0]["kind"] == "lane"
    result = compare_internal(raw, segments, refs)
    assert result["hypotheses"]["H0_direct"]["max_m"] < 0.001
    assert result["source_corrections_applied"] is False
    assert result["absolute_crs_verified"] is False
    assert result["h0_points"][0]["nearest_source"]["lane_pid"] == "raw-lane"


def test_unsupported_map_coordinate_is_reported_not_silently_discarded(tmp_path):
    _, maps = _source(tmp_path)
    path = maps / "map-fixture.xml"
    path.write_text(path.read_text().replace("position-LatLon", "position-LL1"))
    result = _raw_map_points(path)
    assert result["points"] == []
    assert result["invalid"][0]["code"] == "unsupported-coordinate-branch"


def test_matching_claimed_crs_and_perfect_internal_match_still_block_absolute_release(tmp_path):
    source, maps = _source(tmp_path)
    report = audit_crs(source, maps, repo_root=tmp_path)
    assert report["declarations"]["status"] == "DECLARATIONS_CONSISTENT"
    assert report["projection"]["status"] == "PASS"
    assert report["relative_agreement"]["status"] == "H0_BEST_ALL_FILES"
    assert report["production_export_allowed"] is False
    assert report["production_crs_status"] == "UNVERIFIED"
    assert report["absolute_verification"]["verified_points"] == 0
    assert report["absolute_verification"]["status"] == "NOT_DEMONSTRATED"
    assert report["control_input_contract"]["acceptance"]["max_horizontal_residual_m"] is None


def test_missing_or_conflicting_declarations_do_not_pass(tmp_path):
    source, _ = _source(tmp_path)
    prj = source / "IBD_LANE_LINK.prj"
    prj.write_text("invalid WKT")
    report = _declarations(source)
    assert report["status"] == "REVIEW_REQUIRED"
    assert report["parse_failures"]
    prj.unlink()
    assert _declarations(source)["status"] == "REVIEW_REQUIRED"


def test_no_map_files_is_not_vacuous_agreement(tmp_path):
    source, maps = _source(tmp_path)
    (maps / "map-fixture.xml").unlink()
    report = audit_crs(source, maps, repo_root=tmp_path)
    assert report["relative_agreement"]["status"] == "REVIEW_REQUIRED"
    contract = control_input_contract()
    assert contract["authority"]["independence_from_source"] is None
    assert contract["reference"]["realization_or_epoch"] is None
