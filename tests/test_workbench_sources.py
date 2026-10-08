"""Source workspaces retain raw identities and open independently of conversion."""
import copy
import json
import shutil
from pathlib import Path

import pytest
import shapefile
import yaml

from mapforge.workbench.sources import (SourceCatalog, build_source_snapshot,
                                       verify_source_snapshot)

ROOT = Path(__file__).resolve().parents[1]


def _layer(root, name, fields, rows, shape_type=shapefile.POLYLINE):
    with shapefile.Writer(str(root / name), shapeType=shape_type) as writer:
        for name in fields:
            writer.field(name, "C", size=80)
        for attrs, parts in rows:
            if shape_type == shapefile.NULL:
                writer.null()
            else:
                writer.line(parts)
            writer.record(*attrs)


@pytest.fixture
def source(tmp_path):
    raw = tmp_path / "source"
    raw.mkdir()
    _layer(raw, "junction", ["PID", "ENTER", "LEAVE", "NAME"], [
        (["j1", "r1;missing-road", "r2", "test"], [[[0, 0], [20, 0], [20, 20], [0, 0]]]),
        (["j2", "r3", "", "other"], [[[1000, 0], [1020, 20]]]),
    ])
    _layer(raw, "road", ["PID"], [
        (["r1"], [[[0, 0], [0, 20]]]), (["r2"], [[[20, 0], [20, 20]]]),
        (["r3"], [[[1000, 0], [1000, 20]]]),
    ])
    _layer(raw, "lane", ["PID", "ROAD", "UNFAMILIAR"], [
        (["l1", "r1", "keep-me"], [[[0, 0], [0, 5]], [[0, 10], [0, 15]]]),
        (["l1", "r1", "duplicate"], [[[1, 0], [1, 5]]]),
        (["l2", "r2", ""], [[[20, 0], [20, 20]]]),
        (["l3", "r3", "outside"], [[[1000, 0], [1000, 20]]]),
    ])
    _layer(raw, "boundary", ["PID"], [(["b1"], [[[-1, 0], [-1, 20]]])])
    _layer(raw, "rel", ["LANE", "BOUNDARY", "SIDE"], [
        (["l1", "b1", "99"], []), (["l1", "missing-boundary", "1"], []),
    ], shapefile.NULL)
    _layer(raw, "topo", ["FROM", "TO"], [(["l1", "l2"], []),
                                               (["l1", "missing-lane"], [])], shapefile.NULL)
    (raw / "vendor-extra.bin").write_bytes(b"unknown extension retained")
    profile = tmp_path / "profile.yaml"
    profile.write_text(yaml.safe_dump({
        "profile": "fixture", "units": {"length": "mystery"}, "layers": {
            "junction": {"file": "junction", "fields": {"id": "PID", "enter_roads": "ENTER", "leave_roads": "LEAVE", "name": "NAME"}},
            "road": {"file": "road", "fields": {"id": "PID"}},
            "lane": {"file": "lane", "fields": {"id": "PID", "road": "ROAD"}},
            "boundary": {"file": "boundary", "fields": {"id": "PID"}},
            "lane_boundary_rel": {"file": "rel", "fields": {"lane": "LANE", "boundary": "BOUNDARY", "side": "SIDE"}, "side_values": {"left": 1, "right": 2}},
            "topo": {"file": "topo", "fields": {"from": "FROM", "to": "TO"}},
        }}), encoding="utf-8")
    return raw, profile


def test_duplicate_multipart_unknown_fields_and_missing_refs_are_retained(source):
    snapshot = build_source_snapshot(*source, "j1")
    lanes = [o for o in snapshot["objects"] if o["role"] == "lane" and o["business_id"] == "l1"]
    assert len(lanes) == 3  # two parts of one row plus the duplicate row
    assert len({o["id"] for o in snapshot["objects"]}) == len(snapshot["objects"])
    assert {o["source_ref"]["record_index"] for o in lanes} == {0, 1}
    assert lanes[0]["raw_attributes"]["UNFAMILIAR"] == "keep-me"
    assert all(len(o["points"]) == 2 for o in lanes)
    assert all(o["business_id"] != "l3" for o in snapshot["objects"])
    assert "vendor-extra.bin" in snapshot["scope"]["unmapped_files"]
    assert any(f["relative_path"] == "vendor-extra.bin" for f in snapshot["source_files"])
    codes = {i["code"] for i in snapshot["issues"]}
    assert {"duplicate-business-id", "missing-source-reference", "unknown-boundary-side"} <= codes
    missing = {r["target_business_id"] for r in snapshot["references"] if r["status"] == "missing"}
    assert {"missing-road", "missing-boundary", "missing-lane"} <= missing
    assert snapshot["frame"]["coordinate_unit"] == "unknown"
    assert snapshot["frame"]["attribute_length_unit"] == "mystery"
    assert snapshot["capabilities"]["metric_edit"] is False
    assert snapshot["capabilities"]["production_export"] is False
    json.dumps(snapshot, allow_nan=False)


def test_path_relocation_access_order_and_return_mutation_do_not_change_identity(source, tmp_path):
    raw, profile = source
    catalog = SourceCatalog(raw, profile)
    first = catalog.snapshot("j1")
    catalog.snapshot("j2")
    assert catalog.snapshot("j1") == first
    for rows in catalog.rows.values():
        rows.reverse()
    assert catalog.snapshot("j1") == first
    altered = copy.deepcopy(first)
    altered["objects"][0]["points"][0][0] = 9999
    assert catalog.snapshot("j1") == first
    moved = tmp_path / "moved"
    shutil.copytree(raw, moved)
    moved_profile = tmp_path / "relocated.yaml"
    shutil.copy2(profile, moved_profile)
    second = build_source_snapshot(moved, moved_profile, "j1")
    assert second["locator"] != first["locator"]
    assert second["snapshot_id"] == first["snapshot_id"]
    assert second["content_hash"] == first["content_hash"]
    assert second["objects"] == first["objects"]
    assert verify_source_snapshot(first, moved, moved_profile) == {"matches": True, "issues": []}


def test_drift_checks_cover_unknown_files_profile_and_cached_catalog(source):
    raw, profile = source
    catalog = SourceCatalog(raw, profile)
    snapshot = catalog.snapshot("j1")
    (raw / "vendor-extra.bin").write_bytes(b"changed")
    report = verify_source_snapshot(snapshot, raw, profile)
    assert report["matches"] is False
    assert {i["code"] for i in report["issues"]} == {"source-file-changed"}
    with pytest.raises(ValueError, match="源内容已变化"):
        catalog.snapshot("j1")
    profile.write_text(profile.read_text(encoding="utf-8") + "\n# revision\n", encoding="utf-8")
    assert "profile-changed" in {i["code"] for i in verify_source_snapshot(snapshot, raw, profile)["issues"]}


def test_declared_geographic_unit_does_not_become_width_unit_or_verified_crs(source):
    raw, profile = source
    (raw / "lane.prj").write_text('GEOGCS["local claim",DATUM["D_WGS_1984",SPHEROID["WGS84",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]')
    catalog = SourceCatalog(raw, profile)
    snapshot = catalog.snapshot("j1")
    assert snapshot["frame"]["coordinate_unit"] == "degree"
    assert snapshot["frame"]["absolute_crs_status"] == "unverified"
    assert not snapshot["frame"]["metric_edit_allowed"]
    assert snapshot["frame"]["origin"] == [0, 0]
    with pytest.raises(ValueError, match="不一致"):
        catalog.snapshot("j1", frame={"coordinate_unit": "m", "evidence": "manual"})
    with pytest.raises(ValueError, match="evidence"):
        catalog.snapshot("j1", frame={"coordinate_unit": "degree"})


def test_unreadable_layer_is_an_issue_not_a_generated_replacement(source):
    raw, profile = source
    (raw / "boundary.shp").write_bytes(b"bad")
    snapshot = build_source_snapshot(raw, profile, "j1")
    assert not any(o["role"] == "boundary" for o in snapshot["objects"])
    assert any(i["code"] == "source-layer-unreadable" for i in snapshot["issues"])


def test_profile_layer_must_stay_within_registered_source(source):
    raw, profile = source
    data = yaml.safe_load(profile.read_text(encoding="utf-8"))
    data["layers"]["lane"]["file"] = "../outside"
    profile.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(ValueError, match="越出"):
        SourceCatalog(raw, profile)


def test_z_and_measure_values_remain_json_safe_and_bound_to_original_parts(source):
    raw, profile = source
    with shapefile.Writer(str(raw / "lane"), shapeType=shapefile.POLYLINEZ) as writer:
        writer.field("PID", "C")
        writer.field("ROAD", "C")
        writer.linez([[[0, 0, 15, 2], [0, 5, 16, 3]], [[0, 10, 17, 4], [0, 15, 18, 5]]])
        writer.record("l1", "r1")
    snapshot = build_source_snapshot(raw, profile, "j1")
    lanes = [o for o in snapshot["objects"] if o["role"] == "lane"]
    assert len(lanes) == 2
    assert lanes[0]["raw_z"] == [15, 16, 17, 18]
    assert lanes[1]["raw_m"] == [2, 3, 4, 5]
    assert lanes[1]["source_ref"]["part_index"] == 1
    json.dumps(snapshot, allow_nan=False)


def test_selected_via_lanes_keep_internal_topology_without_expanding_next_hop(source):
    raw, profile = source
    _layer(raw, "lane", ["PID", "ROAD", "UNFAMILIAR"], [
        (["l1", "r1", "approach"], [[[0, 0], [0, 5]]]),
        (["l2", "r2", "departure"], [[[10, 10], [20, 20]]]),
        (["via1", "r3", "via"], [[[0, 5], [5, 5]]]),
        (["via2", "r3", "via"], [[[5, 5], [10, 10]]]),
        (["outside", "r3", "must not expand"], [[[10, 10], [100, 100]]]),
    ])
    _layer(raw, "topo", ["FROM", "TO"], [
        (["l1", "via1"], []), (["via1", "via2"], []), (["via1", "via2"], []),
        (["via2", "l2"], []), (["via2", "outside"], []),
    ], shapefile.NULL)
    snapshot = build_source_snapshot(raw, profile, "j1")
    lanes = {o["business_id"] for o in snapshot["objects"] if o["role"] == "lane"}
    assert lanes == {"l1", "l2", "via1", "via2"}
    topo = [o for o in snapshot["objects"] if o["role"] == "topo"]
    assert {o["source_ref"]["record_index"] for o in topo} == {0, 1, 2, 3}
    assert sum(o["raw_attributes"] == {"FROM": "via1", "TO": "via2"} for o in topo) == 2
    assert not any(r["target_business_id"] == "outside" for r in snapshot["references"])
    via_roads = [r for r in snapshot["references"] if r["field"] == "road" and r["target_business_id"] == "r3"]
    assert via_roads and all(r["target_scope"] == "outside-view" for r in via_roads)


def test_six_failed_and_all_sixteen_real_sources_open_without_xodr():
    if not (ROOT / "shp_0222-0326" / "IBD_LANE_LINK.shp").exists():
        pytest.skip("original SHP delivery not installed")
    selection = yaml.safe_load((ROOT / "profiles/validation/generalization-set-v1.yaml").read_text(encoding="utf-8"))
    catalog = SourceCatalog(ROOT / selection["source"], selection["profile"])
    assert len(catalog.list_junctions()) >= 16
    ids = {str(j["pid"]) for j in selection["junctions"]}
    failed = {"2023063015350832057", "2023062110304177600", "2023070118182048481",
              "2023062913464138453", "2023072914272536839", "2023081110220126408"}
    assert failed <= ids
    fingerprints = {}
    for junction in selection["junctions"]:
        snapshot = catalog.snapshot(junction["pid"])
        assert snapshot["objects"] and snapshot["bounds"]
        assert any(o["role"] == "junction" for o in snapshot["objects"])
        assert any(o["role"] == "lane" for o in snapshot["objects"])
        assert snapshot["frame"]["coordinate_unit"] == "degree"
        assert snapshot["frame"]["version_evidence"]
        assert snapshot["capabilities"]["compile"] is False
        assert not any(key in snapshot for key in ("xodr", "candidate"))
        assert len(snapshot["objects"]) < 10000
        selected_lanes = set(snapshot["scope"]["lane_ids"])
        expected_internal = {row["record_index"] for row in catalog.rows["topo"]
                             if all(catalog._value("topo", row, side) in selected_lanes
                                    for side in ("from", "to"))}
        actual_topo = {o["source_ref"]["record_index"] for o in snapshot["objects"] if o["role"] == "topo"}
        assert expected_internal <= actual_topo
        if junction["pid"] == "2023071109582525264":
            assert {6382, 6515, 6516} <= actual_topo
        if junction["pid"] == "2023081110220126408":
            assert {9680, 9683} <= actual_topo
        fingerprints[junction["pid"]] = snapshot["content_hash"]
    for junction in reversed(selection["junctions"]):
        assert catalog.snapshot(junction["pid"])["content_hash"] == fingerprints[junction["pid"]]
