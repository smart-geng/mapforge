"""The one bounded compiler: real XML readback, constraints and refusals."""
import copy
import hashlib
import json
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from mapforge.workbench.boundary_probe import (
    SCHEMA, BoundaryEditRejected, _admit, compile_boundary_edit,
    prepare_boundary_edit, source_residuals,
)
from scripts.internal_edge_jets import states


@pytest.fixture
def example(tmp_path):
    root = ET.Element("OpenDRIVE")
    header = ET.SubElement(root, "header")
    ET.SubElement(header, "geoReference").text = "+proj=tmerc +lat_0=29 +lon_0=106"
    road = ET.SubElement(root, "road", id="12", length="120", junction="-1")
    plan = ET.SubElement(road, "planView")
    ET.SubElement(ET.SubElement(plan, "geometry", s="0", x="0", y="0", hdg="0", length="120"), "line")
    ls = ET.SubElement(road, "lanes")
    sec = ET.SubElement(ls, "laneSection", s="0")
    right = ET.SubElement(sec, "right")
    for lid, sid in [(-1, "one"), (-2, "two"), (-3, "three")]:
        lane = ET.SubElement(right, "lane", id=str(lid), type="driving")
        for start in (0, 20, 100):
            ET.SubElement(lane, "width", sOffset=str(start), a="3", b="0", c="0", d="0")
        ET.SubElement(lane, "userData", code="mapforge.source_lane", value=sid)
        ET.SubElement(lane, "userData", code="mapforge.provenance/v1", value=json.dumps({
            "eligibility": "comparable", "boundary_evidence_trusted": True,
            "policy_class": "shp.field-leg", "support_kind": "shp-field-centerline",
            "status": "TRANSFORMED", "travel_direction": "with_s"}))
    data = ET.tostring(root)
    source = tmp_path / "bound-source.txt"
    source.write_bytes(b"immutable fixture boundary data")
    knots = [20., 40., 60., 80., 100.]
    binding = {"schema": SCHEMA + "/source", "baseline_sha256": hashlib.sha256(data).hexdigest(),
               "road_id": "12", "inner_lane_id": -1, "knots": knots,
               "identities": [{"section_s": 0., "source_lane_ids": ["one", "two"]}],
               "input_files_sha256": {str(source): hashlib.sha256(source.read_bytes()).hexdigest()},
               "source_rows": [{"fixture": True}],
               "local_polylines": {"shared": [[0, -3.02], [120, -3.02]],
                                   "fixed_inner": [[0, 0], [120, 0]],
                                   "fixed_outer": [[0, -6], [120, -6]]}}
    return data, binding, knots, source


def prep(example):
    data, binding, knots, _ = example
    return prepare_boundary_edit(data, binding, "12", -1, knots)


def test_zero_is_original_bytes_and_no_new_records(example):
    candidate, evidence = compile_boundary_edit(prep(example), 0.)
    assert candidate == example[0]
    assert evidence["complexity_before"] == evidence["complexity_after"]
    assert evidence["changed_width_records"] == 0
    assert evidence["delivery"] == "BLOCKED"


def test_shared_edit_moves_both_centers_and_only_shared_edge(example):
    prepared = prep(example)
    candidate, proof = compile_boundary_edit(prepared, -.02)
    old = ET.fromstring(example[0]).find("road")
    actual = ET.fromstring(candidate).find("road")
    for s in np.linspace(0, 120, 241):
        a, b = states(old, -1, float(s), False), states(actual, -1, float(s), False)
        c, d = states(old, -2, float(s), False), states(actual, -2, float(s), False)
        assert b[0] == pytest.approx(a[0], abs=1e-11)
        assert d[1] == pytest.approx(c[1], abs=1e-11)
        assert b[1] == pytest.approx(d[0], abs=1e-11)
        assert np.array(states(old, -3, float(s), False)) == pytest.approx(
            np.array(states(actual, -3, float(s), False)), abs=1e-11)
        if s <= 20 or s >= 100:
            assert np.array(a) == pytest.approx(np.array(b), abs=1e-11)
    assert states(actual, -1, 60, False)[1][1] == pytest.approx(-3.02)
    assert proof["complexity_after"]["width"] - proof["complexity_before"]["width"] == 6
    assert proof["complexity_after"]["geometry"] == 1
    assert proof["endpoint_world_jet_max_error"] < 1e-12
    for item in ("shared", "inner_lane_center", "outer_lane_center"):
        assert proof["source_after"]["items"][item]["mean_m"] < proof["source_before"]["items"][item]["mean_m"]


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), .101, -.101, "0.01"])
def test_target_rejected_before_any_write(example, value):
    with pytest.raises(BoundaryEditRejected):
        compile_boundary_edit(prep(example), value)


def test_stale_source_and_baseline_rejected(example):
    data, binding, knots, source = example
    with pytest.raises(BoundaryEditRejected, match="不一致"):
        prepare_boundary_edit(data + b"\n", binding, "12", -1, knots)
    prepared = prep(example)
    source.write_bytes(b"changed")
    with pytest.raises(BoundaryEditRejected, match="失效"):
        compile_boundary_edit(prepared, -.01)


def test_transition_role_is_not_ordinary_even_with_single_line(example):
    data, _, knots, _ = example
    root = ET.fromstring(data)
    item = root.find("road/lanes/laneSection/right/lane/userData[@code='mapforge.provenance/v1']")
    p = json.loads(item.get("value"))
    p.update(eligibility="excluded", exclusion_code="lane-transition-taper", geometry_adjustment="short-transition-stretched")
    item.set("value", json.dumps(p))
    with pytest.raises(BoundaryEditRejected, match="事件"):
        _admit(ET.tostring(root), "12", -1, knots)


def test_new_dense_width_segments_rejected(example):
    data, _, _, _ = example
    # 21 would split the original [20,100] interval into a 1m interval.
    with pytest.raises(BoundaryEditRejected, match="不足 6m"):
        _admit(data, "12", -1, [21., 40., 60., 80., 100.])


def test_negative_width_analytic_minimum_rejected(example):
    data, binding, knots, _ = example
    root = ET.fromstring(data)
    for width in root.findall("road/lanes/laneSection/right/lane[@id='-2']/width"):
        width.set("a", ".001")
    data = ET.tostring(root)
    binding = copy.deepcopy(binding)
    binding["baseline_sha256"] = hashlib.sha256(data).hexdigest()
    prepared = prepare_boundary_edit(data, binding, "12", -1, knots)
    with pytest.raises(BoundaryEditRejected, match="负宽"):
        compile_boundary_edit(prepared, -.02)


def test_source_partial_or_multivalued_cannot_enable_tool(example):
    data, binding, knots, _ = example
    b = copy.deepcopy(binding)
    b["local_polylines"]["shared"] = [[30, -3], [120, -3]]
    with pytest.raises(BoundaryEditRejected, match="覆盖"):
        prepare_boundary_edit(data, b, "12", -1, knots)
    b["local_polylines"]["shared"] = [[0, -3], [80, -3], [70, -3], [120, -3]]
    with pytest.raises(BoundaryEditRejected, match="非单值"):
        prepare_boundary_edit(data, b, "12", -1, knots)


def test_residual_is_same_bound_source_not_nearest_neighbor(example):
    p = prep(example)
    residual = source_residuals(example[0], p)
    assert residual["items"]["shared"]["max_m"] == pytest.approx(.02)
    assert residual["items"]["inner_lane_center"]["max_m"] == pytest.approx(.01)
