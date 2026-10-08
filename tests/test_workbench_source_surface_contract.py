"""Independent source-fidelity and gate boundaries for auxiliary representations.

These tests use written XML and the unchanged G11 policy.  Source material is
not disposable merely because its area or longitudinal span is small.
"""
from copy import deepcopy
from itertools import count
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from shapely import wkb
from shapely.geometry import MultiPolygon, Point, Polygon, box

from mapforge.adapters.opendrive import writer as W
from mapforge.validate import g11
from mapforge.workbench import source_surface_cells as C
from mapforge.workbench import source_surface_tracks as T


POLICY = Path(__file__).resolve().parents[1] / "profiles/validation/g11-opendrive-v1.draft.yaml"


def write_surface(tmp_path, module, shape, *, status="TRANSFORMED"):
    doc = W.XodrDoc("independent surface contract")
    proof = module.append_polygon(doc, shape, 1, count(50), preferred_axis=(1, 0),
                                  provenance={"status": status, "source_refs": [
                                      {"layer": "original-boundary", "record_index": 17,
                                       "part_index": 0}]})
    path = tmp_path / "contract.xodr"
    doc.write(path)
    root = ET.parse(path).getroot()
    assert module.audit_written(root, proof)["passed"]
    return root, proof, path


@pytest.mark.parametrize("status", ["TRANSFORMED", "INFERRED", "SHORT_SEGMENT_EXCEPTION"])
@pytest.mark.parametrize("module", [C, T], ids=["cells", "tracks"])
def test_true_isolated_short_material_remains_subject_to_existing_g11(tmp_path, status, module):
    root, proof, path = write_surface(tmp_path, module, box(0, 0, .004752184206, 1), status=status)
    actual = module.written_geometry(root, proof["road_ids"])
    assert actual.area == pytest.approx(.004752184206, abs=1e-12)
    assert float(root.find("road").get("length")) == pytest.approx(.004752184206)
    result = g11.audit_file(path, POLICY)
    failures = [x for x in result["groups"]["G11-A"]["issues"]
                if x["code"] == "degenerate_primitive"]
    assert len(failures) == 1
    assert failures[0]["severity"] == "FAIL"
    assert result["status"] == "FAIL"


@pytest.mark.parametrize("module", [C, T], ids=["cells", "tracks"])
def test_source_component_smaller_than_global_area_budget_cannot_be_deleted(tmp_path, module):
    tiny = box(20, 0, 20.004752184206, 1)
    source = MultiPolygon([box(0, 0, 10, 1), tiny])
    root, proof, _ = write_surface(tmp_path, module, source)
    assert tiny.area < .01
    short = min(root.findall("road"), key=lambda road: float(road.get("length")))
    root.remove(short)
    # The remaining material is source-supported, but it is not a faithful
    # representation of the complete source family.
    result = module.audit_written(root, proof)
    assert result["status"] == "FAIL"
    assert not result["passed"]


@pytest.mark.parametrize("module", [C, T], ids=["cells", "tracks"])
def test_zero_width_extension_cannot_turn_short_source_into_long_road(tmp_path, module):
    root, proof, _ = write_surface(tmp_path, module, box(0, 0, .4, 1))
    road = root.find("road")
    road.set("length", "1.4")
    road.find("planView/geometry").set("length", "1.4")
    lane = road.find("lanes/laneSection/right/lane")
    ET.SubElement(lane, "width", sOffset="0.4", a="0", b="0", c="0", d="0")
    offset = road.find("lanes/laneOffset")
    ET.SubElement(road.find("lanes"), "laneOffset", s="0.4", a=offset.get("a"),
                  b="0", c="0", d="0")
    result = module.audit_written(root, proof)
    assert result["status"] == "FAIL"
    assert not result["passed"]


def test_real_parent_partition_preserves_short_fork_and_its_empty_gap(tmp_path):
    """A longer carrier is legitimate only because it carries real parent area."""
    split = 10 - .004752184206
    source = Polygon([(0, 0), (10, 0), (10, 1), (split, 2),
                      (10, 3), (10, 4), (0, 4)])
    root, proof, path = write_surface(tmp_path, T, source)
    actual = T.written_geometry(root, proof["road_ids"])
    assert len(root.findall("road")) == 2
    assert all(float(r.get("length")) == 10 for r in root.findall("road"))
    assert proof["complexity"]["minimum_width_event_span_m"] < .005
    assert proof["complexity"]["endpoint_padding_m"] == 0
    band = proof["numeric_serialization_band_m"]
    assert actual.difference(source.buffer(band)).is_empty
    assert source.difference(actual.buffer(band)).is_empty
    assert not actual.covers(Point((split + 10) / 2, 2))
    assert actual.covers(Point((split + 10) / 2, .5))
    assert actual.covers(Point((split + 10) / 2, 3.5))
    audit = T.audit_written(root, proof)
    assert audit["derived_seams_are_source_boundaries"] is False
    result = g11.audit_file(path, POLICY)
    assert not [x for x in result["groups"]["G11-A"]["issues"]
                if x["code"] == "degenerate_primitive"]


@pytest.mark.parametrize("enclosed", [False, True], ids=["side-gap", "hole"])
def test_real_118_micrometre_gap_cannot_be_normalized_away(tmp_path, enclosed):
    # This is a physical source feature, many orders above projection roundoff.
    # Its area is below .01 m2, so a global area budget cannot protect it alone.
    if enclosed:
        source = Polygon(box(0, 0, 10, 4).exterior.coords,
                         [[(3, 2), (5, 2-.000059), (7, 2), (5, 2+.000059)]])
        interior = Point(5, 2)
    else:
        source = Polygon([(0, 0), (10, 0), (10, 2-.000059), (5, 2),
                          (10, 2+.000059), (10, 4), (0, 4)])
        interior = Point(7.5, 2)
    original = source.wkb
    root, proof, _ = write_surface(tmp_path, T, source)
    assert source.wkb == original
    assert bytes.fromhex(proof["source_geometry_wkb_hex"]) == original
    assert wkb.loads(bytes.fromhex(proof["source_geometry_wkb_hex"])).equals_exact(source, 0)
    actual = T.written_geometry(root, proof["road_ids"])
    assert not actual.covers(interior)
    assert actual.geom_type == "Polygon"
    assert len(actual.interiors) == int(enclosed)
    band = proof["numeric_serialization_band_m"]
    assert band < 1e-7
    assert actual.difference(source.buffer(band)).is_empty
    assert source.difference(actual.buffer(band)).is_empty


@pytest.mark.parametrize("kind", ["component", "hole"])
def test_roundoff_sized_ring_cannot_disappear_from_source(kind):
    # A ring that cannot survive normalization must reject the operation; a
    # representational ambiguity grants no permission to erase that ring.
    if kind == "component":
        source = MultiPolygon([box(0, 0, 10, 4), box(20, 0, 20+1e-14, 1)])
    else:
        source = Polygon(box(0, 0, 10, 4).exterior.coords,
                         [box(3, 2, 3+1e-14, 3).exterior.coords])
    assert source.is_valid
    original = source.wkb
    doc = W.XodrDoc("cannot drop a source ring")
    retained = W.Road(10)
    doc.add_road(retained)
    with pytest.raises(C.SurfaceCellError):
        T.append_polygon(doc, source, 1, count(50), preferred_axis=(1, 0))
    assert source.wkb == original
    assert doc.roads == [retained]


def test_adjacent_ulp_chain_requires_one_common_two_axis_witness_per_group():
    step = math.ulp(15.)
    source = Polygon([(0, 0), *[(15 + i*step, 0) for i in range(200)],
                      (20, 0), (20, 5), (0, 5)])
    normalized, proof = T._normalize_adjacent_vertices(source, (1, 0))
    mappings = {(v["component_index"], v["ring_index"], v["vertex_index"]): v
                for v in proof["source_vertex_mapping"]}
    assert len(mappings) == len(source.exterior.coords)-1
    chain = [mappings[(0, 0, i)] for i in range(1, 201)]
    # Pairwise overlapping neighbours form one long transitive chain. It must
    # NOT become one normalization group because the whole intersection is empty.
    assert len({v["representative_vertex_index"] for v in chain}) > 1
    for group in proof["changed_groups"]:
        key = (group["component_index"], group["ring_index"])
        members = [mappings[(*key, i)] for i in group["vertex_indices"]]
        representative = mappings[(*key, group["representative_vertex_index"])]
        for axis in (0, 1):
            lo = max(math.nextafter(v["original_local_st_m"][axis] -
                                   v["projection_roundoff_st_m"][axis], -math.inf) for v in members)
            hi = min(math.nextafter(v["original_local_st_m"][axis] +
                                   v["projection_roundoff_st_m"][axis], math.inf) for v in members)
            assert lo <= hi
            assert lo <= representative["original_local_st_m"][axis] <= hi
            assert group["common_projection_roundoff_intervals_st_m"][axis] == [lo, hi]
        assert all(v["world_shift_m"] <= v["derived_world_shift_bound_m"] for v in members)
    assert normalized.is_valid
    assert normalized.symmetric_difference(source).area == 0
    assert proof["deleted_vertices"] == 0


def test_same_station_does_not_merge_real_transverse_side_edge():
    source = Polygon([(0, 0), (10, 0), (10, 2), (10, 2+.000118), (10, 4), (0, 4)])
    normalized, proof = T._normalize_adjacent_vertices(source, (1, 0))
    assert normalized.wkb == source.wkb
    assert proof["changed_groups"] == []
    assert proof["changed_vertex_count"] == 0
    assert len(proof["source_vertex_mapping"]) == len(source.exterior.coords)-1


@pytest.mark.parametrize("change", ["band", "normalized-shape", "mapping", "deleted-vertices"])
def test_normalization_evidence_cannot_grant_itself_more_source_error(tmp_path, change):
    source = Polygon([(0, 0), (10, 0), (10, 4), (0, 4), (0, 4-math.ulp(4.))])
    root, original, _ = write_surface(tmp_path, T, source)
    proof = deepcopy(original)
    normalization = proof["source_numerical_normalization"]
    assert normalization["changed_vertex_count"] == 1
    assert bytes.fromhex(proof["source_geometry_wkb_hex"]) == source.wkb
    if change == "band":
        normalization["maximum_derived_source_shift_bound_m"] = 10.
        proof["numeric_serialization_band_m"] += 10.
    elif change == "normalized-shape":
        substituted = box(0, 0, 10, 5)
        normalization["normalized_geometry_wkb_hex"] = substituted.wkb.hex()
        normalization["normalized_geometry_sha256"] = C._sha(substituted.wkb)
    elif change == "mapping":
        row = next(v for v in normalization["source_vertex_mapping"] if v["changed"])
        row["original_xy"] = row["normalized_xy"]
        row["world_shift_m"] = 0.
    else:
        normalization["deleted_vertices"] = 1
    result = T.audit_written(root, proof)
    assert result["status"] == "FAIL"
    assert result["error_code"] == "track-evidence-does-not-match-source"
    assert result["source_partition_verified"] is False
