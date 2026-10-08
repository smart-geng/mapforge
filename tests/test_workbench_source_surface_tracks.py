"""Real-area parent partitions, source topology, and serialized track proofs."""
from copy import deepcopy
from itertools import count
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from shapely import wkb
from shapely.affinity import affine_transform
from shapely.geometry import MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.validate import g11
from mapforge.workbench import source_surface_cells as C
from mapforge.workbench import source_surface_tracks as T


def fork_shape():
    return Polygon([(0, 0), (10, 0), (10, 3), (5, 5), (10, 7), (10, 10), (0, 10)])


def hole_shape():
    return Polygon(box(0, 0, 12, 10).exterior.coords,
                   [[(3, 5), (5, 3), (8, 5), (5, 7)]])


def write_actual(tmp_path, shape, axis=(1, 0), lane_type="restricted", provenance=None):
    doc = W.XodrDoc("source tracks")
    proof = T.append_polygon(doc, shape, 1, count(50), preferred_axis=axis,
                             lane_type=lane_type, provenance=provenance)
    path = tmp_path / "tracks.xodr"
    doc.write(path)
    root = ET.parse(path).getroot()
    # Report files undergo a real JSON roundtrip in workbench evidence.
    proof = json.loads(json.dumps(proof))
    audit = T.audit_written(root, proof)
    assert audit["passed"], audit
    assert audit["source_partition_verified"]
    assert not audit["whole_map_gates_evaluated"]
    actual = T.written_geometry(root, proof["road_ids"])
    assert actual.symmetric_difference(shape).area < 1e-5
    partition = proof["track_partition"]
    assert not partition["minimum_length_or_gate_policy_used"]
    assert not partition["axis_search_performed"]
    assert partition["endpoint_padding_m"] == 0
    assert not partition["derived_seams_are_source_boundaries"]
    assert proof["production_qualification"] == "NOT_EVALUATED"
    axis = np.asarray(proof["sweep_axis"])
    origin = np.asarray(proof["sweep_origin_xy"])
    normal = np.array([-axis[1], axis[0]])
    supported = []
    for row in proof["cells"]:
        assert row["source_end_s_m"] > row["source_start_s_m"]
        station = row["source_start_s_m"]
        for piece in row["source_pieces"]:
            assert piece["start_s_m"] == station
            assert piece["end_s_m"] > station
            assert min(piece["high0"]-piece["low0"], piece["high1"]-piece["low1"]) >= 0
            quad = C._quad(origin, axis, normal, station, piece["end_s_m"],
                           piece["low0"], piece["high0"], piece["low1"], piece["high1"])
            assert quad.area > 0  # No zero-width carrier prefix, suffix or bridge.
            supported.append(quad)
            station = piece["end_s_m"]
        assert station == row["source_end_s_m"]
        road = next(r for r in root.findall("road") if r.get("id") == row["road_id"])
        assert len(road.findall("planView/geometry")) == 1
        assert road.find("planView/geometry/line") is not None
        assert road.find("link") is None
        assert len(road.findall("lanes/laneSection")) == 1
        assert road.find("lanes/laneSection/right/lane").get("type") == lane_type
    union = unary_union(supported)
    assert union.symmetric_difference(shape).area < 1e-10
    assert abs(sum(q.area for q in supported)-union.area) < 1e-10
    return root, proof, actual


def test_single_source_cell_stays_one_full_real_track(tmp_path):
    _, proof, _ = write_actual(tmp_path, box(0, 0, 20, 4))
    assert proof["complexity"]["line_roads"] == 1
    assert proof["track_partition"]["events"] == []


def test_point_fork_partitions_whole_parent_and_keeps_both_arms(tmp_path):
    _, proof, actual = write_actual(tmp_path, fork_shape())
    assert proof["complexity"]["source_cells_before_partition"] == 3
    assert proof["complexity"]["line_roads"] == 2
    assert [r["length_m"] for r in proof["cells"]] == [10, 10]
    event = proof["track_partition"]["events"][0]
    assert event["kind"] == "point-fork"
    assert event["partition_fraction"] == .5
    assert event["source_vertex_refs"]
    assert actual.intersection(box(8, 4.5, 9, 5.5)).is_empty


def test_unequal_short_branches_use_only_their_own_real_parent_shares(tmp_path):
    shape = Polygon([(0, 0), (5.49, 0), (5.49, 3), (5, 5),
                     (5.17, 7), (5.17, 10), (0, 10)])
    _, proof, _ = write_actual(tmp_path, shape)
    assert sorted(r["length_m"] for r in proof["cells"]) == pytest.approx([5.17, 5.49])
    assert len(proof["track_partition"]["source_cell_coverage"][0]["owners"]) == 2
    assert proof["complexity"]["minimum_width_event_span_m"] == pytest.approx(.17)


def test_point_merge_reverses_parent_partition(tmp_path):
    shape = affine_transform(fork_shape(), [-1, 0, 0, 1, 10, 0])
    _, proof, _ = write_actual(tmp_path, shape)
    assert proof["track_partition"]["events"][0]["kind"] == "point-merge"
    assert [r["length_m"] for r in proof["cells"]] == [10, 10]


def test_point_fork_and_merge_preserve_hole_with_two_full_area_tracks(tmp_path):
    _, proof, actual = write_actual(tmp_path, hole_shape())
    assert proof["complexity"]["source_cells_before_partition"] == 4
    assert proof["complexity"]["line_roads"] == 2
    assert [e["kind"] for e in proof["track_partition"]["events"]] == ["point-fork", "point-merge"]
    assert not actual.covers(Point(5, 5))
    assert len(actual.interiors) == 1


@pytest.mark.parametrize("fraction", [.2, .7])
def test_parent_seam_uses_source_event_fraction_not_middle(fraction, tmp_path):
    q = fraction * 10
    shape = Polygon([(0, 0), (10, 0), (10, q-1), (5, q),
                     (10, q+1), (10, 10), (0, 10)])
    _, proof, _ = write_actual(tmp_path, shape)
    assert proof["track_partition"]["events"][0]["partition_fraction"] == fraction


def test_parent_partition_retains_exact_zero_width_source_tip():
    tip = -7.78432203086438
    source = {"source_cell_index": 2, "pieces": [{
        "start_s_m": 0., "end_s_m": 1., "low0": -10., "high0": -2.,
        "low1": tip, "high1": tip, "edges": [{"edge_index": 1}, {"edge_index": 2}]}]}
    fraction = .45182818034710004
    lower, _ = T._share(source, 0., fraction)
    upper, _ = T._share(source, fraction, 1.)
    assert lower[0]["high0"] == upper[0]["low0"]
    assert lower[0]["low1"] == lower[0]["high1"] == upper[0]["low1"] == upper[0]["high1"] == tip


@pytest.mark.parametrize("angle", [
    pytest.param(.73456789, marks=pytest.mark.xfail(strict=True, raises=C.SurfaceCellError,
        reason="Frozen cell sweep rejects valid rotated source at its binary64 coverage check; unresolved")),
    -1.2345, 2.7123,
])
def test_rotated_source_and_fixed_axis_roundtrip_without_axis_search(tmp_path, angle):
    c, s = math.cos(angle), math.sin(angle)
    shape = affine_transform(fork_shape(), [c, -s, s, c, 20.2, -10.7])
    _, proof, _ = write_actual(tmp_path, shape, (3*c, 3*s),
                               provenance={"source_refs": [{"record": 7}], "label": "源面"})
    assert proof["write_contract"]["preferred_axis"] == [3*c, 3*s]
    assert proof["numeric_serialization_band_m"] < 1e-6


def test_isolated_true_short_component_keeps_length_and_existing_g11_fail(tmp_path):
    short = .004752184206
    shape = MultiPolygon([box(0, 0, 10, 3), box(20, 0, 20+short, 2)])
    root, proof, _ = write_actual(tmp_path, shape)
    assert proof["source_components"] == 2
    assert sorted(r["length_m"] for r in proof["cells"]) == pytest.approx([short, 10])
    policy = g11.load_policy(Path(__file__).resolve().parents[1] / "profiles/validation/g11-opendrive-v1.draft.yaml")
    issues = g11._audit_a(root, policy)["issues"]
    failures = [i for i in issues if i["severity"] == "FAIL"]
    assert len(failures) == 1
    assert failures[0]["code"] == "degenerate_primitive"
    assert failures[0]["road_id"] == "51"


@pytest.mark.parametrize("shape,code", [
    (Polygon(box(0, 0, 12, 10).exterior.coords, [box(3, 3, 8, 7).exterior.coords]),
     "finite-gap-or-step-at-source-event"),
    (Polygon([(0, 0), (3, 0), (3, -2), (8, -2), (8, 4), (0, 4)]),
     "vertical-source-step-not-supported"),
    (Polygon(box(0, 0, 12, 12).exterior.coords,
             [[(3, 3), (5, 2), (8, 3), (5, 4)], [(3, 8), (5, 7), (8, 8), (5, 9)]]),
     "unsupported-source-cell-topology"),
])
def test_unsupported_topology_rejects_atomically(shape, code):
    doc = W.XodrDoc("unsupported")
    original = W.Road(10)
    doc.add_road(original)
    with pytest.raises(T.SurfaceTrackError, match=code):
        T.append_polygon(doc, shape, 1, count(50), preferred_axis=(1, 0))
    assert doc.roads == [original]


@pytest.mark.parametrize("change", ["shift", "drop", "negative-width", "duplicate-station", "routing"])
def test_actual_xml_changes_cannot_pass_source_proof(tmp_path, change):
    root, proof, _ = write_actual(tmp_path, fork_shape())
    road = root.find("road")
    if change == "shift":
        road.find("planView/geometry").set("y", ".1")
    elif change == "drop":
        root.remove(road)
    elif change == "negative-width":
        road.find("lanes/laneSection/right/lane/width").set("a", "-1")
    elif change == "duplicate-station":
        road.findall("lanes/laneSection/right/lane/width")[1].set("sOffset", "0")
    else:
        junction = ET.SubElement(root, "junction", id="1")
        ET.SubElement(junction, "connection", incomingRoad=road.get("id"), connectingRoad="100")
    assert not T.audit_written(root, proof)["passed"]


@pytest.mark.parametrize("change", ["band", "hash", "fraction", "cell", "complexity", "omitted-track", "duplicate-id"])
def test_report_cannot_redefine_expected_partition_or_serialization(tmp_path, change):
    root, proof, _ = write_actual(tmp_path, fork_shape())
    if change == "band":
        proof["numeric_serialization_band_m"] = 100
        for row in proof["cells"]:
            row["expected_max_vertex_serialization_error_m"] = 25
    elif change == "hash":
        road = root.find("road")
        road.find("planView/geometry").set("y", ".1")
        proof["cells"][0]["expected_xml_sha256"] = C._sha(C._canonical(road))
    elif change == "fraction":
        proof["track_partition"]["events"][0]["partition_fraction"] = .4
        proof["track_partition_sha256"] = T._json_hash(proof["track_partition"])
    elif change == "cell":
        proof["cells"][0]["source_pieces"][0]["low0"] += .01
    elif change == "complexity":
        proof["complexity"]["minimum_road_length_m"] = 100
    elif change == "omitted-track":
        proof["cells"].pop()
        proof["road_ids"].pop()
    else:
        proof["road_ids"][1] = proof["road_ids"][0]
    report = T.audit_written(root, proof)
    assert not report["passed"] and not report["source_partition_verified"]


def test_median_lane_and_provenance_survive_actual_serialization(tmp_path):
    root, proof, _ = write_actual(tmp_path, fork_shape(), lane_type="median",
                                  provenance={"status": "INFERRED", "source_lanes": ["immutable-source-id"]})
    for road in root.findall("road"):
        text = ET.tostring(road, encoding="unicode")
        assert "immutable-source-id" in text
        assert "representation-only" in text or "derived_seams_are_source_boundaries" in text
    assert proof["write_contract"]["lane_type"] == "median"


def test_id_exhaustion_collision_and_precision_collapse_leave_document_intact():
    doc = W.XodrDoc("atomic")
    original = W.Road(50)
    doc.add_road(original)
    for ids, code in [(iter([51]), "insufficient-road-ids"), (iter([50, 51]), "colliding-road-id")]:
        with pytest.raises(C.SurfaceCellError, match=code):
            T.append_polygon(doc, fork_shape(), 1, ids, preferred_axis=(1, 0))
    shape = Polygon([(0, 0), (10, 0), (10.000000001, .0001), (20, 0), (20, 5), (0, 5)])
    with pytest.raises(C.SurfaceCellError, match="source-events-collapse-at-writer-precision"):
        T.append_polygon(doc, shape, 1, count(51), preferred_axis=(1, 0))
    assert doc.roads == [original]


def test_empty_source_does_not_invent_a_carrier(tmp_path):
    _, proof, actual = write_actual(tmp_path, Polygon())
    assert actual.is_empty and proof["road_ids"] == []


def test_numeric_band_independent_of_mutable_xml_and_evidence(tmp_path):
    root, proof, _ = write_actual(tmp_path, fork_shape())
    altered = deepcopy(proof)
    road = root.find("road")
    road.find("planView/geometry").set("y", "10")
    altered["cells"][0]["expected_xml_sha256"] = C._sha(C._canonical(road))
    altered["cells"][0]["expected_max_vertex_serialization_error_m"] = 10
    altered["numeric_serialization_band_m"] = altered["numeric_arithmetic_roundoff_m"] + 40
    audit = T.audit_written(root, altered)
    assert audit["error_code"] == "track-evidence-does-not-match-source"
    assert not audit["passed"]


def test_real_median_one_ulp_closure_keeps_true_side_strip_and_every_vertex(tmp_path):
    # Fixed 0621 constructed median. Vertices 0 and 8 differ by one world-x
    # ulp, while vertex 7 is a genuine 0.118 mm side feature that must survive.
    shape = wkb.loads(bytes.fromhex(
        "0103000000010000000A0000005435815C1D4D2B400CD1A22EA6D0FDBF"
        "305C986D0D4D2B401A860E12AD7CFEBF0886F053E94C2B408D1A765E380100C0"
        "2C5CFEBBF64A2440C395FE84C0E1FFBF4691FBCD14A92340E2A8D3EC00DEFFBF"
        "B71EDA73C5AB2340D5CE60981A9CFDBF5399A0E553402440771937328B9FFDBF"
        "6CD4FD671D4D2B4024A189292AD0FDBF5335815C1D4D2B400CD1A22EA6D0FDBF"
        "5435815C1D4D2B400CD1A22EA6D0FDBF"))
    original_bytes = shape.wkb
    _, proof, actual = write_actual(tmp_path, shape,
        axis=(-.9999958112660586, .00289438254852023), lane_type="median")
    norm = proof["source_numerical_normalization"]
    assert proof["source_geometry_wkb_hex"] == original_bytes.hex()
    assert shape.wkb == original_bytes
    assert norm["changed_vertex_count"] == 1 and norm["deleted_vertices"] == 0
    mapping = norm["source_vertex_mapping"]
    assert len(mapping) == len(shape.exterior.coords)-1 == proof["complexity"]["source_vertices"] == 9
    assert [r["vertex_index"] for r in mapping if r["changed"]] == [8]
    assert mapping[8]["representative_vertex_index"] == 0
    assert mapping[7]["original_xy"] == mapping[7]["normalized_xy"]
    assert math.dist(mapping[7]["normalized_xy"], mapping[0]["normalized_xy"]) > .000118
    assert norm["maximum_derived_source_shift_bound_m"] < 8e-15
    assert proof["complexity"]["line_roads"] == 1
    assert proof["complexity"]["minimum_width_event_span_m"] > .0048
    assert actual.symmetric_difference(shape).area < 4e-9
