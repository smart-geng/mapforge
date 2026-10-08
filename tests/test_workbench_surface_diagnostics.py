"""Independent geometry and refusal contracts; synthetic data is not acceptance."""
from copy import deepcopy
from itertools import count
import json
import xml.etree.ElementTree as ET

import pytest
from shapely.geometry import Polygon, box, shape
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.workbench import source_surface_partition as P
from mapforge.workbench import source_surface_reconstruction as R
from mapforge.workbench import source_surface_support as S
from mapforge.workbench import source_surface_tracks as T
from mapforge.workbench import surface_diagnostics as D


def case(original_hole=None):
    """Real serializer/auditors and a separately declared synthetic source gap."""
    root = ET.fromstring('''<OpenDRIVE><header/>
      <road name="physical" length="10" id="10" junction="-1">
        <planView><geometry s="0" x="-10" y="0" hdg="0" length="10"><line/></geometry></planView>
        <lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>
          <laneSection s="0"><center><lane id="0" type="none" level="false"/></center>
            <right><lane id="-1" type="driving" level="false">
              <width sOffset="0" a="3" b="0" c="0" d="0"/>
            </lane></right></laneSection></lanes>
      </road><junction id="1"/></OpenDRIVE>''')
    tail, base, median = box(-1, -3, 20, 3), box(0, 4, 20, 12), box(-1, 3, 0, 4)
    if original_hole is not None:
        base = unary_union([base, box(30, 30, 40, 40).difference(original_hole)])
    restricted, total = unary_union([tail, base]), unary_union([tail, base, median])
    doc, ids, families = W.XodrDoc("synthetic diagnosis"), count(50), []
    for index, (geometry, kind, road, lanes, role) in enumerate([
        (tail, "restricted", "10", ["lane-A"], "raw-source-tail"),
        (base, "restricted", None, [], "source-polygon"),
        (median, "median", "10", ["lane-A"], "existing-median-continuation")]):
        provenance = {"support_kind": role, "source_snapshot_id": "a" * 64,
            "source_content_hash": "b" * 64, "source_region_index": index,
            "source_region_sha256": D.hashlib.sha256(geometry.wkb).hexdigest(), "source_lanes": lanes}
        proof = T.append_polygon(doc, geometry, 1, ids, preferred_axis=(1, 0),
                                 lane_type=kind, provenance=provenance)
        families.append({"region_index": index, "lane_type": kind, "source_road": road,
                         "role": role, "provenance": provenance, "representation": proof})
    for road in doc.roads:
        doc._road_el(root, road)
    root = ET.fromstring(ET.tostring(root))
    support = {"schema": S.SCHEMA, "fixed_support": S._record(total),
        "support_geometry_wkb_hex": total.wkb_hex, "raw_source": S._record(tail),
        "old_medians": S._record(median), "source_base": S._record(base),
        "expected_restricted": S._record(restricted), "expected_median": S._record(median),
        "serialization_band_upper_bound_m": 1e-6,
        "source_mouths": [{"road_id": "10", "pose": [0, 0, 0], "enter_link": "enter", "leave_links": ["leave"]}],
        "source_pieces": [{"source_lane": "lane-A", "road_id": "10", **S._record(tail)}],
        "original_holes": S._record(Polygon() if original_hole is None else original_hole),
        "departure_raw_asphalt_base_gap_m": 1.}
    support["evidence_sha256"] = S._json_sha(support)
    evidence = {"schema": D.RECONSTRUCTION_SCHEMA,
        "intent": {"type": R.INTENT_TYPE, "enter_link": "enter", "leave_links": ["leave"],
                   "source_snapshot_id": "a" * 64, "source_content_hash": "b" * 64},
        "non_paving_subtrees_before": P.unchanged_subtrees(root),
        "paving_id_diff": {"candidate_ids": [str(r.road_id) for r in doc.roads]},
        "families": families, "source_support": support}
    return root, evidence


def test_real_audited_xml_diagnosis_is_neither_driving_nor_delivery_acceptance():
    root, evidence = case()
    original_root, original_evidence = ET.tostring(root), deepcopy(evidence)
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "DIAGNOSED", result
    assert result["summary"]["issue_count"] == 0
    assert result["candidate_accepted"] is False
    assert result["whole_map_score_modified"] is False
    assert result["driving_continuity_proven"] is False
    assert result["actual_proof"]["reconstruction_audit"]["non_paving_unchanged_checked"] is False
    assert {layer["id"] for layer in result["layers"]} == {
        "source-restricted", "source-median", "actual-restricted", "actual-median"}
    assert ET.tostring(root) == original_root and evidence == original_evidence
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("evidence", [None, {}, {"schema": "other"}])
def test_absent_or_unsupported_source_evidence_never_becomes_zero_issues_pass(evidence):
    result = D.diagnose_surface(ET.Element("OpenDRIVE"), evidence)
    assert result["status"] == "UNAVAILABLE" and result["summary"] is None


@pytest.mark.parametrize("change", ["width", "curve", "cubic-width", "duplicate", "missing", "routing", "lane-type"])
def test_actual_loss_unsupported_curve_and_routing_changes_are_unavailable(change):
    root, evidence = case()
    road = root.find("road[@name='junction_paving']")
    if change == "width":
        road.find("lanes/laneSection/right/lane/width").set("a", "5.999")
    elif change == "curve":
        geometry = road.find("planView/geometry")
        geometry.remove(geometry.find("line"))
        ET.SubElement(geometry, "arc", curvature="0.001")
    elif change == "cubic-width":
        road.find("lanes/laneSection/right/lane/width").set("d", "0.0001")
    elif change == "duplicate":
        root.append(deepcopy(road))
    elif change == "missing":
        root.remove(road)
    elif change == "routing":
        ET.SubElement(root.find("junction"), "connection", connectingRoad=road.get("id"))
    else:
        road.find("lanes/laneSection/right/lane").set("type", "driving")
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "UNAVAILABLE", result
    assert result["summary"] is None and not result["candidate_accepted"]


@pytest.mark.parametrize("field", ["source_snapshot_id", "source_content_hash"])
def test_valid_family_provenance_cannot_be_attached_to_a_different_intent_source(field):
    root, evidence = case()
    evidence["intent"][field] = "c" * 64
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "UNAVAILABLE"
    assert "does-not-match-reconstruction-intent" in result["reason"]


@pytest.mark.parametrize("change", ["source-hole", "numeric-band", "family-role", "source-road", "provenance"])
def test_changed_proof_or_display_identity_is_refused(change):
    root, evidence = case()
    if change == "source-hole":
        evidence["source_support"]["original_holes"] = S._record(box(0, 3, 20, 4))
    elif change == "numeric-band":
        evidence["families"][0]["representation"]["numeric_serialization_band_m"] = .05
    elif change == "family-role":
        evidence["families"][0]["role"] = "measured-boundary"
    elif change == "source-road":
        evidence["families"][2]["source_road"] = "other"
    else:
        evidence["families"][0]["provenance"]["source_lanes"] = ["other-lane"]
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "UNAVAILABLE", result


def test_median_cannot_hide_a_driving_mouth_gap():
    root, evidence = case()
    root.find("road[@id='10']/planView/geometry").set("y", "4")
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "UNAVAILABLE"
    assert "reconstruction-readback-audit-failed" in result["reason"]


def test_mixed_hole_is_spatially_partitioned_not_classified_by_total_area():
    hole = box(0, 0, 3, 1)
    rows = D._partition_gap(hole, box(0, 0, 1, 1), box(1, 0, 2, 1), 1e-8)
    assert {kind for kind, _ in rows} == {"original-source-gap", "representation-loss", "unattributed-gap"}
    assert all(geometry.area == 1 for _, geometry in rows)
    assert unary_union([geometry for _, geometry in rows]).equals(hole)


def test_equal_area_at_another_location_is_not_original_source_gap():
    rows = D._partition_gap(box(0, 0, 1, 1), box(10, 10, 11, 11), Polygon(), 1e-8)
    assert [kind for kind, _ in rows] == ["unattributed-gap"]


def test_real_microgap_below_old_area_cutoff_is_not_eaten_by_five_cm_tolerance():
    hole = box(0, 0, .000118, 1)
    actual = box(-1, -1, 1, 2).difference(hole)
    assert not D._covered(hole, actual, 4e-8)
    rows = D._partition_gap(hole, Polygon(), Polygon(), 4e-8, actual=actual)
    assert rows[0][0] == "unattributed-gap" and rows[0][1].equals(hole)
    assert rows[0][1].area < D.EXISTING_AREA_CUTOFF_M2


def test_real_serialized_sub_cutoff_source_hole_is_a_locatable_primary_issue():
    hole = Polygon([(35, 34.999), (35.001, 35), (35, 35.001), (34.999, 35)])
    root, evidence = case(hole)
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "DIAGNOSED", result
    actual_issues = [row for row in result["issues"] if row["kind"] == "original-source-gap"]
    assert len(actual_issues) == 1
    assert 0 < actual_issues[0]["area_m2"] < .01
    assert shape(actual_issues[0]["geometry"]).hausdorff_distance(hole) < 1e-7
    assert actual_issues[0]["road_ids"]


def test_supported_exact_geometry_distinguishes_old_sampling_false_hole(monkeypatch):
    root, evidence = case()
    false_hole = box(1, 5, 2, 6)
    sampled = box(0, 4, 20, 12).difference(false_hole)
    monkeypatch.setattr(D.smoothness, "road_surface_polygon", lambda road: sampled)
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "DIAGNOSED", result
    assert result["summary"]["by_kind"] == {"sampling-artifact": 1}
    issue = result["issues"][0]
    assert shape(issue["geometry"]).equals(false_hole)
    assert issue["road_ids"] and issue["roles"] == ["source-polygon"]
    assert result["summary"]["paving_holes_gt1cm2"] == 1
    assert "100 cm²" in result["metric_definition"]["paving_holes_gt1cm2"]


def test_sub_cutoff_sampling_geometry_retained_without_concealing_actual_gap(monkeypatch):
    root, evidence = case()
    false_hole = box(1, 5, 1.001, 6)
    sampled = box(0, 4, 20, 12).difference(false_hole)
    monkeypatch.setattr(D.smoothness, "road_surface_polygon", lambda road: sampled)
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "DIAGNOSED"
    assert result["issues"] == []
    assert result["sampling_details"][0]["classification"] == "sampling-artifact"
    assert shape(result["sampling_details"][0]["geometry"]).equals(false_hole)


def test_failed_sampling_cannot_report_fake_zero_holes(monkeypatch):
    root, evidence = case()
    monkeypatch.setattr(D.smoothness, "road_surface_polygon", lambda road: Polygon())
    result = D.diagnose_surface(root, evidence)
    assert result["status"] == "UNAVAILABLE" and result["summary"] is None


def test_polygon_holes_and_source_identity_remain_in_issue_geometry():
    geometry = box(0, 0, 3, 3).difference(box(1, 1, 2, 2))
    families = [{"geometry": box(3, 0, 5, 3), "region_index": 2, "role": "raw-source-tail",
                 "lane_type": "restricted", "road_ids": ["52"], "source_road": "10", "source_lane_ids": ["A"]}]
    result = D._issue("unattributed-gap", geometry, families, [], 1e-8)
    assert len(shape(result["geometry"]).interiors) == 1
    assert result["road_ids"] == ["52"] and result["source_lane_ids"] == ["A"]
    assert result["source_road_ids"] == ["10"]
