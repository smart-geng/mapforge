"""Independent rejection and written-footprint contracts for the WB11 probe.

Fixtures below are synthetic geometry, not a source-registration or product
acceptance record. The family serializer and its readback remain real.
"""
from copy import deepcopy
from itertools import count
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from shapely.geometry import MultiPolygon, Polygon, box
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W
from mapforge.workbench import source_surface_partition as P
from mapforge.workbench import source_surface_reconstruction as R
from mapforge.workbench import source_surface_support as S
from mapforge.workbench import source_surface_tracks as T
from mapforge.workbench import source_surface_cells as C


def snapshot():
    return {"snapshot_id": P.SNAPSHOT_ID, "content_hash": P.SNAPSHOT_CONTENT_HASH,
            "junction_id": P.JUNCTION_ID}


@pytest.mark.parametrize("change", [
    {"confirmed": False}, {"padding_m": .25}, {"source_holes": "fill"},
    {"affected_scope": "driving-roads"}, {"support_domain": "new-candidate"},
    {"median_action": "use-new-median-as-source"},
])
def test_changed_intent_rejected_before_source_access(change):
    snap = snapshot()
    with pytest.raises(P.PartitionRejected, match="intent-not-registered"):
        R.apply_rebuild(ET.Element("OpenDRIVE"), None, SimpleNamespace(pid=P.JUNCTION_ID),
                        None, [], snap, {**R.make_intent(snap), **change})


@pytest.mark.parametrize("field", ["snapshot_id", "content_hash", "junction_id"])
def test_different_snapshot_cannot_inherit_fixed_source_authorization(field):
    snap = snapshot()
    snap[field] = "different-source"
    with pytest.raises(P.PartitionRejected, match="source-snapshot-not-registered"):
        R.make_intent(snap)


def physical_root():
    return ET.fromstring('''<OpenDRIVE><header/>
      <road name="physical" length="10" id="10" junction="-1">
        <planView><geometry s="0" x="-10" y="0" hdg="0" length="10"><line/></geometry></planView>
        <lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>
          <laneSection s="0"><center><lane id="0" type="none" level="false"/></center>
            <right><lane id="-1" type="driving" level="false">
              <width sOffset="0" a="3" b="0" c="0" d="0"/>
            </lane></right>
          </laneSection>
        </lanes>
      </road><junction id="1"/></OpenDRIVE>''')


def test_median_surface_cannot_substitute_for_driving_mouth():
    result = R.audit_mouth_coverage(physical_root(), [{"road_id": "10"}],
        {"restricted": box(10, 10, 11, 11), "median": box(-1, -4, 1, 1)}, 1e-8)
    assert result["within_existing_source_cover_tolerance"] is False
    assert result["all_exact_contacts_within_numeric_band"] is False
    assert result["rows"][0]["uncovered_exact_width_m"] == 3
    assert result["driving_continuity_proven"] is False


@pytest.mark.parametrize("gap,within_source", [(0., True), (.002, True), (.051, False)])
def test_real_mouth_gap_distinguished_from_serialization_roundoff(gap, within_source):
    result = R.audit_mouth_coverage(physical_root(), [{"road_id": "10"}],
        {"restricted": box(gap, -4, 5, 1), "median": Polygon()}, 1e-8)
    assert result["within_existing_source_cover_tolerance"] is within_source
    assert result["all_exact_contacts_within_numeric_band"] is (gap == 0)
    assert result["rows"][0]["uncovered_exact_width_m"] == (0 if gap == 0 else 3)
    assert result["source_cover_tolerance_m"] == .05
    assert result["driving_continuity_proven"] is False
    assert result["route_seams_evaluated"] is False


@pytest.mark.parametrize("kind", ["missing-road", "duplicate-road", "empty-lanes"])
def test_missing_or_ambiguous_actual_mouth_cannot_pass(kind):
    root = physical_root()
    road = root.find("road")
    if kind == "missing-road":
        root.remove(road)
    elif kind == "duplicate-road":
        root.append(deepcopy(road))
    else:
        road.find("lanes").remove(road.find("lanes/laneSection"))
    with pytest.raises(P.PartitionRejected):
        R.audit_mouth_coverage(root, [{"road_id": "10"}],
            {"restricted": box(-1, -4, 5, 1), "median": Polygon()}, 1e-8)


def audit_case():
    """Actual track XML with deliberately separate asphalt and median layers."""
    tail, base, median = box(-1, -3, 20, 3), box(0, 4, 20, 12), box(-1, 3, 0, 4)
    restricted = unary_union([tail, base])
    support = unary_union([restricted, median])
    root, doc, ids = physical_root(), W.XodrDoc("synthetic audit"), count(50)
    families = []
    for shape, kind in [(tail, "restricted"), (base, "restricted"), (median, "median")]:
        proof = T.append_polygon(doc, shape, 1, ids, preferred_axis=(1, 0), lane_type=kind)
        families.append({"lane_type": kind, "representation": proof})
    for road in doc.roads:
        doc._road_el(root, road)
    # Reparse the actual XML exactly as the production reconstruction audit does.
    root = ET.fromstring(ET.tostring(root))
    source_evidence = {"schema": S.SCHEMA,
        "fixed_support": S._record(support), "raw_source": S._record(tail),
        "old_medians": S._record(median), "source_base": S._record(base),
        "expected_restricted": S._record(restricted), "expected_median": S._record(median),
        "serialization_band_upper_bound_m": 1e-6,
        "source_mouths": [{"road_id": "10", "pose": [0, 0, 0]}],
        "departure_raw_asphalt_base_gap_m": 1.}
    source_evidence["evidence_sha256"] = S._json_sha(source_evidence)
    evidence = {"non_paving_subtrees_before": P.unchanged_subtrees(root),
        "paving_id_diff": {"candidate_ids": [str(r.road_id) for r in doc.roads]},
        "families": families, "source_support": source_evidence}
    context = {"support": support, "represented_support": support, "tails": tail,
               "base": base, "target": {"pose": [0, 0, 0]}}
    return root, evidence, context


def test_real_written_family_support_pass_is_not_candidate_or_driving_acceptance():
    root, evidence, context = audit_case()
    report = R.audit_written(root, evidence, context)
    assert report["status"] == "PASS", report
    assert all(f["passed"] for f in report["families"])
    assert report["source_support_audit"]["passed"]
    assert report["candidate_accepted"] is False
    assert report["driving_continuity_proven"] is False
    assert report["whole_map_validation_required"] is True


def test_failed_family_cannot_be_hidden_by_identical_total_footprint(monkeypatch):
    root, evidence, context = audit_case()
    original = T.audit_written
    failed_id = evidence["families"][0]["representation"]["road_ids"]
    def one_failed(actual, proof):
        report = original(actual, proof)
        if proof["road_ids"] == failed_id:
            return {**report, "status": "FAIL", "passed": False}
        return report
    monkeypatch.setattr(T, "audit_written", one_failed)
    report = R.audit_written(root, evidence, context)
    assert report["status"] == "FAIL"
    assert report["represented_symmetric_difference_m2"] == 0


@pytest.mark.parametrize("kind", ["remove", "duplicate", "rename"])
def test_complete_auxiliary_id_set_required(kind):
    root, evidence, context = audit_case()
    road = root.find('road[@name="junction_paving"]')
    if kind == "remove":
        root.remove(road)
    elif kind == "duplicate":
        root.append(deepcopy(road))
    else:
        road.set("id", "999")
    with pytest.raises(P.PartitionRejected, match="paving-id-set"):
        R.audit_written(root, evidence, context)


def test_driving_road_change_requires_explicit_postprocess_audit_scope():
    root, evidence, context = audit_case()
    root.find('road[@id="10"]').set("name", "modified-by-postprocess")
    with pytest.raises(P.PartitionRejected, match="non-paving-subtree-changed"):
        R.audit_written(root, evidence, context)
    report = R.audit_written(root, evidence, context, require_driving_unchanged=False)
    assert report["status"] == "PASS", report
    assert report["non_paving_unchanged_checked"] is False
    assert report["driving_continuity_proven"] is False


def test_postprocess_mouth_movement_is_checked_against_actual_xml():
    root, evidence, context = audit_case()
    root.find('road[@id="10"]/planView/geometry').set("x", "-12")
    report = R.audit_written(root, evidence, context, require_driving_unchanged=False)
    assert report["status"] == "FAIL"
    assert report["written_road_mouth_coverage"]["within_existing_source_cover_tolerance"] is False


def test_layer_reassignment_cannot_pass_even_when_total_material_is_equal():
    root, evidence, context = audit_case()
    for family in evidence["families"]:
        family["lane_type"] = "median" if family["lane_type"] == "restricted" else "restricted"
    report = R.audit_written(root, evidence, context)
    assert report["status"] == "FAIL"
    assert report["represented_symmetric_difference_m2"] == 0
    assert report["source_support_audit"]["passed"] is False


def protocol_surface(gap, *, baseline=False):
    """Synthetic, actually serialized surfaces; no source-registration claim.

    The protocol's material-support precondition is supplied separately below.
    All mouth and missing-subset geometry uses actual XML, with the old cells
    serializer for the baseline and new tracks serializer for the candidate.
    """
    shape = box(-1, -3, 5, 0) if gap is None else MultiPolygon([
        box(-1, -3, 5, gap[0]), box(-1, gap[1], 5, 0)])
    module = C if baseline else T
    root, doc = physical_root(), W.XodrDoc("synthetic protocol")
    proof = module.append_polygon(doc, shape, 1, count(50), preferred_axis=(1, 0))
    for road in doc.roads:
        doc._road_el(root, road)
    root = ET.fromstring(ET.tostring(root))
    assert module.audit_written(root, proof)["passed"]
    mouths = [{"road_id": "10", "pose": [0, 0, 0]}]
    evidence = {"families": [{"lane_type": "restricted", "representation": proof}],
        "non_paving_subtrees_before": P.unchanged_subtrees(root),
        "paving_id_diff": {"candidate_ids": proof["road_ids"]},
        "source_support": {"source_mouths": mouths}}
    layers = {"restricted": module.written_geometry(root, proof["road_ids"]), "median": Polygon()}
    mouth_audit = R.audit_mouth_coverage(root, mouths, layers, proof["numeric_serialization_band_m"])
    current_audit = {"status": "PASS" if mouth_audit["within_existing_source_cover_tolerance"] else "FAIL",
        "material_support_passed": True, "written_road_mouth_coverage": mouth_audit,
        "candidate_accepted": False, "driving_continuity_proven": False}
    return root, evidence, current_audit


def compare_protocol(old_gap, new_gap):
    baseline_root, baseline_evidence, _ = protocol_surface(old_gap, baseline=True)
    root, evidence, audit = protocol_surface(new_gap)
    return R.compare_preprocess_mouths(root, baseline_root, evidence, baseline_evidence, audit)


def test_inherited_preprocess_mouth_failure_can_defer_without_becoming_pass():
    report = compare_protocol((-2, -1), (-2, -1))
    assert report["allow_postprocess"] is True, report
    assert report["regressions"] == []
    assert report["non_paving_unchanged"] is True
    assert report["current_mouth_coverage"]["within_existing_source_cover_tolerance"] is False
    assert report["baseline_mouth_coverage"]["within_existing_source_cover_tolerance"] is False
    assert report["current_mouth_coverage"]["driving_continuity_proven"] is False


def test_equal_length_gap_at_different_position_is_not_inherited():
    report = compare_protocol((-2, -1), (-1.5, -.5))
    assert report["allow_postprocess"] is False
    assert report["regressions"]
    old = report["baseline_mouth_coverage"]["rows"][0]
    new = report["current_mouth_coverage"]["rows"][0]
    assert old["uncovered_exact_width_m"] == new["uncovered_exact_width_m"] == 1


def test_expanded_gap_cannot_use_inherited_failure_exception():
    report = compare_protocol((-2, -1), (-2.2, -.8))
    assert report["allow_postprocess"] is False
    assert report["regressions"]


def test_previous_source_tolerance_pass_cannot_become_fail():
    report = compare_protocol((-2, -1.998), (-2.1, -1.9))
    assert report["baseline_mouth_coverage"]["within_existing_source_cover_tolerance"] is True
    assert report["current_mouth_coverage"]["within_existing_source_cover_tolerance"] is False
    assert report["allow_postprocess"] is False
    assert report["regressions"]


def test_smaller_inherited_gap_remains_a_failure_until_postprocess_rechecks_it():
    report = compare_protocol((-2, -1), (-1.9, -1.1))
    assert report["allow_postprocess"] is True, report
    assert report["current_mouth_coverage"]["within_existing_source_cover_tolerance"] is False


def test_changed_physical_road_cannot_use_preprocess_exception():
    baseline_root, baseline_evidence, _ = protocol_surface((-2, -1), baseline=True)
    root, evidence, audit = protocol_surface((-2, -1))
    root.find('road[@id="10"]').set("name", "changed-physical-subtree")
    report = R.compare_preprocess_mouths(root, baseline_root, evidence, baseline_evidence, audit)
    assert report["allow_postprocess"] is False
    assert report["non_paving_unchanged"] is False


@pytest.mark.parametrize("material", [False, None])
def test_material_failure_or_missing_check_cannot_defer_as_mouth_failure(material):
    baseline_root, baseline_evidence, _ = protocol_surface((-2, -1), baseline=True)
    root, evidence, audit = protocol_surface((-2, -1))
    if material is None:
        audit.pop("material_support_passed")
    else:
        audit["material_support_passed"] = material
    report = R.compare_preprocess_mouths(root, baseline_root, evidence, baseline_evidence, audit)
    assert report["allow_postprocess"] is False
