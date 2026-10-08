"""Reject source-scope changes and missing/extra material in written surfaces."""
import copy
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from shapely.geometry import box
from shapely.ops import unary_union

from mapforge.workbench import source_surface_partition as P
from mapforge.workbench import source_surface_rebuild as R


def snapshot():
    return {"snapshot_id": P.SNAPSHOT_ID, "content_hash": P.SNAPSHOT_CONTENT_HASH,
            "junction_id": P.JUNCTION_ID}


@pytest.mark.parametrize("change", [{"confirmed": False}, {"padding_m": .25},
    {"source_holes": "fill"}, {"affected_scope": "driving-roads"}])
def test_changed_scope_is_rejected_before_generation(change):
    snap = snapshot()
    with pytest.raises(P.PartitionRejected, match="intent-not-registered"):
        R.apply_rebuild(ET.Element("OpenDRIVE"), None, SimpleNamespace(pid=P.JUNCTION_ID),
                        None, [], snap, {**R.make_intent(snap), **change})


def audit_case(monkeypatch, written, *, family_passed=True):
    tail, base = box(-1, -3, 20, 3), box(0, 4, 20, 12)
    supported = unary_union([tail, base])
    root = ET.fromstring('<OpenDRIVE><header/><road id="10"/><road id="50" name="junction_paving"/><junction id="1"/></OpenDRIVE>')
    evidence = {"non_paving_subtrees_before": P.unchanged_subtrees(root),
        "paving_id_diff": {"candidate_ids": ["50"]}, "families": [{"representation": {}}]}
    context = {"support": supported, "represented_support": supported, "tails": tail,
               "base": base, "target": {"pose": [0, 0, 0]}}
    monkeypatch.setattr(R.C, "audit_written", lambda *a: {"passed": family_passed})
    monkeypatch.setattr(R.C, "written_geometry", lambda *a: written(supported))
    return root, evidence, context


def test_readback_requires_bidirectional_coverage(monkeypatch):
    root, evidence, context = audit_case(monkeypatch, lambda g: g.difference(box(9, -1, 10, 1)))
    result = R.audit_written(root, evidence, context)
    assert result["status"] == "FAIL"
    assert result["outside_source_support_m2"] == 0
    assert result["missing_source_support_m2"] > 1


def test_gap_crossing_is_rejected_even_when_small_area(monkeypatch):
    root, evidence, context = audit_case(monkeypatch, lambda g: unary_union([g, box(4.838, 3, 4.840, 4)]))
    result = R.audit_written(root, evidence, context)
    assert result["represented_symmetric_difference_m2"] < .01
    assert result["status"] == "FAIL"
    assert not result["full_paving_gap"]["gap_preserved"]


def test_a_failed_family_cannot_hide_in_successful_union(monkeypatch):
    root, evidence, context = audit_case(monkeypatch, lambda g: g, family_passed=False)
    assert R.audit_written(root, evidence, context)["status"] == "FAIL"


def test_passing_local_audit_does_not_accept_candidate(monkeypatch):
    root, evidence, context = audit_case(monkeypatch, lambda g: g)
    result = R.audit_written(root, evidence, context)
    assert result["status"] == "PASS"
    assert not result["candidate_accepted"]
    assert result["whole_map_validation_required"]


def test_changed_non_auxiliary_xml_or_auxiliary_ids_rejected(monkeypatch):
    root, evidence, context = audit_case(monkeypatch, lambda g: g)
    changed = copy.deepcopy(root)
    changed.find('road[@id="10"]').set("name", "changed")
    with pytest.raises(P.PartitionRejected, match="non-paving"):
        R.audit_written(changed, evidence, context)
    for modification in ("remove", "duplicate"):
        changed = copy.deepcopy(root)
        road = changed.find('road[@id="50"]')
        changed.remove(road) if modification == "remove" else changed.append(copy.deepcopy(road))
        with pytest.raises(P.PartitionRejected, match="paving-id-set"):
            R.audit_written(changed, evidence, context)
