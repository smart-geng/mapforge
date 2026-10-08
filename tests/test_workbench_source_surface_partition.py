"""Source partition must preserve absent support and all driving semantics."""
from copy import deepcopy
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from shapely.geometry import box
from shapely.ops import unary_union

from mapforge.workbench import source_surface_partition as P


def snapshot():
    return {"snapshot_id": P.SNAPSHOT_ID, "content_hash": P.SNAPSHOT_CONTENT_HASH,
            "junction_id": P.JUNCTION_ID}


@pytest.mark.parametrize("field", ["snapshot_id", "content_hash", "junction_id"])
def test_unregistered_source_cannot_select_known_partition(field):
    source = snapshot()
    source[field] = "different"
    with pytest.raises(P.PartitionRejected, match="source-snapshot-not-registered"):
        P.make_intent(source)


def test_unconfirmed_or_expanded_intent_cannot_reach_geometry():
    source = snapshot()
    confirmed = P.make_intent(source)
    for change in ({"confirmed": False}, {"source_lanes": ["another"]}, {"gap_action": "fill"}):
        intent = {**confirmed, **change}
        with pytest.raises(P.PartitionRejected, match="intent-not-registered"):
            P.apply_partition(ET.Element("OpenDRIVE"), None, SimpleNamespace(pid=P.JUNCTION_ID),
                              None, [], source, intent)


def test_xml_invariant_excludes_only_auxiliary_paving():
    root = ET.fromstring('''<OpenDRIVE><header/><road id="10" name="approach">
      <link><successor elementId="1"/></link><lanes><laneSection s="0"><right>
      <lane id="-1" type="driving"><speed max="60" unit="km/h"/></lane>
      </right></laneSection></lanes></road><road id="90" name="junction_paving"/>
      <junction id="1"><connection incomingRoad="10" connectingRoad="100"/></junction></OpenDRIVE>''')
    baseline = P.unchanged_subtrees(root)
    paving_change = deepcopy(root)
    paving_change.find("road[@id='90']").set("id", "51")
    ET.indent(paving_change)
    assert P.unchanged_subtrees(paving_change) == baseline
    for xpath, field, value in [("road[@id='10']/link/successor", "elementId", "2"),
                                ("road[@id='10']//speed", "max", "80"),
                                ("junction/connection", "connectingRoad", "101")]:
        changed = deepcopy(root)
        changed.find(xpath).set(field, value)
        assert P.unchanged_subtrees(changed) != baseline


def test_existing_support_tolerance_does_not_allow_an_unsupported_strip():
    support = box(0, 0, 10, 2)
    assert P.audit_support(box(0, 0, 10, 2.04), support)["supported"]
    rejected = P.audit_support(box(0, 0, 10, 2.1), support)
    assert not rejected["supported"]
    assert rejected["outside_existing_support_tolerance_m2"] > .4


def test_source_hole_cannot_be_filled_by_partition():
    support = box(0, 0, 20, 20).difference(box(8, 8, 12, 12))
    assert not P.audit_support(box(0, 0, 20, 20), support)["supported"]


def test_preserved_separator_and_added_crossing_are_distinguished():
    tail, base = box(-1, -3, 20, 3), box(0, 4, 20, 12)
    spec = {"pose": [0, 0, 0]}
    original = unary_union([tail, base])
    check = P.check_preserved_gap(original, tail, base, spec)
    assert check["gap_preserved"] and check["length_m"] == pytest.approx(1)
    for new_support in (box(4, 3, 5, 4), box(4, 3.4, 5, 3.6)):
        check = P.check_preserved_gap(unary_union([original, new_support]), tail, base, spec)
        assert not check["gap_preserved"]
        assert check["written_paving_in_gap_m"] >= .19


def test_changed_pre_registered_gap_is_not_silently_reinterpreted():
    spec = {"pose": [0, 0, 0]}
    tail, base = box(-1, -3, 20, 3), box(0, 2, 20, 12)
    with pytest.raises(P.PartitionRejected, match="gap-shape-changed"):
        P.check_preserved_gap(unary_union([tail, base]), tail, base, spec)
