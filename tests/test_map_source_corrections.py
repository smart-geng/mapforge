"""Approved MAP source corrections: bound to bytes and identity, derived copy only, DROPPED recorded."""
import copy
import hashlib
from pathlib import Path

import pytest

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops import map_source_corrections as C

ROOT = Path(__file__).resolve().parents[1]
NODE18 = ROOT / "v2x_map_xml" / "map凤苑路-金剑路node18.xml"
NODE4 = ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"
pytestmark = pytest.mark.skipif(not (NODE18.exists() and NODE4.exists()), reason="local MAP sample not installed")


def _lane_points(path):
    node = parse_map_xml(str(path))
    return {(link.name, lane.lane_id): len(lane.points) for link in node.links for lane in link.lanes}


def test_decision_binds_exact_bytes_and_drops_only_the_outlier(tmp_path):
    decisions = C.applicable(NODE18)
    assert [d["id"] for d in decisions] == ["map-node18-north-lane1-p1-drop-v1"]
    # every approved decision applies to its own source file only
    assert {p.name: [d["id"] for d in C.applicable(p)] for p in NODE18.parent.glob("map*.xml") if C.applicable(p)} == {
        NODE18.name: ["map-node18-north-lane1-p1-drop-v1"], NODE4.name: ["map-node4-west-lane2-lane3-p5-drop-v1"]}
    before = hashlib.sha256(NODE18.read_bytes()).hexdigest()
    out = tmp_path / "node18.corrected.xml"
    record = C.apply(NODE18, decisions, out)
    assert hashlib.sha256(NODE18.read_bytes()).hexdigest() == before  # original untouched
    assert record["summary"] == {"DROPPED": 1}
    item = record["items"][0]
    assert item["source_lane_id"] == "map:500:18:from:500:20:north:lane:1" and item["point_index_0based"] == 1
    assert abs(item["evidence"]["distance_to_neighbor_chord_m"] - 1719.94) < 1.0
    old, new = _lane_points(NODE18), _lane_points(out)
    assert {k for k in old if old[k] != new[k]} == {("north", 1)} and new[("north", 1)] == old[("north", 1)] - 1


def test_changed_source_bytes_do_not_match_any_decision(tmp_path):
    changed = tmp_path / NODE18.name
    changed.write_bytes(NODE18.read_bytes() + b"\n")
    assert C.applicable(changed) == []


@pytest.mark.parametrize("field, value", [("raw", {"lon": 1063346866, "lat": 295139806}),
                                          ("evidence_offset", 1500.0)])
def test_mismatched_identity_or_evidence_is_refused(tmp_path, field, value):
    decision = copy.deepcopy(C.applicable(NODE18)[0])
    target = decision["corrections"][0]
    if field == "raw":
        target["target"]["raw"] = value
    else:
        target["evidence"]["distance_to_neighbor_chord_m"] = value
    with pytest.raises(ValueError):
        C.apply(NODE18, [decision], tmp_path / "x.xml")


def test_node4_decision_drops_the_two_lateral_spike_points_only(tmp_path):
    decisions = C.applicable(NODE4)
    before = hashlib.sha256(NODE4.read_bytes()).hexdigest()
    record = C.apply(NODE4, decisions, tmp_path / "node4.corrected.xml")
    assert hashlib.sha256(NODE4.read_bytes()).hexdigest() == before  # original untouched
    assert record["summary"] == {"DROPPED": 2}
    assert [(i["source_lane_id"], i["point_index_0based"]) for i in record["items"]] == [
        ("map:500:4:from:500:18:west:lane:2", 5), ("map:500:4:from:500:18:west:lane:3", 5)]
    assert all(i["evidence"]["rule"].startswith("lateral-spike") for i in record["items"])
    old, new = _lane_points(NODE4), _lane_points(tmp_path / "node4.corrected.xml")
    assert {k: new[k] - old[k] for k in old if old[k] != new[k]} == {("west", 2): -1, ("west", 3): -1}
