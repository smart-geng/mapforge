import numpy as np

from mapforge.adapters.opendrive.writer import Lane, LaneSection, Road, XodrDoc
from mapforge.validate.g8_model import make_manifest, source_lane
from mapforge.validate.lane_fidelity import evaluate_g8, extract_target_components


def _prov(direction="with_s"):
    return {"eligibility": "comparable", "role": "approach",
            "status": "EXACT", "support_kind": "synthetic",
            "policy_class": "test.measured", "travel_direction": direction}


def _manifest(points):
    lane = source_lane(
        "A", points, owner={"format": "synthetic"}, role="approach",
        status="EXACT", support_kind="synthetic", policy_class="test.measured",
        travel_direction="against_s" if points[0, 0] > points[-1, 0] else "with_s",
    )
    return make_manifest(source_format="synthetic", source_profile="test",
                         comparison_crs={"units": "m", "id": "local"}, lanes=[lane])


def test_adjacent_lane_sections_form_one_component(tmp_path, g8_policy):
    doc = XodrDoc("sections")
    road = Road(1)
    road.add_geometry("line", 0, 0, 0, 100)
    road.add_offset(0, 0)
    for i, s in enumerate((0.0, 50.0)):
        sec = LaneSection(s)
        lane = Lane(-1, source_id="A", provenance=_prov())
        lane.add_width(3.5)
        lane.pred = -1 if i else None
        lane.succ = -1 if i == 0 else None
        sec.right.append(lane)
        road.sections.append(sec)
    doc.add_road(road)
    out = tmp_path / "sections.xodr"
    doc.write(out)
    extracted = extract_target_components(out)
    assert len(extracted["components"]) == 1
    x = np.linspace(0, 100, 101)
    result = evaluate_g8(out, _manifest(np.column_stack([x, x * 0 - 1.75])), g8_policy)
    assert result["status"] == "PASS", result


def test_zero_width_gap_does_not_create_vstack_bridge(tmp_path, g8_policy):
    doc = XodrDoc("gap")
    road = Road(1)
    road.add_geometry("line", 0, 0, 0, 100)
    road.add_offset(0, 0)
    for s, width in ((0.0, 3.5), (40.0, 0.0), (60.0, 3.5)):
        sec = LaneSection(s)
        lane = Lane(-1, source_id="A", provenance=_prov())
        lane.add_width(width)
        sec.right.append(lane)
        road.sections.append(sec)
    doc.add_road(road)
    out = tmp_path / "gap.xodr"
    doc.write(out)
    extracted = extract_target_components(out)
    assert len(extracted["components"]) == 2
    x = np.linspace(0, 100, 101)
    result = evaluate_g8(out, _manifest(np.column_stack([x, x * 0 - 1.75])), g8_policy)
    assert result["status"] == "FAIL"
    assert result["issues"]["one_source_many_targets"] == ["A"]


def test_left_lane_is_compared_in_travel_direction(tmp_path, g8_policy):
    doc = XodrDoc("left")
    road = Road(1)
    road.add_geometry("line", 0, 0, 0, 100)
    road.add_offset(0, 0)
    sec = LaneSection(0)
    lane = Lane(1, source_id="A", provenance=_prov("against_s"))
    lane.add_width(3.5)
    sec.left.append(lane)
    road.sections.append(sec)
    doc.add_road(road)
    out = tmp_path / "left.xodr"
    doc.write(out)
    x = np.linspace(100, 0, 101)
    result = evaluate_g8(out, _manifest(np.column_stack([x, x * 0 + 1.75])), g8_policy)
    assert result["status"] == "PASS", result
    assert result["per_lane"][0]["endpoint"]["travel_start_m"] < 1e-6
