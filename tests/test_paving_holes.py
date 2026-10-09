"""Junction paving holes (user decisions 2026-10-09): breakpoint sampling and bound source voids."""
import hashlib
import json
import math
from pathlib import Path

import pytest
import yaml
from lxml import etree
from shapely.geometry import Polygon, box

from mapforge.validate import paving_holes as P
from mapforge.validate import scoreboard as sb
from mapforge.validate import smoothness

ROOT = Path(__file__).resolve().parents[1]
W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"


def _road(rid, x, y, hdg, length, left=(), right=(), sections=None):
    """A straight junction_paving road; ``left``/``right`` are width records (sOffset, a, b) of one lane."""
    road = etree.Element("road", id=str(rid), name="junction_paving", junction="1", length=str(length))
    plan = etree.SubElement(road, "planView")
    geometry = etree.SubElement(plan, "geometry", s="0", x=str(x), y=str(y), hdg=str(hdg), length=str(length))
    etree.SubElement(geometry, "line")
    lanes = etree.SubElement(road, "lanes")
    etree.SubElement(lanes, "laneOffset", s="0", a="0", b="0", c="0", d="0")
    for start, (left_widths, right_widths) in (sections or [(0.0, (left, right))]):
        section = etree.SubElement(lanes, "laneSection", s=str(start))
        for side, lane_id, widths in (("left", "1", left_widths), ("right", "-1", right_widths)):
            if not widths:
                continue
            group = etree.SubElement(section, side)
            lane = etree.SubElement(group, "lane", id=lane_id, type="driving", level="false")
            for s_offset, a, b in widths:
                etree.SubElement(lane, "width", sOffset=str(s_offset), a=str(a), b=str(b), c="0", d="0")
        center = etree.SubElement(section, "center")
        etree.SubElement(center, "lane", id="0", type="none", level="false")
    return road


def _root(*roads):
    root = etree.Element("OpenDRIVE")
    root.extend(roads)
    return root


def _seam_pair():
    """Two roads sharing one edge. A widens 1 -> 3 m over s 4.9-5.0 through width records inside one laneSection;
    B, above it, narrows over the same span through laneSection starts. Exactly, they abut without any gap."""
    k = 20.0
    a = _road(1, 0.0, 0.0, 0.0, 10.0, left=[(0.0, 1.0, 0.0), (4.9, 1.0, k), (5.0, 3.0, 0.0)])
    b = _road(2, 0.0, 4.0, 0.0, 10.0, sections=[(0.0, ((), [(0.0, 3.0, 0.0)])),
                                                (4.9, ((), [(0.0, 3.0, -k)])),
                                                (5.0, ((), [(0.0, 1.0, 0.0)]))])
    return _root(a, b)


def _frame():
    """Four roads around a 2 m x 2 m hole at x, y in [2, 4]."""
    north = math.pi / 2
    return _root(_road(1, 0.0, 0.0, 0.0, 6.0, left=[(0.0, 2.0, 0.0)]),
                 _road(2, 0.0, 4.0, 0.0, 6.0, left=[(0.0, 2.0, 0.0)]),
                 _road(3, 0.0, 0.0, north, 6.0, right=[(0.0, 2.0, 0.0)]),
                 _road(4, 6.0, 0.0, north, 6.0, left=[(0.0, 2.0, 0.0)]))


def _evidence(geometry, **change):
    raw = geometry.wkb
    record = {"wkb_hex": raw.hex(), "sha256": hashlib.sha256(raw).hexdigest(), "area_m2": geometry.area}
    record.update(change)
    return {"source_support": {"original_holes": record}}


def test_breakpoint_outline_is_exact_where_the_grid_outline_is_a_chord():
    road = _seam_pair().find("road")
    exact = Polygon([(0, 0), (10, 0), (10, 3), (5, 3), (4.9, 1), (0, 1)])
    assert P.road_outline(road).symmetric_difference(exact).area < 1e-9
    # the 0.1 m grid misses both kinks: a 0.0245 m2 triangle gained at s 4.9 and one lost at s 5.0
    assert smoothness.road_surface_polygon(road).symmetric_difference(exact).area > 0.04


def test_a_chord_sliver_on_a_shared_edge_is_no_longer_a_hole():
    root = _seam_pair()
    assert smoothness.surface_continuity(root)["paving_holes_gt1cm2"] == 1
    assert P.paving_holes(root) == []
    out = P.audit(root)
    assert out["paving_holes_resampled"] == out["paving_holes_counted"] == 0
    assert out["paving_source_void_evidence"] == "none"


def test_a_hole_counts_unless_it_coincides_with_a_bound_source_hole():
    root = _frame()
    holes = P.paving_holes(root)
    assert len(holes) == 1 and holes[0].area == pytest.approx(4.0, abs=1e-9)
    assert P.audit(root)["paving_holes_counted"] == 1
    out = P.audit(root, _evidence(box(2.0, 2.0, 4.0, 4.0)))
    assert out["paving_holes_source_void"] == 1 and out["paving_holes_counted"] == 0
    assert out["paving_source_void_area_m2"] == pytest.approx(4.0)
    assert out["paving_hole_counted_area_max_m2"] == 0.0 and out["paving_source_void_evidence"] == "bound"


@pytest.mark.parametrize("source", [box(1.5, 1.5, 4.5, 4.5), box(2.0, 2.0, 3.9, 4.0), box(10.0, 10.0, 12.0, 12.0)])
def test_coinciding_is_two_sided(source):
    """A larger or smaller recorded source hole, or one elsewhere, does not excuse the written hole."""
    out = P.audit(_frame(), _evidence(source))
    assert out["paving_holes_counted"] == 1 and out["paving_holes_source_void"] == 0
    assert out["paving_hole_counted_area_max_m2"] == pytest.approx(4.0)


@pytest.mark.parametrize("change", [{"sha256": "0" * 64}, {"area_m2": 3.9}, {"wkb_hex": "zz"},
                                    {"wkb_hex": "0103", "sha256": hashlib.sha256(bytes.fromhex("0103")).hexdigest()}])
def test_unverifiable_evidence_binds_nothing(change):
    out = P.audit(_frame(), _evidence(box(2.0, 2.0, 4.0, 4.0), **change))
    assert out["paving_source_void_evidence"] == "invalid"
    assert out["paving_holes_counted"] == 1 and out["paving_holes_source_void"] == 0
    assert P.audit(_frame(), {"source_support": {}})["paving_source_void_evidence"] == "invalid"


def test_policy_grades_counted_holes_in_t1_with_the_unchanged_cutoff():
    policy = yaml.safe_load(sb.POLICY.read_bytes())
    assert policy["version"] == "0.9-draft"
    t1 = {c["metric"]: c for c in policy["tiers"]["T1"]["checks"]}
    assert t1["paving_holes_counted"]["op"] == "le" and t1["paving_holes_counted"]["value"] == 0
    assert "paving_holes_gt1cm2" not in t1 and P.HOLE_MIN_M2 == 0.01
    for name in ("paving_holes_counted", "paving_holes_resampled", "paving_holes_source_void",
                 "paving_source_void_evidence", "paving_holes_gt1cm2"):
        assert name in sb.METRICS


@pytest.mark.skipif(not (W0621 / "surface-evidence.json").is_file(), reason="0621 workbench evidence not restored")
def test_0621_keeps_only_the_source_void():
    root = etree.parse(str(W0621 / "candidate.xodr")).getroot()
    evidence = json.loads((W0621 / "surface-evidence.json").read_text(encoding="utf-8"))
    assert smoothness.surface_continuity(root)["paving_holes_gt1cm2"] == 2
    out = P.audit(root, evidence)
    assert out["paving_holes_resampled"] == out["paving_holes_source_void"] == 1
    assert out["paving_holes_counted"] == 0 and out["paving_source_void_evidence"] == "bound"
    assert out["paving_source_void_area_m2"] == pytest.approx(9.741, abs=1e-3)
    assert P.audit(root)["paving_holes_counted"] == 1
