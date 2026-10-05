"""MAP road-side refit: edge observations from MAP lane centres, narrow-lane coupling and the mirrored side."""
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import map_lane_refit as M

# Straight 100 m road along +x; approach (right) lanes carry MAP source lanes, the departure (left) lanes
# mirror them (no source lane, same width records).
ROAD = """<OpenDRIVE><road id="12" length="100" junction="-1">
 <planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
 <lanes>
  <laneSection s="0">
   <left>
    <lane id="2" type="driving"><width sOffset="0" a="{w2}" b="0" c="0" d="0"/></lane>
    <lane id="1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
   </left>
   <center><lane id="0" type="none"/></center>
   <right>
    <lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
     <userData code="mapforge.source_lane" value="S1"/></lane>
    <lane id="-2" type="driving"><width sOffset="0" a="{w2}" b="0" c="0" d="0"/>
     <userData code="mapforge.source_lane" value="S2"/></lane>
   </right>
  </laneSection>
 </lanes>
</road></OpenDRIVE>"""


def _road(w2=3.0):
    root = etree.fromstring(ROAD.format(w2=w2))
    return root, LR._Road(root.find("road"))


def _line(y):
    return np.array([[0.0, y], [100.0, y]])


def test_boundary_samples_put_the_lane_centres_on_the_map_points_and_the_widths_give_way():
    _, road = _road()
    # lane -1 lies 0.2 m left of its xodr centre, lane -2 on it: the centre spacing is 0.2 m wider than the
    # current half widths allow, so both widths grow (by 0.2 m each, the least change) and both centres hold
    observe = M.make_observe({"S1": _line(-1.55), "S2": _line(-5.0)})
    b0, b1, b2 = (observe(road, "right", 0, k) for k in (0, 1, 2))
    assert observe(road, "left", 0, 0) is None   # the centre boundary is solved once for both sides
    assert b0[:, 0].min() == pytest.approx(0.0) and b0[:, 0].max() == pytest.approx(100.0)
    assert (b0[:, 1] + b1[:, 1]) / 2 == pytest.approx(np.full(len(b0), -1.55), abs=5e-3)
    assert (b1[:, 1] + b2[:, 1]) / 2 == pytest.approx(np.full(len(b0), -5.0), abs=5e-3)
    assert b0[:, 1] - b1[:, 1] == pytest.approx(np.full(len(b0), 3.7), abs=0.01)
    assert b1[:, 1] - b2[:, 1] == pytest.approx(np.full(len(b0), 3.2), abs=0.01)


def test_a_narrow_lane_follows_its_fitted_inner_boundary_minus_its_width():
    _, road = _road(w2=1.0)   # an opening turn bay: narrower than NARROW_WIDTH_M
    fitted = {("right", 0, 1): [(0.0, 100.0, -3.4, 0.0, 0.0, 0.0)]}
    for centres in ({"S1": _line(-1.75), "S2": _line(-4.0)}, {"S1": _line(-1.75)}):
        observe = M.make_observe(centres)
        # solved alone, the narrow lane keeps its width (with or without its own point list) ...
        b1, b2 = observe(road, "right", 0, 1), observe(road, "right", 0, 2)
        assert b1[:, 1] - b2[:, 1] == pytest.approx(np.full(len(b1), 1.0), abs=1e-3)
        # ... and once its inner boundary is fitted, its outer boundary is that fit minus the width: the two
        # boundaries of a narrow lane are never fitted independently
        rows = observe(road, "right", 0, 2, fitted)
        assert rows[:, 1] == pytest.approx(np.full(len(rows), -3.4 - 1.0), abs=1e-3)


def test_a_point_list_starting_inside_the_lane_is_blended_in_before_its_first_point():
    _, road = _road()
    # lane -2's MAP points start at x = 60, 0.3 m right of the current lane centre (-5.0)
    observe = M.make_observe({"S1": _line(-1.75), "S2": np.array([[60.0, -5.3], [100.0, -5.3]])})
    b1, b2 = observe(road, "right", 0, 1), observe(road, "right", 0, 2)
    x, centre = b1[:, 0], (b1[:, 1] + b2[:, 1]) / 2
    before = x <= 60.0 - M.SOURCE_RAMP_M
    assert before.any() and centre[before] == pytest.approx(np.full(before.sum(), -5.0), abs=1e-6)
    assert centre[x >= 60.0] == pytest.approx(np.full(np.sum(x >= 60.0), -5.3), abs=0.01)
    assert np.max(np.abs(np.diff(centre))) < 0.3 * 4 * M.LR.GRID / M.SOURCE_RAMP_M   # no step, a ramp


def test_without_source_the_boundary_keeps_its_current_geometry():
    _, road = _road()
    rows = M.make_observe({})(road, "right", 0, 2)
    assert rows[:, 1] == pytest.approx(np.full(len(rows), -6.5))
    assert len(rows) >= 3


def test_mirrored_departure_lanes_are_recorded_and_take_the_refitted_widths():
    root, road = _road()
    road_el = root.find("road")
    assert M.mirrored_lanes(road_el) == {(0, "1"), (0, "2")}
    # a departure lane with its own width (or its own source lane) is not a mirror
    left2 = road_el.find("lanes/laneSection/left/lane[@id='2']")
    left2.find("width").set("a", "3.1")
    assert M.mirrored_lanes(road_el) == {(0, "1")}
    left2.find("width").set("a", "3.0")
    mirrored = M.mirrored_lanes(road_el)
    right1 = road_el.find("lanes/laneSection/right/lane[@id='-1']")
    right1.find("width").set("b", "0.002")   # refit changed the approach width
    assert M._remirror(road_el, mirrored) == 2
    assert M._widths(road_el.find("lanes/laneSection/left/lane[@id='1']")) == M._widths(right1)


def test_mirrored_boundaries_reflect_the_refitted_approach_about_the_centre_line():
    _, road = _road()
    fitted = {("right", 0, 0): [(0.0, 100.0, 0.5, 0.001, 0.0, 0.0)],
              ("right", 0, 1): [(0.0, 50.0, -3.0, 0.0, 0.0, 0.0), (50.0, 100.0, -3.0, 0.0, 1e-4, 0.0)],
              ("right", 0, 2): [(0.0, 100.0, -6.0, 0.0, 0.0, 0.0)]}
    assert M.mirror_fitted(road, fitted, {(0, "1")}) == 1   # left lane 2 is not a mirror: left boundary 2 stays
    assert ("left", 0, 2) not in fitted
    for x in np.linspace(0.0, 100.0, 21):
        centre = LR._at(fitted[("right", 0, 0)], x)
        assert LR._at(fitted[("left", 0, 1)], x) - centre == pytest.approx(centre - LR._at(fitted[("right", 0, 1)], x))


def test_refit_tree_moves_lane_centres_onto_the_map_points_and_keeps_the_mirror(tmp_path):
    root, _ = _road()
    # MAP lane 1 bends 0.6 m to the left over the middle of the road; lane 2 stays put
    s1 = np.array([[0.0, -1.75], [30.0, -1.75], [45.0, -1.15], [100.0, -1.15]])
    manifest = {"lanes": [{"source_lane_id": "S1", "geometry": {"coordinates": s1.tolist()}},
                          {"source_lane_id": "S2", "geometry": {"coordinates": _line(-5.0).tolist()}}]}
    path = tmp_path / "case.source-lanes.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    report = M.refit_tree(root, path)
    assert report["rewritten"] == 1 and not report["skipped"]
    road_el = root.find("road")
    road = LR._Road(road_el)
    for x, y1 in ((10.0, -1.75), (80.0, -1.15)):
        right = road.lane_edges_at(road_el, x, "right")
        assert (right[0] + right[1]) / 2 == pytest.approx(y1, abs=0.02)
        assert (right[1] + right[2]) / 2 == pytest.approx(-5.0, abs=0.02)
    left1 = road_el.find("lanes/laneSection/left/lane[@id='1']")
    right1 = road_el.find("lanes/laneSection/right/lane[@id='-1']")
    assert M._widths(left1) == M._widths(right1)
    rec = json.loads(road_el.find(f"userData[@code='{M.CODE}']").get("value"))
    assert rec["mirrored_boundaries"] >= 1 and rec["mirrored_lanes"] == 2
    ET.fromstring(etree.tostring(root))   # still a well-formed document


TAPER = """<OpenDRIVE><road id="12" length="120" junction="-1">
 <link><successor elementType="junction" elementId="1"/></link>
 <planView><geometry s="0" x="0" y="0" hdg="0" length="120"><line/></geometry></planView>
 <lanes>{sections}</lanes>
</road></OpenDRIVE>"""
TAPER_PROV = ('<userData code="mapforge.provenance/v1" value=\'{"eligibility":"excluded","exclusion_code":'
              '"lane-transition-taper","role":"approach","travel_direction":"with_s"}\'/>')


def _taper_road():
    def section(s, w2, sid2, link):
        return (f'<laneSection s="{s}"><center><lane id="0" type="none"/></center><right>'
                f'<lane id="-1" type="driving"><link>{link}</link><width sOffset="0" a="3.5" b="0" c="0" d="0"/>'
                '<userData code="mapforge.source_lane" value="S1"/></lane>'
                f'<lane id="-2" type="driving">{w2}<userData code="mapforge.source_lane" value="{sid2}"/>'
                f'{TAPER_PROV}</lane></right></laneSection>')
    zero = '<width sOffset="0" a="0" b="0" c="0" d="0"/>'
    # the bay opens over the last section (cubic 0 -> 3.5 m over 40 m), its MAP points start at 110 m
    opening = '<width sOffset="0" a="0" b="0" c="0.0065625" d="-0.0001093750"/>'
    pred, succ = '<predecessor id="-1"/>', '<successor id="-1"/>'
    secs = section(0, zero, "S2", succ) + section(40, zero, "S2", pred + succ) + section(80, opening, "S2", pred)
    return etree.fromstring(TAPER.format(sections=secs)).find("road")


def test_taper_pieces_are_linked_into_one_lane():
    road_el = _taper_road()
    assert M.link_tapers(road_el) == [(0, "right", "-2"), (1, "right", "-2")]
    secs = road_el.findall("lanes/laneSection")
    assert secs[0].find("right/lane[@id='-2']/link/successor").get("id") == "-2"
    assert secs[2].find("right/lane[@id='-2']/link/predecessor").get("id") == "-2"
    assert list(secs[1].find("right/lane[@id='-2']"))[0].tag == "link"   # link first, as the schema wants
    assert M.link_tapers(road_el) == []                                    # already linked
    other = _taper_road()
    other.findall("lanes/laneSection")[1].find("right/lane[@id='-2']/userData").set("value", "S9")
    assert M.link_tapers(other) == []                                      # different source lanes stay apart


def test_a_zero_width_taper_never_crosses_its_inner_boundary():
    road_el = _taper_road()
    M.link_tapers(road_el)
    # lane -1 zigzags 0.15 m about its xodr centre: fitted on its own, lane -2's outer edge would cross it
    s1 = np.array([[x, -1.75 + (0.15 if i % 2 else -0.15)] for i, x in enumerate(np.arange(0.0, 121.0, 12.0))])
    s2 = np.array([[110.0, -5.25], [120.0, -5.25]])
    observe = M.make_observe({"S1": s1, "S2": s2})
    params = {"mode": "g2", "c2_ends": True, "curb_shoulder": False, **M.MAP_LEVEL}
    road, fitted, report = LR.refit_road(road_el, None, None, params, observe=observe, relative=M.narrow_chain)
    assert report.get("relative_chains", 0) >= 1
    def width(i, x):
        return LR._at(fitted[("right", i, 1)], x) - LR._at(fitted[("right", i, 2)], x)
    # fitted as a width (a profile <= 0 relative to its inner boundary), the zero-width stretch stays within
    # the fit tolerance and never crosses: the RDP vertices lie on the profile and corners round outwards
    closed = [width(i, x) for i in (0, 1) for x in np.linspace(road.s[i], road.s[i + 1], 161)]
    assert min(closed) >= -1e-9 and max(closed) <= M.MAP_LEVEL["rdp_tol"]
    assert min(width(2, x) for x in np.linspace(80.0, 120.0, 161)) >= -1e-9
    centre = (LR._at(fitted[("right", 2, 1)], 110.0) + LR._at(fitted[("right", 2, 2)], 110.0)) / 2
    assert centre == pytest.approx(-5.25, abs=0.05)                                    # on its first MAP point
    assert LR.write_road(road, fitted, report)
