"""MAP free road ends extended to the MAP lane end points beyond them."""
import json
import math

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import map_far_end as F
from mapforge.validate.smoothness import lane_edges_at, sample_road_ref

ROAD = """<OpenDRIVE><road id="12" length="{length}" junction="-1">
 <link>{link}</link>
 <planView>{geometry}</planView>
 <lanes>
  <laneOffset s="0" a="0.2" b="0.01" c="0" d="0"/>
  <laneOffset s="30" a="0.5" b="0" c="0" d="0"/>
  <laneSection s="0">
   <left><lane id="1" type="driving"><width sOffset="0" a="3.5" b="0.002" c="0" d="0"/>
    <roadMark sOffset="0" type="solid" weight="standard" color="standard" width="0.15"/></lane></left>
   <center><lane id="0" type="none"/></center>
   <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0.002" c="0" d="0"/>
     <width sOffset="20" a="3.54" b="0" c="0" d="0"/>
     <roadMark sOffset="0" type="solid" weight="standard" color="standard" width="0.15"/>
     <speed sOffset="5" max="50" unit="km/h"/>
     <userData code="mapforge.source_lane" value="S1"/>
     <userData code="mapforge.provenance/v1" value='{{"eligibility":"comparable","support_s":[0.0,40.0],"travel_direction":"with_s"}}'/></lane></right>
  </laneSection>
  <laneSection s="40">
   <center><lane id="0" type="none"/></center>
   <right><lane id="-1" type="driving"><width sOffset="0" a="3.54" b="0" c="0" d="0"/>
     <userData code="mapforge.source_lane" value="S1"/></lane></right>
  </laneSection>
 </lanes>
</road></OpenDRIVE>"""
LINE = '<geometry s="0" x="0" y="0" hdg="0" length="{length}"><line/></geometry>'
SPIRAL = ('<geometry s="0" x="0" y="0" hdg="0.3" length="30"><spiral curvStart="0.004" curvEnd="0.02"/></geometry>'
          '<geometry s="30" x="{x1}" y="{y1}" hdg="{h1}" length="30"><arc curvature="0.02"/></geometry>')
JUNCTION = '<successor elementType="junction" elementId="1"/>'


def _road(geometry=None, link=JUNCTION, length=60.0):
    xml = ROAD.format(length=length, link=link, geometry=geometry or LINE.format(length=length))
    return etree.fromstring(xml).find("road")


def _centre(road, s):
    e = lane_edges_at(road, s, "right")
    return (e[0] + e[1]) / 2


def test_a_free_start_reaches_the_first_map_point_and_keeps_everything_else():
    road = _road()
    before = etree.fromstring(etree.tostring(road))
    # MAP lane -1 begins 0.3 m before the road start (its first point is not on the road's first section)
    centres = {"S1": np.array([[-0.3, -1.55], [60.0, -1.95]])}
    record = F.extend(road, centres, sample_road_ref(road, 0.25))
    assert record["start"]["extended_m"] == pytest.approx(0.3)
    assert "end" not in record                                   # the junction end is never touched
    geom = road.find("planView/geometry")
    assert float(geom.get("x")) == pytest.approx(-0.3) and float(geom.get("length")) == pytest.approx(60.3)
    assert float(road.get("length")) == pytest.approx(60.3)
    assert [float(sec.get("s")) for sec in road.findall("lanes/laneSection")] == pytest.approx([0.0, 40.3])
    for s in (0.0, 7.5, 20.0, 35.0, 50.0):                        # same lane geometry at the same places
        assert _centre(road, s + 0.3) == pytest.approx(_centre(before, s), abs=1e-12)
        assert lane_edges_at(road, s + 0.3, "left") == pytest.approx(lane_edges_at(before, s, "left"), abs=1e-12)
    lane = road.find("lanes/laneSection/right/lane")
    assert float(lane.find("speed").get("sOffset")) == pytest.approx(5.3)
    assert float(lane.find("roadMark").get("sOffset")) == 0.0
    prov = json.loads(lane.find("userData[@code='mapforge.provenance/v1']").get("value"))
    assert prov["support_s"] == pytest.approx([0.0, 40.3])


def test_a_spiral_start_is_continued_along_its_own_curve():
    x1 = y1 = h1 = 0.0
    # end pose of the first spiral, from the sampler itself
    probe = _road(SPIRAL.format(x1=0, y1=0, h1=0))
    pts, ss, hh = sample_road_ref(probe, 0.01)
    i = int(np.argmin(np.abs(ss - 30.0)))
    x1, y1, h1 = pts[i][0], pts[i][1], hh[i]
    road = _road(SPIRAL.format(x1=x1, y1=y1, h1=h1))
    old = sample_road_ref(road, 0.05)
    record = F.extend(road, {"S1": np.array([[-0.6 * math.cos(0.3), -0.6 * math.sin(0.3)], [50.0, 20.0]])}, old)
    delta = record["start"]["extended_m"]
    assert delta == pytest.approx(0.6, abs=0.01)
    new = sample_road_ref(road, 0.05)
    for s in (0.0, 10.0, 29.0):
        a = np.array([np.interp(s, old[1], old[0][:, 0]), np.interp(s, old[1], old[0][:, 1])])
        b = np.array([np.interp(s + delta, new[1], new[0][:, 0]), np.interp(s + delta, new[1], new[0][:, 1])])
        assert np.linalg.norm(a - b) < 1e-6
    spiral = road.find("planView/geometry/spiral")
    rate = (0.02 - 0.004) / 30.0
    assert float(spiral.get("curvStart")) == pytest.approx(0.004 - rate * delta)
    assert float(road.findall("planView/geometry")[1].get("s")) == pytest.approx(30.0 + delta)


def test_small_gaps_and_linked_ends_are_left_alone():
    road = _road()
    assert F.extend(road, {"S1": np.array([[-0.02, -1.55], [60.0, -1.95]])}, sample_road_ref(road, 0.25)) is None
    linked = _road(link='<predecessor elementType="road" elementId="9" contactPoint="end"/>' + JUNCTION)
    assert F.extend(linked, {"S1": np.array([[-0.5, -1.55], [60.0, -1.95]])}, sample_road_ref(linked, 0.25)) is None
    assert float(linked.get("length")) == 60.0


def test_a_free_far_end_at_the_road_end_is_continued_too():
    # junction at the start, free end: the MAP lane runs 0.4 m beyond the road end
    road = _road(link='<predecessor elementType="junction" elementId="1"/>')
    record = F.extend(road, {"S1": np.array([[0.0, -1.55], [60.4, -1.95]])}, sample_road_ref(road, 0.25))
    assert record["end"]["extended_m"] == pytest.approx(0.4) and "start" not in record
    assert float(road.get("length")) == pytest.approx(60.4)
    assert float(road.find("planView/geometry").get("x")) == 0.0          # the start does not move
    prov = json.loads(road.find("lanes/laneSection/right/lane/userData[@code='mapforge.provenance/v1']").get("value"))
    assert prov["support_s"] == pytest.approx([0.0, 40.0])                # support inside the road is unchanged
