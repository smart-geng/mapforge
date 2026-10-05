"""Gap lanes between the lanes of two parallel source links (lane_gap)."""
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_gap as G
from mapforge.ops.lane_refit import _Road

LAT0, LON0 = 29.5, 106.0


def _deg(xy):
    xy = np.asarray(xy, float)
    r = 6378137.0
    return np.column_stack([LON0 + np.degrees(xy[:, 0] / (r * math.cos(math.radians(LAT0)))),
                            LAT0 + np.degrees(xy[:, 1] / r)])


def _lane(lid, width, src, link=None):
    links = "".join(f'<{tag} id="{lid}"/>' for tag in (link or []))
    return (f'<lane id="{lid}" type="driving" level="false"><link>{links}</link>'
            f'<width sOffset="0" a="{width}" b="0" c="0" d="0"/>'
            f'<userData code="mapforge.source_lane" value="{src}"/></lane>')


def _xodr(n_sections=2):
    secs = []
    for i in range(n_sections):
        tags = (["predecessor"] if i > 0 else []) + (["successor"] if i < n_sections - 1 else [])
        left = "".join(_lane(k, 3.5, f"L{k}", tags) for k in (3, 2, 1))
        secs.append(f'<laneSection s="{i * 25.0}"><left>{left}</left>'
                    f'<center><lane id="0" type="none" level="false"/></center><right/></laneSection>')
    road = (f'<road id="10" length="50" junction="-1"><link><successor elementType="junction" elementId="1"/></link>'
            f'<planView><geometry s="0" x="0" y="0" hdg="0" length="50"><line/></geometry></planView>'
            f'<lanes>{"".join(secs)}</lanes></road>')
    conn = ('<road id="100" length="20" junction="1"><link><predecessor elementType="road" elementId="11" '
            'contactPoint="end"/><successor elementType="road" elementId="10" contactPoint="end"/></link>'
            '<planView><geometry s="0" x="0" y="0" hdg="0" length="20"><line/></geometry></planView>'
            '<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center><right>'
            '<lane id="-1" type="driving" level="false"><link><predecessor id="-1"/><successor id="3"/></link>'
            '<width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road>')
    return etree.fromstring(
        f'<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0={LAT0} +lon_0={LON0}]]></geoReference>'
        f'</header>{road}{conn}</OpenDRIVE>')


class FakeSource:
    """Lanes 1 and 2 on link A (3.5 m each), lane 3 on link B, ``gap`` metres further out."""

    def __init__(self, gap):
        edges = {1: (0.0, 3.5), 2: (3.5, 7.0), 3: (7.0 + gap, 10.5 + gap)}
        self.recs = {f"L{k}": SimpleNamespace(link_pid="A" if k < 3 else "B") for k in edges}
        self.bnds = {f"L{k}": [_deg([(-2.0, t), (52.0, t)]) for t in edges[k]] for k in edges}

    def lane(self, sid):
        return self.recs.get(sid)

    def lane_boundary_geometries(self, sid):
        return self.bnds[sid]


def test_gap_between_two_links_is_found_and_becomes_a_lane():
    root = _xodr()
    road_el = root.find("road[@id='10']")
    origin = (LAT0, LON0)
    gaps = G.find(_Road(road_el), FakeSource(1.0), origin)
    assert [(g["side"], g["k"]) for g in gaps] == [("left", 2)]
    assert gaps[0]["separation_m"] == pytest.approx([1.0, 1.0], abs=0.02)
    G.insert(root, road_el, "left", 2, gaps[0]["separation_m"])
    for i, sec in enumerate(road_el.findall("lanes/laneSection")):
        lanes = sec.findall("left/lane")
        assert [ln.get("id") for ln in lanes] == ["4", "3", "2", "1"]
        gap = lanes[1]
        assert gap.get("type") == G.GAP_TYPE and gap.find("userData[@code='mapforge.source_lane']") is None
        prov = json.loads(gap.find("userData[@code='mapforge.provenance/v1']").get("value"))
        assert prov["exclusion_code"] == "physical-edge-fill" and prov["status"] == "INFERRED"
        assert lanes[0].find("userData[@code='mapforge.source_lane']").get("value") == "L3"
        if i == 0:
            assert lanes[0].find("link/successor").get("id") == "4"
        else:
            assert lanes[0].find("link/predecessor").get("id") == "4"
    # the connector that ended on lane 3 now ends on lane 4
    assert root.find("road[@id='100']/lanes/laneSection/right/lane/link/successor").get("id") == "4"


def test_touching_links_get_no_gap_lane():
    root = _xodr()
    assert G.find(_Road(root.find("road[@id='10']")), FakeSource(0.05), (LAT0, LON0)) == []
