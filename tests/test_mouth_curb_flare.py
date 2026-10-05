"""Curb-return flares at junction mouths: the source review rule and the flare-free lane-centre report."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree

from mapforge.validate import lane_centre_flare as F
from mapforge.validate import shp_source_review as R

ORIGIN = {"lon": 106.0, "lat": 29.5}


def _deg(xy):
    xy = np.asarray(xy, float)
    r = 6378137.0
    lon = ORIGIN["lon"] + np.degrees(xy[:, 0] / (r * math.cos(math.radians(ORIGIN["lat"]))))
    lat = ORIGIN["lat"] + np.degrees(xy[:, 1] / r)
    return np.column_stack([lon, lat])


def _lane(lid, src):
    return (f'<lane id="{lid}" type="driving" level="false"><link/><width sOffset="0" a="3.5" b="0" c="0" d="0"/>'
            f'<userData code="mapforge.source_lane" value="{src}"/></lane>')


def _xodr():
    """Road 10 (x 0..60 along y = 0) ends at junction 1; right lanes -1 (A1) and -2 (A2); connector 100 leaves
    lane -2 at that end."""
    road = ('<road id="10" length="60" junction="-1"><link><successor elementType="junction" elementId="1"/></link>'
            '<planView><geometry s="0" x="0" y="0" hdg="0" length="60"><line/></geometry></planView>'
            '<lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/><laneSection s="0">'
            '<center><lane id="0" type="none" level="false"/></center>'
            f'<right>{_lane(-1, "A1")}{_lane(-2, "A2")}</right></laneSection></lanes></road>')
    conn = ('<road id="100" length="20" junction="1"><link><predecessor elementType="road" elementId="10" '
            'contactPoint="end"/></link><planView><geometry s="0" x="60" y="-5.25" hdg="0" length="20"><line/>'
            '</geometry></planView><lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center>'
            '<right><lane id="-1" type="driving" level="false"><link><predecessor id="-2"/></link>'
            '<width sOffset="0" a="3.5" b="0" c="0" d="0"/><userData code="mapforge.source_lane" value="V1"/>'
            '</lane></right></laneSection></lanes></road>')
    return etree.fromstring(f"<OpenDRIVE><header/>{road}{conn}</OpenDRIVE>")


class FakeSource:
    """A2 (x 52..63, continued upstream by A2p) widens 2 m towards its end and its centreline swings 1 m out;
    A1 (x 0..63) moves 0.5 m outward near its end as a whole (both boundaries with it): no flare."""

    def __init__(self):
        self.lanes, self.bnds = {}, {}
        self.topo_in, self.topo_out = {"A2": ["A2p"]}, {"A2p": ["A2"]}
        self._add("A2p", [(0.0, -5.25), (52.0, -5.25)], [(0.0, -3.5), (52.0, -3.5)], [(0.0, -7.0), (52.0, -7.0)])
        self._add("A2", [(52.0, -5.25), (55.0, -5.25), (63.0, -6.25)], [(52.0, -3.5), (63.0, -3.5)],
                  [(52.0, -7.0), (55.0, -7.0), (63.0, -9.0)])
        self._add("A1", [(0.0, -1.75), (55.0, -1.75), (63.0, -2.25)], [(0.0, 0.0), (55.0, 0.0), (63.0, -0.5)],
                  [(0.0, -3.5), (55.0, -3.5), (63.0, -4.0)])

    def _add(self, sid, centre, inner, outer):
        self.lanes[sid] = SimpleNamespace(geometry=_deg(centre), geometry_source="field", link_pid="L-" + sid)
        self.bnds[sid] = [_deg(inner), _deg(outer)]

    def lane(self, sid):
        return self.lanes.get(sid)

    def lane_boundary_geometries(self, sid):
        return self.bnds.get(sid, [])


def test_a_lane_widening_with_a_swinging_centreline_at_a_mouth_is_recorded():
    root = _xodr()
    meta = {"A2": {"roles": ["approach"], "g8_comparable": True}}
    (f,) = R.mouth_flares(root, FakeSource(), ORIGIN, meta)
    assert f["rule"] == "mouth-curb-flare" and f["source_lane_id"] == "A2" and f["resolution"] == "regular-mouth"
    assert (f["road"], f["contact"], f["lane"], f["travel"]) == ("10", "end", -2, "approach")
    assert f["widening_m"] == pytest.approx(2.0, abs=0.05) and f["swing_m"] == pytest.approx(1.0, abs=0.05)
    assert f["mouth_widening_m"] == pytest.approx(1.25, abs=0.05)       # 5 of the 8 flaring metres lie before it
    assert f["lane_end_from_mouth_m"] == pytest.approx(-3.0, abs=0.01)
    assert f["flare_from_mouth_m"] == pytest.approx(4.5, abs=0.5)       # (0.05 m out about 0.4 m into the flare)
    assert f["connectors"] == ["100"] and f["connector_lanes"] == ["V1"]
    assert (f["mouth"]["x"], f["mouth"]["y"]) == pytest.approx((60.0, -5.25), abs=1e-3)
    assert f["roles"] == ["approach"] and f["g8_comparable"] is True


def test_a_lane_moving_out_as_a_whole_is_no_flare_and_a_regular_lane_neither():
    src = FakeSource()
    findings = R.mouth_flares(_xodr(), src, ORIGIN)
    assert [f["source_lane_id"] for f in findings] == ["A2"]            # A1 shifts 0.5 m, its width stays
    # the same lane without its widening: nothing
    src._add("A2", [(52.0, -5.25), (63.0, -5.25)], [(52.0, -3.5), (63.0, -3.5)], [(52.0, -7.0), (63.0, -7.0)])
    assert R.mouth_flares(_xodr(), src, ORIGIN) == []


def test_flare_free_lane_centre_statistics_leave_out_the_flared_mouth(monkeypatch):
    via = [(60.0, -6.25), (60.0, -5.25), (80.0, -5.25)]     # (1 m steps on both lines: no sampling phase)
    manifest = {"lanes": [
        {"source_lane_id": "A2", "comparison": {"eligible": True},
         "geometry": {"coordinates": [(0.0, -5.25), (55.0, -5.25), (63.0, -6.25)]}},
        {"source_lane_id": "V1", "comparison": {"eligible": True}, "geometry": {"coordinates": via}}]}
    components = [{"source_lane_id": "A2", "points": [(0.0, -5.25), (60.0, -5.25)]},
                  {"source_lane_id": "V1", "points": [(60.0, -5.25), (80.0, -5.25)]}]
    monkeypatch.setattr(F, "extract_target_components", lambda root, **kw: {"components": components})
    review = {"findings": [{"rule": "mouth-curb-flare", "source_lane_id": "A2", "connector_lanes": ["V1"],
                            "mouth": {"x": 60.0, "y": -5.25}}]}
    plain = F.audit(None, manifest, {"findings": []})
    assert plain["lane_center_noflare_max_m"] > 0.9 and plain["lane_center_flare_samples_left_out"] == 0
    out = F.audit(None, manifest, review)
    assert out["mouth_curb_flares"] == 1 and out["lane_center_flare_samples_left_out"] > 0
    assert out["lane_center_noflare_max_m"] < 1e-6                   # all of the swing lies within 10 m of the mouth
    assert F.zones(review) == {"A2": [(60.0, -5.25)], "V1": [(60.0, -5.25)]}
