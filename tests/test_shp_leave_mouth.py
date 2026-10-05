"""SHP departure carriageways: the mouth moved past the via joints and the curb-return flare (shp_leave_mouth)."""
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import shp_leave_mouth as M

LAT0, LON0 = 29.5, 106.0
ORIGIN = (LAT0, LON0)


def _deg(xy):
    xy = np.asarray(xy, float)
    r = 6378137.0
    return np.column_stack([LON0 + np.degrees(xy[:, 0] / (r * math.cos(math.radians(LAT0)))),
                            LAT0 + np.degrees(xy[:, 1] / r)])


def _lane(lid, src, support, succ=None):
    link = f'<link><successor id="{succ}"/></link>' if succ is not None else "<link/>"
    prov = json.dumps({"status": "TRANSFORMED", "support_s": support}, sort_keys=True, separators=(",", ":"))
    return (f'<lane id="{lid}" type="driving" level="false">{link}<width sOffset="0" a="3.5" b="0" c="0" d="0"/>'
            f'<userData code="mapforge.source_lane" value="{src}"/>'
            f"<userData code=\"mapforge.provenance/v1\" value='{prov}'/></lane>")


def _connector(rid, y, src, succ):
    return (f'<road id="{rid}" length="20" junction="1"><link><successor elementType="road" elementId="30" '
            f'contactPoint="start"/></link><planView><geometry s="0" x="-20" y="{y}" hdg="0" length="20"><line/>'
            f'</geometry></planView><lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/><laneSection s="0">'
            f'<center><lane id="0" type="none" level="false"/></center>'
            f'<right>{_lane(-1, src, [0.0, 20.0], succ)}</right></laneSection></lanes></road>')


def _xodr():
    road = ('<road id="30" length="60" junction="-1"><link><predecessor elementType="junction" elementId="1"/></link>'
            '<planView><geometry s="0" x="0" y="0" hdg="0" length="60"><line/></geometry></planView>'
            '<lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/><laneSection s="0">'
            '<center><lane id="0" type="none" level="false"/></center>'
            f'<right>{_lane(-1, "D1", [0.0, 60.0])}{_lane(-2, "D2", [0.0, 60.0])}</right>'
            '</laneSection></lanes></road>')
    return etree.fromstring(
        f'<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0={LAT0} +lon_0={LON0}]]></geoReference>'
        f'</header>{road}{_connector(100, 0.0, "V1", -1)}{_connector(101, -3.5, "V2", -2)}</OpenDRIVE>')


class FakeSource:
    """Via lines V1, V2 ending ``joint`` metres inside road 30 on its departure lanes D1, D2; D2's outer
    boundary flares 1 m wider over the first 4.5 m when ``flare``."""

    def __init__(self, joint=(1.0, 1.2), flare=False, taper_at=None):
        j1, j2 = joint
        outer = [(j2, -8.0), (j2 + 4.5, -7.0), (60.0, -7.0)] if flare else [(j2, -7.0), (60.0, -7.0)]
        if taper_at is not None:                   # D2's outer boundary only from taper_at on, widening there
            outer = [(taper_at, -8.0), (taper_at + 4.5, -7.0), (60.0, -7.0)]
        self.geom = {"V1": [(-20.0, -1.75), (j1, -1.75)], "V2": [(-20.0, -5.25), (j2, -5.25)],
                     "D1": [(j1, -1.75), (60.0, -1.75)], "D2": [(j2, -5.25), (60.0, -5.25)]}
        self.bnds = {"D1": [[(j1, 0.0), (60.0, 0.0)], [(j1, -3.5), (60.0, -3.5)]],
                     "D2": [[(j2, -3.5), (60.0, -3.5)], outer], "V1": [], "V2": []}

    def lane(self, sid):
        if sid not in self.geom:
            return None
        return SimpleNamespace(geometry=_deg(self.geom[sid]), link_pid="L" if sid.startswith("D") else "J")

    def lane_boundary_geometries(self, sid):
        return [_deg(b) for b in self.bnds.get(sid, [])]


def _window(sid, coords):
    return {"source_lane_id": sid, "geometry": {"coordinates": [list(c) for c in coords]},
            "travel": {"start": list(coords[0]), "end": list(coords[-1])}}


def _support(lane):
    return json.loads(lane.find("userData[@code='mapforge.provenance/v1']").get("value"))["support_s"]


def test_flare_end_is_where_the_boundary_slope_settles():
    s = np.linspace(1.2, 30.0, 200)
    straight = np.column_stack([s, np.full_like(s, -7.0)])
    assert M.flare_end(straight) is None
    drift = np.column_stack([s, -7.0 + 0.03 * s])                  # a steady slope is no flare
    assert M.flare_end(drift) is None
    flared = np.column_stack([s, np.where(s < 5.7, -8.0 + (s - 1.2) / 4.5, -7.0)])
    assert M.flare_end(flared) == pytest.approx(5.7, abs=1e-9)


def test_plan_moves_the_road_past_its_joints_and_its_flare():
    root = _xodr()
    [item] = M.plan(root, FakeSource(), ORIGIN)
    assert item["road"] == "30" and item["joints"] == {"100": pytest.approx(1.0), "101": pytest.approx(1.2)}
    assert item["shift_m"] == pytest.approx(1.2 + M.ROOM_M) and item["flare_end_m"] is None
    [item] = M.plan(root, FakeSource(flare=True), ORIGIN)
    assert item["shift_m"] == pytest.approx(5.7 + M.ROOM_M) and item["flare_end_m"] == pytest.approx(5.7)
    # a boundary that begins further on (a lane taper) is no curb-return flare
    [item] = M.plan(root, FakeSource(taper_at=3.0), ORIGIN)
    assert item["shift_m"] == pytest.approx(1.2 + M.ROOM_M) and item["flare_end_m"] is None
    # joints already well inside the connectors: nothing moves
    assert M.plan(root, FakeSource(joint=(-4.0, -4.0)), ORIGIN) == []


def test_apply_cuts_the_road_lengthens_the_connectors_and_moves_the_windows():
    root = _xodr()
    manifest = {"lanes": [_window("V1", [(-20.0, -1.75), (0.0, -1.75)]),
                          _window("V2", [(-20.0, -5.25), (0.0, -5.25)]),
                          _window("D1", [(1.0, -1.75), (60.0, -1.75)]),
                          _window("D2", [(1.2, -5.25), (60.0, -5.25)])]}
    [rep] = M.apply(root, manifest, FakeSource(), ORIGIN)
    d = 1.2 + M.ROOM_M
    assert rep["shift_m"] == pytest.approx(d) and rep["new_mouth"] == pytest.approx([d, 0.0], abs=1e-3)
    assert sorted(rep["windows"]["extended"]) == ["V1", "V2"] and sorted(rep["windows"]["cropped"]) == ["D1", "D2"]

    road = root.find("road[@id='30']")
    assert float(road.get("length")) == pytest.approx(60.0 - d)
    g = road.find("planView/geometry")
    assert float(g.get("x")) == pytest.approx(d) and float(g.get("length")) == pytest.approx(60.0 - d)
    for lane in road.iter("lane"):
        if lane.get("id") != "0":
            assert _support(lane) == pytest.approx([0.0, 60.0 - d])

    for rid, y in (("100", 0.0), ("101", -3.5)):
        conn = root.find(f"road[@id='{rid}']")
        assert float(conn.get("length")) == pytest.approx(20.0 + d)
        last = conn.findall("planView/geometry")[-1]
        assert last[0].tag == "line" and float(last.get("s")) == pytest.approx(20.0)
        assert (float(last.get("x")), float(last.get("y"))) == pytest.approx((0.0, y))
        assert float(last.get("length")) == pytest.approx(d)
        lane = conn.find("lanes/laneSection/right/lane")
        widths = lane.findall("width")
        assert [float(w.get("sOffset")) for w in widths] == pytest.approx([0.0, 20.0])
        assert float(widths[-1].get("a")) == pytest.approx(3.5)
        assert _support(lane) == pytest.approx([0.0, 20.0 + d])

    lanes = {lane["source_lane_id"]: lane for lane in manifest["lanes"]}
    via = np.asarray(lanes["V1"]["geometry"]["coordinates"])
    assert via[-1] == pytest.approx([d, -1.75], abs=1e-6)
    assert any(np.allclose(p, [1.0, -1.75]) for p in via)          # the joint stays a vertex of the window
    assert lanes["V1"]["travel"]["end"] == pytest.approx([d, -1.75], abs=1e-6)
    dep = np.asarray(lanes["D2"]["geometry"]["coordinates"])
    assert dep[0] == pytest.approx([d, -5.25], abs=1e-6) and dep[-1] == pytest.approx([60.0, -5.25])
    assert lanes["D2"]["window_adjustments"] == [{"code": M.CODE, "mouth_shift_m": round(d, 3)}]
    assert "manifest_sha256" in manifest


def test_window_extension_starts_from_the_segment_the_window_ends_on():
    """A window cut between two vertices of its via line runs on through the next vertex (no chord)."""
    lane = _window("V", [(-10.0, 0.0), (-2.0, 0.0)])
    path = np.array([(-10.0, 0.0), (-1.0, 0.0), (0.0, 0.5), (8.0, 0.5)])
    assert M._extend_window(lane, path, np.array([4.0, 0.5]), 0.0, 4.0)
    assert np.asarray(lane["geometry"]["coordinates"]) == pytest.approx(
        np.array([(-10.0, 0.0), (-2.0, 0.0), (-1.0, 0.0), (0.0, 0.5), (4.0, 0.5)]))
