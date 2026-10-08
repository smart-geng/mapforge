"""MAP departure sides eased: a mirrored bay opening beside an approach bay, mouths and approach side kept."""
import json

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import map_departure_ease as M
from mapforge.validate.smoothness import lane_edges_kinematics_at

DEP = json.dumps({"role": "departure", "status": "INFERRED", "support_kind": "mirror"})
APP = json.dumps({"role": "approach", "status": "TRANSFORMED", "support_kind": "map-lane-point-list"})
# a cubic step 0 -> 3 m over s 60-75 (the approach bay opening on the inside pushes the centre line out)
C, D = 3 * 3 / 15 ** 2, -3 * 2 / 15 ** 3


def _lane(lid, width_records, prov, link=""):
    widths = "".join(f'<width sOffset="{s}" a="{a}" b="{b}" c="{c}" d="{d}"/>' for s, a, b, c, d in width_records)
    return f'<lane id="{lid}" type="driving">{link}{widths}<userData code="mapforge.provenance/v1" value=\'{prov}\'/></lane>'


def _road(bay=True):
    """120 m straight road, junction at its end; sections [0, 40], [40, 60], [60, 120]."""
    rise = [(0, 0, 0, C, D), (15, 3, 0, 0, 0)] if bay else [(0, 0, 0, 0, 0)]
    offsets = ('<laneOffset s="0" a="0" b="0" c="0" d="0"/>'
               + (f'<laneOffset s="60" a="0" b="0" c="{C}" d="{D}"/><laneOffset s="75" a="3" b="0" c="0" d="0"/>'
                  if bay else ""))
    zero = [(0, 0, 0, 0, 0)]
    through = [(0, 3.5, 0, 0, 0)]
    secs = []
    for i, s in enumerate((0, 40, 60)):
        last = i == 2
        pre = lambda lid: f'<predecessor id="{lid}"/>' if i else ""
        suc = lambda lid: f'<successor id="{lid}"/>' if not last else ""
        link = lambda lid: f"<link>{pre(lid)}{suc(lid)}</link>"
        left = (_lane(1, rise if last else zero, DEP) + _lane(2, through, DEP, link(2)) + _lane(3, through, DEP, link(3)))
        right = (_lane(-1, rise if last else zero, APP) + _lane(-2, through, APP, link(-2))
                 + _lane(-3, through, APP, link(-3)))
        secs.append(f'<laneSection s="{s}"><left>{left}</left><center><lane id="0" type="none"/></center>'
                    f'<right>{right}</right></laneSection>')
    xml = (f'<OpenDRIVE><road id="10" length="120" junction="-1"><link><successor elementType="junction" '
           f'elementId="1"/></link><planView><geometry s="0" x="0" y="0" hdg="0" length="120"><line/></geometry>'
           f'</planView><lanes>{offsets}{"".join(secs)}</lanes></road></OpenDRIVE>')
    return etree.fromstring(xml)


def _bend(road, xs):
    """Largest |t''| of the departure through lanes (+2, +3) over xs."""
    worst = 0.0
    for x in xs:
        e = lane_edges_kinematics_at(road, float(x), "left")
        for a, b in zip(e[-3:], e[-2:]):
            worst = max(worst, abs((a[2] + b[2]) / 2))
    return worst


def test_a_mirrored_bay_opens_earlier_and_the_departure_lanes_swing_gently():
    root = _road()
    road = root.find("road")
    before = etree.tostring(road)
    right_before = [etree.tostring(s.find("right")) for s in road.findall("lanes/laneSection")]
    xs = np.arange(0.5, 119.5, 0.5)
    bend_before = _bend(road, xs)
    mouth_before = lane_edges_kinematics_at(road, 120 - 1e-6, "left")
    start_before = lane_edges_kinematics_at(road, 1e-6, "left")
    report = M.apply(root)
    road = root.find("road")
    assert report["eased"] == 1 and etree.tostring(road) != before
    # the approach side and the centre line are untouched
    assert [etree.tostring(s.find("right")) for s in road.findall("lanes/laneSection")] == right_before
    assert [o.attrib for o in road.findall("lanes/laneOffset")] == [o.attrib for o in _road().find("road").findall(
        "lanes/laneOffset")]
    # the mouth and the far end keep every departure boundary's value, slope and curvature
    for x, old in ((120 - 1e-6, mouth_before), (1e-6, start_before)):
        new = lane_edges_kinematics_at(road, x, "left")
        assert np.array(new) == pytest.approx(np.array(old), abs=1e-6)
    # widths never negative; the bay mirror opens at most PAD_M earlier than its birth at 60 m
    for x in np.arange(0.25, 120, 0.25):
        e = [t for t, _, _ in lane_edges_kinematics_at(road, float(x), "left")]
        assert min(np.diff(e)) >= -1e-6
        if x < 60 - M.PAD_M - 1e-6:
            assert e[1] - e[0] == pytest.approx(0.0, abs=1e-3)
    # the departure through lanes bend less: they no longer move 6 m within 15 m. A bay that only widens and never
    # gets wider than at the mouth cannot let them bend less than the centre line itself (t0'' up to 2C) as it rises
    after = _bend(road, xs)
    assert after < 0.75 * bend_before and after < 1.4 * 2 * C
    rows = json.loads(road.find(f"userData[@code='{M.CODE}']").get("value"))
    assert len(rows) == 1 and "skipped" not in rows[0] and rows[0]["opened_earlier_m"][0] > 0.0
    # no links were added: the unlinked zero-width pieces stay unlinked
    assert road.findall("lanes/laneSection")[2].find("left/lane[@id='1']/link") is None


def test_a_straight_departure_side_is_left_alone():
    root = _road(bay=False)
    before = etree.tostring(root)
    report = M.apply(root)
    assert report["eased"] == 0 and report["roads"] == []
    assert etree.tostring(root) == before
