"""MAP approach sides faired within a band of their MAP point lists."""
import json

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import map_approach_fair as F
from mapforge.validate.smoothness import lane_edges_kinematics_at

APP = json.dumps({"role": "approach", "status": "TRANSFORMED", "support_kind": "map-lane-point-list"})
# centre line: straight, then a 1 m cubic blend into a 1:10 slope (a corner at a MAP point, s = 50)
C, D = 0.1, -0.1 / 3
T1 = C + D            # offset at the end of the blend (s = 50.5)


def _road():
    lanes = "".join(f'<lane id="{lid}" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>'
                    f'<userData code="mapforge.source_lane" value="S{-lid}"/>'
                    f'<userData code="mapforge.provenance/v1" value=\'{APP}\'/></lane>' for lid in (-1, -2))
    xml = ('<OpenDRIVE><road id="10" length="100" junction="-1"><link><successor elementType="junction" '
           'elementId="1"/></link><planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry>'
           '</planView><lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>'
           f'<laneOffset s="49.5" a="0" b="0" c="{C}" d="{D}"/><laneOffset s="50.5" a="{T1}" b="0.1" c="0" d="0"/>'
           f'<laneSection s="0"><center><lane id="0" type="none"/></center><right>{lanes}</right></laneSection>'
           '</lanes></road></OpenDRIVE>')
    return etree.fromstring(xml)


def _centres():
    end = T1 + 0.1 * 49.5
    return {f"S{k}": np.array([[0.0, -1.75 - 3.5 * (k - 1)], [50.0, -1.75 - 3.5 * (k - 1)],
                               [100.0, end - 1.75 - 3.5 * (k - 1)]]) for k in (1, 2)}


def _lane_centre(road, x, k):
    e = lane_edges_kinematics_at(road, x, "right")
    return (e[k - 1][0] + e[k][0]) / 2, abs((e[k - 1][2] + e[k][2]) / 2)


def _source_offset(x, k):
    line = _centres()[f"S{k}"]
    return float(np.interp(x, line[:, 0], line[:, 1]))


def test_a_sharp_corner_at_a_map_point_is_rounded_within_the_band():
    root = _road()
    road = root.find("road")
    xs = np.arange(0.25, 100, 0.25)
    before = [(_lane_centre(road, x, 1), _lane_centre(road, x, 2)) for x in xs]
    ends = [lane_edges_kinematics_at(road, x, "right") for x in (1e-6, 100 - 1e-6)]
    report = F.apply(root, _centres())
    road = root.find("road")
    assert report["faired"] == 1, report
    after = [(_lane_centre(road, x, 1), _lane_centre(road, x, 2)) for x in xs]
    for x, b, a in zip(xs, before, after):
        for k in (1, 2):
            src = _source_offset(x, k)
            allow = max(F.TAU_M, abs(b[k - 1][0] - src))
            assert abs(a[k - 1][0] - src) <= allow + F.BAND_TOL_M
    # the bend at the corner drops to a fraction (1 m blend of a 1:10 turn, then a corner within 5 cm)
    assert max(a[0][1] for a in after) < 0.35 * max(b[0][1] for b in before)
    # both road ends keep the centre line and the lane boundaries with their slopes and curvatures
    for x, old in zip((1e-6, 100 - 1e-6), ends):
        assert np.array(lane_edges_kinematics_at(road, x, "right")) == pytest.approx(np.array(old), abs=1e-6)
    row = json.loads(road.find(f"userData[@code='{F.CODE}']").get("value"))
    assert "skipped" not in row and row["centre_shift_max_m"] <= F.TAU_M + F.BAND_TOL_M + 0.03


def test_a_straight_approach_stays_as_it_is():
    root = _road()
    road = root.find("road")
    for el in road.findall("lanes/laneOffset")[1:]:
        road.find("lanes").remove(el)
    line = {f"S{k}": np.array([[0.0, -1.75 - 3.5 * (k - 1)], [100.0, -1.75 - 3.5 * (k - 1)]]) for k in (1, 2)}
    before = [lane_edges_kinematics_at(road, x, "right") for x in np.arange(0.5, 100, 5.0)]
    report = F.apply(root, line)
    road = root.find("road")
    after = [lane_edges_kinematics_at(road, x, "right") for x in np.arange(0.5, 100, 5.0)]
    assert np.array(after) == pytest.approx(np.array(before), abs=1e-6)
    assert report["faired"] == 0 or report["roads"][0]["centre_bend_max_per_m"][1] <= 1e-6
