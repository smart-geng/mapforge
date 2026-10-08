"""Lane-centre fairness report items: peak curvature of through road lanes, S-bends of straight connectors."""
import xml.etree.ElementTree as ET

import pytest

from mapforge.validate import lane_fairness as F

# a cubic lateral step of 1 m over s 40-60: t'' = +-2c at its ends, c = 3 h / L^2
C, D = 3 * 1.0 / 20 ** 2, -2 * 1.0 / 20 ** 3


def _road(rid, junction, width=3.5):
    return (f'<road id="{rid}" length="100" junction="{junction}"><planView><geometry s="0" x="0" y="{rid}00" hdg="0" '
            f'length="100"><line/></geometry></planView><lanes>'
            f'<laneOffset s="0" a="0" b="0" c="0" d="0"/><laneOffset s="40" a="0" b="0" c="{C}" d="{D}"/>'
            f'<laneOffset s="60" a="1" b="0" c="0" d="0"/>'
            f'<laneSection s="0"><center><lane id="0" type="none"/></center><right><lane id="-1" type="driving">'
            f'<width sOffset="0" a="{width}" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road>')


def test_a_lateral_step_shows_as_peak_curvature_and_an_s_bend():
    root = ET.fromstring(f"<OpenDRIVE>{_road(1, '-1')}{_road(2, '7')}</OpenDRIVE>")
    m = F.audit(root)
    assert m["fair_road_kappa_max_per_m"] == pytest.approx(2 * C, rel=0.02)
    # the curvature rate is a finite difference over DS_M, as in G11-D: the step's curvature jumps at its ends
    # (a cubic step is only C1) read as jump / DS_M
    assert m["fair_road_sharpness_max_per_m2"] == pytest.approx(2 * C / F.DS_M, rel=0.05)
    assert m["fair_straight_conn_kappa_max_per_m"] == pytest.approx(2 * C, rel=0.02)
    assert m["fair_straight_conn_s_bends"] == 1
    assert "fair_turn_conn_sharpness_p90_per_m2" not in m


def test_narrow_and_changing_lanes_are_reported_apart():
    root = ET.fromstring(f"<OpenDRIVE>{_road(1, '-1', width=1.0)}</OpenDRIVE>")
    m = F.audit(root)
    assert "fair_road_kappa_max_per_m" not in m
    assert m["fair_road_event_kappa_max_per_m"] == pytest.approx(2 * C, rel=0.02)
