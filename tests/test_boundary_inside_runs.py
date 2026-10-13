"""boundary_inside_runs: an SHP lane's outer-edge records paired run by run where records in between are left out
(user decision 2026-10-13, SHP T2 from 0.11-draft). Synthetic road: a left-curving arc (radius 50 m) with one right
lane 3.5 m wide in three laneSections of 20 m; the source boundary is the circle the lane's outer edge lies on."""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from mapforge.validate import boundary_inside as BI
from mapforge.validate import boundary_inside_runs as R
from mapforge.validate import scoreboard as sb
from mapforge.validate.shp_boundary_fidelity import R_EARTH

LAT0, LON0 = 29.5, 106.3
RADIUS, WIDTH, LENGTH = 50.0, 3.5, 60.0
TRUSTED = {"boundary_evidence_trusted": True}


def _lonlat(xy):
    xy = np.asarray(xy, float)
    lon = LON0 + np.degrees(xy[:, 0] / (R_EARTH * math.cos(math.radians(LAT0))))
    lat = LAT0 + np.degrees(xy[:, 1] / R_EARTH)
    return np.column_stack([lon, lat])


def _circle(t, bump=lambda s: 0.0 * s):
    """Points at lateral offset t (left positive) along the reference arc, pushed outwards by bump(s)."""
    s = np.arange(0.0, LENGTH + 1e-9, 0.25)
    th = s / RADIUS
    r = RADIUS - t + bump(s)
    return np.column_stack([r * np.sin(th), RADIUS - r * np.cos(th)])


class Source:
    def __init__(self, bump=lambda s: 0.0 * s):
        self.geometries = [_lonlat(_circle(0.0)), _lonlat(_circle(-WIDTH, bump))]

    def lane_boundary_geometries(self, source_id):
        return self.geometries if source_id == "L1" else []


def _xodr(tmp_path, provenances):
    root = ET.Element("OpenDRIVE")
    ET.SubElement(ET.SubElement(root, "header"), "geoReference").text = f"+proj=tmerc +lat_0={LAT0} +lon_0={LON0}"
    road = ET.SubElement(root, "road", id="1", junction="-1", length=str(LENGTH))
    geometry = ET.SubElement(ET.SubElement(road, "planView"), "geometry", s="0", x="0", y="0", hdg="0",
                             length=str(LENGTH))
    ET.SubElement(geometry, "arc", curvature=str(1.0 / RADIUS))
    lanes = ET.SubElement(road, "lanes")
    for i, provenance in enumerate(provenances):
        section = ET.SubElement(lanes, "laneSection", s=str(20.0 * i))
        lane = ET.SubElement(ET.SubElement(section, "right"), "lane", id="-1", type="driving")
        ET.SubElement(lane, "width", sOffset="0", a=str(WIDTH), b="0", c="0", d="0")
        ET.SubElement(lane, "userData", code="mapforge.source_lane", value="L1")
        ET.SubElement(lane, "userData", code="mapforge.provenance/v1", value=json.dumps(provenance))
    path = tmp_path / "road.xodr"
    ET.ElementTree(root).write(path, encoding="utf-8")
    return path


@pytest.mark.parametrize("left_out", [{"boundary_evidence_trusted": False}, {"exclusion_code": "source-extension"}])
def test_a_lane_split_by_left_out_records_is_paired_run_by_run(tmp_path, left_out):
    xodr = _xodr(tmp_path, [TRUSTED, left_out, TRUSTED])
    stacked = BI.audit(xodr, None, src=Source())
    lanes = []
    runs = R.audit(xodr, None, src=Source(), lanes=lanes)
    sagitta = (RADIUS + WIDTH) * (1.0 - math.cos(10.0 / RADIUS))     # the chord over s 20-40 against the circle
    assert stacked["boundary_inside_max_m"] == pytest.approx(sagitta, abs=0.01)
    assert runs["boundary_inside_runs_max_m"] < 0.01 and runs["boundary_inside_runs_p95_m"] < 0.01
    assert runs["boundary_inside_runs_split"] == 1
    assert lanes == [{"road": "1", "source_lane_id": "L1", "records": 2, "runs": 2,
                      "max_m": runs["boundary_inside_runs_max_m"]}]


def test_a_run_still_measures_its_own_edge(tmp_path):
    bump = lambda s: np.where((s > 45.0) & (s < 55.0), 0.4, 0.0)       # the source 0.4 m further out at s 45-55
    xodr = _xodr(tmp_path, [TRUSTED, {"boundary_evidence_trusted": False}, TRUSTED])
    runs = R.audit(xodr, None, src=Source(bump))
    assert runs["boundary_inside_runs_max_m"] == pytest.approx(0.4, abs=0.01)


@pytest.mark.parametrize("middle", [TRUSTED, {}, {"exclusion_code": "source-extension", **TRUSTED}])
def test_a_lane_without_gaps_scores_as_boundary_inside(tmp_path, middle):
    bump = lambda s: 0.15 * np.sin(s / 7.0)
    xodr = _xodr(tmp_path, [TRUSTED, middle, {}])
    stacked = BI.audit(xodr, None, src=Source(bump))
    runs = R.audit(xodr, None, src=Source(bump))
    assert runs["boundary_inside_runs_split"] == 0
    assert runs["boundary_inside_runs_p95_m"] == stacked["boundary_inside_p95_m"] > 0.05
    assert runs["boundary_inside_runs_max_m"] == stacked["boundary_inside_max_m"]


def test_records_that_meet_stay_one_run():
    a = np.array([[0.0, 0.0], [5.0, 0.0]])
    near, far = a + [5.9, 0.0], a + [6.1, 0.0]
    assert len(R.runs([a, near])) == 1 and len(R.runs([a, far])) == 2
    assert len(R.runs([a, near, near + [10.0, 0.0]])) == 2
    (one,) = R.runs([a, near])
    assert one.tolist() == np.vstack([a, near]).tolist()


def test_nothing_paired_reports_no_value(tmp_path):
    xodr = _xodr(tmp_path, [{"boundary_evidence_trusted": False}] * 3)
    assert R.audit(xodr, None, src=Source()) == {"boundary_inside_runs_p95_m": None, "boundary_inside_runs_max_m": None,
                                                 "boundary_inside_runs_split": 0}


def test_policy_grades_the_runs_with_the_unchanged_cutoffs():
    policy = yaml.safe_load(sb.POLICY.read_bytes())
    assert tuple(int(x) for x in policy["version"].removesuffix("-draft").split(".")) >= (0, 11)
    t2 = {c["metric"]: c for c in policy["tiers"]["T2"]["checks"]}
    assert t2["boundary_inside_runs_p95_m"]["value"] == 0.10 and t2["boundary_inside_runs_max_m"]["value"] == 0.35
    assert t2["boundary_inside_runs_p95_m"]["op"] == t2["boundary_inside_runs_max_m"]["op"] == "le"
    assert t2["boundary_inside_runs_p95_m"]["applies_to"] == ["shp"]
    assert "boundary_inside_p95_m" not in t2 and "boundary_inside_max_m" not in t2
    for name in ("boundary_inside_p95_m", "boundary_inside_max_m", "boundary_inside_runs_p95_m",
                 "boundary_inside_runs_max_m", "boundary_inside_runs_split"):
        assert name in sb.METRICS
