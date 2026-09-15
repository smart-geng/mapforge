import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from mapforge.validate.g11 import _advance, audit_file, load_policy


ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / "profiles/validation/g11-opendrive-v1.draft.yaml"


def _write_xodr(tmp_path, specs, *, junction="1", name="", status="INFERRED",
                speed_ms=None, width_records=None, gap_at=None):
    root = ET.Element("OpenDRIVE")
    road = ET.SubElement(root, "road", name=name, id="1", junction=junction,
                         length=str(sum(x["length"] for x in specs)), rule="RHT")
    pv = ET.SubElement(road, "planView")
    x = y = h = s = 0.0
    for i, spec in enumerate(specs):
        gx = x + (0.25 if gap_at == i else 0.0)
        g = ET.SubElement(pv, "geometry", s=str(s), x=str(gx), y=str(y),
                          hdg=str(h), length=str(spec["length"]))
        kind = spec["kind"]
        if kind == "line":
            ET.SubElement(g, "line")
            p = {"kind": kind, "x": x, "y": y, "hdg": h,
                 "length": spec["length"], "k0": 0.0, "k1": 0.0}
        elif kind == "arc":
            ET.SubElement(g, "arc", curvature=str(spec["k0"]))
            p = {"kind": kind, "x": x, "y": y, "hdg": h,
                 "length": spec["length"], "k0": spec["k0"], "k1": spec["k0"]}
        elif kind == "spiral":
            ET.SubElement(g, "spiral", curvStart=str(spec["k0"]),
                          curvEnd=str(spec["k1"]))
            p = {"kind": kind, "x": x, "y": y, "hdg": h,
                 "length": spec["length"], "k0": spec["k0"], "k1": spec["k1"]}
        else:
            ET.SubElement(g, "paramPoly3", aU="0", bU="1", cU="0", dU="0",
                          aV="0", bV="0", cV="0", dV="0", pRange="arcLength")
            p = {"kind": kind, "x": x, "y": y, "hdg": h,
                 "length": spec["length"], "k0": float("nan"), "k1": float("nan")}
        end = _advance(p)
        if end is not None:
            x, y, h, _ = end
        else:
            x += spec["length"]
        s += spec["length"]
    lanes = ET.SubElement(road, "lanes")
    sec = ET.SubElement(lanes, "laneSection", s="0")
    center = ET.SubElement(sec, "center")
    ET.SubElement(center, "lane", id="0", type="none", level="false")
    right = ET.SubElement(sec, "right")
    lane_type = "restricted" if name == "junction_paving" else "driving"
    lane = ET.SubElement(right, "lane", id="-1", type=lane_type, level="false")
    records = width_records or [(0.0, 3.5, 0.0, 0.0, 0.0)]
    for so, a, b, c, d in records:
        ET.SubElement(lane, "width", sOffset=str(so), a=str(a), b=str(b),
                      c=str(c), d=str(d))
    if speed_ms is not None:
        ET.SubElement(lane, "speed", sOffset="0", max=str(speed_ms))
    ET.SubElement(lane, "userData", code="mapforge.provenance/v1", value=json.dumps({
        "status": status,
        "role": "paving" if name == "junction_paving" else "connector",
        "support_kind": "synthetic-connector" if status == "INFERRED" else "measured",
    }))
    path = tmp_path / "case.xodr"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def test_g11_a_rejects_fragmented_inferred_connector(tmp_path):
    path = _write_xodr(tmp_path, [{"kind": "line", "length": 2.0}] * 6)
    result = audit_file(path, POLICY)
    group = result["groups"]["G11-A"]
    codes = {x["code"] for x in group["issues"] if x["severity"] == "FAIL"}
    assert group["status"] == "FAIL"
    assert "inferred_connector_too_many_primitives" in codes
    assert "canonicalization_mergeable_primitives" in codes


def test_g11_straight_road_speed_report_is_strict_json(tmp_path):
    path = _write_xodr(tmp_path, [{"kind": "line", "length": 60.0}], speed_ms=10.)
    result = audit_file(path, POLICY)
    json.dumps(result, allow_nan=False)
    row = result["groups"]["G11-D"]["roads"][0]
    assert row["supported_speed_kmh"] is None
    assert row["supported_speed_state"] == "no-curvature-bound"
    assert row["evaluated_driving_tracks"] == 1


def test_g11_sim_paving_cannot_claim_universal_non_drivability(tmp_path):
    path = _write_xodr(tmp_path, [{"kind":"line", "length":30.}], name="junction_paving")
    result = audit_file(path, POLICY)
    assert any(x["code"] == "sim_paving_requires_consumer_routing_exclusion"
               for x in result["groups"]["G11-E"]["issues"])
    root = ET.parse(path).getroot()
    ET.SubElement(root.find("road"), "link")
    ET.ElementTree(root).write(path)
    result = audit_file(path, POLICY)
    assert result["status"] == "FAIL"
    assert any(x["code"] == "auxiliary_paving_has_driving_topology"
               for x in result["groups"]["G11-E"]["issues"])


@pytest.mark.parametrize('exclusion_code',[
    'source-support-too-short','lane-transition-taper','physical-edge-fill','median-non-driving'])
def test_g11_missing_source_support_cannot_exclude_written_dynamics(tmp_path, exclusion_code):
    path = _write_xodr(tmp_path, [{"kind":"arc", "length":30., "k0":.1}],
                       speed_ms=15.)
    root = ET.parse(path).getroot()
    ud = root.find("road/lanes/laneSection/right/lane/userData")
    prov = json.loads(ud.get("value"))
    prov["exclusion_code"] = exclusion_code
    ud.set("value", json.dumps(prov))
    ET.ElementTree(root).write(path)
    policy = load_policy(POLICY)
    policy["dynamics"]["excluded_provenance_codes"].append(exclusion_code)
    result = audit_file(path, policy)
    assert result["groups"]["G11-D"]["status"] == "FAIL"
    assert result["groups"]["G11-D"]["roads"][0]["evaluated_driving_tracks"] == 1
    row = result["groups"]["G11-D"]["failed_lane_intervals"][0]
    assert row["lane_id"] == "-1"
    assert row["exclusion_code"] == exclusion_code
    assert result["groups"]["G11-D"]["roads"][0]["ignored_driving_provenance_exclusions"] == 1


def test_g11_short_section_uses_actual_sample_spacing(tmp_path):
    path = _write_xodr(tmp_path,
        [{"kind":"spiral", "length":.1, "k0":0., "k1":.01}], speed_ms=1.)
    row = audit_file(path, POLICY)["groups"]["G11-D"]["roads"][0]
    # For a 3.5m lane offset the analytic value is ~0.1/(1+1.75*k)^3.
    # Using the nominal 0.25m grid despite a 0.1m interval underestimates by 60%.
    assert .09 < row["sharpness_max_per_m2"] < .101


def test_g11_a_short_three_piece_g2_is_warning_not_failure(tmp_path):
    specs = [
        {"kind": "spiral", "length": 2.0, "k0": 0.0, "k1": 0.02},
        {"kind": "spiral", "length": 4.0, "k0": 0.02, "k1": 0.02},
        {"kind": "spiral", "length": 4.0, "k0": 0.02, "k1": 0.0},
    ]
    path = _write_xodr(tmp_path, specs)
    group = audit_file(path, POLICY)["groups"]["G11-A"]
    assert group["status"] == "PASS"
    assert group["level"] == "WARNING"
    assert any(x["code"] == "short_primitive" for x in group["issues"])


def test_g11_c_rejects_independent_geometry_start_gap(tmp_path):
    path = _write_xodr(
        tmp_path,
        [{"kind": "line", "length": 10.0}, {"kind": "line", "length": 10.0}],
        gap_at=1,
    )
    group = audit_file(path, POLICY)["groups"]["G11-C"]
    assert group["status"] == "FAIL"
    assert any(x["code"] == "internal_position_discontinuity"
               for x in group["issues"])


def test_g11_b_rejects_width_derivative_jump(tmp_path):
    path = _write_xodr(
        tmp_path, [{"kind": "line", "length": 10.0}], junction="-1",
        status="TRANSFORMED",
        width_records=[(0.0, 3.5, 0.0, 0.0, 0.0),
                       (5.0, 3.5, 1.0, 0.0, 0.0)],
    )
    group = audit_file(path, POLICY)["groups"]["G11-B"]
    assert group["status"] == "FAIL"
    assert any(x["code"] == "cross_section_first_derivative_jump"
               for x in group["issues"])


def test_g11_d_checks_lane_center_dynamics(tmp_path):
    path = _write_xodr(
        tmp_path, [{"kind": "arc", "length": 20.0, "k0": 0.1}],
        junction="-1", status="TRANSFORMED", speed_ms=60.0 / 3.6,
    )
    group = audit_file(path, POLICY)["groups"]["G11-D"]
    assert group["status"] == "FAIL"
    assert any(x["code"] == "lateral_acceleration_exceeded"
               for x in group["issues"])
    assert group["roads"][0]["supported_speed_kmh"] < 60.0


@pytest.mark.parametrize('width_slope',[0.,.04])
def test_world_lane_curvature_join_is_not_implied_by_reference_g2(tmp_path,width_slope):
    path=_write_xodr(tmp_path,[{'kind':'spiral','length':20.,'k0':0.,'k1':.001},
                              {'kind':'arc','length':20.,'k0':.001}],
                     junction='-1',status='TRANSFORMED',speed_ms=3.,
                     width_records=[(0.,3.5,width_slope,0.,0.)])
    report=audit_file(path,POLICY)
    assert report['groups']['G11-C']['status']=='PASS'
    group=report['groups']['G11-D']
    jumps=[x for x in group['issues'] if x['code']=='driving_lane_curvature_jump']
    assert bool(jumps)==bool(width_slope)
    if width_slope:
        assert jumps[0]['joints'][0]['s_m']==20.
        assert group['status']=='FAIL'


def test_g11_e_consumer_profile_can_reject_paving(tmp_path):
    path = _write_xodr(
        tmp_path, [{"kind": "line", "length": 20.0}],
        junction="1", name="junction_paving", status="INFERRED",
    )
    policy = copy.deepcopy(load_policy(POLICY))
    policy["consumer_profile"]["id"] = "odr15-ad-strict"
    policy["consumer_profile"]["allow_auxiliary_paving"] = False
    group = audit_file(path, policy)["groups"]["G11-E"]
    assert group["status"] == "FAIL"
    assert any(x["code"] == "auxiliary_paving_not_allowed"
               for x in group["issues"])
