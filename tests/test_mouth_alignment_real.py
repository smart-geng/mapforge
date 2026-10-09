"""Junction-mouth edge consistency on a freshly converted real MAP intersection (node13)."""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from lxml import etree

from mapforge.ops import mouth_edge_match, mouth_frame_align
from mapforge.ops.mouth_edge_match import WIDTH_OVERSHOOT_TOL_M
from mapforge.validate.junction_edges import audit
from mapforge.validate.scoreboard import connector_widths
from mapforge.validate.smoothness import route_continuity

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "v2x_map_xml" / "map凤阁路-金玥路node13.xml"


@pytest.fixture(scope="module")
def baseline(tmp_path_factory):
    if not SOURCE.exists():
        pytest.skip("local MAP sample not installed")
    out = tmp_path_factory.mktemp("mouth") / "map-node13.xodr"
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    subprocess.run([sys.executable, "-m", "mapforge.cli", "convert", str(SOURCE), "--to", "xodr", "--post", "none", "-o", str(out)],
                   cwd=str(ROOT), env=env, capture_output=True)
    assert out.exists()
    return out


def _schema():
    return etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))


@pytest.mark.slow
def test_hermite_edges_remove_kinks_but_leave_cross_section_stagger(baseline, tmp_path):
    before = audit(ET.parse(baseline).getroot())
    out = tmp_path / "edge.xodr"
    report = mouth_edge_match.apply(baseline, out)
    after = audit(ET.parse(out).getroot())
    assert report["rewritten"] == before["count"] // 4 and not report["skipped"]
    # Kinks remain only where the width slope had to be limited, and as reported.
    kink = max([r["edge_heading_kink_deg"] for r in report["rows"]] + [0.0])
    assert after["maxima"]["heading_deg"] <= kink + 1e-6 < before["maxima"]["heading_deg"]
    assert all(r["width_slope_limited"] or r["edge_heading_kink_deg"] == 0 for r in report["rows"])
    assert connector_widths(ET.parse(out).getroot())["connector_width_bulge_max_m"] <= WIDTH_OVERSHOOT_TOL_M + 1e-6
    # Remaining position mismatch is purely along-track: the documented cross-section tilt.
    stagger = max(r["along_track_residual_m"] for r in report["rows"])
    assert abs(after["maxima"]["position_m"] - stagger) < 1e-6
    assert _schema().validate(etree.parse(str(out)))


@pytest.mark.slow
def test_frame_alignment_makes_mouth_edges_and_centres_continuous(baseline, tmp_path):
    out = tmp_path / "align.xodr"
    report = mouth_frame_align.apply(baseline, out)
    root = ET.parse(out).getroot()
    edges = audit(root)
    centres = route_continuity(root)
    assert report["rewritten"] >= 14 and not report.get("fallback_edge_match", {}).get("skipped")
    kink = max([r["edge_heading_kink_deg"] for r in report["rows"]] + [0.0])
    assert edges["maxima"]["position_m"] < 1e-6 and edges["maxima"]["heading_deg"] <= kink + 1e-6
    assert connector_widths(root)["connector_width_bulge_max_m"] <= WIDTH_OVERSHOOT_TOL_M + 1e-6
    assert max(max(x["gap_in"], x["gap_out"]) for x in centres) < 1e-6
    assert max(max(x["dh_in_deg"], x["dh_out_deg"]) for x in centres) < 1e-3
    tol = mouth_frame_align.FIT_TOL["max_dev_tol"]
    for row in report["rows"]:
        assert row["lane_centre_shift_max_m"] <= tol + 0.05  # lane centre stays near the old path
        # no segment shorter than the connector already had (6 m cap), at most 5 primitives
        assert row["min_primitive_m"] >= min(row["min_primitive_old_m"], 6.0) - 1e-9 and row["primitives"] <= 5
    assert _schema().validate(etree.parse(str(out)))


@pytest.mark.slow
def test_end_curvature_match_makes_lane_centres_g2_across_the_mouths(baseline, tmp_path):
    from mapforge.validate.smoothness import junction_lane_interfaces
    out = tmp_path / "g2.xodr"
    report = mouth_frame_align.apply(baseline, out, match_end_curvature=True)
    root = ET.parse(out).getroot()
    rows = junction_lane_interfaces(root)
    assert report["rewritten"] >= 14 and rows and not any("error" in r for r in rows)
    # lane centres: G2 across every junction laneLink (G11-C strict value 1e-7 /m)
    assert max(r["curvature_per_m"] for r in rows) < 1e-9
    assert max(r["position_m"] for r in rows) < 1e-6 and max(r["heading_deg"] for r in rows) < 1e-3
    # lane edges: positions exact, curvature residual bounded (T2 draft 1e-3 /m)
    edges = audit(root)
    assert edges["maxima"]["position_m"] < 1e-6 and edges["maxima"]["curvature_per_m"] < 1e-3
    assert _schema().validate(etree.parse(str(out)))


def test_support_domain_follows_the_refitted_reference():
    from mapforge.ops.mouth_frame_align import _remap_support_s
    import json
    import numpy as np
    lane = etree.Element("lane")
    etree.SubElement(lane, "userData", code="mapforge.provenance/v1", value=json.dumps({"support_s": [5.0, 15.0]}))
    old_ss = np.linspace(0, 20, 81)
    old_path = np.column_stack([old_ss, np.zeros_like(old_ss)])
    new_ss = np.linspace(0, 22, 89)  # same straight path, reference started 2 m earlier
    new_path = np.column_stack([new_ss - 2.0, np.zeros_like(new_ss)])
    moved = _remap_support_s(lane, old_path, old_ss, new_path, new_ss)
    assert np.allclose(moved["new"], [7.0, 17.0])
    assert json.loads(lane.find("userData").get("value"))["support_s"] == moved["new"]


def test_g8_ceiling_guard_reads_the_active_policy():
    med, p95 = mouth_frame_align._via_limits()
    assert (med, p95) == pytest.approx((0.6 - mouth_frame_align.GUARD_MARGIN_M, 1.5 - mouth_frame_align.GUARD_MARGIN_M))
