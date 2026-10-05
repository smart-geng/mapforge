"""Curb-return shoulders at strong mouth flares and the localized mouth blend of connectors."""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import mouth_frame_align as MFA

ROOT = Path(__file__).resolve().parents[1]


def _flared(s, start=40.0, end=48.0, amplitude=1.5):
    """Straight edge, then an outward curb-return flare whose slope grows towards the mouth."""
    u = np.clip((s - start) / (end - start), 0.0, 1.0)
    return 16.4 + 0.004 * s + amplitude * u ** 2


def test_flare_start_is_found_where_the_edge_turns_outward():
    s = np.arange(0.0, 48.0, 0.5)
    s_f = LR._flare_start(s, _flared(s), 0.0, 48.0, outward=1.0)
    assert s_f is not None and 38.0 <= s_f <= 42.5
    assert LR._flare_start(s, 16.4 + 0.004 * s, 0.0, 48.0, outward=1.0) is None  # no flare, no shoulder


def test_curb_tail_is_tangent_follows_the_source_and_levels_off_with_bounded_curvature():
    s = np.arange(40.0, 48.0, 0.25)
    t = _flared(s)
    v0, m0 = 16.4 + 0.004 * 40.0, 0.004
    pieces = LR._curb_tail(s, t, 40.0, 48.0, 54.0, v0, m0)
    assert LR._at(pieces, 40.0) == pytest.approx(v0) and LR._at(pieces, 40.0, 1) == pytest.approx(m0)
    assert max(abs(LR._at(pieces, x) - y) for x, y in zip(s, t)) < 0.05
    for k in pieces[1:]:  # C1 at every knot
        prev = next(p for p in pieces if abs(p[1] - k[0]) < 1e-9)
        assert LR._eval(prev, k[0]) == pytest.approx(LR._eval(k, k[0]), abs=1e-9)
        assert LR._eval(prev, k[0], 1) == pytest.approx(LR._eval(k, k[0], 1), abs=1e-9)
    after = np.linspace(48.0, 54.0, 61)
    assert max(abs(LR._at(pieces, x, 2)) for x in after) <= LR.CURB_HOLD_KAPPA + 1e-9


def test_blend_down_is_c2_and_its_peak_curvature_is_8_over_d_squared():
    for d in (4.0, 19.0):
        b = MFA._blend_down(d)
        assert [MFA._pc_eval(b, 0.0, o) for o in (0, 1, 2)] == pytest.approx([1.0, 0.0, 0.0], abs=1e-12)
        assert [MFA._pc_eval(b, d, o) for o in (0, 1, 2)] == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)
        for left, right in zip(b, b[1:]):
            for o in (0, 1, 2):
                assert LR._eval(left, right[0], o) == pytest.approx(LR._eval(right, right[0], o), abs=1e-12)
        peak = max(abs(MFA._pc_eval(b, x, 2)) for x in np.linspace(0, d, 801))
        assert peak == pytest.approx(8.0 / d ** 2, rel=1e-6)


def test_localized_offsets_keep_both_mouths_and_leave_the_middle_alone():
    left, right = (2.0, 0.01, 0.001, -2e-5), (-1.5, -0.02, 5e-4, 1e-5)
    length = 60.0
    offset, centre, width, info = MFA._localized_offsets(left, right, length, 0.01)
    for s in (0.0, length):  # edges keep value and slope at both mouths
        for edge_old, edge_new in ((left, MFA._pc_eval(offset, s)),
                                   (right, MFA._pc_eval(offset, s) - MFA._poly(width, s))):
            assert edge_new == pytest.approx(MFA._poly(edge_old, s), abs=1e-9)
        slope_old = left[1] + 2 * left[2] * s + 3 * left[3] * s * s
        assert MFA._pc_eval(offset, s, 1) == pytest.approx(slope_old, abs=1e-9)
    # outside both blend lengths the centre is the slope-only cubic: the end shifts do not reach it
    single = tuple((x + y) / 2 for x, y in zip(left, right))
    slope1 = single[1] + 2 * single[2] * length + 3 * single[3] * length ** 2
    base = MFA._hermite(0.0, single[1], 0.0, slope1, length)
    mid = info["start_blend_m"] + 1.0
    assert mid < length - info["end_blend_m"]
    assert MFA._pc_eval(centre, mid) == pytest.approx(MFA._poly(base, mid), abs=1e-9)
    assert abs(MFA._poly(single, mid) - MFA._poly(base, mid)) > 0.1  # the single cubic would have dragged it
    assert info["start_blend_m"] == pytest.approx(math.sqrt(8 * abs(info["start_shift_m"]) / 0.01))


def _quarter_turn(radius=20.0, gap=0.0):
    """Lane centres of a 90 deg left turn (east -> north) and a source arc between them."""
    a = np.linspace(-math.pi / 2, 0.0, 200)
    source = np.column_stack([radius * np.cos(a), radius + radius * np.sin(a)])  # (0,0) east ... (R,R) north
    source = source + np.array([0.0, gap])

    def edges(x, y, h):
        n = np.array([-math.sin(h), math.cos(h)])
        return {"left": {"x": x + 1.75 * n[0], "y": y + 1.75 * n[1], "heading": h},
                "right": {"x": x - 1.75 * n[0], "y": y - 1.75 * n[1], "heading": h}}
    targets = {"start": edges(0.0, 0.0, 0.0), "end": edges(radius, radius, math.pi / 2)}
    fa, fc = (0.0, 0.0, 0.0, 0.0), (radius, radius, math.pi / 2, 0.0)
    return source, targets, fa, fc


def test_source_guided_refit_follows_the_source_between_the_lane_centres():
    source, targets, fa, fc = _quarter_turn()
    [(prims, local, info)] = MFA._source_guided(source, targets, fa, fc, 6.0)  # no reversal: one fit
    assert prims[0][1:4] == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    xe, ye, he = MFA._end_pose(prims)
    assert (xe, ye) == pytest.approx((20.0, 20.0), abs=0.02) and he == pytest.approx(math.pi / 2, abs=1e-3)
    assert prims[0][5] == pytest.approx(0.0, abs=1e-9) and prims[-1][6] == pytest.approx(0.0, abs=1e-9)
    assert info["min_primitive_m"] >= 6.0 - 1e-9 and len(prims) <= 5
    for (contact, side), (t, slope, along) in local.items():  # edges sit at +/-1.75 m on the lane centre
        assert abs(t) == pytest.approx(1.75, abs=0.02) and abs(along) < 0.02


def test_source_guided_refit_needs_a_source_that_meets_both_mouths():
    source, targets, fa, fc = _quarter_turn(gap=1.5)
    assert MFA._source_guided(source, targets, fa, fc, 6.0) is None


@pytest.mark.slow
def test_real_node18_flare_becomes_a_curb_return_shoulder(tmp_path):
    shp = ROOT / "shp_0222-0326"
    like = ROOT / "v2x_map_xml" / "map凤苑路-金剑路node18.xml"
    if not shp.exists() or not like.exists():
        pytest.skip("local SHP/MAP samples not installed")
    base = tmp_path / "shp-node18.xodr"
    subprocess.run([sys.executable, "-m", "mapforge.cli", "convert", str(shp), "--like", str(like), "--to", "xodr",
                    "--shp-mouth", "legacy", "--post", "none", "-o", str(base)], cwd=str(ROOT), env=dict(os.environ, PYTHONIOENCODING="utf-8"), capture_output=True)
    out = tmp_path / "curb.xodr"
    report = LR.apply(base, out, LR.VARIANTS["g2-k04-curb"])
    shoulders = [(r["road"], s) for r in report["rows"] for s in r["curb_return_shoulders"]]
    assert [(road, s["side"], s["lane"]) for road, s in shoulders] == [("10", "left", 5)]
    assert not any(r["strong_mouth_flares"] for r in report["rows"])
    root = etree.parse(str(out))
    assert etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd"))).validate(root)
    road = next(r for r in root.getroot().findall("road") if r.get("id") == "10")
    lane = road.findall("lanes/laneSection")[-1].find("left/lane[@id='5']")
    assert lane.get("type") == "shoulder" and lane.find("link/successor") is None
    prov = json.loads(lane.find("userData[@code='mapforge.provenance/v1']").get("value"))
    assert prov["eligibility"] == "excluded" and prov["curb_return"] is True
    from mapforge.validate.smoothness import lane_edges_at
    length = float(road.get("length"))
    slope = (lane_edges_at(road, length - 1e-6, "left")[4] - lane_edges_at(road, length - 0.5, "left")[4]) / 0.5
    assert abs(slope) <= LR.MOUTH_SLOPE_CAP + 1e-3  # the driving edge meets connectors smoothly
