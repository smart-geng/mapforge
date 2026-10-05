"""Event-aligned lane-boundary refit: corner algebra, fitter behaviour and a real SHP regression."""
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR

ROOT = Path(__file__).resolve().parents[1]


def _eval_pieces(pieces, s, order=0):
    return LR._at(pieces, s, order)


def test_g2_corner_lands_on_both_lines_with_continuous_curvature():
    m1, m2, h, s_i, t_i = 0.0, 0.17, 6.0, 50.0, -3.0
    left, right = LR._g2_corner(s_i, t_i, m1, m2, h)
    line1 = lambda s: t_i + m1 * (s - s_i)
    line2 = lambda s: t_i + m2 * (s - s_i)
    assert LR._eval(left, s_i - h) == pytest.approx(line1(s_i - h))
    assert LR._eval(left, s_i - h, 1) == pytest.approx(m1) and LR._eval(left, s_i - h, 2) == pytest.approx(0.0)
    assert LR._eval(right, s_i + h) == pytest.approx(line2(s_i + h))
    assert LR._eval(right, s_i + h, 1) == pytest.approx(m2) and LR._eval(right, s_i + h, 2) == pytest.approx(0.0)
    for order in (0, 1, 2):  # C2 at the middle knot
        assert LR._eval(left, s_i, order) == pytest.approx(LR._eval(right, s_i, order))
    assert LR._eval(left, s_i) - t_i == pytest.approx((m2 - m1) * h / 6)
    assert LR._eval(left, s_i, 2) == pytest.approx((m2 - m1) / h)


def test_fit_keeps_straight_lines_and_rounds_a_taper_within_the_corner_bound():
    s = np.arange(0.0, 100.0, 0.5)
    straight = LR.fit_boundary(s, -1.75 + 0.01 * s, 0.0, 99.5)
    assert max(abs(_eval_pieces(straight, x) - (-1.75 + 0.01 * x)) for x in s) < 1e-6
    # taper: flat, 10 deg ramp over 17 m, flat
    taper = np.where(s < 40, -3.5, np.where(s < 57, -3.5 - 0.176 * (s - 40), -3.5 - 0.176 * 17))
    pieces = LR.fit_boundary(s, taper, 0.0, 99.5, mode="g2", kappa_cap=0.04)
    dev = max(abs(_eval_pieces(pieces, x) - y) for x, y in zip(s, taper))
    h_max = 0.176 / 0.04
    assert dev <= 0.176 * h_max / 6 + 0.02
    knots = [p[0] for p in pieces[1:]]
    for k in knots:  # value and slope continuous at every knot
        before = next(p for p in pieces if abs(p[1] - k) < 1e-9)
        after = next(p for p in pieces if abs(p[0] - k) < 1e-9)
        assert LR._eval(before, k) == pytest.approx(LR._eval(after, k), abs=1e-9)
        assert LR._eval(before, k, 1) == pytest.approx(LR._eval(after, k, 1), abs=1e-9)


def test_birth_starts_on_the_inner_boundary_and_mouth_slope_is_capped():
    s = np.arange(0.0, 60.0, 0.5)
    obs = np.where(s < 20, -3.5 - 3.5 * s / 20, -7.0)  # source lane opening to full width
    pieces = LR.fit_boundary(s, obs, 0.0, 59.5, start=(-3.5, 0.0), ramp_start=True, end=(None, "mouth"))
    assert _eval_pieces(pieces, 0.0) == pytest.approx(-3.5) and _eval_pieces(pieces, 0.0, 1) == pytest.approx(0.0)
    assert abs(_eval_pieces(pieces, 59.5, 1)) <= LR.MOUTH_SLOPE_CAP + 1e-9


def test_split_and_width_difference_preserve_values():
    a = [(0.0, 10.0, 1.0, 0.1, 0.01, -0.001)]
    b = [(0.0, 10.0, -2.0, 0.0, 0.0, 0.0)]
    split = LR._split(a, [3.0, 7.5])
    assert [p[0] for p in split] == [0.0, 3.0, 7.5]
    for x in np.linspace(0, 10, 41):
        assert _eval_pieces(split, x) == pytest.approx(LR._eval(a[0], x))
    width = LR._difference(split, b, 0.0, 10.0, 1.0)
    for x in np.linspace(0, 10, 41):
        assert _eval_pieces(width, x) == pytest.approx(LR._eval(a[0], x) + 2.0)


def test_small_crossing_is_lifted_from_the_next_lane_and_outer_edge_stays():
    class FakeRoad:
        s = [0.0, 40.0]
    # lane 1 dips to -0.06 m around s=20 (two independent fits crossing); lane 2 is 3.5 m wide
    dip = [(0.0, 20.0, 0.3, 0.0, -0.0009, 0.0), (20.0, 40.0, -0.06, 0.0, 0.0009, 0.0)]
    widths = {(0, "right", 1): dip, (0, "right", 2): [(0.0, 40.0, 3.5, 0.0, 0.0, 0.0)]}
    outer_before = {x: LR._at(dip, x) + 3.5 for x in np.linspace(0, 40, 81)}
    report = {}
    assert LR._repair_crossings(FakeRoad(), widths, report)
    assert report["crossing_repairs"] and report["crossing_repairs"][0]["lane"] == 1
    w1, w2 = widths[(0, "right", 1)], widths[(0, "right", 2)]
    assert min(LR._at(w1, x) for x in np.linspace(0, 40, 401)) >= -LR.NEGATIVE_WIDTH_TOL
    for x, total in outer_before.items():  # the outer edge of lane 2 does not move
        assert LR._at(w1, x) + LR._at(w2, x) == pytest.approx(total, abs=1e-9)


@pytest.mark.slow
def test_real_node4_refit_is_valid_and_closer_to_the_source(tmp_path):
    shp = ROOT / "shp_0222-0326"
    like = ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"
    if not shp.exists() or not like.exists():
        pytest.skip("local SHP/MAP samples not installed")
    base = tmp_path / "shp-node4.xodr"
    subprocess.run([sys.executable, "-m", "mapforge.cli", "convert", str(shp), "--like", str(like), "--to", "xodr",
                    "--shp-mouth", "legacy", "--post", "none", "-o", str(base)], cwd=str(ROOT), env=dict(os.environ, PYTHONIOENCODING="utf-8"), capture_output=True)
    out = tmp_path / "refit.xodr"
    report = LR.apply(base, out, LR.VARIANTS["g2-k04"])
    assert report["rewritten"] == 5 and not report["skipped"]
    assert all(r["min_width_m"] >= -LR.NEGATIVE_WIDTH_TOL for r in report["rows"])
    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    assert schema.validate(etree.parse(str(out)))
    from mapforge.validate.shp_boundary_fidelity import evaluate_shp_outer_edges
    before = evaluate_shp_outer_edges(shp, base, src=LR._shp_source())
    after = evaluate_shp_outer_edges(shp, out, src=LR._shp_source())
    assert after["source_to_target"]["p95_m"] < 0.7 * before["source_to_target"]["p95_m"]
