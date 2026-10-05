"""C2 lane boundaries, exact mouth edge headings and short reference primitives (smoothness step)."""
import math

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import mouth_frame_align as MFA
from mapforge.ops import refline_merge


def _jumps(pieces, order):
    """Largest jump of the given derivative across the internal knots of a piecewise cubic."""
    out = 0.0
    for left, right in zip(pieces, pieces[1:]):
        out = max(out, abs(LR._eval(left, right[0], order) - LR._eval(right, right[0], order)))
    return out


def test_g2_transition_matches_both_ends_and_is_c2_inside():
    pieces = LR._g2_transition(2.0, 14.0, (1.0, 0.05, 0.004), (1.9, -0.02, -0.001))
    assert [LR._eval(pieces[0], 2.0, o) for o in range(3)] == pytest.approx([1.0, 0.05, 0.004], abs=1e-12)
    assert [LR._eval(pieces[-1], 14.0, o) for o in range(3)] == pytest.approx([1.9, -0.02, -0.001], abs=1e-12)
    assert max(_jumps(pieces, o) for o in range(3)) < 1e-12


def test_c2_fit_is_curvature_continuous_including_a_birth_start():
    s = np.arange(0.0, 80.0, 0.5)
    taper = np.where(s < 30, -3.5 - 3.5 * s / 30, np.where(s < 50, -7.0, -7.0 - 0.1 * (s - 50)))
    birth = (-3.5, 0.0, 0.002)  # inner boundary state at the birth: value, slope, curvature
    pieces = LR.fit_boundary(s, taper, 0.0, 79.5, start=birth, end=(None, "mouth"), ramp_start=True, c2_ends=True)
    assert [LR._eval(pieces[0], 0.0, o) for o in range(3)] == pytest.approx(list(birth), abs=1e-9)
    assert _jumps(pieces, 0) < 1e-9 and _jumps(pieces, 1) < 1e-9 and _jumps(pieces, 2) < 1e-9
    assert abs(LR._eval(pieces[-1], 79.5, 1)) <= LR.MOUTH_SLOPE_CAP + 1e-9
    c1 = LR.fit_boundary(s, taper, 0.0, 79.5, start=birth[:2], end=(None, "mouth"), ramp_start=True)
    assert _jumps(c1, 2) > 1e-3  # the C1 end blends this replaces do jump in t''


def test_c2_repair_bump_lifts_the_crossing_and_keeps_the_outer_edge():
    class FakeRoad:
        s = [0.0, 40.0]
    dip = [(0.0, 20.0, 0.3, 0.0, -0.0009, 0.0), (20.0, 40.0, -0.06, 0.0, 0.0009, 0.0)]
    widths = {(0, "right", 1): dip, (0, "right", 2): [(0.0, 40.0, 3.5, 0.0, 0.0, 0.0)]}
    report = {"c2_ends": True}
    assert LR._repair_crossings(FakeRoad(), widths, report)
    w1, w2 = widths[(0, "right", 1)], widths[(0, "right", 2)]
    assert min(LR._at(w1, x) for x in np.linspace(0, 40, 401)) >= -LR.NEGATIVE_WIDTH_TOL
    for x in np.linspace(0, 40, 81):
        assert LR._at(w1, x) + LR._at(w2, x) == pytest.approx(LR._at(dip, x) + 3.5, abs=1e-9)


def test_width_local_slopes_meets_both_mouths_within_the_bulge_budget():
    # end widths 3.5 -> 3.2 with strongly opposite end slopes: the single cubic would bulge far
    local = {("start", "left"): (1.75, 0.08, 0.0), ("start", "right"): (-1.75, -0.08, 0.0),
             ("end", "left"): (1.6, -0.06, 0.0), ("end", "right"): (-1.6, 0.06, 0.0)}
    length = 30.0
    single = MFA._hermite(3.5, 0.16, 3.2, -0.12, length)
    bulge_single = max(MFA._poly(single, length * i / 300) for i in range(301)) - 3.5
    centre, pieces, info = MFA._width_local_slopes(local, length, 0.25)
    assert MFA._pc_eval(pieces, 0.0) == pytest.approx(3.5) and MFA._pc_eval(pieces, 0.0, 1) == pytest.approx(0.16)
    assert MFA._pc_eval(pieces, length) == pytest.approx(3.2) and MFA._pc_eval(pieces, length, 1) == pytest.approx(-0.12)
    widths = [MFA._pc_eval(pieces, length * i / 600) for i in range(601)]
    assert bulge_single > 0.25 and max(widths) - 3.5 <= 0.25 + 1e-9
    assert max(_jumps(pieces, o) for o in range(3)) < 1e-9


def test_end_curvature_match_makes_the_lane_centre_g2_and_keeps_the_mouths():
    from mapforge.validate.smoothness import _edge_world_curvature
    prims = [("spiral", 0.0, 0.0, 0.0, 12.0, 0.01, 0.06), ("arc", 0.0, 0.0, 0.0, 18.0, 0.06, 0.06)]
    length = 30.0
    offset = [(0.0, length, 1.7, 0.03, -0.001, 0.00002)]
    width = [(0.0, length, 3.4, 0.01, 0.0005, -0.00001)]
    centre_t = {"start": {"curvature": 0.012}, "end": {"curvature": 0.055}}
    edge_t = {"start": {"left": {"curvature": 0.011}, "right": {"curvature": 0.0135}},
              "end": {"left": {"curvature": 0.05}, "right": {"curvature": 0.062}}}
    off, cen, wid, info = MFA._match_end_curvature(prims, offset, width, centre_t, edge_t)
    ends = {"start": (0.0, 0.01, 0.05 / 12.0), "end": (length, 0.06, 0.0)}
    for c, (u, kappa, sharp) in ends.items():
        o = [MFA._pc_eval(off, u, n) for n in range(3)]
        w = [MFA._pc_eval(wid, u, n) for n in range(3)]
        centre = [x - y / 2 for x, y in zip(o, w)]
        assert _edge_world_curvature(*centre, kappa, sharp) == pytest.approx(centre_t[c]["curvature"], abs=1e-12)
        # positions and slopes at the mouths are untouched
        for n in range(2):
            assert o[n] == pytest.approx(MFA._pc_eval(offset, u, n), abs=1e-12)
            assert w[n] == pytest.approx(MFA._pc_eval(width, u, n), abs=1e-12)
        # both edges keep the same, smallest residual in t''
        residual = []
        for side, curve in (("left", o), ("right", [x - y for x, y in zip(o, w)])):
            along = 1 - kappa * curve[0]
            k = _edge_world_curvature(*curve, kappa, sharp)
            residual.append((edge_t[c][side]["curvature"] - k) * (along ** 2 + curve[1] ** 2) ** 1.5 / along)
        assert residual[0] == pytest.approx(residual[1], abs=1e-12)
        assert residual == pytest.approx(info[c]["edge_t2_residual"], abs=1e-12)
    # C2 everywhere, and the connector between the two 10 m blends is unchanged
    assert max(_jumps(off, o) for o in range(3)) < 1e-12 and max(_jumps(wid, o) for o in range(3)) < 1e-12
    for s in np.linspace(10.0, 20.0, 11):
        assert MFA._pc_eval(off, s) == pytest.approx(MFA._pc_eval(offset, s), abs=1e-12)
        assert MFA._pc_eval(wid, s) == pytest.approx(MFA._pc_eval(width, s), abs=1e-12)


def test_end_curvature_match_reuses_the_width_transition_knots():
    # a width with its own 6 m end-slope transitions: the curvature corrections add no new knots
    length = 30.0
    width = MFA._sum_pieces([[(0.0, length, 3.4, 0.0, 0.0, 0.0)],
                             MFA._scaled(LR._g2_transition(0.0, 6.0, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), 0.04),
                             MFA._scaled(MFA._mirror(LR._g2_transition(0.0, 6.0, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),
                                                     length), -0.03)], length)
    offset = MFA._sum_pieces([[(0.0, length, 0.0, 0.0, 0.0, 0.0)], MFA._scaled(width, 0.5)], length)
    prims = [("arc", 0.0, 0.0, 0.0, length, 0.02, 0.02)]
    centre_t = {"start": {"curvature": 0.025}, "end": {"curvature": 0.018}}
    edge_t = {"start": {"left": {"curvature": 0.024}, "right": {"curvature": 0.027}},
              "end": {"left": {"curvature": 0.017}, "right": {"curvature": 0.020}}}
    off, _, wid, info = MFA._match_end_curvature(prims, offset, width, centre_t, edge_t, (6.0, 6.0))
    knots = lambda pieces: {round(p[0], 9) for p in pieces}
    assert knots(wid) == knots(width) and knots(off) == knots(offset)
    assert info["start"]["blend_m"] == info["end"]["blend_m"] == 6.0


def test_end_blends_share_existing_transitions_and_meet_in_the_middle():
    assert MFA._end_blends(None, 30.0) == (10.0, 10.0)
    assert MFA._end_blends(None, 20.4) == pytest.approx((10.2, 10.2))  # no 0.4 m piece between the two
    assert MFA._end_blends((6.0, 0.0), 30.0) == (6.0, 10.0)            # the width's own transition first
    assert MFA._end_blends((0.0, 0.0), 30.0, (5.34, 8.73)) == (5.34, 8.73)  # then a centre blend >= 3 m
    assert MFA._end_blends((2.0, 0.0), 30.0, (0.9, 0.0)) == (10.0, 10.0)    # short ones are not reused


def test_only_short_centre_blends_are_lengthened_to_the_shared_length():
    length, kappa = 30.0, 0.01
    width = [(0.0, length, 3.4, 0.0, 0.0, 0.0)]
    # end shifts of 1 mm (blend 0.89 m) and 3.6 cm (blend 5.4 m)
    centre = MFA._hermite(0.001, 0.0, 0.036, 0.0, length)
    _, _, _, info = MFA._localized_from(centre, width, length, kappa, snap=(10.0, 10.0))
    assert info["start_blend_m"] == 10.0  # moves the lane centre by at most the 1 mm shift
    assert info["end_blend_m"] == pytest.approx(math.sqrt(8 * 0.036 / kappa))  # kept: lengthening moves it cm
    _, _, _, near = MFA._localized_from(MFA._hermite(0.0, 0.0, 0.13, 0.0, length), width, length, kappa,
                                        snap=(10.0, 10.0))
    assert near["end_blend_m"] == 10.0  # 10.2 m blend within 0.5 m of the shared length: shortened to it


def _road_with_short_start():
    road = etree.fromstring('<road id="7" length="0" junction="-1"><link/><planView/>'
                            '<lanes><laneSection s="0"><center><lane id="0" type="none"/></center></laneSection></lanes></road>')
    # a 3 m straight start (as the converter leaves after cutting an axis) and a gentle spiral
    prims = [("line", 0.0, 0.0, 0.0, 3.0, 0.0, 0.0), ("spiral", 3.0, 0.0, 0.0, 40.0, 0.0, 0.003)]
    MFA._write_planview(road, prims)
    return road


def test_short_reference_primitive_is_merged_with_both_span_ends_kept():
    from mapforge.validate.smoothness import _geoms, sample_road_ref
    road = _road_with_short_start()
    pts0, ss0, hh0 = sample_road_ref(road, 0.1)
    records = refline_merge.merge_road(road)
    assert records and records[0]["status"] == "MERGED"
    geoms = _geoms(road)
    assert min(g[4] for g in geoms) >= refline_merge.MIN_SEG_M - 1e-9
    pts1, ss1, hh1 = sample_road_ref(road, 0.1)
    assert np.linalg.norm(pts1[0] - pts0[0]) < 1e-9 and np.linalg.norm(pts1[-1] - pts0[-1]) < 0.01
    assert abs(math.remainder(hh1[-1] - hh0[-1], 2 * math.pi)) < 1e-3
    assert records[0]["max_deviation_m"] <= refline_merge.MERGE_TOL["max_dev_tol"]


def test_fit_order_puts_parent_boundaries_first():
    # a: median edge before the birth, then the continuing line; c: median edge after the birth (peels off a)
    a = {"members": {0: 1, 1: 1, 2: 2, 3: 2}}
    c = {"members": {2: 1, 3: 1}, "birth": True, "peel": "outer"}
    b = {"members": {0: 2, 1: 2, 2: 3, 3: 3}}
    order = LR._fit_order([c, b, a])
    assert order.index(a) < order.index(c)


def test_observed_slope_needs_enough_support():
    obs = np.column_stack([np.arange(0.0, 10.0, 0.5), 7.6 - 0.05 * np.arange(0.0, 10.0, 0.5)])
    assert LR._obs_slope(obs, 0.0, 10.0) == pytest.approx(-0.05)
    assert LR._obs_slope(obs, 0.0, 0.6) is None and LR._obs_slope(None, 0.0, 1.0) is None


def test_pipeline_variant_names():
    from mapforge import pipeline
    assert pipeline.variant_name("none") is None and pipeline.variant_name("c2") == "g2-k04-c2"
    assert pipeline.variant_name("g2-k04-curb-guided") == "g2-k04-curb-guided"
    assert pipeline.DEFAULT_VARIANT == "g2-k04-c2"
    with pytest.raises(ValueError):
        pipeline.variant_name("smooth-please")
