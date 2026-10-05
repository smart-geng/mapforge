"""Turning connectors curve one way (counter-curvature diagnostic and the turn-sign bounded refit)."""
import math

import numpy as np
import pytest

from mapforge.ops import mouth_frame_align as MFA
from mapforge.validate.turn_shape import counter_curvature


def test_counter_curvature_flags_an_opposite_lobe_inside_a_turn():
    step = 0.25
    profile = [-0.1] * 40 + [0.05] * 8 + [-0.1] * 40  # right turn with a 2 m left lobe
    turn, counter, length = counter_curvature(profile, step)
    assert turn == pytest.approx(math.degrees((-0.1 * 80 + 0.05 * 8) * step))
    assert counter == pytest.approx(0.05) and length == pytest.approx(2.0)
    assert counter_curvature([-0.1] * 88, step)[1] == 0.0


def test_aligned_refit_ends_on_the_linked_lane_centres():
    from mapforge.ops.refline_fit import _chain_from_knots, _planview_at_s, planview_prims
    # old connector: a gentle left turn, lane centre on the reference
    lens, ks = np.array([15.0, 15.0]), np.array([0.0, 0.02, 0.0])
    pv = _chain_from_knots((0.0, 0.0, 0.0), lens, ks)
    old, (xe, ye, he) = planview_prims(pv)
    ss = np.arange(0.0, lens.sum() + 1e-9, 0.25)
    old_path = _planview_at_s(pv, ss)
    # heading along the chain: curvature 0 -> 0.02 -> 0, linear in s on each 15 m spiral
    hh = np.array([0.02 * s * s / 30 if s <= 15 else 0.15 + 0.02 * (s - 15) - 0.02 * (s - 15) ** 2 / 30 for s in ss])
    fa, fc = (0.0, 0.0, 0.0, 0.0), (xe, ye, he, 0.0)

    def edges(x, y, h, shift):  # linked lane: 3.5 m wide, centre moved laterally by ``shift``
        n = np.array([-np.sin(h), np.cos(h)])
        c = np.array([x, y]) + shift * n
        return {"left": {"x": c[0] + 1.75 * n[0], "y": c[1] + 1.75 * n[1], "heading": h},
                "right": {"x": c[0] - 1.75 * n[0], "y": c[1] - 1.75 * n[1], "heading": h}}, c

    start, c0 = edges(0.0, 0.0, 0.0, 0.3)
    end, c1 = edges(xe, ye, he, -0.2)
    out, info = MFA._aligned_refits(old, old_path, ss, hh, {"start": start, "end": end}, fa, fc, float(lens.sum()),
                                    0.3, 0, 6.0, 0.01)
    assert [name for _, name in out] == ["aligned"]
    prims = out[0][0]
    assert np.hypot(prims[0][1] - c0[0], prims[0][2] - c0[1]) < 1e-9 and abs(prims[0][3]) < 1e-9
    x1, y1, h1 = MFA._end_pose(prims)
    assert np.hypot(x1 - c1[0], y1 - c1[1]) < 1e-6 and abs(MFA._wrap(h1 - he)) < 1e-6
    # the localized candidate's blend lengths, sqrt(8 |shift| / kappa), at most half the connector
    assert info["blend_m"] == pytest.approx([min(np.sqrt(8 * 0.3 / 0.01), 15.0), np.sqrt(8 * 0.2 / 0.01)], abs=1e-4)


def test_turn_sign_bounded_refit_removes_the_opposite_lobe():
    from mapforge.ops.refline_fit import _chain_from_knots, _planview_at_s, planview_prims
    # a 117 deg right turn split into two peaks with a +0.06 /m lobe between them (as shp-node17 road 107)
    lens, ks = np.array([6.0, 6.0, 6.0, 6.0]), np.array([0.0, -0.2, 0.06, -0.2, 0.0])
    pv = _chain_from_knots((0.0, 0.0, 0.0), lens, ks)
    old, (xe, ye, he) = planview_prims(pv)
    target = _planview_at_s(pv, np.linspace(0.0, lens.sum(), 200))
    p0, p1 = (0.0, 0.0, 0.0), (xe, ye, he)
    sign = MFA._turn_sign(p0, p1)
    assert sign == -1 and MFA._counter_knots(old, sign) == pytest.approx(0.06)
    free, _, _ = MFA._structure_refit(old, p0, p1, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0), float(lens.sum()), target)
    assert MFA._counter_knots(free, sign) > 0.03  # the free refit keeps following the lobe
    mono, end_error, deviation = MFA._structure_refit(old, p0, p1, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0),
                                                      float(lens.sum()), target, 0.3, sign)
    assert end_error < 1e-6 and MFA._counter_knots(mono, sign) <= MFA.TURN_SIGN_TOL + 1e-9
    # same end poses with the turn redistributed (0.67 m here, an exaggerated lobe); the pick judges it by G8
    assert deviation < MFA.MONOTONE_DEV_MAX
    assert len(mono) == len(old) and min(p[4] for p in mono) >= 6.0 - 1e-9
