"""Edge-residual trim: a connector chain solved again with a given end curvature rate (mouth_frame_align)."""
import numpy as np
import pytest

from mapforge.ops import mouth_frame_align as M
from mapforge.ops.refline_fit import _chain_from_knots, planview_prims


def _chain(lens, knots):
    prims, _ = planview_prims(_chain_from_knots((0.0, 0.0, 0.0), np.asarray(lens, float), np.asarray(knots, float)))
    return prims


def _rates(prims):
    return (prims[0][6] - prims[0][5]) / prims[0][4], (prims[-1][6] - prims[-1][5]) / prims[-1][4]


def test_the_end_rate_is_met_with_both_ends_and_the_structure_kept():
    old = _chain([10.0, 12.0, 10.0], [0.0, 0.03, 0.06, 0.0])          # a left turn unwinding at its end
    assert _rates(old)[1] == pytest.approx(-0.006)
    new, dev = M._end_rate_trim(old, {"end": -0.0057}, 0.25, turn_sign=1)     # 5 % towards a straight road
    assert len(new) == len(old) and _rates(new)[1] == pytest.approx(-0.0057, abs=1e-9)
    assert M._end_pose(new) == pytest.approx(M._end_pose(old), abs=1e-6)
    assert (new[0][1], new[0][2], new[0][3], new[0][5]) == pytest.approx((0.0, 0.0, 0.0, 0.0))
    assert new[-1][6] == pytest.approx(0.0)                             # end curvature kept
    assert min(p[4] for p in new) >= min(min(p[4] for p in old), 6.0) - 1e-9   # no segment under the floor
    assert min(p[5] for p in new[1:]) >= -M.TURN_SIGN_TOL               # still on the turn's side
    assert 0.0 < dev < M.TRIM_DEV_MAX_M
    # a fifth of the rate needs a different chain: the deviation says so (the pick's guard rejects it)
    assert M._end_rate_trim(old, {"end": -0.0048}, 0.25, turn_sign=1)[1] > M.TRIM_DEV_MAX_M


def test_both_ends_at_once_need_three_segments():
    two = _chain([12.0, 12.0], [0.0, 0.05, 0.0])
    assert M._end_rate_trim(two, {"start": 0.004, "end": -0.004}, 0.25) is None
    three = _chain([8.0, 10.0, 8.0], [0.0, 0.04, 0.04, 0.0])
    new, _ = M._end_rate_trim(three, {"start": 0.0045, "end": -0.0045}, 0.25)
    assert _rates(new) == pytest.approx((0.0045, -0.0045), abs=1e-9)
    assert M._end_pose(new) == pytest.approx(M._end_pose(three), abs=1e-6)
