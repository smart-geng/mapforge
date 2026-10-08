import math

import numpy as np
import pytest

from mapforge.ops import leg_fit_fallback as LF
from mapforge.ops import refline_fit as RF


def _lanes_along(xs, ys_offsets, step=1.0):
    """Straight lane centre lines/boundaries along x at the given lateral offsets."""
    return [np.column_stack([xs, np.full(len(xs), y)]) for y in ys_offsets]


def _jumping_centre():
    # 150 m straight, then the carriageway centre jumps 3 m to the left within 8 m before the junction end:
    # lanes added on one side move the ROADLINK centre while the lanes run on straight
    x = np.r_[np.arange(0.0, 150.0, 5.0), 150.0, 154.0, 158.0]
    y = np.r_[np.zeros(30), 0.0, 1.5, 3.0]
    return np.column_stack([x, y])


def test_sharpness_of_a_curvature_cap_keeps_the_converter_caps():
    assert LF.sharp_for(LF.BASE_KAPPA) == pytest.approx(LF.BASE_SHARP, rel=0.01)


def test_jumping_centre_with_straight_lanes_gets_a_straight_reference():
    gxy = _jumping_centre()
    with pytest.raises(RF.ReflineFitError):
        RF.fit_leg_refline(gxy)
    xs = np.linspace(-5.0, 165.0, 120)
    lanes = _lanes_along(xs, [-7.0, -5.25, -3.5, -1.75, 0.0, 1.75, 3.5])
    pv, record = LF.fit_leg(gxy, lanes, "end")
    assert pv is not None
    chosen = record["chosen"]
    assert chosen["kappa_cap"] == LF.BASE_KAPPA and not chosen["free_end_curvature"]
    assert chosen["heading_max_deg"] <= 2.0
    assert chosen["kappa_max"] <= LF.BASE_KAPPA
    assert chosen["max_m"] <= LF.JN_TOL_M


def _arc_leg(radius, straight, turn_deg):
    """Straight approach then an arc of ``turn_deg`` ending at the junction end."""
    pts = [(x, 0.0) for x in np.arange(0.0, straight, 2.0)]
    for a in np.linspace(0.0, math.radians(turn_deg), 30):
        pts.append((straight + radius * math.sin(a), radius * (1 - math.cos(a))))
    return np.asarray(pts)


def _offset(line, d):
    seg = np.gradient(line, axis=0)
    n = np.column_stack([-seg[:, 1], seg[:, 0]]) / np.linalg.norm(seg, axis=1)[:, None]
    return line + d * n


def test_a_hard_bend_is_not_accepted_at_the_base_caps():
    gxy = _arc_leg(20.0, 60.0, 45.0)
    lanes = [_offset(gxy, d) for d in (-3.5, -1.75, 0.0, 1.75, 3.5)]
    pv, record = LF.fit_leg(gxy, lanes, "end")
    assert pv is None
    assert all(not row["accepted"] for row in record["tried"])


def test_relaxed_caps_follow_a_real_curve_at_a_lower_design_speed():
    gxy = _arc_leg(30.0, 20.0, 40.0)
    lanes = [_offset(gxy, d) for d in (-3.5, -1.75, 0.0, 1.75, 3.5)]
    assert LF.fit_leg(gxy, lanes, "end")[0] is None
    pv, record = LF.fit_leg(gxy, lanes, "end", relaxed=True)
    assert pv is not None
    chosen = record["chosen"]
    assert chosen["kappa_cap"] > LF.BASE_KAPPA and chosen["free_end_curvature"]
    assert chosen["sharp_cap"] == pytest.approx(LF.sharp_for(chosen["kappa_cap"]))
    assert chosen["heading_max_deg"] <= LF.HEAD_MAX_DEG


def test_hooks_pass_converter_fits_through_and_are_removed():
    from mapforge.ops import shp_to_xodr as S
    fit, chain = S.fit_leg_refline, S._chained_links
    gxy = np.column_stack([np.linspace(0.0, 100.0, 21), np.zeros(21)])
    state = LF.State(src=None)
    with LF.installed(state):
        assert S.fit_leg_refline is not fit and S._chained_links is not chain
        pv, dev, approximated = S.fit_leg_refline(gxy)
    assert S.fit_leg_refline is fit and S._chained_links is chain
    ref = fit(gxy)
    assert [(s.kind, s.length, s.curvature) for s in pv.segs] == [(s.kind, s.length, s.curvature) for s in ref[0].segs]
    assert (dev, approximated) == ref[1:]
    assert state.fitted == [] and state.report()["legs"] == []
