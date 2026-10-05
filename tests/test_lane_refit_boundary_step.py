"""Lane refit, boundary step (2026-10-04): short runs, least-squares vertices, advanced corners, protected runs."""
import numpy as np
import pytest

from mapforge.ops import lane_refit as LR

P = {"mode": "g2", "kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05, "c2_ends": True}


def _max_dev(pieces, s, t):
    return max(abs(LR._at(pieces, x) - y) for x, y in zip(s, t))


def _c2_gap(pieces):
    return max(abs(LR._eval(p, p[1], o) - LR._eval(q, q[0], o)) for p, q in zip(pieces[:-1], pieces[1:])
               for o in (0, 1, 2))


def test_short_run_with_small_corners_is_kept():
    # shp-node4 road 12 lane 3: a 0.15 m source bump that is back on the line 5.5 m after its peak
    s = np.arange(0.0, 176.0, 0.5)
    bump = np.interp(s, [0, 27.5, 33.5, 39, 176], [14.55, 14.55, 14.70, 14.55, 14.55])
    pruned = LR.fit_boundary(s, bump, 0.0, 175.5, **P)
    info = {}
    kept = LR.fit_boundary(s, bump, 0.0, 175.5, short_kappa=True, info=info, **P)
    assert _max_dev(pruned, s, bump) > 0.12
    assert _max_dev(kept, s, bump) < 0.03
    assert 39.0 in info["vs"]
    assert _c2_gap(kept) < 1e-9
    # big corners in a short run are still pruned: the curvature target holds
    zig = np.interp(s, [0, 30, 33, 176], [0.0, 0.0, 1.0, 1.0])
    info = {}
    LR.fit_boundary(s, zig, 0.0, 175.5, short_kappa=True, info=info, **P)
    slopes = np.diff(info["vt"]) / np.diff(info["vs"])
    for i in range(1, len(info["vs"]) - 1):
        assert abs(slopes[i] - slopes[i - 1]) / info["h"][i] <= P["kappa_cap"] + 1e-9


def test_short_run_rule_needs_both_corners_within_the_target():
    vs = np.array([0.0, 20.0, 25.0, 50.0, 60.0])
    flat = np.array([0.0, 0.0, 0.05, 0.05, 0.05])
    assert LR._short_run_fits(vs, flat, 1, 0.04)       # 5 m run, kinks 0.01
    steep = np.array([0.0, 0.0, 1.0, 1.0, 1.0])
    assert not LR._short_run_fits(vs, steep, 1, 0.04)   # kinks 0.2 need 5 m of half length
    assert not LR._short_run_fits(vs, flat, 0, 0.04)    # end runs are left to the end rules


def test_least_squares_vertices_fit_a_rounded_knee_and_keep_fixed_values():
    s = np.arange(0.0, 120.0, 0.5)
    m = np.where(s < 11, 0.33, np.where(s < 16.5, 0.33 - 0.06 * (s - 11), 0.0))
    knee = np.concatenate([[0.0], np.cumsum(0.5 * (m[1:] + m[:-1]) / 2)])
    plain = LR.fit_boundary(s, knee, 0.0, 119.5, start=(0.0, None), **P)
    refined = LR.fit_boundary(s, knee, 0.0, 119.5, start=(0.0, None), lsq_refine=True, **P)
    assert _max_dev(refined, s, knee) < 0.9 * _max_dev(plain, s, knee)
    rms = [np.sqrt(np.mean([(LR._at(pc, x) - y) ** 2 for x, y in zip(s, knee)])) for pc in (plain, refined)]
    assert rms[1] < rms[0]
    assert LR._at(refined, 0.0) == pytest.approx(0.0, abs=1e-12)          # a given end value stays
    assert _c2_gap(refined) < 1e-9
    # protected values stay exact as well
    rel = np.where(s < 40, 0.0, 0.2 * (s - 40))
    pieces = LR.fit_boundary(s, rel, 0.0, 119.5, start=(0.0, 0.0, 0.0), protect=[40.0], protect_values={40.0: 0.0},
                             lsq_refine=True, **P)
    # zero up to the corner (half length |dm| / kappa = 5 m before the protected station)
    assert max(abs(LR._at(pieces, x)) for x in np.arange(0.0, 35.0, 0.5)) < 1e-12
    assert LR._at(pieces, 40.0) == pytest.approx(0.2 * 5.0 / 6.0, abs=1e-3)


def _diverging(a=20.0, b=80.0, x=50.0):
    """Inner boundary turning away at x (slope +0.046 -> -0.13 towards the centre), outer one opening on."""
    s = np.arange(a, b + 1e-9, 0.5)
    p_src = np.where(s < x, -3.0 - 0.046 * (s - x), -3.0 + 0.13 * (s - x))
    base = LR.fit_boundary(s, p_src, a, b, **P)
    d_src = np.where(s < x, p_src, -3.0 - 0.146 * (s - x))
    sel = s > x
    obs = np.column_stack([s[sel], d_src[sel]])
    fill = np.arange(a, x, LR.GRID)
    rel = obs[:, 1] - np.array([LR._at(base, q) for q in obs[:, 0]])
    start = (LR._at(base, a), LR._at(base, a, 1), LR._at(base, a, 2))
    return s, d_src, base, obs, np.concatenate([fill, obs[:, 0]]), np.concatenate([np.zeros(len(fill)), rel]), start


def test_advanced_corner_is_sized_by_the_boundary_curvature():
    # shp-node18 road 10 lane -3: width kink 0.28, of which 0.18 is the inner boundary's own corner
    a, b, x = 20.0, 80.0, 50.0
    s, d_src, base, obs, obs_s, rel, start = _diverging(a, b, x)
    whole, _ = LR._advanced_fit(obs_s, rel, obs, a, b, base, start, None, False, False, [("birth", x)], {x: 6.9}, P)
    old = LR._combine(LR.fit_boundary(obs_s, rel, a, b, start=LR._relative_cond(start, base, a), protect=[x],
                                      corner_h={x: 6.9}, protect_values={x: 0.0}, **P), base, a, b, 1.0)
    near = np.arange(x - 8.0, x + 8.0, 0.1)
    target = np.interp(near, s, d_src)
    assert _max_dev(whole, near, target) < 0.05 < 0.15 < _max_dev(old, near, target)
    assert max(abs(LR._at(whole, q, 2)) for q in near) <= 1.05 * P["kappa_cap"]
    ss = np.arange(a, b, 0.1)
    assert max(LR._at(whole, q) - LR._at(base, q) for q in ss) <= 1e-9       # never on the inner side
    assert max(abs(LR._at(whole, q) - LR._at(base, q)) for q in np.arange(a, x - 6.9, 0.1)) < 1e-12


def test_protected_run_keeps_a_real_corner_next_to_an_advanced_death():
    # shp-node16 road 10 lane 4: the taper steepens 4.8 m before its death
    vs = [60.0, 75.5, 80.32, 86.09]
    vt = [3.8, 1.06, 0.0, 0.0]
    assert LR._keep_protected_run(np.array(vs), np.array(vt), 1, {80.32})
    kept_vs, _ = LR._prune(vs, vt, 6.0, protected={80.32}, protected_runs=True)
    assert 75.5 in kept_vs
    dropped_vs, _ = LR._prune(vs, vt, 6.0, protected={80.32})
    assert 75.5 not in dropped_vs


def test_least_squares_keeps_junction_mouth_values():
    # connectors meet the lane centres at the mouth: a curved source end must not move the mouth value
    s = np.arange(0.0, 60.0, 0.5)
    flare = np.where(s < 5.0, -4.4 - 0.19 * (5.0 - s), -4.4)
    grid, vals = LR._grid_profile(s, flare, 0.0, 59.5)
    pieces = LR.fit_boundary(s, flare, 0.0, 59.5, start=(None, "mouth"), lsq_refine=True, **P)
    assert LR._at(pieces, 0.0) == pytest.approx(vals[0], abs=1e-12)
    assert abs(LR._at(pieces, 0.0, 1)) <= LR.MOUTH_SLOPE_CAP + 1e-9


def test_tail_width_is_held_where_the_lane_ends_in_travel_direction():
    # a lane whose source starts at x from (almost) zero width, at a free road end where it ends in its travel
    # direction: the width is held at TAIL_WIDTH_M up to the road end instead of exactly zero
    a, b, x = 0.0, 60.0, 4.0
    s = np.arange(a, b + 1e-9, 0.5)
    base = LR.fit_boundary(s, np.full(len(s), 7.0), a, b, **P)
    obs = np.column_stack([s[s > x], 7.0 + np.minimum(0.33 * (s[s > x] - x), 3.4)])
    fill = np.arange(a, x, LR.GRID)
    rel = np.concatenate([np.full(len(fill), LR.TAIL_WIDTH_M), obs[:, 1] - 7.0])
    obs_s = np.concatenate([fill, obs[:, 0]])
    whole, _ = LR._advanced_fit(obs_s, rel, obs, a, b, base, None, None, False, False, [("birth", x)], {},
                                P, {x: LR.TAIL_WIDTH_M})
    widths = [LR._at(whole, q) - 7.0 for q in np.arange(a, b, 0.1)]
    assert min(widths) >= LR.TAIL_WIDTH_M - 1e-9
    assert LR._at(whole, 0.0) - 7.0 == pytest.approx(LR.TAIL_WIDTH_M, abs=1e-12)
