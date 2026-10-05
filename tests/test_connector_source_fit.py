"""Fidelity-first G2 chains for connector lane centres (connector_source_fit)."""
import math

import numpy as np
import pytest
from scipy.spatial import cKDTree

from mapforge.ops import connector_source_fit as CSF


def _dense(points, step=0.25):
    return CSF._resample(np.asarray(points, float), step=step)


def _p95(lens, ks, start, source):
    s, xy, h = CSF._Chain(*start[:3]).sample(lens, ks)
    src = _dense(source, 0.5)
    return float(np.percentile(cKDTree(xy).query(src)[0], 95))


def test_a_diagonal_through_movement_is_followed_with_short_turns_at_its_kinks():
    # straight across an offset junction: 6 m to the side over 50 m, digitized with a kink at each end
    source = [(0.0, 0.0), (4.0, 0.0), (46.0, 6.0), (50.0, 6.0)]
    start, end = (0.0, 0.0, 0.0, 0.0), (50.0, 6.0, 0.0, 0.0)
    coarse = CSF.fit(source, start, end, seg_m=25.0)          # two long segments: one smooth S
    fine = CSF.fit(source, start, end, seg_m=4.0)
    assert coarse is not None and fine is not None
    assert fine[2]["p95_m"] < 0.1 < coarse[2]["p95_m"]
    lens, ks, info = fine
    assert lens.min() >= CSF.MIN_SEGMENT_M - 1e-9 and np.abs(ks).max() <= CSF.KAPPA_MAX + 1e-9
    x, y, h = CSF._end_pose_exact(start, lens, ks)                   # lands exactly on the end lane centre
    assert math.hypot(x - end[0], y - end[1]) <= 1e-7 and abs(CSF._wrap(h - end[2])) <= 1e-9
    assert _p95(lens, ks, start, source) == pytest.approx(info["p95_m"], abs=0.05)


def test_a_turn_side_fit_keeps_every_knot_curvature_on_the_turn_side():
    # a left turn of 90 deg on a 15 m radius with straight approaches
    a = np.linspace(0.0, math.pi / 2, 40)
    arc = np.column_stack([10.0 + 15.0 * np.sin(a), 15.0 - 15.0 * np.cos(a)])
    source = np.vstack([[[0.0, 0.0]], arc, [[25.0, 25.0]]])
    start, end = (0.0, 0.0, 0.0, 0.0), (25.0, 25.0, math.pi / 2, 0.0)
    found = CSF.fit(source, start, end, seg_m=5.0, turn_sign=1)
    assert found is not None
    lens, ks, info = found
    assert ks[1:-1].min() >= -CSF.TURN_SIGN_TOL - 1e-12
    assert info["p95_m"] < 0.2


def test_written_primitives_chain_end_to_end():
    lens, ks = np.array([5.0, 7.0, 4.0]), np.array([0.0, 0.05, -0.02, 0.0])
    prims = CSF.primitives((1.0, 2.0, 0.3), lens, ks)
    assert [p[0] for p in prims] == ["spiral", "spiral", "spiral"]
    from pyclothoids import Clothoid
    for a, b in zip(prims, prims[1:]):
        cl = Clothoid.StandardParams(a[1], a[2], a[3], a[5], (a[6] - a[5]) / a[4], a[4])
        assert (cl.XEnd, cl.YEnd) == pytest.approx((b[1], b[2]), abs=1e-9) and a[6] == pytest.approx(b[5])
    s, xy, h = CSF._Chain(1.0, 2.0, 0.3).sample(lens, ks, step=0.01)
    cl = Clothoid.StandardParams(prims[-1][1], prims[-1][2], prims[-1][3], prims[-1][5],
                                 (prims[-1][6] - prims[-1][5]) / prims[-1][4], prims[-1][4])
    assert np.linalg.norm(xy[-1] - (cl.XEnd, cl.YEnd)) < 1e-4            # the fast sampler agrees


def test_inactive_knots_merge_into_long_primitives():
    # 30 m straight, then a 90 deg left arc of radius 20 m: the 3 m knots on the straight and on the arc carry no
    # rate change, so they merge, and at PRIMITIVE_COST_M per primitive the chain becomes line - clothoid - arc
    # (the G2 chain needs the transition the G1 source lacks; seven primitives were 0.026 m closer)
    arc = [(30 + 20 * math.sin(t), 20 - 20 * math.cos(t)) for t in np.linspace(0, math.pi / 2, 60)]
    source = [(0.0, 0.0)] + arc
    start, end = (0.0, 0.0, 0.0, 0.0), (50.0, 20.0, math.pi / 2, 0.05)
    lens, ks, info = CSF.fit(source, start, end, seg_m=3.0)
    assert info["merged"] > 0 and len(lens) <= 4 and lens[0] > 20.0 and lens[-1] > 20.0
    assert info["p95_m"] < 0.08 and abs(ks).max() == pytest.approx(0.05, abs=0.005)
    assert abs(ks[1]) < 1e-3 and ks[-2] == pytest.approx(0.05, abs=0.003)          # straight, then the arc
    x, y, h = CSF._end_pose_exact(start, lens, ks)
    assert math.hypot(x - end[0], y - end[1]) <= 1e-7 and abs(CSF._wrap(h - end[2])) <= 1e-9


def test_merging_keeps_the_end_curvatures_and_only_drops_unchanged_rates():
    lens, ks = np.array([4.0, 4.0, 3.0, 5.0]), np.array([0.0, 0.02, 0.04, 0.04, 0.04])
    m_lens, m_ks = CSF._merge_knots(lens, ks, 1e-6)
    assert m_lens.tolist() == [8.0, 8.0] and m_ks.tolist() == [0.0, 0.04, 0.04]
    assert m_lens.sum() == lens.sum()
    kinked = np.array([0.0, 0.03, 0.0, 0.03, 0.0])
    assert CSF._merge_knots(lens, kinked, 1e-3)[0].tolist() == lens.tolist()


@pytest.mark.parametrize("lateral", [None, (0.01, 0.03, -0.02, -0.04)])
def test_the_analytic_jacobian_matches_finite_differences(lateral):
    from scipy.optimize._numdiff import approx_derivative
    arc = [(30 + 20 * math.sin(t), 20 - 20 * math.cos(t)) for t in np.linspace(0, math.pi / 2, 60)]
    src = CSF._resample(np.array([(0.0, 0.0)] + arc), step=CSF.SAMPLE_M)
    tree = cKDTree(src)
    start, end = (0.0, 0.0, 0.0, 0.0), (50.0, 20.0, math.pi / 2, 0.05)
    rng = np.random.default_rng(3)
    n = 9
    z = np.concatenate([rng.uniform(5.0, 7.5, n), rng.uniform(-0.02, 0.08, n - 1)])
    chain = CSF._Chain(*start[:3])

    def unpack(v):
        return v[:n], np.concatenate([[start[3]], v[n:], [end[3]]])

    def f(v):
        return CSF._residual_and_jacobian(chain, src, tree, end, (0.01, -0.01), *unpack(v), lateral)[0]

    jac = CSF._residual_and_jacobian(chain, src, tree, end, (0.01, -0.01), *unpack(z), lateral, with_jacobian=True)[1]
    num = approx_derivative(f, z, method="3-point", rel_step=1e-7)
    assert jac.shape == num.shape
    assert np.abs(jac - num).max() <= 1e-5 * np.abs(num).max()


def test_the_lane_centre_with_its_tilt_cubic_is_what_follows_the_source():
    # a straight source along the lane centre; the road cross-sections at both mouths are turned 2 deg against it
    # (reference heading -0.035, lane-centre slope +0.035 at both ends)
    source = [(0.0, 0.0), (60.0, 0.0)]
    start, end = (0.0, 0.0, -0.035, 0.0), (60.0, 0.0, -0.035, 0.0)
    lateral = (0.0, 0.035, 0.0, 0.035)

    def written_lane(lens, ks):
        s, xy, hh = CSF._Chain(*start[:3]).sample(lens, ks)
        return CSF._lane_points(s, xy, hh, lateral), xy

    lens, ks, info = CSF.fit(source, start, end, seg_m=6.0, lateral=lateral)
    lane, chain = written_lane(lens, ks)
    assert info["p95_m"] < 0.1 and np.abs(lane[:, 1]).max() < 0.1       # the lane centre follows the source
    assert np.abs(chain[:, 1]).max() > 0.15                                # the chain carries the opposite bow
    bare = CSF.fit(source, start, end, seg_m=6.0)                          # fitted without the tilt cubic
    assert np.abs(written_lane(*bare[:2])[0][:, 1]).max() > 0.15           # its written lane centre bows off


def test_end_zone_fit_may_steer_back_only_near_the_ends():
    # a left turn whose source overshoots the final heading by 6 deg in its last 10 m and hooks back
    from mapforge.ops import connector_source_fit as F
    from mapforge.validate.turn_shape import counter_curvature_zones
    s = np.arange(0.0, 60.0, 0.5)
    heading = np.interp(s, [0, 15, 40, 50, 58, 60], np.radians([0, 0, 96, 96, 90, 90]))
    xy = np.column_stack([np.cumsum(np.cos(heading) * 0.5), np.cumsum(np.sin(heading) * 0.5)])
    start = (float(xy[0, 0]), float(xy[0, 1]), float(heading[0]), 0.0)
    end = (float(xy[-1, 0]), float(xy[-1, 1]), float(heading[-1]), 0.0)
    strict = F.fit(xy, start, end, 4.0, turn_sign=1)
    zoned = F.fit(xy, start, end, 4.0, turn_sign=1, end_zone=F.END_ZONE)
    assert strict is not None and zoned is not None
    lens, ks, info = zoned
    stations = np.cumsum(lens)[:-1]
    inner = np.asarray(ks[1:-1])
    middle = (stations > F.END_ZONE[0]) & (stations < lens.sum() - F.END_ZONE[0])
    assert inner[middle].min() >= -F.TURN_SIGN_TOL - 1e-9
    assert inner.min() >= -F.END_ZONE[1] - 1e-9
    assert info["end_zone"] == list(F.END_ZONE)
    assert info["p95_m"] < strict[2]["p95_m"]
    # the zone split of a curvature profile
    profile = [0.05] * 40 + [-0.015] * 8 + [0.05] * 120 + [-0.018] * 12      # 0.25 m steps, 45 m
    mid, ends = counter_curvature_zones(profile, 10.0)
    assert mid == pytest.approx(0.015) and ends == pytest.approx(0.018)
