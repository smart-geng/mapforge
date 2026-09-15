# -*- coding: utf-8 -*-
"""拟合器单测：合成真值回拍（类型识别/半径还原/假 arc 抑制/横向偏差）。"""
import numpy as np
import pytest

from mapforge.ops.refline_fit import (PlanView, PlanSeg, ReflineFitError, eval_planview,
                                      fit_connector_minimal, fit_leg_refline,
                                      fit_polyline, lateral_deviation,
                                      planview_prims, planview_quality,
                                      solve_g2_balanced)


def synth(segs, step=2.0, noise=0.0, seed=7):
    pv = PlanView(0.0, 0.0, 0.3, [PlanSeg(k, L, c) for k, L, c in segs])
    pts = eval_planview(pv, step=step)
    if noise > 0:
        pts = pts + np.random.default_rng(seed).normal(0.0, noise, pts.shape)
    return pts


def kinds(pv):
    return [s.kind for s in pv.segs]


def test_line_arc_line_recovered():
    pts = synth([("line", 100, 0.0), ("arc", 60, 1 / 80), ("line", 50, 0.0)])
    pv, _ = fit_polyline(pts, kappa_th=1 / 600, min_seg_len=6.0, smooth_win=5)
    assert kinds(pv) == ["line", "arc", "line"]
    arc = pv.segs[1]
    assert abs(abs(1 / arc.curvature) - 80) / 80 < 0.10          # 半径误差 <10%（当前能力水平；边界精修后收紧）
    dmax, _ = lateral_deviation(eval_planview(pv, 0.5), pts)
    assert dmax < 0.35


def test_straight_stays_straight_under_noise():
    pts = synth([("line", 300, 0.0)], noise=0.01)
    pv, _ = fit_polyline(pts, kappa_th=1 / 600, min_seg_len=6.0, smooth_win=7)
    assert kinds(pv) == ["line"]                                  # 保真约束：不臆造弯道


def test_false_arc_normalized_to_line():
    # R=10km 的微弯（低于 max_radius=3000 阈值）应被归直
    pts = synth([("line", 80, 0.0), ("arc", 100, 1 / 10000), ("line", 80, 0.0)])
    pv, _ = fit_polyline(pts, kappa_th=1 / 20000, min_seg_len=6.0, smooth_win=5)
    assert all(k == "line" for k in kinds(pv))


def test_s_curve_sign_split():
    pts = synth([("arc", 60, 1 / 60), ("arc", 60, -1 / 60)])
    pv, _ = fit_polyline(pts, kappa_th=1 / 600, min_seg_len=6.0, smooth_win=5)
    arcs = [s for s in pv.segs if s.kind == "arc"]
    assert len(arcs) >= 2
    assert arcs[0].curvature * arcs[-1].curvature < 0             # 符号相反的两段


def test_planview_quality_exposes_micro_segment():
    pv = PlanView(0.0, 0.0, 0.0, [
        PlanSeg("spiral", 12.0, 0.0, 0.01),
        PlanSeg("spiral", 0.6, 0.01, -0.01),
        PlanSeg("spiral", 15.0, -0.01, 0.0),
    ])
    q = planview_quality(pv)
    assert q["seg_min_len"] == pytest.approx(0.6)
    assert q["sharpness_max"] > 0.03
    assert q["sharp_sign_flips"] == 2


def test_leg_fit_never_returns_invalid_fallback(monkeypatch):
    """候选搜索失败必须阻断；旧实现会把不合格 fallback 当成“最优”继续交付。"""
    import mapforge.ops.refline_fit as rf

    monkeypatch.setattr(rf, "_leg_candidate_ok",
                        lambda *_a, **_kw: (False, {}))
    monkeypatch.setattr(rf, "fit_connector_minimal", lambda *_a, **_kw: None)
    pts = np.array([[0.0, 0.0], [20.0, 1.0], [40.0, 0.0]])
    with pytest.raises(ReflineFitError, match="无合格候选"):
        fit_leg_refline(pts)


def test_leg_deviation_cap_cannot_be_relaxed_silently():
    pts = np.array([[0.0, 0.0], [20.0, 0.0], [40.0, 0.0]])
    with pytest.raises(ValueError, match="dev_tol"):
        fit_leg_refline(pts, dev_tol=2.0)


def test_minimal_connector_recovers_measured_g2_shape_without_fragments():
    """四段合成真值：端点精确、来源贴合，且不得退回逐点碎片链。"""
    truth = PlanView(2.0, -3.0, 0.25, [
        PlanSeg("spiral", 8.0, 0.0, -0.02),
        PlanSeg("spiral", 15.0, -0.02, 0.08),
        PlanSeg("spiral", 15.0, 0.08, -0.02),
        PlanSeg("spiral", 8.0, -0.02, 0.0),
    ])
    source = eval_planview(truth, 0.5)
    _p, end = planview_prims(truth)
    got = fit_connector_minimal(source, (truth.x0, truth.y0, truth.hdg), end,
                                max_segments=5)
    assert got is not None
    assert len(got.planview.segs) <= 5
    assert min(x.length for x in got.planview.segs) >= \
        max(1.0, 0.03 * sum(x.length for x in got.planview.segs))
    assert got.metrics["source_to_target"]["p95_m"] <= 1.5
    assert got.metrics["target_to_source"]["p95_m"] <= 1.5
    _out, actual_end = planview_prims(got.planview)
    assert np.linalg.norm(np.asarray(actual_end[:2]) - np.asarray(end[:2])) < 1e-5
    assert abs(np.arctan2(np.sin(actual_end[2] - end[2]),
                          np.cos(actual_end[2] - end[2]))) < 1e-6


def test_minimal_connector_supports_internal_only_source_geometry():
    """IBD 内部连接线可只覆盖中段；两端空白必须由同一少段 G2 链桥接。"""
    truth = PlanView(0.0, 0.0, 0.0, [
        PlanSeg("spiral", 12.0, 0.0, 0.03),
        PlanSeg("spiral", 18.0, 0.03, 0.03),
        PlanSeg("spiral", 12.0, 0.03, 0.0),
    ])
    full = eval_planview(truth, 0.5)
    source = full[16:-16]                                  # 两端各缺约 8m
    _p, end = planview_prims(truth)
    got = fit_connector_minimal(
        source, (truth.x0, truth.y0, truth.hdg), end,
        max_segments=5, source_covers_endpoints=False, min_segment_m=6.0)
    assert got is not None
    assert len(got.planview.segs) <= 5
    assert got.metrics["source_support_s"] is not None
    assert got.metrics["source_support_s"][0] > 3.0
    assert got.metrics["source_support_s"][1] < got.metrics["length_m"] - 3.0
    assert got.metrics["min_primitive_m"] >= 6.0
    assert got.metrics["source_to_target"]["p95_m"] < 0.75


def test_balanced_g2_preserves_exact_boundary_and_does_not_raise_sharpness():
    p0 = (1.5, -2.0, 0.12)
    p1 = (15.0, 8.5, 1.45)
    cls, meta = solve_g2_balanced(p0, p1, 0.0, 0.001)
    pv = PlanView(*p0, [
        PlanSeg("spiral", c.length, c.KappaStart, c.KappaEnd) for c in cls
    ])
    _prims, end = planview_prims(pv)
    assert len(cls) == 3
    assert np.linalg.norm(np.asarray(end[:2]) - np.asarray(p1[:2])) < 1e-6
    assert abs(np.arctan2(np.sin(end[2] - p1[2]),
                          np.cos(end[2] - p1[2]))) < 1e-7
    assert meta["selected"]["sharpness_max_per_m2"] <= \
        meta["base"]["sharpness_max_per_m2"] + 1e-12
    assert meta["selected"]["length_m"] <= \
        meta["base"]["length_m"] * 1.10 + 1e-9
