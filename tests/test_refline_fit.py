# -*- coding: utf-8 -*-
"""拟合器单测：合成真值回拍（类型识别/半径还原/假 arc 抑制/横向偏差）。"""
import numpy as np

from mapforge.ops.refline_fit import (PlanView, PlanSeg, eval_planview, fit_polyline,
                                      lateral_deviation)


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
