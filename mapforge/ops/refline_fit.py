# -*- coding: utf-8 -*-
"""参考线拟合核心：折线 → 曲率域分段（line/arc）→ 逐段拟合 → G1 链化重建。

方案 5.7 默认档的 line/arc 实现（spiral/paramPoly3 档后续接入 pyclothoids/scipy）。
算法依据见 docs/参考文献-参考线拟合与OpenDRIVE生成.md：
- 分段识别：曲率域检测（Camacho-Torregrosa 2015 航向图思想的曲率域实现）
- 圆拟合：Taubin 代数圆拟合（中心）+ 平均距离半径
- 防过拟合：可选预平滑（GCV 平滑样条）+ 噪声感知曲率阈值，低于阈值归零拟直
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

LINE = "line"
ARC = "arc"


# ---------------------------------------------------------------- 基础几何量

def arclength(pts: np.ndarray) -> np.ndarray:
    """折线累计弧长，pts 形如 (n,2)。返回 (n,)。"""
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def discrete_curvature(pts: np.ndarray):
    """离散航向与曲率。

    返回 (s_theta, theta, s_kappa, kappa)：
    theta 定义在各线段中点（n-1 个，已 unwrap），kappa 定义在内部顶点（n-2 个）。
    """
    s = arclength(pts)
    seg = np.diff(pts, axis=0)
    theta = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    s_theta = 0.5 * (s[:-1] + s[1:])
    kappa = np.diff(theta) / np.maximum(np.diff(s_theta), 1e-9)
    s_kappa = s[1:-1]
    return s_theta, theta, s_kappa, kappa


def moving_average(x: np.ndarray, win: int) -> np.ndarray:
    if win <= 1 or x.size < 3:
        return x
    win = min(win | 1, x.size if x.size % 2 == 1 else x.size - 1)  # 奇数窗
    pad = win // 2
    xp = np.pad(x, pad, mode="edge")
    kernel = np.ones(win) / win
    return np.convolve(xp, kernel, mode="valid")


def presmooth(pts: np.ndarray, lam: float | None = None) -> np.ndarray:
    """GCV 平滑样条预平滑（x(s)、y(s) 各自逼近，lam=None 自动）。scipy 缺失时原样返回。"""
    try:
        from scipy.interpolate import make_smoothing_spline
    except ImportError:  # pragma: no cover
        return pts
    s = arclength(pts)
    if np.any(np.diff(s) <= 0):
        keep = np.concatenate([[True], np.diff(s) > 1e-9])
        pts, s = pts[keep], s[keep]
    fx = make_smoothing_spline(s, pts[:, 0], lam=lam)
    fy = make_smoothing_spline(s, pts[:, 1], lam=lam)
    return np.column_stack([fx(s), fy(s)])


# ---------------------------------------------------------------- 分段识别

@dataclass
class Piece:
    kind: str          # LINE / ARC
    i0: int            # 起点索引（含）
    i1: int            # 终点索引（含）
    curvature: float = 0.0
    fit_rms: float = 0.0

    def length_of(self, s: np.ndarray) -> float:
        return float(s[self.i1] - s[self.i0])


def _runs(mask: np.ndarray):
    """布尔序列 → [(value, start, end_inclusive)]（索引针对 mask）。"""
    out = []
    start = 0
    for i in range(1, mask.size + 1):
        if i == mask.size or mask[i] != mask[start]:
            out.append((bool(mask[start]), start, i - 1))
            start = i
    return out


def _split_compound(ks: np.ndarray, a: int, b: int, min_seg_len: float, s_k: np.ndarray):
    """同号 arc 区间 [a,b]（ks 索引）内按 κ 均值突变分割复合曲线。

    贪心：从左向右维持当前段 κ 中位数，遇到相对偏差 >0.35 且剩余长度足够时开新段。"""
    if b - a < 4:
        return [(a, b)]
    cuts = [a]
    ref = np.median(ks[a:min(b + 1, a + 5)])
    for j in range(a + 2, b - 1):
        cur = np.median(ks[max(cuts[-1], j - 3):j + 1])
        if abs(ks[j] - ref) > 0.35 * max(abs(ref), 1e-9) and \
           abs(np.median(ks[j:min(j + 4, b + 1)]) - ref) > 0.35 * max(abs(ref), 1e-9):
            if (s_k[j] - s_k[cuts[-1]]) >= min_seg_len and (s_k[b] - s_k[j]) >= min_seg_len:
                cuts.append(j)
                ref = np.median(ks[j:min(j + 5, b + 1)])
        else:
            ref = 0.7 * ref + 0.3 * cur
    cuts.append(b)
    return [(cuts[i], cuts[i + 1]) for i in range(len(cuts) - 1)]


def segment_polyline(pts: np.ndarray, kappa_th: float, min_seg_len: float,
                     smooth_win: int = 5) -> list[Piece]:
    """曲率域分段：|kappa|>=kappa_th 判 ARC，其余 LINE；短段并邻，ARC 段按符号再分。"""
    s = arclength(pts)
    _, _, s_k, kappa = discrete_curvature(pts)
    ks = moving_average(kappa, smooth_win)
    mask = np.abs(ks) >= kappa_th          # True=arc；索引 j 对应顶点 j+1

    runs = _runs(mask)
    # 短段并入较长邻段，直至稳定
    changed = True
    while changed and len(runs) > 1:
        changed = False
        for j, (val, a, b) in enumerate(runs):
            seg_len = s_k[b] - s_k[a] if b > a else 0.0
            if seg_len < min_seg_len:
                left = runs[j - 1] if j > 0 else None
                right = runs[j + 1] if j + 1 < len(runs) else None
                if left is None and right is None:
                    continue
                take_left = right is None or (
                    left is not None and (s_k[left[2]] - s_k[left[1]]) >= (s_k[right[2]] - s_k[right[1]]))
                if take_left:
                    runs[j - 1] = (left[0], left[1], b)
                else:
                    runs[j + 1] = (right[0], a, right[2])
                runs.pop(j)
                changed = True
                break

    pieces: list[Piece] = []
    for val, a, b in runs:
        i0 = a + 1 - 1  # 顶点索引：kappa[j] 属顶点 j+1，段取包络点
        i0 = max(0, a)
        i1 = min(pts.shape[0] - 1, b + 2)
        if not val:
            pieces.append(Piece(LINE, i0, i1))
            continue
        # ARC run 内先按曲率符号再分（S 弯），再按同号 κ 突变分（复合曲线）
        sign = np.sign(ks[a:b + 1])
        for sval, sa, sb in _runs(sign > 0):
            for ca, cb in _split_compound(ks, a + sa, a + sb, min_seg_len, s_k):
                j0 = max(0, ca)
                j1 = min(pts.shape[0] - 1, cb + 2)
                if s[j1] - s[j0] < min_seg_len and pieces:
                    pieces[-1].i1 = j1        # 太短并入前段
                else:
                    pieces.append(Piece(ARC, j0, j1))
    # 相邻段共享端点，保证覆盖连续
    for k in range(1, len(pieces)):
        pieces[k].i0 = pieces[k - 1].i1
    if pieces:
        pieces[0].i0 = 0
        pieces[-1].i1 = pts.shape[0] - 1
    return [p for p in pieces if p.i1 > p.i0]


# ---------------------------------------------------------------- 逐段拟合

def fit_line(pts: np.ndarray):
    """总最小二乘直线（PCA）。返回 (单位方向, rms)，方向取前进向。"""
    c = pts.mean(axis=0)
    q = pts - c
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    d = vt[0]
    if np.dot(pts[-1] - pts[0], d) < 0:
        d = -d
    rms = float(np.sqrt(np.mean((q @ np.array([-d[1], d[0]])) ** 2)))
    return d, rms


def fit_circle_taubin(pts: np.ndarray):
    """Taubin 代数圆拟合（Chernov 牛顿形式）求中心，半径取平均距离。

    返回 (center(2,), R, rms)。"""
    x = pts[:, 0]; y = pts[:, 1]
    xm, ym = x.mean(), y.mean()
    u, v = x - xm, y - ym
    z = u * u + v * v
    Mxx, Myy, Mxy = (u * u).mean(), (v * v).mean(), (u * v).mean()
    Mxz, Myz, Mzz = (u * z).mean(), (v * z).mean(), (z * z).mean()
    Mz = Mxx + Myy
    Cov_xy = Mxx * Myy - Mxy * Mxy
    Var_z = Mzz - Mz * Mz
    A3 = 4.0 * Mz
    A2 = -3.0 * Mz * Mz - Mzz
    A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy
    xn, yn = 0.0, 1e30
    for _ in range(30):
        yo = yn
        yn = A0 + xn * (A1 + xn * (A2 + xn * A3))
        if abs(yn) > abs(yo):
            xn = 0.0
            break
        dy = A1 + xn * (2 * A2 + xn * 3 * A3)
        if dy == 0:
            break
        xo = xn
        xn = xo - yn / dy
        if xn != 0 and abs((xn - xo) / xn) < 1e-12:
            break
    det = xn * xn - xn * Mz + Cov_xy
    if abs(det) < 1e-15:
        return np.array([xm, ym]), float("inf"), float("inf")
    cx = (Mxz * (Myy - xn) - Myz * Mxy) / det / 2.0
    cy = (Myz * (Mxx - xn) - Mxz * Mxy) / det / 2.0
    center = np.array([cx + xm, cy + ym])
    d = np.linalg.norm(pts - center, axis=1)
    R = float(d.mean())
    rms = float(np.sqrt(np.mean((d - R) ** 2)))
    return center, R, rms


# ---------------------------------------------------------------- G1 链化与评估

@dataclass
class PlanSeg:
    kind: str
    length: float
    curvature: float = 0.0


@dataclass
class PlanView:
    x0: float
    y0: float
    hdg: float
    segs: list[PlanSeg] = field(default_factory=list)


def fit_pieces(pts: np.ndarray, pieces: list[Piece]) -> None:
    """逐段拟合，填 Piece.curvature / fit_rms（LINE 恒 0 曲率）。"""
    for p in pieces:
        sub = pts[p.i0:p.i1 + 1]
        if p.kind == LINE or sub.shape[0] < 5:
            p.kind = LINE
            _, p.fit_rms = fit_line(sub)
            p.curvature = 0.0
        else:
            _, R, rms = fit_circle_taubin(sub)
            if not math.isfinite(R) or R <= 1e-3:
                p.kind = LINE
                _, p.fit_rms = fit_line(sub)
                p.curvature = 0.0
                continue
            seg = np.diff(sub, axis=0)
            th = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
            sign = 1.0 if th[-1] >= th[0] else -1.0
            p.curvature = sign / R
            p.fit_rms = rms


def chain_g1(pts: np.ndarray, pieces: list[Piece]) -> PlanView:
    """按各段 (kind, 弧长, 拟合曲率) 从首点位姿传播——planView 模型天然 G1。"""
    s = arclength(pts)
    first = pieces[0]
    sub0 = pts[first.i0:first.i1 + 1]
    if first.kind == LINE:
        d, _ = fit_line(sub0)
        hdg0 = math.atan2(d[1], d[0])
    else:
        c, R, _ = fit_circle_taubin(sub0)
        r0 = pts[first.i0] - c
        t0 = np.array([-r0[1], r0[0]]) * np.sign(first.curvature)
        hdg0 = math.atan2(t0[1], t0[0])
    pv = PlanView(float(pts[0, 0]), float(pts[0, 1]), hdg0)
    for p in pieces:
        pv.segs.append(PlanSeg(p.kind, p.length_of(s), p.curvature))
    return pv


def eval_planview(pv: PlanView, step: float = 0.5) -> np.ndarray:
    """解析采样重建曲线（line/arc）。"""
    out = [(pv.x0, pv.y0)]
    x, y, h = pv.x0, pv.y0, pv.hdg
    for seg in pv.segs:
        n = max(2, int(seg.length / step) + 1)
        ss = np.linspace(0.0, seg.length, n)[1:]
        if seg.kind == LINE or abs(seg.curvature) < 1e-12:
            xs = x + ss * math.cos(h)
            ys = y + ss * math.sin(h)
            x, y = float(xs[-1]), float(ys[-1])
        else:
            k = seg.curvature
            xs = x + (np.sin(h + k * ss) - math.sin(h)) / k
            ys = y - (np.cos(h + k * ss) - math.cos(h)) / k
            x, y = float(xs[-1]), float(ys[-1])
            h = h + k * seg.length
        out.extend(zip(xs.tolist(), ys.tolist()))
    return np.asarray(out)


def lateral_deviation(recon: np.ndarray, src: np.ndarray):
    """recon 各点到 src 折线的距离（最大 / 均值）。KDTree 最近顶点 + 邻接线段投影。"""
    from scipy.spatial import cKDTree
    tree = cKDTree(src)
    _, idx = tree.query(recon)
    d = np.empty(recon.shape[0])
    for i, (p, j) in enumerate(zip(recon, idx)):
        best = np.linalg.norm(p - src[j])
        for a in (j - 1, j):
            if 0 <= a < src.shape[0] - 1:
                seg = src[a + 1] - src[a]
                L2 = float(seg @ seg)
                if L2 > 1e-12:
                    t = float(np.clip((p - src[a]) @ seg / L2, 0.0, 1.0))
                    best = min(best, float(np.linalg.norm(src[a] + t * seg - p)))
        d[i] = best
    return float(d.max()), float(d.mean())


def refine_planview(pv: PlanView, src: np.ndarray, max_nfev: int = 60) -> PlanView:
    """全局位姿优化：以起点位姿 + 各 arc 段曲率为变量，最小化重建曲线到源折线的距离。

    消除朴素传播的累计漂移（文献路线"分段拟合→整体优化"的优化环节）。
    LINE 段曲率恒 0 不参与优化（保真约束：直就是直）。"""
    from scipy.optimize import least_squares
    from scipy.spatial import cKDTree
    tree = cKDTree(src)
    arc_idx = [i for i, sg in enumerate(pv.segs) if sg.kind == ARC and abs(sg.curvature) > 1e-12]
    x0 = np.array([pv.x0, pv.y0, pv.hdg] + [pv.segs[i].curvature for i in arc_idx])
    scale = np.array([1.0, 1.0, 0.01] + [max(abs(pv.segs[i].curvature), 1e-4) * 0.5 for i in arc_idx])

    def build(v):
        pv2 = PlanView(float(v[0]), float(v[1]), float(v[2]),
                       [PlanSeg(sg.kind, sg.length, sg.curvature) for sg in pv.segs])
        for j, i in enumerate(arc_idx):
            pv2.segs[i].curvature = float(v[3 + j])
        return pv2

    def residual(v):
        d, _ = tree.query(eval_planview(build(v), step=2.0))
        return d

    try:
        res = least_squares(residual, x0, x_scale=scale, max_nfev=max_nfev, method="trf")
        return build(res.x)
    except Exception:
        return pv


def postprocess_planview(pv: PlanView, max_radius: float = 3000.0,
                         merge_arcs: bool = True) -> PlanView:
    """线形保真后处理：① |R|>max_radius 的假 arc 归直（"直就是直"）；② line+line 无损合并
    （传播模型中 line 不改航向）；③ 可选 arc 合并（曲率差 <15% 弧长加权——仅初拟阶段用，
    refine 之后禁用以免把复合弧误并，见 Spike-A road76 教训）。"""
    if not pv.segs:
        return pv
    segs = []
    for s in pv.segs:
        if s.kind == ARC and abs(s.curvature) < 1.0 / max_radius:
            s = PlanSeg(LINE, s.length, 0.0)
        segs.append(PlanSeg(s.kind, s.length, s.curvature))
    merged = [segs[0]]
    for s in segs[1:]:
        last = merged[-1]
        if s.kind == LINE and last.kind == LINE:
            merged[-1] = PlanSeg(LINE, last.length + s.length, 0.0)
        elif (merge_arcs and s.kind == ARC and last.kind == ARC
              and abs(s.curvature - last.curvature) < 0.15 * max(abs(last.curvature), 1e-9)):
            k = (last.curvature * last.length + s.curvature * s.length) / (last.length + s.length)
            merged[-1] = PlanSeg(ARC, last.length + s.length, k)
        else:
            merged.append(s)
    pv.segs = merged
    return pv


def fit_polyline(pts: np.ndarray, kappa_th: float = 1.0 / 2000.0,
                 min_seg_len: float = 8.0, smooth_win: int = 5,
                 do_presmooth: bool = False, refine: bool = True,
                 max_radius: float = 3000.0) -> tuple[PlanView, list[Piece]]:
    """折线 → PlanView 一站式入口（分段 → 逐段拟合 → G1 链化 → 保真后处理 → 全局优化）。"""
    work = presmooth(pts) if do_presmooth else pts
    pieces = segment_polyline(work, kappa_th, min_seg_len, smooth_win)
    if not pieces:
        pieces = [Piece(LINE, 0, pts.shape[0] - 1)]
    fit_pieces(work, pieces)
    pv = chain_g1(work, pieces)
    pv = postprocess_planview(pv, max_radius)
    if refine:
        pv = refine_planview(pv, work)
        pv = postprocess_planview(pv, max_radius, merge_arcs=False)   # refine 后仅归直/并线，不并弧
        pv = refine_planview(pv, work, max_nfev=20)   # 后处理改动曲率后轻量收拾，防传播漂移
    return pv, pieces
