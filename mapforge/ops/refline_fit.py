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
    kind: str                          # line / arc / spiral
    length: float
    curvature: float = 0.0             # spiral 时为起点曲率
    curvature_end: float | None = None  # 仅 spiral：终点曲率


@dataclass
class PlanView:
    x0: float
    y0: float
    hdg: float
    segs: list[PlanSeg] = field(default_factory=list)
    fit_meta: dict = field(default_factory=dict)


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
    """解析采样重建曲线（line/arc/spiral）。"""
    out = [(pv.x0, pv.y0)]
    x, y, h = pv.x0, pv.y0, pv.hdg
    for seg in pv.segs:
        n = max(2, int(seg.length / step) + 1)
        ss = np.linspace(0.0, seg.length, n)[1:]
        if seg.kind == "spiral":
            from pyclothoids import Clothoid
            dk = (seg.curvature_end - seg.curvature) / max(seg.length, 1e-9)
            cl = Clothoid.StandardParams(x, y, h, seg.curvature, dk, seg.length)
            xs_l, ys_l = cl.SampleXY(n)
            xs, ys = np.asarray(xs_l[1:]), np.asarray(ys_l[1:])
            x, y, h = float(cl.XEnd), float(cl.YEnd), float(cl.ThetaEnd)
        elif seg.kind == LINE or abs(seg.curvature) < 1e-12:
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


def resample_polyline(pts: np.ndarray, step: float = 2.0) -> np.ndarray:
    """按弧长等距重采样（稀疏折线拟合前的稳定化）。必含终点——丢末点会让拟合路短一截。"""
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < 2 * step:
        return pts
    u = np.arange(0.0, s[-1], step)
    if s[-1] - u[-1] > 1e-9:
        u = np.append(u, s[-1])
    return np.column_stack([np.interp(u, s, pts[:, 0]), np.interp(u, s, pts[:, 1])])


def vertex_tangents(pts: np.ndarray) -> np.ndarray:
    """稀疏折线各顶点的切向估计：内点取相邻弦向单位向量之和（圆上等弦时精确等于切线），
    端点用相邻内点切向对弦向反射外推。"""
    seg = np.diff(pts, axis=0)
    ang = np.arctan2(seg[:, 1], seg[:, 0])
    u = np.column_stack([np.cos(ang), np.sin(ang)])
    th = np.empty(pts.shape[0])
    inner = u[:-1] + u[1:]
    th[1:-1] = np.arctan2(inner[:, 1], inner[:, 0])
    th[0] = 2 * ang[0] - th[1]
    th[-1] = 2 * ang[-1] - th[-2]
    return th


def _dedupe_vertices(pts: np.ndarray, min_gap: float = 3.0) -> np.ndarray:
    """近重复顶点合并（保首末点）。拼链接点常留 0.0–0.9m 间距的双点，
    在其上做 G1 插值会产生 κ>20 的退化微回旋线。"""
    keep = [0]
    for i in range(1, pts.shape[0]):
        if float(np.linalg.norm(pts[i] - pts[keep[-1]])) >= min_gap:
            keep.append(i)
    if keep[-1] != pts.shape[0] - 1:
        if len(keep) > 1 and float(np.linalg.norm(pts[-1] - pts[keep[-1]])) < min_gap:
            keep[-1] = pts.shape[0] - 1                  # 用真实末点顶掉近末点
        else:
            keep.append(pts.shape[0] - 1)
    return pts[keep]


def fit_sparse_g1_spline(pts: np.ndarray) -> PlanView | None:
    """稀疏链档：逐顶点 G1 clothoid 插值（Bertolazzi–Frego G1Hermite，文献档 §1）。

    适用：附录 D 抽稀后的长弦点列——保留点是真实路面点，重采样再分段会把渐变曲率
    （缓和曲线）误拟成直线+圆角链；clothoid 顶点插值精确过点、曲率分段线性、可直接
    写为 OpenDRIVE spiral。仅在稀疏链（中位弦长 >10m）触发——密集含噪数据禁用
    插值路线（方案 5.7 保真约束），仍走逼近拟合。顶点先去重；产物含 |κ|>0.15
    （R<6.7m，干线主线不可能）视为退化，返回 None 由调用侧弃用。"""
    from pyclothoids import Clothoid
    pts = _dedupe_vertices(pts)
    if pts.shape[0] < 3:
        return None
    th = vertex_tangents(pts)
    segs = []
    for i in range(pts.shape[0] - 1):
        cl = Clothoid.G1Hermite(pts[i, 0], pts[i, 1], th[i],
                                pts[i + 1, 0], pts[i + 1, 1], th[i + 1])
        if max(abs(cl.KappaStart), abs(cl.KappaEnd)) > 0.15:
            return None                                  # 退化微回旋线：整条弃用
        segs.append(PlanSeg("spiral", cl.length, cl.KappaStart, cl.KappaEnd))
    return PlanView(float(pts[0, 0]), float(pts[0, 1]), float(th[0]), segs)


def fit_polyline_auto(pts: np.ndarray, resample_step: float = 2.0,
                      min_seg_len: float = 6.0) -> tuple[PlanView, float]:
    """带偏差驱动升级档的拟合入口（SHP 直转与 MAP 还原共用）。三档：

    ① 基线（2m 重采样 line/arc）；② 细档（1m + 小分段，抓 5° 微折角 → line-arc-line）；
    ③ 稀疏链档（G1 clothoid 顶点插值 spline）——处理抽稀点列上的渐变曲率（缓和曲线）。
    择优与返回的偏差一律**对原始顶点**度量：抽稀保留点是真实路面点，弦上插值点不是测量，
    平滑曲线正确鼓出弦线不应被惩罚。返回 (PlanView, 顶点最大偏差 m)。"""
    def vdev(pv):
        curve = eval_planview(pv, 0.25)
        return float(np.linalg.norm(curve[None, :, :] - pts[:, None, :],
                                    axis=2).min(axis=1).max())

    pv1, _ = fit_polyline(resample_polyline(pts, resample_step),
                          kappa_th=1 / 600, min_seg_len=min_seg_len, smooth_win=3)
    best, dbest = pv1, vdev(pv1)
    if dbest > 0.30:
        cands = [fit_polyline(resample_polyline(pts, 1.0),
                              kappa_th=1 / 3000, min_seg_len=2.0, smooth_win=3)[0]]
        seglen = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        if pts.shape[0] >= 4 and float(np.median(seglen)) > 10.0:
            sp = fit_sparse_g1_spline(pts)
            if sp is not None:                           # None=退化（微回旋线熔断）
                cands.append(sp)
        for pv in cands:
            d = vdev(pv)
            if d < dbest:
                best, dbest = pv, d
    return best, dbest


def planview_prims(pv: PlanView):
    """PlanView → writer 几何原语 [(kind, x, y, hdg, L, k0, k1)] + 末端位姿。
    起点位姿逐段解析推进（line/arc 闭式、spiral 用 clothoid），供自研 writer 消费。"""
    prims = []
    x, y, h = pv.x0, pv.y0, pv.hdg
    for seg in pv.segs:
        if seg.kind == "spiral":
            from pyclothoids import Clothoid
            dk = (seg.curvature_end - seg.curvature) / max(seg.length, 1e-9)
            cl = Clothoid.StandardParams(x, y, h, seg.curvature, dk, seg.length)
            prims.append(("spiral", x, y, h, seg.length, seg.curvature, seg.curvature_end))
            x, y, h = float(cl.XEnd), float(cl.YEnd), float(cl.ThetaEnd)
        elif seg.kind == "line" or abs(seg.curvature) < 1e-12:
            prims.append(("line", x, y, h, seg.length, 0.0, 0.0))
            x += seg.length * math.cos(h)
            y += seg.length * math.sin(h)
        else:
            k = seg.curvature
            prims.append(("arc", x, y, h, seg.length, k, k))
            x = x + (math.sin(h + k * seg.length) - math.sin(h)) / k
            y = y - (math.cos(h + k * seg.length) - math.cos(h)) / k
            h += k * seg.length
    return prims, (x, y, h)


def seg_kappa(pv: PlanView, at_end: bool) -> float:
    """参考线端部曲率（G2 桥/接缝用）。"""
    seg = pv.segs[-1] if at_end else pv.segs[0]
    if seg.kind == "spiral":
        return seg.curvature_end if at_end else seg.curvature
    return seg.curvature if seg.kind == "arc" else 0.0


def _movavg_keep_ends(pts: np.ndarray, w: int) -> np.ndarray:
    """内点滑动平均（端点不动——路口侧位姿是接缝锚点）。"""
    if len(pts) <= w:
        return pts
    k = np.ones(w) / w
    sm = np.column_stack([np.convolve(pts[:, 0], k, mode="same"),
                          np.convolve(pts[:, 1], k, mode="same")])
    h = w // 2 + 1
    sm[:h], sm[-h:] = pts[:h], pts[-h:]
    return sm


def max_kappa(pv: PlanView) -> float:
    out = 0.0
    for seg in pv.segs:
        if seg.kind == "arc":
            out = max(out, abs(seg.curvature))
        elif seg.kind == "spiral":
            out = max(out, abs(seg.curvature), abs(seg.curvature_end or 0.0))
    return out


class ReflineFitError(ValueError):
    """参考线无法同时满足来源偏差与可驾驶几何硬门禁。"""


@dataclass
class MinimalConnectorFit:
    """路口连接线的少段约束拟合结果。

    ``primitives`` 使用 writer 的 ``(kind, x, y, hdg, L, k0, k1)`` 契约；
    ``metrics`` 同时保留双向来源偏差和动力学相关的曲率指标，避免调用侧只看
    “端点接上了”便把几何误判为合格。
    """

    planview: PlanView
    primitives: list[tuple]
    method: str
    metrics: dict


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _resample_count(pts: np.ndarray, count: int) -> np.ndarray:
    pts = np.asarray(pts, float)
    s = arclength(pts)
    if s[-1] <= 1e-9:
        return np.repeat(pts[:1], count, axis=0)
    q = np.linspace(0.0, float(s[-1]), count)
    return np.column_stack([np.interp(q, s, pts[:, 0]),
                            np.interp(q, s, pts[:, 1])])


def _planview_at_s(pv: PlanView, query_s: np.ndarray) -> np.ndarray:
    """在给定弧长位置解析求值；优化器使用，不能依赖离散点索引对应。"""
    from pyclothoids import Clothoid

    qs = np.asarray(query_s, float)
    bounds = np.concatenate([[0.0], np.cumsum([x.length for x in pv.segs])])
    x, y, h = pv.x0, pv.y0, pv.hdg
    curves = []
    for seg in pv.segs:
        k0 = 0.0 if seg.kind == "line" else float(seg.curvature)
        k1 = (float(seg.curvature_end) if seg.kind == "spiral"
              else k0)
        cl = Clothoid.StandardParams(x, y, h, k0,
                                     (k1 - k0) / max(seg.length, 1e-12),
                                     seg.length)
        curves.append(cl)
        x, y, h = float(cl.XEnd), float(cl.YEnd), float(cl.ThetaEnd)
    out = []
    for s in qs:
        i = int(np.searchsorted(bounds, s, side="right") - 1)
        i = min(max(i, 0), len(curves) - 1)
        u = min(max(float(s - bounds[i]), 0.0), pv.segs[i].length)
        out.append((float(curves[i].X(u)), float(curves[i].Y(u))))
    return np.asarray(out)


def _chain_from_knots(p0, lengths: np.ndarray, kappas: np.ndarray) -> PlanView:
    segs = []
    for length, k0, k1 in zip(lengths, kappas[:-1], kappas[1:]):
        if abs(k1 - k0) <= 1e-11:
            segs.append(PlanSeg("line" if abs(k0) <= 1e-11 else "arc",
                                float(length), float(k0), None))
        else:
            segs.append(PlanSeg("spiral", float(length), float(k0), float(k1)))
    return PlanView(float(p0[0]), float(p0[1]), float(p0[2]), segs)


def _canonicalize_connector(pv: PlanView) -> PlanView:
    """仅合并数学上完全等价的相邻曲率线性段，不做近似删段。"""
    out: list[PlanSeg] = []
    for seg in pv.segs:
        k0 = 0.0 if seg.kind == "line" else float(seg.curvature)
        k1 = float(seg.curvature_end) if seg.kind == "spiral" else k0
        if out:
            prev = out[-1]
            p0 = 0.0 if prev.kind == "line" else float(prev.curvature)
            p1 = float(prev.curvature_end) if prev.kind == "spiral" else p0
            slope0 = (p1 - p0) / max(prev.length, 1e-12)
            slope1 = (k1 - k0) / max(seg.length, 1e-12)
            if abs(p1 - k0) <= 1e-10 and abs(slope0 - slope1) <= 1e-10:
                L = prev.length + seg.length
                if abs(k1 - p0) <= 1e-11:
                    out[-1] = PlanSeg("line" if abs(p0) <= 1e-11 else "arc",
                                      L, p0, None)
                else:
                    out[-1] = PlanSeg("spiral", L, p0, k1)
                continue
        out.append(PlanSeg(seg.kind, float(seg.length), float(seg.curvature),
                           (None if seg.curvature_end is None
                            else float(seg.curvature_end))))
    return PlanView(pv.x0, pv.y0, pv.hdg, out)


def _connector_fidelity(pv: PlanView, src_pts: np.ndarray, *,
                        source_support_only: bool = False) -> dict:
    src = np.asarray(src_pts, float)
    total = sum(x.length for x in pv.segs)
    target = _planview_at_s(pv, np.linspace(0.0, total,
                                            max(41, int(total / 0.25) + 1)))
    # 源数据通常已是 0.5m 级密采样；仍统一重采样，避免不同图商点密度改变统计权重。
    source = _resample_count(src, max(41, int(arclength(src)[-1] / 0.25) + 1))
    pair_distance = np.linalg.norm(source[:, None, :] - target[None, :, :], axis=2)
    s2t = np.min(pair_distance, axis=1)
    support_s = None
    measured_target = target
    if source_support_only and len(target) >= 2:
        # IBD 路口内连接车道常只覆盖完整 connecting road 的中段。来源首末点
        # 在目标曲线上的最近投影定义可比较支持域；两端口部桥接仍由 G2/曲率/
        # 最短段门禁负责，不能把没有来源点的桥接区反向计成 t2s 失真。
        i0 = int(np.argmin(pair_distance[0]))
        i1 = int(np.argmin(pair_distance[-1]))
        lo, hi = sorted((i0, i1))
        if hi > lo:
            measured_target = target[lo:hi + 1]
            support_s = [total * lo / (len(target) - 1),
                         total * hi / (len(target) - 1)]
    t2s = np.min(np.linalg.norm(
        measured_target[:, None, :] - source[None, :, :], axis=2), axis=1)

    def stats(a):
        return {"median_m": float(np.median(a)),
                "p95_m": float(np.percentile(a, 95)),
                "max_m": float(np.max(a))}

    q = planview_quality(pv)
    return {
        "source_to_target": stats(s2t),
        "target_to_source": stats(t2s),
        "start_m": (float(s2t[0]) if source_support_only
                    else float(np.linalg.norm(source[0] - target[0]))),
        "end_m": (float(s2t[-1]) if source_support_only
                  else float(np.linalg.norm(source[-1] - target[-1]))),
        "source_support_s": support_s,
        "length_m": float(total),
        "n_primitives": int(len(pv.segs)),
        "min_primitive_m": float(q["seg_min_len"]),
        "sharpness_max": float(q["sharpness_max"]),
        "sharp_sign_flips": int(q["sharp_sign_flips"]),
        "kappa_max": float(q["kappa_max"]),
    }


def _connector_metrics_ok(m: dict, *, max_segments: int, max_kappa: float,
                          sharpness_cap: float, endpoint_tol: float,
                          median_tol: float, p95_tol: float,
                          max_dev_tol: float,
                          min_segment_m: float = 3.0) -> bool:
    total = m["length_m"]
    # 连接路也禁止用 0.x m 回旋子段制造“数值上 G2、消费端看起来抖动”的
    # 假平滑。相对门限约束长连接，3 m 绝对下限用于阻断端部“急调航向”碎片。
    hard_min = max(float(min_segment_m), 0.03 * total)
    return (
        m["n_primitives"] <= max_segments
        and m["min_primitive_m"] + 1e-9 >= hard_min
        and m["kappa_max"] <= max_kappa + 1e-9
        and m["sharpness_max"] <= sharpness_cap + 1e-12
        and m["start_m"] <= endpoint_tol
        and m["end_m"] <= endpoint_tol
        and all(m[d]["median_m"] <= median_tol
                and m[d]["p95_m"] <= p95_tol
                and m[d]["max_m"] <= max_dev_tol
                for d in ("source_to_target", "target_to_source"))
    )


def _g2_seed(p0, p1, k0: float, k1: float, n_segments: int):
    from pyclothoids import SolveG2

    cls = SolveG2(float(p0[0]), float(p0[1]), float(p0[2]), float(k0),
                  float(p1[0]), float(p1[1]), float(p1[2]), float(k1))
    total = float(sum(c.length for c in cls))
    s0 = np.concatenate([[0.0], np.cumsum([c.length for c in cls])])
    kk = np.asarray([c.KappaStart for c in cls] + [cls[-1].KappaEnd], float)
    lengths = np.full(n_segments, total / n_segments, dtype=float)
    knots = np.interp(np.linspace(0.0, total, n_segments + 1), s0, kk)
    knots[0], knots[-1] = k0, k1
    return cls, lengths, knots


def _optimize_connector(p0, p1, k0: float, k1: float, src_pts: np.ndarray,
                        n_segments: int, max_kappa: float,
                        sharpness_cap: float | None = None,
                        min_segment_m: float = 3.0) -> PlanView | None:
    """固定段数、固定两端曲率的联合拟合；SLSQP 等式约束保证终点 G2。"""
    from scipy.optimize import minimize

    try:
        _base, lengths, knots = _g2_seed(p0, p1, k0, k1, n_segments)
    except Exception:
        return None
    source = np.asarray(src_pts, float)
    target = _resample_count(np.vstack([np.asarray(p0[:2]), source,
                                        np.asarray(p1[:2])]), 41)
    frac = np.linspace(0.0, 1.0, target.shape[0])
    z0 = np.concatenate([lengths, knots[1:-1]])
    chord = float(np.linalg.norm(np.asarray(p1[:2]) - np.asarray(p0[:2])))
    max_length = max(8.0, 3.0 * chord)

    def unpack(z):
        return z[:n_segments], np.concatenate([[k0], z[n_segments:], [k1]])

    def objective(z):
        lens, ks = unpack(z)
        pv = _chain_from_knots(p0, lens, ks)
        pred = _planview_at_s(pv, frac * float(np.sum(lens)))
        err = np.linalg.norm(pred - target, axis=1)
        huber = np.where(err <= 1.0, 0.5 * err * err, err - 0.5)
        sharp = np.diff(ks) / np.maximum(lens, 1e-12)
        # 来源保真是首要连续目标；小正则只在同等保真时抑制高频曲率变化。
        return float(np.mean(huber) + 0.02 * np.mean(sharp * sharp)
                     * float(np.sum(lens)) ** 2)

    def endpoint_constraint(z):
        lens, ks = unpack(z)
        _p, end = planview_prims(_chain_from_knots(p0, lens, ks))
        return np.asarray([end[0] - p1[0], end[1] - p1[1],
                           _angle_delta(end[2], p1[2])])

    def sharpness_constraint(z):
        lens, ks = unpack(z)
        return float(sharpness_cap) - np.abs(np.diff(ks)) / np.maximum(lens, 1e-12)

    bounds = ([(float(min_segment_m), max_length)] * n_segments
              + [(-max_kappa, max_kappa)] * (n_segments - 1))
    constraints = [{"type": "eq", "fun": endpoint_constraint}]
    if sharpness_cap is not None:
        constraints.append({"type": "ineq", "fun": sharpness_constraint})
    sol = minimize(objective, z0, method="SLSQP", bounds=bounds,
                   constraints=constraints,
                   options={"maxiter": 450, "ftol": 1e-10, "disp": False})
    if not sol.success or float(np.linalg.norm(endpoint_constraint(sol.x))) > 1e-5:
        return None
    lens, ks = unpack(sol.x)
    return _canonicalize_connector(_chain_from_knots(p0, lens, ks))


def fit_connector_minimal(src_pts: np.ndarray, p0, p1, *, k0: float = 0.0,
                          k1: float = 0.0, max_segments: int = 5,
                          max_kappa: float = 0.20,
                          sharpness_cap: float = 0.20,
                          endpoint_tol: float = 1.5,
                          median_tol: float = 0.5,
                          p95_tol: float = 1.45,
                          max_dev_tol: float = 2.8,
                          source_covers_endpoints: bool = True,
                          min_segment_m: float = 3.0) -> MinimalConnectorFit | None:
    """实测点列→少段 G2 连接路。

    候选按段数 1→3→4→5 的字典序搜索；只有满足端点、来源偏差、曲率、
    ``dκ/ds`` 和相对短段全部硬约束的候选才进入排序。``dκ/ds`` 上限是几何
    病态熔断，不代替按实际转向限速计算的动力学门禁。找不到解返回 ``None``，
    调用侧必须显式记录失败，禁止退回逐点/桥接碎片链。
    """
    src = np.asarray(src_pts, float)
    if src.ndim != 2 or src.shape[0] < 2 or src.shape[1] != 2:
        return None
    p0, p1 = tuple(map(float, p0[:3])), tuple(map(float, p1[:3]))
    candidates: list[tuple] = []

    # 单回旋线只在天然满足两端曲率时成立；不为“少一段”牺牲 G2。
    try:
        from pyclothoids import Clothoid
        cl = Clothoid.G1Hermite(*p0, *p1)
        if (abs(cl.KappaStart - k0) <= 1e-6
                and abs(cl.KappaEnd - k1) <= 1e-6):
            one = _chain_from_knots(p0, np.asarray([cl.length]),
                                    np.asarray([k0, k1]))
            m = _connector_fidelity(
                one, src, source_support_only=not source_covers_endpoints)
            if _connector_metrics_ok(m, max_segments=max_segments,
                                     max_kappa=max_kappa,
                                     sharpness_cap=sharpness_cap,
                                     endpoint_tol=endpoint_tol,
                                     median_tol=median_tol, p95_tol=p95_tol,
                                     max_dev_tol=max_dev_tol,
                                     min_segment_m=min_segment_m):
                candidates.append((1, "single-clothoid", one, m))
    except Exception:
        pass

    if candidates:
        chosen = min(candidates, key=lambda x: (
            x[0], x[3]["sharp_sign_flips"], x[3]["sharpness_max"],
            x[3]["source_to_target"]["p95_m"],
            x[3]["target_to_source"]["p95_m"]))
        prims, _ = planview_prims(chosen[2])
        return MinimalConnectorFit(chosen[2], prims, chosen[1], chosen[3])

    for n in (3, 4, 5):
        if n > max_segments:
            continue
        # 同段数内同时比较权威 SolveG2 解与来源约束优化解。
        same_n = []
        try:
            cls, _lens, _knots = _g2_seed(p0, p1, k0, k1, n)
            if n == 3:
                base = _canonicalize_connector(PlanView(
                    p0[0], p0[1], p0[2],
                    [PlanSeg("spiral", c.length, c.KappaStart, c.KappaEnd)
                     for c in cls]))
                same_n.append(("solve-g2", base))
        except Exception:
            pass
        optimized = _optimize_connector(
            p0, p1, k0, k1, src, n, max_kappa,
            sharpness_cap=sharpness_cap,
            min_segment_m=min_segment_m)
        if optimized is not None:
            same_n.append((f"constrained-{n}-clothoid", optimized))
        feasible = []
        for method, pv in same_n:
            m = _connector_fidelity(
                pv, src, source_support_only=not source_covers_endpoints)
            if _connector_metrics_ok(m, max_segments=max_segments,
                                     max_kappa=max_kappa,
                                     sharpness_cap=sharpness_cap,
                                     endpoint_tol=endpoint_tol,
                                     median_tol=median_tol, p95_tol=p95_tol,
                                     max_dev_tol=max_dev_tol,
                                     min_segment_m=min_segment_m):
                feasible.append((method, pv, m))
        if feasible:
            # 同一最小段数内，先选最贴近来源的曲线。一次 ``dκ/ds`` 正负变化
            # 只是正常的“进入转弯—退出转弯”曲率单峰，不能排在来源保真前面；
            # 旧排序因此会把偏差超过 1m 的对称 SolveG2 误选在 3 段优化解之前。
            # 高频翻转和 sharpness 已分别受硬门限约束，只作为保真近似时的次序项。
            method, pv, metrics = min(feasible, key=lambda x: (
                x[2]["source_to_target"]["p95_m"],
                x[2]["target_to_source"]["p95_m"],
                x[2]["source_to_target"]["max_m"]
                + x[2]["target_to_source"]["max_m"],
                x[2]["sharp_sign_flips"], x[2]["sharpness_max"]))
            prims, _ = planview_prims(pv)
            return MinimalConnectorFit(pv, prims, method, metrics)
    return None


def planview_quality(pv: PlanView) -> dict:
    """直接审计中间 PlanView，供候选选择和写出后的 G7 使用同一语义。

    极小段不是孤立指标：真正危险的是极小段承载较大的曲率变化，形成高
    ``|dκ/ds|`` 和密集正负翻转。这里仍保留最短段硬指标，防止消费端收到
    0.x m 的几何碎片，即使该碎片碰巧没有越过旧的中位数门禁。
    """
    if not pv.segs:
        return {"length": 0.0, "n_segs": 0, "seg_min_len": 0.0,
                "seg_median_len": 0.0, "sharpness_max": float("inf"),
                "sharp_sign_flips": 0, "sharp_sign_flips_per_100m": float("inf"),
                "kappa_max": float("inf")}
    lens = [float(s.length) for s in pv.segs]
    sharp = [((float(s.curvature_end) - float(s.curvature)) / max(float(s.length), 1e-12))
             if s.kind == "spiral" else 0.0 for s in pv.segs]
    flips = sum(1 for a, b in zip(sharp, sharp[1:])
                if a * b < 0.0 and min(abs(a), abs(b)) > 1e-6)
    total = sum(lens)
    return {"length": total, "n_segs": len(lens),
            "seg_min_len": min(lens), "seg_median_len": float(np.median(lens)),
            "sharpness_max": max(map(abs, sharp), default=0.0),
            "sharp_sign_flips": flips,
            "sharp_sign_flips_per_100m": flips / max(total, 1e-9) * 100.0,
            "kappa_max": max_kappa(pv)}


def solve_g2_balanced(p0, p1, k0: float = 0.0, k1: float = 0.0, *,
                      length_growth_cap: float = 0.10,
                      kappa_growth_cap: float = 0.10,
                      dmax_candidates=(0.0, 0.10, 0.20, 0.30, 0.40,
                                       0.50, 0.70, 1.00)):
    """返回动力学更温和的 Bertolazzi--Frego 三回旋线 G2 解。

    ``SolveG2`` 的默认解满足端点位置、航向和曲率，但三段长度可能很不均衡，
    使很短的首/末段承担过大的 ``dκ/ds``。官方求解器的 ``dmax`` 参数用于
    限制解相对初始猜测的角度偏离；这里仅在仍满足精确 G2 边界条件的候选中，
    以最小最大 sharpness 为第一目标、较长的最短段为第二目标择优。

    总长和最大曲率只允许小幅增长；返回 ``(clothoids, diagnostics)``，便于
    provenance 明确记录调参及前后指标。
    """
    from pyclothoids import SolveG2

    p0 = tuple(map(float, p0[:3]))
    p1 = tuple(map(float, p1[:3]))
    k0, k1 = float(k0), float(k1)

    def as_planview(cls):
        return PlanView(p0[0], p0[1], p0[2], [
            PlanSeg("spiral", float(c.length), float(c.KappaStart),
                    float(c.KappaEnd)) for c in cls
        ])

    base = tuple(SolveG2(*p0, k0, *p1, k1))
    base_q = planview_quality(as_planview(base))
    length_cap = base_q["length"] * (1.0 + float(length_growth_cap))
    kappa_cap = max(base_q["kappa_max"] * (1.0 + float(kappa_growth_cap)),
                    base_q["kappa_max"] + 1e-12)
    candidates = []
    seen = set()
    for dmax in dmax_candidates:
        try:
            cls = tuple(SolveG2(*p0, k0, *p1, k1, 0.0, float(dmax)))
        except Exception:
            continue
        key = tuple(round(float(c.length), 10) for c in cls)
        if key in seen:
            continue
        seen.add(key)
        if any(not math.isfinite(float(c.length)) or float(c.length) <= 0.0
               for c in cls):
            continue
        q = planview_quality(as_planview(cls))
        if (q["length"] > length_cap + 1e-9
                or q["kappa_max"] > kappa_cap + 1e-9):
            continue
        candidates.append((q["sharpness_max"], -q["seg_min_len"],
                           q["kappa_max"], q["length"], float(dmax), cls, q))
    if not candidates:
        candidates.append((base_q["sharpness_max"], -base_q["seg_min_len"],
                           base_q["kappa_max"], base_q["length"], 0.0,
                           base, base_q))
    chosen = min(candidates, key=lambda item: item[:5])
    cls, q = chosen[5], chosen[6]
    return cls, {
        "method": "solve-g2-balanced",
        "dmax": chosen[4],
        "base": {
            "length_m": base_q["length"],
            "segment_lengths_m": [float(c.length) for c in base],
            "min_primitive_m": base_q["seg_min_len"],
            "kappa_max_per_m": base_q["kappa_max"],
            "sharpness_max_per_m2": base_q["sharpness_max"],
        },
        "selected": {
            "length_m": q["length"],
            "segment_lengths_m": [float(c.length) for c in cls],
            "min_primitive_m": q["seg_min_len"],
            "kappa_max_per_m": q["kappa_max"],
            "sharpness_max_per_m2": q["sharpness_max"],
        },
    }


def _robust_endpoint_heading(pts: np.ndarray, at_start: bool,
                             window_m: float = 30.0) -> float:
    """用端部长窗口 PCA 求航向，避免 0.1～3m 重复/短弦支配参考线。"""
    src = np.asarray(pts, float)
    ss = arclength(src)
    span = min(float(window_m), max(5.0, 0.35 * float(ss[-1])))
    q = src[ss <= span + 1e-9] if at_start else src[ss >= ss[-1] - span - 1e-9]
    if len(q) < 2:
        q = src[:2] if at_start else src[-2:]
    centered = q - q.mean(axis=0)
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    direction = vh[0]
    forward = q[-1] - q[0]
    if float(direction @ forward) < 0.0:
        direction = -direction
    return math.atan2(float(direction[1]), float(direction[0]))


def _single_arc_candidate(src: np.ndarray) -> PlanView | None:
    """过首/末/最大弦垂点的单圆弧候选；三点近共线时不臆造大圆。"""
    a, b = np.asarray(src[0], float), np.asarray(src[-1], float)
    chord = b - a
    chord_len = float(np.linalg.norm(chord))
    if chord_len <= 1e-6 or len(src) < 3:
        return None
    rel = src - a
    cross = chord[0] * rel[:, 1] - chord[1] * rel[:, 0]
    i = int(np.argmax(np.abs(cross)))
    cpt = np.asarray(src[i], float)
    if abs(float(cross[i])) / chord_len < 0.05:
        return None
    ax, ay = a
    bx, by = cpt
    cx, cy = b
    det = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(det) < 1e-9:
        return None
    aa, bb, cc = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    center = np.array([
        (aa * (by - cy) + bb * (cy - ay) + cc * (ay - by)) / det,
        (aa * (cx - bx) + bb * (ax - cx) + cc * (bx - ax)) / det,
    ])
    radius = float(np.linalg.norm(a - center))
    if not math.isfinite(radius) or radius < 1.0:
        return None
    angles = np.unwrap(np.arctan2(src[:, 1] - center[1],
                                  src[:, 0] - center[0]))
    delta = float(angles[-1] - angles[0])
    if abs(delta) < 1e-7 or abs(delta) > math.pi:
        return None
    sign = 1.0 if delta > 0.0 else -1.0
    hdg = float(angles[0] + sign * math.pi / 2.0)
    return PlanView(float(a[0]), float(a[1]), hdg,
                    [PlanSeg("arc", abs(delta) * radius, sign / radius)])


def _remove_impulse_outliers(src: np.ndarray) -> tuple[np.ndarray, dict]:
    """只删除折线中的孤立横向脉冲，不平滑真实缓弯。

    MAP Link 偶尔含有“前后总体航向不变、单个顶点却形成相反急转”的坏点。
    若逐点追随，会把坏点放大成 OpenDRIVE 蛇形。判据同时要求：

    * 顶点到相邻点弦线距离至少 1m；
    * 顶点转角至少 15°；
    * 顶点两侧的背景航向差不超过 4°；
    * 进入/离开脉冲相对背景都至少偏转 5°。

    因此渐变圆曲线、回旋线和真正的折角不会被本规则吞掉。每 8 个点最多
    删除 1 个，且首末两点及其直接邻点永不删除。
    """
    pts = np.asarray(src, float)
    if len(pts) < 7:
        return pts, {"removed_count": 0, "removed_indices": [],
                     "max_residual_m": 0.0}
    work = pts.copy()
    original_indices = list(range(len(pts)))
    removed: list[int] = []
    residuals: list[float] = []
    max_remove = max(1, len(pts) // 8)

    def hdg(a, b):
        d = b - a
        return math.atan2(float(d[1]), float(d[0]))

    for _ in range(max_remove):
        candidates = []
        for i in range(2, len(work) - 2):
            h_before = hdg(work[i - 2], work[i - 1])
            h_in = hdg(work[i - 1], work[i])
            h_out = hdg(work[i], work[i + 1])
            h_after = hdg(work[i + 1], work[i + 2])
            turn = abs(_angle_delta(h_out, h_in))
            background = abs(_angle_delta(h_after, h_before))
            shoulder_in = abs(_angle_delta(h_in, h_before))
            shoulder_out = abs(_angle_delta(h_after, h_out))
            chord = work[i + 1] - work[i - 1]
            den = float(chord @ chord)
            if den <= 1e-12:
                continue
            u = float(np.clip((work[i] - work[i - 1]) @ chord / den, 0.0, 1.0))
            residual = float(np.linalg.norm(
                work[i] - (work[i - 1] + u * chord)))
            if (residual >= 1.0 and turn >= math.radians(15.0)
                    and background <= math.radians(4.0)
                    and shoulder_in >= math.radians(5.0)
                    and shoulder_out >= math.radians(5.0)):
                candidates.append((residual, i))
        if not candidates:
            break
        residual, i = max(candidates)
        removed.append(original_indices[i])
        residuals.append(residual)
        work = np.delete(work, i, axis=0)
        del original_indices[i]
    return work, {
        "removed_count": len(removed),
        "removed_indices": removed,
        "max_residual_m": max(residuals, default=0.0),
    }


def _leg_candidate_ok(pv: PlanView, src: np.ndarray, *, tol: float,
                      kappa_cap: float, sharp_cap: float,
                      median_tol: float = 0.60,
                      p95_tol: float = 1.25) -> tuple[bool, dict]:
    metrics = _connector_fidelity(pv, src)
    ok = _connector_metrics_ok(
        metrics, max_segments=5, max_kappa=kappa_cap,
        sharpness_cap=sharp_cap, endpoint_tol=0.10,
        median_tol=min(median_tol, tol), p95_tol=min(p95_tol, tol),
        max_dev_tol=tol)
    return ok, metrics


def fit_leg_refline(pts: np.ndarray, kappa_cap: float = 0.009,
                    resample_step: float = 2.0, min_seg_len: float = 6.0,
                    dev_tol: float | None = None, sharp_cap: float = 0.000216,
                    min_output_seg_len: float = 3.0,
                    flips_per_100m_cap: float = 8.0,
                    hard_dev_cap: float = 1.5):
    """普通道路少段 G2 拟合。

    输入点列只是误差约束，不逐点插值。候选按字典序搜索：

    1. 单段 ``line`` / ``arc`` / ``spiral``；
    2. 固定 3、4、5 段的共享曲率节点 clothoid 链。

    所有候选先同时通过双向来源偏差、G2、最短相对段长、曲率与
    ``dκ/ds`` 硬约束，再选段数最少者。默认曲率与曲率变化率分别对应
    60km/h 下 2.5m/s² 横向加速度和 1.0m/s³ 横向加加速度上限。
    五段内无解必须显式失败，不恢复米级碎片链。

    返回 ``(PlanView, 双向最大来源偏差, approximated)``。
    ``resample_step/min_seg_len/min_output_seg_len`` 仅保留 API 兼容。
    """
    raw_src = np.asarray(pts, float)
    if raw_src.ndim != 2 or raw_src.shape[0] < 2 or raw_src.shape[1] != 2:
        raise ReflineFitError("参考线源点至少需要两个二维点")
    src, clean_meta = _remove_impulse_outliers(raw_src)

    def done(pv: PlanView, dev: float, approximated: bool, selection: str,
             metrics: dict | None = None):
        pv.fit_meta.update({"impulse_filter": clean_meta,
                            "fit_selection": selection})
        if metrics is not None:
            pv.fit_meta["source_fidelity"] = metrics
        return pv, dev, approximated or bool(clean_meta["removed_count"])
    tol = hard_dev_cap if dev_tol is None else float(dev_tol)
    if tol <= 0.0 or tol > hard_dev_cap + 1e-9:
        raise ValueError(f"dev_tol 必须在 (0, {hard_dev_cap}] m 内，收到 {tol}")

    # 端点完全一致的单直线只在 0.35m 内才认为“真直线”；
    # 否则 1.5m 的总容差会把真实缓弯过度压成直线。
    chord = src[-1] - src[0]
    chord_len = float(np.linalg.norm(chord))
    if chord_len <= 1e-6:
        raise ReflineFitError("参考线首末点重合")
    chord_h = math.atan2(float(chord[1]), float(chord[0]))
    line = PlanView(float(src[0, 0]), float(src[0, 1]), chord_h,
                    [PlanSeg("line", chord_len)])
    line_ok, line_m = _leg_candidate_ok(
        line, src, tol=min(0.35, tol), kappa_cap=kappa_cap,
        sharp_cap=sharp_cap, median_tol=0.20, p95_tol=0.30)
    if line_ok:
        dev = max(line_m["source_to_target"]["max_m"],
                  line_m["target_to_source"]["max_m"])
        return done(line, dev, False, "single-line", line_m)

    h0 = _robust_endpoint_heading(src, True)
    h1 = _robust_endpoint_heading(src, False)
    one_segment: list[tuple[str, PlanView, dict]] = []

    arc = _single_arc_candidate(src)
    if arc is not None:
        ok, m = _leg_candidate_ok(arc, src, tol=tol, kappa_cap=kappa_cap,
                                  sharp_cap=sharp_cap)
        if ok:
            one_segment.append(("single-arc", arc, m))

    try:
        from pyclothoids import Clothoid
        cl = Clothoid.G1Hermite(float(src[0, 0]), float(src[0, 1]), h0,
                                float(src[-1, 0]), float(src[-1, 1]), h1)
        spiral = PlanView(float(src[0, 0]), float(src[0, 1]), h0, [
            PlanSeg("spiral", float(cl.length), float(cl.KappaStart),
                    float(cl.KappaEnd))])
        ok, m = _leg_candidate_ok(spiral, src, tol=tol,
                                  kappa_cap=kappa_cap, sharp_cap=sharp_cap)
        if ok:
            one_segment.append(("single-spiral", spiral, m))
    except Exception:
        pass

    # 先追求“少而准”：最多仍只有 5 个长 G2 原语，但只要增加一两个长段能把
    # P95 从约 1m 降到 0.75m 内，就不应因为更粗候选勉强通过 1.5m 上限而提前
    # 返回。该搜索保留 3% 总长的相对最短段硬约束，不会退化为逐点碎片链。
    strict = fit_connector_minimal(
        src, (src[0, 0], src[0, 1], h0), (src[-1, 0], src[-1, 1], h1),
        k0=0.0, k1=0.0, max_segments=5, max_kappa=kappa_cap,
        sharpness_cap=sharp_cap, endpoint_tol=0.10,
        median_tol=min(0.35, tol), p95_tol=min(0.75, tol),
        max_dev_tol=min(1.0, tol))
    if strict is not None:
        m = strict.metrics
        dev = max(m["source_to_target"]["max_m"],
                  m["target_to_source"]["max_m"])
        return done(strict.planview, dev, True,
                    f"strict-{strict.method}", m)

    if one_segment:
        # 严格保真无解时才接受仍在硬上限内的一段式近似，并显式记为 fallback。
        _method, pv, m = min(one_segment, key=lambda item: (
            item[2]["sharp_sign_flips"], item[2]["sharpness_max"],
            item[2]["source_to_target"]["p95_m"],
            item[2]["target_to_source"]["p95_m"],
            item[2]["source_to_target"]["max_m"]
            + item[2]["target_to_source"]["max_m"]))
        dev = max(m["source_to_target"]["max_m"],
                  m["target_to_source"]["max_m"])
        return done(pv, dev, True, f"fallback-{_method}", m)

    # 单段无解才允许 3→4→5 段。两端零曲率使道路口部与连接路
    # 天然 G2；内部段共享曲率节点，不存在隐性 Δκ。
    got = fit_connector_minimal(
        src, (src[0, 0], src[0, 1], h0), (src[-1, 0], src[-1, 1], h1),
        k0=0.0, k1=0.0, max_segments=5, max_kappa=kappa_cap,
        sharpness_cap=sharp_cap, endpoint_tol=0.10,
        median_tol=min(0.60, tol), p95_tol=min(1.25, tol),
        max_dev_tol=tol)
    if got is None:
        raise ReflineFitError(
            "参考线拟合无合格候选：1–5 段内无法同时满足 "
            f"dev≤{tol:.3f}m, |κ|≤{kappa_cap:.6f}, "
            f"|dκ/ds|≤{sharp_cap:.6f}")
    q = planview_quality(got.planview)
    if q["sharp_sign_flips_per_100m"] > flips_per_100m_cap + 1e-9:
        raise ReflineFitError(
            f"参考线候选曲率变化反复：{q['sharp_sign_flips_per_100m']:.2f}/"
            f"{flips_per_100m_cap:.2f} per 100m")
    m = got.metrics
    dev = max(m["source_to_target"]["max_m"],
              m["target_to_source"]["max_m"])
    return done(got.planview, dev, True, f"fallback-{got.method}", m)


def _prim_pose_at(prim, ell):
    """几何原语内部 arc length=ell 处的位姿+曲率 (x, y, h, k)。"""
    kind, x, y, h, L, k0, k1 = prim
    ell = min(max(ell, 0.0), L)
    if kind == "line":
        return x + ell * math.cos(h), y + ell * math.sin(h), h, 0.0
    if kind == "arc":
        return (x + (math.sin(h + k0 * ell) - math.sin(h)) / k0,
                y - (math.cos(h + k0 * ell) - math.cos(h)) / k0,
                h + k0 * ell, k0)
    from pyclothoids import Clothoid
    dk = (k1 - k0) / max(L, 1e-12)
    cl = Clothoid.StandardParams(x, y, h, k0, dk, L)
    return cl.X(ell), cl.Y(ell), h + k0 * ell + 0.5 * dk * ell * ell, k0 + dk * ell


def g2ify_planview(pv: PlanView, max_win: float = 12.0) -> tuple[PlanView, int]:
    """参考线全程 G2 化：结点两侧开窗切除，SolveG2（Bertolazzi–Frego 三段回旋线）
    精确重连——两端位置/航向/曲率全匹配，下游几何零漂移；Δκ 结点的侧向加速度
    阶跃（v²Δκ）由此消除。短段（<1.25m，样条链常见）整段吞入过渡窗，右侧贪心
    跨段扩展；双趟迭代收残余。返回 (新 PlanView, 修复结点数)。"""
    from pyclothoids import SolveG2
    fixed_total = 0
    for _pass in range(2):
        prims, _ = planview_prims(pv)
        work = [list(p) for p in prims]
        out: list[PlanSeg] = []
        fixed = 0
        i = 0
        while i < len(work):
            cur = work[i]
            kind, x, y, h, L, k0, k1 = cur
            if i == len(work) - 1 or abs(k1 - work[i + 1][5]) <= 1e-9:
                if L > 1e-6:
                    out.append(PlanSeg(kind, L, k0, k1 if kind == "spiral" else None))
                i += 1
                continue
            # —— 左窗：短段整段切除 ——
            wl = min(max_win, 0.4 * L)
            if wl < 0.5:
                wl = L
            cutL = _prim_pose_at(tuple(cur), L - wl)
            # —— 右窗：贪心吞并连续短段 ——
            j = i + 1
            while j < len(work) - 1 and 0.4 * work[j][4] < 0.5:
                j += 1
            wr = min(max_win, 0.4 * work[j][4])
            if wr < 0.5:
                wr = work[j][4]
            cutR = _prim_pose_at(tuple(work[j]), wr)
            span = wl + wr + sum(w[4] for w in work[i + 1:j])
            try:
                cls = SolveG2(cutL[0], cutL[1], cutL[2], cutL[3],
                              cutR[0], cutR[1], cutR[2], cutR[3])
            except Exception:
                cls = None
            if cls is None or sum(c.length for c in cls) > 4.0 * span + 2.0:
                if L > 1e-6:                             # 解失控：保持 G1
                    out.append(PlanSeg(kind, L, k0, k1 if kind == "spiral" else None))
                i += 1
                continue
            if L - wl > 1e-6:
                out.append(PlanSeg(kind, L - wl, k0,
                                   cutL[3] if kind == "spiral" else None))
            for c in cls:
                if c.length > 1e-6:
                    out.append(PlanSeg("spiral", c.length, c.KappaStart, c.KappaEnd))
            nxt = work[j]
            nx, ny, nh, nk = cutR
            nxt[1], nxt[2], nxt[3] = nx, ny, nh
            if nxt[0] == "spiral":
                nxt[5] = nk
            nxt[4] = nxt[4] - wr
            fixed += 1
            i = j if nxt[4] > 1e-6 else j + 1
            if nxt[4] <= 1e-6 and j == len(work) - 1:
                break
        pv = PlanView(pv.x0, pv.y0, pv.hdg, out)
        fixed_total += fixed
        if fixed == 0:
            break
    return pv, fixed_total


def fc_clamp(y0, m0, y1, m1, L):
    """Hermite 端点斜率的 Fritsch–Carlson 单调限幅（消段内过冲小钩子）。
    两管道横断面 Hermite 共用。"""
    sec = (y1 - y0) / max(L, 1e-6)
    if abs(sec) < 1e-9:
        return 0.0, 0.0
    a = min(max(m0 / sec, 0.0), 3.0)
    b = min(max(m1 / sec, 0.0), 3.0)
    return a * sec, b * sec


# ---------------------------------------------------------------- 曲率域段精简

def sample_curvature(pv: PlanView, ds: float = 0.5):
    """PlanView → 沿弧长的 (s, kappa) 采样序列（段内线性，段界共点）。"""
    ss, kk, s0 = [], [], 0.0
    for seg in pv.segs:
        k0 = seg.curvature
        k1 = seg.curvature_end if seg.kind == "spiral" else seg.curvature
        if seg.kind == "line":
            k0 = k1 = 0.0
        n = max(2, int(seg.length / ds) + 1)
        u = np.linspace(0.0, seg.length, n)
        ss.append(s0 + u)
        kk.append(k0 + (k1 - k0) * u / max(seg.length, 1e-9))
        s0 += seg.length
    return np.concatenate(ss), np.concatenate(kk)


def _rdp_curvature(s: np.ndarray, k: np.ndarray, tol: float) -> np.ndarray:
    """(s,κ) 折线的 Douglas–Peucker 简化，返回保留点索引（升序）。
    简化后的折线顶点即"曲率控制点"：相邻两点间 κ 线性 = 一段回旋线。"""
    keep = np.zeros(s.size, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, s.size - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        t = (s[a + 1:b] - s[a]) / max(s[b] - s[a], 1e-9)
        dev = np.abs(k[a + 1:b] - (k[a] + (k[b] - k[a]) * t))
        m = int(np.argmax(dev))
        if dev[m] > tol:
            idx = a + 1 + m
            keep[idx] = True
            stack += [(a, idx), (idx, b)]
    return np.flatnonzero(keep)


def _dev_to_src(pv: PlanView, src: np.ndarray) -> float:
    ref = eval_planview(pv, 0.5)
    return float(max(np.min(np.linalg.norm(ref - p[None, :], axis=1)) for p in src))


def _absorb_short_segs(pv: PlanView, src: np.ndarray, min_len: float,
                       dev_tol: float) -> PlanView:
    """短段吸收：把 <min_len 的段并入邻段（合并成一条 κ 线性的 clothoid），
    **每次合并都用对原始顶点的偏差兜底**，超差即放弃该次合并。

    注意不能在曲率图上直接删控制点——那等于改变 ∫κ ds，航向随即漂移
    （实测偏差可爆到 30m 量级）；合并必须在段链上做且逐次验偏差。"""
    changed = True
    while changed and len(pv.segs) > 1:
        changed = False
        for i in sorted(range(len(pv.segs)), key=lambda t: pv.segs[t].length):
            if pv.segs[i].length >= min_len:
                break
            for j in (i - 1, i + 1):
                if not 0 <= j < len(pv.segs):
                    continue
                a, b = min(i, j), max(i, j)
                sa, sb = pv.segs[a], pv.segs[b]
                L = sa.length + sb.length
                k0 = 0.0 if sa.kind == "line" else sa.curvature
                k1 = 0.0 if sb.kind == "line" else (
                    sb.curvature_end if sb.kind == "spiral" else sb.curvature)
                if abs(k1 - k0) < 1e-9:
                    merged = PlanSeg("line" if abs(k0) < 1e-9 else "arc", L, k0, None)
                else:
                    merged = PlanSeg("spiral", L, k0, k1)
                cand = PlanView(pv.x0, pv.y0, pv.hdg,
                                pv.segs[:a] + [merged] + pv.segs[b + 1:])
                if _dev_to_src(cand, src) <= dev_tol:
                    pv = cand
                    changed = True
                    break
            if changed:
                break
    return pv


def _rebuild_from_knots(pv: PlanView, s: np.ndarray, k: np.ndarray,
                        idx: np.ndarray) -> PlanView:
    """曲率控制点 → PlanView（相邻点间一段：κ 恒零=line、κ 恒定=arc、否则 spiral）。
    相邻段首尾曲率天然相接 ⇒ 全程 G2，无需再做结点过渡。"""
    segs = []
    for a, b in zip(idx[:-1], idx[1:]):
        L = float(s[b] - s[a])
        if L <= 1e-6:
            continue
        k0, k1 = float(k[a]), float(k[b])
        if abs(k1 - k0) < 1e-9:
            segs.append(PlanSeg("line" if abs(k0) < 1e-9 else "arc", L, k0, None))
        else:
            segs.append(PlanSeg("spiral", L, k0, k1))
    return PlanView(pv.x0, pv.y0, pv.hdg, segs)


def _smooth_kappa(k: np.ndarray, win: int) -> np.ndarray:
    """κ 序列保端点平滑（中值去脉冲 + 均值去抖）。源折线是数字化产物，其曲率
    含大量噪声；逐顶点保留噪声 = 曲率蛇行（方向盘微抖），必须在容差内去噪。"""
    if win <= 1 or k.size < 2 * win + 3:
        return k
    out = k.copy()
    half = win // 2
    med = np.array([np.median(k[max(0, i - half):i + half + 1]) for i in range(k.size)])
    ker = np.ones(win) / win
    sm = np.convolve(np.pad(med, half, mode="edge"), ker, mode="valid")[:k.size]
    out[half:-half] = sm[half:-half]                     # 端点曲率保持（接缝锚点）
    return out


def simplify_planview(pv: PlanView, src_pts: np.ndarray, dev_tol: float = 0.35,
                      min_seg_len: float = 8.0, *,
                      hard_min_seg_len: float = 0.0,
                      sharp_cap: float | None = None,
                      flips_per_100m_cap: float | None = None,
                      kappa_cap: float | None = None):
    """**曲率域段精简 + 去噪**：κ 序列平滑后在 (s,κ) 上 Douglas–Peucker 简化，
    把逐顶点碎段还原成有物理长度的 line/arc/clothoid（直路 1 段、标准弯 3 段）。

    两个动机（都指向自动驾驶消费端）：
    ① 段数：每 1–2m 一段是把数字化噪声当几何，消费端读到高频曲率锯齿；
    ② 蛇行：sharpness(dκ/ds) 反复变号 ⇒ 方向盘来回微抖。故优化目标是
       **先最少变号、再最少段数**，而不是单纯"段够长"。

    在 (平滑窗 × κ容差) 网格里搜索偏差 ≤ dev_tol 的最优解。调用方可再给出
    最短段、|dκ/ds|、翻转密度和 |κ| 硬门禁；任一门禁没有可行解即返回 None，
    不返回“最接近但仍不合格”的 fallback。
    返回 (PlanView, dev, n_segs) 或 None。"""
    src = np.asarray(src_pts, float)
    # 前置：κ 必须**连续**才能在曲率域做折线简化——G1 链（arc↔arc 阶跃）直接简化
    # 会把阶跃拉成长斜坡，几何面目全非（实测偏差 20m 量级）。先 G2 化再精简。
    prims, _ep = planview_prims(pv)
    if any(abs(prims[i][6] - prims[i + 1][5]) > 1e-9 for i in range(len(prims) - 1)):
        pv, _n = g2ify_planview(pv)
    s, k0 = sample_curvature(pv, 0.5)
    if s.size < 4:
        return None
    # 焊平残留 κ 阶跃：g2ify 的"解失控保持 G1"分支会留下零长度跳变，重建时
    # L=0 的段被跳过 ⇒ 断差原样穿透到输出。把阶跃两侧取平均，让**简化的输出
    # 无条件 G2**（几何代价由 dev_tol 兜底），这是本管道对消费端的硬承诺。
    for i in range(s.size - 1):
        if s[i + 1] - s[i] < 1e-9 and abs(k0[i + 1] - k0[i]) > 1e-9:
            k0[i] = k0[i + 1] = 0.5 * (k0[i] + k0[i + 1])
    L_tot = float(s[-1])
    min_len = min(min_seg_len, L_tot / 3)
    hard_min = min(max(float(hard_min_seg_len), 0.0), L_tot / 3)
    best = None                                          # (score, pv, dev)
    strict_mode = (hard_min > 0.0 or sharp_cap is not None
                   or flips_per_100m_cap is not None or kappa_cap is not None)
    # 旧实现遇到首个“偏差合格”候选就提前停止，会选中 0.6–1m 的尖锐碎段；
    # 现在只有候选通过全部硬门禁才允许停止，否则继续搜索后续窗/κ容差。
    wins = [w for w in (1, 5, 9, 15, 25, 41, 61) if w <= max(3, k0.size // 4)] or [1]
    for win in wins:
        k = _smooth_kappa(k0, win)
        for tol_k in (8e-3, 4e-3, 2e-3, 1e-3, 5e-4, 2e-4, 1e-4, 5e-5, 2e-5):
            idx = _rdp_curvature(s, k, tol_k)
            if idx.size < 2:
                continue
            cand = _rebuild_from_knots(pv, s, k, idx)
            if not cand.segs:
                continue
            dev = _dev_to_src(cand, src)
            if dev > dev_tol:
                continue
            cand = _absorb_short_segs(cand, src, min_len, dev_tol)
            cand = weld_g2(cand)
            dev = _dev_to_src(cand, src)                 # weld 后必须重新验来源偏差
            if dev > dev_tol + 1e-9:
                continue
            q = planview_quality(cand)
            if hard_min and q["seg_min_len"] < hard_min - 1e-9:
                continue
            if sharp_cap is not None and q["sharpness_max"] > sharp_cap + 1e-12:
                continue
            if (flips_per_100m_cap is not None
                    and q["sharp_sign_flips_per_100m"] > flips_per_100m_cap + 1e-9):
                continue
            if kappa_cap is not None and q["kappa_max"] > kappa_cap + 1e-12:
                continue
            score = (q["sharp_sign_flips_per_100m"], q["n_segs"],
                     q["sharpness_max"], dev, -q["seg_min_len"])
            # 严格模式按“小窗→大窗、粗 κ→细 κ”搜索；第一个通过全部硬门禁的
            # 候选已足够安全，同时保留最少平滑。继续遍历只会显著拖慢批量生成。
            if strict_mode:
                return cand, dev, len(cand.segs)
            if best is None or score < best[0]:
                best = (score, cand, dev)
            break                                        # 普通模式保持旧的粗容差优先策略
        if (not strict_mode and best is not None
                and best[0][0] <= max(1.0, L_tot / 100.0 * 6.0)):
            break
    if best is None:
        return None
    return best[1], best[2], len(best[1].segs)


def weld_g2(pv: PlanView) -> PlanView:
    """兜底焊平：把相邻段残留的 κ 断差取平均消除（arc 段升为 spiral）。
    g2ify 在病态位姿下会保留 G1 结点；本函数保证输出**无条件 G2**，
    代价是端点曲率微调（断差本身很小，几何影响远小于拟合容差）。"""
    def k_start(sg):
        return 0.0 if sg.kind == "line" else sg.curvature

    def k_end(sg):
        if sg.kind == "line":
            return 0.0
        return sg.curvature_end if sg.kind == "spiral" else sg.curvature

    segs = list(pv.segs)
    for i in range(len(segs) - 1):
        ka, kb = k_end(segs[i]), k_start(segs[i + 1])
        if abs(ka - kb) <= 1e-12:
            continue
        m = 0.5 * (ka + kb)
        segs[i] = PlanSeg("spiral", segs[i].length, k_start(segs[i]), m)
        segs[i + 1] = PlanSeg("spiral", segs[i + 1].length, m, k_end(segs[i + 1]))
    out = []
    for sg in segs:                                      # 退化 spiral 归位 arc/line
        if sg.kind == "spiral" and abs((sg.curvature_end or 0.0) - sg.curvature) < 1e-12:
            k = sg.curvature
            out.append(PlanSeg("line" if abs(k) < 1e-12 else "arc", sg.length, k, None))
        else:
            out.append(sg)
    return PlanView(pv.x0, pv.y0, pv.hdg, out)
