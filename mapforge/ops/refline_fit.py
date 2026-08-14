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


def fit_leg_refline(pts: np.ndarray, kappa_cap: float = 1.0 / 30.0,
                    resample_step: float = 2.0, min_seg_len: float = 6.0):
    """leg 参考线拟合 + 曲率封顶 + 全程 G2 化。
    封顶：|κ|>cap 的中段折角是数字化噪声（城市路段中途不存在 R<30m 的物理弯），
    渐进滑动平均后重拟合直到达标——双侧车道模型里 1−tκ→0 会让车道中心驻点/回卷；
    G2 化：line-arc 结点 SolveG2 过渡（消 v²Δκ 侧向加速度阶跃）。
    返回 (PlanView, 对原始顶点偏差, smoothed: bool)。"""
    pv, dev = fit_polyline_auto(pts, resample_step, min_seg_len)
    smoothed = False
    if max_kappa(pv) > kappa_cap:
        base = resample_polyline(np.asarray(pts, float), 2.0)
        for w in (3, 5, 9, 15):
            sm = _movavg_keep_ends(base, w)
            pv, _d = fit_polyline_auto(sm, resample_step, min_seg_len)
            if max_kappa(pv) <= kappa_cap:
                break
        smoothed = True
    src = np.asarray(pts, float)
    base_dev = _dev_to_src(pv, src) if pv.segs else 0.0
    # 曲率域段精简：把逐顶点碎段还原成有物理长度的 line/arc/clothoid（消费端要求）
    got = simplify_planview(pv, src, dev_tol=min(0.9, max(0.50, base_dev * 1.5)),
                            min_seg_len=12.0)
    # 统一 G2：精简输出只剩微小残差 → 焊平（不增段，护住段长中位）；
    # 未精简则是 line-arc 大阶跃 → 必须插入过渡段，再焊平兜底病态结点
    pv = weld_g2(got[0]) if got is not None else weld_g2(g2ify_planview(pv)[0])
    # 偏差始终对原始顶点报告（平滑/G2 过渡/精简是修复，不是新的真值）
    dev = _dev_to_src(pv, src)
    return pv, dev, smoothed


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
                      min_seg_len: float = 8.0):
    """**曲率域段精简 + 去噪**：κ 序列平滑后在 (s,κ) 上 Douglas–Peucker 简化，
    把逐顶点碎段还原成有物理长度的 line/arc/clothoid（直路 1 段、标准弯 3 段）。

    两个动机（都指向自动驾驶消费端）：
    ① 段数：每 1–2m 一段是把数字化噪声当几何，消费端读到高频曲率锯齿；
    ② 蛇行：sharpness(dκ/ds) 反复变号 ⇒ 方向盘来回微抖。故优化目标是
       **先最少变号、再最少段数**，而不是单纯"段够长"。

    在 (平滑窗 × κ容差) 网格里搜索偏差 ≤ dev_tol 的最优解；全部超差返回 None。
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
    best = None                                          # (score, pv, dev)
    # 平滑窗**从小到大**：够用即止，保留最多真实几何（大窗优先会把真弯也抹平，
    # 横向偏差无谓变大）。窗口上限随采样点数自适应，短段（连接路中段）才吃得到去噪。
    wins = [w for w in (1, 5, 9, 15, 25, 41, 61) if w <= max(3, k0.size // 4)] or [1]
    for win in wins:
        k = _smooth_kappa(k0, win)
        for tol_k in (4e-3, 2e-3, 1e-3, 5e-4, 2e-4, 1e-4, 5e-5):
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
            sharp = [0.0 if sg.length <= 1e-9 else
                     ((sg.curvature_end if sg.kind == "spiral" else sg.curvature)
                      - sg.curvature) / sg.length for sg in cand.segs]
            flips = sum(1 for a, b in zip(sharp, sharp[1:])
                        if a * b < 0 and min(abs(a), abs(b)) > 1e-6)
            score = (flips, len(cand.segs))
            if best is None or score < best[0]:
                best = (score, cand, _dev_to_src(cand, src))
            break                                        # 同一窗下取最粗 κ 容差（段最少）
        # 蛇行已达标（每 100m 变号 ≤6，相当于每弯 2–3 次）：停止加大平滑
        if best is not None and best[0][0] <= max(1, int(L_tot / 100.0 * 6)):
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
