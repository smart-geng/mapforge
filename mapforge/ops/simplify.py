# -*- coding: utf-8 -*-
"""点列抽稀：T/CSAE 159-2020 附录 D 判据（弦距容差 + 首末点保留）。

判据映射：任意相邻两点弦线与实际中心线垂距 < 容差 → Douglas–Peucker 距离阈值；
首末点强制保留（Link 首点=上游进入第一点、末点=停止线中心，由调用方保证点列端点语义）。
"""
from __future__ import annotations

import numpy as np


def _dp(pts: np.ndarray, i0: int, i1: int, tol: float, keep: np.ndarray):
    if i1 <= i0 + 1:
        return
    a, b = pts[i0], pts[i1]
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-12:
        d = np.linalg.norm(pts[i0 + 1:i1] - a, axis=1)
    else:
        t = np.clip((pts[i0 + 1:i1] - a) @ ab / L2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d = np.linalg.norm(pts[i0 + 1:i1] - proj, axis=1)
    imax = int(np.argmax(d))
    if d[imax] > tol:
        j = i0 + 1 + imax
        keep[j] = True
        _dp(pts, i0, j, tol, keep)
        _dp(pts, j, i1, tol, keep)


def simplify_appendix_d(pts: np.ndarray, tol: float = 0.30) -> np.ndarray:
    """Douglas–Peucker 抽稀，返回保留点索引（含首末）。tol 单位米（附录 D 容差可配）。"""
    n = pts.shape[0]
    if n <= 2:
        return np.arange(n)
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    _dp(pts, 0, n - 1, tol, keep)
    return np.nonzero(keep)[0]
