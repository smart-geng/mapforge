# -*- coding: utf-8 -*-
"""xodr 俯视填充渲染：按查看器的消费方式（planView + laneOffset + 宽度堆叠）把每条车道
铺成填充多边形，输出 PNG——与 odrviewer 顶视等价，用于诊断"镂空/不平滑"到底来自哪里。

用法：.venv/Scripts/python scripts/xodr_topdown.py out/direct_xodr/node4.xodr [out.png]
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.validate.smoothness import sample_road_ref, cross_edges_at  # noqa: E402


def road_patches(rd, ds=0.5):
    """返回 [(lane_poly(n,2), is_junction)]：逐 laneSection 按全断面边界扫掠成多边形
    （左侧+右侧车道都画）。采样必含 section 边界点（落在网格间会渲染出假缝）。"""
    pts, ss, hh = sample_road_ref(rd, ds)
    sec_s = [float(sec.get("s")) for sec in rd.findall("lanes/laneSection")]
    sec_s.append(float(ss[-1]))
    is_junc = rd.get("junction") not in (None, "-1")
    out = []
    for s0, s1 in zip(sec_s[:-1], sec_s[1:]):
        if s1 - s0 < 1e-6:
            continue
        u = np.linspace(s0, s1, max(2, int((s1 - s0) / ds) + 2))
        u[-1] = s1 - 1e-4                                # 边界点取本 section 一侧
        px = np.interp(u, ss, pts[:, 0])
        py = np.interp(u, ss, pts[:, 1])
        ph = np.interp(u, ss, hh)
        p = np.column_stack([px, py])
        nrm = np.column_stack([-np.sin(ph), np.cos(ph)])
        edges = np.array([cross_edges_at(rd, s) for s in u])      # (n, nedge)
        for k in range(edges.shape[1] - 1):
            left = p + edges[:, k:k + 1] * nrm
            right = p + edges[:, k + 1:k + 2] * nrm
            out.append((np.vstack([left, right[::-1]]), is_junc))
    return out


def render(path, out_png, crop=None, annotate=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    root = ET.parse(str(path)).getroot()
    fig, ax = plt.subplots(figsize=(16, 12), dpi=110)
    ax.set_facecolor("#87b7e8")                          # 天蓝底：镂空处会露出来
    normal, junc, labels = [], [], []
    for rd in root.findall("road"):
        patches = road_patches(rd)
        for poly, is_j in patches:
            (junc if is_j else normal).append(poly)
        if annotate and patches and rd.get("junction") in (None, "-1"):
            mid = patches[0][0][len(patches[0][0]) // 4]
            labels.append((mid, rd.get("id")))
    for poly in normal:
        ax.add_patch(Polygon(poly, closed=True, facecolor="#4a4a44",
                             edgecolor="#8a8a80" if annotate else "none", lw=0.3))
    for poly in junc:
        ax.add_patch(Polygon(poly, closed=True, facecolor="#4a4a44", edgecolor="none"))
    for (x, y), rid in labels:
        ax.annotate(rid, (x, y), color="#ffdd44", fontsize=11, weight="bold")
    ax.autoscale()
    ax.set_aspect("equal")
    if crop:
        ax.set_xlim(crop[0], crop[1])
        ax.set_ylim(crop[2], crop[3])
    ax.set_title(Path(path).name)
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)
    print(out_png)


if __name__ == "__main__":
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else str(Path(src).with_suffix(".png"))
    crop = tuple(float(x) for x in sys.argv[3].split(",")) if len(sys.argv) > 3 else None
    render(src, dst, crop=crop, annotate=crop is not None)
