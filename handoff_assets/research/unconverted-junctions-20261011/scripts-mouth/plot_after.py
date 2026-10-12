"""Plot a converted XODR's reference lines (roads, connectors) over the source lanes of one junction.

usage (cwd = experiment tree): python plot_after.py <topo.json> <xodr> <out.png> <title> <cx> <cy> <half_m>
"""
from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

sys.path.insert(0, str(Path.cwd()))
from mapforge.validate.smoothness import sample_road_ref  # noqa: E402

for f in font_manager.findSystemFonts():
    if "wqy-zenhei" in f:
        font_manager.fontManager.addfont(f)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name()
plt.rcParams["axes.unicode_minus"] = False


def main(topo_json, xodr, out_png, title, cx, cy, half):
    topo = json.load(open(topo_json, encoding="utf-8"))
    poly = np.asarray(topo["polygon_xy"])
    root = ET.parse(xodr).getroot()
    fig, ax = plt.subplots(figsize=(11, 8.5), dpi=110)
    ax.fill(poly[:, 0], poly[:, 1], color="#d9d8d3", alpha=0.75, zorder=0, label="源路口面")
    first = True
    for info in topo["links"].values():
        for l in info["lanes"]:
            xy = np.asarray(l["xy"])
            if len(xy) >= 2:
                ax.plot(xy[:, 0], xy[:, 1], color="#9a9a96", lw=1.0, zorder=1, label="源车道中心线" if first else None)
                first = False
    seen = set()
    for rd in root.findall("road"):
        if rd.get("name") == "junction_paving":
            continue
        pts, _ss, _hh = sample_road_ref(rd, 0.5)
        conn = rd.get("junction") not in (None, "-1")
        col, lab = ("#eb6834", "连接路参考线") if conn else ("#2a78d6", "道路参考线")
        ax.plot(pts[:, 0], pts[:, 1], color=col, lw=2.0 if not conn else 1.6, zorder=3,
                label=None if lab in seen else lab)
        seen.add(lab)
        if not conn:
            ax.plot(*pts[-1], marker="o", ms=6, color=col, zorder=4)
            ax.plot(*pts[0], marker="s", ms=5, color=col, zorder=4)
            mid = pts[np.argmin(np.linalg.norm(pts - np.array([cx, cy]), axis=1))]
            ax.text(mid[0], mid[1] + 1.2, f"road {rd.get('id')}", fontsize=9, color="#0b0b0b", zorder=6,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half * 0.75, cy + half * 0.75)
    ax.set_aspect("equal")
    ax.grid(color="#e6e5e0", lw=0.6)
    ax.set_xlabel("x [m]（路口中心局部平面）", color="#52514e")
    ax.set_ylabel("y [m]", color="#52514e")
    ax.legend(loc="lower left", fontsize=9, framealpha=0.9)
    ax.set_title(title, fontsize=11.5)
    fig.tight_layout()
    fig.savefig(out_png)
    print("wrote", out_png)


if __name__ == "__main__":
    a = sys.argv
    main(a[1], a[2], a[3], a[4], float(a[5]), float(a[6]), float(a[7]))
