"""Plot one failing junction from dump_topo.py / diag_leg.py JSON (two panels: overview + zoom on the failing leg).

usage: python plot_case.py <topo.json> <leg.json> <out.png> <title> [zoom_halfwidth_m] [overview_halfwidth_m]
"""
from __future__ import annotations

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for f in font_manager.findSystemFonts():
    if "wqy-zenhei" in f:
        font_manager.fontManager.addfont(f)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name()
plt.rcParams["axes.unicode_minus"] = False

C = {"seed": "#2a78d6", "upstream_enter": "#eb6834", "enter": "#1baf7a", "leave": "#eda100",
     "inside": "#e87ba4", "other": "#9a9a96", "mouth": "#e34948", "axis": "#0b0b0b", "keep": "#6250d6",
     "chain": "#86b6ef", "paired_leave": "#6250d6", "unlisted": "#008300"}
LABELS = {"seed": "失败种子进口 link", "upstream_enter": "同向上游（亦为进口道路）", "enter": "其他进口道路",
          "leave": "出口道路", "inside": "路口内 via link", "other": "邻接 link", "chain": "种子上游链（未列入进口）",
          "paired_leave": "与种子配对的对向出口", "unlisted": "未列入 ENTER_ROAD 的主车道进口"}


def stations(g):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))]


def at_station(g, s):
    ss = stations(g)
    if s <= ss[-1]:
        return np.array([np.interp(s, ss, g[:, 0]), np.interp(s, ss, g[:, 1])])
    dv = g[-1] - g[-2]
    dv /= np.linalg.norm(dv)
    return g[-1] + (s - ss[-1]) * dv


def main(topo_json, leg_json, out_png, title, zoom=30.0, over=90.0, overrides=None):
    topo = json.load(open(topo_json, encoding="utf-8"))
    leg = json.load(open(leg_json, encoding="utf-8"))
    fl = leg["failing_leg"]
    seed = fl["enter_link"]
    chain_links = [r["link"] for r in fl["chain_upstream_to_junction"]]
    upstream_enters = set(leg.get("upstream_enter_roads", []))
    poly = np.asarray(topo["polygon_xy"])
    axis = np.asarray(leg.get("axis_xy") or [], float)

    overrides = overrides or {}
    paired = set(fl.get("leave_links", []))

    def role(pid, info):
        if pid in overrides:
            return overrides[pid]
        if pid == seed:
            return "seed"
        if pid in upstream_enters or (info["role"] == "enter" and pid in chain_links):
            return "upstream_enter"
        if pid in paired:
            return "paired_leave"
        if pid in chain_links:
            return "chain"
        if info["role"] in ("enter", "leave"):
            return info["role"]
        lanes = info["lanes"]
        if lanes and all(l["start_in_surface"] and l["end_in_surface"] for l in lanes):
            return "inside"
        return "other"

    fig, axes = plt.subplots(1, 2, figsize=(16, 8.2), dpi=110)
    centre = poly.mean(axis=0)
    mp = fl.get("mouth_point_xy")
    for k, ax in enumerate(axes):
        ax.fill(poly[:, 0], poly[:, 1], color="#d9d8d3", alpha=0.75, zorder=0, label="源路口面")
        ax.plot(np.r_[poly[:, 0], poly[0, 0]], np.r_[poly[:, 1], poly[0, 1]], color="#8a8984", lw=0.8, zorder=1)
        seen = set()
        for pid, info in topo["links"].items():
            r = role(pid, info)
            col = C[r]
            for l in info["lanes"]:
                xy = np.asarray(l["xy"])
                if len(xy) < 2:
                    continue
                lab = LABELS[r]
                ax.plot(xy[:, 0], xy[:, 1], color=col,
                        lw=2.4 if r in ("seed", "upstream_enter", "unlisted", "paired_leave") else 1.4,
                        zorder=3 if r == "seed" else 2, label=None if lab in seen else lab)
                seen.add(lab)
                if len(xy) >= 2:
                    d = xy[-1] - xy[-2]
                    n = np.linalg.norm(d)
                    if n > 1e-6:
                        ax.annotate("", xy=xy[-1], xytext=xy[-1] - d / n * 1.6,
                                    arrowprops=dict(arrowstyle="-|>", color=col, lw=1.2), zorder=4)
            if info.get("centre_xy"):
                cxy = np.asarray(info["centre_xy"])
                ax.plot(cxy[:, 0], cxy[:, 1], color="#52514e", lw=0.8, ls="--", zorder=2,
                        label=None if "道路中心线" in seen else "道路中心线")
                seen.add("道路中心线")
            # label at the middle of the first lane
            if info["lanes"]:
                xy = np.asarray(info["lanes"][len(info["lanes"]) // 2]["xy"])
                if len(xy) >= 2:
                    m = at_station(xy, stations(xy)[-1] / 2)
                    lim = zoom if k == 1 else over
                    ref = (np.asarray(mp) if (k == 1 and mp) else centre)
                    if np.all(np.abs(m - ref) < lim):
                        ax.text(m[0], m[1], "…" + pid[-5:], fontsize=7.5 if k == 0 else 9, color="#0b0b0b",
                                zorder=6, ha="center", va="center",
                                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75))
        if len(axis):
            ax.plot(axis[:, 0], axis[:, 1], color=C["axis"], lw=1.0, ls=":", zorder=5, label="种子参考轴（转换器 gxy）")
        if mp:
            ax.plot(*mp, marker="X", ms=11, color=C["mouth"], zorder=7, label=f"算得的口部 station={fl['mouth_station']}")
        kp = fl.get("min_keep_point_xy")
        if kp:
            ax.plot(*kp, marker="|", ms=16, mew=2.5, color=C["keep"], zorder=7,
                    label=f"最小保留 station={fl['min_keep_m']:.3f}")
        el = fl.get("earliest_end_xy")
        if el:
            ax.plot(*el, marker="o", ms=8, mfc="none", mec=C["mouth"], mew=2, zorder=7,
                    label="最早真实车道端点")
        ref = np.asarray(mp) if (k == 1 and mp) else centre
        lim = zoom if k == 1 else over
        ax.set_xlim(ref[0] - lim, ref[0] + lim)
        ax.set_ylim(ref[1] - lim, ref[1] + lim)
        ax.set_aspect("equal")
        ax.grid(color="#e6e5e0", lw=0.6)
        ax.set_xlabel("x [m]（路口中心局部平面）", color="#52514e")
        ax.set_ylabel("y [m]", color="#52514e")
        for sp in ax.spines.values():
            sp.set_color("#c3c2b7")
        ax.set_title(("全貌" if k == 0 else "失败进口放大") + f"（±{lim:.0f} m）", fontsize=11, color="#0b0b0b")
        if k == 0:
            ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
    fig.suptitle(title, fontsize=12.5)
    fig.tight_layout()
    fig.savefig(out_png)
    print("wrote", out_png)


if __name__ == "__main__":
    a = sys.argv
    main(a[1], a[2], a[3], a[4], float(a[5]) if len(a) > 5 else 30.0, float(a[6]) if len(a) > 6 else 90.0,
         json.loads(a[7]) if len(a) > 7 else None)
