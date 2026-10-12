"""Plan view + heading profile of a failing entering road (read-only on sources), plus a coordinate CSV.

usage: python plot_case.py <case_tag> <junction_pid> <seed_link> <capture.pkl> <out_dir> [context_link ...]
"""
from __future__ import annotations

import csv
import math
import pickle
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

TREE = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/diag-leg-tree")
sys.path.insert(0, str(TREE))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import leg_fit_fallback as LF  # noqa: E402
from mapforge.ops import refline_fit as RF  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
GREY, INK, MUTED = "#b9b8b3", "#0b0b0b", "#52514e"
plt.rcParams.update({"font.family": ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"], "font.size": 9,
                     "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.spines.top": False, "axes.spines.right": False})


def stations(p):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]


def headings(p):
    d = np.diff(p, axis=0)
    return np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))


def pv_points(pv, step=0.1):
    total = sum(s.length for s in pv.segs)
    return RF._planview_at_s(pv, np.linspace(0.0, total, max(50, int(total / step) + 1)))


def main():
    tag, jpid, seed, cap, out = sys.argv[1:6]
    context = [a for a in sys.argv[6:] if not a.startswith(("after=", "note="))]
    note = next((a[5:] for a in sys.argv[6:] if a.startswith("note=")), None)
    after = next((a[6:] for a in sys.argv[6:] if a.startswith("after=")), None)
    out = Path(out)
    src = ProfileSource(str(TREE / "shp_0222-0326"), "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == jpid)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: S._proj(p, lat0, lon0)  # noqa: E731
    data = pickle.load(open(cap, "rb"))
    fits = [e for e in data["fits"] if e["seed"] == seed and not e["ok"]]
    fbs = [f for f in data["fallbacks"] if f["leg"] and f["leg"][1] == seed]
    first, last = fits[0], fits[-1]
    last_fb = fbs[-1]

    centre = proj(src.roadcenters[seed])
    lanes = [(l, proj(l.geometry)) for l in src.lanes_of(seed) if len(l.geometry) >= 2]
    bounds = [proj(b) for l, _ in lanes for b in src.lane_boundary_geometries(l.lane_pid)]

    # best relaxed candidate the fallback produced on the seed-only leg (re-fitted, same inputs)
    gxy = last_fb["gxy"]
    src_pts, _ = RF._remove_impulse_outliers(gxy)
    field = LF.heading_field(src_pts, last_fb["polylines"])
    cand = {}
    for kcap in (0.009, 0.1):
        n = max(3, int(math.ceil(stations(src_pts)[-1] / 25.0)))
        pv, _info = LF._fit_once(src_pts, field, n=n, kappa_cap=kcap, sharp_cap=LF.sharp_for(kcap) if kcap > 0.009
                                 else LF.BASE_SHARP, junction_end="end", free_ends=kcap > 0.009)
        cand[kcap] = (pv, LF.assess(pv, src_pts, field, "end"))

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(13, 6.4), gridspec_kw={"width_ratios": [1.15, 1]})
    poly = proj(junc.polygon)
    ax.fill(poly[:, 0], poly[:, 1], color="#ecebe8", zorder=0)
    ax.plot(np.r_[poly[:, 0], poly[:1, 0]], np.r_[poly[:, 1], poly[:1, 1]], color=GREY, lw=1, zorder=1)
    for cl in context:
        for l in src.lanes_of(cl):
            if len(l.geometry) >= 2:
                g = proj(l.geometry)
                ax.plot(g[:, 0], g[:, 1], color=GREY, lw=1, zorder=2)
        c = src.roadcenters.get(cl)
        if c is not None:
            g = proj(c)
            ax.plot(g[:, 0], g[:, 1], color=GREY, lw=1.6, ls="--", zorder=2)
    for b in bounds:
        ax.plot(b[:, 0], b[:, 1], color=BLUE, lw=1, ls=(0, (4, 2)), zorder=3)
    for _l, g in lanes:
        ax.plot(g[:, 0], g[:, 1], color=BLUE, lw=3.2, zorder=4)
    ax.plot(first["gxy"][:, 0], first["gxy"][:, 1], color=ORANGE, lw=1, ls=":", zorder=4)
    ax.plot(centre[:, 0], centre[:, 1], color=ORANGE, lw=2.2, zorder=5)
    ax.plot(*centre[0], marker="o", ms=8, color=ORANGE, mec="white", mew=2, zorder=6)
    for kcap, (pv, a) in cand.items():
        q = pv_points(pv)
        ax.plot(q[:, 0], q[:, 1], color=AQUA, lw=2 if kcap > 0.009 else 1.2, ls="-" if kcap > 0.009 else "--", zorder=5)
    after_info = None
    if after:
        adata = pickle.load(open(after, "rb"))
        afits = [e for e in adata["fits"] if e["seed"] == seed and e["attempt"] > max(f["attempt"] for f in fits)]
        if afits:
            e = afits[-1]
            apv = None
            if e["ok"]:
                apv = RF.fit_leg_refline(e["gxy"])[0]
                how = f"转换器自身拟合 {e['selection']}，离输入最大 {e['dev']:.2f} m"
            else:
                acc = [f for f in adata["fallbacks"] if f["leg"] and f["leg"][1] == seed and f["accepted"]]
                if acc:
                    apv = acc[-1]["pv"]
                    ch = [r for r in acc[-1]["tried"] if r["accepted"]][-1]
                    how = (f"兜底 κ≤{ch['kappa_cap']:g} 接受，方向偏差最大 {ch['heading_max_deg']:.1f}°，"
                           f"离输入最大 {ch['max_m']:.2f} m")
                else:
                    how = "车道轴上仍未拟合成功"
            ax.plot(e["gxy"][:, 0], e["gxy"][:, 1], color=INK, lw=1, ls=(0, (1, 1.5)), zorder=7)
            if apv is not None:
                q = pv_points(apv)
                ax.plot(q[:, 0], q[:, 1], color=INK, lw=1.4, zorder=7)
                qs, qh = stations(q), headings(q)
                after_info = (q, how)
            else:
                after_info = (None, how)
    # view: around the seed link and its lanes
    allp = np.vstack([centre] + [g for _l, g in lanes] + bounds)
    lo, hi = allp.min(axis=0) - 8, allp.max(axis=0) + 8
    span = max(hi - lo)
    mid = 0.5 * (lo + hi)
    ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
    ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
    ax.set_aspect("equal")
    ax.set_xlabel("x 东 [m]（路口中心局部坐标）")
    ax.set_ylabel("y 北 [m]")
    ax.grid(color="#e4e3df", lw=0.6)
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=ORANGE, lw=2.2, marker="o", mec="white", label=f"种子 link {seed[-6:]} 的 ROADCENTER（●=起点）"),
               Line2D([], [], color=ORANGE, lw=1, ls=":", label=f"首次拟合输入（拼链 {len(first['chain'])} 个 link，口部前移后）"),
               Line2D([], [], color=BLUE, lw=3.2, label="该 link 的车道中心线（LANE_LINK）"),
               Line2D([], [], color=BLUE, lw=1, ls=(0, (4, 2)), label="该车道的两条边界（LANE_BOUNDARY）"),
               Line2D([], [], color=AQUA, lw=1.2, ls="--", label="兜底拟合 κ≤0.009（基础档）"),
               Line2D([], [], color=AQUA, lw=2, label="兜底拟合 κ≤0.1（最宽档，仍不接受）"),
               Line2D([], [], color=GREY, lw=1, label="相邻 link 车道 / 虚线=其 ROADCENTER")]
    if after_info is not None:
        handles += [Line2D([], [], color=INK, lw=1, ls=(0, (1, 1.5)), label="改后：车道轴拟合输入（口部前移后）"),
                    Line2D([], [], color=INK, lw=1.4, label="改后参考线：" + after_info[1])]
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.0, -0.09), fontsize=8, frameon=False, ncol=1)
    if note:
        ax.text(0.02, 0.98, note.replace("|", "\n"), transform=ax.transAxes, va="top", ha="left", fontsize=8,
                color=INK, bbox={"facecolor": "white", "edgecolor": GREY, "alpha": 0.92, "boxstyle": "round,pad=0.4"})
    ax.set_title(f"{tag}：进口道路 {seed} 平面", fontsize=10, color=INK, loc="left")

    sc, hc = stations(src_pts), headings(src_pts)
    bx.plot(0.5 * (sc[1:] + sc[:-1]), hc, color=ORANGE, lw=2, label="拟合输入（种子 ROADCENTER，口部前移后）的航向")
    grid, heading, _base = field
    bx.plot(grid, np.degrees(heading), color=BLUE, lw=2, label="车道方向场（车道中心线/边界航向中位数）")
    for kcap, (pv, a) in cand.items():
        q = pv_points(pv)
        sq = stations(q)
        bx.plot(0.5 * (sq[1:] + sq[:-1]), headings(q), color=AQUA, lw=2 if kcap > 0.009 else 1.2,
                ls="-" if kcap > 0.009 else "--",
                label=(f"兜底 κ≤{kcap:g}：方向偏差 P95 {a['heading_p95_deg']:.1f}° / 最大 {a['heading_max_deg']:.1f}°，"
                       f"离中心线中位 {a['median_m']:.2f} m，方向场覆盖 {a['field_coverage']:.0%}"))
    if after_info is not None and after_info[0] is not None:
        q = after_info[0]
        mid = 0.5 * (q[1:] + q[:-1])
        st_on_src, _off = LF._project(src_pts, mid)
        bx.plot(st_on_src, headings(q), color=INK, lw=2.4, alpha=0.75,
                label="改后参考线（车道轴）航向，按投影到改前输入的里程画")
    bx.set_xlabel("沿拟合输入的里程 s [m]（远端→路口）")
    bx.set_ylabel("航向 [°]（东=0，逆时针）")
    bx.grid(color="#e4e3df", lw=0.6)
    bx.legend(loc="upper left", bbox_to_anchor=(0.0, -0.09), fontsize=8, frameon=False)
    bx.set_title("航向剖面：参考线输入 vs 车道", fontsize=10, color=INK, loc="left")
    fig.tight_layout(rect=(0, 0.0, 1, 1))
    png = out / f"{tag}_{jpid}_leg_{seed[-6:]}.png"
    fig.savefig(png, dpi=130, bbox_inches="tight")
    print("wrote", png)

    rows = []
    for name, ident, g in ([("ROADCENTER", seed, centre)] + [("LANE_CENTRE", l.lane_pid, g) for l, g in lanes]
                           + [("LANE_BOUNDARY", f"{lanes[0][0].lane_pid}#b{k}", b) for k, b in enumerate(bounds)]
                           + [("FIT_INPUT_FIRST", seed, first["gxy"]), ("FIT_INPUT_LAST", seed, last["gxy"])]):
        s = stations(g)
        h = np.r_[headings(g), np.nan] if len(g) > 1 else [np.nan]
        for i, (p, si, hi_) in enumerate(zip(g, s, h)):
            rows.append([name, ident, i, round(float(p[0]), 3), round(float(p[1]), 3), round(float(si), 3),
                         None if not np.isfinite(hi_) else round(float(hi_), 2)])
    with (out / f"{tag}_{jpid}_coords.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["object", "id", "index", "x_east_m", "y_north_m", "station_m", "heading_to_next_deg"])
        w.writerows(rows)
    print("wrote", out / f"{tag}_{jpid}_coords.csv", len(rows), "rows")
    for kcap, (pv, a) in cand.items():
        print(kcap, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in a.items()})


if __name__ == "__main__":
    main()
