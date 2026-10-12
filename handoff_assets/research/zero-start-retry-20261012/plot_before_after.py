"""One road's written lane edges before and after a change, against its source lane boundaries, over an s range, in
the XODR's local metric frame (both XODRs must share the reference line). cwd = code tree.
argv: before xodr, after xodr, road id, s0, s1, png"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import shp_boundary_fidelity as B
from mapforge.validate.smoothness import lane_edges_at, sample_road_ref

before, after, rid = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
s0, s1, png = float(sys.argv[4]), float(sys.argv[5]), sys.argv[6]
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")
fig, ax = plt.subplots(figsize=(13, 8))
sources = set()
for path, colour, style, label in ((before, "0.45", "--", "before (converter geometry kept)"),
                                   (after, "tab:blue", "-", "after (boundary refit)")):
    root = ET.parse(path).getroot()
    lat0, lon0 = B._origin(root)
    road = next(r for r in root.findall("road") if r.get("id") == rid)
    pts, ss, hh = sample_road_ref(road, 0.1)
    keep = (ss >= s0 - 1e-9) & (ss <= s1 + 1e-9)
    pts, ss, hh = pts[keep], ss[keep], hh[keep]
    nrm = np.column_stack([-np.sin(hh), np.cos(hh)])
    edges = [lane_edges_at(road, float(s), side="right") for s in ss]
    width = max(len(e) for e in edges)
    for k in range(width):
        off = np.array([e[k] if k < len(e) else np.nan for e in edges], float)
        line = pts + off[:, None] * nrm
        ax.plot(line[:, 0], line[:, 1], color=colour, ls=style, lw=1.3, label=label if k == 0 else None)
    for lane in road.iter("lane"):
        u = lane.find("userData[@code='mapforge.source_lane']")
        if u is not None:
            sources.add(u.get("value"))
    for s in np.arange(np.ceil(s0), s1 + 1e-9, 5.0) if path == after else []:
        i = int(np.argmin(abs(ss - s)))
        ax.plot(*pts[i], "k+", ms=6)
        ax.annotate(f"s={s:.0f}", pts[i], fontsize=7)
# source boundaries of the lanes in the range
xy_after = None
first = True
for sid in sorted(sources):
    for g in src.lane_boundary_geometries(sid):
        P = B._project(g, lat0, lon0)
        ax.plot(P[:, 0], P[:, 1], color="tab:red", lw=0.8, alpha=0.8, label="source lane boundaries" if first else None)
        ax.plot(P[:, 0], P[:, 1], ".", color="tab:red", ms=2.5)
        first = False
# keep the view on the requested stretch
root = ET.parse(after).getroot()
road = next(r for r in root.findall("road") if r.get("id") == rid)
pts, ss, _ = sample_road_ref(road, 0.5)
sel = pts[(ss >= s0) & (ss <= s1)]
pad = 8.0
ax.set_xlim(sel[:, 0].min() - pad, sel[:, 0].max() + pad)
ax.set_ylim(sel[:, 1].min() - pad - 4, sel[:, 1].max() + pad)
ax.set_aspect("equal")
ax.set_title(f"road {rid}, s {s0:g}-{s1:g}: lane edges before / after vs source lane boundaries (red)")
ax.legend(loc="best", fontsize=8)
fig.tight_layout()
fig.savefig(png, dpi=130)
print("saved", png)
