"""Plot one road in its own (s, t) frame: the written lane edges (blue) against both boundaries of every source lane the
road's lanes name (colour, labelled with the last 5 digits of the source lane id), projected onto the road's reference
line as lane_refit projects them. Section boundaries dotted. cwd = code tree.
argv: xodr, road id, s0, s1, png [, side: left|right|both]"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from lxml import etree
from mapforge.ops import lane_refit as LR
from mapforge.validate.shp_boundary_fidelity import _origin, _densify, _project as to_local
from mapforge.validate.smoothness import lane_edges_at

xodr, rid, s0, s1, png = Path(sys.argv[1]), sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
which = sys.argv[6] if len(sys.argv) > 6 else "both"
root = etree.parse(str(xodr)).getroot()
origin = _origin(root)
road_el = next(r for r in root.findall("road") if r.get("id") == rid)
road = LR._Road(road_el)
src = LR._shp_source()
fig, ax = plt.subplots(figsize=(16, 8))
sides = ("left", "right") if which == "both" else (which,)
seen = {}
for i in range(len(road.sections)):
    for side in sides:
        for lane in road.lanes(i, side):
            sid = lane.find("userData[@code='mapforge.source_lane']")
            if sid is None or sid.get("value") in seen:
                continue
            seen[sid.get("value")] = True
            colour = plt.cm.tab20(len(seen) % 20)
            for g in src.lane_boundary_geometries(sid.get("value")):
                pts = _densify(to_local(g, *origin), 0.5)
                st = np.asarray([(s, t) for s, t, clamped in LR._project(pts, *road.ref) if not clamped])
                if not len(st):
                    continue
                keep = (st[:, 0] >= s0 - 5) & (st[:, 0] <= s1 + 5)
                if keep.any():
                    ax.plot(st[keep, 0], st[keep, 1], color=colour, lw=1.4, alpha=0.9)
                    j = int(np.argmax(keep))
                    ax.annotate(sid.get("value")[-5:], (st[j, 0], st[j, 1]), color=colour, fontsize=7)
ss = np.arange(s0, s1, 0.1)
edges = {}
for s in ss:
    for side in sides:
        for k, t in enumerate(lane_edges_at(road_el, s, side=side)):
            edges.setdefault((side, k, road.s[int(np.searchsorted(road.s, s, "right")) - 1]), []).append((s, t))
for key, pts in edges.items():
    pts = np.asarray(pts)
    ax.plot(pts[:, 0], pts[:, 1], color="tab:blue", lw=0.8)
for x in road.s:
    if s0 <= x <= s1:
        ax.axvline(x, color="0.7", ls=":", lw=0.8)
ax.set_xlim(s0, s1)
ax.set_xlabel("s [m]")
ax.set_ylabel("t [m] (left +)")
ax.set_title(f"{xodr.name} road {rid}: written lane edges (blue) vs source lane boundaries (colour)")
fig.tight_layout()
fig.savefig(png, dpi=110)
print(png)
