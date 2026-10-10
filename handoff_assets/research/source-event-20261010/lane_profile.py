"""Source vs written edges of one source lane on one road: s, source outer/inner t, width, nearest written edges."""
import json, math, sys
import numpy as np, xml.etree.ElementTree as ET
sys.path.insert(0, ".")
from mapforge.validate import shp_boundary_fidelity as B
from mapforge.validate.smoothness import sample_road_ref, lane_edges_at
from mapforge.adapters.shp.profile_source import ProfileSource
X, RID, SID = sys.argv[1], sys.argv[2], sys.argv[3]
root = ET.parse(X).getroot(); lat0, lon0 = B._origin(root)
src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
road = next(r for r in root.findall("road") if r.get("id") == RID)
pts, ss, hh = sample_road_ref(road, 0.05)
def st_of(P):
    out = []
    for p in P:
        i = int(np.argmin(np.linalg.norm(pts - p, axis=1)))
        tv = np.array([math.cos(hh[i]), math.sin(hh[i])]); nv = np.array([-tv[1], tv[0]]); d = p - pts[i]
        out.append((ss[i] + d @ tv, d @ nv))
    a = np.array(out); return a[np.argsort(a[:, 0])]
bs = [st_of(B._densify(B._project(g, lat0, lon0), 0.5)) for g in src.lane_boundary_geometries(SID)]
L = float(road.get("length"))
lo = max(0, max(b[0, 0] for b in bs)); hi = min(L, min(b[-1, 0] for b in bs))
res = []
for s in np.arange(lo, hi, 1.0):
    ts = [float(np.interp(s, b[:, 0], b[:, 1])) for b in bs]
    edges = lane_edges_at(road, s, "right") + lane_edges_at(road, s, "left")[1:]
    r = [min((t - e for e in edges), key=abs) for t in ts]
    res.append((s, ts, r))
w = [abs(x[1][0] - x[1][1]) for x in res]
worst = max(range(len(res)), key=lambda i: max(abs(v) for v in res[i][2]))
print(f"road {RID} lane {SID} s {lo:.1f}..{hi:.1f}  worst at s={res[worst][0]:.1f} r={[round(v,3) for v in res[worst][2]]}")
for i in range(max(0, worst - 14), min(len(res), worst + 12), 2):
    s, ts, r = res[i]
    print(f"  s {s:6.1f} width {w[i]:6.3f}  res {[f'{v:+.3f}' for v in r]}")
