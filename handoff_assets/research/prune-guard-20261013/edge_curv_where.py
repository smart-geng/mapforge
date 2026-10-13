"""World curvature of every lane edge of one road side (smoothness.edge_shape_quality's sampling and formula), per
laneSection: the largest |curvature| of each edge with its station, for two XODRs side by side.
cwd = code tree. argv: before xodr, after xodr, road id, side [, min curvature to list]"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge.validate import smoothness as SM


def table(xodr, rid, side):
    road = next(r for r in ET.parse(xodr).getroot().findall("road") if r.get("id") == rid)
    length, secs, out = float(road.get("length")), SM._sections(road), {}
    for si, sec in enumerate(secs):
        s0 = max(0.0, float(sec[0]))
        s1 = float(secs[si + 1][0]) if si + 1 < len(secs) else length
        margin = min(0.05, (s1 - s0) * 0.1)
        for s in np.arange(s0 + margin, s1 - margin + 1e-10, 0.25):
            kappa, sharp = SM._ref_kappa_at(road, float(s))
            for j, (t, d1, d2) in enumerate(SM.lane_edges_kinematics_at(road, float(s), side)):
                k = abs(SM._edge_world_curvature(t, d1, d2, kappa, sharp))
                key = (si, j)
                if key not in out or k > out[key][0]:
                    out[key] = (k, float(s), round(s0, 2), round(s1, 2))
    return out


before, after, rid, side = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
lim = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
a, b = table(before, rid, side), table(after, rid, side)
for key in sorted(set(a) | set(b)):
    x, y = a.get(key), b.get(key)
    if max((x or (0,))[0], (y or (0,))[0]) < lim:
        continue
    sec = (x or y)[2:]
    fx = f"{x[0]:.4f} @ {x[1]:6.2f}" if x else "-"
    fy = f"{y[0]:.4f} @ {y[1]:6.2f}" if y else "-"
    print(f"section {key[0]} s {sec[0]:7.2f}-{sec[1]:7.2f} edge {key[1]}:  {fx}   ->   {fy}")
