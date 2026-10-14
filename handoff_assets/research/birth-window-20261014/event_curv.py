"""Largest world curvature of the lane edges on the event side within WIN m of each new event (new-events.txt), two
XODRs of the same junction side by side (smoothness.edge_shape_quality's sampling and formula, as edge_curv_where.py):
the edge, its station and value, before and after; the station s of the event is the source event's. cwd = code tree.
argv: before dir, after dir, suffix ('.xodr' or '.stage.xodr'), events 'name:road:side:s' ..."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge.validate import smoothness as SM

WIN = 15.0


def peak(xodr, rid, side, s_ev):
    road = next(r for r in ET.parse(xodr).getroot().findall("road") if r.get("id") == rid)
    length, secs = float(road.get("length")), SM._sections(road)
    best = (0.0, None, None)
    for si, sec in enumerate(secs):
        s0 = max(0.0, float(sec[0]))
        s1 = float(secs[si + 1][0]) if si + 1 < len(secs) else length
        if s1 < s_ev - WIN or s0 > s_ev + WIN:
            continue
        margin = min(0.05, (s1 - s0) * 0.1)
        for s in np.arange(s0 + margin, s1 - margin + 1e-10, 0.25):
            if abs(s - s_ev) > WIN:
                continue
            kappa, sharp = SM._ref_kappa_at(road, float(s))
            for j, (t, d1, d2) in enumerate(SM.lane_edges_kinematics_at(road, float(s), side)):
                k = abs(SM._edge_world_curvature(t, d1, d2, kappa, sharp))
                if k > best[0]:
                    best = (k, float(s), j)
    return best


before, after, suffix = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
for arg in sys.argv[4:]:
    name, rid, side, s = arg.split(":")
    a = peak(str(before / f"{name}{suffix}"), rid, side, float(s))
    b = peak(str(after / f"{name}{suffix}"), rid, side, float(s))
    print(f"{name:24s} road {rid:>3s} {side:5s} s {float(s):7.2f}:  {a[0]:.4f} (edge {a[2]} @ {a[1]:7.2f})  ->  "
          f"{b[0]:.4f} (edge {b[2]} @ {b[1]:7.2f})")
