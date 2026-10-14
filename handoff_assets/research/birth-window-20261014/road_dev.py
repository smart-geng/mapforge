"""Boundary deviation from the source along whole roads, two versions of the same XODR side by side.

For every boundary of the given roads with source observations (lane_refit._source_observations, the observations
the lane refit fits), |written - source| at each observation; per (road, side, boundary named by the source lane on
its inner side) the largest value and its station, in three parts of the road: within WIN m of the road's new
events, and before / after that window. Boundaries are matched across versions by that name, so new sections do not
shift them. cwd = code tree. argv: before dir, after dir, suffix ('.xodr' or '.stage.xodr'), then
'name:road:side:s_event[,s_event...]' ..."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from lxml import etree
from mapforge.ops import lane_refit as LR
from mapforge.validate.shp_boundary_fidelity import _origin

WIN = 15.0
src = LR._shp_source()


def name_of(lanes, k):
    if k == 0:
        return "centre"
    ud = lanes[k - 1].find("userData[@code='mapforge.source_lane']")
    return ud.get("value")[-6:] if ud is not None else f"lane{lanes[k - 1].get('id')}"


def devs(path, rid, side):
    root = etree.parse(str(path)).getroot()
    origin = _origin(root)
    road = LR._Road(next(r for r in root.findall("road") if r.get("id") == rid))
    out = {}
    for i in range(len(road.sections)):
        lanes = road.lanes(i, side)
        for k in range(len(lanes) + 1):
            try:
                obs = LR._source_observations(road, src, origin, side, i, k)
            except IndexError:
                obs = None
            if obs is None or not len(obs):
                continue
            key = (k, name_of(lanes, k))
            d = np.array([road.current(i, side, k, s) - t for s, t in obs])
            out.setdefault(key, []).extend(zip(obs[:, 0].tolist(), d.tolist()))
    return out


def parts(rows, events):
    res = {}
    for s, d in rows:
        near = any(abs(s - e) <= WIN for e in events)
        part = "near" if near else ("before" if s < min(events) else "after")
        if part not in res or abs(d) > abs(res[part][0]):
            res[part] = (d, s)
    return res


before, after, suffix = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
for arg in sys.argv[4:]:
    name, rid, side, evs = arg.split(":")
    events = [float(e) for e in evs.split(",")]
    a, b = devs(before / f"{name}{suffix}", rid, side), devs(after / f"{name}{suffix}", rid, side)
    print(f"== {name} road {rid} {side} events {events}")
    for key in sorted(set(a) | set(b)):
        pa, pb = parts(a.get(key, []), events), parts(b.get(key, []), events)
        cells = []
        for part in ("before", "near", "after"):
            x, y = pa.get(part), pb.get(part)
            fx = "-" if x is None else f"{abs(x[0]):.3f}@{x[1]:.1f}"
            fy = "-" if y is None else f"{abs(y[0]):.3f}@{y[1]:.1f}"
            mark = "*" if x and y and abs(abs(x[0]) - abs(y[0])) >= 0.01 else " "
            cells.append(f"{part} {fx} -> {fy}{mark}")
        print(f"   k{key[0]} ({key[1]}): " + " | ".join(cells))
