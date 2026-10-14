"""Source and written width of an event lane near its birth/death: per source observation station of the outer and
inner boundary, the source width (outer - inner, inner interpolated) and the written width, for two stage XODRs of the
same raw input. cwd = code tree. argv: raw xodr, old stage xodr, new stage xodr, road id, side, lane id, s0, s1."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from lxml import etree
from mapforge.ops import lane_refit as LR
from mapforge.validate.shp_boundary_fidelity import _origin

raw, old, new, rid, side, lid, s0, s1 = sys.argv[1:9]
lid, s0, s1 = int(lid), float(s0), float(s1)
src = LR._shp_source()


def load(p):
    root = etree.parse(p).getroot()
    return LR._Road(next(r for r in root.findall("road") if r.get("id") == rid)), _origin(root)


def source_width(road, origin):
    k = abs(lid)
    pts = []
    for i in range(len(road.sections)):
        if road.s[i + 1] < s0 or road.s[i] > s1:
            continue
        lanes = road.lanes(i, side)
        if k > len(lanes):
            continue
        try:
            o = LR._source_observations(road, src, origin, side, i, k)
            n = LR._source_observations(road, src, origin, side, i, k - 1)
        except IndexError:
            continue
        if o is None or n is None or len(o) < 1 or len(n) < 2:
            continue
        order = np.argsort(n[:, 0])
        for s, t in o:
            if s0 <= s <= s1 and n[order[0], 0] <= s <= n[order[-1], 0]:
                pts.append((s, abs(t - np.interp(s, n[order, 0], n[order, 1]))))
    return sorted(pts)


def written(road, s):
    for i in range(len(road.sections)):
        if road.s[i] <= s < road.s[i + 1]:
            lanes = road.lanes(i, side)
            k = abs(lid)
            if k > len(lanes):
                return 0.0
            return abs(road.current(i, side, k, s) - road.current(i, side, k - 1, s))
    return float("nan")


r_raw, o_raw = load(raw)
r_old, _ = load(old)
r_new, _ = load(new)
for s, w in source_width(r_raw, o_raw):
    print(f"s {s:7.2f}  source {w:6.3f}  written main {written(r_old, s):6.3f}  window {written(r_new, s):6.3f}")
