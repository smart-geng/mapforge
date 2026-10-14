"""Boundary deviation from the source near given lane events, for two versions of the same XODR.

For every ordinary road boundary with source observations (lane_refit._source_observations), the largest
|written - source| among observations within WIN m of each event station, per (road, side) and event, old vs new.
Only events the old report did not advance. cwd = code tree. argv: old and new events json (adv_report.py),
old dir, new dir, names..."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from lxml import etree
from mapforge.ops import lane_refit as LR
from mapforge.validate.shp_boundary_fidelity import _origin

WIN = 15.0
old_events = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
events_by_file = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
old_dir, new_dir = Path(sys.argv[3]), Path(sys.argv[4])
src = LR._shp_source()


def devs(path, rid, side, s_ev):
    root = etree.parse(str(path)).getroot()
    origin = _origin(root)
    road_el = next(r for r in root.findall("road") if r.get("id") == rid)
    road = LR._Road(road_el)
    out = {}
    for i in range(len(road.sections)):
        if road.s[i + 1] < s_ev - WIN or road.s[i] > s_ev + WIN:
            continue
        lanes = road.lanes(i, side)
        for k in range(len(lanes) + 1):
            try:
                obs = LR._source_observations(road, src, origin, side, i, k)
            except IndexError:
                obs = None
            if obs is None or not len(obs):
                continue
            obs = obs[np.abs(obs[:, 0] - s_ev) <= WIN]
            if not len(obs):
                continue
            d = np.array([road.current(i, side, k, s) - t for s, t in obs])
            j = int(np.argmax(np.abs(d)))
            # the boundary by the source lane on its inner side (stable across new sections)
            key = (k, lanes[k - 1].find("userData[@code='mapforge.source_lane']").get("value")[-6:]
                   if k >= 1 and lanes[k - 1].find("userData[@code='mapforge.source_lane']") is not None else "c")
            if key not in out or abs(d[j]) > abs(out[key][0]):
                out[key] = (float(d[j]), float(obs[j, 0]))
    return out


for name in sys.argv[5:]:
    raw = f"{name}.raw.xodr"
    rep = events_by_file[raw]["report"]
    before = {(r, e["kind"], e["side"], e["lane"], e["s"]) for r, v in old_events[raw]["report"].items() for e in v
              if e.get("advanced_m")}
    for rid, evs in rep.items():
        for ev in evs:
            if not ev.get("advanced_m") or (rid, ev["kind"], ev["side"], ev["lane"], ev["s"]) in before:
                continue
            o = devs(old_dir / f"{name}.stage.xodr", rid, ev["side"], ev["s"])
            n = devs(new_dir / f"{name}.stage.xodr", rid, ev["side"], ev["s"])
            rows = []
            for key in sorted(set(o) | set(n)):
                a, b = o.get(key), n.get(key)
                if a and b and abs(abs(a[0]) - abs(b[0])) < 0.01:
                    continue
                rows.append(f"k{key[0]}({key[1]}) {'-' if not a else f'{a[0]:+.3f}@{a[1]:.1f}'} -> {'-' if not b else f'{b[0]:+.3f}@{b[1]:.1f}'}")
            print(name[-10:], "road", rid, ev["kind"], ev["side"], ev["lane"], "s", ev["s"], "adv", ev["advanced_m"], "|", "; ".join(rows) or "no change > 1 cm")
