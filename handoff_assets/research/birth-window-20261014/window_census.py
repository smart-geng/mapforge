"""Every lane birth/death lane_birth_advance finds in the lane_refit inputs, by whether its opening slope can be
measured: on the event section's own observations (main) and through the birth window (refit_birth_window). For the
ones still not measured, why: a boundary without source observations, or too few / too short even through the window.
cwd = code tree with refit_birth_window. argv: output json, raw xodr..."""
import json, sys
from collections import Counter
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from lxml import etree
from mapforge.ops import lane_birth_advance as BA
from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_birth_window as BW
from mapforge.validate.shp_boundary_fidelity import _origin


def span(obs, ev):
    if obs is None:
        return None
    lo, hi = (ev["s"], ev["s"] + BA.SLOPE_WINDOW_M) if ev["kind"] == "birth" else (ev["s"] - BA.SLOPE_WINDOW_M, ev["s"])
    sel = obs[(obs[:, 0] >= lo) & (obs[:, 0] <= hi)]
    return [int(len(sel)), round(float(np.ptp(sel[:, 0])) if len(sel) else 0.0, 2)]


src = LR._shp_source()
rows, tally = [], Counter()
for f in map(Path, sys.argv[2:]):
    root = etree.parse(str(f)).getroot()
    origin = _origin(root)
    for road_el in root.findall("road"):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        road = LR._Road(road_el)

        def observe(side, i, k, road=road):
            try:
                return LR._source_observations(road, src, origin, side, i, k)
            except IndexError:
                return None
        window = BW.observer(road, observe)
        for ev in BA.events(road_el):
            old = BA._opening_slope(observe, ev)
            new = BA._opening_slope(window, ev)
            k = abs(ev["lane"])
            sec = road.s[ev["section"] + 1] - road.s[ev["section"]]
            if old is not None:
                state = "measured on its own section"
            elif new is not None:
                state = "measured through the window"
            elif observe(ev["side"], ev["section"], k) is None or observe(ev["side"], ev["section"], k - 1) is None:
                state = "a boundary without source observations"
            else:
                state = "too short even through the window"
            tally[state] += 1
            rows.append({"file": f.name, "road": road_el.get("id"), "kind": ev["kind"], "side": ev["side"],
                         "lane": ev["lane"], "s": round(ev["s"], 2), "section_m": round(sec, 2), "state": state,
                         "slope_main": None if old is None else round(old, 3),
                         "slope_window": None if new is None else round(new, 3),
                         "own_outer_inner": [span(observe(ev["side"], ev["section"], kk), ev) for kk in (k, k - 1)],
                         "window_outer_inner": [span(window(ev["side"], ev["section"], kk), ev) for kk in (k, k - 1)]})
Path(sys.argv[1]).write_text(json.dumps({"tally": tally, "events": rows}, indent=1, ensure_ascii=False), encoding="utf-8")
print(json.dumps(tally, ensure_ascii=False, indent=1))
for r in rows:
    if r["state"] not in ("measured on its own section",):
        print(f"{r['file'][4:-9]:22s} road {r['road']:>3s} {r['kind']:5s} {r['side']:5s} {r['lane']:>3d} s {r['s']:7.2f} "
              f"sec {r['section_m']:5.2f}  {r['state']:40s} {r['slope_window']}  own {r['own_outer_inner']} window {r['window_outer_inner']}")
