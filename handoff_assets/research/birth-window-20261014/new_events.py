"""The births/deaths the birth window lets lane_birth_advance measure (advanced by the new code, not by main's): event,
its section length in the raw stage XODR, the span and count of the own-section observations of both boundaries,
the opening slope measured through the window and the advance. cwd = code tree (with refit_birth_window).
argv: adv-old.json, adv-new.json (adv_report.py), raw dir."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from lxml import etree
from mapforge.ops import lane_refit as LR
from mapforge.validate.shp_boundary_fidelity import _origin

old, new = (json.loads(Path(p).read_text(encoding="utf-8")) for p in sys.argv[1:3])
raw_dir = Path(sys.argv[3])
src = LR._shp_source()
for name, rec in new.items():
    before = {(r, e["kind"], e["side"], e["lane"], e["s"]) for r, v in old[name]["report"].items() for e in v
              if e.get("advanced_m")}
    fresh = [(r, e) for r, v in rec["report"].items() for e in v
             if e.get("advanced_m") and (r, e["kind"], e["side"], e["lane"], e["s"]) not in before]
    if not fresh:
        continue
    root = etree.parse(str(raw_dir / name)).getroot()
    origin = _origin(root)
    for rid, e in fresh:
        road = LR._Road(next(r for r in root.findall("road") if r.get("id") == rid))
        i, k = e["section"], abs(e["lane"])
        spans = []
        for kk in (k, k - 1):
            try:
                obs = LR._source_observations(road, src, origin, e["side"], i, kk)
            except IndexError:
                obs = None
            spans.append("-" if obs is None else f"{len(obs)} obs / {np.ptp(obs[:, 0]):.2f} m")
        print(f"{name[:-9]:24s} road {rid:>3s} {e['kind']:5s} {e['side']:5s} lane {e['lane']:>3d} s {e['s']:8.2f}"
              f"  section {road.s[i + 1] - road.s[i]:5.2f} m  own obs (outer; inner) {spans[0]}; {spans[1]}"
              f"  slope {e['slope']:.3f}  advance {e['advanced_m']:.2f} m")
