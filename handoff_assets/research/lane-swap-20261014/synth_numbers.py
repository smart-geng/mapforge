"""Numbers of the synthetic lane-swap tests (cwd = code tree)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd())); sys.path.insert(0, str(Path.cwd() / "tests"))
import numpy as np
from lxml import etree
import test_refit_lane_swap as T
from mapforge.ops import lane_refit as LR
for kind in ("inner", "outer"):
    bounds = T._boundaries(kind)
    LR._source_observations = lambda road, src, origin, side, i, k, b=bounds: b.get((i, k)) if side == "right" else None
    root = etree.fromstring(T._xodr(kind).encode())
    for sw in (False, True):
        _, fitted, report = LR.refit_road(root.find("road"), None, T.ORIGIN, {**T.P, "lane_swaps": sw})
        dev = [(abs(T._outer_edge(fitted, s) - T.EDGE), s) for s in T.STATIONS]
        k2 = max(abs(LR._at(fitted[("right", T._section(s), 2)], s, 2)) for s in np.arange(20.0, 40.0, 0.05))
        print(kind, "swaps" if sw else "apart", "outer edge max dev %.3f at s %.1f" % max(dev), "divider max |t''| %.3f" % k2,
              report.get("lane_swaps"))
