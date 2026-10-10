"""Stations of the outer-edge curvature sign flips counted by smoothness.edge_shape_quality (same sampling and
0.003 /m floor). From the repo root:
PYTHONPATH=. python handoff_assets/research/mouth-anchor-keep-slope-20261010/outer_flips_locate.py <xodr>..."""
import sys

import numpy as np
from lxml import etree

from mapforge.validate import smoothness as SM


def flips(path):
    root = etree.parse(path).getroot()
    out = []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        length = float(road.get("length"))
        secs = SM._sections(road)
        for si, sec in enumerate(secs):
            s0 = max(0.0, float(sec[0]))
            s1 = float(secs[si + 1][0]) if si + 1 < len(secs) else length
            if s1 - s0 <= 1e-6:
                continue
            margin = min(0.05, (s1 - s0) * 0.1)
            grid = np.arange(s0 + margin, s1 - margin + 1e-10, 0.25)
            for side in ("left", "right"):
                run = []
                for s in grid:
                    k, sh = SM._ref_kappa_at(road, float(s))
                    t, d1, d2 = SM.lane_edges_kinematics_at(road, float(s), side)[-1]
                    run.append((float(s), SM._edge_world_curvature(t, d1, d2, k, sh)))
                for (_sa, a), (sb, b) in zip(run, run[1:]):
                    if a * b < 0 and min(abs(a), abs(b)) > 0.003:
                        out.append({"road": road.get("id"), "side": side, "s": round(sb, 2), "length": round(length, 2)})
    return out


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(p, flips(p))
