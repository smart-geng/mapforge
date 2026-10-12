"""Source-data exploration for one SHP junction (read-only).

usage: python explore.py <junction_pid> [<enter_link> ...]
Prints the junction's enter/leave roads (ROADLINK attributes), and for each named enter link: the converter's
chained links, centreline geometry vs lanes in the local metric frame, lane attributes and lane topology.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import shapefile

TREE = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/diag-leg-tree")
sys.path.insert(0, str(TREE))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402

SHP = TREE / "shp_0222-0326"


def raw_rows(layer, key, values):
    r = shapefile.Reader(str(SHP / layer), encoding="gbk")
    fields = [f[0] for f in r.fields[1:]]
    out = {}
    for sr in r.iterShapeRecords():
        m = dict(zip(fields, sr.record))
        k = str(m[key]).strip()
        if k in values:
            m = {a: (str(b).strip() if isinstance(b, str) else b) for a, b in m.items()}
            out.setdefault(k, []).append((m, np.asarray(sr.shape.points) if sr.shape.points else None))
    return out


def polylen(p):
    p = np.asarray(p, float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def heading_deg(p):
    d = np.diff(np.asarray(p, float), axis=0)
    return np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))


def main():
    pid = sys.argv[1]
    enters = sys.argv[2:]
    src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == pid)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: S._proj(p, lat0, lon0)  # noqa: E731
    cj = proj(junc.polygon).mean(axis=0)
    print("junction", junc.pid, junc.name, "center", lon0, lat0, "cj", cj.round(2))
    print("enter_roads", len(junc.enter_roads), "leave_roads", len(junc.leave_roads))
    links = set(junc.enter_roads) | set(junc.leave_roads)
    rl = raw_rows("IBD_ROADLINK", "LINK_PID", links)
    lf = raw_rows("IBD_LINK_FORM", "LINK_PID", links)
    keep = ("KIND", "SEPARATION", "DIRECT", "LANE_NUM", "LEFT_NUM", "ROADNAME", "IS_REFLINE", "S_NODE_PID",
            "E_NODE_PID", "LENGTH", "UPDOWN_DIR")
    for role, lst in (("enter", junc.enter_roads), ("leave", junc.leave_roads)):
        for p in lst:
            m = rl.get(p, [({}, None)])[0][0]
            forms = [x[0].get("ROAD_FORM") for x in lf.get(p, [])]
            c = S._center_of(src, p)
            g = proj(c) if c is not None else None
            info = {k: m.get(k) for k in keep}
            lanes = src.lanes_of(p)
            near = None
            if g is not None:
                end = g[-1] if np.linalg.norm(g[-1] - cj) < np.linalg.norm(g[0] - cj) else g[0]
                near = end.round(1).tolist()
            print(f"  {role} {p} forms={forms} lanes={len(lanes)} centre_len={polylen(g) if g is not None else None:.1f}"
                  f" junction_end={near} {json.dumps(info, ensure_ascii=False)}")
    pairs, single = S._pair_legs(src, junc, proj, cj)
    print("pairs:")
    for e, ls in pairs:
        print("  ", e, "->", ls)
    print("single leaves:", single)


if __name__ == "__main__":
    main()
