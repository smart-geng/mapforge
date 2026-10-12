"""Junction enter/leave lists versus lane topology through the junction-interior links (read-only).

Junction-interior links: ROADLINK MEMO == junction pid or ROADCENTER IS_JUNC == 1 with lanes touching the junction
surface. A link is a topological enter road when one of its lanes continues into an interior lane and it is not
interior itself; a topological leave road likewise from an interior lane.

usage (cwd = experiment tree): python junction_roles.py <junction pid> [more pids ...]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import shapefile

sys.path.insert(0, str(Path.cwd()))
from shapely.geometry import Point, Polygon  # noqa: E402

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402

SHP = Path("shp_0222-0326")


def main(pids, use_memo=True):
    src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
    isj, memo = {}, {}
    r = shapefile.Reader(str(SHP / "IBD_ROADCENTER"), encoding="gbk")
    f = [x[0] for x in r.fields[1:]]
    for rec in r.iterRecords():
        m = dict(zip(f, rec))
        isj.setdefault(str(m["LINK_PID"]).strip(), set()).add(int(m["IS_JUNC"] or 0))
    r = shapefile.Reader(str(SHP / "IBD_ROADLINK"), encoding="gbk")
    f = [x[0] for x in r.fields[1:]]
    for rec in r.iterRecords():
        m = dict(zip(f, rec))
        memo[str(m["LINK_PID"]).strip()] = str(m.get("MEMO") or "").strip()
    juncs = {j.pid: j for j in src.junctions}
    for pid in pids:
        junc = juncs[pid]
        lon0, lat0 = float(junc.center[0]), float(junc.center[1])
        proj = lambda p: S._proj(p, lat0, lon0)  # noqa: E731
        poly = Polygon(proj(junc.polygon)).buffer(1.0)
        interior = {k for k, v in memo.items() if v == pid} if use_memo else set()
        # IS_JUNC links touching this junction's surface
        for k, flags in isj.items():
            if 1 in flags and k not in interior:
                lanes = src.lanes_of(k)
                if lanes and any(poly.contains(Point(proj(l.geometry)[len(l.geometry) // 2]))
                                 for l in lanes if len(l.geometry) >= 2):
                    interior.add(k)
        enter_topo, leave_topo = set(), set()
        for k in interior:
            for l in src.lanes_of(k):
                for q in src.topo_in.get(l.lane_pid, []):
                    ql = src.lane(q)
                    if ql is not None and ql.link_pid not in interior:
                        enter_topo.add(ql.link_pid)
                for q in src.topo_out.get(l.lane_pid, []):
                    ql = src.lane(q)
                    if ql is not None and ql.link_pid not in interior:
                        leave_topo.add(ql.link_pid)
        listed_e, listed_l = set(junc.enter_roads), set(junc.leave_roads)
        print("junction", pid, junc.name, "interior links", len(interior))
        print("  listed enter not topological:", sorted(listed_e - enter_topo),
              {p: ("interior" if p in interior else "dangling" if p not in src.roadlinks else "other")
               for p in sorted(listed_e - enter_topo)})
        print("  topological enter not listed:", sorted(enter_topo - listed_e),
              {p: (src.roadlinks[p].name, src.roadlinks[p].lane_num) for p in sorted(enter_topo - listed_e)
               if p in src.roadlinks})
        print("  listed leave not topological:", sorted(listed_l - leave_topo),
              {p: ("interior" if p in interior else "dangling" if p not in src.roadlinks else "other")
               for p in sorted(listed_l - leave_topo)})
        print("  topological leave not listed:", sorted(leave_topo - listed_l),
              {p: (src.roadlinks[p].name, src.roadlinks[p].lane_num) for p in sorted(leave_topo - listed_l)
               if p in src.roadlinks})


if __name__ == "__main__":
    args = sys.argv[1:]
    use_memo = "--is-junc-only" not in args
    main([a for a in args if not a.startswith("--")], use_memo)
