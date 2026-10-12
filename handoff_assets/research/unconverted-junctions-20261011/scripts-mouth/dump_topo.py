"""Dump source attributes and lane topology around one junction (read-only).

usage (cwd = experiment tree): python dump_topo.py <lon> <lat> <out.json> [extra link ids ...]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import shapefile

sys.path.insert(0, str(Path.cwd()))
from shapely.geometry import LineString, Point, Polygon  # noqa: E402

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402

SHP = Path("shp_0222-0326")


def rows(layer, key, wanted):
    r = shapefile.Reader(str(SHP / layer), encoding="gbk")
    fields = [f[0] for f in r.fields[1:]]
    out = {}
    for sr in r.iterShapeRecords():
        m = dict(zip(fields, sr.record))
        k = str(m.get(key, "")).strip()
        if k in wanted:
            out.setdefault(k, []).append({"attrs": {a: (str(b) if not isinstance(b, (int, float)) else b)
                                                    for a, b in m.items()},
                                          "points": [list(p) for p in sr.shape.points]})
    return out


def polylen(g):
    g = np.asarray(g, float)
    return float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()) if len(g) > 1 else 0.0


def main(lon, lat, out_json, extra):
    src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
    junc, _d = src.find_junction(lon, lat)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: S._proj(p, lat0, lon0)  # noqa: E731
    cj = proj(junc.polygon).mean(axis=0)
    poly = Polygon(proj(junc.polygon))
    links = list(dict.fromkeys(junc.enter_roads + junc.leave_roads + list(extra)))
    # one ring of lane-topology neighbours
    lane_link = {}
    for pid in list(links):
        for l in src.lanes_of(pid):
            lane_link[l.lane_pid] = pid
    nb_links = set()
    for lp in list(lane_link):
        for q in src.topo_in.get(lp, []) + src.topo_out.get(lp, []):
            ql = src.lane(q)
            if ql is not None:
                nb_links.add(ql.link_pid)
    rl_rows = rows("IBD_ROADLINK", "LINK_PID", set(links) | nb_links)
    out = {"junction": junc.pid, "name": junc.name, "enter": junc.enter_roads, "leave": junc.leave_roads,
           "polygon_xy": proj(junc.polygon).round(3).tolist(), "links": {}}
    for pid in sorted(set(links) | nb_links):
        rl = src.roadlinks.get(pid)
        info = {"role": ("enter" if pid in junc.enter_roads else "leave" if pid in junc.leave_roads
                         else "neighbour"),
                "name": rl.name if rl else None, "lane_num": rl.lane_num if rl else None,
                "has_centre": pid in src.roadcenters,
                "roadlink_attrs": [r["attrs"] for r in rl_rows.get(pid, [])],
                "roadlink_xy": [proj(np.asarray(r["points"])).round(3).tolist() for r in rl_rows.get(pid, [])],
                "centre_xy": proj(src.roadcenters[pid]).round(3).tolist() if pid in src.roadcenters else None,
                "lanes": []}
        for l in src.lanes_of(pid):
            lg = proj(l.geometry) if l.geometry.shape[0] else np.zeros((0, 2))
            ins = [lp for lp in src.topo_in.get(l.lane_pid, [])]
            outs = [lp for lp in src.topo_out.get(l.lane_pid, [])]
            info["lanes"].append({
                "lane": l.lane_pid, "seq": l.seq, "width": l.width_mm, "s_w": l.s_width_mm, "e_w": l.e_width_mm,
                "type": l.lane_type, "len_m": round(polylen(lg), 3), "xy": lg.round(3).tolist(),
                "start_in_surface": bool(len(lg) and poly.buffer(0.05).contains(Point(lg[0]))),
                "end_in_surface": bool(len(lg) and poly.buffer(0.05).contains(Point(lg[-1]))),
                "topo_in": [(q, src.lane(q).link_pid if src.lane(q) else None) for q in ins],
                "topo_out": [(q, src.lane(q).link_pid if src.lane(q) else None) for q in outs],
                "boundaries_xy": [proj(b).round(3).tolist() for b in src.lane_boundary_geometries(l.lane_pid)]})
        out["links"][pid] = info
    Path(out_json).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    for pid, info in out["links"].items():
        print(pid, info["role"], info["name"], "lanes", info["lane_num"], "centre", info["has_centre"])
        for l in info["lanes"]:
            print("   ", l["lane"], "seq", l["seq"], "len", l["len_m"], "in:", l["topo_in"], "out:", l["topo_out"],
                  "start_in_surf", l["start_in_surface"], "end_in_surf", l["end_in_surface"])


if __name__ == "__main__":
    main(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], sys.argv[4:])
