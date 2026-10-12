"""Read-only: which raw SHP objects lie in / around the unpaved pocket (0621) and the enclosed hole (0630)?

Reads the problem polygon (WKT) from the diagnosis JSON written by diag_0621.py / diag_0630.py, projects the raw
objects of every relevant IBD layer with the converter's projection (shp_to_xodr._proj about the junction centre),
and lists the objects that intersect the polygon grown by 1 m, with their raw attributes and overlap.

    cd <tree>; <python> <this> <diag_json> <pid> <out_json>
"""
from __future__ import annotations

import json
import sys

import numpy as np
import shapefile
from shapely import wkt
from shapely.geometry import LineString, Polygon, box, shape
from shapely.ops import transform

sys.path.insert(0, ".")
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops.shp_to_xodr import _proj  # noqa: E402

LAYERS = ["IBD_OBJECT_INTERSECTION_SURFACE", "IBD_OBJECT_ROAD_SURFACE", "IBD_OBJECT_DIVERSION",
          "IBD_OBJECT_ISOLATION_BELTS", "IBD_OBJECT_CURB", "IBD_OBJECT_TRAFFIC_BARRIER", "IBD_OBJECT_ZEBRA",
          "IBD_OBJECT_STOPLINE", "IBD_OBJECT_CROSS_BIKE", "IBD_OBJECT_WALL", "IBD_LANE_BOUNDARY", "IBD_LANE_LINK",
          "IBD_ROADCENTER", "IBD_OBJECT_ARROW", "IBD_OBJECT_SYMBOL"]


def main(diag_json, pid, out_json):
    ev = json.loads(open(diag_json, encoding="utf-8").read())
    holes = ev.get("holes") or ev.get("holes_base_union_family")
    target = wkt.loads(holes[0]["wkt"])
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == pid)
    lat0, lon0 = float(junc.center[1]), float(junc.center[0])

    def project(x, y, z=None):
        p = _proj(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)]), lat0, lon0)
        return p[:, 0], p[:, 1]

    zone = target.buffer(1.0)
    report = {"pid": pid, "target_area_m2": target.area, "target_bounds": list(target.bounds), "layers": {}}
    for name in LAYERS:
        rd = shapefile.Reader(f"shp_0222-0326/{name}.shp", encoding="gbk")
        fields = [f[0] for f in rd.fields[1:]]
        rows = []
        for sr in rd.iterShapeRecords():
            if not sr.shape.points:
                continue
            b = sr.shape.bbox
            if b[2] < lon0 - .002 or b[0] > lon0 + .002 or b[3] < lat0 - .002 or b[1] > lat0 + .002:
                continue
            try:
                g = transform(project, shape(sr.shape.__geo_interface__))
            except Exception:
                continue
            if not g.is_valid:
                g = g.buffer(0) if g.geom_type.endswith("Polygon") else g
            if not g.intersects(zone):
                continue
            rec = dict(zip(fields, sr.record))
            row = {"fields": {k: (v if isinstance(v, (int, float, str)) else str(v)) for k, v in rec.items()},
                   "type": g.geom_type, "distance_to_target_m": float(g.distance(target))}
            if g.geom_type.endswith("Polygon"):
                row["overlap_with_target_m2"] = float(g.intersection(target).area)
                row["target_fraction_covered"] = float(g.intersection(target).area / target.area)
            else:
                row["length_inside_target_m"] = float(g.intersection(target).length)
            rows.append(row)
        report["layers"][name] = rows
        print(name, len(rows))
        for r in rows:
            f = r["fields"]
            key = {k: f[k] for k in list(f)[:1]}
            extra = {k: f[k] for k in ("BORDERTYPE", "MARK_COUNT", "TRAVERSAL", "ADJACENCY", "TYPE", "LINE_TYPE",
                                       "SIDE", "HEIGHT", "IS_BLOCKED", "MATERIAL", "LINK_PID", "SEQ_NUM", "LANE_TYPE",
                                       "ROAD_PID", "STOP_TYPE") if k in f}
            print("   ", key, extra, {k: round(v, 3) for k, v in r.items() if isinstance(v, float)})
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:4])
