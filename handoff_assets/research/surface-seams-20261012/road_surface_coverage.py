"""Read-only: how much of each paving region is covered by raw IBD_OBJECT_ROAD_SURFACE polygons?

For both junctions: the source junction polygon, every raw lane tail piece of every mouth, the 0621 pocket and the
0630 hole (WKT from the diagnosis JSONs). Shows whether ROAD_SURFACE is a usable, independent source witness of
"paved" vs "not paved" here.

    cd <tree>; <python> <this> <diag_dir> <run_dir> <out_json>
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import shapefile
from shapely import wkt
from shapely.geometry import shape
from shapely.ops import transform, unary_union

sys.path.insert(0, ".")
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import junction_surface as JS  # noqa: E402
from mapforge.ops.shp_to_xodr import _proj  # noqa: E402

CASES = {"2023062110304177600": ("0621-road10-diagnosis.json", "holes_base_union_family", []),
         "2023063015350832057": ("0630-road30-diagnosis.json", "holes", ["30", "31"])}


def surfaces(pid, src):
    junc = next(j for j in src.junctions if j.pid == pid)
    lat0, lon0 = float(junc.center[1]), float(junc.center[0])

    def project(x, y, z=None):
        p = _proj(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)]), lat0, lon0)
        return p[:, 0], p[:, 1]
    rd = shapefile.Reader("shp_0222-0326/IBD_OBJECT_ROAD_SURFACE.shp", encoding="gbk")
    polys = []
    for sr in rd.iterShapeRecords():
        b = sr.shape.bbox
        if not sr.shape.points or b[2] < lon0 - .002 or b[0] > lon0 + .002 or b[3] < lat0 - .002 or b[1] > lat0 + .002:
            continue
        g = transform(project, shape(sr.shape.__geo_interface__))
        polys.append(g if g.is_valid else g.buffer(0))
    return junc, lambda p: _proj(p, lat0, lon0), unary_union(polys), len(polys)


def main(diag_dir, run_dir, out_json):
    from mapforge.ops.shp_leave_mouth import _source_id
    from mapforge.validate.smoothness import sample_road_ref
    diag_dir, run_dir = Path(diag_dir), Path(run_dir)
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    out = {}
    for pid, (diag, key, moved) in CASES.items():
        junc, project, rs, count = surfaces(pid, src)
        record = json.loads((run_dir / f"shp-{pid}.raw.mouth-envelope.json").read_text(encoding="utf-8"))
        root = ET.parse(run_dir / f"shp-{pid}.raw.xodr").getroot()
        mouths = JS.written_mouth_specs(root, record["stats"]["mouth_envelope_decisions"])
        for rid in moved:
            road = root.find(f"road[@id='{rid}']")
            pts, _, hh = sample_road_ref(road, 0.25)
            links = sorted({src.lane(_source_id(ln)).link_pid for ln in road.iter("lane")
                            if _source_id(ln) and src.lane(_source_id(ln)) is not None})
            mouths.append({"road_id": rid, "enter_link": links[0], "leave_links": links[1:],
                           "pose": [float(pts[0][0]), float(pts[0][1]), float(hh[0]) + math.pi]})
        base, pieces, _u, _s = JS.source_tail_regions(src, junc, project, mouths)
        problem = wkt.loads(json.loads((diag_dir / diag).read_text(encoding="utf-8"))[key][0]["wkt"])
        row = {"road_surface_objects_within_200m": count,
               "junction_polygon_area_m2": base.area,
               "junction_polygon_covered_by_road_surface": base.intersection(rs).area / base.area,
               "problem_area_m2": problem.area,
               "problem_covered_by_road_surface": problem.intersection(rs).area / problem.area,
               "problem_covered_by_road_surface_grown_5cm": problem.intersection(rs.buffer(.05)).area / problem.area,
               "tails": []}
        for p in pieces:
            g = p["geometry"]
            row["tails"].append({"road_id": p["road_id"], "source_lane": p["source_lane"], "area_m2": g.area,
                                 "covered_by_road_surface": g.intersection(rs).area / g.area})
        cov = [t["covered_by_road_surface"] for t in row["tails"]]
        row["tail_coverage_min_median_max"] = [min(cov), float(np.median(cov)), max(cov)]
        out[pid] = row
        print(pid, json.dumps({k: v for k, v in row.items() if k != "tails"}, indent=1))
    Path(out_json).write_text(json.dumps(out, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main(*sys.argv[1:4])
