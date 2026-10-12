"""Read-only: ROAD_SURFACE / TRAFFIC_BARRIER coverage of the median islands the default builder accepts (0621, 0630)."""
import json, math, sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, shapefile
from shapely.geometry import shape
from shapely.ops import transform, unary_union
sys.path.insert(0, ".")
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops import envelope_surface as E, junction_surface as JS
from mapforge.ops.shp_to_xodr import _proj

def layer(src, name, lat0, lon0):
    def project(x, y, z=None):
        p = _proj(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)]), lat0, lon0)
        return p[:, 0], p[:, 1]
    rd = shapefile.Reader(f"shp_0222-0326/{name}.shp", encoding="gbk")
    out = []
    for sr in rd.iterShapeRecords():
        b = sr.shape.bbox
        if not sr.shape.points or b[2] < lon0-.002 or b[0] > lon0+.002 or b[3] < lat0-.002 or b[1] > lat0+.002:
            continue
        g = transform(project, shape(sr.shape.__geo_interface__))
        out.append(g if g.is_valid else g.buffer(0))
    return out

run = Path(sys.argv[1]); src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
res = {}
for pid in ["2023062110304177600", "2023063015350832057"]:
    rec = json.loads((run / f"shp-{pid}.raw.mouth-envelope.json").read_text(encoding="utf-8"))
    root = ET.parse(run / f"shp-{pid}.raw.xodr").getroot()
    junc = next(j for j in src.junctions if j.pid == pid)
    lat0, lon0 = float(junc.center[1]), float(junc.center[0])
    project = lambda p: _proj(p, lat0, lon0)
    mouths = JS.written_mouth_specs(root, rec["stats"]["mouth_envelope_decisions"])
    base, pieces, _, _ = JS.source_tail_regions(src, junc, project, mouths)
    cover = base.buffer(E.COVER_TOL_M)
    tails, _ = E._bridge_tails(base, cover, pieces, mouths)
    meds, _, _ = E._median_tails(base, cover, tails, mouths)
    rs = unary_union(layer(src, "IBD_OBJECT_ROAD_SURFACE", lat0, lon0))
    bar = layer(src, "IBD_OBJECT_TRAFFIC_BARRIER", lat0, lon0)
    rows = []
    for m in meds:
        g = m["geometry"]
        rows.append({"road_id": m["road_id"], "area_m2": g.area, "road_surface_cover": g.intersection(rs).area / g.area,
                     "barrier_length_inside_m": sum(b.intersection(g).length for b in bar)})
    res[pid] = rows
    print(pid, json.dumps(rows, indent=1))
Path(sys.argv[2]).write_text(json.dumps(res, indent=1))
