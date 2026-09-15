"""只读核对路面/隔离带/路缘石对象层，帮助区分真实分隔与生成露空。"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import shapefile
from shapely.geometry import shape, box, mapping
from shapely.ops import transform

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.validate.shp_boundary_fidelity import _origin, _project

LAYERS = ["ISOLATION_BELTS", "CURB", "ROAD_SURFACE", "DIVERSION", "TRAFFIC_BARRIER"]
COLORS = ["#239050", "#cc3333", "#aaaaaa", "#c89100", "#3355aa"]


def inspect(xodr, output):
    lat0, lon0 = _origin(ET.parse(xodr).getroot())

    def project(x, y, z=None):
        p = _project(np.column_stack([np.atleast_1d(x), np.atleast_1d(y)]), lat0, lon0)
        return p[:, 0], p[:, 1]

    view = box(-70, -70, 70, 70)
    evidence = {"artifact":str(xodr), "origin_lat_lon":[lat0, lon0], "layers":[]}
    fig, axes = plt.subplots(2, 3, figsize=(16, 11), dpi=150)
    for name, color, ax in zip(LAYERS, COLORS, axes.flat):
        path = ROOT / "shp_0222-0326" / f"IBD_OBJECT_{name}.shp"
        rd = shapefile.Reader(str(path), encoding="gbk")
        rows = []
        for rec in rd.iterShapeRecords():
            if not rec.shape.points:
                continue
            bounds = rec.shape.bbox
            if bounds[2] < lon0-.002 or bounds[0] > lon0+.002 or bounds[3] < lat0-.002 or bounds[1] > lat0+.002:
                continue
            pg = transform(project, shape(rec.shape.__geo_interface__))
            if not pg.is_valid or not pg.intersects(view):
                continue
            geom = pg.intersection(view)
            rows.append({"fields":rec.record.as_dict(), "type":geom.geom_type,
                         "geometry":mapping(geom), "area_m2":pg.area, "length_m":pg.length})
            for piece in getattr(geom, "geoms", [geom]):
                if piece.geom_type == "Polygon":
                    ax.fill(*piece.exterior.xy, color=color, alpha=.4)
                    for ring in piece.interiors:
                        ax.fill(*ring.xy, color="white")
                elif piece.geom_type == "LineString":
                    ax.plot(*piece.xy, color=color, lw=1)
                elif piece.geom_type == "Point":
                    ax.scatter(piece.x, piece.y, color=color, s=5)
        evidence["layers"].append({"name":name, "source_fields":[f[0] for f in rd.fields[1:]],
                                   "count":len(rows), "objects":rows})
        ax.set(title=f"{name} / {len(rows)} objects", xlabel="local x (m)", ylabel="local y (m)",
               xlim=(-70,70), ylim=(-70,70))
        ax.set_aspect("equal"); ax.grid(alpha=.2)
    axes.flat[-1].axis("off")
    fig.suptitle(f"{xodr.stem}: independent raw SHP object evidence (no generated fill)")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output)
    plt.close(fig)
    output.with_suffix(".json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2,
                                                     allow_nan=False), encoding="utf-8")
    print(json.dumps({"case":xodr.stem, "layers":{q["name"]:q["count"] for q in evidence["layers"]}}))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr", type=Path)
    ap.add_argument("output", type=Path)
    args = ap.parse_args()
    inspect(args.xodr, args.output)
