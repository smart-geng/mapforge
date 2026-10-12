"""Read-only figure: the two seams with the independent raw source objects (ROAD_SURFACE, TRAFFIC_BARRIER, via lanes).

    cd <tree>; <python> <this> <run_dir> <out_dir>
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from shapely import wkt
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, ".")
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import envelope_surface as E, junction_surface as JS, surface_seams as S  # noqa: E402
from mapforge.ops.shp_leave_mouth import _source_id  # noqa: E402
from mapforge.ops.shp_to_xodr import _proj  # noqa: E402
from mapforge.validate.smoothness import sample_road_ref  # noqa: E402


def main(run_dir, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    run_dir, out_dir = Path(run_dir), Path(out_dir)
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    fig, axs = plt.subplots(1, 2, figsize=(22, 10))
    cases = [("2023062110304177600", "10", [], "0621-road10-diagnosis.json", "holes_base_union_family",
              ((-2, 18), (-6, 8))),
             ("2023063015350832057", "30", ["30", "31"], "0630-road30-diagnosis.json", "holes", ((0, 7), (-6, 6)))]
    for ax, (pid, road, moved, diag, key, (xl, yl)) in zip(axs, cases):
        rec = json.loads((run_dir / f"shp-{pid}.raw.mouth-envelope.json").read_text(encoding="utf-8"))
        root = ET.parse(run_dir / f"shp-{pid}.raw.xodr").getroot()
        junc = next(j for j in src.junctions if j.pid == pid)
        project = lambda p, j=junc: _proj(p, float(j.center[1]), float(j.center[0]))  # noqa: E731
        mouths = JS.written_mouth_specs(root, rec["stats"]["mouth_envelope_decisions"])
        for rid in moved:
            rd = root.find(f"road[@id='{rid}']")
            pts, _, hh = sample_road_ref(rd, 0.25)
            links = sorted({src.lane(_source_id(ln)).link_pid for ln in rd.iter("lane")
                            if _source_id(ln) and src.lane(_source_id(ln)) is not None})
            mouths.append({"road_id": rid, "enter_link": links[0], "leave_links": links[1:],
                           "pose": [float(pts[0][0]), float(pts[0][1]), float(hh[0]) + math.pi]})
        base, pieces, _u, _s = JS.source_tail_regions(src, junc, project, mouths)
        spec = next(m for m in mouths if m["road_id"] == road)
        o = np.asarray(spec["pose"][:2])
        a = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        n = np.array([-a[1], a[0]])

        def st(xy):
            xy = np.asarray(xy, float)
            return (xy - o) @ a, (xy - o) @ n

        def poly(g, **kw):
            for p in JS._polygons(g):
                ax.fill(*st(p.exterior.coords), **kw)

        frame = unary_union([base, *[p["geometry"] for p in pieces]]).buffer(30)
        rs = S._source_objects(src, junc, project, S.LAYER_ROAD_SURFACE)
        for i, g in rs:
            if g.intersects(frame) and g.geom_type.endswith("Polygon"):
                for p in JS._polygons(g):
                    ax.fill(*st(p.exterior.coords), facecolor="none", edgecolor="tab:green", hatch="///", lw=.5)
        poly(base, color="0.8", alpha=.8)
        for p in pieces:
            lane = src.lane(p["source_lane"])
            own = p["road_id"] == road
            col = "tab:orange" if own and lane.link_pid != spec["enter_link"] else ("tab:blue" if own else "0.4")
            ax.plot(*st(p["geometry"].exterior.coords), "-", color=col, lw=1.4 if own else .6)
        hole = wkt.loads(json.loads((out_dir / diag).read_text(encoding="utf-8"))[key][0]["wkt"])
        ax.fill(*st(hole.exterior.coords), color="red", alpha=.55)
        for i, g in S._source_objects(src, junc, project, S.LAYER_BARRIER):
            if g.intersects(frame):
                for line in getattr(g, "geoms", [g]):
                    ax.plot(*st(line.coords), "k-", lw=2.5)
                    c = np.asarray(line.coords)
                    ax.text(*[v[len(c) // 2] for v in st(c)], f"TRAFFIC_BARRIER {i[-6:]}", fontsize=8)
        # junction-interior source lanes (via lines) touching the problem area
        for lp, lanes in src._lanes_by_link.items() if src._lanes_by_link else []:
            for ln in lanes:
                if ln.geometry.shape[0] < 2:
                    continue
                xy = project(ln.geometry)
                from shapely.geometry import LineString
                line = LineString(xy)
                if line.distance(hole) < .3:
                    ax.plot(*st(xy), "-", color="tab:purple", lw=.9)
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect("equal")
        ax.grid(alpha=.3)
        ax.set_xlabel(f"s along road-{road} mouth heading (m)")
        ax.set_ylabel("t (m)")
    axs[0].set_title("0621 road 10: red = unpaved pocket (9.741 m2); hatched green = raw ROAD_SURFACE;\n"
                     "black = raw TRAFFIC_BARRIER (one object through the accepted median and the pocket); "
                     "blue/orange = enter/leave tails", fontsize=10)
    axs[1].set_title("0630 road 30: red = enclosed hole (0.2006 m2) left by the 1.5 m sweep; hatched green = raw "
                     "ROAD_SURFACE (covers 98.5 %);\npurple = raw via lanes crossing it; orange = road-30 tails",
                     fontsize=10)
    fig.tight_layout()
    fig.savefig(out_dir / "source-evidence.png", dpi=80)
    print("wrote", out_dir / "source-evidence.png")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
