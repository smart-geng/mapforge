"""Why did the junction paving of one SHP conversion fail? (2026-10-08)

Reads ``<dir>/shp-<pid>.raw.xodr`` and its ``.raw.mouth-envelope.json`` (left by a failed default conversion or by
generator_stage.py), recomputes the lane tails, end caps and median tails with the rules of
mapforge.ops.envelope_surface, and prints per mouth: the written median, the tail parts, every end-cap point that does
not reach the source junction polygon within the 1.5 m budget (ray hit distance up to 10 m, or how far beside the
polygon the ray passes). ``--png`` draws polygon, tails, medians and mouths.

    .venv\\Scripts\\python scripts\\generalization\\surface_diag.py out\\generalize\\<name> <pid> [--png f.png]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", type=Path)
    ap.add_argument("pid")
    ap.add_argument("--png", type=Path)
    a = ap.parse_args(argv)
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.ops import envelope_surface as E
    from mapforge.ops import junction_surface as JS
    from mapforge.ops.shp_to_xodr import _proj
    record = json.loads((a.dir / f"shp-{a.pid}.raw.mouth-envelope.json").read_text(encoding="utf-8"))
    root = ET.parse(a.dir / f"shp-{a.pid}.raw.xodr").getroot()
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == a.pid)
    project = lambda p: _proj(p, float(junc.center[1]), float(junc.center[0]))  # noqa: E731
    mouths = JS.written_mouth_specs(root, record["stats"]["mouth_envelope_decisions"])
    base, pieces, _union, summary = JS.source_tail_regions(src, junc, project, mouths)
    cover = base.buffer(E.COVER_TOL_M)
    tails, issues = E._bridge_tails(base, cover, pieces, mouths)
    medians, median_issues, inside = E._median_tails(base, cover, tails, mouths)
    print("record status:", record.get("status"), "| surface error:", record.get("surface_error"))
    print("tail source issues:", summary["issues"])
    print("bridge issues:", issues)
    print("median issues:", median_issues, "| median tails inside the polygon:", sorted(inside))
    for spec in mouths:
        direction = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        print(f"mouth road {spec['road_id']}: enter {spec['enter_link']} leaves {spec.get('leave_links', [])} "
              f"pose {np.round(spec['pose'], 2).tolist()} median {spec.get('median_interval')}")
        shape = unary_union([p["geometry"] for p in pieces if p["road_id"] == spec["road_id"]])
        for k, polygon in enumerate(JS._polygons(shape)):
            far, beside, n = [], [], 0
            coords = np.asarray(orient(polygon, sign=1).exterior.coords)
            for p0, p1 in zip(coords[:-1], coords[1:]):
                edge = p1 - p0
                el = np.linalg.norm(edge)
                if el < 1e-8 or np.dot([edge[1], -edge[0]], direction) / el < .5:
                    continue
                for w in np.linspace(0., 1., max(2, int(el / .25) + 2)):
                    p = p0 + w * edge
                    n += 1
                    if base.covers(Point(p)):
                        continue
                    ray = LineString([p, p + direction * 10.0])
                    hit = base.intersection(ray)
                    if hit.is_empty:
                        beside.append(round(float(ray.distance(base)), 3))
                    elif Point(p).distance(hit) > 1.5:
                        far.append(round(float(Point(p).distance(hit)), 2))
            print(f"   tail part {k}: {n} end-cap points, beyond 1.5 m: {far}, beside the polygon by: {beside}")
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mapforge.validate.smoothness import sample_road_ref
        fig, ax = plt.subplots(figsize=(14, 14))
        ax.fill(*np.asarray(base.exterior.coords).T, color="0.85")
        for t in tails:
            for g in JS._polygons(t["geometry"]):
                ax.fill(*np.asarray(g.exterior.coords).T, color="tab:blue", alpha=.15)
        for p in pieces:
            for g in JS._polygons(p["geometry"]):
                ax.plot(*np.asarray(g.exterior.coords).T, "b-", lw=.6)
        for m in medians:
            ax.fill(*np.asarray(m["geometry"].exterior.coords).T, color="tab:green", alpha=.5)
        for rd in root.findall("road"):
            if rd.get("junction") in (None, "-1"):
                pts, _, _ = sample_road_ref(rd, .5)
                ax.plot(pts[:, 0], pts[:, 1], "k-", lw=1)
        for spec in mouths:
            p, h = np.array(spec["pose"][:2]), spec["pose"][2]
            nrm = np.array([-math.sin(h), math.cos(h)])
            ax.plot(*np.array([p - 12 * nrm, p + 12 * nrm]).T, "r-", lw=1.5)
            ax.text(*p, spec["road_id"], color="r")
        c = np.asarray(base.centroid.coords)[0]
        ax.set_xlim(c[0] - 70, c[0] + 70)
        ax.set_ylim(c[1] - 70, c[1] + 70)
        ax.set_aspect("equal")
        ax.grid(alpha=.3)
        fig.savefig(a.png, dpi=60)
        print("wrote", a.png)


if __name__ == "__main__":
    main()
