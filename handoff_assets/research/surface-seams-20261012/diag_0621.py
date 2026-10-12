"""0621 (2023062110304177600) road 10: why envelope_surface raises 'unresolved gap inside source surface family'.

Read-only: recomputes exactly what envelope_surface.append_source_paving does for road 10 from the failed raw
conversion (raw.xodr + raw.mouth-envelope.json), logs the median tracker and the family cross-sections in the
road-10 mouth frame (s along the mouth heading into the junction, t to its left), and draws the geometry.

    cd <tree>; <python> <this> <run_dir> <out_dir>
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point, Polygon, mapping
from shapely.ops import unary_union

sys.path.insert(0, ".")
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import envelope_surface as E  # noqa: E402
from mapforge.ops import junction_surface as JS  # noqa: E402
from mapforge.ops.shp_to_xodr import _proj  # noqa: E402

PID = "2023062110304177600"
ROAD = "10"


def frame(spec):
    o = np.asarray(spec["pose"][:2], float)
    a = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    n = np.array([-a[1], a[0]])
    return o, a, n


def st(xy, o, a, n):
    xy = np.asarray(xy, float)
    return np.column_stack([(xy - o) @ a, (xy - o) @ n])


def intervals(geometry, p, normal):
    cut = geometry.intersection(LineString([p - 100 * normal, p + 100 * normal]))
    out = []
    for line in getattr(cut, "geoms", [cut]):
        if line.geom_type == "LineString" and not line.is_empty:
            t = (np.asarray(line.coords) - p) @ normal
            out.append((float(t.min()), float(t.max())))
    return sorted(out)


def main(run_dir, out_dir):
    run_dir, out_dir = Path(run_dir), Path(out_dir)
    record = json.loads((run_dir / f"shp-{PID}.raw.mouth-envelope.json").read_text(encoding="utf-8"))
    root = ET.parse(run_dir / f"shp-{PID}.raw.xodr").getroot()
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == PID)
    project = lambda p: _proj(p, float(junc.center[1]), float(junc.center[0]))  # noqa: E731
    mouths = JS.written_mouth_specs(root, record["stats"]["mouth_envelope_decisions"])
    base, pieces, _u, summary = JS.source_tail_regions(src, junc, project, mouths)
    cover = base.buffer(E.COVER_TOL_M)
    tails, issues = E._bridge_tails(base, cover, pieces, mouths)
    medians, median_issues, inside = E._median_tails(base, cover, tails, mouths)
    spec = next(m for m in mouths if m["road_id"] == ROAD)
    o, a, n = frame(spec)
    ev = {"junction": PID, "road": ROAD, "mouth_pose": spec["pose"], "median_interval_at_mouth": spec.get("median_interval"),
          "enter_link": spec["enter_link"], "leave_links": spec.get("leave_links"),
          "bridge_issues": issues, "median_issues": median_issues, "median_inside": sorted(inside),
          "base_area_m2": base.area, "base_holes": len(base.interiors)}
    # ---- raw source lane pieces of road 10 (s,t extents) ----
    own = [p for p in pieces if p["road_id"] == ROAD]
    ev["pieces"] = []
    for p in own:
        lane = src.lane(p["source_lane"])
        c = st(np.asarray(p["geometry"].exterior.coords), o, a, n)
        ev["pieces"].append({"source_lane": p["source_lane"], "link": lane.link_pid,
                             "role": "enter" if lane.link_pid == spec["enter_link"] else "leave",
                             "s_range": [float(c[:, 0].min()), float(c[:, 0].max())],
                             "t_range": [float(c[:, 1].min()), float(c[:, 1].max())],
                             "area_m2": p["geometry"].area, "gap_to_base_m": p["gap_to_junction_m"]})
    own_tails = [t for t in tails if t["road_id"] == ROAD]
    ev["tails"] = [{k: v for k, v in t.items() if k not in ("geometry", "axis")} |
                   {"s_max": float(st(np.asarray(t["geometry"].exterior.coords), o, a, n)[:, 0].max()),
                    "area_m2": t["geometry"].area} for t in own_tails]
    own_med = [m for m in medians if m["road_id"] == ROAD]
    ev["median_islands"] = [{"area_m2": m["geometry"].area,
                             "s_range": [float(st(np.asarray(m["geometry"].exterior.coords), o, a, n)[:, 0].min()),
                                         float(st(np.asarray(m["geometry"].exterior.coords), o, a, n)[:, 0].max())]}
                            for m in own_med]
    # ---- median tracker, replicated with a log (E._median_tails) ----
    interval = spec["median_interval"]
    geometry = unary_union([t["geometry"] for t in tails if t["road_id"] == ROAD])
    max_s = max(float(((np.asarray(p.exterior.coords) - o) @ a).max()) for p in JS._polygons(geometry))
    previous = sum(interval) / 2
    log, rows = [], []
    for s in np.linspace(-.5, max_s, max(3, int((max_s + .5) / .25) + 2)):
        point = o + s * a
        ranges = intervals(geometry, point, n)
        gaps = [(x[1], y[0]) for x, y in zip(ranges[:-1], ranges[1:])
                if y[0] - x[1] > .01 and x[1] - .5 <= previous <= y[0] + .5]
        base_iv = intervals(base, point, n)
        entry = {"s": float(s), "tail_ranges": ranges, "gaps_considered": gaps, "previous_mid": previous,
                 "base_ranges": base_iv}
        if len(gaps) != 1:
            entry["stop"] = "not exactly one gap"
            log.append(entry)
            if not rows and s <= 1.:
                continue
            break
        lo, hi = gaps[0]
        rows.append((point + n * lo, point + n * hi))
        previous = (lo + hi) / 2
        entry["chosen"] = [lo, hi]
        covered = base.covers(LineString([rows[-1][0], rows[-1][1]]))
        entry["row_covered_by_base"] = bool(covered)
        log.append(entry)
        if covered:
            break
    ev["median_tracker_log"] = log
    # ---- the family exactly as append_source_paving builds it ----
    supported = [x["geometry"] for x in own_tails]
    islands = [x["geometry"] for x in own_med]
    parts = JS._polygons(unary_union(supported + islands))
    ev["family_parts"] = len(parts)
    part, filled = E._fill_slivers(parts[0])
    caps = [JS.directional_sweep(part, a, E.CAP_M).intersection(base)] if not part.interiors else []
    family = unary_union([part, *caps])
    ev["family_part_holes"] = len(part.interiors)
    coords = np.vstack([np.asarray(p.exterior.coords) for p in JS._polygons(family)])
    s_all = (coords - o) @ a
    s0, s1 = float(s_all.min()) - .25, float(s_all.max()) + .25
    us = np.linspace(0., s1 - s0, max(3, int(s1 - s0) + 2))
    cross = []
    first_fail = None
    for u in us:
        s = float(u + s0)
        p = o + s * a
        ranges = intervals(family, p, n)
        gaps = []
        for x, y in zip(ranges[:-1], ranges[1:]):
            if y[0] - x[1] > .02:
                cov = E._covered(cover, p, n, x[1], y[0])
                seg = LineString([p + x[1] * n, p + y[0] * n])
                gaps.append({"t": [x[1], y[0]], "width_m": y[0] - x[1], "covered_by_base_5cm": bool(cov),
                             "uncovered_width_m": float(seg.difference(cover).length)})
                if not cov and first_fail is None:
                    first_fail = {"s": s, "ranges": ranges, "gap": gaps[-1]}
        cross.append({"s": s, "family": ranges, "base": intervals(base, p, n), "median": intervals(unary_union(islands), p, n)
                      if islands else [], "tails": intervals(unary_union(supported), p, n), "gaps": gaps})
    ev["family_cross_sections"] = cross
    ev["first_failure"] = first_fail
    # ---- the unpaved pocket: polygon between family and base around the failure ----
    allsurf = unary_union([base, family])
    holes = [Polygon(r) for poly in JS._polygons(allsurf) for r in poly.interiors]
    ev["holes_base_union_family"] = [{"area_m2": h.area, "centroid_st": st(np.asarray(h.centroid.coords), o, a, n)[0].tolist(),
                                      "s_range": [float(st(np.asarray(h.exterior.coords), o, a, n)[:, 0].min()),
                                                  float(st(np.asarray(h.exterior.coords), o, a, n)[:, 0].max())],
                                      "t_range": [float(st(np.asarray(h.exterior.coords), o, a, n)[:, 1].min()),
                                                  float(st(np.asarray(h.exterior.coords), o, a, n)[:, 1].max())],
                                      "wkt": h.wkt} for h in holes]
    # base boundary near the pocket, in (s,t)
    ring = st(np.asarray(base.exterior.coords), o, a, n)
    near = ring[(ring[:, 0] > -3) & (ring[:, 0] < 15) & (ring[:, 1] > -6) & (ring[:, 1] < 14)]
    ev["base_vertices_near_mouth_st"] = near.tolist()
    (out_dir / "0621-road10-diagnosis.json").write_text(json.dumps(ev, indent=1, default=float), encoding="utf-8")
    print(json.dumps({k: ev[k] for k in ("median_interval_at_mouth", "bridge_issues", "median_issues", "first_failure",
                                         "family_parts", "family_part_holes")}, indent=1, default=float))
    print("pieces", json.dumps(ev["pieces"], indent=1, default=float))
    print("median islands", ev["median_islands"])
    for e in log:
        print("  trk s=%.3f ranges=%s gaps=%s chosen=%s cov=%s stop=%s" % (
            e["s"], [tuple(round(v, 3) for v in r) for r in e["tail_ranges"]],
            [tuple(round(v, 3) for v in g) for g in e["gaps_considered"]], e.get("chosen"),
            e.get("row_covered_by_base"), e.get("stop")))
    for h in ev["holes_base_union_family"]:
        print("hole", {k: v for k, v in h.items() if k != "wkt"})
    # ---- plot ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(20, 10))
    for ax, (xl, yl) in zip(axs, [((-6, 30), (-20, 20)), ((-1, 12), (-5, 13))]):
        def fill(g, **kw):
            for poly in JS._polygons(g):
                c = st(np.asarray(poly.exterior.coords), o, a, n)
                ax.fill(c[:, 0], c[:, 1], **kw)
                for r in poly.interiors:
                    c = st(np.asarray(r.coords), o, a, n)
                    ax.plot(c[:, 0], c[:, 1], "r-", lw=2)
        fill(base, color="0.82", label="source INTERSECTION polygon")
        fill(family, color="tab:blue", alpha=.18, label="road-10 family (tails+median+caps)")
        for i, p in enumerate(own):
            lane = src.lane(p["source_lane"])
            col = "tab:green" if lane.link_pid == spec["enter_link"] else "tab:orange"
            c = st(np.asarray(p["geometry"].exterior.coords), o, a, n)
            ax.plot(c[:, 0], c[:, 1], "-", color=col, lw=1.2)
            ax.text(c[:, 0].max(), c[:, 1].mean(), p["source_lane"][-5:], color=col, fontsize=7)
        for m in own_med:
            fill(m["geometry"], color="tab:purple", alpha=.5)
        for h in holes:
            c = st(np.asarray(h.exterior.coords), o, a, n)
            ax.fill(c[:, 0], c[:, 1], color="red", alpha=.35)
        rowsxy = [(st(np.asarray([r[0]]), o, a, n)[0], st(np.asarray([r[1]]), o, a, n)[0]) for r in rows]
        for r0, r1 in rowsxy:
            ax.plot([r0[0], r1[0]], [r0[1], r1[1]], "m-", lw=.6)
        if first_fail:
            s = first_fail["s"]
            ax.axvline(s, color="r", ls="--", lw=.8)
            g = first_fail["gap"]["t"]
            ax.plot([s, s], g, "r-", lw=4)
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect("equal")
        ax.grid(alpha=.3)
        ax.set_xlabel("s along road-10 mouth heading (m)")
        ax.set_ylabel("t (left of heading, m)")
    axs[0].set_title("0621 road 10: green=enter lanes, orange=leave lanes, purple=median island, red=unpaved pocket")
    axs[1].set_title("zoom: red bar = failing cross-section s=%.3f" % (first_fail["s"] if first_fail else float("nan")))
    fig.tight_layout()
    fig.savefig(out_dir / "0621-road10-gap.png", dpi=80)
    print("wrote", out_dir / "0621-road10-gap.png")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
