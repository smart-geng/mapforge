"""0630 (2023063015350832057) road 30: why shp_leave_mouth.rebuild_paving rejects the tail with end-cap-encloses-gap.

Read-only. Uses the failed raw conversion (raw.xodr is written by shp_leave_mouth.apply before the paving rebuild, so
roads 30/31 are already cut there) and rebuilds the mouth list exactly as rebuild_paving does (the moved roads as
mouths of their own: pose at the road start, heading into the junction). Then replays envelope_surface._bridge_one
for the road-30 tail with a full log (end-cap edges, sample points, rays, misses), extracts the enclosed hole, and
compares the sweep direction with the real exit-boundary directions.

    cd <tree>; <python> <this> <run_dir> <out_dir>
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

sys.path.insert(0, ".")
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import envelope_surface as E  # noqa: E402
from mapforge.ops import junction_surface as JS  # noqa: E402
from mapforge.ops.shp_leave_mouth import _source_id  # noqa: E402
from mapforge.ops.shp_to_xodr import _proj  # noqa: E402
from mapforge.validate.smoothness import sample_road_ref  # noqa: E402

PID = "2023063015350832057"
MOVED = ["30", "31"]
ROAD = "30"


def mouths_of(root, src, decisions):
    mouths = JS.written_mouth_specs(root, decisions)
    for rid in MOVED:
        road = root.find(f"road[@id='{rid}']")
        pts, _, hh = sample_road_ref(road, 0.25)
        links = sorted({src.lane(_source_id(ln)).link_pid for ln in road.iter("lane")
                        if _source_id(ln) and src.lane(_source_id(ln)) is not None})
        mouths.append({"road_id": rid, "enter_link": links[0], "leave_links": links[1:],
                       "pose": [float(pts[0][0]), float(pts[0][1]), float(hh[0]) + math.pi],
                       "decision": "leave-mouth-shift"})
    return mouths


def st(xy, o, a, n):
    xy = np.asarray(xy, float).reshape(-1, 2)
    return np.column_stack([(xy - o) @ a, (xy - o) @ n])


def heading(v):
    return math.degrees(math.atan2(v[1], v[0]))


def main(run_dir, out_dir):
    run_dir, out_dir = Path(run_dir), Path(out_dir)
    record = json.loads((run_dir / f"shp-{PID}.raw.mouth-envelope.json").read_text(encoding="utf-8"))
    root = ET.parse(run_dir / f"shp-{PID}.raw.xodr").getroot()
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == PID)
    project = lambda p: _proj(p, float(junc.center[1]), float(junc.center[0]))  # noqa: E731
    mouths = mouths_of(root, src, record["stats"]["mouth_envelope_decisions"])
    base, pieces, _u, summary = JS.source_tail_regions(src, junc, project, mouths)
    cover = base.buffer(E.COVER_TOL_M)
    spec = next(m for m in mouths if m["road_id"] == ROAD)
    o = np.asarray(spec["pose"][:2], float)
    a = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    n = np.array([-a[1], a[0]])
    own = [p for p in pieces if p["road_id"] == ROAD]
    shape = unary_union([p["geometry"] for p in own])
    polys = JS._polygons(shape)
    ev = {"junction": PID, "road": ROAD, "mouth_pose": spec["pose"], "mouth_heading_deg": heading(a),
          "enter_link": spec["enter_link"], "leave_links": spec["leave_links"], "tail_parts": len(polys),
          "base_holes": len(base.interiors), "union_components": summary["components"]}
    ev["pieces"] = []
    for p in own:
        lane = src.lane(p["source_lane"])
        c = st(np.asarray(p["geometry"].exterior.coords), o, a, n)
        bounds = [project(b) for b in src.lane_boundary_geometries(p["source_lane"])]
        ev["pieces"].append({"source_lane": p["source_lane"], "link": lane.link_pid, "seq": lane.seq,
                             "s_range": [float(c[:, 0].min()), float(c[:, 0].max())],
                             "t_range": [float(c[:, 1].min()), float(c[:, 1].max())],
                             "area_m2": p["geometry"].area, "gap_to_base_m": p["gap_to_junction_m"],
                             # junction-side end of each source boundary: heading of its first 1-2 m
                             "boundary_ends_st": [st(b[0] if np.linalg.norm(b[0] - o) < np.linalg.norm(b[-1] - o)
                                                     else b[-1], o, a, n)[0].tolist() for b in bounds]})
    polygon = polys[0]
    # ---- replay of _bridge_one with a log ----
    coords = np.asarray(orient(polygon, sign=1).exterior.coords)
    caps, samples = [], []
    for a0, b0 in zip(coords[:-1], coords[1:]):
        edge = b0 - a0
        el = np.linalg.norm(edge)
        if el < 1e-8:
            continue
        facing = float(np.dot([edge[1], -edge[0]], a) / el)
        if facing < .5:
            continue
        caps.append({"from_st": st(a0, o, a, n)[0].tolist(), "to_st": st(b0, o, a, n)[0].tolist(), "length_m": float(el),
                     "outward_normal_dot_sweep": facing, "edge_heading_deg": heading(edge),
                     "angle_to_sweep_normal_deg": math.degrees(math.acos(min(1., facing)))})
        for w in np.linspace(0., 1., max(2, int(el / .25) + 2)):
            p = a0 + w * edge
            rec = {"st": st(p, o, a, n)[0].tolist()}
            if base.covers(Point(p)):
                rec.update(kind="inside", gap=0.)
            else:
                ray = LineString([p, p + a * 1.5])
                hit = base.intersection(ray)
                if hit.is_empty:
                    hit = cover.intersection(ray)
                    rec["via_cover"] = not hit.is_empty
                if hit.is_empty:
                    long_hit = base.intersection(LineString([p, p + a * 10.]))
                    rec.update(kind="miss", long_ray_gap=float(Point(p).distance(long_hit)) if not long_hit.is_empty else None,
                               ray_distance_to_base=float(LineString([p, p + a * 1.5]).distance(base)))
                else:
                    rec.update(kind="hit", gap=float(Point(p).distance(hit)))
            samples.append(rec)
    hits = [r["gap"] for r in samples if r["kind"] in ("hit", "inside")]
    misses = [r for r in samples if r["kind"] == "miss"]
    shift = min(1.5, max(hits) + .25) if hits and max(hits) > 1e-6 else 0.
    patched = JS.directional_sweep(polygon, a, shift) if shift else polygon
    joined = unary_union([patched, base])
    new_holes = [Polygon(r) for r in joined.interiors] if joined.geom_type == "Polygon" else []
    base_holes = [Polygon(r) for r in base.interiors]
    holes = [h for h in new_holes if not any(h.equals(b) for b in base_holes)]
    ev.update(end_cap_edges=caps, end_cap_points=len(samples), end_cap_misses=len(misses),
              miss_points=misses, forward_gaps_max_m=max(hits) if hits else None, sweep_m=shift,
              joined_type=joined.geom_type, joined_holes=len(new_holes))
    ev["holes"] = []
    for h in holes:
        c = st(np.asarray(h.exterior.coords), o, a, n)
        others = [{"road_id": p["road_id"], "source_lane": p["source_lane"], "overlap_m2": p["geometry"].intersection(h).area}
                  for p in pieces if p["geometry"].intersection(h).area > 0]
        ev["holes"].append({"area_m2": h.area, "s_range": [float(c[:, 0].min()), float(c[:, 0].max())],
                            "t_range": [float(c[:, 1].min()), float(c[:, 1].max())],
                            "centroid_st": st(np.asarray(h.centroid.coords), o, a, n)[0].tolist(),
                            "covered_by_any_source_tail_m2": float(unary_union([p["geometry"] for p in pieces]).intersection(h).area),
                            "other_tails": others, "wkt": h.wkt})
    # ---- sweep direction vs. real exit boundary direction ----
    # outer source boundaries of the departure lanes near their junction-side end
    dirs = []
    for p in own:
        for b in src.lane_boundary_geometries(p["source_lane"]):
            xy = project(b)
            if np.linalg.norm(xy[-1] - o) < np.linalg.norm(xy[0] - o):
                xy = xy[::-1]                         # start at the junction side
            s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
            k = int(np.searchsorted(s, 3.0))
            v = xy[0] - xy[min(k, len(xy) - 1)]        # pointing into the junction
            dirs.append({"source_lane": p["source_lane"], "end_st": st(xy[0], o, a, n)[0].tolist(),
                         "heading_into_junction_deg": heading(v),
                         "angle_to_sweep_deg": (heading(v) - heading(a) + 180) % 360 - 180})
    ev["boundary_directions"] = dirs
    # base (junction polygon) edges that face the tail cap
    ring = np.asarray(base.exterior.coords)
    near = []
    for p0, p1 in zip(ring[:-1], ring[1:]):
        seg = LineString([p0, p1])
        if seg.distance(polygon) < 2.5:
            near.append({"from_st": st(p0, o, a, n)[0].tolist(), "to_st": st(p1, o, a, n)[0].tolist(),
                         "heading_deg": heading(p1 - p0), "distance_to_tail_m": float(seg.distance(polygon))})
    ev["base_edges_near_tail"] = near
    (out_dir / "0630-road30-diagnosis.json").write_text(json.dumps(ev, indent=1, default=float), encoding="utf-8")
    print(json.dumps({k: ev[k] for k in ("mouth_heading_deg", "tail_parts", "end_cap_points", "end_cap_misses",
                                         "forward_gaps_max_m", "sweep_m", "joined_type", "joined_holes")}, indent=1))
    print("pieces", json.dumps(ev["pieces"], indent=1, default=float))
    print("caps", json.dumps(caps, indent=1))
    for m in misses:
        print("miss", m)
    print("holes", json.dumps([{k: v for k, v in h.items() if k != "wkt"} for h in ev["holes"]], indent=1))
    print("boundary dirs", json.dumps(dirs, indent=1))
    print("base edges", json.dumps(near, indent=1))
    # ---- plot ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(20, 10))
    for ax, (xl, yl) in zip(axs, [((-12, 12), (-14, 10)), ((-3, 4), (-6, 3))]):
        def fill(g, **kw):
            for poly in JS._polygons(g):
                c = st(np.asarray(poly.exterior.coords), o, a, n)
                ax.fill(c[:, 0], c[:, 1], **kw)
        fill(base, color="0.82")
        for p in pieces:
            c = st(np.asarray(p["geometry"].exterior.coords), o, a, n)
            ax.plot(c[:, 0], c[:, 1], "-", color="tab:orange" if p["road_id"] == ROAD else "tab:green", lw=1)
        fill(patched, color="tab:blue", alpha=.18)
        for h in holes:
            c = st(np.asarray(h.exterior.coords), o, a, n)
            ax.fill(c[:, 0], c[:, 1], color="red", alpha=.6)
        for r in samples:
            col = {"inside": "k", "hit": "tab:blue", "miss": "red"}[r["kind"]]
            ax.plot(r["st"][0], r["st"][1], ".", color=col, ms=5)
            if r["kind"] == "miss":
                ax.plot([r["st"][0], r["st"][0] + 1.5], [r["st"][1], r["st"][1]], "r-", lw=.6)
        for d in dirs:
            e = np.asarray(d["end_st"])
            ang = math.radians(d["angle_to_sweep_deg"])
            ax.arrow(e[0], e[1], 1.2 * math.cos(ang), 1.2 * math.sin(ang), color="purple", width=.01, head_width=.12)
        ax.arrow(0, 0, 1.5, 0, color="k", width=.02, head_width=.2)
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect("equal")
        ax.grid(alpha=.3)
        ax.set_xlabel("s along road-30 mouth heading (into junction, m)")
        ax.set_ylabel("t (m)")
    axs[0].set_title("0630 road 30: orange=road-30 source tails, blue=swept tail, red=enclosed hole, red dots=end-cap misses")
    axs[1].set_title("zoom: purple=source boundary directions at their junction-side end, black=sweep direction")
    fig.tight_layout()
    fig.savefig(out_dir / "0630-road30-hole.png", dpi=80)
    print("wrote", out_dir / "0630-road30-hole.png")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
