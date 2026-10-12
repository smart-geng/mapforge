"""Instrumented reproduction of the 'source-envelope mouth would remove the seed road' failure.

Runs leg_fit_fallback.build exactly as shp_mouth_envelope.convert does, with read-only hooks on
shp_to_xodr._chained_links / _source_envelope_axis (monkeypatched in this process only), and dumps the
failing leg: chain (ids, lengths, directions), mouth points per lane, computed mouth station, the seed link's
extent in the same station frame and where the source junction surface lies on the axis.

usage (cwd = experiment tree):  python diag_leg.py <lon> <lat> <out.json>
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.cwd()))
from shapely.geometry import LineString, Point, Polygon  # noqa: E402

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import leg_fit_fallback as LFF  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402


def polylen(g):
    g = np.asarray(g, float)
    return float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()) if len(g) > 1 else 0.0


def stations(g):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))]


def project_on(axis, pts):
    """station (extended linearly past both ends) and signed offset (left +) of pts on the polyline axis."""
    ls = LineString(axis)
    ss = stations(axis)
    out = []
    for p in np.atleast_2d(pts):
        s = ls.project(Point(p))
        if s <= 1e-9:
            t = axis[1] - axis[0]
            t /= np.linalg.norm(t)
            s = float((p - axis[0]) @ t)
            base, tv = axis[0], t
        elif s >= ss[-1] - 1e-9:
            t = axis[-1] - axis[-2]
            t /= np.linalg.norm(t)
            s = float(ss[-1] + (p - axis[-1]) @ t)
            base, tv = axis[-1], t
        else:
            q = ls.interpolate(s)
            i = int(np.searchsorted(ss, s) - 1)
            tv = axis[i + 1] - axis[i]
            tv /= np.linalg.norm(tv)
            base = np.array([q.x, q.y])
        d = float(tv[0] * (p[1] - base[1]) - tv[1] * (p[0] - base[0]))
        if 1e-9 < s < ss[-1] - 1e-9:
            out.append((float(s), d))
        else:
            out.append((float(s), d))
    return out


def main(lon, lat, out_json):
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    junc, dist = src.find_junction(lon, lat)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: S._proj(p, lat0, lon0)  # noqa: E731
    cj = proj(junc.polygon).mean(axis=0)
    poly = Polygon(proj(junc.polygon))
    pairs, single = S._pair_legs(src, junc, proj, cj)

    calls = {"chain": [], "axis": []}
    orig_chain, orig_axis = S._chained_links, S._source_envelope_axis

    def chain_hook(src_, seed_pid, is_enter, proj_, cj_, max_len=160.0, hops=5):
        ch = orig_chain(src_, seed_pid, is_enter, proj_, cj_, max_len=max_len, hops=hops)
        calls["chain"].append({"seed": seed_pid, "is_enter": is_enter, "max_len": max_len,
                               "links": [(p, polylen(proj_(c))) for p, c in (ch or [])]})
        return ch

    def axis_hook(points, mouth_points, *, margin_m=3.0, min_keep_m=6.0):
        enter = [c for c in calls["chain"] if c["is_enter"]][-1]
        rec = {"seed": enter["seed"], "points": np.asarray(points).tolist(),
               "mouth_points": np.asarray(mouth_points).tolist(), "margin_m": margin_m,
               "min_keep_m": min_keep_m}
        calls["axis"].append(rec)
        try:
            res = orig_axis(points, mouth_points, margin_m=margin_m, min_keep_m=min_keep_m)
            rec["ok"] = True
            rec["decision"] = res[1]
            return res
        except ValueError as exc:
            rec["ok"] = False
            rec["error"] = str(exc)
            raise

    S._chained_links, S._source_envelope_axis = chain_hook, axis_hook
    error = None
    try:
        LFF.build(src, junc, Path(out_json).with_suffix(".probe.xodr"), connect_mode="data", allow_uturn=False,
                  mouth_policy="source-envelope-candidate", mouth_margin_m=3.0)
    except Exception as exc:  # noqa: BLE001
        error = f"{type(exc).__name__}: {exc}"
    finally:
        S._chained_links, S._source_envelope_axis = orig_chain, orig_axis

    report = {"junction": junc.pid, "name": junc.name, "locator_distance_m": dist,
              "enter_roads": junc.enter_roads, "leave_roads": junc.leave_roads,
              "pairs": pairs, "single_leaves": single, "error": error,
              "centre_missing": {"enter": [p for p in junc.enter_roads if p not in src.roadcenters],
                                 "leave": [p for p in junc.leave_roads if p not in src.roadcenters]},
              "legs_ok": [{"seed": a["seed"], "decision": a.get("decision")} for a in calls["axis"] if a.get("ok")]}
    failing = [a for a in calls["axis"] if not a.get("ok")]
    if not failing:
        Path(out_json).write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
        print("no failing leg", error)
        return
    fa = failing[-1]
    e_pid = fa["seed"]
    l_pids = next((l for e, l in pairs if e == e_pid), [])
    enter_call = [c for c in calls["chain"] if c["is_enter"] and c["seed"] == e_pid][-1]
    g = np.asarray(fa["points"], float)
    ss = stations(g)
    L = float(ss[-1])
    dv = g[-1] - g[-2]
    dv /= np.linalg.norm(dv)
    links = enter_call["links"]  # upstream -> junction
    bounds = np.r_[0.0, np.cumsum([x[1] for x in links])]
    rl = src.roadlinks
    chain_rows = []
    for (pid, ln), a, b in zip(links, bounds[:-1], bounds[1:]):
        chain_rows.append({"link": pid, "name": rl[pid].name if pid in rl else None,
                           "lane_num": rl[pid].lane_num if pid in rl else None,
                           "has_centre": pid in src.roadcenters, "length_m": round(ln, 3),
                           "station_range": [round(float(a), 3), round(float(b), 3)],
                           "is_junction_enter_road": pid in junc.enter_roads,
                           "is_junction_leave_road": pid in junc.leave_roads,
                           "lanes": [{"lane": l.lane_pid, "seq": l.seq, "len_m": round(polylen(proj(l.geometry)), 3),
                                      "width_mm": l.width_mm, "s_w": l.s_width_mm, "e_w": l.e_width_mm,
                                      "type": l.lane_type}
                                     for l in src.lanes_of(pid)]})
    # every lane whose end entered the min(): seed lanes + opposite seed lanes, as in build_leg
    seed_lanes = [l for l in src.lanes_of(e_pid) if l.geometry.shape[0] >= 2]
    opp_lanes = [(lp, l) for lp in l_pids for l in src.lanes_of(lp) if l.geometry.shape[0] >= 2]
    mouth_rows = []
    for owner, l in [(e_pid, l) for l in seed_lanes] + opp_lanes:
        lg = proj(l.geometry)
        lp = lg[0] if np.linalg.norm(lg[0] - cj) < np.linalg.norm(lg[-1] - cj) else lg[-1]
        far = lg[-1] if lp is lg[0] else lg[0]
        s_near, d_near = project_on(g, lp[None])[0]
        s_far, d_far = project_on(g, far[None])[0]
        mouth_rows.append({"owner_link": owner, "side": "enter" if owner == e_pid else "leave",
                           "lane": l.lane_pid, "seq": l.seq, "len_m": round(polylen(lg), 3),
                           "end_proj_along_axis_end_dir_m": round(float((lp - g[-1]) @ dv), 3),
                           "junction_end_station": round(s_near, 3), "junction_end_offset": round(d_near, 3),
                           "far_end_station": round(s_far, 3), "far_end_offset": round(d_far, 3),
                           "junction_end_in_surface": bool(poly.buffer(0.05).contains(Point(lp))),
                           "junction_end_dist_to_surface_m": round(float(poly.exterior.distance(Point(lp))), 3)})
    deltas = [r["end_proj_along_axis_end_dir_m"] for r in mouth_rows]
    earliest = min(mouth_rows, key=lambda r: r["end_proj_along_axis_end_dir_m"])
    # where the axis meets the junction surface
    axis_ext = np.vstack([g, g[-1] + 60.0 * dv])
    inter = LineString(axis_ext).intersection(poly)
    surf_s = None
    if not inter.is_empty:
        pts = []
        geoms = getattr(inter, "geoms", [inter])
        for gg in geoms:
            pts.extend(list(gg.coords))
        sp = [project_on(g, np.array([p]))[0][0] for p in pts]
        surf_s = [round(min(sp), 3), round(max(sp), 3)]
    poly_s = [project_on(g, np.array([p]))[0] for p in np.asarray(poly.exterior.coords)]

    def at_station(s):
        if s <= L:
            return [float(np.interp(s, ss, g[:, 0])), float(np.interp(s, ss, g[:, 1]))]
        return (g[-1] + (s - L) * dv).tolist()

    mouth_station = L + min(deltas) - fa["margin_m"]
    el = next(l for l in seed_lanes + [l for _o, l in opp_lanes] if l.lane_pid == earliest["lane"])
    elg = proj(el.geometry)
    el_end = elg[0] if np.linalg.norm(elg[0] - cj) < np.linalg.norm(elg[-1] - cj) else elg[-1]
    report.update({"axis_xy": g.round(3).tolist(),
                   "upstream_enter_roads": [r["link"] for r in chain_rows[:-1] if r["is_junction_enter_road"]]})
    report.update({
        "failing_leg": {"enter_link": e_pid, "leave_links": l_pids, "error": fa["error"],
                        "axis_length_m": round(L, 3), "axis_points": len(g),
                        "chain_upstream_to_junction": chain_rows,
                        "seed_station_range": chain_rows[-1]["station_range"],
                        "min_keep_m": fa["min_keep_m"], "margin_m": fa["margin_m"],
                        "earliest_lane_end": earliest,
                        "mouth_station": round(mouth_station, 3),
                        "mouth_point_xy": at_station(max(mouth_station, 0.0)),
                        "min_keep_point_xy": at_station(fa["min_keep_m"]),
                        "earliest_end_xy": el_end.tolist(),
                        "axis_surface_station_range": surf_s,
                        "surface_polygon_station_range": [round(min(p[0] for p in poly_s), 3),
                                                          round(max(p[0] for p in poly_s), 3)],
                        "mouth_points": sorted(mouth_rows, key=lambda r: r["end_proj_along_axis_end_dir_m"])},
        "all_chain_calls": calls["chain"],
    })
    Path(out_json).write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    fl = report["failing_leg"]
    print(json.dumps({k: fl[k] for k in ("enter_link", "leave_links", "error", "axis_length_m", "seed_station_range",
                                         "mouth_station", "min_keep_m", "axis_surface_station_range")},
                     ensure_ascii=False, indent=1))
    print("earliest:", json.dumps(earliest, ensure_ascii=False))
    for r in chain_rows:
        print({k: r[k] for k in r if k != "lanes"})


if __name__ == "__main__":
    main(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3])
