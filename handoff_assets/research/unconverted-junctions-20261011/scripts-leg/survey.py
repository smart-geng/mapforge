"""Survey (read-only): does each enter/leave ROADLINK's ROADCENTER run along its own lanes?

For the 7 scoreboard SHP junctions and the 16 generalization junctions, per enter/leave link:
  * off_frac / off_max: share of the ROADCENTER (0.5 m samples) lying more than 0.5 m outside the link's lane corridor
    (union of its lanes buffered by half their width), and the largest distance outside;
  * head_max: largest heading difference ROADCENTER vs the link's lane heading field where the field exists;
  * shared_start: another enter road of the same junction starts at the same S_NODE (diverge) / leave road ends at same
    E_NODE (merge).
Writes a CSV and prints the links that would be flagged.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np
import shapefile
import yaml
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

TREE = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/diag-leg-tree")
sys.path.insert(0, str(TREE))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml  # noqa: E402
from mapforge.ops import leg_fit_fallback as LF  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402
from mapforge.validate.scoreboard import CASES  # noqa: E402

SHP = TREE / "shp_0222-0326"


def link_attrs():
    r = shapefile.Reader(str(SHP / "IBD_ROADLINK"), encoding="gbk")
    fields = [f[0] for f in r.fields[1:]]
    out = {}
    for rec in r.iterRecords():
        m = dict(zip(fields, rec))
        out[str(m["LINK_PID"]).strip()] = {"S": str(m["S_NODE_PID"]).strip(), "E": str(m["E_NODE_PID"]).strip()}
    r = shapefile.Reader(str(SHP / "IBD_LINK_FORM"), encoding="gbk")
    fields = [f[0] for f in r.fields[1:]]
    for rec in r.iterRecords():
        m = dict(zip(fields, rec))
        out.setdefault(str(m["LINK_PID"]).strip(), {}).setdefault("forms", []).append(str(m["ROAD_FORM"]).strip())
    return out


def resample(p, step=0.5):
    p = np.asarray(p, float)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    if s[-1] <= 0:
        return p[:1]
    q = np.linspace(0, s[-1], max(2, int(s[-1] / step) + 1))
    return np.column_stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])])


def measure(src, pid, proj):
    c = src.roadcenters.get(pid)
    lanes = [l for l in src.lanes_of(pid) if len(l.geometry) >= 2]
    if c is None or len(c) < 2 or not lanes:
        return None
    g = proj(c)
    polys = []
    lines = []
    for l in lanes:
        lg = proj(l.geometry)
        w = max(l.width_mm, l.s_width_mm, l.e_width_mm, 2000) / 1000.0
        if len(lg) >= 2 and LineString(lg).length > 0.2:
            polys.append(LineString(lg).buffer(w / 2, cap_style=2))
        lines.append(lg)
        for b in src.lane_boundary_geometries(l.lane_pid):
            if len(b) >= 2:
                lines.append(proj(b))
    if not polys:
        return None
    corridor = unary_union(polys)
    q = resample(g)
    dist = np.array([corridor.distance(Point(x, y)) for x, y in q])
    off = dist > 0.5
    grid, heading, base = LF.heading_field(g, lines)
    ok = np.isfinite(heading)
    dh = np.degrees(np.abs(np.angle(np.exp(1j * (heading[ok] - base[ok]))))) if ok.any() else np.array([np.nan])
    return {"centre_len": round(float(LineString(g).length), 2), "off_frac": round(float(off.mean()), 3),
            "off_max": round(float(dist.max()), 2), "field_cov": round(float(ok.mean()), 3),
            "head_max": round(float(np.nanmax(dh)), 1), "head_p95": round(float(np.nanpercentile(dh, 95)), 1),
            "lanes": len(lanes)}


def main():
    out_csv = Path(sys.argv[1])
    src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
    attrs = link_attrs()
    juncs = []
    for label, xml in CASES:
        ref = parse_map_xml(str(TREE / "v2x_map_xml" / xml))
        j, _d = src.find_junction(ref.ref_lon, ref.ref_lat)
        juncs.append(("scoreboard:" + label, j))
    gen = yaml.safe_load((TREE / "profiles/validation/generalization-set-v1.yaml").read_text(encoding="utf-8"))
    by_pid = {j.pid: j for j in src.junctions}
    for row in gen["junctions"]:
        juncs.append(("generalization:" + row["kind"], by_pid[str(row["pid"])]))
    rows = []
    for label, j in juncs:
        lon0, lat0 = float(j.center[0]), float(j.center[1])
        proj = lambda p, lat0=lat0, lon0=lon0: S._proj(p, lat0, lon0)  # noqa: E731
        starts = {}
        ends = {}
        for p in j.enter_roads:
            starts.setdefault(attrs.get(p, {}).get("S"), []).append(p)
        for p in j.leave_roads:
            ends.setdefault(attrs.get(p, {}).get("E"), []).append(p)
        for role, lst in (("enter", j.enter_roads), ("leave", j.leave_roads)):
            for p in lst:
                m = measure(src, p, proj)
                a = attrs.get(p, {})
                shared = (len(starts.get(a.get("S"), [])) > 1) if role == "enter" else (len(ends.get(a.get("E"), [])) > 1)
                row = {"set": label, "junction": j.pid, "role": role, "link": p, "forms": "/".join(a.get("forms", [])),
                       "shared_node": shared, **(m or {"centre_len": None})}
                rows.append(row)
    keys = ["set", "junction", "role", "link", "forms", "shared_node", "lanes", "centre_len", "off_frac", "off_max",
            "field_cov", "head_p95", "head_max"]
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})
    print(f"{len(rows)} links written to {out_csv}")
    for r in rows:
        if r.get("off_frac") is not None and (r["off_frac"] > 0.1 or r["head_max"] > 20):
            print({k: r.get(k) for k in keys})


if __name__ == "__main__":
    main()
