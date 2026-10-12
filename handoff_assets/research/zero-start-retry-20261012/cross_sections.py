"""Cross sections of one road: the written lane edges (t of each edge, lane ids) and where every source lane
boundary of the road's source lanes crosses the same normal line. cwd = code tree.
argv: xodr, road id, comma-separated stations, output json"""
import json, sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from shapely.geometry import LineString, Point
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import shp_boundary_fidelity as B
from mapforge.validate.smoothness import lane_edges_at, sample_road_ref

xodr, rid = Path(sys.argv[1]), sys.argv[2]
stations = [float(x) for x in sys.argv[3].split(",")]
SIDES = sys.argv[5] if len(sys.argv) > 5 else "right"
root = ET.parse(xodr).getroot()
road = next(r for r in root.findall("road") if r.get("id") == rid)
lat0, lon0 = B._origin(root)
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")
pts, ss, hh = sample_road_ref(road, 0.05)
secs = road.findall("lanes/laneSection")
sources = set()
for sec in secs:
    for lane in sec.iter("lane"):
        u = lane.find("userData[@code='mapforge.source_lane']")
        if u is not None:
            sources.add(u.get("value"))
bounds = {sid: [LineString(B._project(g, lat0, lon0)[:, :2]) for g in src.lane_boundary_geometries(sid)]
          for sid in sorted(sources)}
rows = []
for s in stations:
    i = int(np.argmin(abs(ss - s)))
    p, h = pts[i], hh[i]
    n = np.array([-np.sin(h), np.cos(h)])
    normal = LineString([p - 30 * n, p + 30 * n])
    k = max(j for j, sec in enumerate(secs) if float(sec.get("s")) <= s + 1e-9)
    sec = secs[k]
    written = []
    for side in (["left", "right"] if SIDES == "both" else ["right"]):
        lanes = sorted(sec.findall(f"{side}/lane"), key=lambda l: abs(int(l.get("id"))))
        edges = lane_edges_at(road, s, side=side)
        if side == "right" or not written:
            written.append({"t": round(float(edges[0]), 3), "between": "offset"})
        for j, lane in enumerate(lanes):
            u = lane.find("userData[@code='mapforge.source_lane']")
            written.append({"t": round(float(edges[j + 1]), 3), "outer_of": int(lane.get("id")), "type": lane.get("type"),
                            "source": u.get("value")[-5:] if u is not None else None,
                            "width": round(float(abs(edges[j + 1] - edges[j])), 3)})
    written.sort(key=lambda e: -e["t"])
    crossings = []
    for sid, lines in bounds.items():
        for gi, line in enumerate(lines):
            x = line.intersection(normal)
            for g in getattr(x, "geoms", [x]):
                if g.is_empty or g.geom_type != "Point":
                    continue
                t = float((np.array([g.x, g.y]) - p) @ n)
                crossings.append({"source": sid[-5:], "boundary": gi, "t": round(t, 3)})
    rows.append({"s": s, "section": k, "written_right_edges": written,
                 "source_boundaries": sorted(crossings, key=lambda c: -c["t"])})
Path(sys.argv[4]).write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
for r in rows:
    w = "  ".join(f"{e['t']:+.2f}" + (f"[{e['outer_of']}{'m' if e.get('type') == 'median' else ''}:{e['source']} w{e['width']:.2f}]" if "outer_of" in e else "[off]")
                  for e in r["written_right_edges"])
    c = "  ".join(f"{e['t']:+.2f}({e['source']}.{e['boundary']})" for e in r["source_boundaries"])
    print(f"s={r['s']:6.2f} sec{r['section']:2d} written: {w}\n            source : {c}")
