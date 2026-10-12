"""One road's outer-edge deviations, per section and sample, as the boundary audits pair them (boundary_inside:
nearest point inside the other line only). Also the road's lane layout and its lanes' provenance.
cwd = code tree. argv: xodr, road id, output json."""
import json, sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import shp_boundary_fidelity as B
from mapforge.validate.smoothness import lane_edges_at, sample_road_ref

xodr, rid = Path(sys.argv[1]), sys.argv[2]
root = ET.parse(xodr).getroot()
road = next(r for r in root.findall("road") if r.get("id") == rid)
lat0, lon0 = B._origin(root)
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")
out = {"road": rid, "length": float(road.get("length")), "sections": [], "outer_edges": []}

# lane layout
secs = road.findall("lanes/laneSection")
for k, sec in enumerate(secs):
    s0 = float(sec.get("s"))
    s1 = float(secs[k + 1].get("s")) if k + 1 < len(secs) else float(road.get("length"))
    lanes = []
    for side in ("left", "right"):
        for lane in sec.findall(f"{side}/lane"):
            w0 = lane_edges_at(road, s0 + 1e-6, side=side)
            meta = lane.find("userData[@code='mapforge.provenance/v1']")
            prov = json.loads(meta.get("value")) if meta is not None and meta.get("value") else {}
            sid = lane.find("userData[@code='mapforge.source_lane']")
            lanes.append({"id": int(lane.get("id")), "type": lane.get("type"),
                          "source": sid.get("value") if sid is not None else None,
                          "status": prov.get("status"), "eligibility": prov.get("eligibility"),
                          "exclusion_code": prov.get("exclusion_code"), "support_s": prov.get("support_s"),
                          "support_kind": prov.get("support_kind"), "role": prov.get("role"),
                          "trusted": prov.get("boundary_evidence_trusted")})
    out["sections"].append({"k": k, "s0": round(s0, 3), "s1": round(s1, 3),
                            "lanes": sorted(lanes, key=lambda x: x["id"])})

# outer edges as the audits pair them, with stations
pts, ss, hh = sample_road_ref(road, 0.5)
for record in B._target_outer_edges(road):
    target = record["points"]
    cands = [B._densify(B._project(g, lat0, lon0)) for g in src.lane_boundary_geometries(record["source_lane_id"])]
    cands = [g for g in cands if len(g) >= 2]
    if not cands:
        continue
    pick = int(np.argmin([float(np.median(B._point_polyline_distance(target, g)[0])) for g in cands]))
    source = B._clip_source_to_target_support(cands[pick], target)
    a, ai = B._point_polyline_distance(source, target)
    b, bi = B._point_polyline_distance(target, source)
    # station of each target sample along the road reference line
    st = [float(ss[np.argmin(np.linalg.norm(pts - p, axis=1))]) for p in target]
    big = [(round(st[i], 2), round(float(b[i]), 3)) for i in range(len(b)) if bi[i] and b[i] > 0.1]
    out["outer_edges"].append({
        "source_lane_id": record["source_lane_id"], "exclusion_code": record.get("exclusion_code"),
        "trusted": record.get("boundary_evidence_trusted"), "candidates": len(cands), "picked": pick,
        "target_s": [round(min(st), 2), round(max(st), 2)], "n_target": len(target),
        "t2s_inside_max": round(float(b[bi].max()), 3) if bi.any() else None,
        "s2t_inside_max": round(float(a[ai].max()), 3) if ai.any() else None,
        "t2s_samples_gt_0.1": big})
Path(sys.argv[3]).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
for sec in out["sections"]:
    print(f"section {sec['k']} s {sec['s0']}-{sec['s1']}")
    for l in sec["lanes"]:
        print("   ", l["id"], l["type"], l["source"], l["status"], l["eligibility"], l["exclusion_code"],
              l["support_s"] and [round(x, 2) for x in l["support_s"]], l["support_kind"], l["role"])
for e in out["outer_edges"]:
    print(e["source_lane_id"], e["target_s"], "t2s", e["t2s_inside_max"], "s2t", e["s2t_inside_max"],
          "cands", e["candidates"], e["picked"], e["exclusion_code"], e["trusted"])
    if e["t2s_samples_gt_0.1"]:
        print("    ", e["t2s_samples_gt_0.1"][:40])
