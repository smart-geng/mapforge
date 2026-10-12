"""Where a converted SHP junction's failing T2 metrics come from: the worst items of each metric with their roads,
computed with the scoreboard's own audit code (same exclusions). cwd = code tree. argv: xodr path, output json."""
import json, sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import shp_boundary_fidelity as B
from mapforge.validate import tight_turn_conflict as T
from mapforge.validate import endpoint_lateral as EL
from mapforge.validate.lane_fidelity import _resample, extract_target_components

xodr = Path(sys.argv[1])
side = lambda suffix: json.loads(xodr.with_suffix(suffix).read_text(encoding="utf-8")) \
    if xodr.with_suffix(suffix).exists() else None
manifest, g8, review = side(".source-lanes.json"), side(".g8.json"), side(".source-review.json")
g11, contacts = side(".g11.json"), side(".edge-contacts.json")
root = ET.parse(xodr).getroot()
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")
road_of = {}
for comp in extract_target_components(root)["components"]:
    road_of.setdefault(comp["source_lane_id"], set()).add(str(comp.get("road_id") or comp.get("target", ["?"])[0]))
out = {"xodr": xodr.name}

# lane centre without flare zones and tight-turn conflicts (lane_center_noconflict_*)
conflicts = {f["source_lane_id"] for f in T.findings(manifest)}
lanes, _ = T.lane_distances(root, manifest, review)
rows = []
for sid, (a, b) in lanes.items():
    if sid in conflicts or not len(a) or not len(b):
        continue
    rows.append({"source_lane_id": sid, "roads": sorted(road_of.get(sid, [])),
                 "max_m": round(float(max(a.max(), b.max())), 4),
                 "p95_m": round(float(max(np.percentile(a, 95), np.percentile(b, 95))), 4)})
out["lane_center_noconflict"] = sorted(rows, key=lambda r: -r["max_m"])[:6]

# endpoint lateral (lane_endpoint_lateral_max_m)
lanes_m = {lane["source_lane_id"]: lane for lane in manifest.get("lanes", [])}
comps = defaultdict(list)
for comp in extract_target_components(root)["components"]:
    comps[comp["source_lane_id"]].append(comp)
rows = []
for record in (g8 or {}).get("per_lane", []):
    sid = record["source_lane_id"]
    if sid not in lanes_m or len(comps.get(sid, [])) != 1:
        continue
    s = _resample((lanes_m[sid].get("geometry") or {}).get("coordinates", []), 1.0)
    t = _resample(comps[sid][0]["points"], 1.0)
    if len(s) < 2 or len(t) < 2:
        continue
    for at_start in (True, False):
        v = EL._lateral(s, t, at_start)
        if v is not None:
            rows.append({"source_lane_id": sid, "roads": sorted(road_of.get(sid, [])),
                         "end": "start" if at_start else "end", "lateral_m": round(v, 4)})
out["lane_endpoint_lateral"] = sorted(rows, key=lambda r: -r["lateral_m"])[:6]

# boundary inside (boundary_inside_*), per road, same exclusions as boundary_inside.audit
lat0, lon0 = B._origin(root)
rows = []
for road in root.findall("road"):
    if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
        continue
    grouped = defaultdict(list)
    for record in B._target_outer_edges(road):
        if record.get("boundary_evidence_trusted") is False:
            continue
        if (record.get("exclusion_code") in {"source-extension", "source-support-unavailable",
                                             "source-support-too-short"}
                and record.get("boundary_evidence_trusted") is not True):
            continue
        grouped[record["source_lane_id"]].append(record["points"])
    for sid, parts in grouped.items():
        target = np.vstack(parts)
        cands = [B._densify(B._project(g, lat0, lon0)) for g in src.lane_boundary_geometries(sid)]
        cands = [g for g in cands if len(g) >= 2]
        if not cands:
            continue
        source = min(cands, key=lambda g: float(np.median(B._point_polyline_distance(target, g)[0])))
        source = B._clip_source_to_target_support(source, target)
        a, ai = B._point_polyline_distance(source, target)
        b, bi = B._point_polyline_distance(target, source)
        vals = np.concatenate([a[ai], b[bi]])
        if len(vals):
            rows.append({"road": road.get("id"), "source_lane_id": sid, "max_m": round(float(vals.max()), 4),
                         "n_gt_0.35": int((vals > 0.35).sum()), "n": int(len(vals))})
out["boundary_inside"] = sorted(rows, key=lambda r: -r["max_m"])[:6]

# lane join curvature jumps (G11-D) and edge contacts
jumps = []
for issue in (g11 or {}).get("groups", {}).get("G11-D", {}).get("issues", []) or []:
    if issue.get("code") == "driving_lane_curvature_jump":
        for j in issue.get("joints", []):
            jumps.append({k: j.get(k) for k in j if k != "samples"} | {"issue_road": issue.get("road")})
out["lane_join_curvature_jump"] = sorted(jumps, key=lambda r: -r["curvature_jump_per_m"])[:6]
out["edge_contact_curvature"] = sorted((contacts or {}).get("rows", []), key=lambda r: -r["curvature_per_m"])[:6]
Path(sys.argv[2]).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
print(json.dumps(out, ensure_ascii=False, default=str)[:6000])
