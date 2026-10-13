"""boundary_inside (mapforge.validate.boundary_inside.audit) measured two ways on one SHP XODR: as the audit does it
(all included outer-edge records of a source lane on a road stacked into one polyline), and with that polyline split
where consecutive records do not meet (the next record's first point more than GAP_M from the previous record's last),
each contiguous part paired with the source on its own. A gap left by excluded records otherwise becomes a straight
chord between the parts that the source is measured against. Lists the source lanes whose result differs.
cwd = code tree. argv: xodr, output json."""
import json, sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import shp_boundary_fidelity as B

GAP_M = 1.0
xodr = Path(sys.argv[1])
root = ET.parse(xodr).getroot()
lat0, lon0 = B._origin(root)
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")


def pair(sid, target):
    cands = [B._densify(B._project(g, lat0, lon0)) for g in src.lane_boundary_geometries(sid)]
    cands = [g for g in cands if len(g) >= 2]
    if not cands:
        return None
    source = min(cands, key=lambda g: float(np.median(B._point_polyline_distance(target, g)[0])))
    source = B._clip_source_to_target_support(source, target)
    a, ai = B._point_polyline_distance(source, target)
    b, bi = B._point_polyline_distance(target, source)
    return a[ai].tolist(), b[bi].tolist()


ways = {"stacked": ([], []), "split": ([], [])}
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
        got = pair(sid, np.vstack(parts))
        if got is None:
            continue
        ways["stacked"][0].extend(got[0]); ways["stacked"][1].extend(got[1])
        stacked_max = max(got[0] + got[1]) if got[0] + got[1] else 0.0
        runs = [[parts[0]]]
        for p in parts[1:]:
            if np.linalg.norm(p[0] - runs[-1][-1][-1]) > GAP_M:
                runs.append([p])
            else:
                runs[-1].append(p)
        split_vals = []
        for run in runs:
            g = pair(sid, np.vstack(run))
            if g is not None:
                ways["split"][0].extend(g[0]); ways["split"][1].extend(g[1])
                split_vals += g[0] + g[1]
        split_max = max(split_vals) if split_vals else 0.0
        if len(runs) > 1:
            rows.append({"road": road.get("id"), "source_lane_id": sid, "parts": len(parts), "runs": len(runs),
                         "stacked_max_m": round(stacked_max, 4), "split_max_m": round(split_max, 4),
                         "stacked_n_gt_0.35": int((np.asarray(got[0] + got[1]) > 0.35).sum()),
                         "split_n_gt_0.35": int((np.asarray(split_vals) > 0.35).sum())})


def summary(s2t, t2s):
    if not s2t or not t2s:
        return None
    return {"boundary_inside_p95_m": round(float(max(np.percentile(s2t, 95), np.percentile(t2s, 95))), 4),
            "boundary_inside_max_m": round(float(max(max(s2t), max(t2s))), 4)}


out = {"xodr": xodr.name, "gap_m": GAP_M, "stacked": summary(*ways["stacked"]), "split": summary(*ways["split"]),
       "split_lanes": sorted(rows, key=lambda r: -r["stacked_max_m"])}
Path(sys.argv[2]).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
print(json.dumps({k: out[k] for k in ("stacked", "split")}), len(rows), "lanes split")
for r in out["split_lanes"][:8]:
    print("  ", json.dumps(r))
