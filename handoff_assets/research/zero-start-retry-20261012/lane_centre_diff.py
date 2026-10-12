"""Per source lane, the lane-centre distances the scoreboard's lane_center_noconflict_* use (tight-turn conflicts and
flare zones left out), before vs after, with the roads each lane is written on. cwd = code tree.
argv: before xodr, after xodr, output json."""
import json, sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge.validate import tight_turn_conflict as T
from mapforge.validate.lane_fidelity import extract_target_components


def per_lane(xodr):
    xodr = Path(xodr)
    side = lambda suffix: json.loads(xodr.with_suffix(suffix).read_text(encoding="utf-8")) \
        if xodr.with_suffix(suffix).exists() else None
    manifest, review = side(".source-lanes.json"), side(".source-review.json")
    root = ET.parse(xodr).getroot()
    road_of = {}
    for comp in extract_target_components(root)["components"]:
        road_of.setdefault(comp["source_lane_id"], set()).add(str(comp.get("road_id") or comp.get("target", ["?"])[0]))
    conflicts = {f["source_lane_id"] for f in T.findings(manifest)}
    lanes, _ = T.lane_distances(root, manifest, review)
    out, pooled = {}, []
    for sid, (a, b) in lanes.items():
        if sid in conflicts or not len(a) or not len(b):
            continue
        pooled += list(a) + list(b)
        out[sid] = {"roads": sorted(road_of.get(sid, [])), "max_m": float(max(a.max(), b.max())),
                    "p95_m": float(max(np.percentile(a, 95), np.percentile(b, 95))), "n": int(len(a) + len(b))}
    return out, pooled


before, pb = per_lane(sys.argv[1])
after, pa = per_lane(sys.argv[2])
rows = []
for sid in sorted(set(before) | set(after)):
    b, a = before.get(sid), after.get(sid)
    if b != a:
        rows.append({"source_lane_id": sid, "roads": (a or b)["roads"],
                     "before": {k: round(v, 4) for k, v in b.items() if k != "roads"} if b else None,
                     "after": {k: round(v, 4) for k, v in a.items() if k != "roads"} if a else None})
res = {"pooled_p95_before": float(np.percentile(pb, 95)), "pooled_p95_after": float(np.percentile(pa, 95)),
       "lanes_before": len(before), "lanes_after": len(after), "changed": rows}
Path(sys.argv[3]).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
print("pooled P95", round(res["pooled_p95_before"], 4), "->", round(res["pooled_p95_after"], 4),
      "| lanes", len(before), len(after), "| changed", len(rows))
for r in sorted(rows, key=lambda r: -abs(((r["after"] or {}).get("p95_m", 0)) - ((r["before"] or {}).get("p95_m", 0))))[:15]:
    print(r["source_lane_id"][-5:], r["roads"], "p95", (r["before"] or {}).get("p95_m"), "->", (r["after"] or {}).get("p95_m"),
          "max", (r["before"] or {}).get("max_m"), "->", (r["after"] or {}).get("max_m"))
