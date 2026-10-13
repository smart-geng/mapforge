"""boundary_inside_runs per source lane on a road, two XODRs side by side: the lanes whose worst value changed or is
over a floor. cwd = code tree. argv: output json, floor, then pairs 'name=before.xodr,after.xodr'."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import boundary_inside_runs as R
SHP = Path.cwd() / "shp_0222-0326"
src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
floor, out = float(sys.argv[2]), {}
for arg in sys.argv[3:]:
    name, pair = arg.split("=", 1)
    res = {}
    for tag, path in zip(("before", "after"), pair.split(",")):
        lanes = []
        res[tag] = {"metrics": R.audit(Path(path), SHP, src=src, lanes=lanes),
                    "lanes": {f"{l['road']}/{l['source_lane_id']}": l for l in lanes}}
    rows = []
    for key in sorted(set(res["before"]["lanes"]) | set(res["after"]["lanes"])):
        b, a = res["before"]["lanes"].get(key), res["after"]["lanes"].get(key)
        vb, va = b and b["max_m"], a and a["max_m"]
        if vb != va or max(vb or 0, va or 0) > floor:
            rows.append({"lane": key, "before": vb and round(vb, 3), "after": va and round(va, 3)})
    out[name] = {"before": res["before"]["metrics"], "after": res["after"]["metrics"], "lanes": rows}
    print("==", name, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in res["before"]["metrics"].items()},
          "->", {k: (round(v, 4) if isinstance(v, float) else v) for k, v in res["after"]["metrics"].items()})
    for r in sorted(rows, key=lambda r: -max(r["before"] or 0, r["after"] or 0)):
        print("   ", r["lane"], r["before"], "->", r["after"])
Path(sys.argv[1]).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
