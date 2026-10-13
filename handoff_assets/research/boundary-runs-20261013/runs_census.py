"""boundary_inside (stacked records) next to boundary_inside_runs (records split where they do not meet) for SHP XODR
files, with the source lanes paired in more than one run. cwd = code tree carrying boundary_inside_runs.
argv: output json, then xodr files."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import boundary_inside as BI
from mapforge.validate import boundary_inside_runs as R

SHP = Path.cwd() / "shp_0222-0326"
src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
out = {"gap_m": R.GAP_M, "files": {}}
for name in sys.argv[2:]:
    x = Path(name)
    stacked = BI.audit(x, SHP, src=src)
    lanes = []
    runs = R.audit(x, SHP, src=src, lanes=lanes)
    split = sorted((l for l in lanes if l["runs"] > 1), key=lambda l: -(l["max_m"] or 0.0))
    worst = max(lanes, key=lambda l: l["max_m"] or 0.0) if lanes else None
    out["files"][x.name] = {"path": str(x), "boundary_inside": {k: round(v, 4) for k, v in stacked.items()},
                            "boundary_inside_runs": {k: (round(v, 4) if isinstance(v, float) else v)
                                                     for k, v in runs.items()},
                            "runs_worst_lane": worst, "split_lanes": split}
    print(x.name, out["files"][x.name]["boundary_inside"], out["files"][x.name]["boundary_inside_runs"],
          "worst", worst and (worst["road"], worst["source_lane_id"], round(worst["max_m"], 3)), flush=True)
Path(sys.argv[1]).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
