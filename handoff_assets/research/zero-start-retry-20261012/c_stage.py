"""lane_refit.apply (the part of variant g2-k04-c2 that apply_with_mouths gives it) on a stage XODR with the code of
the cwd tree; prints the skipped roads and the rows with a zero_start_retry. argv: in xodr, out xodr, output json."""
import json, shutil, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge import pipeline
from mapforge.ops import lane_refit as LR

xin, xout, jout = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
params = dict(LR.VARIANTS[pipeline.DEFAULT_VARIANT])
params.pop("edge_joins", None)
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
report = LR.apply(xin, xout, params)
shutil.copyfile(xin.with_suffix(".source-lanes.json"), xout.with_suffix(".source-lanes.json"))
retried = [r for r in report.get("rows", []) if r.get("zero_start_retry")]
out = {"rewritten": report.get("rewritten"), "skipped": report.get("skipped"),
       "retried": [{k: r.get(k) for k in ("road", "zero_start_retry", "advanced_events", "crossing_repairs",
                                         "min_width_m")} for r in retried]}
jout.write_text(json.dumps(out, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
print(json.dumps(out, ensure_ascii=False, default=str))
