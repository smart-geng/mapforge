"""Trace lane_refit's negative-width check on one road: run lane_refit.apply (variant g2-k04-c2, the part
apply_with_mouths gives it) on a stage XODR, recording for that road every lane width's minimum before the
crossing repair, and each repair attempt. cwd = code tree. argv: in xodr, out xodr, road id, output json."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge import pipeline
from mapforge.ops import lane_refit as LR

xin, xout, rid, jout = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3], Path(sys.argv[4])
trace = {"road": rid, "before_repair": [], "repairs": None, "result": None, "fits": []}
orig_repair = LR._repair_crossings
orig_bump = LR._combine


def repair(road, widths, report):
    if road.el.get("id") != rid:
        return orig_repair(road, widths, report)
    for (i, side, k), pieces in sorted(widths.items()):
        vals = LR._samples(pieces)
        low_s, low = min(vals, key=lambda x: x[1])
        neg = [s for s, v in vals if v < -LR.NEGATIVE_WIDTH_TOL]
        trace["before_repair"].append({"section": i, "s0": road.s[i], "s1": road.s[i + 1], "side": side, "lane": k,
                                       "min_width_m": round(low, 4), "at_s": round(low_s, 3),
                                       "negative_s": [round(min(neg), 3), round(max(neg), 3)] if neg else None})
    ok = orig_repair(road, widths, report)
    trace["repairs"] = report.get("crossing_repairs")
    after = []
    for (i, side, k), pieces in sorted(widths.items()):
        vals = LR._samples(pieces)
        low_s, low = min(vals, key=lambda x: x[1])
        if low < -LR.NEGATIVE_WIDTH_TOL:
            after.append({"section": i, "side": side, "lane": k, "min_width_m": round(low, 4), "at_s": round(low_s, 3)})
    trace["result"] = {"ok": ok, "negative_after": after}
    return ok


LR._repair_crossings = repair
params = dict(LR.VARIANTS[pipeline.DEFAULT_VARIANT])
params.pop("edge_joins", None)
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
report = LR.apply(xin, xout, params)
trace["skipped"] = report.get("skipped")
jout.write_text(json.dumps(trace, ensure_ascii=False, indent=1), encoding="utf-8")
for row in trace["before_repair"]:
    if row["min_width_m"] < 0.05:
        print("before", row)
print("repairs", trace["repairs"])
print("result", trace["result"])
print("skipped", trace["skipped"])
