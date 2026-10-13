"""lane_refit (the g2-k04-c2 part, as c_stage_hold.py) on a raw XODR (the lane_refit input) with the prune guard's calls
recorded: road, span, the RDP vertices, the plain pruning, the guarded pruning and their errors, for every call the
guard was used in (one per fitted span and vertex set). cwd = code tree (with refit_prune_guard). argv: raw xodr,
output json."""
import json, sys, tempfile
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from mapforge import pipeline
from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_prune_guard as G

xin, jout = Path(sys.argv[1]), Path(sys.argv[2])
calls, state = [], {"road": None}
orig_guard, orig_refit = G.guard, LR.refit_road


def refit_road(road_el, *a, **k):
    state["road"] = road_el.get("id")
    return orig_refit(road_el, *a, **k)


def guard(ref_s, ref_t, vs, vt, grid, vals, seg_min, protected, keep_first, keep_last, protected_runs, short_kappa,
          info=None):
    out = orig_guard(ref_s, ref_t, vs, vt, grid, vals, seg_min, protected, keep_first, keep_last, protected_runs,
                     short_kappa, info)
    if info and info.get("used"):
        r = lambda x: [round(float(v), 3) for v in x]
        row = {"road": state["road"], "a": round(float(grid[0]), 3), "seg_min": seg_min,
               "protected": r(protected), "keep_first": bool(keep_first), "keep_last": bool(keep_last),
               "rdp_s": r(ref_s), "rdp_t": r(ref_t), "pruned_s": r(vs), "pruned_t": r(vt),
               "guarded_s": r(out[0]), "guarded_t": r(out[1]), **info}
        key = (row["road"], tuple(row["rdp_s"]), tuple(row["rdp_t"]))
        if key not in {(c["road"], tuple(c["rdp_s"]), tuple(c["rdp_t"])) for c in calls}:
            calls.append(row)
    return out


G.guard, LR.refit_road = guard, refit_road
params = dict(LR.VARIANTS[pipeline.DEFAULT_VARIANT])
params.pop("edge_joins", None)
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
with tempfile.TemporaryDirectory() as tmp:
    LR.apply(xin, Path(tmp) / "stage.xodr", params)
jout.write_text(json.dumps(calls, ensure_ascii=False, indent=1), encoding="utf-8")
for c in calls:
    print(c["road"], c["a"], "err", c["pruned_error_m"], "->", c["guarded_error_m"])
    print("   rdp    ", list(zip(c["rdp_s"], c["rdp_t"])))
    print("   pruned ", list(zip(c["pruned_s"], c["pruned_t"])))
    print("   guarded", list(zip(c["guarded_s"], c["guarded_t"])))
