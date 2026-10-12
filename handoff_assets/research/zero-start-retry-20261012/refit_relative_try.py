"""Experiment: lane_refit.apply (g2-k04-c2 part) on a stage XODR where refit_road, for the named roads, fits the
chains whose lane went negative in the plain refit relative to their fitted inner boundary (the existing `relative`
hook of refit_road, used by the MAP turn bays). Two passes per road: plain; if write_road would skip it for a
negative width, again with those chains relative. cwd = code tree. argv: in xodr, out xodr, comma-separated road ids,
output json."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge import pipeline
from mapforge.ops import lane_refit as LR

xin, xout, roads, jout = Path(sys.argv[1]), Path(sys.argv[2]), set(sys.argv[3].split(",")), Path(sys.argv[4])
MODE = sys.argv[5] if len(sys.argv) > 5 else "relative"
log = {}
orig_refit, orig_write = LR.refit_road, LR.write_road


def negative_lanes(road, fitted):
    """(side, section, lane) whose width goes below -NEGATIVE_WIDTH_TOL, from the fitted boundaries."""
    out = []
    for i in range(len(road.sections)):
        for side, sign in (("right", 1.0), ("left", -1.0)):
            for lane in road.lanes(i, side):
                k = abs(int(lane.get("id")))
                w = LR._difference(fitted[(side, i, k - 1)], fitted[(side, i, k)], road.s[i], road.s[i + 1], sign)
                low = min(v for _, v in LR._samples(w))
                if low < -LR.NEGATIVE_WIDTH_TOL:
                    out.append((side, i, k, round(low, 4)))
    return out


def refit(road_el, src, origin, params=None, observe=None, relative=None):
    rid = road_el.get("id")
    if rid not in roads or relative is not None:
        return orig_refit(road_el, src, origin, params, observe, relative)
    road, fitted, report = orig_refit(road_el, src, origin, dict(params or {}), observe, None)
    bad = negative_lanes(road, fitted)
    log[rid] = {"plain_negative": bad}
    if not bad:
        return road, fitted, report
    targets = {(side, i, k) for side, i, k, _ in bad}
    if MODE == "relative":
        def rel(road_, side, chain):
            return any((side, i, k) in targets for i, k in chain["members"].items())
        road, fitted, report = orig_refit(road_el, src, origin, dict(params or {}), observe, rel)
        log[rid]["retry_negative"] = negative_lanes(road, fitted)
        log[rid]["relative_chains"] = report.get("relative_chains")
        return road, fitted, report
    # MODE "zero-start": the unobserved free-end stretch rule of _zero_starts without its 1 m minimum
    saved = LR.UNOBSERVED_END_M
    LR.UNOBSERVED_END_M = float(MODE.split(":")[1]) if ":" in MODE else 0.0
    try:
        road, fitted, report = orig_refit(road_el, src, origin, dict(params or {}), observe, None)
    finally:
        LR.UNOBSERVED_END_M = saved
    log[rid]["retry_negative"] = negative_lanes(road, fitted)
    log[rid]["advanced_events"] = report.get("advanced_events")
    return road, fitted, report


LR.refit_road = refit
params = dict(LR.VARIANTS[pipeline.DEFAULT_VARIANT])
params.pop("edge_joins", None)
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
report = LR.apply(xin, xout, params)
log["skipped"] = report.get("skipped")
log["rows"] = [r for r in report.get("rows", []) if str(r.get("road")) in roads]
jout.write_text(json.dumps(log, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
print(json.dumps({k: v for k, v in log.items() if k != "rows"}, ensure_ascii=False, default=str))
for r in log["rows"]:
    print({k: r.get(k) for k in ("road", "chains", "relative_chains", "crossing_repairs", "min_width_m", "advanced_events")})
