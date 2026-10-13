"""Experiment on a stage XODR (input of lane_refit.apply): for the named roads, when the plain refit (and the
zero-start retry of the cwd tree) still leaves a negative lane width, try
  B  "relative": the chains of the negative lanes fitted as width over their fitted inner boundary, whole chain;
  C  "hold":     at a free chain end without source, the width is held at its value where the source starts (the
                 zero-start mechanism with that width instead of zero), relative up to there, then handed over to
                 the boundary's own fit (_advanced_fit).
and report, per boundary of those roads, the deviation of the written boundary from its source samples
(|t written - t source| at the source stations, as lane_refit pairs them). cwd = code tree.
argv: in xodr, out xodr, comma-separated road ids, mode (plain|relative|hold), output json."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import numpy as np
from lxml import etree
from mapforge import pipeline
from mapforge.ops import lane_refit as LR

xin, xout, roads, MODE, jout = Path(sys.argv[1]), Path(sys.argv[2]), set(sys.argv[3].split(",")), sys.argv[4], Path(sys.argv[5])
log = {}
orig_refit, orig_zs = LR.refit_road, LR._zero_starts
TARGETS = {}


def negative_lanes(road, fitted):
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


def zs(road, side, chain, obs, fitted, start, end, a, b, unobserved_m=LR.UNOBSERVED_END_M):
    out = orig_zs(road, side, chain, obs, fitted, start, end, a, b, unobserved_m)
    rid = road.el.get("id")
    if MODE != "hold" or rid not in TARGETS or out:
        return out
    if not any((side, i, k) in TARGETS[rid] for i, k in chain["members"].items()):
        return out
    secs = sorted(chain["members"])
    inner = [fitted.get((side, i, chain["members"][i] - 1)) for i in secs]
    if not all(inner) or not len(obs):
        return out
    base = [p for part in inner for p in part]
    lo, hi = float(np.min(obs[:, 0])), float(np.max(obs[:, 0]))
    if start is None and not chain.get("birth") and lo > a + LR.GRID:
        first = obs[obs[:, 0] <= lo + LR.GRID]
        out["birth"] = (lo, None, float(np.median(first[:, 1])) - LR._at(base, lo))
    if end is None and not chain.get("death") and hi < b - LR.GRID:
        last = obs[obs[:, 0] >= hi - LR.GRID]
        out["death"] = (hi, None, float(np.median(last[:, 1])) - LR._at(base, hi))
    log.setdefault(rid, {}).setdefault("hold_events", []).append(
        {"side": side, "members": {str(i): k for i, k in chain["members"].items()},
         **{kind: [round(v[0], 3), round(v[2], 4)] for kind, v in out.items()}})
    return out


def refit(road_el, src, origin, params=None, observe=None, relative=None):
    rid = road_el.get("id")
    if rid not in roads or relative is not None or MODE == "plain":
        return orig_refit(road_el, src, origin, params, observe, relative)
    road, fitted, report = orig_refit(road_el, src, origin, dict(params or {}), observe, None)
    bad = negative_lanes(road, fitted)
    log.setdefault(rid, {})["plain_negative"] = bad
    if not bad:
        return road, fitted, report
    targets = {(side, i, k) for side, i, k, _ in bad}
    if MODE == "relative":
        def rel(road_, side, chain):
            return any((side, i, k) in targets for i, k in chain["members"].items())
        road, fitted, report = orig_refit(road_el, src, origin, dict(params or {}), observe, rel)
    else:
        TARGETS[rid] = targets
        road, fitted, report = orig_refit(road_el, src, origin, dict(params or {}), observe, None)
        del TARGETS[rid]
    log[rid]["retry_negative"] = negative_lanes(road, fitted)
    log[rid]["relative_chains"] = report.get("relative_chains")
    log[rid]["local_relative"] = report.get("local_relative")
    return road, fitted, report


LR.refit_road, LR._zero_starts = refit, zs
params = dict(LR.VARIANTS[pipeline.DEFAULT_VARIANT])
params.pop("edge_joins", None)
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
report = LR.apply(xin, xout, params)
log["skipped"] = report.get("skipped")
log["rows"] = {str(r.get("road")): {k: r.get(k) for k in ("chains", "relative_chains", "local_relative",
                                                        "crossing_repairs", "min_width_m", "advanced_events")}
               for r in report.get("rows", []) if str(r.get("road")) in roads}

# per-boundary deviation of the written roads from their source samples
from mapforge.validate.shp_boundary_fidelity import _origin
root = etree.parse(str(xout)).getroot()
origin = _origin(root)
src = LR._shp_source()
dev = {}
for road_el in root.findall("road"):
    rid = road_el.get("id")
    if rid not in roads:
        continue
    road = LR._Road(road_el)
    rows = []
    for side in ("right", "left"):
        for i in range(len(road.sections)):
            for k in range(0, len(road.lanes(i, side)) + 1):
                try:
                    obs = LR._source_observations(road, src, origin, side, i, k)
                except IndexError:
                    obs = None
                if obs is None or not len(obs):
                    continue
                d = np.array([abs(road.current(i, side, k, s) - t) for s, t in obs])
                rows.append({"side": side, "section": i, "k": k, "s": [round(float(obs[:, 0].min()), 2),
                                                                     round(float(obs[:, 0].max()), 2)],
                             "n": int(len(d)), "p95": round(float(np.percentile(d, 95)), 4),
                             "max": round(float(d.max()), 4),
                             "max_at_s": round(float(obs[int(np.argmax(d)), 0]), 2)})
    dev[rid] = rows
log["boundary_deviation"] = dev
jout.write_text(json.dumps(log, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
print(json.dumps({k: v for k, v in log.items() if k not in ("rows", "boundary_deviation")}, ensure_ascii=False,
                 default=str))
print(json.dumps(log["rows"], ensure_ascii=False, default=str))
for rid, rows in dev.items():
    for r in rows:
        print(rid, r["side"], r["section"], r["k"], r["s"], r["n"], "p95", r["p95"], "max", r["max"], "@", r["max_at_s"])
