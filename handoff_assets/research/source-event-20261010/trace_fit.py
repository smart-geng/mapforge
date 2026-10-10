"""Trace lane_refit.fit_boundary calls for one road of a surface (lane_refit stage only, default params)."""
import json, sys
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.ops import lane_refit as LR
src, rid = Path(sys.argv[1]), sys.argv[2]
params = dict(LR.VARIANTS["g2-k04-c2"]); params.pop("edge_joins")
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
orig = LR.fit_boundary
CUR = {"road": None}
calls = []
def traced(obs_s, obs_t, a, b, *args, **kw):
    info = kw.get("info")
    if info is None:
        info = {}; kw["info"] = info
    out = orig(obs_s, obs_t, a, b, *args, **kw)
    if CUR["road"] == rid:
        calls.append({"a": a, "b": b, "start": kw.get("start"), "end": kw.get("end"), "ramp_start": kw.get("ramp_start"),
                      "ramp_end": kw.get("ramp_end"), "protect": list(kw.get("protect") or []), "corner_h": kw.get("corner_h"),
                      "t_range": [float(np.min(obs_t)), float(np.max(obs_t))], "n": len(obs_s),
                      "vs": np.round(info.get("vs", []), 2).tolist(), "vt": np.round(info.get("vt", []), 3).tolist(),
                      "h": np.round(info.get("h", []), 2).tolist(), "h0": info.get("h0"), "hn": info.get("hn")})
    return out
LR.fit_boundary = traced
orig_refit = LR.refit_road
def refit_road(road_el, *a, **k):
    CUR["road"] = road_el.get("id")
    return orig_refit(road_el, *a, **k)
LR.refit_road = refit_road
LR.apply(src, Path(sys.argv[3]), params)
json.dump(calls, open(sys.argv[4], "w"), default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o), indent=0)
for c in calls:
    print(round(c["a"], 2), round(c["b"], 2), "t", [round(x, 2) for x in c["t_range"]], "start", c["start"] and [round(x, 3) if isinstance(x, float) else x for x in c["start"]], "end", c["end"] and [round(x, 3) if isinstance(x, float) else x for x in c["end"]], "ramp", c["ramp_start"], c["ramp_end"], "protect", [round(x, 2) for x in c["protect"]], "corner_h", c["corner_h"])
    print("    vs", c["vs"], "\n    vt", c["vt"], "\n    h", c["h"], "h0", c["h0"] and round(c["h0"], 2), "hn", c["hn"] and round(c["hn"], 2))
