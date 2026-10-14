"""For every road the window advances more (refit_birth_window.keep_own_way), the farthest point from its source of
the window's road and of the own-section road (refit_birth_window.worst), and the roads put back: lane_refit.apply as
the SHP boundary step runs it (c_stage_hold.py's parameters) on stage inputs, with ``worst`` logged.
cwd = code tree. argv: output json, raw xodr ..."""
import json, sys, tempfile
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge import pipeline
from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_birth_window as BW

params = dict(LR.VARIANTS[pipeline.DEFAULT_VARIANT])
params.pop("edge_joins", None)
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
plain = BW.worst
log = []


def logged(road_el, src, origin):
    v = plain(road_el, src, origin)
    log.append((road_el.get("id"), v))
    return v


BW.worst = logged
out = {}
for raw in sys.argv[2:]:
    log.clear()
    with tempfile.TemporaryDirectory() as tmp:
        report = LR.apply(Path(raw), Path(tmp) / "out.xodr", params)
    pairs = {}
    for j in range(0, len(log), 2):
        (rid, far), (_, near) = log[j], log[j + 1]
        pairs[rid] = {"window_m": round(far, 4), "own_m": round(near, 4), "back": bool(far > near + BW.WORSE_M)}
    name = Path(raw).name.replace(".raw.xodr", "")
    out[name] = {"compared": pairs, "redone": report.get("birth_window_redone", [])}
    print(name, json.dumps(pairs), [(n["road"], n["reason"]) for n in out[name]["redone"]], flush=True)
Path(sys.argv[1]).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
