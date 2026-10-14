"""Slopes and corner sizes of each divider refit_lane_swap builds on lane_refit stage inputs: the full width W and its
slope W' at the middle of the swap, mu0 / mu1, the corner half length each slope asks for at the curvature target
(mu / KAPPA_CAP) and the one taken (h_m). cwd = code tree. argv: raw xodr..., then '--' and the c_stage_hold.py path.
The stage outputs go to tmp-*.stage.* beside this script. With several inputs in one call only the first output is the
one a stage run of its own writes (the first run leaves module state behind); the printed sizes are the same."""
import runpy, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.ops import lane_refit as LR, refit_lane_swap as SW

orig = SW.divider


def divider(road, side, chain, fitted, kappa=None, obs=None):
    out = orig(road, side, chain, fitted, kappa, obs)
    sw, secs = chain["swap"], sorted(chain["members"])
    a, b = road.s[secs[0]], road.s[secs[-1] + 1]
    inner = [p for i in secs for p in fitted[(side, i, chain["members"][i] - 1)]]
    outer = [p for i in secs for p in fitted[(side, i, chain["members"][i] + 1)]]
    sign = 1.0 if side == "left" else -1.0
    full = [(p[0], p[1], *(sign * v for v in p[2:])) for p in LR._combine(outer, inner, a, b, -1.0)]
    run, mid = sw["s_d"] - sw["s_b"], 0.5 * (sw["s_b"] + sw["s_d"])
    w, w1 = LR._at(full, mid), LR._at(full, mid, 1)
    mu0, mu1 = w / run + 0.5 * w1, w / run - 0.5 * w1
    k = kappa or LR.KAPPA_CAP
    print(f"road {road.el.get('id')} {side} {sw['type']}: run {run:.3f} W {w:.3f} W' {w1:.4f} mu0 {mu0:.4f} mu1 {mu1:.4f}"
          f" asks h {mu0 / k:.2f} / {mu1 / k:.2f} took {sw['row'].get('h_m')} corner offset mu*h/6"
          f" {[round(m * h / 6, 3) for m, h in zip((mu0, mu1), sw['row'].get('h_m') or (0, 0))]}")
    return out


SW.divider = divider
args = sys.argv[1:]
cut = args.index("--")
raws, stage = args[:cut], args[cut + 1]
for raw in raws:
    out = Path(sys.argv[0]).with_name("tmp-" + Path(raw).name.replace(".raw.", ".stage."))
    sys.argv = [stage, raw, str(out), str(out.with_suffix(".json"))]
    runpy.run_path(stage, run_name="__main__")
