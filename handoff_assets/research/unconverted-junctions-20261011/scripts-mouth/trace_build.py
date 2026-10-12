"""Trace mouth_seed_recovery.build (experiment tree): every leg_fit_fallback rebuild step and the recovery report.

usage (cwd = experiment tree): python trace_build.py <lon> <lat> <out_dir> [max_rebuilds]
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import leg_fit_fallback as LFF  # noqa: E402
from mapforge.ops import mouth_seed_recovery as M  # noqa: E402


def main(lon, lat, out_dir, max_rebuilds=None):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if max_rebuilds:
        LFF.MAX_REBUILDS = int(max_rebuilds)
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    junc, _ = src.find_junction(lon, lat)
    steps = []
    orig_next = LFF._next_step

    def next_step(state, exc):
        ok = orig_next(state, exc)
        steps.append({"t": round(time.time() - t0, 1), "error": " ".join(str(exc).split())[:400],
                      "last_leg": state.last_leg, "link_caps": dict(state.link_caps),
                      "length_caps": {k: round(v, 1) for k, v in state.length_caps.items()},
                      "relaxed": sorted(map(list, state.relaxed)), "continue": ok,
                      "fitted": [{k: e.get(k) for k in ("kind", "seed", "accepted")} for e in state.fitted]})
        print(json.dumps(steps[-1], ensure_ascii=False), flush=True)
        return ok

    LFF._next_step = next_step
    t0 = time.time()
    result = {"junction": junc.pid}
    try:
        stats, legs = M.build(src, junc, out_dir / f"trace-{junc.pid}.xodr", connect_mode="data", allow_uturn=False,
                              mouth_policy="source-envelope-candidate", mouth_margin_m=3.0)
        result.update(status="built", recovery=stats.get("mouth_seed_recovery"), legs=legs,
                      decisions=stats.get("mouth_envelope_decisions"))
    except Exception as exc:  # noqa: BLE001
        result.update(status=f"{type(exc).__name__}: {' '.join(str(exc).split())[:600]}",
                      recovery=getattr(exc, "stats", {}).get("mouth_seed_recovery") if hasattr(exc, "stats") else None)
    result["recovery_report"] = M.LAST_REPORT
    result["lff_steps"] = steps
    result["seconds"] = round(time.time() - t0, 1)
    (out_dir / f"trace-{junc.pid}.json").write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str),
                                                    encoding="utf-8")
    print(result["status"], result["seconds"])


if __name__ == "__main__":
    main(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], sys.argv[4] if len(sys.argv) > 4 else None)
