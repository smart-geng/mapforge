"""Capture every leg reference fit of one SHP junction conversion (generation stage only, read-only on sources).

Runs leg_fit_fallback.build exactly like shp_mouth_envelope.convert (mouth policy source-envelope-candidate, margin 3 m)
with recording wrappers around the converter's fit_leg_refline and the fallback's fit_leg. Writes <out>/<pid>.capture.pkl
and prints a summary. The tree's files are not modified (wrappers are installed at runtime).

usage: python capture.py <junction_pid> <out_dir>
"""
from __future__ import annotations

import json
import math
import pickle
import sys
import time
import traceback
from pathlib import Path

import numpy as np

TREE = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/diag-leg-tree")
sys.path.insert(0, str(TREE))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import leg_fit_fallback as LF  # noqa: E402
from mapforge.ops import refline_fit as RF  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402


def diag_original(gxy, kappa_cap=0.009, sharp_cap=0.000216, tol=1.5):
    """Re-evaluate fit_leg_refline's candidates and report each one's metrics and failed checks."""
    src, meta = RF._remove_impulse_outliers(np.asarray(gxy, float))
    rows = []

    def check(name, pv, *, median_tol, p95_tol, max_tol, max_segments=5):
        if pv is None:
            rows.append({"candidate": name, "built": False})
            return
        m = RF._connector_fidelity(pv, src)
        total = m["length_m"]
        fails = []
        if m["n_primitives"] > max_segments:
            fails.append(f"segments {m['n_primitives']}>{max_segments}")
        if m["min_primitive_m"] + 1e-9 < max(3.0, 0.03 * total):
            fails.append(f"min_primitive {m['min_primitive_m']:.2f}<{max(3.0, 0.03 * total):.2f}")
        if m["kappa_max"] > kappa_cap + 1e-9:
            fails.append(f"kappa {m['kappa_max']:.4f}>{kappa_cap}")
        if m["sharpness_max"] > sharp_cap + 1e-12:
            fails.append(f"sharpness {m['sharpness_max']:.6f}>{sharp_cap}")
        if m["start_m"] > 0.10:
            fails.append(f"start {m['start_m']:.2f}>0.10")
        if m["end_m"] > 0.10:
            fails.append(f"end {m['end_m']:.2f}>0.10")
        for d in ("source_to_target", "target_to_source"):
            if m[d]["median_m"] > median_tol:
                fails.append(f"{d}.median {m[d]['median_m']:.2f}>{median_tol}")
            if m[d]["p95_m"] > p95_tol:
                fails.append(f"{d}.p95 {m[d]['p95_m']:.2f}>{p95_tol}")
            if m[d]["max_m"] > max_tol:
                fails.append(f"{d}.max {m[d]['max_m']:.2f}>{max_tol}")
        rows.append({"candidate": name, "built": True, "fails": fails,
                     "s2t": {k: round(v, 3) for k, v in m["source_to_target"].items()},
                     "t2s": {k: round(v, 3) for k, v in m["target_to_source"].items()},
                     "kappa_max": round(m["kappa_max"], 5), "sharpness_max": m["sharpness_max"],
                     "n": m["n_primitives"], "min_seg": round(m["min_primitive_m"], 2),
                     "length": round(total, 2)})

    chord = src[-1] - src[0]
    from mapforge.ops.refline_fit import PlanSeg, PlanView
    line = PlanView(float(src[0, 0]), float(src[0, 1]), math.atan2(chord[1], chord[0]),
                    [PlanSeg("line", float(np.linalg.norm(chord)))])
    check("single-line", line, median_tol=0.20, p95_tol=0.30, max_tol=0.35)
    check("single-arc", RF._single_arc_candidate(src), median_tol=0.6, p95_tol=1.25, max_tol=tol)
    h0 = RF._robust_endpoint_heading(src, True)
    h1 = RF._robust_endpoint_heading(src, False)
    try:
        from pyclothoids import Clothoid
        cl = Clothoid.G1Hermite(float(src[0, 0]), float(src[0, 1]), h0, float(src[-1, 0]), float(src[-1, 1]), h1)
        sp = PlanView(float(src[0, 0]), float(src[0, 1]), h0,
                      [PlanSeg("spiral", float(cl.length), float(cl.KappaStart), float(cl.KappaEnd))])
    except Exception:  # noqa: BLE001
        sp = None
    check("single-spiral", sp, median_tol=0.6, p95_tol=1.25, max_tol=tol)
    p0, p1 = (src[0, 0], src[0, 1], h0), (src[-1, 0], src[-1, 1], h1)
    for n in (3, 4, 5):
        if n == 3:
            try:
                cls, _l, _k = RF._g2_seed(p0, p1, 0.0, 0.0, 3)
                base = RF._canonicalize_connector(PlanView(p0[0], p0[1], p0[2], [
                    PlanSeg("spiral", c.length, c.KappaStart, c.KappaEnd) for c in cls]))
            except Exception:  # noqa: BLE001
                base = None
            check("solve-g2-3", base, median_tol=0.6, p95_tol=1.25, max_tol=tol)
        opt = RF._optimize_connector(p0, p1, 0.0, 0.0, src, n, kappa_cap, sharpness_cap=sharp_cap)
        check(f"constrained-{n}", opt, median_tol=0.6, p95_tol=1.25, max_tol=tol)
    return {"impulse": meta, "end_headings_deg": [round(math.degrees(h0), 2), round(math.degrees(h1), 2)],
            "length_m": float(np.linalg.norm(np.diff(src, axis=0), axis=1).sum()), "candidates": rows}


MOUTH = __import__("os").environ.get("CAPTURE_MOUTH", "source-envelope-candidate")


def main():
    pid, out = sys.argv[1], Path(sys.argv[2])
    mode = sys.argv[3] if len(sys.argv) > 3 else "fallback"          # fallback | lane-axis
    extra = tuple(float(x) for x in sys.argv[4].split(",")) if len(sys.argv) > 4 else ()
    out.mkdir(parents=True, exist_ok=True)
    src = ProfileSource(str(TREE / "shp_0222-0326"), "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == pid)
    holder = {"attempt": 0}
    fits, fallbacks = [], []

    class RecState(LF.State):
        def __init__(self, s):
            super().__init__(s)
            holder["state"] = self

    orig_state = LF.State
    orig_fit = S.fit_leg_refline
    orig_fit_leg = LF.fit_leg
    orig_build = S.build_junction_xodr

    def rec_fit(pts, *a, **k):
        st = holder["state"]
        kind, seed = st.last_leg or (None, None)
        entry = {"attempt": holder["attempt"], "kind": kind, "seed": seed,
                 "chain": list(st.chains.get((seed, kind == "enter"), [])),
                 "link_caps": dict(st.link_caps), "length_caps": dict(st.length_caps),
                 "relaxed": sorted(st.relaxed), "gxy": np.asarray(pts, float).copy()}
        t0 = time.time()
        try:
            r = orig_fit(pts, *a, **k)
            entry.update(ok=True, dev=float(r[1]), selection=r[0].fit_meta.get("fit_selection"))
            return r
        except RF.ReflineFitError as exc:
            entry.update(ok=False, error=str(exc), diag=diag_original(pts))
            raise
        finally:
            entry["secs"] = round(time.time() - t0, 2)
            fits.append(entry)

    def rec_fit_leg(gxy, polylines, junction_end, relaxed=False):
        pv, record = orig_fit_leg(gxy, polylines, junction_end, relaxed=relaxed)
        st = holder["state"]
        fallbacks.append({"attempt": holder["attempt"], "leg": st.last_leg, "relaxed": relaxed,
                          "junction_end": junction_end, "gxy": np.asarray(gxy, float).copy(),
                          "polylines": [np.asarray(p, float) for p in polylines],
                          "tried": record["tried"], "accepted": pv is not None,
                          "pv": pv})
        return pv, record

    def rec_build(*a, **k):
        holder["attempt"] += 1
        return orig_build(*a, **k)

    LF.State, LF.fit_leg = RecState, rec_fit_leg
    S.fit_leg_refline, S.build_junction_xodr = rec_fit, rec_build
    t0 = time.time()
    status = "built"
    err = None
    try:
        if mode == "lane-axis":
            from mapforge.ops import leg_lane_axis as LA
            LA.EXTRA_KAPPA = extra
            stats, rep = LA.build(src, junc, out / f"{pid}.raw.xodr", connect_mode="data", allow_uturn=False,
                                  mouth_policy=MOUTH, mouth_margin_m=3.0)
        else:
            stats, rep = LF.build(src, junc, out / f"{pid}.raw.xodr", connect_mode="data", allow_uturn=False,
                                  mouth_policy=MOUTH, mouth_margin_m=3.0)
        summary_stats = {k: v for k, v in stats.items() if k in ("leg_lane_axis", "leg_fit_fallback", "roads_enter",
                         "roads_leave", "connections", "lanelinks", "skipped", "fit_dev_max", "legs_two_way",
                         "carriageway_split_decisions", "directional_carriageway_splits")}
        holder["stats"] = summary_stats
    except Exception as exc:  # noqa: BLE001
        status = type(exc).__name__
        err = " ".join(str(exc).split())
        traceback.print_exc(limit=3)
    finally:
        LF.State, LF.fit_leg = orig_state, orig_fit_leg
        S.fit_leg_refline, S.build_junction_xodr = orig_fit, orig_build
    st = holder.get("state")
    summary = {"junction": pid, "status": status, "error": err, "secs": round(time.time() - t0, 1),
               "attempts": holder["attempt"], "report": st.report() if st else None, "mode": mode,
               "extra_kappa": list(extra), "stats": holder.get("stats"),
               "chains": {f"{k[0]}:{'enter' if k[1] else 'leave'}": v for k, v in (st.chains.items() if st else [])}}
    tag = "" if MOUTH == "source-envelope-candidate" else ".legacy-mouth"
    with (out / f"{pid}.{mode}{tag}.capture.pkl").open("wb") as f:
        pickle.dump({"summary": summary, "fits": fits, "fallbacks": fallbacks}, f)
    print(json.dumps(summary, ensure_ascii=False, indent=1, default=str)[:6000])
    for e in fits:
        if not e["ok"]:
            print("FAILED FIT attempt", e["attempt"], e["kind"], e["seed"], "chain", e["chain"], "caps", e["link_caps"],
                  e["length_caps"], "relaxed", e["relaxed"])
            for c in e["diag"]["candidates"]:
                print("   ", json.dumps(c, ensure_ascii=False, default=str))
    for fb in fallbacks:
        print("FALLBACK attempt", fb["attempt"], fb["leg"], "relaxed", fb["relaxed"], "accepted", fb["accepted"])
        for row in fb["tried"]:
            print("    kcap %.3f n=%d seg-solver=%s off=%s med=%.2f outP95=%.2f outMax=%.2f zoneMax=%.2f max=%.2f cov=%.2f "
                  "hP95=%s hMax=%s flips=%.1f heading=%s near=%s lanes=%s" % (
                      row["kappa_cap"], row["segments"], row["solver_success"], row["end_offsets_m"], row["median_m"],
                      row["outside_p95_m"], row["outside_max_m"], row["zone_max_m"], row["max_m"],
                      row["field_coverage"],
                      None if row["heading_p95_deg"] is None else round(row["heading_p95_deg"], 1),
                      None if row["heading_max_deg"] is None else round(row["heading_max_deg"], 1),
                      row["flips_per_100m"], row["heading"], row["near_centre"], row["with_lanes"]))


if __name__ == "__main__":
    main()
