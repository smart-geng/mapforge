"""Leg reference fits of SHP junctions: capture, and the fallback's acceptance on legs the converter fits (2026-10-08).

``capture``: runs ``shp_to_xodr.build_junction_xodr`` (default mouth policy) through mapforge.ops.leg_fit_fallback's
hooks and pickles, for every leg fit, the centre polyline, the leg kind and seed, the chained links, their lane centre
lines and boundaries, and whether the converter's own ``fit_leg_refline`` accepted it.

``calibrate``: on the captured legs the converter fits itself, runs the fallback (``leg_fit_fallback.fit_leg``) and
compares deviations from the centre polyline and the heading off the lane field with the converter's own fit. This
is how the acceptance rules were checked (2026-10-08: 43 legs, 40 accepted, P95 off the centre outside the 30 m
junction zone 0.191 m against the converter's 0.193 m).

    .venv\\Scripts\\python scripts\\generalization\\leg_fits.py capture <dir> <pid>...
    .venv\\Scripts\\python scripts\\generalization\\leg_fits.py calibrate <dir>...
"""
from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def capture(out: Path, pids):
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.ops import leg_fit_fallback as LF
    from mapforge.ops import refline_fit as RF
    from mapforge.ops import shp_to_xodr as S
    out.mkdir(parents=True, exist_ok=True)
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    for pid in pids:
        junc = next(j for j in src.junctions if j.pid == pid)
        legs, state = [], LF.State(src)
        original = S.fit_leg_refline

        def recording(pts, *args, **kwargs):
            enters = [c for c in state.calls if c[1]]
            leg = enters[-1] if enters else (state.calls[-1] if state.calls else None)
            rec = {"gxy": np.asarray(pts, float).copy(), "kind": "enter" if (leg and leg[1]) else "independent_leave",
                   "seed": leg[0] if leg else None, "links": [p for p, _c in leg[2]] if leg else []}
            proj = leg[3] if leg else None
            rec["lanes"] = [proj(l.geometry) for p in rec["links"] for l in src.lanes_of(p) if len(l.geometry) >= 2]
            rec["bounds"] = [proj(b) for p in rec["links"] for l in src.lanes_of(p)
                             for b in src.lane_boundary_geometries(l.lane_pid) if len(b) >= 2]
            state.calls = []
            try:
                result = original(pts, *args, **kwargs)
                rec["converter_fit"] = {"ok": True, "dev": float(result[1])}
                return result
            except RF.ReflineFitError as exc:
                rec["converter_fit"] = {"ok": False, "error": str(exc)}
                raise
            finally:
                legs.append(rec)

        chain_hook = LF._chain_hook(state, S._chained_links)
        saved = S.fit_leg_refline, S._chained_links
        S.fit_leg_refline, S._chained_links = recording, chain_hook
        try:
            S.build_junction_xodr(src, junc, out / f"{pid}.xodr", connect_mode="data", allow_uturn=False,
                                  mouth_policy="source-envelope-candidate", mouth_margin_m=3.0)
            status = "built"
        except Exception as exc:  # noqa: BLE001
            status = type(exc).__name__
        finally:
            S.fit_leg_refline, S._chained_links = saved
        with (out / f"{pid}.legs.pkl").open("wb") as f:
            pickle.dump(legs, f)
        print(pid, status, [(r["kind"][0], (r["seed"] or "")[-6:], r["converter_fit"]["ok"]) for r in legs], flush=True)


def calibrate(dirs):
    from mapforge.ops import leg_fit_fallback as LF
    from mapforge.ops import refline_fit as RF
    rows = []
    for d in dirs:
        for f in sorted(Path(d).glob("*.legs.pkl")):
            for rec in pickle.loads(f.read_bytes()):
                if not rec["converter_fit"]["ok"]:
                    continue
                end = "end" if rec["kind"] == "enter" else "start"
                lines = rec["lanes"] + rec["bounds"]
                pv, record = LF.fit_leg(rec["gxy"], lines, end)
                src, _ = RF._remove_impulse_outliers(np.asarray(rec["gxy"], float))
                own = LF.assess(RF.fit_leg_refline(rec["gxy"])[0], src, LF.heading_field(src, lines), end)
                chosen = record.get("chosen") or record["tried"][-1]
                rows.append((pv is not None, own, chosen))
                if pv is None:
                    print("not accepted:", f.stem, (rec["seed"] or "")[-6:],
                          {k: chosen[k] for k in ("heading_p95_deg", "heading_max_deg", "field_coverage",
                                                  "flips_per_100m", "near_centre", "with_lanes")})
    print(f"{len(rows)} legs the converter fits, fallback accepts {sum(r[0] for r in rows)}")
    for key in ("outside_p95_m", "outside_max_m", "max_m", "heading_p95_deg"):
        own = np.array([r[1][key] for r in rows if r[1][key] is not None], float)
        new = np.array([r[2][key] for r in rows if r[2][key] is not None], float)
        print(f"  {key}: converter median {np.median(own):.3f} max {own.max():.3f} | "
              f"fallback median {np.median(new):.3f} max {new.max():.3f}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("dir", type=Path)
    c.add_argument("pids", nargs="+")
    k = sub.add_parser("calibrate")
    k.add_argument("dirs", nargs="+")
    a = ap.parse_args(argv)
    if a.cmd == "capture":
        capture(a.dir, a.pids)
    else:
        calibrate(a.dirs)


if __name__ == "__main__":
    main()
