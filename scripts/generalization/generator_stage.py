"""Generator stage of the default SHP path on many junctions, with the failure cause of each (2026-10-08).

Runs, per junction and in its own process, what ``mapforge.pipeline.convert_shp`` runs before post-processing:
``shp_mouth_envelope.convert(..., finalize=False)`` (mouths moved 3 m, leg reference fallback, junction paving) and
``shp_leave_mouth.apply_file``. Much faster than the full CLI (no lane refit, no gates), so it is the way to see how
many junctions of the whole delivery get through the converter and why the others stop.

    .venv\\Scripts\\python scripts\\generalization\\generator_stage.py out\\generalize\\<name> [--all | --set] [--workers 4]
    .venv\\Scripts\\python scripts\\generalization\\generator_stage.py out\\generalize\\<name> --tally

``--all``: the 163 junctions of shp_0222-0326 (several hours with 4 workers); ``--set`` (default): the generalisation
set (profiles/validation/generalization-set-v1.yaml). Rows go to <out>/rows.jsonl (appended; done junctions are
skipped on a rerun). ``--one PID LON LAT`` is the per-process worker.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import traceback
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
SHP, PROFILE = ROOT / "shp_0222-0326", "ibd-smarteditor-v1"


def one(pid, lon, lat, out: Path):
    from mapforge.ops import shp_leave_mouth, shp_mouth_envelope
    raw = out / f"shp-{pid}.raw.xodr"
    t0 = time.time()
    row = {"pid": pid}
    try:
        rec = shp_mouth_envelope.convert(SHP, None, raw, 3.0, PROFILE, finalize=False, at=(lon, lat))
        row["junction"] = rec["junction"]
        legs = rec["stats"].get("leg_fit_fallback")
        if legs:
            row["legs"] = [(x["kind"], x["seed"], x["accepted"], (x["chosen"] or {}).get("kappa_cap"))
                           for x in legs["legs"]]
            row["truncated_enter_links"], row["relaxed"] = legs["truncated_enter_links"], legs["relaxed"]
        manifest = json.loads(raw.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
        moved = shp_leave_mouth.apply_file(raw, manifest, SHP, PROFILE, (lon, lat),
                                           rec["stats"].get("mouth_envelope_decisions", []))
        row.update(status="OK", leave_moved=len(moved.get("moved", [])))
    except Exception as exc:  # noqa: BLE001 - every cause is recorded
        row.update(status=type(exc).__name__, msg=" ".join(str(exc).split())[:2000],
                   where=traceback.extract_tb(exc.__traceback__)[-1].name)
    row["seconds"] = round(time.time() - t0, 1)
    print(json.dumps(row, ensure_ascii=False), flush=True)


def cause(row):
    from mapforge.validate.generalization import cause_of
    return "OK" if row["status"] == "OK" else cause_of(f"{row['status']}: {row.get('msg', '')}")


def jobs(all_junctions: bool):
    if all_junctions:
        from mapforge.adapters.shp.profile_source import ProfileSource
        src = ProfileSource(str(SHP), PROFILE)
        out = []
        for j in src.junctions:
            c = j.center
            lon, lat = (float(c[0][0]), float(c[0][1])) if hasattr(c[0], "__len__") else (float(c[0]), float(c[1]))
            out.append({"pid": j.pid, "kind": f"{len(j.enter_roads)}x{len(j.leave_roads)}", "lon": lon, "lat": lat})
        return out
    from mapforge.validate.generalization import load_set
    return load_set()["junctions"]


def run(out: Path, all_junctions: bool, workers: int):
    out.mkdir(parents=True, exist_ok=True)
    rows_path = out / "rows.jsonl"
    done = set()
    if rows_path.exists():
        done = {json.loads(x)["pid"] for x in rows_path.read_text(encoding="utf-8").splitlines() if x.strip()}
    todo = [j for j in jobs(all_junctions) if j["pid"] not in done]
    env = dict(os.environ, PYTHONIOENCODING="utf-8")

    def work(job):
        try:
            p = subprocess.run([sys.executable, __file__, str(out), "--one", job["pid"], repr(job["lon"]),
                                repr(job["lat"])], cwd=str(ROOT), env=env, capture_output=True, timeout=3600)
            lines = [x for x in p.stdout.decode("utf-8", "replace").splitlines() if x.startswith("{")]
            row = json.loads(lines[-1]) if lines else {"pid": job["pid"], "status": "NOOUTPUT",
                                                       "msg": p.stderr.decode("utf-8", "replace")[-500:]}
        except subprocess.TimeoutExpired:
            row = {"pid": job["pid"], "status": "TIMEOUT"}
        row["kind"] = job.get("kind")
        with rows_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
        return row

    print(f"{len(todo)} junctions to run, {len(done)} done", flush=True)
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for row in pool.map(work, todo):
            print(row["pid"], row.get("kind"), cause(row), row.get("seconds"), flush=True)
    tally(out)


def tally(out: Path):
    rows = [json.loads(x) for x in (out / "rows.jsonl").read_text(encoding="utf-8").splitlines() if x.strip()]
    counts = Counter(cause(r) for r in rows)
    print(f"{len(rows)} junctions")
    for name, n in counts.most_common():
        print(f"  {n:4d}  {name}  e.g. {[r['pid'] for r in rows if cause(r) == name][:3]}")
    fallback = [r for r in rows if r.get("legs")]
    print(f"leg reference fallback used at {len(fallback)} junctions, {sum(r['status'] == 'OK' for r in fallback)} OK")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--all", action="store_true", help="全部 163 个路口（默认只跑泛化集）")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--tally", action="store_true", help="只汇总已有 rows.jsonl")
    ap.add_argument("--one", nargs=3, metavar=("PID", "LON", "LAT"), help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    if a.one:
        a.out.mkdir(parents=True, exist_ok=True)
        one(a.one[0], float(a.one[1]), float(a.one[2]), a.out)
    elif a.tally:
        tally(a.out)
    else:
        run(a.out, a.all, a.workers)


if __name__ == "__main__":
    main()
