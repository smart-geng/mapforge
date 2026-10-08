"""Generalisation set: the default SHP pipeline on junctions the scoreboard never sees (2026-10-08).

Every geometry step since 2026-09 was developed and judged on the scoreboard's 7 SHP junctions (and 7 MAP files).
On 16 other SHP junctions of the same delivery (profiles/validation/generalization-set-v1.yaml, chosen once by a
fixed rule) the default CLI converted 1 on 2026-10-08, and that one failed T1 and T2. This module runs the default
CLI on the fixed set, scores every converted file with the scoreboard's own metrics and policy (it has no tiers of
its own and never changes the scoreboard), and reports conversions, tier passes and failure causes.

    python -m mapforge.validate.generalization --out out/generalize/<name> [--workers 4] [--no-generate]
                                               [--register NOTE]
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SET = ROOT / "profiles" / "validation" / "generalization-set-v1.yaml"
REGISTRY = ROOT / "experiments" / "generalization.jsonl"
CODE = "mapforge.generalization/v1"
TIMEOUT_S = 3600

# failure causes, matched on the conversion log in this order
CAUSES = (
    ("leg-reference-fit", r"ReflineFitError"),
    ("mouth-would-remove-seed-road", r"source-envelope mouth would remove the seed road"),
    ("leave-mouth-paving", r"leave-mouth paving not generated"),
    ("junction-paving", r"envelope surface not generated|unresolved gap inside source surface family"),
    ("source-key-conflict", r"SHP source key"),
)


def load_set(path=SET):
    import yaml
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def cause_of(log: str) -> str:
    """Cause of a failed conversion from its log: the last exception line and what follows it (the CLI's rich
    traceback also prints source lines, so earlier text may name other exceptions)."""
    lines = list(re.finditer(r"(?m)^\s*([A-Za-z_][\w.]*(?:Error|Exception)): ", log))
    tail = log[lines[-1].start():] if lines else log
    text = " ".join(tail.split())
    for name, pattern in CAUSES:
        if re.search(pattern, text):
            return name
    return f"other:{lines[-1].group(1)}" if lines else "other"


def generate(junctions, out: Path, workers: int = 4, shp_dir=None):
    """Default CLI (``mapforge.cli convert --at lon,lat --to xodr``) per junction; returns rows."""
    out.mkdir(parents=True, exist_ok=True)
    shp_dir = Path(shp_dir or ROOT / "shp_0222-0326")
    env = dict(os.environ, PYTHONIOENCODING="utf-8")

    def run(job):
        xodr = out / f"shp-{job['pid']}.xodr"
        cmd = [sys.executable, "-m", "mapforge.cli", "convert", str(shp_dir), "--at", f"{job['lon']},{job['lat']}",
               "--to", "xodr", "-o", str(xodr)]
        t0 = time.time()
        try:
            p = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, timeout=TIMEOUT_S)
            log, code = p.stdout.decode("utf-8", "replace") + p.stderr.decode("utf-8", "replace"), p.returncode
        except subprocess.TimeoutExpired:
            log, code = f"timeout after {TIMEOUT_S} s", None
        xodr.with_suffix(".convert.log").write_text(log, encoding="utf-8")
        row = {**job, "returncode": code, "seconds": round(time.time() - t0, 1), "generated": xodr.exists()}
        if not row["generated"]:
            row["cause"] = cause_of(log)
        return row

    with ThreadPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(run, junctions))
    (out / "jobs.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    return rows


def score(rows, out: Path, shp_dir=None, profile="ibd-smarteditor-v1"):
    """Scoreboard metrics and tiers (same policy) of every generated file."""
    import yaml
    from lxml import etree
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.validate import scoreboard as sb
    policy = yaml.safe_load(sb.POLICY.read_text(encoding="utf-8"))
    schema = etree.XMLSchema(etree.parse(str(sb.XSD)))
    src = ProfileSource(str(shp_dir or ROOT / "shp_0222-0326"), profile)
    results = []
    for r in rows:
        x = out / f"shp-{r['pid']}.xodr"
        if not x.exists():
            log = x.with_suffix(".convert.log")
            results.append({**r, "generated": False,
                            "cause": cause_of(log.read_text(encoding="utf-8")) if log.exists() else "other"})
            continue
        try:
            metrics = sb.evaluate(x, "shp", schema, src)
            tiers = sb.apply_tiers(metrics, "shp", policy)
            results.append({**r, "metrics": metrics, "tiers": tiers})
        except Exception as exc:  # noqa: BLE001 - recorded, not hidden
            results.append({**r, "score_error": f"{type(exc).__name__}: {exc}"})
    summary = summarize(results, policy)
    payload = {"schema": CODE, "set": SET.name, "policy": {k: policy.get(k) for k in ("id", "version", "lifecycle")},
               "summary": summary, "junctions": results}
    (out / "generalization.json").write_text(json.dumps(payload, ensure_ascii=False, indent=1, default=str),
                                             encoding="utf-8")
    return payload


def summarize(results, policy=None):
    causes, failed_metrics = {}, {}
    tier_pass = {}
    for r in results:
        if not r.get("generated"):
            causes[r.get("cause", "other")] = causes.get(r.get("cause", "other"), 0) + 1
            continue
        for tier, verdict in (r.get("tiers") or {}).items():
            tier_pass[tier] = tier_pass.get(tier, 0) + (verdict["status"] == "PASS")
            for f in verdict["failed"]:
                key = f"{tier}:{f['metric']}"
                failed_metrics[key] = failed_metrics.get(key, 0) + 1
    return {"junctions": len(results), "generated": sum(bool(r.get("generated")) for r in results),
            "not_generated_by_cause": dict(sorted(causes.items(), key=lambda kv: -kv[1])),
            "tier_pass": tier_pass, "failed_metrics": dict(sorted(failed_metrics.items(), key=lambda kv: -kv[1]))}


def register(payload, out: Path, note: str):
    head = subprocess.run(["git", "-c", "safe.directory=*", "rev-parse", "HEAD"], cwd=str(ROOT),
                          capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "-c", "safe.directory=*", "status", "--porcelain"], cwd=str(ROOT),
                                capture_output=True, text=True).stdout.strip())
    row = {"run": out.name, "created": _dt.datetime.now().astimezone().isoformat(timespec="seconds"), "note": note,
           "git": {"head": head, "dirty": dirty}, "set": payload["set"], "policy": payload["policy"],
           **payload["summary"]}
    REGISTRY.parent.mkdir(parents=True, exist_ok=True)
    with REGISTRY.open("a", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(row, ensure_ascii=False) + "\n")
    return row


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m mapforge.validate.generalization")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--no-generate", action="store_true", help="只对已有输出打分")
    ap.add_argument("--register", metavar="NOTE", help="把本次结果追加到 experiments/generalization.jsonl")
    a = ap.parse_args(argv)
    junctions = load_set()["junctions"]
    if a.no_generate:
        rows = json.loads((a.out / "jobs.json").read_text(encoding="utf-8"))
    else:
        rows = generate(junctions, a.out, a.workers)
    payload = score(rows, a.out)
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=1))
    if a.register:
        register(payload, a.out, a.register)
    return 0


if __name__ == "__main__":
    sys.exit(main())
