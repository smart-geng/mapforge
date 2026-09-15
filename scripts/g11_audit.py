# -*- coding: utf-8 -*-
"""批量运行金凤 14 文件 G11，并写独立评估报告/sidecar。"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.validate.g11 import audit_file, load_policy, write_result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=Path,
                    default=ROOT / "out/closed-loop-inputs.json")
    ap.add_argument("--policy", type=Path,
                    default=ROOT / "profiles/validation/g11-opendrive-v1.draft.yaml")
    ap.add_argument("--baseline", type=Path,
                    default=ROOT / "out/v131-opendrive-baseline.json")
    ap.add_argument("--output", type=Path,
                    default=ROOT / "out/g11-opendrive-assessment.json")
    args = ap.parse_args()

    index = json.loads(args.index.read_text(encoding="utf-8"))
    policy = load_policy(args.policy)
    baseline = (json.loads(args.baseline.read_text(encoding="utf-8"))
                if args.baseline.exists() else None)
    entries = []
    print(f"{'file':28s} {'G11':5s} {'level':8s}  A B C D E  fail/warn")
    for entry in index.get("entries", []):
        artifact = ROOT / entry["artifact"]
        result = audit_file(artifact, policy, baseline)
        sidecar = artifact.with_suffix(".g11.json")
        write_result(sidecar, result)
        groups = result["groups"]
        marks = " ".join(groups[f"G11-{x}"]["level"][0] for x in "ABCDE")
        print(f"{entry['artifact']:28s} {result['status']:5s} {result['level']:8s}  "
              f"{marks}  {result['summary']['failures']}/{result['summary']['warnings']}")
        entries.append({
            "case": entry["case"], "pipeline": entry["pipeline"],
            "artifact": entry["artifact"],
            "sidecar": str(sidecar.relative_to(ROOT)), "result": result,
        })
    by_pipeline = {}
    for pipeline in sorted(set(x["pipeline"] for x in entries)):
        subset = [x for x in entries if x["pipeline"] == pipeline]
        codes = Counter(
            issue["code"]
            for x in subset for group in x["result"]["groups"].values()
            for issue in group["issues"] if issue["severity"] == "FAIL")
        by_pipeline[pipeline] = {
            "files": len(subset),
            "passed": sum(x["result"]["status"] == "PASS" for x in subset),
            "failed": sum(x["result"]["status"] == "FAIL" for x in subset),
            "failure_codes": dict(sorted(codes.items())),
        }
    report = {
        "schema": "mapforge/g11-assessment/v1",
        "suite": index.get("suite"),
        "policy": {
            "id": policy.get("id"), "version": policy.get("version"),
            "lifecycle": policy.get("lifecycle"), "sha256": policy.get("_sha256"),
            "consumer_profile": policy["consumer_profile"].get("id"),
        },
        "baseline_id": (baseline or {}).get("baseline_id"),
        "status": "PASS" if all(x["result"]["status"] == "PASS" for x in entries)
        else "FAIL",
        "summary": by_pipeline,
        "entries": entries,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"report: {args.output}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
