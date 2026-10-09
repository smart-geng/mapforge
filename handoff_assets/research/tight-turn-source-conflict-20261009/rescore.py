"""Before/after of policy 0.8-draft (tight-turn source conflicts) on the frozen scoreboard, generalization and 0621.

Geometry is unchanged, so nothing is converted again: the stored Windows metrics of each case get the new
tight-turn metrics (computed from the stored XODR, source manifest and source review) and are graded by the
0.7-draft policy (from git HEAD) and by the working-tree 0.8-draft policy. Re-grading the stored metrics with
0.7 must reproduce the stored tiers before any 0.8 row means anything.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import yaml
from lxml import etree

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT))
from mapforge.validate import scoreboard as sb  # noqa: E402
from mapforge.validate import tight_turn_conflict as T  # noqa: E402

SCORE = ROOT / "out/scoreboard/20261008-safe-mouth-recovery-v3"
GEN = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3"
W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
BEFORE = "2a0b0f4"  # last main commit with policy 0.7-draft


def policy_at_head():
    raw = subprocess.check_output(["git", "show", BEFORE + ":profiles/validation/static-acceptance-v1.draft.yaml"], cwd=ROOT)
    return yaml.safe_load(raw)


def new_metrics(xodr: Path):
    stem = xodr.with_suffix("")
    manifest = json.loads(Path(str(stem) + ".source-lanes.json").read_text(encoding="utf-8"))
    review = json.loads(Path(str(stem) + ".source-review.json").read_text(encoding="utf-8"))
    out = T.audit(etree.parse(str(xodr)).getroot(), manifest, review)
    out["_conflict_lanes"] = [f["source_lane_id"] for f in T.findings(manifest)]
    return out


def grade(name, pipeline, metrics, stored_tiers, xodr, old, new):
    reproduced = sb.apply_tiers(metrics, pipeline, old)
    assert reproduced == stored_tiers, (name, reproduced, stored_tiers)
    extra = new_metrics(xodr) if pipeline == "shp" else {}
    lanes = extra.pop("_conflict_lanes", [])
    after = sb.apply_tiers({**metrics, **extra}, pipeline, new)
    fails = lambda tiers: {t: [f["metric"] for f in v["failed"]] + [m + "(unavailable)" for m in v["unavailable"]]
                           for t, v in tiers.items()}
    return {"case": name, "pipeline": pipeline,
            "before": {t: v["status"] for t, v in stored_tiers.items()},
            "after": {t: v["status"] for t, v in after.items()},
            "changed": {t: v["status"] for t, v in stored_tiers.items()} != {t: v["status"] for t, v in after.items()},
            "before_failed": fails(stored_tiers), "after_failed": fails(after),
            "lane_center_noflare_p95_m": metrics.get("lane_center_noflare_p95_m"),
            "lane_center_noflare_max_m": metrics.get("lane_center_noflare_max_m"),
            **{k: v for k, v in extra.items()}, "conflict_lanes": lanes}


def main():
    old, new = policy_at_head(), yaml.safe_load(sb.POLICY.read_bytes())
    assert old["version"] == "0.7-draft" and new["version"] == "0.8-draft"
    rows = []
    board = json.loads((SCORE / "scoreboard.json").read_text(encoding="utf-8"))
    for row in board["rows"]:
        name = f"scoreboard/{row['pipeline']}-{row['case']}"
        rows.append(grade(name, row["pipeline"], row["metrics"], row["tiers"],
                          SCORE / f"{row['pipeline']}-{row['case']}.xodr", old, new))
    gen = json.loads((GEN / "generalization.json").read_text(encoding="utf-8"))
    for j in gen["junctions"]:
        if not j.get("generated") or not isinstance(j.get("metrics"), dict):
            rows.append({"case": f"generalize/{j['pid']}", "pipeline": j["pipeline"], "generated": False,
                         "before": j.get("tiers"), "after": j.get("tiers"), "changed": False})
            continue
        rows.append(grade(f"generalize/{j['pid']}", j["pipeline"], j["metrics"], j["tiers"],
                          GEN / f"shp-{j['pid']}.xodr", old, new))
    w = json.loads((W0621 / "scoreboard.json").read_text(encoding="utf-8"))["rows"][0]
    rows.append(grade("0621/candidate", "shp", w["metrics"], w["tiers"], W0621 / "candidate.xodr", old, new))
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("case", "before", "after", "changed", "lane_center_noflare_p95_m",
                                                 "lane_center_noconflict_p95_m", "tight_turn_source_conflicts")},
                         ensure_ascii=False), flush=True)
    summary = {"schema": "mapforge/research/tight-turn-policy-rescore/v1", "policy_before": "0.7-draft",
               "policy_after": "0.8-draft", "rows": rows,
               "tier_changes": [r["case"] for r in rows if r.get("changed")],
               "stored_tiers_reproduced_with_0_7": True, "geometry_changed": False,
               "note": "stored Windows metrics (esmini included) plus Linux-computed tight-turn metrics"}
    (OUT / "rescore.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf8")


if __name__ == "__main__":
    main()
