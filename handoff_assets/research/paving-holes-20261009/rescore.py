"""Before/after of policy 0.9-draft (paving holes: breakpoint sampling, bound source voids) on the frozen scoreboard,
generalization and 0621.

Geometry is unchanged, so nothing is converted again. Each case's stored Windows metrics plus the tight-turn metrics
(as in the 0.8 regrade) are graded by the 0.8-draft policy from git; that must reproduce the 0.8 regrade's tiers
before any 0.9 row means anything. Then the paving-hole metrics, computed from the stored XODR (and, for 0621 only,
the bound source-surface reconstruction evidence of its workbench run), are added and graded by the working-tree
0.9-draft policy. Only 0621 has such evidence; every other case counts every hole.
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
from mapforge.validate import paving_holes as P  # noqa: E402
from mapforge.validate import scoreboard as sb  # noqa: E402
from mapforge.validate import smoothness  # noqa: E402
from mapforge.validate import tight_turn_conflict as T  # noqa: E402

SCORE = ROOT / "out/scoreboard/20261008-safe-mouth-recovery-v3"
GEN = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3"
W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
BEFORE = "bb8008b"  # last main commit with policy 0.8-draft
PRIOR = ROOT / "handoff_assets/research/tight-turn-source-conflict-20261009/rescore.json"


def policy_at(ref):
    return yaml.safe_load(subprocess.check_output(
        ["git", "show", ref + ":profiles/validation/static-acceptance-v1.draft.yaml"], cwd=ROOT))


def tight_turn(xodr: Path):
    stem = xodr.with_suffix("")
    manifest = json.loads(Path(str(stem) + ".source-lanes.json").read_text(encoding="utf-8"))
    review = json.loads(Path(str(stem) + ".source-review.json").read_text(encoding="utf-8"))
    return T.audit(etree.parse(str(xodr)).getroot(), manifest, review)


def fails(tiers):
    return {t: [f["metric"] for f in v["failed"]] + [m + "(unavailable)" for m in v["unavailable"]]
            for t, v in tiers.items()}


def grade(name, pipeline, metrics, xodr, old, new, prior, evidence=None):
    before_metrics = {**metrics, **(tight_turn(xodr) if pipeline == "shp" else {})}
    before = sb.apply_tiers(before_metrics, pipeline, old)
    assert {t: v["status"] for t, v in before.items()} == prior[name]["after"], name
    assert fails(before) == prior[name]["after_failed"], name
    root = etree.parse(str(xodr)).getroot()
    paving = P.audit(root, evidence)
    grid = smoothness.surface_continuity(root)
    after = sb.apply_tiers({**before_metrics, **paving}, pipeline, new)
    status = lambda tiers: {t: v["status"] for t, v in tiers.items()}
    return {"case": name, "pipeline": pipeline, "before": status(before), "after": status(after),
            "changed": status(before) != status(after), "before_failed": fails(before), "after_failed": fails(after),
            "paving_holes_gt1cm2_stored": metrics.get("paving_holes_gt1cm2"),
            "paving_holes_gt1cm2_recomputed": grid["paving_holes_gt1cm2"],
            "paving_hole_area_max_grid_m2": grid["paving_hole_area_max"], **paving}


def main():
    old, new = policy_at(BEFORE), yaml.safe_load(sb.POLICY.read_bytes())
    assert old["version"] == "0.8-draft" and new["version"] == "0.9-draft"
    prior = {r["case"]: r for r in json.loads(PRIOR.read_text(encoding="utf-8"))["rows"]}
    rows = []
    board = json.loads((SCORE / "scoreboard.json").read_text(encoding="utf-8"))
    for row in board["rows"]:
        name = f"scoreboard/{row['pipeline']}-{row['case']}"
        rows.append(grade(name, row["pipeline"], row["metrics"],
                          SCORE / f"{row['pipeline']}-{row['case']}.xodr", old, new, prior))
    gen = json.loads((GEN / "generalization.json").read_text(encoding="utf-8"))
    for j in gen["junctions"]:
        name = f"generalize/{j['pid']}"
        if not j.get("generated") or not isinstance(j.get("metrics"), dict):
            rows.append({"case": name, "pipeline": j["pipeline"], "generated": False,
                         "before": j.get("tiers"), "after": j.get("tiers"), "changed": False})
            continue
        rows.append(grade(name, j["pipeline"], j["metrics"], GEN / f"shp-{j['pid']}.xodr", old, new, prior))
    w = json.loads((W0621 / "scoreboard.json").read_text(encoding="utf-8"))["rows"][0]
    evidence = json.loads((W0621 / "surface-evidence.json").read_text(encoding="utf-8"))
    rows.append(grade("0621/candidate", "shp", w["metrics"], W0621 / "candidate.xodr", old, new, prior, evidence))
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("case", "before", "after", "changed", "paving_holes_gt1cm2_stored",
                                                 "paving_holes_resampled", "paving_holes_source_void",
                                                 "paving_holes_counted")}, ensure_ascii=False), flush=True)
    summary = {"schema": "mapforge/research/paving-holes-policy-rescore/v1", "policy_before": "0.8-draft",
               "policy_after": "0.9-draft", "rows": rows,
               "tier_changes": [r["case"] for r in rows if r.get("changed")],
               "tiers_reproduced_with_0_8": True, "geometry_changed": False,
               "evidence_0621": {"path": "out/workbench/wb11-source-tracks-20261009-v2/surface-evidence.json",
                                 "original_holes_sha256": evidence["source_support"]["original_holes"]["sha256"]},
               "note": "stored Windows metrics (esmini included) plus Linux-computed tight-turn and paving-hole metrics"}
    (OUT / "rescore.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf8")


if __name__ == "__main__":
    main()
