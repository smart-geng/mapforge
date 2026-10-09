"""Before/after of policy 0.10-draft (unlinked SHP stop lines: nearest match within 2 m, else a recorded source gap)
on the frozen scoreboard, generalization and 0621.

Geometry is unchanged, so nothing is converted again. Each case's stored Windows metrics plus the tight-turn and
paving-hole metrics (as in the 0.9 regrade; 0621 with its bound surface evidence) are graded by the 0.9-draft policy
from git; that must reproduce the 0.9 regrade's tiers and failed checks before any 0.10 row means anything. Then the
stop-line metrics, computed from the stored XODR, source manifest and G8 sidecar plus the source SHP, are added and
graded by the working-tree 0.10-draft policy. A rerun of evaluate_g8 on the unchanged manifest must reproduce the
stored G8 status and reasons wherever stop lines are resolved.
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
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.validate import paving_holes as P  # noqa: E402
from mapforge.validate import scoreboard as sb  # noqa: E402
from mapforge.validate import stopline_match as S  # noqa: E402
from mapforge.validate import tight_turn_conflict as T  # noqa: E402
from mapforge.validate.lane_fidelity import evaluate_g8  # noqa: E402

SCORE = ROOT / "out/scoreboard/20261008-safe-mouth-recovery-v3"
GEN = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3"
W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
BEFORE = "f98231d"  # last main commit with policy 0.9-draft
PRIOR = ROOT / "handoff_assets/research/paving-holes-20261009/rescore.json"


def policy_at(ref):
    return yaml.safe_load(subprocess.check_output(
        ["git", "show", ref + ":profiles/validation/static-acceptance-v1.draft.yaml"], cwd=ROOT))


def sidecar(xodr: Path, suffix: str):
    return json.loads(xodr.with_suffix(suffix).read_text(encoding="utf-8"))


def fails(tiers):
    return {t: [f["metric"] for f in v["failed"]] + [m + "(unavailable)" for m in v["unavailable"]]
            for t, v in tiers.items()}


def status(tiers):
    return {t: v["status"] for t, v in tiers.items()}


def grade(name, pipeline, metrics, xodr, old, new, prior, src, evidence=None):
    root = etree.parse(str(xodr)).getroot()
    before_metrics = dict(metrics)
    if pipeline == "shp":
        before_metrics.update(T.audit(root, sidecar(xodr, ".source-lanes.json"), sidecar(xodr, ".source-review.json")))
    before_metrics.update(P.audit(root, evidence))
    before = sb.apply_tiers(before_metrics, pipeline, old)
    assert status(before) == prior[name]["after"] and fails(before) == prior[name]["after_failed"], name
    g8 = sidecar(xodr, ".g8.json")
    assert g8["status"] == metrics["g8_status"], name
    extra, found = {}, []
    if pipeline == "shp":
        manifest = sidecar(xodr, ".source-lanes.json")
        found = S.findings(manifest, src)
        extra = S.audit(xodr, manifest, g8, src)
        if any(f["g8_comparable"] for f in found):
            rerun = evaluate_g8(xodr, manifest, S.G8_POLICY)
            assert rerun["status"] == g8["status"] and rerun["failure_reasons"] == g8["failure_reasons"], name
    else:
        extra = {"g8_status_stopline_resolved": g8["status"]}
    after = sb.apply_tiers({**before_metrics, **extra}, pipeline, new)
    return {"case": name, "pipeline": pipeline, "before": status(before), "after": status(after),
            "changed": status(before) != status(after), "before_failed": fails(before), "after_failed": fails(after),
            "g8_status": g8["status"], **extra,
            "findings": [{k: v for k, v in f.items() if k != "geometry"} for f in found]}


def main():
    old, new = policy_at(BEFORE), yaml.safe_load(sb.POLICY.read_bytes())
    assert old["version"] == "0.9-draft" and new["version"] == "0.10-draft"
    prior = {r["case"]: r for r in json.loads(PRIOR.read_text(encoding="utf-8"))["rows"]}
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    rows = []
    board = json.loads((SCORE / "scoreboard.json").read_text(encoding="utf-8"))
    for row in board["rows"]:
        name = f"scoreboard/{row['pipeline']}-{row['case']}"
        rows.append(grade(name, row["pipeline"], row["metrics"],
                          SCORE / f"{row['pipeline']}-{row['case']}.xodr", old, new, prior, src))
    gen = json.loads((GEN / "generalization.json").read_text(encoding="utf-8"))
    for j in gen["junctions"]:
        name = f"generalize/{j['pid']}"
        if not j.get("generated") or not isinstance(j.get("metrics"), dict):
            rows.append({"case": name, "pipeline": j["pipeline"], "generated": False,
                         "before": j.get("tiers"), "after": j.get("tiers"), "changed": False})
            continue
        rows.append(grade(name, j["pipeline"], j["metrics"], GEN / f"shp-{j['pid']}.xodr", old, new, prior, src))
    w = json.loads((W0621 / "scoreboard.json").read_text(encoding="utf-8"))["rows"][0]
    evidence = json.loads((W0621 / "surface-evidence.json").read_text(encoding="utf-8"))
    rows.append(grade("0621/candidate", "shp", w["metrics"], W0621 / "candidate.xodr", old, new, prior, src,
                      evidence))
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("case", "before", "after", "changed", "g8_status",
                                                 "g8_status_stopline_resolved", "stopline_matched",
                                                 "stopline_source_absent")}, ensure_ascii=False), flush=True)
    summary = {"schema": "mapforge/research/stopline-policy-rescore/v1", "policy_before": "0.9-draft",
               "policy_after": "0.10-draft", "rows": rows,
               "tier_changes": [r["case"] for r in rows if r.get("changed")],
               "tiers_reproduced_with_0_9": True, "g8_rerun_reproduces_stored": True, "geometry_changed": False,
               "note": "stored Windows metrics (esmini included) plus Linux-computed tight-turn, paving-hole and "
                       "stop-line metrics; stop lines read from shp_0222-0326 through ibd-smarteditor-v1"}
    (OUT / "rescore.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf8")


if __name__ == "__main__":
    main()
