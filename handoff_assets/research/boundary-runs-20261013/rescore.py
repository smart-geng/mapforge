"""Before/after of policy 0.11-draft (SHP T2 boundary measured run by run, boundary_inside_runs) on stored results.

Geometry is unchanged, so nothing is converted again. Groups (argv names those to run, default all):

- frozen: scoreboard and generalization of out/*/20261008-safe-mouth-recovery-v3 and the 0621 workbench candidate, as
  regraded to 0.10-draft on 2026-10-09 (../stopline-match-20261009/rescore.py). Stored Windows metrics plus the
  tight-turn, paving-hole and stop-line metrics computed again from the stored files must reproduce that regrade's
  0.10 tiers and failed checks;
- main: the Linux scoreboard and generalization of main d13d300 (../zero-start-retry-20261012/);
- hold-width: the same after the hold-width retry (../hold-width-retry-20261013/).

For main and hold-width every XODR is checked against the recorded SHA-256, and the stored metrics graded by the
0.10-draft policy (from git) must reproduce the stored tiers. In every group the stacked boundary_inside computed here
must equal the stored value (frozen: computed on Windows, so within 1e-6 m). Then boundary_inside_runs is added and the
working-tree 0.11-draft policy grades the result. The two slow generalization junctions of main and hold-width are not
graded by the module (over its 3600 s cap); their run-by-run values are listed apart.
"""
from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import yaml
from lxml import etree

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.validate import boundary_inside as BI  # noqa: E402
from mapforge.validate import boundary_inside_runs as RUNS  # noqa: E402
from mapforge.validate import paving_holes as P  # noqa: E402
from mapforge.validate import scoreboard as sb  # noqa: E402
from mapforge.validate import stopline_match as S  # noqa: E402
from mapforge.validate import tight_turn_conflict as T  # noqa: E402

BEFORE = "d13d300"  # main with policy 0.10-draft
RESEARCH = ROOT / "handoff_assets/research"
FROZEN_SCORE = ROOT / "out/scoreboard/20261008-safe-mouth-recovery-v3"
FROZEN_GEN = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3"
W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
FROZEN_010 = RESEARCH / "stopline-match-20261009/rescore.json"
# Linux runs of this cloud session (the XODR are not in the repository; their hashes are)
RUNS_DIR = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/runs")
LINUX = {
    "main": {"evidence": RESEARCH / "zero-start-retry-20261012", "board": "scoreboard-zero-start.json",
             "gen": "generalization-zero-start.json", "shp": RUNS_DIR / "sb-zs-shp", "map": RUNS_DIR / "sb-zs-map",
             "gen_dir": RUNS_DIR / "gen-zs", "slow": {"2023070710412621654": RUNS_DIR / "gen-slow-zs",
                                                      "2023061310244790027": RUNS_DIR / "gen-slow-seams"}},
    "hold-width": {"evidence": RESEARCH / "hold-width-retry-20261013", "board": "scoreboard-hold-width.json",
                   "gen": "generalization-hold-width.json", "shp": RUNS_DIR / "sb-rw-shp",
                   "map": RUNS_DIR / "sb-rw-map", "gen_dir": RUNS_DIR / "gen-rw",
                   "slow": {"2023070710412621654": RUNS_DIR / "gen-slow-rw",
                            "2023061310244790027": RUNS_DIR / "gen-slow-seams"}},
}
KEYS = ("boundary_inside_p95_m", "boundary_inside_max_m")
RUN_KEYS = ("boundary_inside_runs_p95_m", "boundary_inside_runs_max_m", "boundary_inside_runs_split")


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


def sha(path: Path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def recorded(path: Path):
    """name -> this change's hash in a two-column hash list (base, this change, name)."""
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line and not line.startswith("#"):
            _base, new, name = line.split()
            out[name] = new
    return out


def runs_metrics(xodr, metrics, src, tol):
    stacked = BI.audit(xodr, None, src=src)
    for k in KEYS:
        a, b = stacked[k], metrics.get(k)
        assert (a is None and b is None) or abs(a - b) <= tol, (xodr.name, k, a, b)
    lanes = []
    found = RUNS.audit(xodr, None, src=src, lanes=lanes)
    worst = max((l for l in lanes if l["max_m"] is not None), key=lambda l: l["max_m"], default=None)
    split = [l for l in lanes if l["runs"] > 1]
    return found, {"worst_lane": worst, "split_lanes": sorted(split, key=lambda l: -(l["max_m"] or 0.0))}


def row(case, pipeline, metrics, before_tiers, new, xodr, src, tol):
    extra, where = {}, {}
    if pipeline == "shp":
        extra, where = runs_metrics(xodr, metrics, src, tol)
    after = sb.apply_tiers({**metrics, **extra}, pipeline, new)
    return {"case": case, "pipeline": pipeline, "before": status(before_tiers), "after": status(after),
            "changed": status(before_tiers) != status(after), "before_failed": fails(before_tiers),
            "after_failed": fails(after), **{k: metrics.get(k) for k in KEYS if pipeline == "shp"}, **extra, **where}


def frozen(old, new, src):
    prior = {r["case"]: r for r in json.loads(FROZEN_010.read_text(encoding="utf-8"))["rows"]}

    def full(name, pipeline, metrics, xodr, evidence=None):
        m = dict(metrics)
        root = etree.parse(str(xodr)).getroot()
        if pipeline == "shp":
            manifest = sidecar(xodr, ".source-lanes.json")
            m.update(T.audit(root, manifest, sidecar(xodr, ".source-review.json")))
            m.update(P.audit(root, evidence))
            m.update(S.audit(xodr, manifest, sidecar(xodr, ".g8.json"), src))
        else:
            m.update(P.audit(root, evidence))
            m["g8_status_stopline_resolved"] = sidecar(xodr, ".g8.json")["status"]
        before = sb.apply_tiers(m, pipeline, old)
        assert status(before) == prior[name]["after"] and fails(before) == prior[name]["after_failed"], name
        return row(name, pipeline, m, before, new, xodr, src, 1e-6)

    rows = []
    board = json.loads((FROZEN_SCORE / "scoreboard.json").read_text(encoding="utf-8"))
    for r in board["rows"]:
        name = f"scoreboard/{r['pipeline']}-{r['case']}"
        rows.append(full(name, r["pipeline"], r["metrics"], FROZEN_SCORE / f"{r['pipeline']}-{r['case']}.xodr"))
    gen = json.loads((FROZEN_GEN / "generalization.json").read_text(encoding="utf-8"))
    for j in gen["junctions"]:
        name = f"generalize/{j['pid']}"
        if not j.get("generated") or not isinstance(j.get("metrics"), dict):
            rows.append({"case": name, "pipeline": "shp", "generated": False, "changed": False})
            continue
        rows.append(full(name, "shp", j["metrics"], FROZEN_GEN / f"shp-{j['pid']}.xodr"))
    w = json.loads((W0621 / "scoreboard.json").read_text(encoding="utf-8"))["rows"][0]
    evidence = json.loads((W0621 / "surface-evidence.json").read_text(encoding="utf-8"))
    rows.append(full("0621/candidate", "shp", w["metrics"], W0621 / "candidate.xodr", evidence))
    return rows, []


def linux(group, old, new, src):
    g = LINUX[group]
    board = json.loads((g["evidence"] / g["board"]).read_text(encoding="utf-8"))
    hashes = recorded(g["evidence"] / "scoreboard-xodr.sha256")
    rows = []
    for r in board["rows"]:
        xodr = g[r["pipeline"]] / r["artifact"]
        assert sha(xodr) == hashes[r["artifact"]], xodr
        before = sb.apply_tiers(r["metrics"], r["pipeline"], old)
        assert before == r["tiers"], (group, r["artifact"])
        rows.append(row(f"scoreboard/{r['pipeline']}-{r['case']}", r["pipeline"], r["metrics"], before, new, xodr,
                        src, 0.0))
    gen = json.loads((g["evidence"] / g["gen"]).read_text(encoding="utf-8"))
    hashes = recorded(g["evidence"] / "generalization-xodr.sha256")
    for j in gen["junctions"]:
        name = f"generalize/{j['pid']}"
        if not j.get("generated") or not isinstance(j.get("metrics"), dict):
            rows.append({"case": name, "pipeline": "shp", "generated": False, "changed": False})
            continue
        xodr = g["gen_dir"] / f"shp-{j['pid']}.xodr"
        assert sha(xodr) == hashes[xodr.name], xodr
        before = sb.apply_tiers(j["metrics"], "shp", old)
        assert before == j["tiers"], (group, j["pid"])
        rows.append(row(name, "shp", j["metrics"], before, new, xodr, src, 0.0))
    slow = []
    for pid, d in g["slow"].items():
        xodr = d / f"shp-{pid}.xodr"
        stacked = BI.audit(xodr, None, src=src)
        lanes = []
        found = RUNS.audit(xodr, None, src=src, lanes=lanes)
        slow.append({"case": f"slow/{pid}", "run": d.name, "xodr_sha256": sha(xodr), **stacked, **found,
                     "split_lanes": sorted((l for l in lanes if l["runs"] > 1), key=lambda l: -(l["max_m"] or 0.0))})
    return rows, slow


def tier_pass(rows, key):
    out = {}
    for r in rows:
        for t, v in (r.get(key) or {}).items():
            out[t] = out.get(t, 0) + (v == "PASS")
    return out


def main():
    groups = sys.argv[1:] or ["frozen", "main", "hold-width"]
    old, new = policy_at(BEFORE), yaml.safe_load(sb.POLICY.read_bytes())
    assert old["version"] == "0.10-draft" and new["version"] == "0.11-draft"
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    result = {"schema": "mapforge/research/boundary-runs-policy-rescore/v1", "policy_before": old["version"],
              "policy_after": new["version"], "gap_m": RUNS.GAP_M, "groups": {}}
    for group in groups:
        rows, slow = frozen(old, new, src) if group == "frozen" else linux(group, old, new, src)
        for r in rows:
            print(group, json.dumps({k: r.get(k) for k in ("case", "before", "after", "changed", *KEYS, *RUN_KEYS)},
                                    ensure_ascii=False), flush=True)
        scored = [r for r in rows if r.get("before")]
        result["groups"][group] = {
            "tier_pass_before": {"scoreboard": tier_pass([r for r in scored if r["case"].startswith("scoreboard/")],
                                                         "before"),
                                 "generalization": tier_pass([r for r in scored if r["case"].startswith("generalize/")],
                                                             "before")},
            "tier_pass_after": {"scoreboard": tier_pass([r for r in scored if r["case"].startswith("scoreboard/")],
                                                        "after"),
                                "generalization": tier_pass([r for r in scored if r["case"].startswith("generalize/")],
                                                            "after")},
            "tier_changes": [r["case"] for r in rows if r.get("changed")],
            "rows": rows, "slow": slow}
    result["note"] = ("stacked = boundary_inside (records of a source lane on a road stacked into one polyline); runs "
                      "= boundary_inside_runs (split where a record starts more than gap_m from where the previous one "
                      "ends); frozen metrics are stored Windows values, the rest Linux; the source boundaries are read "
                      "from shp_0222-0326 through ibd-smarteditor-v1")
    name = "rescore.json" if len(groups) == 3 else "rescore-" + "-".join(groups) + ".json"
    (OUT / name).write_text(json.dumps(result, ensure_ascii=False, indent=1, default=float), encoding="utf8")


if __name__ == "__main__":
    main()
