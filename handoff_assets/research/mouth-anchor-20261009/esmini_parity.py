"""Linux esmini v3.6.0 (exp tree) vs the stored Windows esmini figures of the frozen scoreboard, generalization, 0621."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import scoreboard as sb
A = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/cloud-verify/mapforge")
SCORE, GEN = A / "out/scoreboard/20261008-safe-mouth-recovery-v3", A / "out/generalize/20261008-safe-mouth-recovery-v3"
rows = []
for r in json.loads((SCORE / "scoreboard.json").read_text())["rows"]:
    rows.append((f"{r['pipeline']}-{r['case']}", SCORE / f"{r['pipeline']}-{r['case']}.xodr", r["metrics"]))
for j in json.loads((GEN / "generalization.json").read_text())["junctions"]:
    if j.get("generated") and isinstance(j.get("metrics"), dict):
        rows.append((j["pid"], GEN / f"shp-{j['pid']}.xodr", j["metrics"]))
w = json.loads((A / "out/workbench/wb11-source-tracks-20261009-v2/scoreboard.json").read_text())["rows"][0]
rows.append(("0621", A / "out/workbench/wb11-source-tracks-20261009-v2/candidate.xodr", w["metrics"]))
same = 0
for name, x, m in rows:
    e = sb.esmini_check(x)
    ok = e["pass"] == m.get("esmini_pass") and e["gap_max_cm"] == m.get("esmini_gap_max_cm")
    same += ok
    print(("OK  " if ok else "DIFF"), name, e["pass"], e["gap_max_cm"], "| stored", m.get("esmini_pass"), m.get("esmini_gap_max_cm"))
print(same, "/", len(rows))
