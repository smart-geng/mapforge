"""Failure table of the generalization set for the decision memo: per converted junction (the module's run plus the
two slow junctions scored separately) T1 / T2 and every failed check with its value and threshold.
cwd = code tree. argv: generalization.json, slow score json (score_two.py) ..."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import yaml
from mapforge.validate import scoreboard as sb

SHORT = {"2023041111160971268": "0411", "2023062713394062899": "0627", "2023063015350832057": "0630",
         "2023080118313036830": "0801", "2023041216222042093": "0412", "2023062110304177600": "0621",
         "2023070118182048481": "0701", "2023080810023203229": "0808", "2023062913464138453": "0629",
         "2023072914272536839": "0729", "2023041810414040830": "0418", "2023070710324132429": "0707(0324)",
         "2023081110220126408": "0811", "2023071109582525264": "0711", "2023061310244790027": "061310",
         "2023070710412621654": "0707104"}
policy = yaml.safe_load(sb.POLICY.read_bytes())
limit = {c["metric"]: (c["op"], c["value"]) for t in policy["tiers"].values() for c in t["checks"]}
rows = []
gen = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
for j in gen["junctions"]:
    if j.get("generated"):
        rows.append((j["pid"], j["metrics"], j["tiers"]))
for p in sys.argv[2:]:
    for pid, r in json.loads(Path(p).read_text(encoding="utf-8")).items():
        rows.append((pid, r["metrics"], r["tiers"]))
t1 = sum(1 for _, _, t in rows if t["T1"]["status"] != "PASS")
t2 = sum(1 for _, _, t in rows if t["T2"]["status"] != "PASS")
print(f"converted {len(rows)}, T1 not passed {t1}, T2 not passed {t2}")
print("| 路口 | T1 | T2 | 不过的项（值 / 阈值） |")
print("|---|---|---|---|")
for pid, m, t in sorted(rows, key=lambda r: SHORT[r[0]]):
    failed = []
    for tier in ("T1", "T2"):
        for f in t[tier]["failed"]:
            v = m.get(f["metric"])
            op, lim = limit.get(f["metric"], ("?", "?"))
            failed.append(f"{tier} {f['metric']} {v:.4g} / {lim}" if isinstance(v, (int, float)) else f"{tier} {f['metric']} {v}")
    print(f"| {SHORT[pid]} | {t['T1']['status']} | {t['T2']['status']} | {'；'.join(failed) or '—'} |")
