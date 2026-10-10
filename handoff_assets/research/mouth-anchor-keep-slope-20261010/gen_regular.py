"""Generalization set with the module's own generate/score, minus the two junctions that exceed the module's
3600 s cap on this Linux machine (converted and scored separately without the cap: gen-slow-ks). Their rows are
recorded not generated, as the baseline run recorded them (cause "other", a timeout). cwd = code tree."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import generalization as G
out = Path(sys.argv[1])
SLOW = {"2023061310244790027", "2023070710412621654"}
junctions = G.load_set()["junctions"]
for pid in SLOW:
    assert not (out / f"shp-{pid}.xodr").exists()
rows = {r["pid"]: r for r in G.generate([j for j in junctions if j["pid"] not in SLOW], out, 4)}
for j in junctions:
    if j["pid"] in SLOW:
        (out / f"shp-{j['pid']}.convert.log").write_text(
            "not converted in this run: exceeds the module's 3600 s cap on this machine; converted separately "
            "without the cap (gen-slow-ks)", encoding="utf-8")
        rows[j["pid"]] = {**j, "returncode": None, "seconds": None, "generated": False, "separately": "gen-slow-ks"}
rows = [rows[j["pid"]] for j in junctions]
(out / "jobs.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
payload = G.score(rows, out)
print(json.dumps(payload["summary"], ensure_ascii=False))
