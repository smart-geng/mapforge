"""Finish a generalization run whose driver was stopped (background time limit) after some conversions completed:
convert the junctions named in argv[3] with the module's own generate() (same CLI command and 3600 s cap), keep the
completed conversions as they are, then score everything with the module's score(). Completed rows carry no
returncode/seconds (the driver keeps them in memory until all jobs end); they are marked "row_reconstructed".
cwd = code tree. argv: out dir, name of the separate slow-junction run, comma-separated pids to convert."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import generalization as G
out, slow_run, todo = Path(sys.argv[1]), sys.argv[2], set(sys.argv[3].split(","))
SLOW = {"2023061310244790027", "2023070710412621654"}
junctions = G.load_set()["junctions"]
fresh = {r["pid"]: r for r in G.generate([j for j in junctions if j["pid"] in todo], out, 4)}
rows = []
for j in junctions:
    if j["pid"] in SLOW:
        (out / f"shp-{j['pid']}.convert.log").write_text(
            "not converted in this run: exceeds the module's 3600 s cap on this machine; converted separately "
            f"without the cap ({slow_run})", encoding="utf-8")
        rows.append({**j, "returncode": None, "seconds": None, "generated": False, "separately": slow_run})
    elif j["pid"] in fresh:
        rows.append(fresh[j["pid"]])
    else:
        xodr, log = out / f"shp-{j['pid']}.xodr", out / f"shp-{j['pid']}.convert.log"
        assert log.exists(), j["pid"]          # its conversion completed (the log is written after the subprocess)
        row = {**j, "returncode": None, "seconds": None, "generated": xodr.exists(),
               "row_reconstructed": "driver stopped before writing jobs.json; conversion had completed"}
        if not row["generated"]:
            row["cause"] = G.cause_of(log.read_text(encoding="utf-8"))
        rows.append(row)
(out / "jobs.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
payload = G.score(rows, out)
print(json.dumps(payload["summary"], ensure_ascii=False))
