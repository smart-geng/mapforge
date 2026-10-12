"""Score a generalization run converted in parts (gen_part.py, jobs-<tag>.json) with the module's own score(): the
two junctions over the module's 3600 s cap on this machine are recorded not generated, converted separately without
the cap (argv[2] names that run), as in the earlier runs. A junction that one part lost to the cap (returncode None)
may come again in a later part (same command, same cap, run alone): the later row is scored and keeps the first
attempt as ``rerun_after_cap``. cwd = code tree. argv: out dir, slow run name."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import generalization as G
out, slow_run = Path(sys.argv[1]), sys.argv[2]
SLOW = {"2023061310244790027", "2023070710412621654"}
done = {}
for f in sorted(out.glob("jobs-*.json")):
    for r in json.loads(f.read_text(encoding="utf-8")):
        if r["pid"] in done:
            first = done[r["pid"]]
            assert first["returncode"] is None and not first["generated"], r["pid"]
            r = {**r, "rerun_after_cap": {"part": first["part"], "returncode": first["returncode"],
                                          "seconds": first["seconds"], "generated": first["generated"]}}
        done[r["pid"]] = {**r, "part": f.name}
rows = []
for j in G.load_set()["junctions"]:
    if j["pid"] in SLOW:
        assert not (out / f"shp-{j['pid']}.xodr").exists()
        again = (out.parent / slow_run / f"shp-{j['pid']}.xodr").exists()
        (out / f"shp-{j['pid']}.convert.log").write_text(
            "not converted in this run: exceeds the module's 3600 s cap on this machine; "
            + (f"converted separately without the cap ({slow_run})" if again else
               "not converted again for this change (see slow-junctions.sha256)"), encoding="utf-8")
        rows.append({**j, "returncode": None, "seconds": None, "generated": False,
                     "separately": slow_run if again else None})
    else:
        rows.append(done[j["pid"]])
(out / "jobs.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
payload = G.score(rows, out)
print(json.dumps(payload["summary"], ensure_ascii=False))
