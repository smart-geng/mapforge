"""Convert part of the generalization set with the module's own generate() (same CLI command, 3600 s cap) into an
output folder, keeping the rows as jobs-<tag>.json (generate() itself writes jobs.json, overwritten by each part).
cwd = code tree. argv: out dir, comma-separated pids, workers, tag."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import generalization as G
out, todo, workers, tag = Path(sys.argv[1]), set(sys.argv[2].split(",")), int(sys.argv[3]), sys.argv[4]
junctions = [j for j in G.load_set()["junctions"] if j["pid"] in todo]
assert len(junctions) == len(todo), (len(junctions), todo)
rows = G.generate(junctions, out, workers)
(out / "jobs.json").rename(out / f"jobs-{tag}.json")
print(json.dumps([(r["pid"], r["returncode"], r["seconds"], r["generated"]) for r in rows]))
