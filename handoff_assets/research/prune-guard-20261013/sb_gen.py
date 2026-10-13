"""Convert the scoreboard cases with the scoreboard's own generate() (same CLI commands) without scoring, keeping the
rows as jobs.json; score afterwards with `python -m mapforge.score --out <dir> --no-generate`.
cwd = code tree. argv: out dir, comma-separated pipelines, workers [, comma-separated cases]."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import scoreboard as sb
out, pipelines, workers = Path(sys.argv[1]), tuple(sys.argv[2].split(",")), int(sys.argv[3])
cases = sb.CASES
if len(sys.argv) > 4:
    want = set(sys.argv[4].split(","))
    cases = [c for c in sb.CASES if c[0] in want]
    assert len(cases) == len(want), (cases, want)
rows = sb.generate(out, cases, pipelines, workers)
(out / "jobs.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
print(json.dumps([(r["case"], r["pipeline"], r["returncode"], r["seconds"], r["generated"]) for r in rows]))
