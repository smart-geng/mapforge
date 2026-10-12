"""One scoreboard JSON from runs of separate pipelines (``--pipelines shp`` / ``--pipelines map`` into separate
folders): the rows of all, in the order given. argv: output json, input scoreboard.json files."""
import json, sys
from pathlib import Path
rows, parts = [], []
for p in sys.argv[2:]:
    d = json.loads(Path(p).read_text(encoding="utf-8"))
    rows += d.get("rows", [])
    parts.append({k: v for k, v in d.items() if k != "rows"})
Path(sys.argv[1]).write_text(json.dumps({"parts": parts, "rows": rows}, ensure_ascii=False, indent=1, default=str),
                             encoding="utf-8")
print(len(rows), "rows")
