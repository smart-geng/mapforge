"""Score converted generalization junctions with the scoreboard's own metrics and policy (no generation).

usage (cwd = experiment tree): python score_one.py <dir with shp-<pid>.xodr> <pid> [<pid> ...]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from mapforge.validate import generalization as G  # noqa: E402


def main(out, pids):
    out = Path(out)
    sets = {j["pid"]: j for j in G.load_set()["junctions"]}
    rows = [{**sets[p], "generated": (out / f"shp-{p}.xodr").exists()} for p in pids]
    payload = G.score(rows, out)
    for r in payload["junctions"]:
        tiers = r.get("tiers") or {}
        print(r["pid"], {t: (v["status"], [f["metric"] for f in v["failed"]]) for t, v in tiers.items()},
              r.get("score_error"))
    print(json.dumps(payload["summary"], ensure_ascii=False))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
