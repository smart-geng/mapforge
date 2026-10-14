"""Lane swaps fitted together (and failed ones) that lane_refit's per-road reports record, for XODRs.
argv: output json, xodr..."""
import json, sys
from pathlib import Path
from lxml import etree

CODE = "mapforge.lane_refit/v1"
out, rows = Path(sys.argv[1]), []
for x in sys.argv[2:]:
    for rd in etree.parse(x).getroot().findall("road"):
        rep = rd.find(f"userData[@code='{CODE}']")
        if rep is None:
            continue
        r = json.loads(rep.get("value"))
        for kind in ("lane_swaps", "lane_swaps_failed"):
            got = r.get(kind)
            for row in ([got] if isinstance(got, dict) else got or []):
                rows.append({"file": Path(x).name, "road": rd.get("id"), "kind": kind, **row})
out.write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
for r in rows:
    print(r["file"], "road", r["road"], r["kind"], {k: v for k, v in r.items() if k not in ("file", "road", "kind")})
print(len(rows), "rows")
