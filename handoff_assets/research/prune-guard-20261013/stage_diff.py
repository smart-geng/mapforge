"""Roads that differ between two lane_refit stage outputs of the same raw XODR (c_stage_hold.py run in two code
trees), with the prune guards the second one's lane_refit reports record. argv: output json, pairs a.xodr=b.xodr..."""
import json, sys
from pathlib import Path
from lxml import etree

CODE = "mapforge.lane_refit/v1"
out_path, pairs = Path(sys.argv[1]), [p.split("=") for p in sys.argv[2:]]
result = {}
for a, b in pairs:
    ra, rb = etree.parse(a).getroot(), etree.parse(b).getroot()
    roads_a = {r.get("id"): r for r in ra.findall("road")}
    rows = []
    for rd in rb.findall("road"):
        other = roads_a.get(rd.get("id"))
        same = other is not None and etree.tostring(other) == etree.tostring(rd)
        rep = rd.find(f"userData[@code='{CODE}']")
        guards = json.loads(rep.get("value")).get("prune_guards") if rep is not None else None
        if not same or guards:
            rows.append({"road": rd.get("id"), "differs": not same, "prune_guards": guards})
    result[Path(b).name] = rows
    print(Path(b).name, "roads differing", sum(r["differs"] for r in rows), "with guards",
          sum(bool(r["prune_guards"]) for r in rows))
    for r in rows:
        print("   ", r["road"], "differs" if r["differs"] else "same", json.dumps(r["prune_guards"]))
out_path.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
