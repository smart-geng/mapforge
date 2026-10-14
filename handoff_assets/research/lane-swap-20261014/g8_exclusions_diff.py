"""G8 exclusions whose classification differs between two g8.json files (keyed by target road/side/section/lane), with
the G8 status and the per-lane rows that changed. argv: before g8.json, after g8.json."""
import json, sys

a, b = (json.load(open(p, encoding="utf-8")) for p in sys.argv[1:3])


def ex(d):
    return {tuple(e["target"]): (e["code"], e.get("status"), (e.get("source_lane_id") or "-")[-5:]) for e in d["exclusions"]}


ea, eb = ex(a), ex(b)
print("status", a["status"], "->", b["status"], "| excluded", len(ea), "->", len(eb))
for k in sorted(set(ea) | set(eb), key=str):
    if ea.get(k) != eb.get(k):
        print("  exclusion", k, ea.get(k), "->", eb.get(k))
pa = {e["source_lane_id"]: e for e in a["per_lane"]}
pb = {e["source_lane_id"]: e for e in b["per_lane"]}
for k in sorted(set(pa) | set(pb)):
    x, y = pa.get(k), pb.get(k)
    if x != y:
        f = lambda e: e and (e["target"]["road_id"], e["target"]["lane_id"], round(e["source_to_target"]["max_m"], 4))
        print("  per_lane", k[-5:], f(x), "->", f(y))
