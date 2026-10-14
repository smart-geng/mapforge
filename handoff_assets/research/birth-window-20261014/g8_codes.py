"""G8 of two versions of the same junctions: status, matched/eligible source lanes, excluded occurrences by code, and
the exclusions of lanes marked by lane_birth_advance (INFERRED) by code; with the delivery decision's status and its
blocking / review reasons. argv: pairs 'name=before_dir,after_dir,stem' (stem = file name without .g8.json)."""
import json, sys
from collections import Counter
from pathlib import Path


def load(d, stem):
    g8 = json.loads((Path(d) / f"{stem}.g8.json").read_text(encoding="utf-8"))
    dd = Path(d) / f"{stem}.delivery-decision.json"
    dec = json.loads(dd.read_text(encoding="utf-8")) if dd.exists() else {}
    return g8, dec


def summary(g8, dec):
    codes = Counter(e["code"] for e in g8["exclusions"])
    inferred = Counter(e["code"] for e in g8["exclusions"] if e.get("status") == "INFERRED")
    sc = g8["scope"]
    reasons = {k: dec.get(k) for k in dec if "reason" in k or k in ("status", "delivery_status")}
    return {"status": g8["status"], "matched": f"{sc['matched_source_lanes']}/{sc['eligible_source_lanes']}",
            "excluded": sc["excluded_occurrences"], "codes": dict(sorted(codes.items())),
            "inferred_codes": dict(sorted(inferred.items())), "decision": reasons}


for arg in sys.argv[1:]:
    name, rest = arg.split("=", 1)
    before, after, stem = rest.split(",")
    a, b = summary(*load(before, stem)), summary(*load(after, stem))
    print(f"== {name}")
    for k in a:
        mark = " " if a[k] == b[k] else "*"
        print(f" {mark} {k:15s} {a[k]}" + ("" if a[k] == b[k] else f"\n   {'':15s} -> {b[k]}"))
