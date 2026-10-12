"""Compare scoreboard runs: stored Windows (graded again by the current policy), Linux base, Linux anchor."""
import json, sys
from pathlib import Path
import yaml
A = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/cloud-verify/mapforge")
sys.path.insert(0, str(A))
from mapforge.validate import scoreboard as sb
policy = yaml.safe_load(sb.POLICY.read_bytes())
def rows(path):
    d = json.loads(Path(path).read_text())
    out = {}
    for r in d.get("rows", []):
        out[f"{r['pipeline']}-{r['case']}"] = r
    for j in d.get("junctions", []):
        out[j["pid"]] = j
    return out
base, anch = rows(sys.argv[1]), rows(sys.argv[2])
graded = {c["metric"] for t in policy["tiers"].values() for c in t["checks"]}
def tiers(r, pipe):
    m = r.get("metrics")
    if not isinstance(m, dict): return None
    t = sb.apply_tiers(m, pipe, policy)
    return {k: (v["status"], [f["metric"] for f in v["failed"]] + [u + "(n/a)" for u in v["unavailable"]]) for k, v in t.items()}
changed_rows = 0
for k in base:
    rb, ra = base[k], anch.get(k)
    pipe = rb.get("pipeline", "shp")
    tb, ta = tiers(rb, pipe), tiers(ra, pipe) if ra else None
    mb, ma = rb.get("metrics") or {}, (ra or {}).get("metrics") or {}
    diffs = []
    for m in sorted(set(mb) | set(ma)):
        x, y = mb.get(m), ma.get(m)
        if isinstance(x, (int, float)) and isinstance(y, (int, float)) and not isinstance(x, bool):
            if abs(x - y) > 1e-6 * max(1.0, abs(x)): diffs.append((m, x, y))
        elif x != y and m not in ("artifact",): diffs.append((m, x, y))
    if diffs or tb != ta: changed_rows += 1
    flag = "TIER CHANGE" if tb != ta else ""
    print(f"== {k} {pipe} base {tb and {t: v[0] for t, v in tb.items()}} anchor {ta and {t: v[0] for t, v in ta.items()}} {flag}")
    if tb != ta: print("   base failed", tb, "\n   anch failed", ta)
    for m, x, y in diffs:
        mark = "*" if m in graded else " "
        print(f"   {mark} {m:42s} {x} -> {y}")
print("rows with any change:", changed_rows, "/", len(base))
