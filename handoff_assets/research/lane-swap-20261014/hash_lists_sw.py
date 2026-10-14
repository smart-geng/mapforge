"""Hash lists of the lane-swap round: scoreboard and generalization XODRs (base, this change, name) and every output
file of the slow junction 0707104. Base and change were both converted with OPENBLAS_NUM_THREADS=1.
argv: evidence dir."""
import hashlib, sys
from pathlib import Path
S = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/runs")
E = Path(sys.argv[1])


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else "-"


def write(name, head, pairs, tail):
    lines = [head] + [f"{sha(b)}  {sha(n)}  {n.name}" for b, n in pairs] + ["# " + t for t in tail]
    (E / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((E / name).read_text())


cases = ("NODE5", "node13", "node16", "node17", "node18", "node3", "node4")
write("scoreboard-xodr.sha256",
      "# SHA256 of every scoreboard XODR: main (8a959e8; sb-pg = the prune-guard round's conversions, the outputs "
      "main's code gives) vs this change (sb-sw); both OPENBLAS_NUM_THREADS=1",
      [(S / "sb-pg" / f"{p}-{c}.xodr", S / "sb-sw" / f"{p}-{c}.xodr") for p in ("map", "shp") for c in cases],
      ["No scoreboard junction has a lane swap (stage-diff.json): the lane_refit stage writes the same roads."])
names = sorted({p.name for p in (S / "gen-pg").glob("shp-*.xodr")} | {p.name for p in (S / "gen-sw").glob("shp-*.xodr")})
write("generalization-xodr.sha256",
      "# SHA256 of the generalization XODRs converted in the module: main (8a959e8; gen-pg = the prune-guard round's "
      "conversions) vs this change (gen-sw); both OPENBLAS_NUM_THREADS=1; '-' = not converted",
      [(S / "gen-pg" / n, S / "gen-sw" / n) for n in names],
      ["0411 and 0711 differ by this change (lane swaps fitted together on 0411 road 13 and 0711 road 11)."])
slow = sorted({p.name for p in (S / "gen-slow-pg").glob("shp-2023070710412621654.*")}
              | {p.name for p in (S / "gen-slow-sw").glob("shp-2023070710412621654.*")})
write("slow-junctions.sha256",
      "# SHA256 of the slow junction 0707104's outputs (converted without the module's 3600 s cap, OPENBLAS_NUM_THREADS=1 "
      "both times): main (8a959e8; gen-slow-pg of the prune-guard round) vs this change (gen-slow-sw)",
      [(S / "gen-slow-pg" / n, S / "gen-slow-sw" / n) for n in slow],
      ["061310 was not converted again: the lane_refit stage of its raw input finds no lane swap and writes the same",
       "roads (stage-diff.json)."])
