"""Hash lists of the prune-guard round: scoreboard and generalization XODRs (base, this change, name) and every output
file of the slow junction 0707104, with the single-thread main conversions as comment lines. argv: evidence dir."""
import hashlib, sys
from pathlib import Path
S = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/runs")
E = Path(sys.argv[1])
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else "-"
def write(name, head, pairs, tail, single):
    lines = [head] + [f"{sha(b)}  {sha(n)}  {n.name}" for b, n in pairs] + ["# " + t for t in tail]
    lines += [f"#   {sha(p)}  {p.parent.name}/{p.name}" for p in single]
    (E / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((E / name).read_text())
cases = ("NODE5", "node13", "node16", "node17", "node18", "node3", "node4")
write("scoreboard-xodr.sha256",
      "# SHA256 of every scoreboard XODR: main (cce0bee; sb-base11 = the hold-width round's sb-rw-shp + sb-rw-map, "
      "the outputs main's code gives) vs this change (sb-pg, OPENBLAS_NUM_THREADS=1)",
      [(S / "sb-base11" / f"{p}-{c}.xodr", S / "sb-pg" / f"{p}-{c}.xodr") for p in ("map", "shp") for c in cases],
      ["The base was converted with the default BLAS threads, SHP node18 single-thread. SHP node3 differs by the thread",
       "count: main converted the same way (sb-bi1, single thread) gives the same XODR as this change:"],
      [S / "sb-bi1" / "shp-node3.xodr"])
names = sorted({p.name for p in (S / "gen-base11").glob("shp-*.xodr")} | {p.name for p in (S / "gen-pg").glob("shp-*.xodr")})
write("generalization-xodr.sha256",
      "# SHA256 of the generalization XODRs converted in the module: main (cce0bee; gen-base11 = the hold-width round's "
      "gen-rw) vs this change (gen-pg, OPENBLAS_NUM_THREADS=1); '-' = not converted",
      [(S / "gen-base11" / n, S / "gen-pg" / n) for n in names],
      ["0411, 0627 and 0711 differ by this change (the prune guard is used on 0411 road 13, 0627 road 13, 0711 road 11).",
       "The base converted 0627 with the default BLAS threads, the others single-thread; main converted 0627",
       "single-thread (gen-bi1) gives the base's XODR, so 0627 does not depend on the thread count:"],
      [S / "gen-bi1" / "shp-2023062713394062899.xodr"])
slow = sorted({p.name for p in (S / "gen-slow-rw").glob("shp-2023070710412621654.*")}
              | {p.name for p in (S / "gen-slow-pg").glob("shp-2023070710412621654.*")})
write("slow-junctions.sha256",
      "# SHA256 of the slow junction 0707104's outputs (converted without the module's 3600 s cap, OPENBLAS_NUM_THREADS=1 "
      "both times): main (cce0bee; gen-slow-rw of the hold-width round) vs this change (gen-slow-pg)",
      [(S / "gen-slow-rw" / n, S / "gen-slow-pg" / n) for n in slow],
      ["The XODR differs in road 13 (the prune guard, 0.73 -> 0.005 m) and connectors 138, 139, 140; 061310 was not",
       "converted again: the lane_refit stage of its raw input uses no guard and writes the same roads (stage-diff-pg.json)."],
      [])
