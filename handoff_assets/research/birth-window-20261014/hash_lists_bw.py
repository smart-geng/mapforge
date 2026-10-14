"""Hash lists of the birth-window round: scoreboard and generalization XODRs and every output file of the two slow
junctions, four columns: main, the final version (keep_own_way with the curb-shoulder and source-deviation guards),
the third version (curb-shoulder guard only), the second version (redo_skipped, without the curb-shoulder guard; its
scoreboard and generalization runs were stopped part way, '-' = not converted). All were converted with
OPENBLAS_NUM_THREADS=1.
argv: evidence dir."""
import hashlib, sys
from pathlib import Path
S = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/runs")
E = Path(sys.argv[1])


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else "-"


def write(name, head, rows, tail):
    lines = [head] + [f"{sha(b)}  {sha(n)}  {sha(t)}  {sha(o)}  {n.name}" for b, n, t, o in rows] + ["# " + t for t in tail]
    (E / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((E / name).read_text())


cases = ("NODE5", "node13", "node16", "node17", "node18", "node3", "node4")
write("scoreboard-xodr.sha256",
      "# SHA256 of every scoreboard XODR: main (df7455b; sb-sw = the lane-swap round's conversions, the outputs "
      "main's code gives), the final version (sb-bw4), the third version (sb-bw3), the second version (sb-bw2, "
      "stopped part way); all OPENBLAS_NUM_THREADS=1",
      [(S / "sb-sw" / f"{p}-{c}.xodr", S / "sb-bw4" / f"{p}-{c}.xodr", S / "sb-bw3" / f"{p}-{c}.xodr",
        S / "sb-bw2" / f"{p}-{c}.xodr") for p in ("map", "shp") for c in cases],
      ["shp-node3, node4, node16, node17 and node18 have births/deaths only the window measures (new-events.txt);",
       "NODE5, node13 and the MAP conversions do not use it. The final version puts shp-node4 road 13 back."])
names = sorted({p.name for p in (S / "gen-sw").glob("shp-*.xodr")} | {p.name for p in (S / "gen-bw4").glob("shp-*.xodr")})
names = [n for n in names if not n.endswith((".raw.xodr", ".lane-refit.xodr"))]
write("generalization-xodr.sha256",
      "# SHA256 of the generalization XODRs converted in the module: main (df7455b; gen-sw = the lane-swap round's "
      "conversions), the final version (gen-bw4), the third version (gen-bw3), the second version (gen-bw2, stopped "
      "part way); all OPENBLAS_NUM_THREADS=1; '-' = not converted",
      [(S / "gen-sw" / n, S / "gen-bw4" / n, S / "gen-bw3" / n, S / "gen-bw2" / n) for n in names],
      ["0627 and 0630 have births only the window measures (new-events.txt); the final version puts 0630 road 11",
       "back."])
rows = []
for pid, base in (("2023070710412621654", "gen-slow-sw"), ("2023061310244790027", "gen-slow-base13")):
    for n in sorted({p.name for p in (S / base).glob(f"shp-{pid}.*")} | {p.name for p in (S / "gen-slow-bw4").glob(f"shp-{pid}.*")}):
        rows.append((S / base / n, S / "gen-slow-bw4" / n, S / "gen-slow-bw3" / n, S / "gen-slow-bw2" / n))
write("slow-junctions.sha256",
      "# SHA256 of the slow junctions' outputs (converted without the module's 3600 s cap, OPENBLAS_NUM_THREADS=1 every "
      "time): 0707104 main (df7455b; gen-slow-sw of the lane-swap round), 061310 main (df7455b, converted in this "
      "round: gen-slow-base13); then the final version (gen-slow-bw4), the third version (gen-slow-bw3) and the "
      "second version (gen-slow-bw2)",
      rows,
      ["Both have births only the window measures (new-events.txt); the third and the final version put 0707104",
       "road 10 and 061310 road 12 back as their own sections advance them (keep_own_way)."])
