"""Two-column hash lists (base, this change, name) of the scoreboard and generalization XODRs, with the single-thread
main conversions of the files that differ by the BLAS thread count as comment lines. argv: evidence dir."""
import hashlib, sys
from pathlib import Path
S = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/runs")
E = Path(sys.argv[1])
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else "-"
def write(name, head, pairs, single, tail):
    lines = [head]
    for base, new in pairs:
        lines.append(f"{sha(base)}  {sha(new)}  {new.name}")
    for line in tail:
        lines.append("# " + line)
    for p in single:
        lines.append(f"#   {sha(p)}  {p.parent.name}/{p.name}")
    (E / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((E / name).read_text())
board = [(S / ("sb-zs-" + pipe) / f"{pipe}-{case}.xodr", S / ("sb-rw-" + pipe) / f"{pipe}-{case}.xodr")
         for pipe in ("map", "shp") for case in ("NODE5", "node13", "node16", "node17", "node18", "node3", "node4")]
single_b = sorted((S / "sb-zs1-shp").glob("*.xodr")) if (S / "sb-zs1-shp").exists() else []
write("scoreboard-xodr.sha256",
      "# SHA256 of every scoreboard XODR: main (sb-zs-shp + sb-zs-map, d13d300) vs this change (sb-rw-shp + sb-rw-map, run as two parts)",
      board, single_b, sys.argv[2].split("|") if len(sys.argv) > 2 and sys.argv[2] else [])
gen = sorted({p.name for p in (S / "gen-zs").glob("shp-*.xodr")} | {p.name for p in (S / "gen-rw").glob("shp-*.xodr")})
single_g = sorted((S / "gen-zs1").glob("shp-*.xodr"))
write("generalization-xodr.sha256",
      "# SHA256 of the generalization XODRs converted in the module: main (gen-zs, d13d300) vs this change (gen-rw); '-' = not converted",
      [(S / "gen-zs" / n, S / "gen-rw" / n) for n in gen], single_g,
      sys.argv[3].split("|") if len(sys.argv) > 3 and sys.argv[3] else [])
