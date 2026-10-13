"""Compare two output folders file by file after replacing the new tree/run names in the new files with the base
ones (paths in logs and reports). argv: base dir, new dir, 'new=base' replacements (comma separated)[, glob].
Prints per junction: XODR identical or not, and the files differing after the replacement."""
import hashlib, sys
from collections import defaultdict
from pathlib import Path
base, new = Path(sys.argv[1]), Path(sys.argv[2])
subs = [tuple(x.split("=")) for x in sys.argv[3].split(",")]
pattern = sys.argv[4] if len(sys.argv) > 4 else "shp-*"
def files(d):
    return {p.name: p for p in d.glob(pattern) if p.is_file() and ".raw." not in p.name and ".lane-refit." not in p.name}
a, b = files(base), files(new)
rows = defaultdict(list)
for name in sorted(set(a) | set(b)):
    pid = name.split(".")[0]
    if name not in a: rows[pid].append(f"only-new:{name}"); continue
    if name not in b: rows[pid].append(f"only-base:{name}"); continue
    x, y = a[name].read_bytes(), b[name].read_bytes()
    if x == y: continue
    for n, o in subs:
        y = y.replace(n.encode(), o.encode())
    rows[pid].append(("path-only:" if x == y else "DIFF:") + name.split(".", 1)[1])
for pid in sorted({n.split(".")[0] for n in set(a) | set(b)}):
    print(pid, rows[pid] or "all identical")
