"""Byte comparison of two output folders: every file present in either, by SHA256 (intermediates excluded).
argv: base dir, new dir [, glob]. Prints identical / different / missing per file and a summary."""
import hashlib, sys
from pathlib import Path
base, new = Path(sys.argv[1]), Path(sys.argv[2])
pattern = sys.argv[3] if len(sys.argv) > 3 else "*"
def files(d):
    return {p.name: p for p in d.glob(pattern) if p.is_file() and ".raw." not in p.name and ".lane-refit." not in p.name}
def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
a, b = files(base), files(new)
counts = {}
for name in sorted(set(a) | set(b)):
    if name not in a: state = "only-new"
    elif name not in b: state = "only-base"
    else: state = "identical" if sha(a[name]) == sha(b[name]) else "DIFFERENT"
    counts[state] = counts.get(state, 0) + 1
    if state != "identical": print(f"{state:10s} {name}")
print("summary", counts)
