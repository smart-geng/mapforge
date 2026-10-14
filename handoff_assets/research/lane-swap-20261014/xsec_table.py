"""Per s range, the largest deviation of the written outer edge from the source's outermost boundary on the same normal
(cross_sections.py output, before and after), and over a swap range the largest deviation of the written divider (the
second edge from outside) from the source divider (the second distinct source crossing from outside).
argv: before json, after json, side (right|left), ranges 'a-b,c-d,...', swap 'a-b'."""
import json, sys

before, after = (json.load(open(p, encoding="utf-8")) for p in sys.argv[1:3])
side, ranges = sys.argv[3], [tuple(float(x) for x in r.split("-")) for r in sys.argv[4].split(",")]
swap = tuple(float(x) for x in sys.argv[5].split("-"))
out = 1.0 if side == "left" else -1.0          # sign of t outward


def edges(row):
    ts = sorted({round(e["t"], 3) for e in row["written_right_edges"] if e["t"] * out > 0}, key=lambda t: -t * out)
    src = sorted({round(c["t"], 3) for c in row["source_boundaries"] if c["t"] * out > 0}, key=lambda t: -t * out)
    return ts, src


def dev(rows, lo, hi, k):
    """Largest |written - source| of the k-th edge from outside over stations in [lo, hi): (inward signed, s)."""
    best = None
    for row in rows:
        if not lo <= row["s"] < hi:
            continue
        ts, src = edges(row)
        if len(ts) <= k or len(src) <= k:
            continue
        d = (src[k] - ts[k]) * out                 # > 0: the written edge lies inward of the source
        if best is None or abs(d) > abs(best[0]):
            best = (round(d, 3), row["s"])
    return best


def fmt(x):
    return "-" if x is None else f"{'向内' if x[0] > 0 else '向外'} {abs(x[0]):.2f} (s {x[1]})"


print("outer edge (inward = toward the reference line)")
for lo, hi in ranges:
    print(f"  {lo:6.1f}-{hi:6.1f}  before {fmt(dev(before, lo, hi, 0)):24s} after {fmt(dev(after, lo, hi, 0))}")
print("divider over the swap")
print(f"  {swap[0]:6.1f}-{swap[1]:6.1f}  before {fmt(dev(before, *swap, 1)):24s} after {fmt(dev(after, *swap, 1))}")
