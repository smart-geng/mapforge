"""Which <road> elements (and which of their children) differ between two XODRs. argv: a.xodr b.xodr"""
import sys
from lxml import etree
def roads(p):
    r = etree.parse(p).getroot()
    return {e.get("id"): e for e in r.findall("road")}, r
a, ra = roads(sys.argv[1]); b, rb = roads(sys.argv[2])
out = []
for rid in sorted(set(a) | set(b), key=lambda x: (len(x), x)):
    x, y = a.get(rid), b.get(rid)
    if x is None or y is None:
        out.append(f"{rid}: only in {'b' if x is None else 'a'}"); continue
    if etree.tostring(x) == etree.tostring(y): continue
    parts = [c.tag for c, d in zip(x, y) if etree.tostring(c) != etree.tostring(d)]
    if len(x) != len(y): parts.append("child count")
    out.append(f"{rid} (junction {x.get('junction')}): {', '.join(parts)}")
print("roads differing:", len(out)); print("\n".join(out))
other = [e.tag for e in ra if e.tag != "road"]
same = all(etree.tostring(p) == etree.tostring(q) for p, q in zip([e for e in ra if e.tag != "road"], [e for e in rb if e.tag != "road"]))
print("non-road elements identical:", same)
