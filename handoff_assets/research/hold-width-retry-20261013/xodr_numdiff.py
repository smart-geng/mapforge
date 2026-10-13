"""Largest numeric attribute difference between two XODRs with the same element structure (else reports where the
structure first differs). argv: a.xodr b.xodr"""
import sys
from lxml import etree
a, b = (etree.parse(p).getroot() for p in sys.argv[1:3])
ea, eb = list(a.iter()), list(b.iter())
if len(ea) != len(eb):
    print("element count", len(ea), len(eb))
worst, where, nattr = 0.0, None, 0
for x, y in zip(ea, eb):
    if x.tag != y.tag:
        print("structure differs at", x.tag, y.tag, x.sourceline); break
    for k, v in x.attrib.items():
        w = y.attrib.get(k)
        if w == v: continue
        try:
            d = abs(float(v) - float(w))
        except (TypeError, ValueError):
            print("text attr differs", x.tag, k, v[:60], (w or "")[:60]); continue
        nattr += 1
        if d > worst: worst, where = d, (x.tag, k, v, w, x.sourceline)
print("differing numeric attributes:", nattr, "max abs diff:", worst, where)
