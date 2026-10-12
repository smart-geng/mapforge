"""Which roads differ between two XODRs, and whether only by userData. argv: a xodr, b xodr [, "geom" to ignore
the lane_refit userData]."""
import sys
from lxml import etree

ignore_ud = len(sys.argv) > 3 and sys.argv[3] == "geom"


def roads(path):
    root = etree.parse(path).getroot()
    out = {}
    for r in root.findall("road"):
        r = etree.fromstring(etree.tostring(r))
        if ignore_ud:
            for u in r.findall("userData[@code='mapforge.lane_refit/v1']"):
                r.remove(u)
        out[r.get("id")] = etree.tostring(r)
    rest = etree.fromstring(etree.tostring(root))
    for r in rest.findall("road"):
        rest.remove(r)
    return out, etree.tostring(rest)


a, ra = roads(sys.argv[1])
b, rb = roads(sys.argv[2])
diff = sorted((k for k in set(a) | set(b) if a.get(k) != b.get(k)), key=lambda x: int(x) if x.isdigit() else x)
print("roads", len(a), len(b), "differ:", diff, "| non-road content same:", ra == rb)
