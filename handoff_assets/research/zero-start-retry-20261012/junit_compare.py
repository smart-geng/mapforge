"""Per-test-id comparison of two pytest junit XML files. argv: base xml, change xml."""
import sys
import xml.etree.ElementTree as ET
from collections import Counter


def states(path):
    out = {}
    for tc in ET.parse(path).getroot().iter("testcase"):
        st = "passed"
        for tag in ("failure", "error", "skipped"):
            if tc.find(tag) is not None:
                st = tag
        out[f"{tc.get('classname')}::{tc.get('name')}"] = st
    return out


a, b = states(sys.argv[1]), states(sys.argv[2])
print("base  ", dict(Counter(a.values())))
print("change", dict(Counter(b.values())))
diff = Counter()
for k in sorted(set(a) | set(b)):
    sa, sb = a.get(k, "absent"), b.get(k, "absent")
    if sa != sb:
        diff[(sa, sb, k.split("::")[0])] += 1
for (sa, sb, cls), n in sorted(diff.items()):
    print(f"{n:4d}  {cls}: {sa} -> {sb}")
print("same state:", sum(1 for k in set(a) & set(b) if a[k] == b[k]), "of", len(set(a) & set(b)), "common ids")
