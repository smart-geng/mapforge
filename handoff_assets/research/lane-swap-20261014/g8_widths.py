"""Widths of given lanes in given laneSections of a road (XODR width polynomials), sampled as G8 samples a section
(1 m steps from the section start plus the point just before its end), with the samples wider than 0.05 m.
argv: xodr, road id, 'section:lane,...'."""
import sys
import numpy as np
from lxml import etree

root = etree.parse(sys.argv[1]).getroot()
road = next(r for r in root.iter("road") if r.get("id") == sys.argv[2])
secs = road.find("lanes").findall("laneSection")
ends = [float(s.get("s")) for s in secs[1:]] + [float(road.get("length"))]
for item in sys.argv[3].split(","):
    k, lane_id = (int(x) for x in item.split(":"))
    sec = secs[k]
    s0, s1 = float(sec.get("s")), ends[k]
    lane = next(l for l in sec.iter("lane") if int(l.get("id")) == lane_id)
    polys = [(float(w.get("sOffset")), *(float(w.get(c)) for c in "abcd")) for w in lane.findall("width")]

    def width(ds):
        o, a, b, c, d = [p for p in polys if p[0] <= ds + 1e-9][-1]
        u = ds - o
        return a + b * u + c * u * u + d * u ** 3

    q = np.arange(s0, s1, 1.0)
    end = np.nextafter(s1, s0)
    if end - q[-1] > 0.05:
        q = np.append(q, end)
    w = np.array([width(x - s0) for x in q])
    fine = np.linspace(s0, s1, 2001)
    wf = np.array([width(x - s0) for x in fine])
    wide = [(round(x, 2), round(y, 3)) for x, y in zip(q, w) if y > 0.05]
    print(f"section {k} s {s0:.2f}-{s1:.2f} lane {lane_id}: max width {wf.max():.3f} m at s {fine[wf.argmax()]:.2f}; "
          f"> 0.05 m over s {fine[wf > 0.05].min() if (wf > 0.05).any() else '-'}-{fine[wf > 0.05].max() if (wf > 0.05).any() else '-'}; "
          f"G8 samples > 0.05 m: {wide}")
