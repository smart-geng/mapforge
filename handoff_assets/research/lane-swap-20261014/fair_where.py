"""Where the lane-centre fairness peaks sit (lane_fairness.tracks_by_class): per class, the three tracks with the largest
|curvature|, with road, lane, section range, the s of the peak and the width range. argv: xodr..."""
import sys
import numpy as np
from lxml import etree
from mapforge.validate.lane_fairness import tracks_by_class

for x in sys.argv[1:]:
    print("==", x.split("/")[-2], x.split("/")[-1])
    groups = tracks_by_class(etree.parse(x).getroot())
    for name in ("road", "road_event"):
        rows = sorted(groups.get(name, []), key=lambda t: -float(np.max(np.abs(t["kappa"]))))[:3]
        for t in rows:
            k = np.abs(t["kappa"])
            print(f"  {name:10s} road {t['road']:>4s} lane {t['lane']:>3d} s {t['s0']:7.2f}-{t['s1']:7.2f} "
                  f"kappa {k.max():.4f} (sample {int(k.argmax())}) width {t['width_min']:.2f}-{t['width_max']:.2f}"
                  + (f" sharp {float(np.max(t['sharpness'])):.4f}" if len(t["sharpness"]) else ""))
