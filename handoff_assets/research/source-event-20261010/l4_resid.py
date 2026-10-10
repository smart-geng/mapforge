import json, sys, numpy as np, xml.etree.ElementTree as ET
sys.path.insert(0, ".")
from mapforge.validate.smoothness import lane_edges_at
root = ET.parse(sys.argv[1]).getroot()
road = next(r for r in root.findall("road") if r.get("id") == "11")
d = json.load(open("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/bnd/road11-src-st.json"))
b0 = np.asarray(d["4|2023061413420010456"][0]); b0 = b0[np.argsort(b0[:, 0])]
rows = []
for s in np.arange(20, 64.01, 0.5):
    e = lane_edges_at(road, s, "left")
    rows.append((s, np.interp(s, b0[:, 0], b0[:, 1]) - e[-1]))
r = np.array(rows)
print("L4 outer residual s20-64: max|r| %.3f  n>0.10 %d  mean|r| %.3f" % (np.abs(r[:, 1]).max(), int((np.abs(r[:, 1]) > 0.10).sum()), np.abs(r[:, 1]).mean()))
print(" ".join(f"{s:.0f}:{v:+.3f}" for s, v in r[::4]))
