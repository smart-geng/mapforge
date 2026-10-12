"""Source facts of the 0630 road 13 lanes: raw lane attributes (widths, link, predecessors), centre-line and boundary
end points in the XODR's local frame. cwd = code tree. argv: xodr (for the origin), comma-separated lane pids,
output json."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import xml.etree.ElementTree as ET
import numpy as np
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import shp_boundary_fidelity as B

xodr, pids, jout = Path(sys.argv[1]), sys.argv[2].split(","), Path(sys.argv[3])
lat0, lon0 = B._origin(ET.parse(xodr).getroot())
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")
manifest = json.loads(xodr.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
centres = {l["source_lane_id"]: l for l in manifest.get("lanes", [])}
topo_in, topo_out = src.topo_in, src.topo_out
out = {}
for pid in pids:
    raw = src.lane_raw_records(pid)
    attrs = {}
    for rec in raw:
        for k, v in (rec.get("attributes") or rec).items():
            if k.upper() in ("S_WIDTH", "E_WIDTH", "LINK_PID", "LANE_NO", "SEQ_NUM", "LANE_TYPE", "DIRECTION"):
                attrs[k] = v
    lane = src.lane(pid)
    c = np.asarray((centres.get(pid, {}).get("geometry") or {}).get("coordinates", []), float)
    bounds = [B._project(g, lat0, lon0) for g in src.lane_boundary_geometries(pid)]
    out[pid] = {
        "attributes": attrs,
        "link": getattr(lane, "link_pid", None),
        "predecessors": topo_in.get(pid, []), "successors": topo_out.get(pid, []),
        "centre_first": [round(float(x), 2) for x in c[0, :2]] if len(c) else None,
        "centre_last": [round(float(x), 2) for x in c[-1, :2]] if len(c) else None,
        "boundary_firsts": [[round(float(x), 2) for x in b[0, :2]] for b in bounds],
        "boundary_lasts": [[round(float(x), 2) for x in b[-1, :2]] for b in bounds],
    }
jout.write_text(json.dumps(out, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
print(json.dumps(out, ensure_ascii=False, default=str, indent=1))
