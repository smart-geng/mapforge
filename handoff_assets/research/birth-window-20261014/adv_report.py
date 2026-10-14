"""lane_refit.advance_births of the cwd tree on raw stage XODRs: per file the advance report and a hash of the
advanced tree; with the birth window (advance_births returns a pair) also the roads kept as their own sections would
advance them (own_way). argv: output json, raw xodr..."""
import hashlib, json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from lxml import etree
from mapforge.ops import lane_refit as LR

out = {}
for f in map(Path, sys.argv[2:]):
    root = etree.parse(str(f)).getroot()
    rep = LR.advance_births(root, LR._shp_source())
    own_way = None
    if isinstance(rep, tuple):
        rep, own_way = rep
    out[f.name] = {"sha": hashlib.sha256(etree.tostring(root)).hexdigest(), "report": rep}
    if own_way is not None:
        out[f.name]["own_way"] = {rid: {"sha": hashlib.sha256(etree.tostring(el)).hexdigest(), "events": evs}
                                  for rid, (el, evs) in own_way.items()}
    print(f.name, out[f.name]["sha"][:12], sum(1 for v in rep.values() for e in v if e.get("advanced_m")),
          sorted(own_way) if own_way is not None else "-", flush=True)
Path(sys.argv[1]).write_text(json.dumps(out, indent=1), encoding="utf-8")
