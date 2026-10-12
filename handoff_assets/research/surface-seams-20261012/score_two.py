"""Scoreboard metrics and tiers of named generalization outputs (same policy as the module). cwd = code tree."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
import yaml
from lxml import etree
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate import scoreboard as sb
out = Path(sys.argv[1])
policy = yaml.safe_load(sb.POLICY.read_text(encoding="utf-8"))
schema = etree.XMLSchema(etree.parse(str(sb.XSD)))
src = ProfileSource(str(Path.cwd() / "shp_0222-0326"), "ibd-smarteditor-v1")
res = {}
for pid in sys.argv[2].split(","):
    m = sb.evaluate(out / f"shp-{pid}.xodr", "shp", schema, src)
    res[pid] = {"metrics": m, "tiers": sb.apply_tiers(m, "shp", policy)}
Path(sys.argv[3]).write_text(json.dumps(res, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
print("done")
