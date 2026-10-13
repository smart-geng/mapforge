"""The lane_refit input of an SHP junction (the pipeline's *.raw.xodr: shp_mouth_envelope.convert, then
shp_leave_mouth.apply_file), as pipeline.convert_shp builds it, kept as <out dir>/<name>.raw.xodr with its
.raw.source-lanes.json. cwd = code tree. argv: out dir, name, "like=<MAP xml>" or "at=<lon>,<lat>"."""
import json, sys
from pathlib import Path
ROOT = Path.cwd(); sys.path.insert(0, str(ROOT))
from mapforge import pipeline
from mapforge.ops import shp_leave_mouth, shp_mouth_envelope

out, name, where = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
out.mkdir(parents=True, exist_ok=True)
shp, profile = ROOT / "shp_0222-0326", "ibd-smarteditor-v1"
like, at = None, None
if where.startswith("like="):
    like = Path(where[5:])
else:
    at = tuple(float(x) for x in where[3:].split(","))
raw = out / f"{name}.raw.xodr"
record = shp_mouth_envelope.convert(shp, like, raw, pipeline.SHP_MOUTH_MARGIN_M, profile, finalize=False, at=at)
manifest = json.loads(raw.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
if like is not None:
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    ref = parse_map_xml(str(like))
    locate = (ref.ref_lon, ref.ref_lat)
else:
    locate = at
shp_leave_mouth.apply_file(raw, manifest, shp, profile, locate, record["stats"].get("mouth_envelope_decisions", []))
pipeline._dump(raw.with_suffix(".source-lanes.json"), manifest)
print(raw)
