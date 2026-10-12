"""The seam-rule records of the two junctions the rules converted: the surface record of the final
.mouth-envelope.json (0621: surface; 0630: leave_mouth.surface) and the written lanes whose provenance names the
rule. argv: generalization output dir, output json."""
import json, sys
import xml.etree.ElementTree as ET
from pathlib import Path

out_dir = Path(sys.argv[1])
CASES = {"2023062110304177600": ("surface",), "2023063015350832057": ("leave_mouth", "surface")}
result = {}
for pid, path in CASES.items():
    record = json.loads((out_dir / f"shp-{pid}.mouth-envelope.json").read_text(encoding="utf-8"))
    for key in path:
        record = record[key]
    lanes = []
    root = ET.parse(out_dir / f"shp-{pid}.xodr").getroot()
    for road in root.findall("road"):
        for k, sec in enumerate(road.findall("lanes/laneSection")):
            for lane in sec.iter("lane"):
                for ud in lane.findall("userData"):
                    value = ud.get("value") or ""
                    if "surface_seams" in value:
                        lanes.append({"road": road.get("id"), "section": k, "s": float(sec.get("s")),
                                      "lane": lane.get("id"), "type": lane.get("type"), "code": ud.get("code"),
                                      "value": json.loads(value) if value.startswith("{") else value})
    result[pid] = {"record_path": ".".join(path), "record": record, "lanes_naming_the_rule": lanes}
Path(sys.argv[2]).write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
for pid, r in result.items():
    print(pid, r["record_path"], r["record"].get("surface_builder_fallback"), len(r["lanes_naming_the_rule"]),
          sorted({(x["road"], x["lane"], x["type"]) for x in r["lanes_naming_the_rule"]}))
