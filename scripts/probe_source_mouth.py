"""隔离试验：来源包络口部 + 真实拓扑来源链；不覆盖正式产物。"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES, POLICY, shp_source
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops.shp_to_xodr import build_junction_xodr
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.smoothness import audit_file, surface_continuity
import xml.etree.ElementTree as ET


def main(case, margin=3.0, folder=None):
    name = dict(CASES)[case]
    src = shp_source()
    ref = parse_map_xml(str(ROOT / "v2x_map_xml" / name))
    junc, distance = src.find_junction(ref.ref_lon, ref.ref_lat)
    assert distance < 50
    folder = Path(folder) if folder else ROOT / "out/mouth-candidate"
    folder.mkdir(exist_ok=True)
    output = folder / f"{case}.xodr"
    try:
        stats = build_junction_xodr(src, junc, output,
                                    mouth_policy="source-envelope-candidate",
                                    mouth_margin_m=margin)
    except Exception as exc:
        failed_stats = getattr(exc, "stats", {})
        if failed_stats.get("source_lane_manifest"):
            output.with_suffix(".source-lanes.json").write_text(json.dumps(
                failed_stats["source_lane_manifest"], ensure_ascii=False, indent=2), encoding="utf-8")
        (folder / f"{case}.probe.json").write_text(json.dumps({
            "candidate_only": True, "production_promoted": False,
            "case": case, "margin_m": margin, "status": "GENERATION_FAILED",
            "stats":{k:v for k,v in failed_stats.items() if k != "source_lane_manifest"},
            "error": f"{type(exc).__name__}: {exc}"}, indent=2), encoding="utf-8")
        raise
    final = finalize_opendrive_g8(output, stats["source_lane_manifest"], POLICY,
                                 connect_mode="data")
    surface = surface_continuity(ET.parse(output).getroot())
    result = {"candidate_only": True, "production_promoted": False,
              "stats": {k: v for k, v in stats.items() if k != "source_lane_manifest"},
              "g8": final["gate"]["status"], "surface": surface,
              "smoothness": audit_file(output)}
    (folder / f"{case}.probe.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"case": case, "g8": result["g8"],
                      "fallback_count": stats.get("conn_minimal_excluded", 0),
                      "composite_count": stats.get("conn_source_composite", 0),
                      "mouths": stats.get("mouth_envelope_decisions"),
                      "surface": surface}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("case", choices=dict(CASES))
    p.add_argument("--margin", type=float, default=3.0)
    p.add_argument("--folder", type=Path)
    args = p.parse_args()
    main(args.case, args.margin, args.folder)
