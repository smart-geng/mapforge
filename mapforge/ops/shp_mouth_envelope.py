"""SHP -> XODR with the junction mouth moved before the earliest real lane end.

The default CLI places each leg's common junction mouth at the farthest real lane end
("legacy-max-endpoint"). Where a junction edge is oblique, or an outer lane flares into
the corner curb return, the leg then runs into the junction area, connectors are squeezed
and the mouth carries strong lane flares that no connector can follow smoothly.

``build_junction_xodr`` already has an isolated candidate policy for this
(``mouth_policy="source-envelope-candidate"``, docs/复核报告-源几何保真与路口平滑-v1.35.md):
the common mouth goes ``margin_m`` before the earliest real lane end, every connector is
fitted to the real incoming-tail -> via -> outgoing-head chain, and the junction paving is
rebuilt from the source surface plus the real lane tails. This module only runs that path
with the CLI's own finalization (same G8 policy, same sidecars); it changes no converter
code. The junction paving is then rebuilt by mapforge.ops.envelope_surface (gap-aware: separate
carriageways are paved apart, gaps inside the source junction polygon are not holes); any other
unexplained gap still fails loudly and is never passed off as a result.

    python -m mapforge.ops.shp_mouth_envelope shp_0222-0326 --like v2x_map_xml/<case>.xml \
        --margin 3 -o out/<name>.xodr
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
G8_POLICY = ROOT / "profiles" / "validation" / "g8-opendrive-jinfeng-v1.yaml"
CODE = "mapforge.shp_mouth_envelope/v1"


def _jsonable(stats):
    return {k: v for k, v in stats.items() if k != "source_lane_manifest"}


def convert(shp_dir, like, out, margin_m=3.0, profile="ibd-smarteditor-v1", rebuild_surface=True, finalize=True,
            at=None):
    """``like`` (reference MAP XML) or ``at`` (lon, lat) locates the junction. ``finalize=False`` skips the
    gates and only writes the source manifest next to ``out`` (for a post-processing step that follows)."""
    import xml.etree.ElementTree as ET
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops import envelope_surface
    from mapforge.ops.shp_to_xodr import CandidateSurfaceError, _proj, build_junction_xodr
    from mapforge.report.decision import finalize_opendrive_g8

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    src = ProfileSource(str(shp_dir), profile)
    if like is not None:
        ref = parse_map_xml(str(like))
        lon, lat = ref.ref_lon, ref.ref_lat
    else:
        lon, lat = at
    junc, distance = src.find_junction(lon, lat)
    record = {"schema": CODE, "mouth_policy": "source-envelope-candidate", "margin_m": float(margin_m),
              "junction": junc.pid, "locator_distance_m": round(float(distance), 1)}
    try:
        stats = build_junction_xodr(src, junc, out, connect_mode="data", allow_uturn=False,
                                    mouth_policy="source-envelope-candidate", mouth_margin_m=float(margin_m))
        record["converter_surface"] = "GENERATED"
    except CandidateSurfaceError as exc:
        stats = exc.stats
        record["converter_surface"] = {"status": "FAIL", "error": stats.get("source_surface_error")}
        if not rebuild_surface:
            record.update(status="CANDIDATE_SURFACE_FAILED", stats=_jsonable(stats))
            out.with_suffix(".mouth-envelope.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
            raise
    if rebuild_surface:
        # one surface builder for every junction (also where the converter's own one succeeded)
        lon0, lat0 = float(junc.center[0]), float(junc.center[1])
        root = ET.parse(out).getroot()
        try:
            surface = envelope_surface.replace_source_paving(
                root, src, junc, lambda p: _proj(p, lat0, lon0), stats.get("mouth_envelope_decisions", []))
        except ValueError as exc:
            record.update(status="CANDIDATE_SURFACE_FAILED", surface_error=str(exc), stats=_jsonable(stats))
            out.with_suffix(".mouth-envelope.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
            raise
        record["surface"] = {k: v for k, v in surface.items() if k not in ("mouth_tail_source_components",)}
        if surface["source_surface_status"] != "GENERATED":
            record.update(status="CANDIDATE_SURFACE_FAILED", stats=_jsonable(stats))
            out.with_suffix(".mouth-envelope.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
            raise ValueError(f"envelope surface not generated: {surface['source_surface_status']}")
        ET.indent(root)
        ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=True)
    if finalize:
        final = finalize_opendrive_g8(out, stats["source_lane_manifest"], G8_POLICY, connect_mode="data")
        record.update(status="GENERATED", g8=final["gate"]["status"], g11=final["g11"]["status"],
                      decision=final["decision"]["status"], stats=_jsonable(stats))
    else:
        out.with_suffix(".source-lanes.json").write_text(
            json.dumps(stats["source_lane_manifest"], ensure_ascii=False, indent=1), encoding="utf-8")
        record.update(status="GENERATED", g8=None, g11=None, decision=None, stats=_jsonable(stats))
    out.with_suffix(".mouth-envelope.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    return record


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m mapforge.ops.shp_mouth_envelope")
    ap.add_argument("shp_dir", type=Path)
    ap.add_argument("--like", required=True, type=Path, help="现网参考 MAP XML（只用于定位路口）")
    ap.add_argument("--margin", type=float, default=3.0, help="共同口部放在最早真实车道端点之前的距离 [m]")
    ap.add_argument("-o", "--out", required=True, type=Path)
    a = ap.parse_args(argv)
    try:
        record = convert(a.shp_dir, a.like, a.out, a.margin)
    except Exception as exc:
        print(f"FAILED {type(exc).__name__}: {exc}"[:2000])
        return 3
    print(json.dumps({k: record[k] for k in ("junction", "margin_m", "status", "g8", "g11", "decision")},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
