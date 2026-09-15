"""冻结当前几何复核证据，不把带排除项的技术 PASS 写成来源完全复原。"""
from __future__ import annotations

import hashlib
import json
import statistics
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASES = ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")
FOCUS = {"node18": "2024010516362469683", "node3": "2024010417201931641",
         "node13": "2024010415270546470"}


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    closed = read(ROOT / "out/closed-loop-report.json")
    raw_path = ROOT / "out/unclipped-via-audit-v135.json"
    raw = read(raw_path) if raw_path.exists() else None
    rows = []
    for pipeline, folder in (("shp", "direct_xodr"), ("map", "m2x")):
        for case in CASES:
            path = ROOT / "out" / folder / f"{case}.xodr"
            root = ET.parse(path).getroot()
            g8 = read(path.with_suffix(".g8.json"))
            g11 = read(path.with_suffix(".g11.json"))
            decision = read(path.with_suffix(".delivery-decision.json"))
            groups = {name: [] for name in ("ordinary", "connecting", "paving")}
            unmet = []
            focus = None
            for road in root.findall("road"):
                group = ("paving" if road.get("name") == "junction_paving" else
                         "ordinary" if road.get("junction") == "-1" else "connecting")
                geometry = road.findall("planView/geometry")
                lengths = [float(g.get("length")) for g in geometry]
                record = {"road_id": road.get("id"), "length_m": float(road.get("length")),
                          "segments_m": lengths}
                groups[group].append(record)
                for lane in road.findall(".//lane"):
                    ud = lane.find("userData[@code='mapforge.provenance/v1']")
                    if ud is None:
                        continue
                    provenance = json.loads(ud.get("value", "{}"))
                    code = provenance.get("exclusion_code")
                    if group == "connecting" and code in {
                            "minimal-chain-source-fidelity-unmet", "source-end-state-conflict"}:
                        source = lane.find("userData[@code='mapforge.source_lane']")
                        unmet.append({**record, "source_lane_id": (
                            source.get("value") if source is not None else None),
                            "reason": code, "evidence": provenance.get("source_conflict")})
            if pipeline == "shp" and case in FOCUS:
                row = next(x for x in g8["per_lane"] if x["source_lane_id"] == FOCUS[case])
                road = next(x for x in groups["connecting"]
                            if x["road_id"] == row["target"]["road_id"])
                focus = {**road, "source_lane_id": FOCUS[case],
                         "source_to_target": row["source_to_target"],
                         "target_to_source": row["target_to_source"]}
            summaries = {}
            for name, items in groups.items():
                lengths = [v for r in items for v in r["segments_m"]]
                summaries[name] = {
                    "roads": len(items), "segments": len(lengths),
                    "max_segments_per_road": max((len(r["segments_m"]) for r in items), default=0),
                    "min_m": min(lengths, default=None),
                    "median_m": statistics.median(lengths) if lengths else None,
                    "segments_under_5m": sum(v < 5 for v in lengths),
                }
            rows.append({"case": case, "pipeline": pipeline,
                         "artifact": str(path.relative_to(ROOT)),
                         "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                         "g8": {"status": g8["status"], "metrics": g8["metrics"]},
                         "g11": {"status": g11["status"], "level": g11["level"]},
                         "delivery": decision, "segments": summaries,
                         "focus": focus, "connecting_review_items": unmet})
    hashes = {r["case"]: r["sha256"] for r in rows if r["pipeline"] == "shp"}
    raw_is_current = raw is not None and hashes == {
        r["case"]: r["sha256"] for r in raw.get("artifacts", [])}
    snapshot = {
        "schema": "mapforge/geometry-review-snapshot/v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "technical_closed_loop_status": closed["status"],
        "source_restoration_complete": bool(raw_is_current
            and raw["needs_shape_review"] == 0
            and not any(r["connecting_review_items"] for r in rows)),
        "unclipped_via_audit": {"current": raw_is_current,
            "report": str(raw_path.relative_to(ROOT)),
            "needs_shape_review": raw["needs_shape_review"] if raw_is_current else None},
        "vehicle_kinematic_validation": "not-performed; G11 checks curvature/speed proxies only",
        "notes": ["技术 PASS 包含显式来源排除项，不等于完全保真。",
                  "拟合未达标不能单凭算法失败归因于源数据错误。",
                  "MAP 缺出口/路口几何的镜像和连接属于推断，不能声称有测量真值。"],
        "files": rows,
    }
    output = ROOT / "out/geometry-review-v135.json"
    output.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2, allow_nan=False),
                      encoding="utf-8")
    print(output)
    print(f"{len(rows)} files; technical={closed['status']}; "
          f"source_restoration_complete={snapshot['source_restoration_complete']}")


if __name__ == "__main__":
    main()
