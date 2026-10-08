"""Read-only admission upper bound for the already fixed 0707 WB08 holdout.

No source fitting, editable-window search, targets, candidates or threshold
changes. The frozen straight-road editor is only queried for lane identity;
the separate length proof does NOT grant curved-road admission.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.repair_web.model import intervals, lanes, parse
from mapforge.workbench import boundary_probe as B

CASE = "2023070710324132429"
FIXED_END_CLEARANCE_M = 5.0  # Existing boundary_probe._admit, not a new rule.
REQUIRED_KNOT_COUNT = 5      # Existing boundary_probe._admit.
PREVIOUS = ROOT / "out/workbench/wb08-holdout-scope-audit-20261008-v2/report.json"
SOURCE = ROOT / "shp_0222-0326"
PROFILE = ROOT / "profiles/shp/ibd-smarteditor-v1.yaml"


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def manifest(paths):
    return {str(p.resolve()): {"sha256": sha(p), "size": p.stat().st_size}
            for p in sorted(paths)}


def lane_identity(lane, section_start):
    try:
        if lane.get("type") != "driving" or lane.findall("border") or not lane.findall("width"):
            raise B.BoundaryEditRejected("not width-based driving lane")
        sid = B._identity(lane)
        cuts = [section_start + float(w.get("sOffset")) for w in lane.findall("width")]
        if cuts[0] != section_start or any(b <= a for a, b in zip(cuts, cuts[1:])):
            raise B.BoundaryEditRejected("invalid width record order")
        return {"eligible_ordinary_identity": True, "source_lane_id": sid,
                "width_stations_m": cuts}
    except B.BoundaryEditRejected as exc:
        return {"eligible_ordinary_identity": False, "reason": str(exc)}


def inspect(data):
    roads, spans = [], []
    for road in parse(data).findall("road"):
        rid, length = road.get("id"), float(road.get("length"))
        ordinary = road.get("junction") == "-1" and road.get("name") != "junction_paving"
        row = {"road_id": rid, "name": road.get("name"), "junction": road.get("junction"),
               "length_m": length, "ordinary_road": ordinary,
               "geometry": [{**g.attrib, "type": g[0].tag, "parameters": dict(g[0].attrib)}
                            for g in road.findall("planView/geometry")], "sections": []}
        active_spans = {}
        if ordinary:
            for section, lo, hi in intervals(road):
                infos = {lid: lane_identity(lane, lo) for lid, lane in lanes(section).items()}
                eligible = {lid for lid, info in infos.items() if info["eligible_ordinary_identity"]}
                pairs = [(lid, lid + (1 if lid > 0 else -1)) for lid in sorted(eligible)
                         if lid + (1 if lid > 0 else -1) in eligible]
                row["sections"].append({"start_m": lo, "end_m": hi,
                    "lanes": [{"lane_id": lid, **info} for lid, info in sorted(infos.items())],
                    "ordinary_trusted_adjacent_pairs": pairs})
                next_active = {}
                for pair in pairs:
                    # Deliberately optimistic: identity-qualified adjoining
                    # sections may continue even before source-chain/link
                    # checks. A failed upper bound cannot hide a viable scope.
                    item = active_spans.get(pair)
                    if item is None or item["raw_interval_m"][1] != lo:
                        item = {"road_id": rid, "lane_ids": list(pair), "raw_interval_m": [lo, hi],
                                "source_lane_ids_by_section": []}
                        spans.append(item)
                    else:
                        item["raw_interval_m"][1] = hi
                    item["source_lane_ids_by_section"].append({"section_start_m": lo,
                        "source_lane_ids": [infos[lid]["source_lane_id"] for lid in pair]})
                    start = max(FIXED_END_CLEARANCE_M, item["raw_interval_m"][0])
                    end = min(length - FIXED_END_CLEARANCE_M, hi)
                    item.update(allowed_interval_upper_bound_m=[start, end],
                                allowed_length_upper_bound_m=max(0.0, end - start),
                                minimum_required_support_m=(REQUIRED_KNOT_COUNT - 1) * B.MIN_NEW_SPAN_M)
                    item["enough_length"] = item["allowed_length_upper_bound_m"] >= item["minimum_required_support_m"]
                    next_active[pair] = item
                active_spans = next_active
        roads.append(row)
    return roads, spans


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    out = parser.parse_args().out.resolve()
    if out.exists():
        parser.error("choose a fresh output directory; prior evidence is immutable")
    previous = json.loads(PREVIOUS.read_text(encoding="utf8"))
    fixed = next(c for c in previous["cases"] if c["junction_id"] == CASE)
    old0418 = next(c for c in previous["cases"] if c["junction_id"] == "2023041810414040830")
    baseline = ROOT / fixed["baseline_path"]
    old_baseline = ROOT / old0418["baseline_path"]
    source_files = sorted(p for p in SOURCE.rglob("*") if p.is_file())
    bound_paths = [PREVIOUS, baseline, old_baseline, PROFILE, *source_files]
    code_paths = [Path(__file__).resolve(), ROOT / "mapforge/workbench/boundary_probe.py",
        ROOT / "mapforge/workbench/compiler.py", ROOT / "mapforge/repair_web/model.py",
        ROOT / "mapforge/adapters/shp/profile_source.py", ROOT / "mapforge/adapters/shp/ibd_reader.py",
        ROOT / "scripts/internal_edge_jets.py", ROOT / "mapforge/validate/smoothness.py",
        ROOT / "mapforge/validate/g11.py", ROOT / "uv.lock"]
    before, code_before = manifest(bound_paths), manifest(code_paths)
    if sha(baseline) != fixed["baseline_sha256"] or sha(old_baseline) != old0418["baseline_sha256"]:
        raise ValueError("fixed baseline bytes no longer match the prior holdout report")
    roads, spans = inspect(baseline.read_bytes())
    src = ProfileSource(str(SOURCE), str(PROFILE))
    source_bindings = []
    for span in spans:
        for identities in span["source_lane_ids_by_section"]:
            lane_ids = identities["source_lane_ids"]
            relations = [src.lane_boundary_records(sid) for sid in lane_ids]
            common = set(r["boundary_id"] for r in relations[0]) & set(r["boundary_id"] for r in relations[1])
            source_bindings.append({"road_id": span["road_id"], "lane_ids": span["lane_ids"],
                **identities, "lane_records": [src.lane_raw_records(sid) for sid in lane_ids],
                "boundary_relations": relations, "common_boundary_ids": sorted(common),
                "source_binding_is_not_admission": True})
    source_after = sorted(p for p in SOURCE.rglob("*") if p.is_file())
    after = manifest([PREVIOUS, baseline, old_baseline, PROFILE, *source_after])
    code_after = manifest(code_paths)
    unchanged = before == after and code_before == code_after
    report = {"schema": "mapforge/workbench-curve-scope-audit/v1", "run_kind": "READ_ONLY_FIXED_HOLDOUT_ADMISSION_UPPER_BOUND",
        "junction_id": CASE, "baseline_path": str(baseline), "baseline_sha256": sha(baseline),
        "previous_report_sha256": sha(PREVIOUS), "all_roads_scanned": len(roads),
        "ordinary_roads_scanned": sum(r["ordinary_road"] for r in roads), "roads": roads,
        "ordinary_trusted_pair_spans": spans, "source_bindings": source_bindings,
        "existing_rules": {"knot_count": REQUIRED_KNOT_COUNT, "minimum_new_span_m": B.MIN_NEW_SPAN_M,
            "required_total_support_m": (REQUIRED_KNOT_COUNT - 1) * B.MIN_NEW_SPAN_M,
            "road_end_clearance_m": FIXED_END_CLEARANCE_M, "maximum_new_width_records": B.MAX_NEW_WIDTH_RECORDS,
            "normal_delta_limit_m": B.MAX_NORMAL_DELTA_M},
        "maximum_allowed_length_upper_bound_m": max((s["allowed_length_upper_bound_m"] for s in spans), default=0.0),
        "length_upper_bound_is_optimistic": True,
        "additional_constraints_not_used_to_shrink_upper_bound": ["explicit lane-section links", "source-chain continuity and unique normal intersection", "source coverage", "new knot splits >=6m", "maximum 8 new width records", "source residual improvement"],
        "conclusion": "REJECTED_INSUFFICIENT_TRUSTED_ORDINARY_SPAN" if spans and not any(s["enough_length"] for s in spans) else "FURTHER_READ_ONLY_REVIEW_REQUIRED",
        "representation_also_unsupported_by_current_tool": True,
        "fixed_0418_preserved": {"baseline_sha256": sha(old_baseline), "capability": old0418["capability"],
            "prior_outcome": old0418["outcome"], "window_changed": False},
        "source_file_count": len(source_files), "inputs_before": before, "inputs_after": after,
        "code_before": code_before, "code_after": code_after, "all_bound_bytes_unchanged": unchanged,
        "candidate_generated_count": 0, "targets_proposed_m": [], "sampling_window_reselected": False,
        "guard_modified": False, "independent_customer_operator": False, "wb08_positive_holdout_achieved": False,
        "python": platform.python_version(),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()}
    if not unchanged:
        report["conclusion"] = "REJECTED_INPUT_DRIFT"
    out.mkdir(parents=True, exist_ok=False)
    for path in code_paths:
        target = out / "code-snapshots" / path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(path.read_bytes())
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf8")
    print(json.dumps({k: report[k] for k in ("conclusion", "all_roads_scanned", "ordinary_roads_scanned",
        "maximum_allowed_length_upper_bound_m", "source_file_count", "all_bound_bytes_unchanged", "candidate_generated_count")}))
    return 0 if unchanged and report["conclusion"] == "REJECTED_INSUFFICIENT_TRUSTED_ORDINARY_SPAN" else 1


if __name__ == "__main__":
    raise SystemExit(main())
