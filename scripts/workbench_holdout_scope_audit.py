"""Read-only WB-08 preflight for two fixed, previously unedited junctions.

No candidate is generated. The first scope is determined from source roles and
the first existing width break, before inspecting its residual. If a fixed
endpoint realizes the baseline maximum, the current strict improvement guard
cannot admit this scope. The script records that fact rather than moving the
measurement window, trying targets, or weakening the guard.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from mapforge.repair_web.model import intervals, lanes, parse, shifted
from mapforge.workbench.boundary_probe import (
    _identity,
    _source_samples,
    _stations,
    build_source_binding,
    prepare_boundary_edit,
    source_residuals,
)
from scripts.internal_edge_jets import states

JUNCTIONS = ("2023041810414040830", "2023070710324132429")
RUN = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3"


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def road_summary(root):
    result = []
    for road in root.findall("road"):
        if road.get("junction") != "-1" or road.get("name") == "junction_paving":
            continue
        geometries = road.findall("planView/geometry")
        row = {"id": road.get("id"), "name": road.get("name"),
               "length_m": float(road.get("length")),
               "geometry_types": [g[0].tag for g in geometries], "sections": []}
        for section, lo, hi in intervals(road):
            lane_rows = []
            for lid, lane in lanes(section).items():
                if lane.get("type") != "driving":
                    continue
                provenance = lane.find("userData[@code='mapforge.provenance/v1']")
                source = lane.find("userData[@code='mapforge.source_lane']")
                lane_rows.append({"lane_id": lid,
                                  "source_lane_id": None if source is None else source.get("value"),
                                  "provenance": {} if provenance is None else json.loads(provenance.get("value")),
                                  "width_stations_m": [lo + float(w.get("sOffset")) for w in lane.findall("width")]})
            row["sections"].append({"start_m": lo, "end_m": hi, "lanes": lane_rows})
        result.append(row)
    return result


def analytic_shared_extrema(road, binding, start, end):
    """Exact piecewise cubic residual extrema, independent of 0.1m sampling."""
    g = road.find("planView/geometry")
    heading = float(g.get("hdg"))
    tangent = np.array([math.cos(heading), math.sin(heading)])
    normal = np.array([-math.sin(heading), math.cos(heading)])
    origin = np.array([float(g.get("x")), float(g.get("y"))])
    points = np.asarray(binding["local_polylines"]["shared"])
    source_s, source_t = (points - origin) @ tangent, (points - origin) @ normal
    if np.all(np.diff(source_s) < 0):
        source_s, source_t = source_s[::-1], source_t[::-1]
    if np.any(np.diff(source_s) <= 0) or start < source_s[0] or end > source_s[-1]:
        raise RuntimeError("Analytic source coverage is not single-valued and complete")
    offsets = road.findall("lanes/laneOffset")
    sections = intervals(road)
    cuts = {start, end}
    cuts.update(float(s) for s in source_s if start < s < end)
    cuts.update(float(o.get("s")) for o in offsets if start < float(o.get("s")) < end)
    for section, lo, hi in sections:
        cuts.update(s for s in (lo, hi) if start < s < end)
        lane = lanes(section).get(-1)
        if lane is not None:
            cuts.update(lo + float(w.get("sOffset")) for w in lane.findall("width")
                        if start < lo + float(w.get("sOffset")) < end)
    pieces, extrema = [], []
    for lo, hi in zip(sorted(cuts), sorted(cuts)[1:]):
        if hi - lo < 1e-10:
            continue
        middle = (lo + hi) / 2
        section, section_lo, _ = next((sec, a, b) for sec, a, b in sections if a <= middle < b)
        index = int(np.searchsorted(source_s, middle, side="right") - 1)
        slope = (source_t[index + 1] - source_t[index]) / (source_s[index + 1] - source_s[index])
        expected = np.array([source_t[index] + slope * (lo - source_s[index]), slope, 0.0, 0.0])
        actual = shifted(offsets, "s", lo) - shifted(lanes(section)[-1].findall("width"), "sOffset", lo - section_lo)
        coefficients = expected - actual
        poly = np.polynomial.Polynomial(coefficients)
        roots = poly.deriv().roots()
        critical = [0.0, hi - lo] + [float(r.real) for r in roots
                                            if abs(r.imag) < 1e-10 and 0 < r.real < hi - lo]
        values = [{"station_m": lo + x, "signed_residual_m": float(poly(x))} for x in critical]
        extrema.extend(values)
        pieces.append({"start_m": lo, "end_m": hi,
                       "desired_delta_coefficients_abcd": coefficients.tolist(),
                       "critical_points": values})
    maximum = max(extrema, key=lambda row: abs(row["signed_residual_m"]))
    return {"method": "piecewise cubic extrema over union of source vertices, lane offsets, widths and sections",
            "pieces": pieces, "absolute_max_m": abs(maximum["signed_residual_m"]),
            "absolute_max_station_m": maximum["station_m"],
            "absolute_max_at_fixed_endpoint": min(abs(maximum["station_m"] - start),
                                                   abs(maximum["station_m"] - end)) < 1e-8,
            "endpoint_invariance_tolerance_m": 1e-9,
            "strict_shared_improvement_required_m": 1e-6}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    paths = [RUN / f"shp-{jid}.xodr" for jid in JUNCTIONS]
    before = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    cases = []
    for jid, path in zip(JUNCTIONS, paths):
        root = parse(path.read_bytes())
        cases.append({"junction_id": jid, "baseline_path": str(path.relative_to(ROOT)),
                      "baseline_sha256": before[str(path.relative_to(ROOT))],
                      "roads": road_summary(root)})

    data = paths[0].read_bytes()
    road = parse(data).find("road[@id='11']")
    # First ordinary -1/-2 section in the preselected first junction. The end
    # is its first pre-existing width break. Neither bound depends on errors.
    ordinary = next((sec, lo, hi) for sec, lo, hi in intervals(road)
                    if all(_is_ordinary(lanes(sec).get(lid)) for lid in (-1, -2)))
    section, start, _ = ordinary
    end = min(start + float(w.get("sOffset"))
              for lid in (-1, -2) for w in lanes(section)[lid].findall("width")
              if float(w.get("sOffset")) > 0)
    knots = [start + (end - start) * i / 4 for i in range(5)]
    binding = build_source_binding(data, ROOT / "shp_0222-0326",
                                   ROOT / "profiles/shp/ibd-smarteditor-v1.yaml",
                                   "11", -1, knots, include_neighbor_context=False)
    prepared = prepare_boundary_edit(data, binding, "11", -1, knots)
    residuals = source_residuals(data, prepared)
    g = road.find("planView/geometry")
    heading = float(g.get("hdg"))
    normal = np.array([-math.sin(heading), math.cos(heading)])
    origin = np.array([float(g.get("x")), float(g.get("y"))])
    stations = _stations(start, end)
    source = _source_samples(road, binding["local_polylines"], stations)
    inner = np.array([states(road, -1, float(s), False) for s in stations])
    outer = np.array([states(road, -2, float(s), False) for s in stations])
    actual = {"shared": (inner[:, 1, :2] - origin) @ normal,
              "inner_lane_center": ((inner[:, 0, :2] + inner[:, 1, :2]) / 2 - origin) @ normal,
              "outer_lane_center": ((outer[:, 0, :2] + outer[:, 1, :2]) / 2 - origin) @ normal}
    expected = {"shared": source["shared"],
                "inner_lane_center": (source["shared"] + source["fixed_inner"]) / 2,
                "outer_lane_center": (source["shared"] + source["fixed_outer"]) / 2}
    signed = {}
    for key in actual:
        difference = expected[key] - actual[key]
        index = int(np.argmax(np.abs(difference)))
        signed[key] = {"desired_delta_min_m": float(difference.min()),
                       "desired_delta_max_m": float(difference.max()),
                       "desired_delta_at_anchor_m": float(np.interp(knots[2], stations, difference)),
                       "absolute_max_station_m": float(stations[index]),
                       "absolute_max_at_fixed_endpoint": index in (0, len(stations) - 1)}
    analytic = analytic_shared_extrema(road, binding, start, end)
    if abs(analytic["absolute_max_m"] - residuals["items"]["shared"]["max_m"]) > 1e-9:
        raise RuntimeError("Analytic and existing sampled baseline maxima disagree")
    blocked = analytic["absolute_max_at_fixed_endpoint"]
    cases[0].update({"capability": prepared.capability(), "source_before": residuals,
                     "signed_source_residuals": signed,
                     "analytic_shared_residual": analytic,
                     "scope_selection": "first ordinary pair section start to first existing pair width break; evenly spaced five knots",
                     "source_binding_file": "0418-source-binding.json",
                     "outcome": "REJECTED_STRICT_IMPROVEMENT_IMPOSSIBLE" if blocked else "REQUIRES_PREREGISTRATION",
                     "reason": "固定端点实现原始最大残差；保持端点的候选无法让 shared max 比基线至少改善 1 微米。" if blocked else "尚未生成或批准目标。",
                     "targets_proposed_m": [], "targets_generated": 0})
    single_lines = [r for r in cases[1]["roads"] if r["geometry_types"] == ["line"]]
    if single_lines:
        raise RuntimeError("Second fixed case now has a single-Line road; baseline premise drifted")
    cases[1].update({"outcome": "UNSUPPORTED_REPRESENTATION",
                     "reason": "该路口没有完整单 Line 的普通道路；当前共享边界算子不能准入。",
                     "targets_proposed_m": [], "targets_generated": 0})
    after = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    source_after = {p: sha(p) for p in binding["input_files_sha256"]}
    if before != after or source_after != binding["input_files_sha256"]:
        raise RuntimeError("Input bytes changed during audit")
    files = [Path(__file__), ROOT / "mapforge/workbench/boundary_probe.py",
             ROOT / "mapforge/workbench/compiler.py", ROOT / "mapforge/repair_web/model.py",
             ROOT / "scripts/internal_edge_jets.py", ROOT / "mapforge/adapters/shp/profile_source.py",
             ROOT / "mapforge/validate/shp_boundary_fidelity.py", ROOT / "uv.lock"]
    report = {"schema": "mapforge/workbench-holdout-scope-audit/v1",
              "run_kind": "READ_ONLY_BASELINE_PREFLIGHT", "cases": cases,
              "code_sha256": {str(p.relative_to(ROOT)): sha(p) for p in files},
              "python": sys.version, "baseline_bytes_unchanged": before == after,
              "bound_source_bytes_unchanged": source_after == binding["input_files_sha256"],
              "candidate_generated_count": 0, "sampling_window_reselected_after_residual": False,
              "guard_modified": False, "independent_customer_operator": False,
              "wb08_positive_holdout_achieved": False}
    write_json(args.out / "0418-source-binding.json", binding)
    write_json(args.out / "report.json", report)
    print(json.dumps({"report": str(args.out / "report.json"),
                      "outcomes": [c["outcome"] for c in cases], "candidates": 0}))


def _is_ordinary(lane):
    if lane is None:
        return False
    try:
        _identity(lane)
    except ValueError:
        return False
    return True


if __name__ == "__main__":
    main()
