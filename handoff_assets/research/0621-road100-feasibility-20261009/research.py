"""Road 100 feasibility: how close can the connector get to its 0621 source window under two constraint sets?

Research only (item A, 2026-10-09). Scope and rules follow 0621-fidelity-audit-20261009/probe_road100.py:
only road 100 is offered to the existing mouth writer, every other road and the header/junction subtrees stay
byte-identical, the frozen source manifest and all source windows are kept, and the original G8/G11 finalizer and
scoreboard evaluator score the whole file. Nothing here is accepted, wired to the product, or a policy change.

Linux runs no Windows consumer: metrics come from scoreboard.evaluate (esmini not run) and are reported as such.

Settings:
  control-baseline   the frozen candidate itself, re-evaluated here (must reproduce the Windows numbers)
  control-production the harness with the production fit family for road 100 (checks the harness itself)
  g11a-*             final road 100 primitive count must be <= 5 (G11-A draft), min segment 3 m
  t2-*               only T2's own checks: connector segments >= 1 m, no segment-count limit
kappa_max is the fitter's own curvature cap (KAPPA_MAX = 0.25 /m), not a T1/T2 threshold; it is varied on purpose.
"""
from __future__ import annotations

from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
OUT = Path(__file__).parent
BASE = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
BASE_XODR_SHA = "9aed2e47be1fc04deee6220d7f4bccdcc02004f65e31c99c1d7e3888aa96c224"
LANE = "2023061415364058085"          # road 100 / lane -1
WINDOWS_P95 = 0.16092286128015673      # frozen Windows scoreboard values for the same bytes
WINDOWS_BOUNDARY_P95 = 0.11800138723332713
SPEED_MPS = 40 / 3.6                   # source speed field of road 100's link

SETTINGS = (
    [{"name": "control-baseline"}, {"name": "control-production"}]
    + [{"name": f"g11a-seg{seg:g}-k{k:.2f}", "group": "g11a", "seg_m": seg, "min_seg": 3.0, "kappa_max": k}
       for seg in (9.0, 11.0, 15.0) for k in (0.25, 0.40, 0.60)]
    + [{"name": f"t2-seg{seg:g}-k{k:.2f}", "group": "t2", "seg_m": seg, "min_seg": 1.0, "kappa_max": k}
       for seg in (3.0, 2.0, 1.5, 1.0) for k in (0.25, 0.40, 0.60)]
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def road_shape(road):
    """Reference-line primitives of one road: count, shortest, |kappa| and |dkappa/ds| maxima."""
    lengths, kappas, rates = [], [], []
    for g in road.findall("planView/geometry"):
        length = float(g.get("length"))
        lengths.append(length)
        child = g[0]
        if child.tag == "line":
            k0 = k1 = 0.0
        elif child.tag == "arc":
            k0 = k1 = float(child.get("curvature"))
        else:
            k0, k1 = float(child.get("curvStart")), float(child.get("curvEnd"))
        kappas += [abs(k0), abs(k1)]
        rates.append(abs(k1 - k0) / length)
    kmax, rmax = max(kappas), max(rates)
    return {"primitives": len(lengths), "min_primitive_m": min(lengths), "length_m": sum(lengths),
            "kappa_max_per_m": kmax, "kappa_rate_max_per_m2": rmax,
            "lateral_accel_at_source_speed_mps2": SPEED_MPS ** 2 * kmax,
            "lateral_jerk_proxy_at_source_speed_mps3": SPEED_MPS ** 3 * rmax}


def centre_samples(path, manifest):
    """Per-sample G8 centre distances with the evaluator's own sampling (as in 0621-fidelity-audit audit.py)."""
    import numpy as np
    from lxml import etree
    from scipy.spatial import cKDTree
    from mapforge.validate.lane_fidelity import _resample, extract_target_components
    root = etree.parse(str(path)).getroot()
    comps = defaultdict(list)
    for c in extract_target_components(root, ds=1.0, zero_width_epsilon_m=0.05)["components"]:
        comps[c["source_lane_id"]].append(c)
    pool, lane = [], None
    for row in manifest["lanes"]:
        sid = row["source_lane_id"]
        if not row.get("comparison", {}).get("eligible") or len(comps[sid]) != 1:
            continue
        s = _resample(np.asarray(row["geometry"]["coordinates"])[:, :2], 1.0)
        t = _resample(np.asarray(comps[sid][0]["points"])[:, :2], 1.0)
        dt = cKDTree(s).query(t)[0]
        pool.extend(dt)
        if sid == LANE:
            lane = dt
    pool = np.asarray(pool)
    return {"pool_count": int(len(pool)), "pool_above_015": int((pool > 0.15).sum()),
            "pool_p95_m": float(np.percentile(pool, 95)),
            "road100_count": int(len(lane)), "road100_above_015": int((lane > 0.15).sum()),
            "road100_p95_m": float(np.percentile(lane, 95)), "road100_max_m": float(lane.max())}


def main():
    import numpy as np
    import yaml
    from lxml import etree
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.ops import connector_edge_joins as E, connector_source_fit as F, mouth_frame_align as M
    from mapforge.report.decision import finalize_opendrive_g8
    from mapforge.validate import scoreboard as sb
    from mapforge.validate.g8_model import json_safe

    assert sha(BASE / "candidate.xodr") == BASE_XODR_SHA
    inputs_before = {p.name: sha(p) for p in BASE.iterdir() if p.is_file()}
    code_paths = sorted({*ROOT.glob("mapforge/ops/*.py"), *ROOT.glob("mapforge/validate/*.py"),
                         *ROOT.glob("mapforge/report/*.py")})
    code_before = {p.relative_to(ROOT).as_posix(): sha(p) for p in code_paths}
    policy_before = {p.relative_to(ROOT).as_posix(): sha(p) for p in (ROOT / "profiles/validation").glob("*.yaml")}
    source = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    manifest_bytes = (BASE / "candidate.source-lanes.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    via = {row["source_lane_id"]: np.asarray(row["geometry"]["coordinates"])[:, :2] for row in manifest["lanes"]
           if row["role"] == "junction-via" and row.get("comparison", {}).get("eligible")}
    policy = yaml.safe_load(sb.POLICY.read_bytes())
    schema = etree.XMLSchema(etree.parse(str(sb.XSD)))
    g8_policy = ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"
    original_supported, original_fits = M._supported, M._source_fits
    rows = []
    for setting in SETTINGS:
        started = time.monotonic()
        directory = OUT / setting["name"]
        directory.mkdir(exist_ok=False)
        path = directory / "candidate.xodr"
        fit_calls, transform, edge = [], None, None
        if setting["name"] == "control-baseline":
            for name in ("candidate.xodr", "candidate.source-review.json"):
                shutil.copyfile(BASE / name, directory / name)
        else:
            tree = etree.parse(str(BASE / "candidate.xodr"), etree.XMLParser(remove_blank_text=False))
            root = tree.getroot()
            roads = {r.get("id"): r for r in root.findall("road")}
            unchanged = {rid: etree.tostring(road) for rid, road in roads.items() if rid != "100"}
            fixed = {tag: [etree.tostring(e) for e in root.findall(tag)] for tag in ("header", "junction")}

            def only100(road):
                return original_supported(road) if road.get("id") == "100" else "outside explicit research operation"

            def scoped_fit(points, targets, fa, fc, rates, monotone=False, end_zone=None):
                centre = {c: ((targets[c]["left"]["x"] + targets[c]["right"]["x"]) / 2,
                              (targets[c]["left"]["y"] + targets[c]["right"]["y"]) / 2) for c in ("start", "end")}
                p0, p1 = (*centre["start"], fa[2]), (*centre["end"], fc[2])
                start = (*p0, M._parallel_curvature(fa, p0))
                end = (*p1, M._parallel_curvature(fc, p1))
                turn = M._turn_sign(p0, p1) if monotone else 0
                lateral = M._tilt_cubic(targets, fa, fc, p0, p1)
                call = {"turn_sign": turn, "end_zone": end_zone, "source_points": len(points)}
                fit_calls.append(call)
                value = F.fit(points, start, end, seg_m=setting["seg_m"], min_seg=setting["min_seg"],
                              kappa_max=setting["kappa_max"], turn_sign=turn, end_rates=rates, lateral=lateral,
                              end_zone=end_zone)
                if value is None:
                    call["fit_info"] = None
                    return []
                lens, ks, info = value
                call["fit_info"] = info
                return [(F.primitives(start, lens, ks), info)]

            M._supported = only100
            if setting["name"] != "control-production":
                M._source_fits = scoped_fit
            try:
                transform = M.align_tree(root, via_centrelines=via, mouth_blend_kappa=.01, kappa_bound_scale=1.25,
                                         mouth_blend_pick=True, source_guided=True, width_local_slopes=True,
                                         match_end_curvature=True, monotone_turns=True, aligned_frame=True,
                                         source_fit=True, turn_end_zone=F.END_ZONE)
            finally:
                M._supported, M._source_fits = original_supported, original_fits
            edge = E.fix_connector(roads["100"])
            assert unchanged == {rid: etree.tostring(road) for rid, road in roads.items() if rid != "100"}
            assert fixed == {tag: [etree.tostring(e) for e in root.findall(tag)] for tag in ("header", "junction")}
            tree.write(str(path), xml_declaration=True, encoding="UTF-8")
            shutil.copyfile(BASE / "candidate.source-review.json", directory / "candidate.source-review.json")
        finalize_opendrive_g8(path, manifest, g8_policy)
        assert json.loads(path.with_suffix(".source-lanes.json").read_bytes()) == manifest
        path.with_suffix(".source-lanes.json").write_bytes(manifest_bytes)
        metrics = json_safe(sb.evaluate(path, "shp", schema=schema, shp_source=source))
        metrics["esmini_pass"], metrics["esmini_gap_max_cm"] = "NOT_RUN_LINUX", None
        tiers = sb.apply_tiers(metrics, "shp", policy)
        g8 = json.loads(path.with_suffix(".g8.json").read_bytes())
        g11 = json.loads(path.with_suffix(".g11.json").read_bytes())
        lane = next(x for x in g8["per_lane"] if x["source_lane_id"] == LANE)
        shape = road_shape(etree.parse(str(path)).getroot().find("road[@id='100']"))
        samples = centre_samples(path, manifest)
        method = None
        if transform:
            method = next((r.get("fit_method") for r in transform.get("rows", []) if str(r.get("road")) == "100"),
                          transform["rows"][0].get("fit_method") if transform.get("rows") else None)
        row = {"setting": setting, "fit_method": method, "fit_calls": fit_calls,
               "road100_shape": shape, "road100_g8": {"p95_m": lane["target_to_source"]["p95_m"],
                                                      "max_m": lane["target_to_source"]["max_m"]},
               "samples": samples,
               "whole_map": {k: metrics.get(k) for k in (
                   "lane_center_noflare_p95_m", "lane_center_noflare_max_m", "boundary_inside_p95_m",
                   "boundary_inside_max_m", "lane_endpoint_lateral_max_m", "conn_seg_min_m", "primitives",
                   "ref_kappa_step_max_per_m", "lane_join_curvature_jump_max_per_m",
                   "road_interface_curvature_max_per_m", "edge_contact_curvature_max_per_m",
                   "conn_edge_join_curvature_jump_max_per_m", "connector_width_bulge_max_m",
                   "outer_curvature_flips_per_100m_max", "paving_holes_gt1cm2", "g8_status")},
               "t2_own_failures": [f["metric"] for f in tiers["T2"]["failed"]],
               "t2_own_unavailable": tiers["T2"].get("unavailable", []),
               "g11_status": g11.get("status"),
               "xodr_sha256": sha(path), "elapsed_s": round(time.monotonic() - started, 1),
               "edge_joins": edge}
        (directory / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf8")
        (directory / "row.json").write_text(json.dumps(json_safe(row), ensure_ascii=False, indent=2), encoding="utf8")
        if transform is not None:
            (directory / "transform.json").write_text(json.dumps(json_safe(transform), ensure_ascii=False, indent=2),
                                                      encoding="utf8")
        rows.append(json_safe(row))
        if setting["name"] == "control-baseline":
            # Linux must reproduce the frozen Windows numbers before any research row means anything.
            got = (metrics["lane_center_noflare_p95_m"], metrics["boundary_inside_p95_m"])
            assert all(math.isclose(a, b, rel_tol=0, abs_tol=1e-12) for a, b in zip(got, (WINDOWS_P95, WINDOWS_BOUNDARY_P95))), got
        print(json.dumps({"name": setting["name"], "method": method, "prims": shape["primitives"],
                          "kmax": round(shape["kappa_max_per_m"], 3), "r100_p95": round(lane["target_to_source"]["p95_m"], 4),
                          "r100_above": samples["road100_above_015"],
                          "p95": metrics["lane_center_noflare_p95_m"], "bnd": metrics["boundary_inside_p95_m"],
                          "t2_fail": row["t2_own_failures"], "s": row["elapsed_s"]}, ensure_ascii=False), flush=True)
    baseline = rows[0]["whole_map"]
    reproduction = {"lane_center_noflare_p95_m": baseline["lane_center_noflare_p95_m"] - WINDOWS_P95,
                    "boundary_inside_p95_m": baseline["boundary_inside_p95_m"] - WINDOWS_BOUNDARY_P95}
    assert inputs_before == {p.name: sha(p) for p in BASE.iterdir() if p.is_file()}
    assert code_before == {p.relative_to(ROOT).as_posix(): sha(p) for p in code_paths}
    assert policy_before == {p.relative_to(ROOT).as_posix(): sha(p) for p in (ROOT / "profiles/validation").glob("*.yaml")}
    summary = {"schema": "mapforge/research/0621-road100-feasibility/v1", "rows": rows,
               "base_xodr_sha256": BASE_XODR_SHA, "input_run_sha256": inputs_before,
               "implementation_sha256": code_before, "policy_sha256": policy_before,
               "windows_reproduction_delta": reproduction, "platform": sys.platform, "python": sys.version.split()[0],
               "consumer": "esmini not run on Linux; T1 is not judged here",
               "accepted": False, "formal_delivery": False}
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf8")


if __name__ == "__main__":
    main()
