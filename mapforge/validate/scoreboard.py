"""Static scoreboard: Jinfeng 7 intersections x {MAP, SHP} -> numeric metrics + draft tiers.

Values are always reported. Tier verdicts come from a versioned policy file and
never alter values. Dynamics at policy fallback speed are kept in a separate
``scenario`` group (2026-09-15 separated-acceptance decision) and never enter
static tiers. Generation calls the existing CLI in subprocesses, so the baseline
pipeline runs unchanged.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SHP_DIR = ROOT / "shp_0222-0326"
XSD = ROOT / "OpenDRIVE_1.5M.xsd"
POLICY = ROOT / "profiles/validation/static-acceptance-v1.draft.yaml"
CASES = (
    ("node3", "map含金路-金玥路node3.xml"),
    ("node4", "map凤苑路-金玥路node4.xml"),
    ("NODE5", "map凤苑路-金坪路NODE5.xml"),
    ("node13", "map凤阁路-金玥路node13.xml"),
    ("node16", "map凤阁路-金剑路路口node16.xml"),
    ("node17", "map含金路-金剑路路口node17.xml"),
    ("node18", "map凤苑路-金剑路node18.xml"),
)
PIPELINES = ("map", "shp")
JOIN_JUMP_BINS = (1e-7, 1e-4, 1e-3)

# metric -> (group, unit, better); "better" drives comparisons, not verdicts.
METRICS = {
    "xsd_valid": ("validity", "", "true"),
    "esmini_pass": ("validity", "", "true"),
    "esmini_gap_max_cm": ("validity", "cm", "lower"),
    "planview_violations": ("validity", "count", "lower"),
    "route_gap_max_m": ("validity", "m", "lower"),
    "road_interface_position_max_m": ("validity", "m", "lower"),
    "road_interface_heading_max_deg": ("validity", "deg", "lower"),
    "edge_contacts": ("validity", "count", "info"),
    "edge_contact_position_max_m": ("validity", "m", "lower"),
    "edge_contact_heading_max_deg": ("validity", "deg", "lower"),
    "lane_edge_step_max_m": ("validity", "m", "lower"),
    "connector_width_bulge_max_m": ("validity", "m", "lower"),
    "connector_width_min_m": ("validity", "m", "higher"),
    "paving_components": ("validity", "count", "info"),
    "paving_holes_gt1cm2": ("validity", "count", "lower"),
    "g8_status": ("fidelity", "", "pass"),
    "lane_center_median_m": ("fidelity", "m", "lower"),
    "lane_center_p95_m": ("fidelity", "m", "lower"),
    "lane_center_max_m": ("fidelity", "m", "lower"),
    "lane_endpoint_max_m": ("fidelity", "m", "lower"),
    # its part across the source direction (endpoint_lateral; reported, not a tier check)
    "lane_endpoint_lateral_max_m": ("fidelity", "m", "lower"),
    # the same lanes measured point to polyline: G8 without its sampling phase (reported, not a tier check)
    "lane_center_poly_median_m": ("fidelity", "m", "lower"),
    "lane_center_poly_p95_m": ("fidelity", "m", "lower"),
    "lane_center_poly_max_m": ("fidelity", "m", "lower"),
    # G8's statistics without the curb-return flare zones at junction mouths (lane_centre_flare, SHP; the flares
    # are recorded by the source review, user decision 2026-10-05; reported, not a tier check)
    "lane_center_noflare_median_m": ("fidelity", "m", "lower"),
    "lane_center_noflare_p95_m": ("fidelity", "m", "lower"),
    "lane_center_noflare_max_m": ("fidelity", "m", "lower"),
    "mouth_curb_flares": ("fidelity", "count", "info"),
    "lane_center_flare_samples_left_out": ("fidelity", "count", "info"),
    "boundary_median_m": ("fidelity", "m", "lower"),
    "boundary_p95_m": ("fidelity", "m", "lower"),
    "boundary_max_m": ("fidelity", "m", "lower"),
    # the same pairing on the common extent only (boundary_inside; reported, not a tier check)
    "boundary_inside_p95_m": ("fidelity", "m", "lower"),
    "boundary_inside_max_m": ("fidelity", "m", "lower"),
    "map_source_integrity": ("fidelity", "", "pass"),
    "source_dropped_points": ("fidelity", "count", "info"),
    # suspected MAP source errors (map_source_review) without an approved decision (reported, not a tier check)
    "source_review_open": ("fidelity", "count", "lower"),
    "source_issues_recorded": ("fidelity", "count", "info"),
    "ref_kappa_step_max_per_m": ("smoothness", "1/m", "lower"),
    "primitives": ("smoothness", "count", "lower"),
    "leg_seg_min_m": ("smoothness", "m", "higher"),
    "conn_seg_min_m": ("smoothness", "m", "higher"),
    "leg_sharp_flips_per_100m_max": ("smoothness", "1/100m", "lower"),
    "outer_curvature_max_per_m": ("smoothness", "1/m", "lower"),
    "outer_heading_step_max_deg": ("smoothness", "deg", "lower"),
    "outer_curvature_flips_per_100m_max": ("smoothness", "1/100m", "lower"),
    "lane_join_curvature_jump_max_per_m": ("smoothness", "1/m", "lower"),
    # junction laneLink interfaces: lane centre (G11-C) and both lane edges (edge contacts)
    "road_interface_curvature_max_per_m": ("smoothness", "1/m", "lower"),
    "edge_contact_curvature_max_per_m": ("smoothness", "1/m", "lower"),
    # every lane edge at the joins inside a road (D2 edge bound pending: reported, not a tier check)
    "lane_edge_join_curvature_jump_max_per_m": ("smoothness", "1/m", "lower"),
    "lane_edge_join_jumps_gt_1e-03": ("smoothness", "count", "lower"),
    # the same on connector lane edges alone (2026-10-07; draft T2 check from 0.7-draft)
    "conn_edge_join_curvature_jump_max_per_m": ("smoothness", "1/m", "lower"),
    # lane centre curving against the turn inside connectors turning >= 30 deg (reported, not a tier check)
    "turn_counter_curvature_max_per_m": ("smoothness", "1/m", "lower"),
    "turn_counter_curvature_gt_0.02": ("smoothness", "count", "lower"),
    "lane_join_jumps_gt_1e-07": ("smoothness", "count", "lower"),
    "lane_join_jumps_gt_1e-04": ("smoothness", "count", "lower"),
    "lane_join_jumps_gt_1e-03": ("smoothness", "count", "lower"),
    "width_records": ("smoothness", "count", "lower"),
    "lane_sections": ("smoothness", "count", "lower"),
    # fairness of driving lane centres between joins (lane_fairness; reported, not tier checks): peak curvature
    # of through road lanes and of straight connectors, curvature rate of turning connectors
    "fair_road_kappa_max_per_m": ("smoothness", "1/m", "lower"),
    "fair_road_kappa_p90_per_m": ("smoothness", "1/m", "lower"),
    "fair_road_sharpness_max_per_m2": ("smoothness", "1/m2", "lower"),
    "fair_road_event_kappa_max_per_m": ("smoothness", "1/m", "lower"),
    "fair_straight_conn_kappa_max_per_m": ("smoothness", "1/m", "lower"),
    "fair_straight_conn_kappa_median_per_m": ("smoothness", "1/m", "lower"),
    "fair_straight_conn_s_bends": ("smoothness", "count", "lower"),
    "fair_turn_conn_sharpness_p90_per_m2": ("smoothness", "1/m2", "lower"),
    "fair_turn_conn_sharpness_max_per_m2": ("smoothness", "1/m2", "lower"),
    "policy_speed_ay_max_mps2": ("scenario", "m/s2", "lower"),
    "policy_speed_jerk_max_mps3": ("scenario", "m/s3", "lower"),
    "kappa_max_per_m": ("scenario", "1/m", "lower"),
    "crs": ("redline", "", "info"),
    "cli_delivery": ("redline", "", "info"),
}


def _sidecar(xodr: Path, suffix: str):
    path = xodr.with_suffix(suffix)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def generate(run_dir: Path, cases=CASES, pipelines=PIPELINES, workers: int = 4,
             shp_mouth: str = "envelope", mouth_margin: float = 3.0, post: str = "c2") -> list[dict]:
    """Run the CLI (default pipeline unless overridden) for every case/pipeline into ``run_dir``.

    The CLI applies approved MAP source corrections itself (derived copy and DROPPED record next
    to the output; the raw file is never modified). ``post="none"`` writes the converters' raw
    output (input for ``transform`` experiments); ``shp_mouth="legacy"`` the old SHP mouths.
    """
    run_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    jobs = []
    for label, map_xml in cases:
        source = ROOT / "v2x_map_xml" / map_xml
        for pipeline in pipelines:
            out = run_dir / f"{pipeline}-{label}.xodr"
            cmd = [sys.executable, "-m", "mapforge.cli", "convert"]
            if pipeline == "map":
                cmd += [str(source), "--to", "xodr"]
            else:
                cmd += [str(SHP_DIR), "--like", str(source), "--to", "xodr", "--shp-mouth", shp_mouth,
                        "--mouth-margin", str(mouth_margin)]
            cmd += ["--post", post, "-o", str(out)]
            jobs.append({"case": label, "pipeline": pipeline, "artifact": out, "cmd": cmd})

    def run(job):
        t0 = time.time()
        proc = subprocess.run(job["cmd"], cwd=str(ROOT), env=env, capture_output=True)
        log = proc.stdout.decode("utf-8", "replace") + proc.stderr.decode("utf-8", "replace")
        job["artifact"].with_suffix(".convert.log").write_text(log, encoding="utf-8")
        # CLI exits 2 when the delivery decision is BLOCKED; the file is still written.
        return {"case": job["case"], "pipeline": job["pipeline"], "artifact": str(job["artifact"]),
                "returncode": proc.returncode, "seconds": round(time.time() - t0, 1),
                "generated": job["artifact"].exists()}

    with ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        return list(pool.map(run, jobs))


TRANSFORMS = {
    "mouth-edge-match": "mapforge.ops.mouth_edge_match:apply",
    "mouth-frame-align": "mapforge.ops.mouth_frame_align:apply",
    "lane-refit": "mapforge.ops.lane_refit:apply",
    "lane-refit+mouths": "mapforge.ops.lane_refit:apply_with_mouths",
    "lane-g2-k04": "mapforge.ops.lane_refit:apply_g2_k04",
    "lane-g2-k02": "mapforge.ops.lane_refit:apply_g2_k02",
    "lane-c1-k04": "mapforge.ops.lane_refit:apply_c1_k04",
    "lane-c1-k02": "mapforge.ops.lane_refit:apply_c1_k02",
    "lane-g2-k04-curb": "mapforge.ops.lane_refit:apply_g2_k04_curb",
    "lane-g2-k04-curb-local": "mapforge.ops.lane_refit:apply_g2_k04_curb_local",
    "lane-g2-k04-curb-pick": "mapforge.ops.lane_refit:apply_g2_k04_curb_pick",
    "lane-g2-k04-curb-guided": "mapforge.ops.lane_refit:apply_g2_k04_curb_guided",
    "lane-g2-k04-c2": "mapforge.ops.lane_refit:apply_g2_k04_c2",
}
G8_POLICY = ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"


def transform(from_dir: Path, run_dir: Path, name: str, cases=CASES, pipelines=PIPELINES) -> list[dict]:
    """Apply a registered post-process to a scored run and re-run the CLI's own finalization.

    Sidecars (G8, G11, edge contacts, MAP source integrity, decision) are recomputed by
    ``finalize_opendrive_g8`` with the source manifest of the input run, exactly as the CLI does.
    """
    import importlib
    from mapforge.report.decision import finalize_opendrive_g8

    module, func = TRANSFORMS[name].split(":")
    fn = getattr(importlib.import_module(module), func)
    run_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for label, map_xml in cases:
        for pipeline in pipelines:
            src = from_dir / f"{pipeline}-{label}.xodr"
            out = run_dir / f"{pipeline}-{label}.xodr"
            entry = {"case": label, "pipeline": pipeline, "artifact": str(out), "from": str(src)}
            t0 = time.time()
            try:
                report = fn(src, out)
                manifest = json.loads(src.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
                raw = [ROOT / "v2x_map_xml" / map_xml] if pipeline == "map" else None
                record = src.with_suffix(".source-corrections.json")
                if pipeline == "map" and record.exists():
                    raw = [Path(json.loads(record.read_text(encoding="utf-8"))["derived"]["file"])]
                    for suffix in (".source-corrections.json", ".loss-report.json"):
                        out.with_suffix(suffix).write_bytes(src.with_suffix(suffix).read_bytes())
                finalize_opendrive_g8(out, manifest, G8_POLICY, connect_mode="data", raw_map_paths=raw)
                out.with_suffix(".transform.json").write_text(
                    json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
                entry.update(generated=True, transform={k: v for k, v in report.items() if k != "rows"})
            except Exception as exc:  # report, never hide
                entry.update(generated=out.exists(), error=f"{type(exc).__name__}: {exc}")
            entry["seconds"] = round(time.time() - t0, 1)
            entries.append(entry)
    return entries


ESMINI_DLL = ROOT / "esmini" / "bin" / "esminiRMLib.dll"


def esmini_check(xodr: Path) -> dict | None:
    """Independent consumer check (esmini RoadManager, scripts/esmini_rm_check.py); None if unavailable."""
    import re
    if not ESMINI_DLL.exists():
        return None
    proc = subprocess.run([sys.executable, "-u", str(ROOT / "scripts" / "esmini_rm_check.py"), str(xodr)],
                          cwd=str(ROOT), capture_output=True)
    raw = proc.stdout
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        text = raw.decode("gbk", "replace")
    line = next((x for x in text.splitlines() if x.startswith(xodr.name + ":")), "")
    gaps = [float(v) for v in re.findall(r"([0-9.]+)cm", line)]
    return {"pass": proc.returncode == 0 and line.rstrip().endswith("PASS"),
            "gap_max_cm": max(gaps) if gaps else None, "line": line.strip()}


def connector_widths(root) -> dict:
    """Junction connector lane widths: bulge/pinch beyond the end widths and the minimum width."""
    bulge, minimum = 0.0, None
    for road in root.findall("road"):
        if road.get("junction") in (None, "-1") or road.get("name") == "junction_paving":
            continue
        sections = road.findall("lanes/laneSection")
        length = float(road.get("length"))
        for k, sec in enumerate(sections):
            s0 = float(sec.get("s"))
            s1 = float(sections[k + 1].get("s")) if k + 1 < len(sections) else length
            for lane in sec.findall("left/lane") + sec.findall("right/lane"):
                rows = sorted((float(w.get("sOffset")), *(float(w.get(c)) for c in "abcd"))
                              for w in lane.findall("width"))
                if not rows or lane.get("type") != "driving":
                    continue
                values = []
                for i in range(201):
                    u = (s1 - s0) * i / 200
                    r = [x for x in rows if x[0] <= u + 1e-9][-1]
                    v = u - r[0]
                    values.append(r[1] + v * (r[2] + v * (r[3] + v * r[4])))
                ends = (values[0], values[-1])
                bulge = max(bulge, max(values) - max(ends), min(ends) - min(values))
                minimum = min(values) if minimum is None else min(minimum, min(values))
    return {"connector_width_bulge_max_m": bulge, "connector_width_min_m": minimum}


def _join_jumps(g11: dict) -> list[float]:
    values = []
    for issue in g11.get("groups", {}).get("G11-D", {}).get("issues", []) or []:
        if issue.get("code") == "driving_lane_curvature_jump":
            values += [j["curvature_jump_per_m"] for j in issue.get("joints", [])]
    return values


def evaluate(xodr: Path, pipeline: str, schema=None, shp_source=None) -> dict:
    """Numeric metrics for one written file; reuses the CLI sidecars plus file audits."""
    from lxml import etree
    from mapforge.validate.planview_check import check_file
    from mapforge.validate.smoothness import audit_file, curvature_audit, route_continuity

    schema = schema or etree.XMLSchema(etree.parse(str(XSD)))
    g8, g11 = _sidecar(xodr, ".g8.json"), _sidecar(xodr, ".g11.json")
    contacts, decision = _sidecar(xodr, ".edge-contacts.json"), _sidecar(xodr, ".delivery-decision.json")
    integrity = _sidecar(xodr, ".source-integrity.json")
    corrections = _sidecar(xodr, ".source-corrections.json")
    root = ET.parse(xodr).getroot()
    audit = audit_file(xodr)
    routes = route_continuity(root)
    curvature = curvature_audit(root)
    groups = (g11 or {}).get("groups", {})
    m = {}
    m["xsd_valid"] = bool(schema.validate(etree.parse(str(xodr))))
    esmini = esmini_check(xodr)
    if esmini is not None:
        m["esmini_pass"] = esmini["pass"]
        m["esmini_gap_max_cm"] = esmini["gap_max_cm"]
    m["planview_violations"] = len(check_file(str(xodr))["violations"])
    gaps = [x["gap_in"] for x in routes] + [x["gap_out"] for x in routes if not math.isnan(x["gap_out"])]
    m["route_gap_max_m"] = max(gaps, default=0.0)
    c_metrics = groups.get("G11-C", {}).get("metrics", {})
    m["road_interface_position_max_m"] = c_metrics.get("interface_position_m")
    m["road_interface_heading_max_deg"] = c_metrics.get("interface_heading_deg")
    m["road_interface_curvature_max_per_m"] = c_metrics.get("interface_curvature_per_m")
    if contacts:
        m["edge_contacts"] = contacts.get("count")
        m["edge_contact_position_max_m"] = contacts.get("maxima", {}).get("position_m")
        m["edge_contact_heading_max_deg"] = contacts.get("maxima", {}).get("heading_deg")
        m["edge_contact_curvature_max_per_m"] = contacts.get("maxima", {}).get("curvature_per_m")
    m["lane_edge_step_max_m"] = audit.get("lane_edge_step_max")
    m.update(connector_widths(root))
    m["paving_components"] = audit.get("paving_components")
    m["paving_holes_gt1cm2"] = audit.get("paving_holes_gt1cm2")

    if g8:
        gm = g8.get("metrics", {})
        both = [gm.get("source_to_target", {}), gm.get("target_to_source", {})]
        m["g8_status"] = g8.get("status")
        m["lane_center_median_m"] = max((x.get("median_m", 0.0) for x in both), default=None)
        m["lane_center_p95_m"] = max((x.get("p95_m", 0.0) for x in both), default=None)
        m["lane_center_max_m"] = max((x.get("max_m", 0.0) for x in both), default=None)
        ends = [gm.get("endpoint_start", {}).get("max_m"), gm.get("endpoint_end", {}).get("max_m")]
        m["lane_endpoint_max_m"] = max((x for x in ends if x is not None), default=None)
    manifest = _sidecar(xodr, ".source-lanes.json")
    if manifest:
        from mapforge.validate.lane_centre_poly import audit as poly_audit
        m.update(poly_audit(root, manifest))
        if pipeline == "shp":
            from mapforge.validate.lane_centre_flare import audit as flare_audit
            m.update(flare_audit(root, manifest, _sidecar(xodr, ".source-review.json")))
        if g8:
            from mapforge.validate.endpoint_lateral import audit as endpoint_lateral_audit
            m.update(endpoint_lateral_audit(root, manifest, g8))
    if pipeline == "shp":
        from mapforge.validate.shp_boundary_fidelity import evaluate_shp_outer_edges
        b = evaluate_shp_outer_edges(SHP_DIR, xodr, src=shp_source)
        both = [b["source_to_target"], b["target_to_source"]]
        m["boundary_median_m"] = max(x["median_m"] for x in both)
        m["boundary_p95_m"] = max(x["p95_m"] for x in both)
        m["boundary_max_m"] = max(x["max_m"] for x in both)
        from mapforge.validate.boundary_inside import audit as boundary_inside_audit
        m.update(boundary_inside_audit(xodr, SHP_DIR, src=shp_source))
    if integrity is not None:
        m["map_source_integrity"] = integrity.get("status")
    m["source_dropped_points"] = (corrections or {}).get("summary", {}).get("DROPPED", 0)
    review = _sidecar(xodr, ".source-review.json")
    if review is not None:
        m["source_review_open"] = review.get("open")
        if review.get("recorded") is not None:
            m["source_issues_recorded"] = review.get("recorded")

    m["ref_kappa_step_max_per_m"] = audit.get("kappa_step_max")
    m["primitives"] = groups.get("G11-A", {}).get("metrics", {}).get("primitives")
    m["leg_seg_min_m"] = curvature.get("leg", {}).get("seg_min_len")
    m["conn_seg_min_m"] = curvature.get("conn", {}).get("seg_min_len")
    m["leg_sharp_flips_per_100m_max"] = curvature.get("leg", {}).get("flips_per_100m_max")
    m["outer_curvature_max_per_m"] = audit.get("outer_curvature_max")
    m["outer_heading_step_max_deg"] = audit.get("outer_edge_heading_step_max_deg")
    m["outer_curvature_flips_per_100m_max"] = audit.get("outer_curvature_flips_per_100m_max")
    d_metrics = groups.get("G11-D", {}).get("metrics", {})
    m["lane_join_curvature_jump_max_per_m"] = d_metrics.get("lane_curvature_join_jump_max_per_m")
    jumps = _join_jumps(g11 or {})
    for edge in JOIN_JUMP_BINS:
        m[f"lane_join_jumps_gt_{edge:.0e}"] = sum(1 for x in jumps if x > edge)
    from mapforge.validate.lane_edge_joins import audit as lane_edge_audit
    from mapforge.validate.turn_shape import audit as turn_audit
    m.update(lane_edge_audit(root))
    m.update(turn_audit(root))
    from mapforge.validate.lane_fairness import audit as fairness_audit
    m.update(fairness_audit(root))
    b_metrics = groups.get("G11-B", {}).get("metrics", {})
    m["width_records"] = b_metrics.get("width_border_records")
    m["lane_sections"] = b_metrics.get("lane_sections")

    m["policy_speed_ay_max_mps2"] = d_metrics.get("lateral_acceleration_max_mps2")
    m["policy_speed_jerk_max_mps3"] = d_metrics.get("lateral_jerk_max_mps3")
    m["kappa_max_per_m"] = d_metrics.get("kappa_max_per_m")
    reasons = [r.get("code") for r in (decision or {}).get("blocked_reasons", [])]
    m["crs"] = "internally-consistent" if "crs_not_absolutely_verified" in reasons else "unchecked"
    m["cli_delivery"] = (decision or {}).get("status")
    return m


def _check(value, op, target):
    if value is None:
        return None
    if op == "eq":
        return value == target
    if op == "le":
        return value <= target
    if op == "ge":
        return value >= target
    raise ValueError(f"unknown op {op}")


def apply_tiers(metrics: dict, pipeline: str, policy: dict) -> dict:
    """Evaluate draft tiers; missing metrics make a tier UNAVAILABLE, never PASS."""
    out = {}
    for tier, spec in policy["tiers"].items():
        rows, verdicts = [], []
        for c in spec["checks"]:
            if c.get("applies_to") and pipeline not in c["applies_to"]:
                continue
            ok = _check(metrics.get(c["metric"]), c["op"], c["value"])
            rows.append({**c, "actual": metrics.get(c["metric"]), "pass": ok})
            verdicts.append(ok)
        status = ("UNAVAILABLE" if any(v is None for v in verdicts)
                  else "PASS" if all(verdicts) else "FAIL")
        required = spec.get("requires")
        if required and out.get(required, {}).get("status") != "PASS" and status == "PASS":
            status = "BLOCKED_BY_" + required
        out[tier] = {"status": status, "failed": [r for r in rows if r["pass"] is False],
                     "unavailable": [r["metric"] for r in rows if r["pass"] is None]}
    return out


def score(run_dir: Path, entries: list[dict], policy_path: Path = POLICY) -> dict:
    from lxml import etree
    from mapforge.adapters.shp.profile_source import ProfileSource

    policy = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    schema = etree.XMLSchema(etree.parse(str(XSD)))
    shp_source = None
    rows = []
    for e in entries:
        xodr = Path(e["artifact"])
        row = {"case": e["case"], "pipeline": e["pipeline"], "artifact": xodr.name}
        if e.get("error") or not xodr.exists():
            row.update(error=e.get("error") or "not generated", tiers={})
            rows.append(row)
            continue
        if e["pipeline"] == "shp" and shp_source is None:
            shp_source = ProfileSource(str(SHP_DIR), "ibd-smarteditor-v1")
        try:
            row["metrics"] = evaluate(xodr, e["pipeline"], schema=schema, shp_source=shp_source)
            row["tiers"] = apply_tiers(row["metrics"], e["pipeline"], policy)
        except Exception as exc:  # report, never hide
            row.update(error=f"{type(exc).__name__}: {exc}", tiers={})
        rows.append(row)
    tiers = list(policy["tiers"])
    summary = {t: sum(1 for r in rows if r.get("tiers", {}).get(t, {}).get("status") == "PASS")
               for t in tiers}
    return {"schema": "mapforge/scoreboard/v1", "policy": {"id": policy["id"], "version": policy["version"],
            "lifecycle": policy["lifecycle"]}, "run_dir": str(run_dir), "files": len(rows),
            "tier_pass": summary, "rows": rows}


def _fmt(v):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "✓" if v else "✗"
    if isinstance(v, float):
        if v == 0:
            return "0"
        return f"{v:.2e}" if abs(v) < 1e-2 or abs(v) >= 1e4 else f"{v:.3f}"
    return str(v)


def markdown(board: dict) -> str:
    rows = board["rows"]
    keys = [k for k in METRICS if any(k in r.get("metrics", {}) for r in rows)]
    head = ["指标", "组"] + [f"{r['pipeline']}-{r['case']}" for r in rows]
    lines = [f"# 评分板 {Path(board['run_dir']).name}", "",
             f"策略 `{board['policy']['id']}` {board['policy']['version']}（{board['policy']['lifecycle']}，"
             "草案阈值只用于方案比较，不作交付放行）", "",
             "| 等级 | " + " | ".join(head[2:]) + " |", "|---|" + "---|" * len(rows)]
    for tier in board["tier_pass"]:
        lines.append(f"| {tier} | " + " | ".join(r.get("tiers", {}).get(tier, {}).get("status", "ERROR")
                                                 for r in rows) + " |")
    lines += ["", "| " + " | ".join(head) + " |", "|---|---|" + "---|" * len(rows)]
    for k in keys:
        group = METRICS[k][0]
        lines.append(f"| {k} | {group} | " + " | ".join(_fmt(r.get("metrics", {}).get(k)) for r in rows) + " |")
    errors = [f"- {r['pipeline']}-{r['case']}: {r['error']}" for r in rows if r.get("error")]
    if errors:
        lines += ["", "## 错误", ""] + errors
    return "\n".join(lines) + "\n"
