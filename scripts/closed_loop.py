# -*- coding: utf-8 -*-
"""金凤 14 文件 G1–G11 + esmini 闭环；JSON 是正式机器结果。"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
POLICY = ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"
G11_POLICY = ROOT / "profiles/validation/g11-opendrive-v1.draft.yaml"
G11_BASELINE = ROOT / "out/v131-opendrive-baseline.json"
INDEX = ROOT / "out/closed-loop-inputs.json"
REPORT = ROOT / "out/closed-loop-report.json"

LIM = {
    "leg": {"flips": 8.0, "sharp": 0.0045, "jerk": 21.0, "med": 3.0, "min": 3.0},
    "conn": {"flips": 30.0, "sharp": 0.70, "jerk": 400.0, "med": 2.0, "min": 1.0},
}
EDGE_LIM = {
    # 来自 7 个真实 SHP + 7 个真实 MAP 的锁定样本，并以旧版蛇形文件做故障对照。
    # 阈值约束的是最终世界坐标道路边缘，不是参考线或中间断面控制点。
    "lateral_second_max": 0.40,
    "outer_curvature_max": 0.25,
    "outer_heading_step_deg": 5.0,
    "outer_flips_per_100m": 5.0,
    "shp_boundary_median_m": 0.30,
    "shp_boundary_p95_m": 0.65,
    "shp_boundary_paired_roads": 4,
}


def _gate(status, **data):
    return {"status": status, **data}


def main():
    regen_rc = 0
    if "--no-regen" not in sys.argv:
        print("== 再生成固定 14 文件 ==")
        run = subprocess.run([sys.executable, str(ROOT / "scripts/gen_all.py")],
                             capture_output=True, text=True, cwd=str(ROOT))
        regen_rc = run.returncode
        if run.stdout:
            print(run.stdout[-4000:])
        if run.stderr:
            print(run.stderr[-2000:])
    if not INDEX.exists():
        print("缺 out/closed-loop-inputs.json")
        return 2
    index = json.loads(INDEX.read_text(encoding="utf-8"))
    entries = index.get("entries", [])
    if len(entries) != 14:
        print(f"输入矩阵不是 14 文件：{len(entries)}")

    from lxml import etree
    from mapforge.report.decision import finalize_opendrive_g8
    from mapforge.validate.g8_model import write_json
    from mapforge.validate.g11 import audit_file as audit_g11, load_policy as load_g11_policy
    from mapforge.validate.planview_check import check_file
    from mapforge.validate.smoothness import audit_file, curvature_audit, route_continuity
    from scripts.gen_all import CASES
    raw_map_paths = [ROOT/'v2x_map_xml'/name for _, name in CASES]

    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    g11_policy = load_g11_policy(G11_POLICY)
    g11_baseline = (json.loads(G11_BASELINE.read_text(encoding="utf-8"))
                    if G11_BASELINE.exists() else None)
    shp_profile_source = None
    files, artifact_paths = [], []
    print(f"{'file':30s} G1 G2 G3 G4 G5 G7 G8 G9 G10 G11 route(cm) "
          "leg(min/med) conn(min/med)")
    for entry in entries:
        rel = entry["artifact"]
        path = ROOT / rel
        artifact_paths.append(path)
        root = ET.parse(path).getroot()
        xsd = schema.validate(etree.parse(str(path)))
        pv = check_file(str(path))
        audit = audit_file(path)
        routes = route_continuity(root)
        gin = max((x["gap_in"] for x in routes), default=0.0)
        gout = max((x["gap_out"] for x in routes if not math.isnan(x["gap_out"])), default=0.0)
        curvature = curvature_audit(root)
        cq_ok = all(q["flips_per_100m_max"] <= LIM[k]["flips"]
                    and q["sharpness_max"] <= LIM[k]["sharp"]
                    and q["jerk_max"] <= LIM[k]["jerk"]
                    and q["seg_median_len"] + 1e-8 >= LIM[k]["med"]
                    # 生成器后置条件已按 1e-9 容差锁定 3.0m；十位有效数字写出
                    # 可能成为 2.999999999m。这里只容纳 10nm 数值误差，不改变
                    # 工程阈值，也不会放过 2.99999m 的真实短段。
                    and q["seg_min_len"] + 1e-8 >= LIM[k]["min"]
                    for k, q in curvature.items())
        surface_ok = (
            audit.get("paving_roads", 0) >= 1
            and audit.get("paving_road_components_max") == 1
            and audit.get("paving_components") == 1
            and audit.get("paving_holes_gt1cm2") == 0
            and audit.get("paving_leg_overlap_min", 0.0) >= 0.2
        )
        boundary = {"applicable": entry["pipeline"] == "shp-to-opendrive"}
        boundary_ok = True
        if boundary["applicable"]:
            try:
                from mapforge.adapters.shp.profile_source import ProfileSource
                from mapforge.validate.shp_boundary_fidelity import evaluate_shp_outer_edges
                if shp_profile_source is None:
                    shp_profile_source = ProfileSource(
                        str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
                boundary.update(evaluate_shp_outer_edges(
                    ROOT / "shp_0222-0326", path, src=shp_profile_source))
                boundary_ok = (
                    boundary["paired_roads"] >= EDGE_LIM["shp_boundary_paired_roads"]
                    and boundary["source_to_target"]["median_m"]
                    <= EDGE_LIM["shp_boundary_median_m"]
                    and boundary["target_to_source"]["median_m"]
                    <= EDGE_LIM["shp_boundary_median_m"]
                    and boundary["source_to_target"]["p95_m"]
                    <= EDGE_LIM["shp_boundary_p95_m"]
                    and boundary["target_to_source"]["p95_m"]
                    <= EDGE_LIM["shp_boundary_p95_m"]
                )
            except Exception as exc:
                boundary_ok = False
                boundary["error"] = f"{type(exc).__name__}: {exc}"
        edge_ok = (
            audit.get("edge_lateral_second_max", float("inf"))
            <= EDGE_LIM["lateral_second_max"]
            and audit.get("outer_curvature_max", float("inf"))
            <= EDGE_LIM["outer_curvature_max"]
            and audit.get("outer_edge_heading_step_max_deg", float("inf"))
            <= EDGE_LIM["outer_heading_step_deg"]
            and audit.get("outer_curvature_flips_per_100m_max", float("inf"))
            <= EDGE_LIM["outer_flips_per_100m"]
            and boundary_ok
        )
        manifest_path = ROOT / entry["sidecars"]["source_manifest"]
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        final = finalize_opendrive_g8(path, manifest, POLICY, connect_mode="data",
                                     g11_policy=g11_policy,g11_baseline=g11_baseline,
                                     raw_map_paths=raw_map_paths)
        g8 = final["gate"]
        g11 = final["g11"]
        g11_sidecar = path.with_suffix(".g11.json")
        write_json(g11_sidecar, g11)
        gates = {
            "G1": _gate("PASS" if xsd else "FAIL"),
            "G2": _gate("PASS" if not pv["violations"] else "FAIL",
                        violations=pv["violations"]),
            "G3": _gate("PASS" if audit["kappa_step_max"] < 1e-6 else "FAIL",
                        kappa_step_max=audit["kappa_step_max"]),
            "G4": _gate("PASS" if audit["lane_edge_step_max"] < 0.05 else "FAIL",
                        lane_edge_step_max=audit["lane_edge_step_max"]),
            "G5": _gate("PASS" if gin < 0.01 and gout < 0.01 else "FAIL",
                        gap_in_m=gin, gap_out_m=gout),
            "G7": _gate("PASS" if cq_ok else "FAIL", metrics=curvature),
            "G8": g8,
            "G9": _gate(
                "PASS" if surface_ok else "FAIL",
                paving_roads=audit.get("paving_roads", 0),
                paving_road_components_max=audit.get("paving_road_components_max"),
                paving_components=audit.get("paving_components"),
                paving_holes_gt1cm2=audit.get("paving_holes_gt1cm2"),
                paving_hole_area_max=audit.get("paving_hole_area_max"),
                paving_leg_overlap_min=audit.get("paving_leg_overlap_min"),
            ),
            "G10": _gate(
                "PASS" if edge_ok else "FAIL",
                edge_lateral_second_max=audit.get("edge_lateral_second_max"),
                edge_lateral_second_max_road=audit.get("edge_lateral_second_max_road"),
                outer_curvature_max=audit.get("outer_curvature_max"),
                outer_curvature_max_road=audit.get("outer_curvature_max_road"),
                outer_edge_heading_step_max_deg=audit.get(
                    "outer_edge_heading_step_max_deg"),
                outer_edge_heading_step_max_deg_road=audit.get(
                    "outer_edge_heading_step_max_deg_road"),
                outer_curvature_flips_per_100m_max=audit.get(
                    "outer_curvature_flips_per_100m_max"),
                shp_boundary_fidelity=boundary,
                limits=EDGE_LIM,
            ),
            "G11": g11,
        }
        # Do not report a suite PASS while omitting newer required sub-gates
        # (raw MAP source integrity or true lane-edge contacts).
        gates.update(final['quality']['gates'])
        ok = all(x["status"] == "PASS" for x in gates.values())
        lg, cn = curvature.get("leg", {}), curvature.get("conn", {})
        print(f"{'OK ' if ok else 'FAIL'} {rel:25s} "
              f"{' '.join(gates[x]['status'][0] for x in ('G1','G2','G3','G4','G5','G7'))} "
              f"{g8['status']:<4s} {gates['G9']['status']:<4s} {gates['G10']['status']:<4s} "
              f"{g11['status']:<4s}/{g11.get('level', 'UNAVAILABLE')[0]} "
              f"{gin*100:.1f}/{gout*100:.1f} "
              f"{lg.get('seg_min_len',0):.1f}/{lg.get('seg_median_len',0):.1f} "
              f"{cn.get('seg_min_len',0):.1f}/{cn.get('seg_median_len',0):.1f}")
        files.append({"case": entry["case"], "pipeline": entry["pipeline"],
                      "artifact": rel, "gates": gates,
                      "delivery_decision": final["decision"]})

    print("== G6 esmini RoadManager 独立行驶（suite-level） ==")
    run = subprocess.run([sys.executable, "-u", str(ROOT / "scripts/esmini_rm_check.py")]
                         + [str(x) for x in artifact_paths], capture_output=True, cwd=str(ROOT))
    text = run.stdout.decode("gbk", errors="replace")
    for line in text.splitlines():
        if "PASS" in line or "CHECK" in line:
            print("  " + line.strip())
    g6 = _gate("PASS" if run.returncode == 0 else "FAIL", scope="suite",
               files=len(artifact_paths), returncode=run.returncode)
    all_file_gates = all(all(g["status"] == "PASS" for g in x["gates"].values())
                         for x in files)
    overall = (len(files) == 14 and not index.get("errors") and regen_rc == 0
               and all_file_gates and g6["status"] == "PASS")
    report = {
        "schema": "mapforge/closed-loop-report/v1",
        "suite": "jinfeng-opendrive-14",
        "status": "PASS" if overall else "FAIL",
        "files": files,
        "suite_gates": {"G6": g6},
        "generation": {"returncode": regen_rc, "errors": index.get("errors", [])},
        "summary": {
            "files": len(files),
            "g8_passed": sum(x["gates"]["G8"]["status"] == "PASS" for x in files),
            "g9_passed": sum(x["gates"]["G9"]["status"] == "PASS" for x in files),
            "g10_passed": sum(x["gates"]["G10"]["status"] == "PASS" for x in files),
            "g11_passed": sum(x["gates"]["G11"]["status"] == "PASS" for x in files),
            "delivery_blocked": sum(x["delivery_decision"]["status"] == "BLOCKED"
                                    for x in files),
        },
    }
    write_json(REPORT, report)
    print("\n=>", "G1–G11 + esmini 全部 PASS" if overall else "存在 FAIL/UNAVAILABLE")
    print(f"报告: {REPORT.relative_to(ROOT)}")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
