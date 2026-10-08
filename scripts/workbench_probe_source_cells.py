"""Source-supported interval representation on the fixed WB11 source, in an owned spawn process.

Example: .venv/Scripts/python.exe scripts/workbench_probe_source_cells.py
         --out out/workbench/wb11-source-cells-20261008
Never overwrites an earlier run and never accepts a production candidate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=str), encoding="utf8")


def worker(out_name):
    out = Path(out_name)
    log = (out / "worker.log").open("w", encoding="utf8", buffering=1)
    sys.stdout = sys.stderr = log
    report = {"schema": "mapforge/wb11-source-cells-probe/v1", "status": "RUNNING",
              "worker_pid": os.getpid(), "parent_pid": os.getppid(), "candidate_accepted": False,
              "formal_release_verified": False, "independent_operator_verified": False}
    start = time.monotonic()
    bindings = {}
    snapshot = None
    source_dir = ROOT / "shp_0222-0326"
    profile = ROOT / "profiles/shp/ibd-smarteditor-v1.yaml"

    def stage(name):
        report["stage"] = name
        report["elapsed_s"] = round(time.monotonic()-start, 3)
        dump(out / "report.json", report)
        print(name, flush=True)

    try:
        from mapforge.adapters.shp.profile_source import ProfileSource
        from mapforge.ops import shp_to_xodr as S, envelope_surface as E, shp_leave_mouth
        from mapforge.workbench.sources import SourceCatalog, verify_source_snapshot
        from mapforge.workbench import source_surface_partition as P
        from mapforge.workbench import source_surface_rebuild as R
        from mapforge import pipeline

        stage("SOURCE_BINDING")
        code_paths = sorted((ROOT / "mapforge").rglob("*.py"))
        code_paths += [Path(__file__).resolve(), ROOT / "uv.lock", profile,
                       ROOT / "profiles/validation/static-acceptance-v1.draft.yaml",
                       ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"]
        bindings = {p.relative_to(ROOT).as_posix(): sha(p) for p in code_paths}
        for name in ["mapforge/workbench/source_surface_cells.py", "mapforge/workbench/source_surface_rebuild.py",
                     "scripts/workbench_probe_source_cells.py"]:
            saved = out / "code-snapshots" / name
            saved.parent.mkdir(parents=True, exist_ok=True)
            saved.write_bytes((ROOT / name).read_bytes())
        snapshot = SourceCatalog(source_dir, profile).snapshot(P.JUNCTION_ID)
        intent = R.make_intent(snapshot)
        dump(out / "source-snapshot.json", snapshot)
        dump(out / "confirmed-probe-intent.json", intent)
        dump(out / "code-policy-binding.json", bindings)
        report["source_snapshot_id"] = snapshot["snapshot_id"]
        source = ProfileSource(str(source_dir), str(profile))
        junction = next(j for j in source.junctions if j.pid == P.JUNCTION_ID)
        raw = out / "source-generated.partial.xodr"
        stage("SOURCE_GENERATION")
        try:
            stats = S.build_junction_xodr(source, junction, raw, connect_mode="data", allow_uturn=False,
                     mouth_policy="source-envelope-candidate", mouth_margin_m=3.0)
        except S.CandidateSurfaceError as exc:
            stats = exc.stats
            report["original_generator_failure"] = str(exc)
        else:
            raise P.PartitionRejected("expected-default-surface-failure-not-reproduced")
        dump(out / "source-generation.stats.json", stats)
        manifest = stats["source_lane_manifest"]
        dump(raw.with_suffix(".source-lanes.json"), manifest)
        root = ET.parse(raw).getroot()
        decisions = stats["mouth_envelope_decisions"]
        project = lambda p: S._proj(p, float(junction.center[1]), float(junction.center[0]))
        try:
            E.replace_source_paving(ET.fromstring(ET.tostring(root)), source, junction, project, decisions)
        except ValueError as exc:
            report["default_envelope_failure"] = str(exc)
        else:
            raise P.PartitionRejected("expected-default-envelope-failure-not-reproduced")
        stage("EXPLICIT_SOURCE_CONNECTED_INTERVALS")
        candidate, evidence, context = R.apply_rebuild(root, source, junction, project, decisions,
                                                         snapshot, intent)
        actual = out / "surface.unaccepted.xodr"
        ET.indent(candidate)
        ET.ElementTree(candidate).write(actual, encoding="utf-8", xml_declaration=True)
        reread = ET.parse(actual).getroot()
        audit = R.audit_written(reread, evidence, context)
        evidence["written_sha256"] = sha(actual)
        evidence["actual_xml_support_audit"] = audit
        dump(out / "surface-evidence.json", evidence)
        report["local_support_audit"] = audit
        report["source_partial_sha256"] = sha(raw)
        report["unaccepted_surface_sha256"] = sha(actual)
        if audit["status"] != "PASS":
            report["status"] = "REJECTED_LOCAL_SOURCE_SUPPORT"
            report["default_postprocess_executed"] = False
            report["whole_map_score_executed"] = False
            stage("REJECTED_BEFORE_POSTPROCESS")
            return

        # The existing stage expects lxml; it is read-only here. A movement
        # means the pre-registered experiment no longer covers its dependency.
        from lxml import etree
        moved = shp_leave_mouth.plan(etree.parse(str(actual)).getroot(), source,
                     (float(junction.center[0]), float(junction.center[1])))
        report["departure_plan"] = moved
        if moved:
            raise P.PartitionRejected("unexpected-departure-dependency")
        stage("DEFAULT_POSTPROCESS")
        candidate_file = out / "candidate.xodr"
        final = pipeline.postprocess(actual, candidate_file, manifest, pipeline.DEFAULT_VARIANT,
                    align_windows=True, midpoint_source=(source_dir, str(profile)))
        pipeline._dump(candidate_file.with_suffix(".source-review.json"),
                    pipeline._shp_source_review(source_dir, str(profile), manifest, candidate_file))
        report["default_postprocess_executed"] = True
        report["delivery"] = final["decision"]
        final_audit = R.audit_written(ET.parse(candidate_file).getroot(), evidence, context,
                                      require_driving_unchanged=False)
        report["postprocessed_support_audit"] = final_audit
        dump(out / "postprocessed-support-audit.json", final_audit)
        if final_audit["status"] != "PASS":
            report["status"] = "REJECTED_POSTPROCESS_SOURCE_SUPPORT"
            report["whole_map_score_executed"] = False
            stage("REJECTED_AFTER_POSTPROCESS")
            return
        stage("WHOLE_MAP_SCORE")
        import yaml
        from mapforge.validate import scoreboard as sb
        from mapforge.validate.g8_model import json_safe
        policy = yaml.safe_load(sb.POLICY.read_text(encoding="utf8"))
        metrics = sb.evaluate(candidate_file, "shp", shp_source=source)
        row = {"case": P.JUNCTION_ID, "pipeline": "shp", "artifact": candidate_file.name,
               "metrics": metrics, "tiers": sb.apply_tiers(metrics, "shp", policy)}
        board = json_safe({"schema": "mapforge/scoreboard/v1", "run_dir": str(out), "files": 1,
            "policy": {k:policy[k] for k in ("id", "version", "lifecycle")}, "rows": [row],
            "tier_pass": {tier: int(row["tiers"][tier]["status"] == "PASS") for tier in policy["tiers"]}})
        dump(out / "scoreboard.json", board)
        (out / "scoreboard.md").write_text(sb.markdown(board), encoding="utf8")
        report.update(status="EXPERIMENT_EVALUATED", candidate_sha256=sha(candidate_file),
                      whole_map_score_executed=True, tiers=row["tiers"])
    except Exception as exc:
        report.update(status="FAILED", error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        report["elapsed_s"] = round(time.monotonic()-start, 3)
        report["code_policy_changed"] = [name for name,value in bindings.items() if sha(ROOT/name) != value]
        if snapshot is not None:
            from mapforge.workbench.sources import verify_source_snapshot
            report["source_integrity_after"] = verify_source_snapshot(snapshot, source_dir, profile)
        if report["code_policy_changed"] or not report.get("source_integrity_after", {}).get("matches", False):
            report["status_before_integrity_rejection"] = report["status"]
            report["status"] = "REJECTED_INPUT_DRIFT"
        report["candidate_accepted"] = False
        dump(out / "report.json", report)
        print(json.dumps({k:report.get(k) for k in ("status", "stage", "elapsed_s", "error")}, ensure_ascii=False))
        log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    process = multiprocessing.get_context("spawn").Process(target=worker, args=(str(out),))
    process.start()
    print(json.dumps({"worker_pid": process.pid, "parent_pid": os.getpid(), "out": str(out)}), flush=True)
    previous = None
    started = time.monotonic()
    while process.is_alive():
        process.join(5)
        path = out / "report.json"
        if path.exists():
            try:
                report = json.loads(path.read_text(encoding="utf8"))
            except json.JSONDecodeError:
                continue
            if report.get("stage") != previous:
                previous = report.get("stage")
                print(previous, flush=True)
        if time.monotonic()-started > 7200:
            process.terminate()
            process.join(10)
            dump(out / "parent-timeout.json", {"status":"TIMED_OUT", "worker_pid":process.pid})
            raise SystemExit(2)
    print(f"worker_exit={process.exitcode}", flush=True)
    if (out / "report.json").exists():
        report = json.loads((out / "report.json").read_text(encoding="utf8"))
        print(json.dumps({k:report.get(k) for k in ("status", "elapsed_s", "error")}, ensure_ascii=False))
    raise SystemExit(process.exitcode or 0)


if __name__ == "__main__":
    main()
