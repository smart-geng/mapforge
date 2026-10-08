"""Strict process/result wrapper for the new WB11 supported-tracks experiment.

Exit 0 means a complete EXPERIMENT_EVALUATED record, including its FAIL scores
if any. It NEVER means candidate acceptance or product release. Rejection is
exit 2, timeout exit 3, and errors/incomplete evidence exit 1. The preceding
source-cells probe and wrapper are unchanged. A fresh output directory is used.
The outer timeout defaults to 7300 seconds. This probe has its own
7200-second timeout; increasing --timeout cannot extend that inner limit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
PROBE = ROOT / "scripts/workbench_probe_source_tracks.py"
SCHEMA = "mapforge/wb11-source-tracks-runner/v1"
REPORT_SCHEMA = "mapforge/wb11-source-tracks-probe/v1"
REJECTED = {"REJECTED_LOCAL_SOURCE_SUPPORT", "REJECTED_POSTPROCESS_SOURCE_SUPPORT", "REJECTED_INPUT_DRIFT",
            "REJECTED_LOCAL_STRUCTURE", "REJECTED_PREPROCESS_MOUTH_REGRESSION", "REJECTED_POSTPROCESS_MOUTHS"}
# Additional repository inputs used by this fixed SHP postprocess/score path.
# decision.finalize_opendrive_g8 reads G11_POLICY; scoreboard.evaluate reads the
# standalone XSD and launches this helper, whose only repository runtime input
# is the named DLL. The schema has no xs:include/import/redefine dependencies.
# Non-Python scoring inputs remain separately bound by the parent. The new
# probe additionally binds every repository script, including consumer helpers.
EXTRA_DEPENDENCIES = (
    "profiles/validation/g11-opendrive-v1.draft.yaml",
    "OpenDRIVE_1.5M.xsd",
    "scripts/esmini_rm_check.py",
    "esmini/bin/esminiRMLib.dll",
)
REQUIRED_OUTPUTS = (
    "report.json", "source-snapshot.json", "confirmed-probe-intent.json",
    "code-policy-binding.json", "source-generation.stats.json",
    "source-generated.partial.xodr", "source-generated.partial.source-lanes.json",
    "surface.unaccepted.xodr", "surface.unaccepted.source-lanes.json", "surface-evidence.json",
    "postprocessed-support-audit.json", "candidate.xodr", "candidate.source-review.json",
    "candidate.source-lanes.json", "candidate.transform.json", "candidate.g8.json",
    "candidate.g11.json", "candidate.edge-contacts.json", "candidate.quality-report.json",
    "candidate.delivery-decision.json", "scoreboard.json", "scoreboard.md", "worker.log",
    "local-structure.json", "local-consumer.json",
    "original-surface.reference.unaccepted.xodr", "original-surface.reference-evidence.json",
    "preprocess-mouth-comparison.json",
    "code-snapshots/mapforge/workbench/source_surface_tracks.py",
    "code-snapshots/mapforge/workbench/source_surface_reconstruction.py",
    "code-snapshots/mapforge/workbench/source_surface_support.py",
    "code-snapshots/scripts/workbench_probe_source_tracks.py",
)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf8")


def _file_binding(path):
    path = Path(path)
    data = path.read_bytes()
    return {"path": str(path.resolve()), "size": len(data), "sha256": _sha(data)}


def _extra_dependency_binding(relative):
    path = ROOT / relative
    if (Path(relative).is_absolute() or not path.is_file() or path.is_symlink()
            or not path.resolve().is_relative_to(ROOT.resolve())):
        raise ValueError("missing or nonlocal scoring dependency: " + relative)
    return _file_binding(path)


def _json(data):
    # Reject NaN/Infinity and duplicate keys; neither is valid frozen evidence.
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result
    def invalid(value):
        raise ValueError("nonfinite JSON number: " + value)
    value = json.loads(data.decode("utf8"), object_pairs_hook=pairs, parse_constant=invalid)
    if not isinstance(value, dict):
        raise ValueError("JSON evidence must be an object")
    return value


def _required_code_paths():
    return [*sorted(ROOT.glob("mapforge/**/*.py")), *sorted(ROOT.glob("scripts/**/*.py")), PROBE, ROOT / "uv.lock",
            ROOT / "profiles/shp/ibd-smarteditor-v1.yaml",
            ROOT / "profiles/validation/static-acceptance-v1.draft.yaml",
            ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"]


def assess_output(out: Path, child_exit_code: int, *, timed_out=False) -> dict:
    """Read the terminal evidence; file existence/process exit alone cannot pass."""
    out = Path(out).resolve()
    result = {"status": "INCOMPLETE_EVIDENCE", "exit_code": 1, "problems": [],
              "report_status": None, "evidence_bindings": {}, "experiment_complete": False,
              "diagnostic_evaluation_complete": False,
              "candidate_accepted": False, "formal_release_verified": False}
    cache = {}

    def read(name):
        if name in cache:
            return cache[name]
        path = out / name
        if not path.is_file() or path.is_symlink() or not path.resolve().is_relative_to(out):
            raise ValueError("missing or nonlocal evidence: " + name)
        data = path.read_bytes()
        if not data:
            raise ValueError("empty evidence: " + name)
        result["evidence_bindings"][name] = {"sha256": _sha(data), "size": len(data)}
        cache[name] = data
        return data

    try:
        report = _json(read("report.json"))
        result["report_status"] = report.get("status")
    except (OSError, ValueError) as exc:
        result.update(status="INVALID_REPORT", problems=[str(exc)])
        report = None
    if timed_out:
        return {**result, "status": "TIMEOUT", "exit_code": 3,
                "problems": [*result["problems"], "owned child tree exceeded deadline"]}
    controlled_rejection = (child_exit_code == 2 and isinstance(report, dict)
                            and report.get("status") in REJECTED)
    if child_exit_code != 0 and not controlled_rejection:
        return {**result, "status": "CHILD_FAILED", "exit_code": 1,
                "problems": [*result["problems"], f"child exit code {child_exit_code}"]}
    if report is None:
        return result
    if (report.get("schema") != REPORT_SCHEMA
            or report.get("candidate_accepted") is not False
            or report.get("formal_release_verified") is not False
            or report.get("independent_operator_verified") is not False):
        return {**result, "status": "INVALID_REPORT", "problems": ["invalid schema or experiment-only boundary"]}
    rejected_mouths = report.get("status") == "REJECTED_POSTPROCESS_MOUTHS"
    if report.get("status") in REJECTED and not rejected_mouths:
        return {**result, "status": "REJECTED", "exit_code": 2,
                "problems": ["probe explicitly rejected: " + report["status"]]}
    if report.get("status") != "EXPERIMENT_EVALUATED" and not rejected_mouths:
        return {**result, "status": "INVALID_REPORT", "problems": ["missing or unsuccessful terminal status"]}
    try:
        if (report.get("stage") != "WHOLE_MAP_SCORE"
                or report.get("default_postprocess_executed") is not True
                or report.get("whole_map_score_executed") is not True
                or report.get("code_policy_changed") != []
                or report.get("source_integrity_after") != {"matches": True, "issues": []}):
            raise ValueError("phase completion or source/code integrity is not verified")
        documents = {name: _json(read(name)) for name in REQUIRED_OUTPUTS if name.endswith(".json")}
        for name in REQUIRED_OUTPUTS:
            read(name)
        for key, name in (("source_partial_sha256", "source-generated.partial.xodr"),
                          ("unaccepted_surface_sha256", "surface.unaccepted.xodr"),
                          ("original_surface_reference_sha256", "original-surface.reference.unaccepted.xodr"),
                          ("candidate_sha256", "candidate.xodr")):
            if report.get(key) != _sha(read(name)):
                raise ValueError("artifact hash mismatch: " + name)
        snapshot = documents["source-snapshot.json"]
        intent = documents["confirmed-probe-intent.json"]
        if (not snapshot.get("snapshot_id")
                or report.get("source_snapshot_id") != snapshot["snapshot_id"]
                or intent.get("source_snapshot_id") != snapshot["snapshot_id"]
                or intent.get("source_content_hash") != snapshot.get("content_hash")
                or intent.get("junction_id") != snapshot.get("junction_id")
                or intent.get("confirmed") is not True):
            raise ValueError("source snapshot/intent identity mismatch")
        evidence = documents["surface-evidence.json"]
        if (evidence.get("written_sha256") != report["unaccepted_surface_sha256"]
                or evidence.get("intent") != intent or evidence.get("candidate_accepted") is not False
                or evidence.get("non_paving_unchanged") is not True
                or evidence.get("actual_xml_support_audit") != report.get("local_support_audit")):
            raise ValueError("surface evidence/intent/local audit mismatch")
        final_audit = documents["postprocessed-support-audit.json"]
        if final_audit != report.get("postprocessed_support_audit"):
            raise ValueError("postprocessed support audit mismatch")
        for index, audit in enumerate((report.get("local_support_audit"), final_audit)):
            if (not isinstance(audit, dict) or audit.get("material_support_passed") is not True
                    or audit.get("status") not in ({"PASS", "FAIL"} if index == 0
                                                   else {"FAIL" if rejected_mouths else "PASS"})
                    or audit.get("candidate_accepted") is not False
                    or audit.get("driving_continuity_proven") is not False
                    or audit.get("source_support_audit", {}).get("passed") is not True
                    or not audit.get("families")
                    or any(f.get("passed") is not True for f in audit["families"])):
                raise ValueError("source-support phase did not pass")
            mouth = audit.get("written_road_mouth_coverage", {})
            if (not isinstance(mouth, dict)
                    or not isinstance(mouth.get("within_existing_source_cover_tolerance"), bool)
                    or (index == 1 and mouth["within_existing_source_cover_tolerance"] is not (not rejected_mouths))
                    or mouth.get("driving_continuity_proven") is not False
                    or mouth.get("route_seams_evaluated") is not False):
                raise ValueError("written road mouth coverage is missing or contradictory")
            if (audit["status"] == "PASS") != mouth["within_existing_source_cover_tolerance"]:
                raise ValueError("mouth diagnostic status contradicts local audit")
        comparison = documents["preprocess-mouth-comparison.json"]
        reference = documents["original-surface.reference-evidence.json"]
        if (comparison != report.get("preprocess_mouth_comparison")
                or comparison.get("allow_postprocess") is not True
                or comparison.get("current_material_support_passed") is not True
                or comparison.get("non_paving_unchanged") is not True
                or comparison.get("regressions") != []
                or comparison.get("final_mouth_gate_unchanged") is not True
                or comparison.get("current_mouth_coverage") != report["local_support_audit"]["written_road_mouth_coverage"]
                or reference.get("written_sha256") != report["original_surface_reference_sha256"]
                or reference.get("non_paving_subtrees_before") != evidence.get("non_paving_subtrees_before")):
            raise ValueError("fresh partial mouth inheritance proof is missing or inconsistent")
        structure = documents["local-structure.json"]
        consumer = documents["local-consumer.json"]
        if (structure != report.get("local_structure") or consumer != report.get("local_consumer")
                or structure.get("xsd", {}).get("passed") is not True
                or structure.get("auxiliary_hard_failures") != [] or consumer.get("status") != "PASS"):
            raise ValueError("local structure or consumer readback did not pass")
        code = documents["code-policy-binding.json"]
        required = {p.relative_to(ROOT).as_posix() for p in _required_code_paths()}
        if not required <= code.keys():
            raise ValueError("incomplete code/policy binding")
        for name, expected in code.items():
            relative = Path(name)
            path = (ROOT / relative).resolve()
            if relative.is_absolute() or not path.is_relative_to(ROOT) or _sha(path.read_bytes()) != expected:
                raise ValueError("code/policy binding changed: " + name)
        for name in REQUIRED_OUTPUTS:
            if name.startswith("code-snapshots/"):
                relative = name.removeprefix("code-snapshots/")
                if _sha(read(name)) != code.get(relative):
                    raise ValueError("code snapshot mismatch: " + name)
        quality = documents["candidate.quality-report.json"]
        delivery = documents["candidate.delivery-decision.json"]
        if (quality.get("schema") != "mapforge/opendrive-quality-report/v1"
                or Path(quality.get("artifact", "")).resolve() != out / "candidate.xodr"
                or quality.get("delivery_decision") != delivery or delivery != report.get("delivery")
                or delivery.get("schema") != "mapforge/delivery-decision/v1"):
            raise ValueError("quality/delivery evidence mismatch")
        for gate, name in (("G8", "candidate.g8.json"), ("G11", "candidate.g11.json"),
                           ("G11-edge-contacts", "candidate.edge-contacts.json")):
            if quality.get("gates", {}).get(gate) != documents[name] or not documents[name].get("status"):
                raise ValueError("gate evidence mismatch: " + gate)
        board = documents["scoreboard.json"]
        rows = board.get("rows")
        if (board.get("schema") != "mapforge/scoreboard/v1" or board.get("files") != 1
                or Path(board.get("run_dir", "")).resolve() != out
                or not isinstance(rows, list) or len(rows) != 1):
            raise ValueError("invalid single-case scoreboard")
        row = rows[0]
        if (not isinstance(row, dict) or row.get("artifact") != "candidate.xodr"
                or row.get("case") != intent["junction_id"] or row.get("pipeline") != "shp"
                or not isinstance(row.get("metrics"), dict) or not row["metrics"]
                or not isinstance(row.get("tiers"), dict) or not row["tiers"]
                or row["tiers"] != report.get("tiers")):
            raise ValueError("scoreboard row/report mismatch")
        expected_counts = {tier: int(value.get("status") == "PASS") for tier, value in row["tiers"].items()}
        if board.get("tier_pass") != expected_counts:
            raise ValueError("scoreboard tier counts mismatch")
        for name, data in cache.items():
            if (out / name).read_bytes() != data:
                raise ValueError("evidence changed while checking: " + name)
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        result["problems"].append(str(exc))
        return result
    if rejected_mouths:
        return {**result, "status": "REJECTED_POSTPROCESS_MOUTHS", "exit_code": 2,
                "diagnostic_evaluation_complete": True, "experiment_complete": False,
                "problems": ["complete diagnostic scores retained; final mouth coverage still fails"]}
    return {**result, "status": "EXPERIMENT_COMPLETE", "exit_code": 0, "experiment_complete": True,
            "diagnostic_evaluation_complete": True}


# Reuse the unchanged, tested owned-process implementation.
from scripts.workbench_run_source_cells import _run_child


def run_experiment(out: Path, timeout_s: float = 7300) -> dict:
    original = Path(out)
    result = {"schema": SCHEMA, "status": "RUNNER_FAILED", "exit_code": 1,
              "experiment_complete": False, "candidate_accepted": False, "formal_release_verified": False,
              "diagnostic_evaluation_complete": False,
              "limitation": "Exit zero means the experiment was fully evaluated, including FAIL scores; never candidate, operator, map or product acceptance.",
              "runner_result_path": None}
    if original.exists() or original.is_symlink():
        return {**result, "status": "OUTPUT_EXISTS", "problems": ["output must not already exist"]}
    if sys.version_info[:3] != (3, 11, 16):
        return {**result, "status": "UNSUPPORTED_RUNTIME", "problems": ["Python 3.11.16 is required for this bound experiment"]}
    out = original.resolve()
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        return {**result, "status": "INVALID_TIMEOUT", "problems": ["timeout must be finite and positive"]}
    command = [sys.executable, str(PROBE.resolve()), "--out", str(out)]
    binding = {"child_command": command, "child_command_sha256": _sha(_canonical(command)),
               "python": {"executable": sys.executable, "version": sys.version},
               "extra_dependencies": {}}
    result["binding"] = binding
    start = time.monotonic()
    process = None
    try:
        binding["runner"] = _file_binding(Path(__file__))
        binding["worker"] = _file_binding(PROBE)
        for relative in EXTRA_DEPENDENCIES:
            binding["extra_dependencies"][relative] = _extra_dependency_binding(relative)
    except Exception as exc:
        result.update(status="DEPENDENCY_UNAVAILABLE", problems=[f"{type(exc).__name__}: {exc}"])
    else:
        try:
            process = _run_child(command, timeout_s)
            result.update(assess_output(out, process["exit_code"], timed_out=process["timed_out"]))
        except Exception as exc:
            result.update(status="RUNNER_FAILED", problems=[f"{type(exc).__name__}: {exc}"])
    result["elapsed_s"] = time.monotonic() - start
    result["timeout_s"] = timeout_s
    result["probe_internal_timeout_s"] = 7200
    result["timeout_scope"] = "Outer timeout includes bootstrap; the unchanged probe independently stops its worker after 7200 seconds. Increasing the outer timeout cannot extend that limit."
    result["probe_created_output_directory"] = out.is_dir()
    for label in ("runner", "worker"):
        if label not in binding:
            continue
        try:
            changed = _file_binding(binding[label]["path"])["sha256"] != binding[label]["sha256"]
        except OSError:
            changed = True
        if changed:
            result.update(status="BINDING_CHANGED", exit_code=1, experiment_complete=False,
                          diagnostic_evaluation_complete=False)
            result.setdefault("problems", []).append(label + " changed during execution")
    dependency_issues = []
    dependency_after = {}
    for relative, before in binding["extra_dependencies"].items():
        try:
            after = _extra_dependency_binding(relative)
            dependency_after[relative] = after
            if after != before:
                dependency_issues.append({"path": relative, "code": "dependency-changed"})
        except (OSError, ValueError) as exc:
            dependency_issues.append({"path": relative, "code": "dependency-unavailable", "error": str(exc)})
    complete_dependency_set = set(binding["extra_dependencies"]) == set(EXTRA_DEPENDENCIES)
    result["extra_dependency_integrity_after"] = {
        "matches": complete_dependency_set and not dependency_issues,
        "issues": dependency_issues, "bindings": dependency_after,
    }
    if dependency_issues:
        result.update(status="BINDING_CHANGED", exit_code=1, experiment_complete=False,
                      diagnostic_evaluation_complete=False)
        result.setdefault("problems", []).extend(
            "scoring dependency changed during execution: " + issue["path"]
            for issue in dependency_issues)
    if process:
        result["process"] = {key: value for key, value in process.items() if key not in {"stdout", "stderr"}}
    try:
        # No directory was created before giving the new path to the probe.
        # If it failed before mkdir, retain its FAILURE evidence here afterwards.
        out.mkdir(parents=True, exist_ok=True)
        if out.is_symlink() or out.resolve() != out:
            raise ValueError("output directory was redirected")
        if process:
            for channel in ("stdout", "stderr"):
                path = out / ("runner-child." + channel + ".txt")
                with path.open("xb") as stream:
                    stream.write(process[channel])
                result.setdefault("evidence_bindings", {})[path.name] = _file_binding(path)
        target = out / "runner-result.json"
        result["runner_result_path"] = str(target)
        # Exclusive creation preserves any conflicting evidence from a child.
        with target.open("x", encoding="utf8", newline="\n") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError) as exc:
        result.update(status="RESULT_PUBLICATION_FAILED", exit_code=1, experiment_complete=False,
                      diagnostic_evaluation_complete=False,
                      runner_result_path=None)
        result.setdefault("problems", []).append(str(exc))
    return result


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--child-bootstrap":
        if sys.stdin.buffer.readline() != b"RUN\n":
            return 1
        command = json.loads(sys.argv[2])
        if not isinstance(command, list) or not all(isinstance(v, str) for v in command):
            return 1
        return subprocess.run(command, cwd=ROOT, shell=False).returncode
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=7300,
                        help="outer deadline in seconds (default 7300); cannot extend the probe's fixed 7200-second limit")
    args = parser.parse_args()
    result = run_experiment(args.out, args.timeout)
    print(json.dumps({key: result.get(key) for key in
                     ("status", "report_status", "exit_code", "runner_result_path", "problems")}, ensure_ascii=False))
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
