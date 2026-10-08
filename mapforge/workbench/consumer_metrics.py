"""Versioned, opt-in RoadManager transport replacement for existing metrics.

The original scoreboard still computes every metric. Only the two consumer
values may be replaced, after a complete frozen check on the same input bytes.
This module does not change geometry, policy, tiers, or delivery decisions.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re

from mapforge.validate import scoreboard as sb

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "mapforge/workbench-consumer-metrics/v1"
CONSUMER_SCHEMA = "mapforge/esmini-portable-check/v1"
REPLACED_METRICS = ("esmini_pass", "esmini_gap_max_cm")
TRANSPORT_FILES = ("input", "dll", "checker", "adapter")


def _binding_paths(xodr):
    return {
        "input": xodr,
        "dll": ROOT / "esmini/bin/esminiRMLib.dll",
        "checker": ROOT / "scripts/esmini_rm_check.py",
        "adapter": ROOT / "scripts/workbench_esmini_portable.py",
        "scoreboard": Path(sb.__file__).resolve(),
        "metrics_adapter": Path(__file__).resolve(),
    }


def _bindings(paths):
    result, errors = {}, []
    for name, path in paths.items():
        try:
            path = Path(path).resolve(strict=True)
            data = path.read_bytes()
            result[name] = {"path": str(path), "size": len(data),
                            "sha256": hashlib.sha256(data).hexdigest()}
        except (OSError, ValueError) as exc:
            result[name] = None
            errors.append(f"{name}: {type(exc).__name__}: {exc}")
    return result, errors


def _check_file(xodr):
    from scripts.workbench_esmini_portable import check_file
    return check_file(xodr)


def _consumer_problems(result, xodr, before, after):
    """Validate the transport contract without introducing geometry thresholds."""
    problems = []
    if not isinstance(result, dict):
        return ["consumer did not return an object"]
    if result.get("schema") != CONSUMER_SCHEMA:
        problems.append("unsupported consumer schema")
    if result.get("status") != "CHECKED":
        problems.append("consumer did not complete the full check")
    verdict = result.get("pass")
    if type(verdict) is not bool:
        problems.append("consumer verdict is not a boolean")
    gap = result.get("gap_max_cm")
    valid_gap = type(gap) in (int, float) and math.isfinite(gap) and gap >= 0
    if not valid_gap:
        problems.append("consumer gap is missing or invalid")
    line = result.get("line")
    pattern = (re.escape(xodr.name) + r":\s*换乘缝隙 max 进侧 ([0-9.]+)cm / "
               r"出侧 ([0-9.]+)cm[^\r\n]* -> (PASS|CHECK)")
    match = re.fullmatch(pattern, line) if isinstance(line, str) else None
    if match is None:
        problems.append("consumer has no complete summary for the actual input")
    else:
        try:
            gaps = [float(match[1]), float(match[2])]
            if (not all(math.isfinite(v) and v >= 0 for v in gaps)
                    or not valid_gap or max(gaps) != gap):
                problems.append("consumer gap differs from its complete summary")
        except ValueError:
            problems.append("consumer summary gap is invalid")
        if type(verdict) is not bool or (match[3] == "PASS") != verdict:
            problems.append("consumer summary and verdict disagree")
    evidence = result.get("evidence")
    if not isinstance(evidence, dict):
        return problems + ["consumer transport evidence is missing"]
    for flag in ("exact_bytes", "full_check_executed", "unchanged"):
        if evidence.get(flag) is not True:
            problems.append("consumer did not establish " + flag)
    if evidence.get("loader") != "RM_InitWithString":
        problems.append("consumer used a different loader")
    if type(evidence.get("worker_exit_code")) is not int or evidence["worker_exit_code"] != 0:
        problems.append("consumer worker did not complete successfully")
    if not all(isinstance(evidence.get(name), str) for name in ("stdout", "stderr")):
        problems.append("consumer process output is missing")
    for phase, actual in (("before", before), ("after", after)):
        expected = {name: actual[name] for name in TRANSPORT_FILES}
        reported = evidence.get(phase)
        if (any(value is None for value in expected.values())
                or not isinstance(reported, dict) or reported != expected):
            problems.append("consumer " + phase + " binding differs from actual files")
    return problems


def evaluate(xodr, pipeline, schema=None, shp_source=None):
    """Return (original metrics with two replacements, bound transport evidence).

    A consumer exception or incomplete result is a failed consumer metric, never
    a fallback to an old PASS. Original scoreboard exceptions propagate: there
    are then no complete original metrics to retain. No files are written.
    """
    xodr = Path(xodr).resolve()
    paths = _binding_paths(xodr)
    before, before_errors = _bindings(paths)
    original = sb.evaluate(xodr, pipeline, schema=schema, shp_source=shp_source)
    metrics = dict(original)
    legacy = {name: {"present": name in original, "value": original.get(name)}
              for name in REPLACED_METRICS}
    result, error = None, None
    try:
        result = _check_file(xodr)
    except Exception as exc:
        error = {"type": type(exc).__name__, "message": str(exc)}
    after, after_errors = _bindings(paths)
    unchanged = not before_errors and not after_errors and before == after
    problems = [*before_errors, *after_errors]
    if not unchanged:
        problems.append("input or implementation changed during evaluation")
    if error:
        problems.append("consumer raised an exception")
    else:
        problems.extend(_consumer_problems(result, xodr, before, after))
    retained_result = result
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError):
        retained_result = {"invalid_return_repr": repr(result)}
        problems.append("consumer return is not finite JSON evidence")
    valid = not problems
    metrics["esmini_pass"] = result["pass"] if valid else False
    metrics["esmini_gap_max_cm"] = result["gap_max_cm"] if valid else None
    transport = {
        "schema": SCHEMA,
        "scope": "consumer transport only; not product acceptance or a policy change",
        "status": "CHECKED" if valid else ("FAILED" if error else "INVALID"),
        "replaced_metrics": list(REPLACED_METRICS),
        "legacy_consumer_metrics": legacy,
        "consumer_result": retained_result,
        "before": before,
        "after": after,
        "unchanged": unchanged,
        "problems": problems,
        "error": error,
    }
    return metrics, transport
