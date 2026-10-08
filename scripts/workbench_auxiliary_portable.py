"""Opt-in byte transport for the existing auxiliary RoadManager probe.

The existing build_probes/_check_rm calculations, lane types, sample stations
and tolerances are reused unchanged. Only loading the self-contained XML uses
RM_InitWithString, so the engineering directory may contain Chinese characters.
No map or production acceptance is granted by this transport check.
"""
from __future__ import annotations

import ctypes
import json
import math
from numbers import Real
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import workbench_check_auxiliary_consumer as legacy
from scripts import workbench_esmini_portable as transport


def _binding(path, dll):
    result = legacy._binding(path, dll)
    for p in (Path(__file__).resolve(), Path(transport.__file__).resolve()):
        result["implementation"].append({"path": str(p), "sha256": legacy._sha(p.read_bytes())})
    return result


def _worker(binding):
    result = legacy._base(binding)
    result.update(worker_pid=os.getpid(), loader="RM_InitWithString", exact_bytes=False)
    if drift := legacy._drift(binding):
        return {**result, "status": "BINDING_CHANGED", "binding_drift": drift}
    data = Path(binding["xodr"]["path"]).read_bytes()
    validation = transport.validate_input(data)
    if validation["sha256"] != binding["xodr"]["sha256"]:
        raise ValueError("input changed before byte initialization")
    plan = legacy.build_probes(data)
    result.update(roads=plan["roads"], planned_width_probes=len(plan["probes"]),
                  planned_position_probes=3*len(plan["probes"]), input_validation=validation)
    rm = ctypes.CDLL(binding["consumer"]["path"])
    legacy._bind(rm)
    rm.RM_InitWithString.argtypes = [ctypes.c_char_p]
    rm.RM_InitWithString.restype = ctypes.c_int
    try:
        result["load_code"] = rm.RM_InitWithString(data)
        if result["load_code"] != 0:
            return {**result, "status": "LOAD_FAILED"}
        result["exact_bytes"] = True
        handle = rm.RM_CreatePosition()
        result["position_handle"] = handle
        if handle < 0:
            return {**result, "status": "CREATE_POSITION_FAILED"}
        rows = legacy._check_rm(rm, handle, plan["probes"])
        failures = [row for row in rows if not row["ok"]]
        zeros = [row for row in rows if row["zero_width"] and row["road_endpoint"]]
        result.update(status="FAIL" if failures else "PASS", rows=rows, failures=failures,
            checked_positions=sum(row["codes"]["position"] is not None for row in rows),
            checked_widths=len(plan["probes"]), zero_width_endpoint_rows=zeros,
            zero_width_endpoint_support=("FAIL" if any(not row["ok"] for row in zeros)
                                         else "PASS" if zeros else "NOT_PRESENT"))
        for field in ("width_error_m", "position_error_m", "station_error_m"):
            result["max_"+field] = max((row[field] for row in rows if row.get(field) is not None), default=None)
    finally:
        rm.RM_Close()
        if drift := legacy._drift(binding):
            result.update(status="BINDING_CHANGED", binding_drift=drift)
    return result


def _number(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError("consumer report contains a missing or nonfinite number")
    return value


def _validate_pass(result, plan, validation, exit_code):
    """Independently account for the unchanged XML probe plan and readback.

    Counts or a worker's row.ok alone cannot establish complete execution.
    This repeats only the old fixed readback comparisons, not a new map gate.
    """
    count = len(plan["probes"])
    if (not count or exit_code != 0 or result.get("exact_bytes") is not True
            or type(result.get("load_code")) is not int or result["load_code"] != 0
            or type(result.get("position_handle")) is not int or result["position_handle"] < 0
            or type(result.get("worker_pid")) is not int
            or result["worker_pid"] <= 0 or result["worker_pid"] == os.getpid()
            or result.get("input_validation") != validation
            or result.get("roads") != plan["roads"]):
        raise ValueError("consumer completion does not support PASS")
    for name, expected in (("planned_width_probes", count), ("checked_widths", count),
                           ("planned_position_probes", 3*count), ("checked_positions", 3*count)):
        if type(result.get(name)) is not int or result[name] != expected:
            raise ValueError("consumer probe accounting differs from complete XML plan: " + name)
    base = legacy._base(result["binding"])
    for name in ("scope", "limitations", "binding_api", "position_tolerance_m", "width_tolerance_m",
                 "station_tolerance_m", "side_epsilon_max_m", "tolerance_basis", "skipped"):
        if result.get(name) != base[name]:
            raise ValueError("consumer report changed fixed scope or policy: " + name)
    rows = result.get("rows")
    if not isinstance(rows, list) or len(rows) != 3*count or result.get("failures") != []:
        raise ValueError("consumer rows or failures do not establish complete PASS")
    errors = {name: [] for name in ("width_error_m", "position_error_m", "station_error_m")}
    for probe, group in zip(plan["probes"], (rows[i:i+3] for i in range(0,len(rows),3))):
        for row, (edge, direction) in zip(group, (("inner",-1),("center",0),("outer",1))):
            if (not isinstance(row, dict) or any(row.get(k) != v for k,v in probe.items())
                    or row.get("edge") != edge or row.get("ok") is not True
                    or row.get("failure_reasons") != []):
                raise ValueError("consumer row is missing, duplicated or inconsistent with its XML probe")
            codes = row.get("codes")
            if (not isinstance(codes, dict) or set(codes) != {"width","position","read"}
                    or any(type(v) is not int or v < 0 for v in codes.values())):
                raise ValueError("consumer PASS row lacks successful native return codes")
            if (type(row.get("actual_road_id")) is not int or type(row.get("actual_lane_id")) is not int
                    or row["actual_road_id"] != probe["road_id"] or row["actual_lane_id"] != probe["lane_id"]):
                raise ValueError("consumer PASS row changed road or lane identity")
            width = _number(row.get("actual_width_m"))
            xy = row.get("actual_xy")
            if width < 0 or not isinstance(xy,list) or len(xy) != 2:
                raise ValueError("consumer PASS row lacks nonnegative width or complete XY")
            xy = [_number(v) for v in xy]
            station = _number(row.get("actual_s_m"))
            offset = width/2*(1 if probe["lane_id"] > 0 else -1)*direction
            if _number(row.get("offset_m")) != offset:
                raise ValueError("consumer lateral offset disagrees with read width")
            measured = {"width_error_m": abs(width-probe["expected_width_m"]),
                        "position_error_m": math.hypot(xy[0]-probe["expected_xy"][edge][0],
                                                       xy[1]-probe["expected_xy"][edge][1]),
                        "station_error_m": abs(station-probe["s_m"])}
            for field, error in measured.items():
                if _number(row.get(field)) != error or error > legacy.TOLERANCE_M:
                    raise ValueError("consumer PASS row contradicts fixed readback tolerance: " + field)
                errors[field].append(error)
        if len({row["actual_width_m"] for row in group}) != 1:
            raise ValueError("three edge rows disagree about their single width readback")
    zeros = [r for r in rows if r["zero_width"] and r["road_endpoint"]]
    if (result.get("zero_width_endpoint_rows") != zeros
            or result.get("zero_width_endpoint_support") != ("PASS" if zeros else "NOT_PRESENT")):
        raise ValueError("zero-width endpoint accounting is incomplete")
    for field, values in errors.items():
        if _number(result.get("max_"+field)) != max(values):
            raise ValueError("consumer maximum differs from complete row errors: " + field)


def audit_file(path, *, dll=legacy.DLL, timeout_s=120):
    """Use a fresh native process; preserve the original probe's report schema."""
    binding = None
    result = {"schema": legacy.SCHEMA, "status": "UNAVAILABLE", "map_accepted": False,
              "binding": None, "loader": "RM_InitWithString", "exact_bytes": False,
              "rows": [], "failures": [], "zero_width_endpoint_support": "NOT_TESTED"}
    try:
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout must be finite and positive")
        path, dll = Path(path).resolve(), Path(dll).resolve()
        binding = _binding(path, dll)
        result.update(legacy._base(binding))
        if not dll.is_file():
            raise ValueError("esmini RoadManager DLL is missing")
        data = path.read_bytes()
        validation = transport.validate_input(data)
        if validation["sha256"] != binding["xodr"]["sha256"]:
            raise ValueError("input changed before parent probe planning")
        plan = legacy.build_probes(data)
        with tempfile.TemporaryDirectory(prefix="mapforge-auxiliary-bytes-") as folder:
            request, output = Path(folder)/"request.json", Path(folder)/"result.json"
            request.write_text(json.dumps(binding, allow_nan=False), encoding="utf8")
            command = [sys.executable, "-I", "-B", str(Path(__file__).resolve()), "--worker", str(request), str(output)]
            process = subprocess.run(command, cwd=folder, capture_output=True, timeout=timeout_s,
                stdin=subprocess.DEVNULL, shell=False,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            decoded = json.loads(output.read_text(encoding="utf8")) if output.is_file() else None
            if (not isinstance(decoded, dict) or decoded.get("schema") != legacy.SCHEMA
                    or decoded.get("binding") != binding or decoded.get("loader") != "RM_InitWithString"
                    or decoded.get("map_accepted") is not False
                    or decoded.get("status") not in {"PASS", "FAIL", "ERROR", "BINDING_CHANGED",
                                                      "LOAD_FAILED", "CREATE_POSITION_FAILED"}):
                raise ValueError("incomplete or mismatched byte transport result")
            json.dumps(decoded, allow_nan=False)
            result = decoded
            result["process"] = {"exit_code": process.returncode, "argv": command,
                "stdout": process.stdout.decode("utf8", errors="replace"),
                "stderr": process.stderr.decode("utf8", errors="replace")}
            if result["status"] == "PASS":
                _validate_pass(result, plan, validation, process.returncode)
    except subprocess.TimeoutExpired:
        result.update(status="TIMEOUT", error=f"consumer exceeded {timeout_s} seconds")
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
        result.update(status="UNAVAILABLE", error=f"{type(exc).__name__}: {exc}")
    if binding is not None and (drift := legacy._drift(binding)):
        result.update(status="BINDING_CHANGED", binding_drift=drift)
    return result


def main():
    if len(sys.argv) != 4 or sys.argv[1] != "--worker":
        raise SystemExit("Internal worker; use audit_file from the registered source pipeline")
    binding = json.loads(Path(sys.argv[2]).read_text(encoding="utf8"))
    try:
        result = _worker(binding)
    except Exception as exc:
        result = {**legacy._base(binding), "status": "ERROR", "loader": "RM_InitWithString",
                  "exact_bytes": False, "error": f"{type(exc).__name__}: {exc}"}
    with Path(sys.argv[3]).open("x", encoding="utf8") as stream:
        json.dump(result, stream, ensure_ascii=False, allow_nan=False)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
