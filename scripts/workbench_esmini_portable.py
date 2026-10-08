"""Run the frozen RoadManager checker on self-contained UTF-8 bytes.

Only the instance's RM_Init transport changes to RM_InitWithString. The old
checker still parses the original path and applies all its original checks.
Each call owns a fresh subprocess; a timeout kills and waits for that process.
This does not accept a map or alter geometry, thresholds, or the old checker.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import ctypes
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = Path(__file__).resolve()
CHECKER = ROOT / "scripts/esmini_rm_check.py"
DLL = ROOT / "esmini/bin/esminiRMLib.dll"
FROZEN_CHECKER_SHA256 = "81e36d15eefe9584c492c707e744c6cd0f822dafde77f2b8e399708d0d41b680"
SCHEMA = "mapforge/esmini-portable-check/v1"


class UnsupportedInput(ValueError):
    pass


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _binding(path, data=None):
    path = Path(path).resolve(strict=True)
    data = path.read_bytes() if data is None else data
    return {"path": str(path), "size": len(data), "sha256": _sha(data)}


def validate_input(data):
    """Reject resource-relative XML instead of silently losing its base path."""
    if not isinstance(data, bytes) or not data or b"\x00" in data:
        raise UnsupportedInput("empty/non-byte input or embedded NUL")
    try:
        text = data.decode("utf-8-sig", errors="strict")
    except UnicodeDecodeError as exc:
        raise UnsupportedInput("OpenDRIVE must be strict UTF-8") from exc
    if re.search(r"<!\s*(?:DOCTYPE|ENTITY)\b", text, re.I):
        raise UnsupportedInput("DTD and entity declarations are unsupported")
    declaration = re.match(r"\s*<\?xml\s+(.*?)\?>", text, re.S)
    if declaration:
        encoding = re.search(r"\bencoding\s*=\s*(['\"])(.*?)\1", declaration[1], re.I)
        if encoding and encoding[2].casefold() not in {"utf-8", "utf8"}:
            raise UnsupportedInput("XML declaration must name UTF-8")
    if re.search(r"<\?(?!xml(?:\s|\?>))", text, re.I):
        raise UnsupportedInput("processing instructions may refer to external resources")
    try:
        root = ET.fromstring(data)
    except (ET.ParseError, ValueError) as exc:
        raise UnsupportedInput("invalid XML") from exc
    if root.tag != "OpenDRIVE":
        raise UnsupportedInput("root must be OpenDRIVE")
    external_tags = {"include", "crg", "opencrg", "file", "external", "externalreference", "resource"}
    external_attributes = {"file", "filename", "filepath", "href", "url", "uri", "src", "path",
                           "base", "schemalocation", "nonamespaceschemalocation", "resource",
                           "datafile", "texturefile"}
    for node in root.iter():
        tag = node.tag.rsplit("}", 1)[-1].casefold()
        if tag in external_tags:
            raise UnsupportedInput("external-resource element: " + tag)
        for name in node.attrib:
            if name.rsplit("}", 1)[-1].casefold() in external_attributes:
                raise UnsupportedInput("external-resource attribute: " + name)
    return {"encoding": "UTF-8", "self_contained": True, "size": len(data), "sha256": _sha(data)}


class _BytesInitProxy:
    """No mutation of the DLL object or of any function in the old checker."""
    def __init__(self, rm, data, original_path):
        self._rm = rm
        self._data = data
        self._original_path = str(original_path).encode("utf8")
        self.loader_calls = 0
        self.load_succeeded = False

    def RM_Init(self, requested_path):
        if requested_path != self._original_path or self.loader_calls:
            raise ValueError("checker requested a different path or repeated initialization")
        self.loader_calls += 1
        result = self._rm.RM_InitWithString(self._data)
        self.load_succeeded = result == 0
        return result

    def __getattr__(self, name):
        return getattr(self._rm, name)


def _summary(text, path, verdict):
    prefix = Path(path).name + ":"
    lines = [line.strip() for line in text.splitlines() if line.startswith(prefix)]
    if len(lines) != 1 or type(verdict) is not bool:
        raise ValueError("old checker did not emit exactly one complete summary")
    line = lines[0]
    pattern = r"换乘缝隙 max 进侧 ([0-9.]+)cm / 出侧 ([0-9.]+)cm.* -> (PASS|CHECK)$"
    match = re.fullmatch(pattern, line[len(prefix):].strip())
    if match is None or (match[3] == "PASS") != verdict:
        raise ValueError("old checker verdict and actual summary disagree")
    gaps = [float(match[1]), float(match[2])]
    if not all(math.isfinite(v) and v >= 0 for v in gaps):
        raise ValueError("old checker summary has invalid gap values")
    return line, max(gaps)


def _load_checker(path):
    spec = importlib.util.spec_from_file_location("_frozen_esmini_portable_checker", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_old_check(rm, checker, data, path):
    if hasattr(rm, "RM_SetLogFilePath"):
        rm.RM_SetLogFilePath.argtypes = [ctypes.c_char_p]
        rm.RM_SetLogFilePath.restype = None
        rm.RM_SetLogFilePath(b"")
    checker._bind(rm)
    rm.RM_InitWithString.argtypes = [ctypes.c_char_p]
    rm.RM_InitWithString.restype = ctypes.c_int
    proxy = _BytesInitProxy(rm, data, path)
    capture = io.StringIO()
    checker_returned = False
    try:
        with redirect_stdout(capture):
            verdict = checker.check(proxy, str(path))
        checker_returned = True  # The frozen checker closes RM before its normal return.
        line, gap = _summary(capture.getvalue(), path, verdict)
        if proxy.loader_calls != 1 or not proxy.load_succeeded:
            raise ValueError("byte loader did not successfully initialize the same document once")
        return {"pass": verdict, "line": line, "gap_max_cm": gap,
                "check_stdout": capture.getvalue(), "loader_calls": proxy.loader_calls,
                "loaded_bytes_sha256": _sha(data), "full_check_executed": True}
    except Exception:
        if proxy.load_succeeded and not checker_returned:
            try:
                rm.RM_Close()
            except Exception:
                pass
        raise
    finally:
        # Keep the original Python output in the owned worker's stdout too;
        # the separately retained string is used to parse its exact summary.
        sys.stdout.write(capture.getvalue())
        sys.stdout.flush()


def _worker(request_path, output_path):
    result = {"status": "FAILED", "pass": False, "gap_max_cm": None, "line": "",
              "full_check_executed": False}
    before = {}
    try:
        request = json.loads(Path(request_path).read_text(encoding="utf8"))
        supplied = request["before"]
        if not isinstance(supplied, dict) or set(supplied) != {"input", "dll", "checker", "adapter"}:
            raise ValueError("incomplete worker binding")
        before = supplied
        for item in before.values():
            if _binding(item["path"]) != item:
                raise ValueError("dependency changed before worker execution")
        if before["checker"]["sha256"] != FROZEN_CHECKER_SHA256:
            raise ValueError("checker differs from frozen original")
        if before["adapter"]["path"] != str(ADAPTER):
            raise ValueError("worker adapter identity mismatch")
        path = Path(before["input"]["path"])
        data = path.read_bytes()
        validation = validate_input(data)
        if validation["sha256"] != before["input"]["sha256"]:
            raise ValueError("input changed before byte loading")
        checker = _load_checker(before["checker"]["path"])
        rm = ctypes.CDLL(before["dll"]["path"])
        result.update(_run_old_check(rm, checker, data, path))
        result.update(status="CHECKED", input_validation=validation)
    except Exception as exc:
        result.update(error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
    after = {}
    errors = []
    for name, item in before.items():
        try:
            after[name] = _binding(item["path"])
        except (OSError, ValueError) as exc:
            errors.append(name + ": " + str(exc))
    result.update(before=before, after=after)
    if errors or after != before:
        result.update(status="FAILED", **{"pass": False}, gap_max_cm=None,
                      full_check_executed=False, error="worker binding drift: " + repr(errors))
    with Path(output_path).open("x", encoding="utf8") as stream:
        json.dump(result, stream, ensure_ascii=False, allow_nan=False)
    return 0 if result["status"] == "CHECKED" else 1


def check_file(path, *, timeout_s=120):
    """Return old-check metrics plus transport evidence; every failure is nonpass."""
    evidence = {"before": {}, "after": {}, "unchanged": False,
                "loader": "RM_InitWithString", "exact_bytes": False,
                "full_check_executed": False, "worker_exit_code": None,
                "stdout": "", "stderr": ""}
    result = {"schema": SCHEMA, "status": "FAILED", "pass": False,
              "gap_max_cm": None, "line": "", "evidence": evidence}
    process = None
    worker = None
    try:
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout must be finite and positive")
        path = Path(path).resolve(strict=True)
        data = path.read_bytes()
        evidence["before"]["input"] = _binding(path, data)
        evidence["input_validation"] = validate_input(data)
        for name, dependency in (("dll", DLL), ("checker", CHECKER), ("adapter", ADAPTER)):
            evidence["before"][name] = _binding(dependency)
        if evidence["before"]["checker"]["sha256"] != FROZEN_CHECKER_SHA256:
            raise ValueError("checker differs from frozen original")
        with tempfile.TemporaryDirectory(prefix="mapforge-esmini-portable-") as temporary:
            directory = Path(temporary)
            request = directory / "request.json"
            response = directory / "result.json"
            request.write_text(json.dumps({"before": evidence["before"]}, ensure_ascii=False), encoding="utf8")
            command = [sys.executable, "-I", "-B", str(ADAPTER), "--worker", str(request), str(response)]
            evidence["command"] = command
            process = subprocess.Popen(command, cwd=directory, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            evidence["worker_pid"] = process.pid
            try:
                stdout, stderr = process.communicate(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()  # Handle-owned worker only; the frozen checker spawns no processes.
                stdout, stderr = process.communicate(timeout=10)
                result["status"] = "TIMED_OUT"
            evidence.update(worker_exit_code=process.returncode,
                            stdout=stdout.decode("utf8", errors="replace"),
                            stderr=stderr.decode("utf8", errors="replace"),
                            stdout_sha256=_sha(stdout), stderr_sha256=_sha(stderr))
            if result["status"] == "TIMED_OUT":
                result["error"] = "owned checker worker exceeded deadline"
            elif process.returncode != 0:
                if response.is_file():
                    worker = json.loads(response.read_text(encoding="utf8"))
                result["error"] = (worker or {}).get("error", "checker worker exited without completion")
            else:
                worker = json.loads(response.read_text(encoding="utf8"))
                if (worker.get("status") != "CHECKED" or worker.get("full_check_executed") is not True
                        or worker.get("before") != evidence["before"]
                        or worker.get("after") != evidence["before"]
                        or worker.get("loader_calls") != 1
                        or worker.get("loaded_bytes_sha256") != evidence["before"]["input"]["sha256"]):
                    raise ValueError("incomplete or mismatched worker evidence")
                line, gap = _summary(worker.get("check_stdout", ""), path, worker.get("pass"))
                if worker.get("line") != line or worker.get("gap_max_cm") != gap:
                    raise ValueError("worker metrics differ from old checker summary")
                result.update(status="CHECKED", **{"pass": worker["pass"]}, line=line, gap_max_cm=gap)
                evidence.update(exact_bytes=True, full_check_executed=True)
            if worker:
                evidence["worker_result"] = worker
                evidence["check_stdout"] = worker.get("check_stdout", "")
    except UnsupportedInput as exc:
        result.update(status="REJECTED_INPUT", error=str(exc))
    except Exception as exc:
        result.update(status="FAILED", error=f"{type(exc).__name__}: {exc}")
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate(timeout=10)
        errors = []
        for name, item in evidence["before"].items():
            try:
                evidence["after"][name] = _binding(item["path"])
            except (OSError, ValueError) as exc:
                errors.append(name + ": " + str(exc))
        evidence["unchanged"] = (set(evidence["before"]) == {"input", "dll", "checker", "adapter"}
                                  and not errors and evidence["after"] == evidence["before"])
        if not evidence["unchanged"] and result["status"] == "CHECKED":
            result.update(status="FAILED", error="input/dependency changed during consumer check: " + repr(errors))
        if result["status"] != "CHECKED":
            result.update(**{"pass": False}, gap_max_cm=None)
            evidence["full_check_executed"] = False
            evidence["exact_bytes"] = False
    return result


def main():
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf8", errors="strict")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", nargs=2, metavar=("REQUEST", "RESULT"))
    parser.add_argument("path", nargs="?")
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    if args.worker:
        return _worker(*args.worker)
    if not args.path:
        parser.error("path is required")
    result = check_file(args.path, timeout_s=args.timeout)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result["status"] == "CHECKED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
