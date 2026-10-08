"""Read back every junction_paving road with the installed esmini RoadManager.

This is a version-bound width/XY consumer check, not map, route, source-fidelity
or product acceptance. ``audit_file`` ALWAYS runs RM in a fresh child process.
No lane types, zero widths, XML bytes or failures are substituted or skipped.

Usage: python scripts/workbench_check_auxiliary_consumer.py INPUT.xodr OUTPUT.json
The output must not already exist. Missing DLL, crashes and empty scope fail.
"""
from __future__ import annotations

import argparse
import bisect
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.esmini_rm_check import DLL, RMPos, _bind
from scripts.internal_edge_jets import states

# The existing written-export readback requires position error <= 1e-5 m and
# probes width joins at +/-1e-5 m. Apply that SAME length precision to widths
# and returned longitudinal positions; there are no CLI tolerance overrides.
TOLERANCE_M = 1e-5
SIDE_EPSILON_M = 1e-5
SCHEMA = "mapforge/auxiliary-consumer-readback/v1"


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _number(element, name):
    value = float(element.attrib[name])
    if not math.isfinite(value):
        raise ValueError(f"nonfinite {element.tag}.{name}")
    return value


def _ordered_starts(elements, attr, length, *, required=True):
    values = [_number(e, attr) for e in elements]
    if not values:
        if required:
            raise ValueError(f"missing {attr} records")
        return values
    if values[0] != 0 or any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError(f"{attr} records must start at zero and increase strictly")
    if any(s < 0 or s > length for s in values):
        raise ValueError(f"{attr} record outside its domain")
    return values


def _polynomial(elements, attr, station):
    if not elements:
        return 0.0
    starts = [_number(e, attr) for e in elements]
    item = elements[max(0, bisect.bisect_right(starts, station) - 1)]
    ds = station - _number(item, attr)
    a, b, c, d = (_number(item, key) for key in "abcd")
    return a + ds * (b + ds * (c + ds * d))


def _sample_stations(events):
    """Keep exact events; sided samples never cross another written event."""
    cuts = sorted(events)
    samples = {s: {"event"} for s in cuts}
    for lo, hi in zip(cuts, cuts[1:]):
        middle = lo + (hi - lo) / 2
        left = lo + min(SIDE_EPSILON_M, (hi - lo) / 4)
        right = hi - min(SIDE_EPSILON_M, (hi - lo) / 4)
        if not (lo < left <= middle <= right < hi):
            raise ValueError("events too close for distinct floating-point sided probes")
        for s, label in ((left, "right_of_event"), (middle, "interval_midpoint"),
                         (right, "left_of_event")):
            samples.setdefault(s, set()).add(label)
    return [{"s_m": s, "sample_kinds": sorted(labels)} for s, labels in sorted(samples.items())]


def build_probes(data: bytes) -> dict:
    """Pure XML expectation construction; never loads the consumer DLL.

    Width-based flat line/arc/spiral auxiliary roads are supported. Unsupported
    representations fail explicitly rather than producing partial coverage.
    The exact knot adopts the new record/section; its left limit is probed on
    the left. Road endpoints are queried exactly, including zero-width ends.
    """
    root = ET.fromstring(data)
    if root.tag != "OpenDRIVE":
        raise ValueError("input is not OpenDRIVE")
    all_roads = root.findall("road")
    ids = [r.get("id") for r in all_roads]
    if None in ids or len(ids) != len(set(ids)):
        raise ValueError("missing or duplicate road IDs")
    numeric_ids = [int(rid) for rid in ids]
    if len(numeric_ids) != len(set(numeric_ids)):
        raise ValueError("road IDs collide after consumer integer conversion")
    roads = [r for r in all_roads if r.get("name") == "junction_paving"]
    if not roads:
        raise ValueError("NO_AUXILIARY_ROADS: no junction_paving road")
    result = {"roads": [], "probes": []}
    for road in roads:
        rid = int(road.get("id"))
        if not 0 <= rid < 2**32:
            raise ValueError("auxiliary road ID outside consumer uint32 range")
        length = _number(road, "length")
        if length <= 0:
            raise ValueError(f"road {rid}: nonpositive length")
        # states() computes flat XY. A banked/shape surface requires a separate
        # expected-position implementation, never an implicit flat fallback.
        if any(_number(e, k) != 0 for e in road.findall("lateralProfile/*")
               for k in "abcd" if k in e.attrib):
            raise ValueError(f"road {rid}: unsupported nonflat lateralProfile")
        events = {0.0: {"road_start"}, length: {"road_end"}}

        def event(s, kind):
            if not 0 <= s <= length:
                raise ValueError(f"road {rid}: event outside road")
            events.setdefault(s, set()).add(kind)

        geometries = road.findall("planView/geometry")
        _ordered_starts(geometries, "s", length)
        for geometry in geometries:
            for key in ("x", "y", "hdg"):
                _number(geometry, key)
            if _number(geometry, "length") <= 0:
                raise ValueError("nonpositive geometry length")
            children = list(geometry)
            if len(children) != 1 or children[0].tag not in ("line", "arc", "spiral"):
                raise ValueError("unsupported reference primitive")
            for key in children[0].attrib:
                _number(children[0], key)
            event(_number(geometry, "s"), "geometry")
        offsets = road.findall("lanes/laneOffset")
        _ordered_starts(offsets, "s", length, required=False)
        for offset in offsets:
            for key in "abcd":
                _number(offset, key)
            event(_number(offset, "s"), "laneOffset")
        sections = road.findall("lanes/laneSection")
        section_starts = _ordered_starts(sections, "s", length)
        if section_starts[-1] >= length:
            raise ValueError("zero-length final laneSection")
        section_lanes = []
        for i, section in enumerate(sections):
            lo = section_starts[i]
            hi = section_starts[i + 1] if i + 1 < len(sections) else length
            event(lo, "laneSection")
            lanes = section.findall("left/lane") + section.findall("right/lane")
            lane_ids = [int(lane.get("id")) for lane in lanes]
            if not lanes or 0 in lane_ids or len(lane_ids) != len(set(lane_ids)):
                raise ValueError("auxiliary section has missing/duplicate/zero side lanes")
            if any(not -(2**31) <= lid < 2**31 for lid in lane_ids):
                raise ValueError("lane ID outside consumer int32 range")
            for side, sign in (("left", 1), ("right", -1)):
                if any(int(lane.get("id")) * sign <= 0 for lane in section.findall(side + "/lane")):
                    raise ValueError("lane ID sign disagrees with side")
            section_lanes.append(lanes)
            for lane in lanes:
                if lane.findall("border"):
                    raise ValueError("unsupported border-based auxiliary lane")
                widths = lane.findall("width")
                _ordered_starts(widths, "sOffset", hi - lo)
                for width in widths:
                    for key in "abcd":
                        _number(width, key)
                    event(lo + _number(width, "sOffset"), "width")
        samples = _sample_stations(events)
        result["roads"].append({"road_id": rid, "length_m": length,
                                "events": [{"s_m": s, "kinds": sorted(kinds)}
                                           for s, kinds in sorted(events.items())],
                                "station_count": len(samples)})
        for sample in samples:
            s = sample["s_m"]
            si = bisect.bisect_right(section_starts, s) - 1
            for lane in section_lanes[si]:
                lid = int(lane.get("id"))
                width = _polynomial(lane.findall("width"), "sOffset", s - section_starts[si])
                if not math.isfinite(width) or width < 0:
                    raise ValueError(f"road {rid} lane {lid} at {s}: negative/nonfinite XML width {width}")
                inner, outer = states(road, lid, s, False)
                expected = {"inner": list(inner[:2]), "outer": list(outer[:2]),
                            "center": [(a + b) / 2 for a, b in zip(inner[:2], outer[:2])]}
                if not all(math.isfinite(v) for xy in expected.values() for v in xy):
                    raise ValueError("nonfinite XML expected position")
                result["probes"].append({"road_id": rid, "lane_id": lid,
                    "lane_type": lane.get("type"), **sample, "expected_width_m": width,
                    "expected_xy": expected, "road_endpoint": s in (0.0, length),
                    "zero_width": width == 0.0})
    return result


def _binding(path, dll):
    code_paths = [Path(__file__).resolve(), ROOT / "scripts/esmini_rm_check.py",
                  ROOT / "scripts/internal_edge_jets.py", ROOT / "mapforge/validate/g11.py",
                  ROOT / "mapforge/validate/smoothness.py", ROOT / "scripts/validate_repair_export.py"]
    files = [{"path": str(p), "sha256": _sha(p.read_bytes())} for p in code_paths]
    return {"xodr": {"path": str(path), "sha256": _sha(path.read_bytes())},
            "consumer": {"path": str(dll), "sha256": _sha(dll.read_bytes()) if dll.is_file() else None},
            "implementation": files}


def _drift(binding):
    problems = []
    for item in [binding["xodr"], binding["consumer"], *binding["implementation"]]:
        try:
            actual = _sha(Path(item["path"]).read_bytes())
        except OSError:
            actual = None
        if actual != item["sha256"]:
            problems.append({**item, "actual_sha256": actual})
    return problems


def _base(binding):
    dll = Path(binding["consumer"]["path"])
    version_file = dll.parent.parent / "version.txt"
    version = ({"path": str(version_file), "sha256": _sha(version_file.read_bytes()),
                "text": version_file.read_text(encoding="utf8")} if version_file.is_file() else None)
    return {"schema": SCHEMA, "status": "NOT_RUN", "map_accepted": False,
            "scope": "all junction_paving noncenter lanes: sampled XML width and inner/center/outer XY readback only",
            "limitations": "Only the exact consumer DLL SHA256 and input bytes tested; not other esmini versions, routing, dynamics, continuous geometry proof or product acceptance.",
            "binding": binding, "consumer_version_file": version,
            "binding_api": "scripts/esmini_rm_check.py RMPos/_bind (esmini v3.6.0 signatures)",
            "position_tolerance_m": TOLERANCE_M, "width_tolerance_m": TOLERANCE_M,
            "station_tolerance_m": TOLERANCE_M, "side_epsilon_max_m": SIDE_EPSILON_M,
            "tolerance_basis": "scripts/validate_repair_export.py: position <= 1e-5 m and sided knots +/-1e-5 m; same length precision for width and returned s",
            "skipped": [], "rows": [], "failures": [], "zero_width_endpoint_rows": [],
            "zero_width_endpoint_support": "NOT_TESTED"}


def _finite(value):
    return float(value) if math.isfinite(value) else None


def _check_rm(rm, handle, probes):
    """The injectable call loop is tested without loading or claiming a DLL."""
    rows = []
    for probe in probes:
        rid, lid, s = probe["road_id"], probe["lane_id"], probe["s_m"]
        width = ctypes.c_double(float("nan"))
        wc = rm.RM_GetLaneWidthByRoadId(rid, lid, s, ctypes.byref(width))
        for edge, direction in (("inner", -1), ("center", 0), ("outer", 1)):
            # Width failure stays a failure; do not silently use XML width.
            # RM is still queried with its returned offset so other failures
            # remain visible. NaN is not sent to native code.
            offset = width.value / 2 * (1 if lid > 0 else -1) * direction
            if not math.isfinite(offset):
                rows.append({**probe, "edge": edge, "codes": {"width": wc, "position": None, "read": None},
                             "actual_width_m": _finite(width.value), "ok": False,
                             "failure_reasons": ["nonfinite consumer width; unsafe native position call withheld"]})
                continue
            rc = rm.RM_SetLanePosition(handle, rid, lid, offset, s, True)
            pos = RMPos()
            pc = rm.RM_GetPositionData(handle, ctypes.byref(pos))
            error = math.hypot(pos.x - probe["expected_xy"][edge][0], pos.y - probe["expected_xy"][edge][1])
            width_error = abs(width.value - probe["expected_width_m"])
            s_error = abs(pos.s - s)
            reasons = []
            if any(code < 0 for code in (wc, rc, pc)):
                reasons.append("consumer return code indicates failure")
            if pos.roadId != rid or pos.laneId != lid:
                reasons.append("consumer changed road/lane identity")
            for label, value in (("position", error), ("width", width_error), ("station", s_error)):
                if not math.isfinite(value) or value > TOLERANCE_M:
                    reasons.append(label + " error exceeds fixed tolerance or is nonfinite")
            if width.value < 0:
                reasons.append("negative consumer width")
            rows.append({**probe, "edge": edge, "codes": {"width": wc, "position": rc, "read": pc},
                         "actual_width_m": _finite(width.value), "width_error_m": _finite(width_error),
                         "position_error_m": _finite(error), "station_error_m": _finite(s_error),
                         "actual_xy": [_finite(pos.x), _finite(pos.y)], "actual_s_m": _finite(pos.s),
                         "actual_road_id": pos.roadId, "actual_lane_id": pos.laneId,
                         "offset_m": offset, "ok": not reasons, "failure_reasons": reasons})
    return rows


def _worker(binding):
    result = _base(binding)
    result["worker_pid"] = os.getpid()
    drift = _drift(binding)
    if drift:
        return {**result, "status": "BINDING_CHANGED", "binding_drift": drift}
    plan = build_probes(Path(binding["xodr"]["path"]).read_bytes())
    result["roads"] = plan["roads"]
    result["planned_width_probes"] = len(plan["probes"])
    result["planned_position_probes"] = 3 * len(plan["probes"])
    rm = ctypes.CDLL(binding["consumer"]["path"])
    _bind(rm)
    try:
        result["load_code"] = rm.RM_Init(binding["xodr"]["path"].encode("utf8"))
        if result["load_code"] != 0:
            result["status"] = "LOAD_FAILED"
            return result
        handle = rm.RM_CreatePosition()
        result["position_handle"] = handle
        if handle < 0:
            result["status"] = "CREATE_POSITION_FAILED"
            return result
        rows = _check_rm(rm, handle, plan["probes"])
        failures = [row for row in rows if not row["ok"]]
        zeros = [row for row in rows if row["zero_width"] and row["road_endpoint"]]
        result.update(status="FAIL" if failures else "PASS", rows=rows, failures=failures,
                      checked_positions=sum(row["codes"]["position"] is not None for row in rows),
                      checked_widths=len(plan["probes"]), zero_width_endpoint_rows=zeros,
                      zero_width_endpoint_support=("FAIL" if any(not r["ok"] for r in zeros) else
                                                   "PASS" if zeros else "NOT_PRESENT"))
        for field in ("width_error_m", "position_error_m", "station_error_m"):
            result["max_" + field] = max((r[field] for r in rows if r.get(field) is not None), default=None)
    finally:
        rm.RM_Close()
        drift = _drift(binding)
        if drift:
            result.update(status="BINDING_CHANGED", binding_drift=drift)
    return result


def audit_file(path: Path, *, dll: Path = DLL, timeout_s: float = 120) -> dict:
    """Public API: isolated RM process even when called from a live server."""
    path, dll = Path(path).resolve(), Path(dll).resolve()
    binding = _binding(path, dll)
    result = _base(binding)
    if not dll.is_file():
        return {**result, "status": "UNAVAILABLE", "error": "esmini RoadManager DLL is missing"}
    with tempfile.TemporaryDirectory(prefix="mapforge-aux-consumer-") as folder:
        temp = Path(folder)
        request, output = temp / "request.json", temp / "result.json"
        request.write_text(json.dumps(binding), encoding="utf8")
        command = [sys.executable, str(Path(__file__).resolve()), "--worker", str(request), str(output)]
        try:
            process = subprocess.run(command, cwd=temp, capture_output=True, timeout=timeout_s)
            if output.is_file():
                try:
                    decoded = json.loads(output.read_text(encoding="utf8"))
                    if not isinstance(decoded, dict) or decoded.get("schema") != SCHEMA:
                        raise ValueError("worker report has invalid schema")
                    result = decoded
                except (UnicodeError, ValueError) as exc:
                    result.update(status="WORKER_FAILED", error=f"invalid worker report: {exc}")
            else:
                result.update(status="WORKER_FAILED", error="worker produced no report")
            result["process"] = {"exit_code": process.returncode,
                "stdout": process.stdout.decode("utf8", errors="replace"),
                "stderr": process.stderr.decode("utf8", errors="replace")}
            if process.returncode != 0 and result["status"] == "PASS":
                result.update(status="WORKER_FAILED", error="worker exit code contradicts PASS")
        except subprocess.TimeoutExpired:
            result.update(status="TIMEOUT", error=f"consumer worker exceeded {timeout_s} seconds")
        except OSError as exc:
            result.update(status="WORKER_FAILED", error=f"cannot run consumer worker: {exc}")
    drift = _drift(binding)
    if drift:
        result.update(status="BINDING_CHANGED", binding_drift=drift)
    return result


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--worker":
        binding = json.loads(Path(sys.argv[2]).read_text(encoding="utf8"))
        try:
            result = _worker(binding)
        except Exception as exc:
            result = {**_base(binding), "status": "ERROR", "worker_pid": os.getpid(),
                      "error": f"{type(exc).__name__}: {exc}"}
        Path(sys.argv[3]).write_text(json.dumps(result, ensure_ascii=False, allow_nan=False), encoding="utf8")
        return 0 if result["status"] == "PASS" else 1
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xodr", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--dll", type=Path, default=DLL)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; preserve evidence and choose a new path")
    result = audit_file(args.xodr, dll=args.dll, timeout_s=args.timeout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf8", newline="\n") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"status": result["status"], "output": str(args.output.resolve()),
                      "checked_positions": result.get("checked_positions", 0),
                      "max_width_error_m": result.get("max_width_error_m"),
                      "max_position_error_m": result.get("max_position_error_m"),
                      "zero_width_endpoint_support": result["zero_width_endpoint_support"]}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
