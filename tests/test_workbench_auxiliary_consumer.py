"""Pure XML and consumer-call contracts; these do not establish DLL support."""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

from scripts import workbench_check_auxiliary_consumer as check


def xml(*, heading=0, lane_id=-1, lane_type="restricted", width=2, offsets=None,
        widths=None, length=10):
    root = ET.Element("OpenDRIVE")
    road = ET.SubElement(root, "road", id="50", name="junction_paving", length=str(length), junction="-1")
    pv = ET.SubElement(road, "planView")
    g = ET.SubElement(pv, "geometry", s="0", x="10", y="20", hdg=str(heading), length=str(length))
    ET.SubElement(g, "line")
    lanes = ET.SubElement(road, "lanes")
    for s, a, b in offsets or [(0, 1, 0)]:
        ET.SubElement(lanes, "laneOffset", s=str(s), a=str(a), b=str(b), c="0", d="0")
    section = ET.SubElement(lanes, "laneSection", s="0")
    center = ET.SubElement(section, "center")
    ET.SubElement(center, "lane", id="0", type="none")
    side = ET.SubElement(section, "left" if lane_id > 0 else "right")
    lane = ET.SubElement(side, "lane", id=str(lane_id), type=lane_type)
    for s, a, b in widths or [(0, width, 0)]:
        ET.SubElement(lane, "width", sOffset=str(s), a=str(a), b=str(b), c="0", d="0")
    return ET.tostring(root)


def edited(data, fn):
    root = ET.fromstring(data)
    fn(root)
    return ET.tostring(root)


def station(plan, s, lane_id=-1):
    return next(p for p in plan["probes"] if p["s_m"] == s and p["lane_id"] == lane_id)


def test_all_written_offset_width_events_sides_and_midpoints():
    data = xml(offsets=[(0, 1, 0), (3, 1, 0.25)],
               widths=[(0, 0, 1), (2, 2, 0), (7, 2, -2 / 3)])
    plan = check.build_probes(data)
    assert [e["s_m"] for e in plan["roads"][0]["events"]] == [0, 2, 3, 7, 10]
    positions = {p["s_m"] for p in plan["probes"]}
    for lo, hi in zip([0, 2, 3, 7], [2, 3, 7, 10]):
        assert {lo, lo + 1e-5, (lo + hi) / 2, hi - 1e-5, hi} <= positions
    assert station(plan, 0)["zero_width"]
    assert station(plan, 0)["road_endpoint"]
    assert station(plan, 10)["zero_width"]
    assert station(plan, 10)["expected_width_m"] == 0
    assert station(plan, 3)["expected_xy"]["inner"] == [13, 21]
    assert station(plan, 7)["expected_xy"]["inner"] == [17, 22]
    assert all(p["lane_type"] == "restricted" for p in plan["probes"])


@pytest.mark.parametrize("lane_id,lane_type", [(-1, "restricted"), (1, "median")])
def test_world_width_and_edge_positions_preserve_side_and_lane_type(lane_id, lane_type):
    plan = check.build_probes(xml(heading=math.pi / 2, lane_id=lane_id, lane_type=lane_type))
    p = station(plan, 10, lane_id)
    assert p["expected_width_m"] == 2
    assert p["expected_xy"]["inner"] == pytest.approx([9, 30])
    assert p["expected_xy"]["outer"] == pytest.approx([9 - lane_id * 2, 30])
    assert p["expected_xy"]["center"] == pytest.approx([9 - lane_id, 30])
    assert p["lane_type"] == lane_type


def test_adjacent_lane_width_affects_outer_lane_world_position():
    def edit(root):
        side = root.find("road/lanes/laneSection/right")
        second = ET.SubElement(side, "lane", id="-2", type="median")
        ET.SubElement(second, "width", sOffset="0", a="3", b="0", c="0", d="0")
    plan = check.build_probes(edited(xml(), edit))
    p = station(plan, 0, -2)
    assert p["expected_xy"] == {"inner": [10, 19], "center": [10, 17.5], "outer": [10, 16]}


def test_close_events_not_rounded_merged_or_crossed():
    hi = 2 + 1e-7
    plan = check.build_probes(xml(widths=[(0, 1, 0), (2, 1, 0), (hi, 1, 0)]))
    values = [p["s_m"] for p in plan["probes"]]
    assert 2 in values and hi in values
    between = [s for s in values if 2 < s < hi]
    assert len(between) == 3
    assert between == sorted(set(between))


def test_section_events_use_new_lane_at_exact_join_old_lane_on_left():
    def edit(root):
        lanes = root.find("road/lanes")
        new = ET.fromstring(ET.tostring(lanes.find("laneSection")))
        new.set("s", "4")
        new.find("right/lane/width").set("a", "3")
        lanes.append(new)
    plan = check.build_probes(edited(xml(), edit))
    assert station(plan, 4)["expected_width_m"] == 3
    assert station(plan, 4 - 1e-5)["expected_width_m"] == 2
    assert station(plan, 4 + 1e-5)["expected_width_m"] == 3
    assert "laneSection" in next(e["kinds"] for e in plan["roads"][0]["events"] if e["s_m"] == 4)


def test_geometry_join_is_a_natural_event():
    def edit(root):
        pv = root.find("road/planView")
        pv[0].set("length", "4")
        g = ET.SubElement(pv, "geometry", s="4", x="14", y="20", hdg="0", length="6")
        ET.SubElement(g, "arc", curvature="0.1")
    plan = check.build_probes(edited(xml(), edit))
    assert station(plan, 4)["expected_xy"]["inner"] == pytest.approx([14, 21])
    assert station(plan, 10)["expected_xy"]["inner"] == pytest.approx(
        [14 + 9 * math.sin(0.6), 20 + 10 - 9 * math.cos(0.6)])


@pytest.mark.parametrize("change,match", [
    (lambda r: r.find("road").set("name", "ordinary"), "NO_AUXILIARY_ROADS"),
    (lambda r: r.find("road").set("id", str(2**32)), "uint32"),
    (lambda r: r.find("road").set("length", "0"), "nonpositive length"),
    (lambda r: r.find("road/planView/geometry").set("x", "nan"), "nonfinite"),
    (lambda r: r.find("road/lanes/laneSection/right/lane/width").set("a", "-1"), "negative"),
    (lambda r: r.find("road/lanes/laneSection/right/lane/width").set("sOffset", "1"), "start at zero"),
    (lambda r: r.find("road/lanes/laneSection/right/lane/width").__setattr__("tag", "border"), "border-based"),
    (lambda r: r.find("road/planView/geometry/line").__setattr__("tag", "poly3"), "unsupported reference"),
    (lambda r: r.append(ET.fromstring(ET.tostring(r[0]))), "duplicate road"),
])
def test_unsupported_or_invalid_input_is_not_partial_pass(change, match):
    with pytest.raises(ValueError, match=match):
        check.build_probes(edited(xml(), change))


class FakeRM:
    """Controlled return values for the call-loop contract, never real esmini."""
    def __init__(self, probes, *, width_delta=0, remap_zero=False, width_code=0):
        self.probes = {(p["road_id"], p["lane_id"], p["s_m"]): p for p in probes}
        self.calls = []
        self.width_delta = width_delta
        self.remap_zero = remap_zero
        self.width_code = width_code

    def RM_GetLaneWidthByRoadId(self, rid, lid, s, output):
        p = self.probes[(rid, lid, s)]
        output._obj.value = p["expected_width_m"] + self.width_delta
        self.calls.append(("width", rid, lid, s))
        return self.width_code

    def RM_SetLanePosition(self, handle, rid, lid, offset, s, align):
        self.current = (rid, lid, offset, s)
        self.calls.append(("position", rid, lid, s, offset))
        return 0

    def RM_GetPositionData(self, handle, output):
        rid, lid, offset, s = self.current
        p = self.probes[(rid, lid, s)]
        output._obj.x, output._obj.y = p["expected_xy"]["center"]
        output._obj.y += offset
        output._obj.roadId = rid
        output._obj.laneId = 0 if self.remap_zero and p["zero_width"] else lid
        output._obj.s = s
        return 0


def test_zero_width_endpoints_are_actually_queried_without_substitution():
    probes = check.build_probes(xml(widths=[(0, 0, 1), (5, 5, -1)]))["probes"]
    fake = FakeRM(probes)
    rows = check._check_rm(fake, 1, probes)
    assert len(rows) == 3 * len(probes)
    assert all(r["ok"] for r in rows)
    zeros = [r for r in rows if r["zero_width"] and r["road_endpoint"]]
    assert len(zeros) == 6
    assert {r["s_m"] for r in zeros} == {0, 10}
    assert all(r["actual_width_m"] == 0 and r["offset_m"] == 0 for r in zeros)
    assert all(r["codes"] == {"width": 0, "position": 0, "read": 0} for r in zeros)


def test_consumer_zero_width_remap_is_failure_not_skip():
    probes = check.build_probes(xml(widths=[(0, 0, 1)]))["probes"]
    rows = check._check_rm(FakeRM(probes, remap_zero=True), 1, probes)
    zeros = [r for r in rows if r["zero_width"]]
    assert len(zeros) == 3
    assert all(not r["ok"] and r["actual_lane_id"] == 0 for r in zeros)
    assert all("consumer changed road/lane identity" in r["failure_reasons"] for r in zeros)


def test_failed_width_return_remains_failure_and_positions_are_still_checked():
    probes = check.build_probes(xml())["probes"]
    fake = FakeRM(probes, width_code=-1)
    rows = check._check_rm(fake, 1, probes)
    assert len(rows) == 3 * len(probes)
    assert all(not r["ok"] and r["codes"]["width"] == -1 for r in rows)
    assert len([c for c in fake.calls if c[0] == "position"]) == len(rows)


def test_nonfinite_consumer_width_fails_without_sending_nan_to_native_call():
    probes = check.build_probes(xml())["probes"]
    fake = FakeRM(probes, width_delta=float("nan"))
    rows = check._check_rm(fake, 1, probes)
    assert len(rows) == 3 * len(probes)
    assert all(not r["ok"] and r["codes"]["position"] is None for r in rows)
    assert not any(c[0] == "position" for c in fake.calls)
    json.dumps(rows, allow_nan=False)


def test_width_error_uses_fixed_export_precision():
    probes = check.build_probes(xml())["probes"]
    rows = check._check_rm(FakeRM(probes, width_delta=1.1e-5), 1, probes)
    assert all(not r["ok"] for r in rows)
    assert check.TOLERANCE_M == 1e-5
    assert all(r["width_error_m"] > 1e-5 for r in rows)


def test_missing_dll_explicitly_fails_without_loading_native_code(tmp_path, monkeypatch):
    source = tmp_path / "input.xodr"
    source.write_bytes(xml())
    monkeypatch.setattr(ctypes, "CDLL", lambda *a: pytest.fail("must not load DLL in parent"))
    result = check.audit_file(source, dll=tmp_path / "missing.dll")
    assert result["status"] == "UNAVAILABLE"
    assert result["map_accepted"] is False
    assert result["binding"]["xodr"]["sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert result["binding"]["consumer"]["sha256"] is None
    assert result["zero_width_endpoint_support"] == "NOT_TESTED"


def test_public_api_uses_isolated_process_and_binds_consumer_and_input(tmp_path, monkeypatch):
    source, dll = tmp_path / "input.xodr", tmp_path / "fake.dll"
    source.write_bytes(xml())
    dll.write_bytes(b"not loaded by this process")
    monkeypatch.setattr(ctypes, "CDLL", lambda *a: pytest.fail("must not load DLL in parent"))
    def fake_run(command, *, cwd, capture_output, timeout):
        assert command[0] == sys.executable
        assert command[2] == "--worker"
        assert cwd != tmp_path and Path(cwd).is_dir()
        binding = json.loads(Path(command[3]).read_text(encoding="utf8"))
        assert binding["xodr"]["path"] == str(source.resolve())
        assert binding["consumer"]["sha256"] == hashlib.sha256(dll.read_bytes()).hexdigest()
        Path(command[4]).write_text(json.dumps({**check._base(binding), "status": "PASS"}), encoding="utf8")
        return subprocess.CompletedProcess(command, 0, b"consumer log", b"")
    monkeypatch.setattr(subprocess, "run", fake_run)
    result = check.audit_file(source, dll=dll)
    assert result["status"] == "PASS"
    assert result["process"]["stdout"] == "consumer log"
    assert result["map_accepted"] is False


@pytest.mark.parametrize("drift_target", ["source", "dll"])
def test_input_or_consumer_drift_during_process_rejects_pass(tmp_path, monkeypatch, drift_target):
    source, dll = tmp_path / "input.xodr", tmp_path / "fake.dll"
    source.write_bytes(xml())
    dll.write_bytes(b"fake")
    def fake_run(command, **kwargs):
        binding = json.loads(Path(command[3]).read_text(encoding="utf8"))
        Path(command[4]).write_text(json.dumps({**check._base(binding), "status": "PASS"}), encoding="utf8")
        (source if drift_target == "source" else dll).write_bytes(b"changed")
        return subprocess.CompletedProcess(command, 0, b"", b"")
    monkeypatch.setattr(subprocess, "run", fake_run)
    result = check.audit_file(source, dll=dll)
    assert result["status"] == "BINDING_CHANGED"
    assert len(result["binding_drift"]) == 1


def test_worker_crash_cannot_be_pass(tmp_path, monkeypatch):
    source, dll = tmp_path / "input.xodr", tmp_path / "fake.dll"
    source.write_bytes(xml())
    dll.write_bytes(b"fake")
    monkeypatch.setattr(subprocess, "run", lambda command, **kw:
                        subprocess.CompletedProcess(command, -1073741819, b"", b"native crash"))
    result = check.audit_file(source, dll=dll)
    assert result["status"] == "WORKER_FAILED"
    assert result["process"]["exit_code"] == -1073741819


@pytest.mark.parametrize("broken", ["{truncated", "[]", '{"status":"PASS"}'])
def test_partial_or_invalid_worker_report_is_explicit_failure(tmp_path, monkeypatch, broken):
    source, dll = tmp_path / "input.xodr", tmp_path / "fake.dll"
    source.write_bytes(xml())
    dll.write_bytes(b"fake")
    def fake_run(command, **kwargs):
        Path(command[4]).write_text(broken, encoding="utf8")
        return subprocess.CompletedProcess(command, -1, b"", b"")
    monkeypatch.setattr(subprocess, "run", fake_run)
    result = check.audit_file(source, dll=dll)
    assert result["status"] == "WORKER_FAILED"
    assert "invalid worker report" in result["error"]


def test_cli_missing_dll_nonzero_exit_and_does_not_overwrite_evidence(tmp_path):
    source, report, missing = tmp_path / "input.xodr", tmp_path / "report.json", tmp_path / "missing.dll"
    source.write_bytes(xml())
    command = [sys.executable, str(Path(check.__file__).resolve()), str(source), str(report), "--dll", str(missing)]
    run = subprocess.run(command, capture_output=True)
    assert run.returncode == 1
    assert json.loads(report.read_text(encoding="utf8"))["status"] == "UNAVAILABLE"
    before = report.read_bytes()
    again = subprocess.run(command, capture_output=True)
    assert again.returncode != 0
    assert report.read_bytes() == before
    assert b"output already exists" in again.stderr
