"""Synthetic inputs/fake DLLs exercise transport; no production maps are run."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import workbench_esmini_portable as P


XML = b'<?xml version="1.0" encoding="UTF-8"?><OpenDRIVE><header name="test"/></OpenDRIVE>'


@pytest.mark.parametrize("data", [
    b"", b"\x00<OpenDRIVE/>", b"<OpenDRIVE>\xff</OpenDRIVE>", b"<root/>",
    b"<OpenDRIVE", b'<?xml version="1.0" encoding="ISO-8859-1"?><OpenDRIVE/>',
    b'<!DOCTYPE OpenDRIVE SYSTEM "file:///secret"><OpenDRIVE/>',
    b'<!DOCTYPE OpenDRIVE [<!ENTITY x "secret">]><OpenDRIVE>&x;</OpenDRIVE>',
    b'<?xml-stylesheet href="style.xsl"?><OpenDRIVE/>',
    b'<OpenDRIVE><include file="fragment.xodr"/></OpenDRIVE>',
    b'<OpenDRIVE xmlns:x="http://www.w3.org/2001/XInclude"><x:include href="a.xml"/></OpenDRIVE>',
    b'<OpenDRIVE><surface><CRG file="road.crg"/></surface></OpenDRIVE>',
    b'<OpenDRIVE><texture file="road.png"/></OpenDRIVE>',
    b'<OpenDRIVE><x href="a"/></OpenDRIVE>', b'<OpenDRIVE><x url="a"/></OpenDRIVE>',
    b'<OpenDRIVE><x filename="a"/></OpenDRIVE>', b'<OpenDRIVE><x uri="a"/></OpenDRIVE>',
    b'<OpenDRIVE xml:base="folder/"></OpenDRIVE>',
    b'<OpenDRIVE xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="a b"/>',
])
def test_unsafe_or_non_self_contained_input_is_rejected(data):
    with pytest.raises(P.UnsupportedInput):
        P.validate_input(data)


def test_utf8_bom_and_original_bytes_are_preserved():
    data = b"\xef\xbb\xbf" + XML.replace(b"test", "中文".encode())
    before = bytes(data)
    proof = P.validate_input(data)
    assert proof["sha256"] == hashlib.sha256(before).hexdigest()
    assert proof["size"] == len(before)
    assert data == before


class Function:
    def __init__(self, result=0):
        self.result = result
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


def fake_rm():
    names = ("RM_Init", "RM_InitWithString", "RM_SetLogFilePath", "RM_SetLanePosition",
             "RM_PositionMoveForward", "RM_GetPositionData", "RM_GetIdOfRoadFromIndex",
             "RM_GetRoadLength", "RM_GetRoadNumberOfDrivableLanes", "RM_GetDrivableLaneIdByIndex",
             "RM_GetLaneWidthByRoadId", "RM_CreatePosition", "RM_GetNumberOfRoads", "RM_Close")
    return SimpleNamespace(**{name: Function() for name in names})


def test_frozen_old_function_calls_bytes_loader_and_keeps_original_path(tmp_path):
    path = tmp_path / "中文 空格.xodr"
    path.write_bytes(XML)
    checker_bytes = P.CHECKER.read_bytes()
    checker = P._load_checker(P.CHECKER)
    rm = fake_rm()
    rm.RM_Init.result = AssertionError("path loader must never run")
    result = P._run_old_check(rm, checker, XML, path)
    assert result["pass"] is False  # The real old checker found zero drivable routes.
    assert result["line"].startswith(path.name + ":")
    assert result["line"].endswith("CHECK")
    assert result["gap_max_cm"] == 0
    assert result["full_check_executed"] is True
    assert rm.RM_Init.calls == []
    assert rm.RM_InitWithString.calls == [(XML,)]
    assert rm.RM_SetLogFilePath.calls == [(b"",)]
    assert P.CHECKER.read_bytes() == checker_bytes
    assert path.read_bytes() == XML


def test_proxy_forwards_every_other_api_without_mutating_instance(tmp_path):
    path = tmp_path / "source.xodr"
    rm = fake_rm()
    proxy = P._BytesInitProxy(rm, XML, path)
    assert proxy.RM_Close is rm.RM_Close
    assert proxy.RM_GetRoadLength is rm.RM_GetRoadLength
    proxy.RM_GetRoadLength(123)
    assert rm.RM_GetRoadLength.calls == [(123,)]
    with pytest.raises(ValueError, match="different path"):
        proxy.RM_Init(b"other.xodr")
    proxy.RM_Init(str(path).encode())
    with pytest.raises(ValueError, match="repeated"):
        proxy.RM_Init(str(path).encode())


def test_successful_loading_without_complete_checker_output_cannot_pass(tmp_path):
    path = tmp_path / "source.xodr"
    rm = fake_rm()
    def load_only(proxy, original):
        assert original == str(path)
        assert proxy.RM_Init(original.encode()) == 0
        return True
    checker = SimpleNamespace(_bind=lambda _: None, check=load_only)
    with pytest.raises(ValueError, match="summary"):
        P._run_old_check(rm, checker, XML, path)


def test_bad_summary_after_normal_checker_return_does_not_close_twice(tmp_path):
    path = tmp_path / "source.xodr"
    rm = fake_rm()
    def finished_check(proxy, original):
        proxy.RM_Init(original.encode())
        proxy.RM_Close()
        print("invalid summary")
        return True
    checker = SimpleNamespace(_bind=lambda _: None, check=finished_check)
    with pytest.raises(ValueError, match="summary"):
        P._run_old_check(rm, checker, XML, path)
    assert rm.RM_Close.calls == [()]


def test_exception_before_checker_return_closes_initialized_rm_once(tmp_path):
    path = tmp_path / "source.xodr"
    rm = fake_rm()
    def broken_check(proxy, original):
        proxy.RM_Init(original.encode())
        raise RuntimeError("interrupted old check")
    checker = SimpleNamespace(_bind=lambda _: None, check=broken_check)
    with pytest.raises(RuntimeError, match="interrupted"):
        P._run_old_check(rm, checker, XML, path)
    assert rm.RM_Close.calls == [()]


def test_worker_exception_is_structured_and_nonpass(tmp_path, monkeypatch):
    path = tmp_path / "source.xodr"
    path.write_bytes(XML)
    before = {name: P._binding(p) for name, p in
              (("input", path), ("dll", P.DLL), ("checker", P.CHECKER), ("adapter", P.ADAPTER))}
    request, output = tmp_path / "request.json", tmp_path / "result.json"
    request.write_text(json.dumps({"before": before}), encoding="utf8")
    checker = SimpleNamespace(_bind=lambda _: None,
                              check=lambda *args: (_ for _ in ()).throw(RuntimeError("controlled check failure")))
    monkeypatch.setattr(P, "_load_checker", lambda _: checker)
    monkeypatch.setattr(P.ctypes, "CDLL", lambda _: fake_rm())
    assert P._worker(request, output) == 1
    result = json.loads(output.read_text(encoding="utf8"))
    assert result["status"] == "FAILED" and result["pass"] is False
    assert result["full_check_executed"] is False
    assert "controlled check failure" in result["error"]


def controlled_worker(tmp_path, monkeypatch, ending=""):
    """A fake child returns protocol fixtures; it never loads a native library."""
    adapter = tmp_path / "controlled_adapter.py"
    adapter.write_text(
        "import json,sys,time\nfrom pathlib import Path\n"
        "request=Path(sys.argv[2]); output=Path(sys.argv[3])\n"
        "before=json.loads(request.read_text(encoding='utf8'))['before']\n"
        "name=Path(before['input']['path']).name\n"
        "line=name+': 换乘缝隙 max 进侧 1.0cm / 出侧 2.0cm（2 对）；行驶穿越 1 次，跳变 0，借道弃计 0，灭车道并线 0  -> PASS'\n"
        "result={'status':'CHECKED','pass':True,'line':line,'gap_max_cm':2.0,"
        "'check_stdout':line+'\\n','full_check_executed':True,'before':before,'after':before,"
        "'loader_calls':1,'loaded_bytes_sha256':before['input']['sha256']}\n"
        + ending +
        "output.write_text(json.dumps(result,ensure_ascii=False),encoding='utf8')\n",
        encoding="utf8",
    )
    monkeypatch.setattr(P, "ADAPTER", adapter)
    path = tmp_path / "中文 输入.xodr"
    path.write_bytes(XML)
    return path


def test_owned_process_protocol_retains_complete_metrics_and_bindings(tmp_path, monkeypatch):
    path = controlled_worker(tmp_path, monkeypatch)
    result = P.check_file(path)
    assert result["status"] == "CHECKED", result
    assert result["pass"] is True and result["gap_max_cm"] == 2.0
    evidence = result["evidence"]
    assert evidence["before"] == evidence["after"]
    assert set(evidence["before"]) == {"input", "dll", "checker", "adapter"}
    assert evidence["unchanged"] and evidence["exact_bytes"] and evidence["full_check_executed"]
    assert evidence["worker_exit_code"] == 0
    assert evidence["loader"] == "RM_InitWithString"


@pytest.mark.parametrize("ending", [
    "result['check_stdout']=''\n",
    "result['pass']=False\n",
    "result['gap_max_cm']=0.0\n",
    "result['loader_calls']=0\n",
    "result['loaded_bytes_sha256']='0'*64\n",
    "result['full_check_executed']=False\n",
    "result['after']={}\n",
    "sys.exit(7)\n",
    "output.write_text('{broken',encoding='utf8');sys.exit(0)\n",
])
def test_false_or_incomplete_worker_success_fails_closed(tmp_path, monkeypatch, ending):
    path = controlled_worker(tmp_path, monkeypatch, ending)
    result = P.check_file(path)
    assert result["status"] == "FAILED"
    assert result["pass"] is False and result["gap_max_cm"] is None
    assert result["evidence"]["full_check_executed"] is False


def test_input_drift_after_complete_fake_check_is_nonpass(tmp_path, monkeypatch):
    path = controlled_worker(tmp_path, monkeypatch,
                             "Path(before['input']['path']).write_bytes(b'<OpenDRIVE/>')\n")
    result = P.check_file(path)
    assert result["status"] == "FAILED" and result["pass"] is False
    assert result["gap_max_cm"] is None
    assert result["evidence"]["unchanged"] is False


@pytest.mark.parametrize("name", ["dll", "checker", "adapter"])
def test_dependency_drift_after_complete_fake_check_is_nonpass(tmp_path, monkeypatch, name):
    if name in {"dll", "checker"}:
        source = getattr(P, name.upper())
        copied = tmp_path / ("copied-" + source.name)
        copied.write_bytes(source.read_bytes())
        monkeypatch.setattr(P, name.upper(), copied)
    path = controlled_worker(
        tmp_path, monkeypatch,
        "changed=Path(before[" + repr(name) + "]['path'])\n"
        "changed.write_bytes(changed.read_bytes()+b'controlled drift')\n",
    )
    result = P.check_file(path)
    assert result["status"] == "FAILED" and result["pass"] is False
    assert result["gap_max_cm"] is None
    assert result["evidence"]["unchanged"] is False
    assert result["evidence"]["before"][name] != result["evidence"]["after"][name]


def test_complete_check_failure_keeps_actual_gap_and_zero_worker_exit(tmp_path, monkeypatch):
    path = controlled_worker(
        tmp_path, monkeypatch,
        "result['pass']=False\n"
        "result['line']=line.removesuffix('PASS')+'CHECK'\n"
        "result['check_stdout']=result['line']+'\\n'\n",
    )
    result = P.check_file(path)
    assert result["status"] == "CHECKED"
    assert result["pass"] is False and result["gap_max_cm"] == 2.0
    assert result["line"].endswith("CHECK")
    assert result["evidence"]["full_check_executed"] is True
    assert result["evidence"]["worker_exit_code"] == 0


def test_missing_dll_and_changed_checker_do_not_start_child(tmp_path, monkeypatch):
    path = tmp_path / "input.xodr"
    path.write_bytes(XML)
    original_dll = P.DLL
    monkeypatch.setattr(P, "DLL", tmp_path / "missing.dll")
    result = P.check_file(path)
    assert result["status"] == "FAILED" and result["pass"] is False
    assert result["evidence"]["worker_exit_code"] is None
    monkeypatch.setattr(P, "DLL", original_dll)
    checker = tmp_path / "changed_checker.py"
    checker.write_text("raise RuntimeError('must not import changed checker')", encoding="utf8")
    monkeypatch.setattr(P, "CHECKER", checker)
    result = P.check_file(path)
    assert result["status"] == "FAILED" and result["pass"] is False
    assert "frozen original" in result["error"]
    assert result["evidence"]["worker_exit_code"] is None


def test_rejected_input_does_not_load_any_native_code(tmp_path, monkeypatch):
    path = tmp_path / "external.xodr"
    path.write_bytes(b'<OpenDRIVE><include file="other.xodr"/></OpenDRIVE>')
    result = P.check_file(path)
    assert result["status"] == "REJECTED_INPUT"
    assert result["pass"] is False and result["gap_max_cm"] is None
    assert result["evidence"]["worker_exit_code"] is None


def test_timeout_kills_and_waits_for_its_owned_fake_worker(tmp_path, monkeypatch):
    path = controlled_worker(tmp_path, monkeypatch,
                             "print('controlled worker started',flush=True);time.sleep(30)\n")
    result = P.check_file(path, timeout_s=.5)
    assert result["status"] == "TIMED_OUT"
    assert result["pass"] is False and result["gap_max_cm"] is None
    assert result["evidence"]["worker_exit_code"] is not None
    assert result["evidence"]["worker_exit_code"] != 0
    assert "controlled worker started" in result["evidence"]["stdout"]
