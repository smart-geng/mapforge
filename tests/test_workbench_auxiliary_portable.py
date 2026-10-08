"""Real DLL path comparisons and separate synthetic response-contract tests.

Only the two native tests establish this installed consumer's readback. Mocked
reports below exercise rejection/accounting and never claim consumer support.
"""
from __future__ import annotations

import copy
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import pytest

from scripts import workbench_auxiliary_portable as P


def tiny_xml():
    root=ET.Element("OpenDRIVE")
    ET.SubElement(root,"header",revMajor="1",revMinor="6",name="中文辅助读回")
    road=ET.SubElement(root,"road",id="50",name="junction_paving",length="10",junction="-1")
    plan=ET.SubElement(road,"planView")
    ET.SubElement(ET.SubElement(plan,"geometry",s="0",x="10",y="20",hdg="0",length="10"),"line")
    lanes=ET.SubElement(road,"lanes")
    ET.SubElement(lanes,"laneOffset",s="0",a="1",b="0",c="0",d="0")
    section=ET.SubElement(lanes,"laneSection",s="0")
    ET.SubElement(ET.SubElement(section,"center"),"lane",id="0",type="none")
    median=ET.SubElement(ET.SubElement(section,"left"),"lane",id="1",type="median")
    ET.SubElement(median,"width",sOffset="0",a="1",b="0",c="0",d="0")
    lane=ET.SubElement(ET.SubElement(section,"right"),"lane",id="-1",type="restricted")
    for s,a,b in ((0,0,1),(5,5,-1)):
        ET.SubElement(lane,"width",sOffset=str(s),a=str(a),b=str(b),c="0",d="0")
    return ET.tostring(root,encoding="utf-8",xml_declaration=True)


def write_input(tmp_path,data=None):
    path=tmp_path/"source.xodr"
    path.write_bytes(tiny_xml() if data is None else data)
    return path


NATIVE_AVAILABLE=os.name=="nt" and Path(P.legacy.DLL).is_file()


def native_pair(tmp_path,data,monkeypatch):
    paths=[tmp_path/"ascii"/"source.xodr",tmp_path/"中文 工程"/"来源 铺面.xodr"]
    for path in paths:
        path.parent.mkdir();path.write_bytes(data)
    if not str(paths[0]).isascii():
        pytest.skip("this host has no ASCII pytest temporary directory for the required comparison")
    monkeypatch.setattr(P.ctypes,"CDLL",lambda *a,**k:pytest.fail("native DLL must only load in fresh child"))
    reports=[P.audit_file(path) for path in paths]
    for path,report in zip(paths,reports):
        assert report["status"]=="PASS",report.get("error",report.get("failures"))
        assert report["worker_pid"]!=os.getpid()
        assert report["map_accepted"] is False and report["exact_bytes"] is True
        assert report["process"]["exit_code"]==0
        assert report["process"]["argv"][1:3]==["-I","-B"]
        assert report["binding"]["xodr"]=={"path":str(path.resolve()),"sha256":hashlib.sha256(data).hexdigest()}
        assert path.read_bytes()==data
        assert report["binding"]["consumer"]["sha256"]==hashlib.sha256(Path(P.legacy.DLL).read_bytes()).hexdigest()
    for field in ("rows","roads","failures","planned_width_probes","checked_widths",
                  "planned_position_probes","checked_positions","zero_width_endpoint_rows",
                  "zero_width_endpoint_support","max_width_error_m","max_position_error_m","max_station_error_m"):
        assert reports[0][field]==reports[1][field],field
    return reports[0]


@pytest.mark.skipif(not NATIVE_AVAILABLE,reason="real Windows RoadManager DLL is required")
def test_real_dll_tiny_ascii_and_chinese_readback_are_identical(tmp_path,monkeypatch):
    report=native_pair(tmp_path,tiny_xml(),monkeypatch)
    assert report["checked_widths"]==18
    assert report["checked_positions"]==54
    assert len(report["zero_width_endpoint_rows"])==6
    assert report["zero_width_endpoint_support"]=="PASS"
    assert {r["lane_type"] for r in report["rows"]}=={"median","restricted"}


@pytest.mark.skipif(not NATIVE_AVAILABLE,reason="real Windows RoadManager DLL is required")
def test_real_dll_0621_ascii_and_chinese_readback_are_identical(tmp_path,monkeypatch):
    source=P.ROOT/"out/workbench/surface-relocation-20261009/中文消费者/来源铺面.xodr"
    if not source.is_file():
        pytest.skip("the read-only existing 0621 evidence is absent on this checkout")
    data=source.read_bytes()
    report=native_pair(tmp_path,data,monkeypatch)
    assert report["checked_widths"]==621 and report["checked_positions"]==1863
    assert len(report["roads"])==13
    assert source.read_bytes()==data


def synthetic_report(binding):
    """Protocol fixture only: all actual values are invented from XML expectations."""
    data=Path(binding["xodr"]["path"]).read_bytes();plan=P.legacy.build_probes(data)
    rows=[]
    for probe in plan["probes"]:
        width=probe["expected_width_m"]
        for edge,direction in (("inner",-1),("center",0),("outer",1)):
            rows.append({**copy.deepcopy(probe),"edge":edge,"codes":{"width":0,"position":0,"read":0},
                "actual_width_m":width,"actual_xy":list(probe["expected_xy"][edge]),
                "actual_s_m":probe["s_m"],"actual_road_id":probe["road_id"],"actual_lane_id":probe["lane_id"],
                "offset_m":width/2*(1 if probe["lane_id"]>0 else -1)*direction,
                "width_error_m":0.,"position_error_m":0.,"station_error_m":0.,"ok":True,"failure_reasons":[]})
    zeros=[row for row in rows if row["zero_width"] and row["road_endpoint"]]
    return {**P.legacy._base(binding),"status":"PASS","loader":"RM_InitWithString","exact_bytes":True,
        "load_code":0,"position_handle":0,"worker_pid":os.getpid()+100000,
        "input_validation":P.transport.validate_input(data),"roads":plan["roads"],"rows":rows,
        "planned_width_probes":len(plan["probes"]),"checked_widths":len(plan["probes"]),
        "planned_position_probes":len(rows),"checked_positions":len(rows),
        "zero_width_endpoint_rows":zeros,"zero_width_endpoint_support":"PASS",
        "max_width_error_m":0.,"max_position_error_m":0.,"max_station_error_m":0.}


def controlled_response(tmp_path,monkeypatch,mutation=lambda r:None,*,exit_code=0,raw=None,drift=None):
    path=write_input(tmp_path);dll=tmp_path/"fake.dll";dll.write_bytes(b"not a consumer")
    def run(command,**kwargs):
        assert kwargs["shell"] is False and kwargs["stdin"]==subprocess.DEVNULL
        binding=json.loads(Path(command[-2]).read_text(encoding="utf8"))
        report=synthetic_report(binding);mutation(report)
        if raw is not None:
            Path(command[-1]).write_bytes(raw)
        else:
            Path(command[-1]).write_text(json.dumps(report),encoding="utf8")
        if drift:
            target=path if drift=="input" else dll;target.write_bytes(target.read_bytes()+b"changed")
        return subprocess.CompletedProcess(command,exit_code,b"synthetic response, not native evidence",b"")
    monkeypatch.setattr(P.subprocess,"run",run)
    return P.audit_file(path,dll=dll)


def test_complete_synthetic_protocol_fixture_only(tmp_path,monkeypatch):
    result=controlled_response(tmp_path,monkeypatch)
    assert result["status"]=="PASS"  # Protocol exercise, not DLL support evidence.
    assert result["process"]["stdout"].startswith("synthetic response")


def shorten_self_consistently(r):
    r["rows"]=r["rows"][:-3]
    r["planned_width_probes"]-=1;r["checked_widths"]-=1
    r["planned_position_probes"]-=3;r["checked_positions"]-=3
    r["zero_width_endpoint_rows"]=[x for x in r["rows"] if x["zero_width"] and x["road_endpoint"]]


@pytest.mark.parametrize("mutation",[
    lambda r:r.update(checked_positions=r["checked_positions"]-1),
    lambda r:r.update(checked_widths=r["checked_widths"]-1),
    lambda r:r.update(planned_width_probes=r["planned_width_probes"]-1),
    lambda r:r.update(planned_position_probes=r["planned_position_probes"]-1),
    lambda r:r.update(checked_widths=True),
    lambda r:r["rows"].pop(),shorten_self_consistently,
    lambda r:r["rows"].__setitem__(3,copy.deepcopy(r["rows"][0])),
    lambda r:r["rows"][0].update(edge="outer"),
    lambda r:r["rows"][0].update(ok=False),
    lambda r:r["rows"][0].update(ok=1),
    lambda r:r["rows"][0].update(failure_reasons=["native failure"]),
    lambda r:r["rows"][0]["codes"].update(position=None),
    lambda r:r["rows"][0]["codes"].update(read=-1),
    lambda r:r["rows"][0].update(actual_lane_id=0),
    lambda r:r["rows"][0].update(actual_xy=[999,999]),
    lambda r:r["rows"][0].update(actual_width_m=999),
    lambda r:r["rows"][0].update(actual_s_m=999),
    lambda r:r["rows"][0].update(actual_xy=[None,20]),
    lambda r:r["rows"][0].update(offset_m=999),
    lambda r:r["rows"][0].update(position_error_m=0.0001),
    lambda r:r.update(max_width_error_m=0.0001),
    lambda r:r.update(zero_width_endpoint_rows=[]),
    lambda r:r.update(zero_width_endpoint_support="NOT_PRESENT"),
    lambda r:r.update(roads=[]),lambda r:r.update(skipped=["one lane"]),
    lambda r:r.update(width_tolerance_m=1),lambda r:r.update(map_accepted=True),
    lambda r:r.update(exact_bytes=False),lambda r:r.update(load_code=1),
    lambda r:r.update(position_handle=-1),lambda r:r.update(worker_pid=os.getpid()),
    lambda r:r.update(input_validation={}),lambda r:r.update(status="UNKNOWN"),
    lambda r:r.pop("status"),lambda r:r.update(max_width_error_m=float("nan")),
    lambda r:r["binding"]["xodr"].update(sha256="0"*64),
])
def test_incomplete_or_contradictory_pass_is_fail_closed(tmp_path,monkeypatch,mutation):
    result=controlled_response(tmp_path,monkeypatch,mutation)
    assert result["status"] in {"UNAVAILABLE","BINDING_CHANGED"}
    assert result["map_accepted"] is False


@pytest.mark.parametrize("raw",[b"{truncated",b"[]",b'{"status":"PASS"}',b"\xff"])
def test_invalid_worker_json_is_fail_closed(tmp_path,monkeypatch,raw):
    result=controlled_response(tmp_path,monkeypatch,raw=raw)
    assert result["status"]=="UNAVAILABLE"


def test_process_exit_cannot_contradict_pass(tmp_path,monkeypatch):
    result=controlled_response(tmp_path,monkeypatch,exit_code=27)
    assert result["status"]=="UNAVAILABLE" and result["process"]["exit_code"]==27


@pytest.mark.parametrize("target",["input","dll"])
def test_byte_drift_overrides_complete_synthetic_pass(tmp_path,monkeypatch,target):
    result=controlled_response(tmp_path,monkeypatch,drift=target)
    assert result["status"]=="BINDING_CHANGED" and result["binding_drift"]


@pytest.mark.parametrize("data",[b"",b"<OpenDRIVE",b"\x00<OpenDRIVE/>",b"<OpenDRIVE>\xff</OpenDRIVE>",
    b'<!DOCTYPE OpenDRIVE><OpenDRIVE/>',b'<OpenDRIVE><include file="external.xodr"/></OpenDRIVE>',
    b'<OpenDRIVE/>'])
def test_rejected_input_never_starts_native_worker(tmp_path,monkeypatch,data):
    path=write_input(tmp_path,data);dll=tmp_path/"fake.dll";dll.write_bytes(b"unused")
    monkeypatch.setattr(P.subprocess,"run",lambda *a,**k:pytest.fail("invalid input started worker"))
    result=P.audit_file(path,dll=dll)
    assert result["status"]=="UNAVAILABLE" and result["map_accepted"] is False


@pytest.mark.parametrize("deadline",[True,0,-1,float("nan"),float("inf"),"1",None])
def test_invalid_timeout_never_starts_worker(tmp_path,monkeypatch,deadline):
    monkeypatch.setattr(P.subprocess,"run",lambda *a,**k:pytest.fail("invalid timeout started worker"))
    assert P.audit_file(write_input(tmp_path),timeout_s=deadline)["status"]=="UNAVAILABLE"


def test_missing_files_return_nonpass_instead_of_uncaught_error(tmp_path,monkeypatch):
    monkeypatch.setattr(P.subprocess,"run",lambda *a,**k:pytest.fail("missing file started worker"))
    assert P.audit_file(tmp_path/"absent.xodr")["status"]=="UNAVAILABLE"
    assert P.audit_file(write_input(tmp_path),dll=tmp_path/"absent.dll")["status"]=="UNAVAILABLE"


def test_native_process_crash_without_result_is_fail_closed(tmp_path,monkeypatch):
    path=write_input(tmp_path);dll=tmp_path/"fake.dll";dll.write_bytes(b"unused")
    monkeypatch.setattr(P.subprocess,"run",lambda cmd,**k:subprocess.CompletedProcess(cmd,-1073741819,b"",b"crash"))
    assert P.audit_file(path,dll=dll)["status"]=="UNAVAILABLE"


def test_timeout_terminates_actual_owned_controlled_process(tmp_path,monkeypatch):
    """Real child lifecycle test, deliberately not a native consumer check."""
    path=write_input(tmp_path);dll=tmp_path/"fake.dll";dll.write_bytes(b"unused")
    worker=tmp_path/"controlled_worker.py";pidfile=tmp_path/"pid.txt"
    worker.write_text("import os,sys,time\nfrom pathlib import Path\nPath(sys.argv[1]).write_text(str(os.getpid()))\ntime.sleep(30)\n",encoding="utf8")
    actual_run=subprocess.run
    monkeypatch.setattr(P.subprocess,"run",lambda cmd,**k:actual_run([sys.executable,"-I","-B",str(worker),str(pidfile)],**k))
    started=time.monotonic();result=P.audit_file(path,dll=dll,timeout_s=1)
    assert result["status"]=="TIMEOUT" and time.monotonic()-started<10
    pid=int(pidfile.read_text())
    if os.name=="nt":
        kernel=ctypes.WinDLL("kernel32",use_last_error=True)
        kernel.OpenProcess.argtypes=[ctypes.c_uint32,ctypes.c_int,ctypes.c_uint32]
        kernel.OpenProcess.restype=ctypes.c_void_p
        kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
        kernel.CloseHandle.argtypes=[ctypes.c_void_p]
        handle=kernel.OpenProcess(0x100000,False,pid)
        if handle:
            try: assert kernel.WaitForSingleObject(handle,0)==0
            finally: kernel.CloseHandle(handle)
        else:
            assert ctypes.get_last_error()==87  # The terminated process no longer exists.
    else:
        with pytest.raises(ProcessLookupError):os.kill(pid,0)
