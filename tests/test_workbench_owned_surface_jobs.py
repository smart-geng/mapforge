"""Real Windows process ownership, not a geometry or candidate acceptance test.

The test context substitutes only the multiprocessing worker entry point. That
real daemon worker directly owns the unchanged source-tracks runner's Windows
Job Object and gated bootstrap. Its child and grandchild are controlled sleepers.
Every process used for cleanup is held by a Process/Popen or duplicated kernel
handle; no recorded PID is reopened or used to terminate a process.
"""
from __future__ import annotations

import json
import multiprocessing as mp
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

from mapforge.workbench.jobs import JobManager


pytestmark = pytest.mark.skipif(os.name != "nt", reason="Windows Job Object lifecycle contract")


# Run under the actual gated bootstrap. Each process gives the pytest process
# a handle to itself before acknowledging readiness. The handle still names
# this exact process after exit and cannot be confused with a reused PID.
_DESCENDANT = r'''
import _winapi, json, os, subprocess, sys, time
from pathlib import Path
owner, directory, depth = int(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3])
remote = _winapi.OpenProcess(_winapi.PROCESS_DUP_HANDLE, False, owner)
try:
    handle = _winapi.DuplicateHandle(_winapi.GetCurrentProcess(),
        _winapi.GetCurrentProcess(), remote, 0x00100001, False, 0)
finally:
    _winapi.CloseHandle(remote)
record = directory / ("descendant-" + str(depth) + ".json")
temporary = record.with_suffix(".tmp")
temporary.write_text(json.dumps({"handle": handle, "pid": os.getpid(),
    "owner": owner}), encoding="utf8")
os.replace(temporary, record)
child = None
try:
    if depth:
        child = subprocess.Popen([sys.executable, "-I", "-B", "-c", sys.argv[4],
            str(owner), str(directory), str(depth-1), sys.argv[4]],
            creationflags=subprocess.CREATE_NO_WINDOW)
    time.sleep(45)
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        child.wait(timeout=5)
'''


def _record_handle(handle, owner, path, pid):
    import _winapi
    remote = _winapi.OpenProcess(_winapi.PROCESS_DUP_HANDLE, False, owner)
    try:
        transferred = _winapi.DuplicateHandle(_winapi.GetCurrentProcess(), int(handle),
                                              remote, 0x00100001, False, 0)
    finally:
        _winapi.CloseHandle(remote)
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps({"handle": transferred, "pid": pid, "owner": owner}),
                         encoding="utf8")
    os.replace(temporary, path)


def _owned_tree_worker(connection, operation, payload):
    """Same direct runner ownership planned for compile_surface; no map code."""
    from scripts import workbench_run_source_cells as owner
    from scripts.workbench_run_source_tracks import _run_child

    assert operation == "compile_surface"
    assert mp.current_process().daemon is True
    directory = Path(payload["handles"])
    Path(payload["started"]).write_text(json.dumps({"pid": os.getpid(), "daemon": True}),
                                        encoding="utf8")
    original_popen = owner.subprocess.Popen

    def capture_bootstrap(*args, **kwargs):
        process = original_popen(*args, **kwargs)
        _record_handle(process._handle, payload["owner"], directory / "bootstrap.json", process.pid)
        return process

    owner.subprocess.Popen = capture_bootstrap
    if payload["buffer_result"]:
        def buffer_after_descendants_start():
            deadline = time.monotonic() + 12
            while not (directory / "descendant-0.json").exists():
                if time.monotonic() > deadline:
                    return
                time.sleep(.01)
            # Deliberately buffered success: cancellation must win, even though
            # this transport-shaped message contains a candidate-shaped object.
            connection.send({"state": "succeeded", "result": {"status": "COMPILED",
                "candidate": {"candidate_id": "controlled-not-a-map"}, "accepted": False}})
        threading.Thread(target=buffer_after_descendants_start, daemon=True).start()
    try:
        result = _run_child([sys.executable, "-I", "-B", "-c", _DESCENDANT,
                            str(payload["owner"]), str(directory), "1", _DESCENDANT], 30)
        # This return must never be observed by the cancelled/timed-out job.
        connection.send({"state": "failed", "error": "controlled tree unexpectedly completed: "
                         + str(result["exit_code"])})
    finally:
        owner.subprocess.Popen = original_popen
        connection.close()


class _OwnedTreeContext:
    """Use real spawn/Pipe, replacing the operation body only."""
    def __init__(self):
        self.context = mp.get_context("spawn")

    def Pipe(self, *args, **kwargs):
        return self.context.Pipe(*args, **kwargs)

    def Process(self, *, target, args, daemon):
        return self.context.Process(target=_owned_tree_worker, args=args, daemon=daemon)


def _wait_until(predicate, *, timeout=12, message):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, message
        time.sleep(.01)


def _collect_handles(directory, held):
    for path in directory.glob("*.json"):
        if path.name in held:
            continue
        record = json.loads(path.read_text(encoding="utf8"))
        assert record["owner"] == os.getpid()
        assert type(record["handle"]) is int and record["handle"] > 0
        held[path.name] = record


@pytest.mark.parametrize("finish,buffer_result", [
    ("cancel", False), ("timeout", False), ("close", False),
    ("cancel", True), ("close", True),
])
def test_outer_job_termination_reaps_owned_tree_and_keeps_terminal_archive(tmp_path, finish, buffer_result):
    import _winapi

    handles = tmp_path / "handles"
    handles.mkdir()
    archive = tmp_path / "jobs"
    project = {"project_id": "7" * 32, "revision": 3, "content_hash": "a" * 64,
               "candidate": None}
    payload = {"handles": str(handles), "started": str(tmp_path / "worker.json"),
               "owner": os.getpid(), "buffer_result": buffer_result}
    manager = JobManager(context=_OwnedTreeContext(), timeout_s=.1 if finish == "timeout" else 25,
                         archive_dir=archive)
    sentinel = subprocess.Popen([sys.executable, "-I", "-B", "-c", "import time; time.sleep(30)"],
                                creationflags=subprocess.CREATE_NO_WINDOW)
    held, job, worker = {}, None, None
    try:
        job = manager.start(project, "compile_surface", payload, "owned-tree")
        worker = manager._jobs[job["job_id"]]["_process"]
        _wait_until(lambda: all((handles / name).is_file() for name in
                               ("bootstrap.json", "descendant-1.json", "descendant-0.json")),
                    message="owned bootstrap, child and grandchild did not start")
        _collect_handles(handles, held)
        assert len(held) == 3 and len({value["pid"] for value in held.values()}) == 3
        assert json.loads(Path(payload["started"]).read_text(encoding="utf8")) == {
            "pid": worker.pid, "daemon": True}
        assert worker.is_alive() and sentinel.poll() is None
        assert all(_winapi.WaitForSingleObject(record["handle"], 0) == _winapi.WAIT_TIMEOUT
                   for record in held.values()), "every descendant must be live before termination"
        if buffer_result:
            _wait_until(lambda: manager._jobs[job["job_id"]]["_pipe"].poll(),
                        message="controlled late result was not buffered")
        if finish == "cancel":
            result = manager.cancel(job["job_id"], project_id=project["project_id"])
        elif finish == "close":
            manager.close()
            result = manager.get(job["job_id"], project_id=project["project_id"])
        else:
            _wait_until(lambda: time.monotonic() - manager._jobs[job["job_id"]]["_started"] > .1,
                        message="real outer deadline did not elapse")
            result = manager.get(job["job_id"], project_id=project["project_id"])
        expected = "timed_out" if finish == "timeout" else "cancelled"
        assert result["state"] == expected and "result" not in result
        assert not worker.is_alive()
        for name, record in held.items():
            assert _winapi.WaitForSingleObject(record["handle"], 5000) == _winapi.WAIT_OBJECT_0, name
        assert sentinel.poll() is None, "independent process was terminated"
        assert project["candidate"] is None
        observed = manager.get(job["job_id"], project_id=project["project_id"])
        assert observed["state"] == expected and "result" not in observed
        assert observed["archive"] == {"state": "persisted", "recorded_state": expected}
        recorded = json.loads((archive / (job["job_id"] + ".json")).read_text(encoding="utf8"))["payload"]
        assert recorded["state"] == expected and "result" not in recorded
        assert "succeeded" not in [event["state"] for event in recorded["events"]]
        reopened = JobManager(context=_OwnedTreeContext(), archive_dir=archive)
        try:
            old = reopened.start(project, "compile_surface", payload, "owned-tree")
            assert old["job_id"] == job["job_id"] and old["state"] == expected
            assert old["historical"] and "result" not in old
            assert "_process" not in reopened._jobs[job["job_id"]]
        finally:
            reopened.close()
        assert sentinel.poll() is None
    finally:
        manager.close()
        # Only handles explicitly duplicated by the exact processes launched
        # above are eligible for cleanup; never reopen a PID from a record.
        _collect_handles(handles, held)
        for record in held.values():
            handle = record["handle"]
            try:
                if _winapi.WaitForSingleObject(handle, 0) == _winapi.WAIT_TIMEOUT:
                    _winapi.TerminateProcess(handle, 99)
                    assert _winapi.WaitForSingleObject(handle, 5000) == _winapi.WAIT_OBJECT_0
            finally:
                _winapi.CloseHandle(handle)
        if worker is not None:
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=5)
            worker.close()
        if sentinel.poll() is None:
            sentinel.terminate()
        sentinel.wait(timeout=5)
