"""Failure isolation of relocation orchestration; no installers or geometry run."""
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tarfile
from types import SimpleNamespace

import pytest

from scripts import workbench_prepare_offline as offline
from scripts import workbench_rehearse_relocation as relocation


def digest(data):
    return hashlib.sha256(data).hexdigest()


def put(path, data=b"small fixture"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


def make_archive(path, entries):
    """Entries are deliberately small and are never executable."""
    with tarfile.open(path, "w:gz") as archive:
        for name, kind, data in entries:
            member = tarfile.TarInfo(name)
            member.type = kind
            if kind in {tarfile.SYMTYPE, tarfile.LNKTYPE}:
                member.linkname = "../../outside"
            member.size = len(data) if kind == tarfile.REGTYPE else 0
            archive.addfile(member, io.BytesIO(data) if member.isfile() else None)
    return path


@pytest.fixture
def media(tmp_path, monkeypatch):
    monkeypatch.setattr(relocation.sys, "path", list(relocation.sys.path))
    root = tmp_path / "original"
    wheelhouse = root / "out/workbench/offline-wheelhouse-20261008"
    for directory in ("mapforge", "scripts", "spikes", "profiles", "ledger"):
        put(root / directory / "fixture.txt")
    for name in relocation.FIXED_ASSETS:
        put(root / name)
    source = put(root / "shp_0222-0326/source.shp", b"source unchanged")
    uv = put(root / "out/tools/uv/bin/uv.exe", b"not executable")
    for name in ("LICENSE-APACHE", "LICENSE-MIT"):
        put(root / "out/tools/uv/uv-0.12.21.dist-info/licenses" / name)
    for name in ("requirements.lock.txt", "wheelhouse-manifest.json"):
        put(wheelhouse / name)
    put(wheelhouse / "wheelhouse/fixture-1.0-py3-none-any.whl")
    archive = root / "out/tools/cpython-3.11.16-complete.tar.gz"
    make_archive(archive, [("python/python.exe", tarfile.REGTYPE, b"fixture runtime"),
                           ("python/LICENSE.txt", tarfile.REGTYPE, b"fixture license")])
    monkeypatch.setattr(relocation, "ROOT", root)
    monkeypatch.setattr(relocation, "PYTHON_SHA256", digest(archive.read_bytes()))
    monkeypatch.setattr(relocation, "UV_SHA256", digest(uv.read_bytes()))
    monkeypatch.setattr(offline, "verify_bundle", lambda *args: {"complete": True})
    monkeypatch.setattr(relocation.shutil, "disk_usage", lambda _: SimpleNamespace(free=2_000_000_000))
    out, target = tmp_path / "evidence", tmp_path / "迁移 演练"
    calls = []
    state = SimpleNamespace(root=root, wheelhouse=wheelhouse, archive=archive, uv=uv,
                            source=source, out=out, target=target, calls=calls, after_smoke=None)

    def command_stub(command, cwd, evidence, label, records, timeout=300):
        # Installation/native smoke are outside this test's scope. Real staging,
        # hashing, tar validation and final report persistence remain active.
        assert cwd == target and evidence == out
        calls.append(label)
        records.append({"label": label, "argv": [str(v) for v in command], "exit_code": 0})
        if label == "04-smoke":
            (target / "smoke-result.json").write_text(
                json.dumps({"status": "PASS_RELOCATED_SAME_HOST"}), encoding="utf8")
            if state.after_smoke:
                state.after_smoke()

    monkeypatch.setattr(relocation, "run_command", command_stub)
    return state


@pytest.mark.parametrize("occupied", ["out", "target"])
def test_existing_directory_rejected_without_changing_or_creating_other_target(media, occupied):
    existing = getattr(media, occupied)
    marker = put(existing / "keep.txt", b"existing evidence must survive")
    with pytest.raises(ValueError, match="both be new"):
        relocation.rehearse(media.out, media.target)
    assert marker.read_bytes() == b"existing evidence must survive"
    assert list(existing.iterdir()) == [marker]
    other = media.target if occupied == "out" else media.out
    assert not other.exists()
    assert media.calls == []


@pytest.mark.parametrize("out_inside_target", [False, True])
def test_nested_evidence_and_target_are_rejected_before_creation(tmp_path, out_inside_target):
    outer = tmp_path / "outer"
    inner = outer / "inner"
    out, target = (inner, outer) if out_inside_target else (outer, inner)
    with pytest.raises(ValueError, match="disjoint"):
        relocation.rehearse(out, target)
    assert not outer.exists()


def test_symbolic_parent_is_rejected_before_any_child_is_created(tmp_path):
    destination = tmp_path / "destination"
    destination.mkdir()
    linked = tmp_path / "linked"
    try:
        linked.symlink_to(destination, target_is_directory=True)
    except (OSError, NotImplementedError) as exc:
        pytest.skip("Host does not permit directory symlinks: " + str(exc))
    try:
        with pytest.raises(ValueError, match="Linked path"):
            relocation.rehearse(tmp_path / "evidence", linked / "new-target")
        assert not (tmp_path / "evidence").exists()
        assert list(destination.iterdir()) == []
    finally:
        linked.unlink()


def test_symbolic_parent_guard_does_not_require_privileged_fixture(tmp_path, monkeypatch):
    linked = tmp_path / "reported-symlink"
    original_is_symlink = Path.is_symlink
    monkeypatch.setattr(Path, "is_symlink", lambda path: path == linked or original_is_symlink(path))
    with pytest.raises(ValueError, match="Linked path"):
        relocation.rehearse(tmp_path / "evidence", linked / "new-target")
    assert not (tmp_path / "evidence").exists()
    assert not linked.exists()


@pytest.mark.skipif(os.name != "nt", reason="Windows junction contract")
def test_windows_junction_parent_is_rejected_before_creation(tmp_path):
    destination = tmp_path / "destination"
    destination.mkdir()
    linked = tmp_path / "junction"
    process = subprocess.run(["cmd.exe", "/c", "mklink", "/J", str(linked), str(destination)],
                             capture_output=True, text=True, shell=False)
    if process.returncode:
        pytest.skip("Host cannot create junction: " + process.stderr)
    try:
        with pytest.raises(ValueError, match="Reparse point"):
            relocation.rehearse(tmp_path / "evidence", linked / "new-target")
        assert not (tmp_path / "evidence").exists()
        assert list(destination.iterdir()) == []
    finally:
        linked.rmdir()  # Remove only the temporary junction, never its target.


def test_input_inventory_rejects_reparse_point_before_reading(media, monkeypatch):
    original_lstat = Path.lstat

    def lstat(path):
        if path == media.root / "mapforge":
            return SimpleNamespace(st_file_attributes=stat.FILE_ATTRIBUTE_REPARSE_POINT,
                                   st_mode=stat.S_IFDIR)
        return original_lstat(path)

    monkeypatch.setattr(Path, "lstat", lstat)
    with pytest.raises(ValueError, match="Reparse point"):
        relocation.inputs(media.root, media.wheelhouse)
    assert not media.out.exists() and not media.target.exists()


def test_missing_declared_input_cannot_be_silently_omitted(media):
    media.uv.unlink()
    with pytest.raises(ValueError, match="Missing declared input"):
        relocation.inputs(media.root, media.wheelhouse)


@pytest.mark.parametrize("kind", [tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.FIFOTYPE,
                                  tarfile.CHRTYPE, tarfile.BLKTYPE])
def test_non_regular_archive_member_rejected_before_any_extraction(tmp_path, monkeypatch, kind):
    archive = make_archive(tmp_path / "runtime.tar.gz", [
        ("python/good.txt", tarfile.REGTYPE, b"good"), ("python/bad", kind, b"")])
    monkeypatch.setattr(relocation, "PYTHON_SHA256", digest(archive.read_bytes()))
    target = tmp_path / "runtime"
    with pytest.raises(ValueError, match="Unsafe Python archive"):
        relocation.extract_python(archive, target)
    assert not target.exists()


@pytest.mark.parametrize("name", ["../outside", "python/../outside", "/python/outside", "other/file",
                                  "python/C:/outside", "python\\outside", "python/a\\outside"])
def test_escaping_or_wrong_root_archive_member_rejected_before_writes(tmp_path, monkeypatch, name):
    archive = make_archive(tmp_path / "runtime.tar.gz", [
        ("python/good.txt", tarfile.REGTYPE, b"good"), (name, tarfile.REGTYPE, b"bad")])
    monkeypatch.setattr(relocation, "PYTHON_SHA256", digest(archive.read_bytes()))
    target = tmp_path / "runtime"
    with pytest.raises(ValueError, match="Unsafe Python archive"):
        relocation.extract_python(archive, target)
    assert not target.exists()
    assert not (tmp_path / "outside").exists()


@pytest.mark.parametrize("second", ["python/a", "python/A", "python/./a", "python//a", "python/a.", "python/a "])
def test_duplicate_archive_alias_rejected_before_any_extraction(tmp_path, monkeypatch, second):
    archive = make_archive(tmp_path / "runtime.tar.gz", [
        ("python/a", tarfile.REGTYPE, b"first"), (second, tarfile.REGTYPE, b"second")])
    monkeypatch.setattr(relocation, "PYTHON_SHA256", digest(archive.read_bytes()))
    target = tmp_path / "runtime"
    with pytest.raises(ValueError, match="Unsafe Python archive|Duplicate Python archive"):
        relocation.extract_python(archive, target)
    assert not target.exists()


def test_archive_wrong_publisher_hash_never_extracts(tmp_path):
    archive = make_archive(tmp_path / "runtime.tar.gz", [("python/a", tarfile.REGTYPE, b"small")])
    target = tmp_path / "runtime"
    with pytest.raises(ValueError, match="official release"):
        relocation.extract_python(archive, target)
    assert not target.exists()


@pytest.mark.parametrize("fault", ["same-size-change", "deleted", "permission-error", "os-error"])
def test_verify_recorded_files_reports_each_read_failure_without_throwing(tmp_path, monkeypatch, fault):
    bad = put(tmp_path / "first.txt", b"abcd")
    good = put(tmp_path / "second.txt", b"untouched")
    manifest = {p.name: {"size": p.stat().st_size, "sha256": digest(p.read_bytes())} for p in (bad, good)}
    if fault == "same-size-change":
        bad.write_bytes(b"abce")
    elif fault == "deleted":
        bad.unlink()
    else:
        original_sha = relocation.sha

        def broken_sha(path):
            if path == bad:
                raise (PermissionError("fixture denied") if fault == "permission-error"
                       else OSError("fixture read failed"))
            return original_sha(path)

        monkeypatch.setattr(relocation, "sha", broken_sha)
    issues = relocation.verify_recorded_files(tmp_path, manifest)
    assert len(issues) == 1 and issues[0]["file"] == "first.txt"
    expected = {"same-size-change": "bytes-changed", "deleted": "FileNotFoundError",
                "permission-error": "PermissionError: fixture denied", "os-error": "OSError: fixture read failed"}
    assert issues[0]["reason"].startswith(expected[fault])


def test_initial_input_read_error_is_saved_without_running_installation(media, monkeypatch):
    original_sha = relocation.sha

    def denied(path):
        if path == media.source:
            raise PermissionError("fixture source denied")
        return original_sha(path)

    monkeypatch.setattr(relocation, "sha", denied)
    result = relocation.rehearse(media.out, media.target)
    saved = json.loads((media.out / "report.json").read_text(encoding="utf8"))
    assert result == saved
    assert result["status"] == "FAILED"
    assert "PermissionError: fixture source denied" in result["error"]
    assert media.calls == []


@pytest.mark.parametrize("changed", ["archive", "runtime", "source"])
@pytest.mark.parametrize("operation", ["modify", "delete"])
def test_post_smoke_input_or_runtime_drift_cannot_pass_and_failure_is_persisted(media, changed, operation):
    paths = {"archive": media.archive, "runtime": media.target / "runtime/python/python.exe", "source": media.source}

    def damage():
        path = paths[changed]
        if operation == "delete":
            path.unlink()
        else:
            data = path.read_bytes()
            path.write_bytes(bytes([data[0] ^ 1]) + data[1:])

    media.after_smoke = damage
    result = relocation.rehearse(media.out, media.target)
    assert media.calls == ["01-venv", "02-sync", "03-dependencies", "04-smoke"]
    assert result == json.loads((media.out / "report.json").read_text(encoding="utf8"))
    assert result["status"] == "FAILED_INPUT_OR_RUNTIME_DRIFT"
    assert not result["formal_release_verified"]
    issues = result["runtime_integrity_issues"] if changed == "runtime" else result["original_inputs_changed"]
    assert issues
    expected = "FileNotFoundError" if operation == "delete" else (
        "python-archive-changed" if changed == "archive" else "bytes-changed")
    assert any(issue["reason"].startswith(expected) for issue in issues)


def test_unrecorded_runtime_file_cannot_hide_behind_successful_smoke(media):
    media.after_smoke = lambda: put(media.target / "runtime/python/unrecorded.dll", b"extra")
    result = relocation.rehearse(media.out, media.target)
    assert result["status"] == "FAILED_INPUT_OR_RUNTIME_DRIFT"
    assert result["runtime_integrity_issues"] == [
        {"file": "runtime/python/unrecorded.dll", "reason": "unrecorded-runtime-file"}]
    assert (media.out / "report.json").is_file()


def test_method_read_failure_at_finalization_does_not_lose_failure_report(media, monkeypatch):
    original_sha = relocation.sha

    def after_smoke():
        def unavailable_method(path):
            if Path(path) == Path(relocation.__file__):
                raise FileNotFoundError("fixture method removed")
            return original_sha(path)
        monkeypatch.setattr(relocation, "sha", unavailable_method)

    media.after_smoke = after_smoke
    result = relocation.rehearse(media.out, media.target)
    assert result["status"] != "PASS_RELOCATED_SAME_HOST"
    saved = json.loads((media.out / "report.json").read_text(encoding="utf8"))
    assert saved == result
    assert "fixture method removed" in json.dumps(saved)


def controlled_bootstrap(monkeypatch):
    # Tests use the current interpreter only as a harmless launcher. Production
    # must select the copied runtime; its path contract is checked separately.
    monkeypatch.setattr(relocation, "_bootstrap_command", lambda cwd, command:
        [sys.executable, "-I", "-B", str(Path(relocation.__file__).resolve()),
         "--child-bootstrap", json.dumps(command)])


def test_bootstrap_uses_only_staged_runtime_and_script(tmp_path):
    target = tmp_path / "中文 deployment"
    command = [str(target / ".venv/Scripts/python.exe"), "参数 有空格"]
    actual = relocation._bootstrap_command(target, command)
    assert actual[:5] == [str(target / "runtime/python/python.exe"), "-I", "-B",
                          str(target / "application/scripts/workbench_rehearse_relocation.py"),
                          "--child-bootstrap"]
    assert json.loads(actual[5]) == command


def test_owned_command_preserves_clean_env_cwd_and_exclusive_logs(tmp_path, monkeypatch):
    controlled_bootstrap(monkeypatch)
    monkeypatch.setenv("PYTHONPATH", "F:/development-must-not-be-imported")
    monkeypatch.setenv("VIRTUAL_ENV", "F:/old-venv")
    monkeypatch.setenv("UV_PYTHON", "F:/old-python.exe")
    code = ("import os,sys,json; print(json.dumps({'cwd':os.getcwd(),"
            "'stdin':sys.stdin.read(),'path':os.environ['PATH'],"
            "'inherited':[v for v in ('PYTHONPATH','VIRTUAL_ENV','UV_PYTHON') if v in os.environ],"
            "'proj_network':os.environ['PROJ_NETWORK']})); print('stderr preserved',file=sys.stderr)")
    records = []
    relocation.run_command([sys.executable, "-I", "-B", "-c", code], tmp_path, tmp_path,
                           "controlled", records, timeout=10)
    result = json.loads((tmp_path / "controlled.stdout.log").read_text())
    assert Path(result["cwd"]) == tmp_path and result["stdin"] == ""
    assert result["inherited"] == [] and result["proj_network"] == "OFF"
    assert "F:" not in result["path"]
    assert "stderr preserved" in (tmp_path / "controlled.stderr.log").read_text()
    assert records[0]["exit_code"] == 0 and records[0]["timed_out"] is False
    old_log = (tmp_path / "controlled.stdout.log").read_bytes()
    with pytest.raises(FileExistsError):
        relocation.run_command([sys.executable, "-c", "raise Exception('must not run')"],
                               tmp_path, tmp_path, "controlled", [], timeout=10)
    assert (tmp_path / "controlled.stdout.log").read_bytes() == old_log


def _windows_process_exited(pid, *, cleanup=False):
    import ctypes
    from ctypes import wintypes as W
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    api.OpenProcess.argtypes = [W.DWORD, W.BOOL, W.DWORD]
    api.OpenProcess.restype = W.HANDLE
    api.WaitForSingleObject.argtypes = [W.HANDLE, W.DWORD]
    api.WaitForSingleObject.restype = W.DWORD
    api.TerminateProcess.argtypes = [W.HANDLE, W.UINT]
    api.TerminateProcess.restype = W.BOOL
    api.CloseHandle.argtypes = [W.HANDLE]
    api.CloseHandle.restype = W.BOOL
    handle = api.OpenProcess(0x00100001 if cleanup else 0x00100000, False, pid)
    if not handle:
        assert ctypes.get_last_error() == 87
        return True
    try:
        exited = api.WaitForSingleObject(handle, 2000) == 0
        if not exited and cleanup:
            assert api.TerminateProcess(handle, 99)
            assert api.WaitForSingleObject(handle, 2000) == 0
        return exited
    finally:
        api.CloseHandle(handle)


@pytest.mark.skipif(os.name != "nt", reason="Windows Job Object ownership contract")
@pytest.mark.parametrize("finish", ["timeout", "failure", "success"])
def test_owned_command_reaps_descendants_without_touching_unrelated_process(tmp_path, monkeypatch, finish):
    controlled_bootstrap(monkeypatch)
    descendant = tmp_path / "owned_descendant.py"
    descendant.write_text(
        "import os,sys,subprocess,time\nfrom pathlib import Path\n"
        "out=Path(sys.argv[1]); depth=int(sys.argv[2]); finish=sys.argv[3]\n"
        "(out/(str(depth)+'.pid')).write_text(str(os.getpid()))\n"
        "if depth: subprocess.Popen([sys.executable,__file__,str(out),str(depth-1),finish])\n"
        "if depth==2 and finish!='timeout':\n"
        "    deadline=time.monotonic()+5\n"
        "    while not (out/'0.pid').exists() and time.monotonic()<deadline: time.sleep(.01)\n"
        "    sys.exit(7 if finish=='failure' else 0)\n"
        "while True: time.sleep(1)\n", encoding="utf8")
    sentinel = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                                creationflags=subprocess.CREATE_NO_WINDOW)
    records = []
    command = [sys.executable, str(descendant), str(tmp_path), "2", finish]
    try:
        if finish == "timeout":
            with pytest.raises(subprocess.TimeoutExpired):
                relocation.run_command(command, tmp_path, tmp_path, "tree", records, timeout=4)
            assert records[0]["timed_out"] is True
        elif finish == "failure":
            with pytest.raises(RuntimeError, match="tree failed"):
                relocation.run_command(command, tmp_path, tmp_path, "tree", records, timeout=10)
            assert records[0]["exit_code"] == 7
        else:
            relocation.run_command(command, tmp_path, tmp_path, "tree", records, timeout=10)
            assert records[0]["exit_code"] == 0
        for depth in range(3):
            pid = int((tmp_path / (str(depth) + ".pid")).read_text())
            assert _windows_process_exited(pid), (depth, pid)
        assert _windows_process_exited(records[0]["bootstrap_pid"])
        assert sentinel.poll() is None, "unrelated process was terminated"
    finally:
        for path in tmp_path.glob("*.pid"):
            _windows_process_exited(int(path.read_text()), cleanup=True)
        sentinel.terminate()
        sentinel.wait(timeout=5)


@pytest.mark.skipif(os.name != "nt", reason="Windows gated launch contract")
def test_failed_job_assignment_never_starts_the_command(tmp_path, monkeypatch):
    controlled_bootstrap(monkeypatch)
    closed = []
    class FailedJob:
        def assign(self, process):
            raise OSError("controlled assignment failure")
        def close(self):
            closed.append(True)
    monkeypatch.setattr(relocation, "_new_windows_job", FailedJob)
    marker = tmp_path / "must-not-run"
    command = [sys.executable, "-c", "from pathlib import Path; Path(" + repr(str(marker)) + ").touch()"]
    records = []
    with pytest.raises(OSError, match="controlled assignment failure"):
        relocation.run_command(command, tmp_path, tmp_path, "assignment", records, timeout=10)
    assert not marker.exists() and closed == [True]
    assert _windows_process_exited(records[0]["bootstrap_pid"])
