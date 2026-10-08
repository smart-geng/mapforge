"""Stage and exercise the current workbench in a fresh, unrelated directory.

This is an INTERNAL_REHEARSAL, not an installer or a distributable release.
It copies source data and fixed assets without changing their bytes, extracts
the publisher-verified Python archive, and uses only the locked local wheels.
Neither the development environment nor an existing target is modified.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import signal
import stat
import subprocess
import sys
import tarfile
import time

ROOT = Path(__file__).resolve().parents[1]
PYTHON_SHA256 = "7b0e380ea5690cef71edbaa282e585ec08ffbece7ea8155546939adeb3377e44"
UV_SHA256 = "15523ea73d7a313f214a1fa2a2c1f0e5d93c0d762b52fb6ddecfcc33c3003401"
DATASET_SHA256 = "c7e9b7866dad49d00f46f27a96ed60d852572e031bd102be9ee2d5716820ab71"
EXPECTED_CANDIDATE = "78b893993494b46ee5aa1b094f1fd2ad9f40efdad2d2c6c7943b515a5ea6cd86"
BASE = "out/scoreboard/20261008-safe-mouth-recovery-v3/shp-node16"
FIXED_ASSETS = (
    BASE + ".xodr", BASE + ".source-lanes.json", BASE + ".source-review.json",
    "out/workbench/20261008-crs-audit/report.json", "esmini/bin/esminiRMLib.dll",
    "OpenDRIVE_1.4H.xsd", "OpenDRIVE_1.5M.xsd", "uv.lock", "pyproject.toml", "LICENSE",
)


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_new(path, value):
    with Path(path).open("x", encoding="utf8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def verify_recorded_files(root, manifest):
    issues = []
    for name, info in manifest.items():
        try:
            path = root / name
            no_links(path)
            if path.stat().st_size != info["size"] or sha(path) != info["sha256"]:
                issues.append({"file": name, "reason": "bytes-changed"})
        except (OSError, ValueError) as exc:
            issues.append({"file": name, "reason": type(exc).__name__ + ": " + str(exc)})
    return issues


def no_links(path):
    """Reject both symlinks and Windows junctions before reading or creating."""
    for part in (Path(path).absolute(), *Path(path).absolute().parents):
        if part.is_symlink():
            raise ValueError("Linked path is not a rehearsal input/target: " + str(part))
        if part.exists() and getattr(part.lstat(), "st_file_attributes", 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            raise ValueError("Reparse point is not a rehearsal input/target: " + str(part))


def clean_environment():
    env = os.environ.copy()
    for key in list(env):
        if key.upper().startswith(("PYTHON", "UV_")) or key.upper() in {"VIRTUAL_ENV", "CONDA_PREFIX"}:
            env.pop(key)
    env.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PROJ_NETWORK="OFF")
    windows = Path(env.get("SystemRoot", r"C:\Windows"))
    env["PATH"] = os.pathsep.join((str(windows / "System32"), str(windows)))
    return env


def inputs(root, wheelhouse):
    """Explicit application/data/media roots; never copy .venv, tokens or jobs."""
    result = {}
    for directory in ("mapforge", "scripts", "spikes", "profiles", "ledger"):
        for p in sorted((root / directory).rglob("*")):
            no_links(p)
            if p.is_file() and "__pycache__" not in p.parts and p.suffix not in {".pyc", ".pyo"}:
                result["application/" + p.relative_to(root).as_posix()] = p
    for name in FIXED_ASSETS:
        result["application/" + name] = root / name
    for p in sorted((root / "shp_0222-0326").rglob("*")):
        no_links(p)
        if p.is_file():
            result["data/shp/" + p.relative_to(root / "shp_0222-0326").as_posix()] = p
    for name in ("requirements.lock.txt", "wheelhouse-manifest.json"):
        result["media/" + name] = wheelhouse / name
    for p in sorted((wheelhouse / "wheelhouse").glob("*.whl")):
        result["media/wheelhouse/" + p.name] = p
    result["tools/uv.exe"] = root / "out/tools/uv/bin/uv.exe"
    for name in ("LICENSE-APACHE", "LICENSE-MIT"):
        result["licenses/uv/" + name] = root / "out/tools/uv/uv-0.12.21.dist-info/licenses" / name
    for p in result.values():
        no_links(p)
        if not p.is_file():
            raise ValueError("Missing declared input: " + str(p))
    return result


def extract_python(archive, target):
    if sha(archive) != PYTHON_SHA256:
        raise ValueError("Python archive differs from official release 20260929 SHA256")
    result = {}
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        seen = set()
        reserved = {"CON", "PRN", "AUX", "NUL", *[f"COM{i}" for i in range(1, 10)],
                    *[f"LPT{i}" for i in range(1, 10)]}
        for member in members:
            raw_name = member.name[:-1] if member.isdir() and member.name.endswith("/") else member.name
            raw_parts = raw_name.split("/")
            name = PurePosixPath(member.name)
            if (not name.parts or name.parts[0] != "python" or name.is_absolute()
                    or any(p in {"", ".", ".."} or p.rstrip(" .") != p
                           or p.split(".")[0].upper() in reserved
                           or any(c in p for c in ':\\<>"|?*') for p in raw_parts)
                    or not (member.isfile() or member.isdir())):
                raise ValueError("Unsafe Python archive member: " + member.name)
            key = member.name.casefold().rstrip("/")
            if key in seen:
                raise ValueError("Duplicate Python archive member: " + member.name)
            seen.add(key)
        for member in members:
            path = target.joinpath(*PurePosixPath(member.name).parts)
            if member.isdir():
                path.mkdir(parents=True, exist_ok=True)
                continue
            path.parent.mkdir(parents=True, exist_ok=True)
            with tar.extractfile(member) as source, path.open("xb") as stream:
                shutil.copyfileobj(source, stream)
            if path.stat().st_size != member.size:
                raise ValueError("Incomplete Python extraction")
            result[path.relative_to(target.parent).as_posix()] = {"size": member.size, "sha256": sha(path)}
    return result


def _bootstrap_command(cwd, command):
    # Even the gated launcher comes from the staged runtime/application. The
    # development interpreter only orchestrates staging and owns the Job handle.
    return [str(cwd / "runtime/python/python.exe"), "-I", "-B",
            str(cwd / "application/scripts/workbench_rehearse_relocation.py"),
            "--child-bootstrap", json.dumps(command, ensure_ascii=False)]


def _new_windows_job():
    from scripts.workbench_run_source_cells import _WindowsJob
    return _WindowsJob()


def run_command(command, cwd, out, label, records, timeout=300):
    record = {"label": label, "argv": [str(v) for v in command], "cwd": str(cwd)}
    records.append(record)
    started = time.monotonic()
    bootstrap = _bootstrap_command(Path(cwd), record["argv"])
    record.update(bootstrap_argv=bootstrap, timed_out=False,
                  tree_control="Windows Job Object with gated launch" if os.name == "nt" else "owned POSIX session")
    process, job = None, None
    try:
        with (out / (label + ".stdout.log")).open("xb") as stdout, (out / (label + ".stderr.log")).open("xb") as stderr:
            job = _new_windows_job() if os.name == "nt" else None
            process = subprocess.Popen(bootstrap, cwd=cwd, env=clean_environment(),
                stdin=subprocess.PIPE, stdout=stdout, stderr=stderr, shell=False,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                start_new_session=os.name != "nt")
            record["bootstrap_pid"] = process.pid
            if job:
                job.assign(process)
            # No installation, smoke or native child can launch before its
            # bootstrap is assigned to the owned Job (or POSIX session).
            try:
                process.communicate(input=b"RUN\n", timeout=timeout)
            except subprocess.TimeoutExpired:
                record["timed_out"] = True
                raise
    finally:
        if job:
            job.close()  # KILL_ON_JOB_CLOSE includes descendants on every exit.
        if process is not None:
            if os.name != "nt":
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if process.poll() is None:
                # This also covers a failed Job assignment before RUN was sent.
                process.kill()
            process.communicate(timeout=10)
        record.update(exit_code=process.returncode if process is not None else None,
                      elapsed_s=time.monotonic() - started)
    if process.returncode:
        raise RuntimeError(label + " failed; see captured logs")


def child_smoke():
    """Run from the COPIED application with its own isolated venv interpreter."""
    sys.path.insert(0, str(ROOT))
    target = ROOT.parent
    assert sys.version_info[:3] == (3, 11, 16)
    assert Path(sys.prefix).resolve() == (target / ".venv").resolve()
    import importlib.metadata as metadata
    from html.parser import HTMLParser
    from fastapi.testclient import TestClient
    from mapforge.workbench.app import create_app
    from mapforge.workbench.sources import SourceCatalog, verify_source_snapshot
    from mapforge.workbench.store import ProjectStore
    from mapforge.workbench.compiler import register_verified_baseline, _quality_guard
    from mapforge.workbench.boundary_probe import compile_boundary_edit
    from mapforge.workbench.measurements import measurement_capability
    from scripts.workbench_esmini_portable import check_file as check_consumer
    import numpy as np
    import scipy.linalg
    from shapely.geometry import box
    from pyproj import Transformer, datadir
    from lxml import etree
    from pyclothoids import Clothoid
    plan = json.loads((target / "media/wheelhouse-manifest.json").read_text(encoding="utf8"))["plan"]
    versions = {w["name"]: metadata.version(w["name"]) for w in plan["wheels"]}
    assert all(versions[w["name"]] == w["version"] for w in plan["wheels"])
    assert np.allclose(scipy.linalg.solve([[2., 0.], [0., 3.]], [4., 6.]), [2., 2.])
    assert box(0, 0, 2, 2).area == 4
    assert etree.fromstring(b"<test/>").tag == "test"
    assert Transformer.from_crs(4326, 3857, always_xy=True).transform(0, 0) == (0., 0.)
    assert Clothoid.G1Hermite(0, 0, 0, 10, 1, .1).length > 0
    assert Path(datadir.get_data_dir()).resolve().is_relative_to(target)
    catalog = SourceCatalog(target / "data/shp", ROOT / "profiles/shp/ibd-smarteditor-v1.yaml")
    assert catalog.snapshot_id == DATASET_SHA256
    snapshot = catalog.snapshot("2023061509381992034")
    assert verify_source_snapshot(snapshot, catalog.source_dir, catalog.profile_path)["matches"]
    metric = measurement_capability(snapshot)
    assert metric["metric_available"] is True
    registration = register_verified_baseline(snapshot)
    candidate, proof = compile_boundary_edit(registration.prepared, -.015)
    _quality_guard(proof, -.015)
    candidate_sha = hashlib.sha256(candidate).hexdigest()
    assert candidate_sha == EXPECTED_CANDIDATE
    artifacts = target / "rehearsal-artifacts"
    artifacts.mkdir()
    (artifacts / "same-candidate.xodr").write_bytes(candidate)
    write_new(artifacts / "local-proof.json", proof)
    store = ProjectStore(target / "projects")
    project = store.create(snapshot, "跨盘符中文路径验证")
    reopened = ProjectStore(target / "projects").load(project["project_id"])
    assert reopened["source_snapshot"]["content_hash"] == snapshot["content_hash"]
    token = "internal-rehearsal-token-never-persisted"
    static = []
    class References(HTMLParser):
        def handle_starttag(self, tag, attrs):
            attrs = dict(attrs)
            url = attrs.get("src" if tag == "script" else "href")
            if tag in {"script", "link"} and url:
                static.append(url)
    with TestClient(create_app(store, catalog, token, 18766), base_url="http://127.0.0.1:18766") as client:
        index = client.get("/")
        assert index.status_code == 200
        References().feed(index.text)
        assert len(static) == 4
        for url in static:
            assert url.startswith("/assets/")
            response = client.get(url)
            assert response.status_code == 200
            assert response.content == (ROOT / "mapforge/workbench/static" / url.split("/")[-1]).read_bytes()
        headers = {"Authorization": "Bearer " + token}
        for url in ("/api/catalog", "/api/projects", "/api/projects/" + project["project_id"]):
            assert client.get(url, headers=headers).status_code == 200
    consumer = check_consumer(artifacts / "same-candidate.xodr")
    assert consumer["pass"] is True, consumer
    write_new(artifacts / "consumer-check.json", consumer)
    modules = {}
    for name, module in tuple(sys.modules.items()):
        location = getattr(module, "__file__", None)
        if not location or location.startswith("<"):
            continue
        resolved = Path(location).resolve()
        assert resolved.is_relative_to(target), (name, str(resolved))
        modules[name] = str(resolved.relative_to(target))
    result = {"schema": "mapforge/relocated-workbench-smoke/v1", "status": "PASS_RELOCATED_SAME_HOST",
        "target": str(target), "python": sys.version, "executable": sys.executable,
        "prefix": sys.prefix, "installed_versions": versions, "loaded_module_origins": modules,
        "all_loaded_python_modules_under_target": True, "proj_data_under_target": True,
        "dataset_sha256": catalog.snapshot_id, "source_file_count": len(catalog.files),
        "project_id": project["project_id"], "project_save_reopen": True,
        "metric_available": metric["metric_available"], "static_urls": static,
        "candidate_sha256": candidate_sha, "candidate_same_as_development_evidence": True,
        "local_candidate_guard": "PASS", "esmini_complete_legacy_check": consumer,
        "candidate_accepted": False, "full_quality_recomputed": False,
        "browser_behavior_verified": False, "independent_clean_windows": False,
        "system_msvc_still_used": True, "redistribution_approved": False, "formal_release_verified": False}
    write_new(target / "smoke-result.json", result)


def rehearse(out, target):
    no_links(out)
    no_links(target)
    out, target = out.resolve(), target.resolve()
    if out.exists() or target.exists():
        raise ValueError("Evidence and target directories must both be new")
    if out.resolve().is_relative_to(target.resolve()) or target.resolve().is_relative_to(out.resolve()):
        raise ValueError("Evidence and target must be disjoint")
    out.mkdir(parents=True)
    target.mkdir(parents=True)
    result = {"schema": "mapforge/portable-rehearsal/v1", "status": "FAILED", "target": str(target),
              "release_scope": "INTERNAL_REHEARSAL_ONLY", "formal_release_verified": False,
              "independent_clean_windows": False, "commands": []}
    mapped = {}
    original = {}
    runtime = {}
    archive = ROOT / "out/tools/cpython-3.11.16-complete.tar.gz"
    try:
        sys.path.insert(0, str(ROOT))
        from scripts.workbench_prepare_offline import verify_bundle
        wheelhouse = ROOT / "out/workbench/offline-wheelhouse-20261008"
        checked = verify_bundle(ROOT / "uv.lock", wheelhouse)
        if checked.get("complete") is not True:
            raise ValueError("Offline media not verified: " + str(checked.get("status")))
        mapped = inputs(ROOT, wheelhouse)
        original = {name: {"size": p.stat().st_size, "sha256": sha(p)} for name, p in mapped.items()}
        if sha(archive) != PYTHON_SHA256 or sha(mapped["tools/uv.exe"]) != UV_SHA256:
            raise ValueError("Publisher-verified runtime/tool bytes changed")
        needed = sum(v["size"] for v in original.values()) + 800_000_000
        if shutil.disk_usage(target).free < needed:
            raise ValueError("Insufficient space for stage plus new venv")
        for name, source in mapped.items():
            destination = target / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as reader, destination.open("xb") as writer:
                shutil.copyfileobj(reader, writer)
            if destination.stat().st_size != original[name]["size"] or sha(destination) != original[name]["sha256"]:
                raise ValueError("Copy verification failed: " + name)
        runtime = extract_python(archive, target / "runtime")
        manifest = {"schema": "mapforge/internal-relocation-stage/v1", "copied_files": original,
                    "runtime_files": runtime, "python_archive_sha256": PYTHON_SHA256,
                    "uv_exe_sha256": UV_SHA256, "publisher_identity_sources": {
                        "python": "https://github.com/astral-sh/python-build-standalone/releases/tag/20260929",
                        "uv": "https://github.com/astral-sh/uv/releases/tag/0.12.21"},
                    "scope": "INTERNAL_REHEARSAL; source and esmini/XSD redistribution not approved"}
        write_new(target / "stage-manifest.json", manifest)
        write_new(out / "stage-manifest.json", manifest)
        uv = target / "tools/uv.exe"
        runtime_python = target / "runtime/python/python.exe"
        python = target / ".venv/Scripts/python.exe"
        common = [str(uv), "--offline", "--no-cache", "--no-config", "--no-python-downloads"]
        run_command([*common, "venv", target / ".venv", "--python", runtime_python], target, out, "01-venv", result["commands"])
        run_command([*common, "pip", "sync", "--python", python, "--no-index", "--find-links", target / "media/wheelhouse",
                     "--require-hashes", "--only-binary", ":all:", "--strict", target / "media/requirements.lock.txt"],
                    target, out, "02-sync", result["commands"])
        run_command([*common, "pip", "check", "--python", python], target, out, "03-dependencies", result["commands"])
        run_command([python, "-I", "-B", target / "application/scripts/workbench_rehearse_relocation.py", "--child-smoke"],
                    target, out, "04-smoke", result["commands"])
        smoke = json.loads((target / "smoke-result.json").read_text(encoding="utf8"))
        if smoke["status"] != "PASS_RELOCATED_SAME_HOST":
            raise ValueError("No complete relocated smoke result")
        changed_stage = verify_recorded_files(target, original)
        if changed_stage:
            raise ValueError("Staged inputs changed during smoke: " + str(changed_stage))
        write_new(out / "smoke-result.json", smoke)
        result.update(status="PASS_RELOCATED_SAME_HOST", staged_files=len(original), runtime_files=len(runtime),
                      stage_manifest_sha256=sha(target / "stage-manifest.json"),
                      smoke_sha256=sha(target / "smoke-result.json"), source_data_copied=True,
                      default_geometry_changed=False, old_projects_modified=False,
                      prior_validation_reused=False, network_downloads_disabled_by_process_flags=True)
    except Exception as exc:
        result["error"] = type(exc).__name__ + ": " + str(exc)
    finally:
        input_issues = []
        for name, info in original.items():
            try:
                no_links(mapped[name])
                if sha(mapped[name]) != info["sha256"]:
                    input_issues.append({"file": name, "reason": "bytes-changed"})
            except (OSError, ValueError) as exc:
                input_issues.append({"file": name, "reason": type(exc).__name__ + ": " + str(exc)})
        try:
            no_links(archive)
            if sha(archive) != PYTHON_SHA256:
                input_issues.append({"file": str(archive), "reason": "python-archive-changed"})
        except (OSError, ValueError) as exc:
            input_issues.append({"file": str(archive), "reason": type(exc).__name__ + ": " + str(exc)})
        runtime_issues = verify_recorded_files(target, runtime)
        if runtime:
            extra = {p.relative_to(target).as_posix() for p in (target / "runtime").rglob("*") if p.is_file()} - set(runtime)
            runtime_issues += [{"file": n, "reason": "unrecorded-runtime-file"} for n in sorted(extra)]
        result["original_inputs_changed"] = input_issues
        result["runtime_integrity_issues"] = runtime_issues
        result["runtime_files_reverified"] = len(runtime)
        try:
            result["method_sha256"] = sha(Path(__file__))
        except OSError as exc:
            input_issues.append({"file": str(Path(__file__)), "reason": "method-unavailable: " + str(exc)})
            result["method_sha256"] = None
        if input_issues or runtime_issues:
            result.update(status="FAILED_INPUT_OR_RUNTIME_DRIFT")
        write_new(out / "report.json", result)
    return result


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--child-bootstrap":
        if sys.stdin.buffer.readline() != b"RUN\n":
            return 1
        command = json.loads(sys.argv[2])
        if not isinstance(command, list) or not command or not all(isinstance(v, str) for v in command):
            return 1
        return subprocess.run(command, cwd=Path.cwd(), stdin=subprocess.DEVNULL,
                              shell=False).returncode
    if sys.argv[1:] == ["--child-smoke"]:
        child_smoke()
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--target", type=Path, required=True)
    args = parser.parse_args()
    result = rehearse(args.out, args.target)
    print(json.dumps({k: result.get(k) for k in ("status", "target", "error")}, ensure_ascii=False))
    return 0 if result["status"] == "PASS_RELOCATED_SAME_HOST" else 1


if __name__ == "__main__":
    raise SystemExit(main())
