"""Rehearse a copied 0621 draft with a new offline runtime and fresh checks.

This is a same-host engineering rehearsal, not a clean-machine installer or
customer acceptance. All targets are new; old evidence keeps its original bytes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import workbench_rehearse_relocation as portable


def _wait(jobs, job, project_id):
    deadline = time.monotonic()+7500
    while time.monotonic() < deadline:
        state = jobs.get(job["job_id"], project_id=project_id)
        if state["state"] in {"succeeded", "failed", "cancelled", "timed_out"}:
            if state["state"] != "succeeded":
                raise RuntimeError("owned task failed: "+str(state))
            return state
        time.sleep(.25)
    raise TimeoutError("owned task did not finish")


def _consumer_summary(board):
    row = board["rows"][0]
    transport = row["consumer_transport"]
    consumer = transport["consumer_result"]
    assert transport["status"] == "CHECKED" and transport["unchanged"] is True
    assert not transport["problems"] and transport["error"] is None
    assert consumer["status"] == "CHECKED" and consumer["pass"] is True and consumer["gap_max_cm"] == 0
    assert all(consumer["evidence"][k] is True for k in ("exact_bytes", "full_check_executed", "unchanged"))
    assert consumer["evidence"]["loader"] == "RM_InitWithString"
    assert row["metrics"]["esmini_pass"] is True and row["metrics"]["esmini_gap_max_cm"] == 0
    return {"transport_status": transport["status"], "consumer_pass": consumer["pass"],
            "gap_max_cm": consumer["gap_max_cm"], "exact_bytes": True, "full_check_executed": True}


def child(project_id):
    from mapforge.workbench.project_relocation import relocate_project, _inventory
    from mapforge.workbench.sources import verify_source_snapshot
    from mapforge.workbench.store import ProjectStore
    from mapforge.workbench.jobs import JobManager
    from mapforge.workbench.surface_editing import SurfaceEditingService
    from mapforge.workbench.surface_checking import SurfaceCheckingService
    from mapforge.workbench.exports import ResearchExportService
    from mapforge.workbench import surface_compiler
    from mapforge.workbench.contracts import StoreConflict
    import pyproj

    target = ROOT.parent
    assert sys.version_info[:3] == (3, 11, 16)
    assert Path(sys.executable).resolve().is_relative_to(target)
    assert Path(sys.prefix).resolve().is_relative_to(target)
    assert Path(pyproj.datadir.get_data_dir()).resolve().is_relative_to(target)
    source, profile = ROOT/"shp_0222-0326", ROOT/"profiles/shp/ibd-smarteditor-v1.yaml"
    incoming = target/"incoming"/project_id
    original = _inventory(incoming)
    old = ProjectStore(incoming.parent).load(project_id)
    print("RELOCATING_SAVED_DRAFT", flush=True)
    moved = relocate_project(incoming, target/"projects", source, profile)
    portable.write_new(target/"relocation-result.json", moved)
    store = ProjectStore(target/"projects", source_validator=lambda s: verify_source_snapshot(s,source,profile)["issues"])
    initial = store.load(project_id)
    assert initial["intents"] == old["intents"] and initial["timeline"] == old["timeline"]
    assert initial["cursor"] == old["cursor"] and initial["content_hash"] == old["content_hash"]
    assert initial["candidate"] == old["candidate"] and initial["validation"] == old["validation"]
    assert initial["status"]["candidate_stale"] and initial["status"]["validation_stale"]
    assert initial["capabilities"] == {} and not initial["status"]["read_only"]
    jobs = JobManager(timeout_s=7400, archive_dir=store.root/".jobs")
    editing = SurfaceEditingService(store,jobs,source_dir=source,profile_path=profile)
    checking = SurfaceCheckingService(store,jobs,source_dir=source,profile_path=profile)
    exports = ResearchExportService(store,source_dir=source,profile_path=profile)
    denied = False
    try:
        exports.create(project_id,initial["revision"])
    except StoreConflict:
        denied = True
    assert denied
    states = []
    try:
        project = editing.enable(project_id,initial["revision"],"relocated-enable")["project"]
        assert project["status"]["candidate_stale"]
        print("FRESH_SOURCE_COMPILATION", flush=True)
        job = editing.compile(project_id,project["revision"],"relocated-compile")["job"]
        states.append(_wait(jobs,job,project_id))
        if states[-1]["result"].get("status") != "COMPILED":
            raise RuntimeError(str(states[-1]["result"]))
        project = editing.accept(project_id,project["revision"],"relocated-accept",job["job_id"])
        assert not project["status"]["candidate_stale"] and project["status"]["validation_stale"]
        print("INDEPENDENT_WHOLE_MAP_CHECK", flush=True)
        job = checking.start(project_id,project["revision"],"relocated-check")["job"]
        states.append(_wait(jobs,job,project_id))
        project = checking.attach(project_id,project["revision"],"relocated-save-check",job["job_id"])
        assert not project["status"]["candidate_stale"] and not project["status"]["validation_stale"]
        print("RESEARCH_EXPORT_AND_REOPEN", flush=True)
        receipt = exports.create(project_id,project["revision"])
        _,archive = exports.download(project_id,receipt["export_id"])
        assert receipt["purpose"] == "RESEARCH_ONLY" and receipt["formal_delivery"] is False
        (target/"relocated-research.zip").write_bytes(archive)
        reopened = ProjectStore(store.root).load(project_id)
        assert reopened["intents"] == old["intents"] and reopened["timeline"] == old["timeline"]
        assert reopened["cursor"] == old["cursor"] and reopened["content_hash"] == old["content_hash"]
        assert not reopened["status"]["candidate_stale"] and not reopened["status"]["validation_stale"]
        assert reopened["candidate"] == project["candidate"] and reopened["validation"] == project["validation"]
        verified = surface_compiler.verify_candidate_artifacts(store.project_path(project_id),project["candidate"])
        primary = Path(verified["primary_path"])
        assert primary.resolve().is_relative_to(target)
        assert hashlib.sha256(primary.read_bytes()).hexdigest() == "9aed2e47be1fc04deee6220d7f4bccdcc02004f65e31c99c1d7e3888aa96c224"
        board = json.loads((Path(verified["run_directory"])/"scoreboard.json").read_text(encoding="utf8"))
        assert all(board["rows"][0]["tiers"][t]["status"]=="FAIL" for t in ("T1","T2"))
        check_board = states[-1]["result"]["report"]["scoreboard"]
        assert check_board["rows"][0]["metrics"] == board["rows"][0]["metrics"]
        assert check_board["rows"][0]["tiers"] == board["rows"][0]["tiers"]
        consumers = {"compilation": _consumer_summary(board), "independent_validation": _consumer_summary(check_board)}
        modules={}
        for name,module in tuple(sys.modules.items()):
            location=getattr(module,"__file__",None)
            if not location or location.startswith("<"):continue
            p=Path(location).resolve()
            assert p.is_relative_to(target),(name,str(p))
            modules[name]=p.relative_to(target).as_posix()
        assert _inventory(incoming)==original
        result={"schema":"mapforge/surface-relocation-rehearsal/v1","status":"PASS_RELOCATED_SAME_HOST",
            "project_id":project_id,"target":str(target),"executable":sys.executable,"prefix":sys.prefix,
            "project_revision":project["revision"],"draft_epoch":project["draft_epoch"],
            "initial_old_candidate_stale":True,"initial_old_validation_stale":True,"old_export_refused":True,
            "fresh_compile_and_independent_check":True,"same_candidate_bytes":True,"candidate_path":str(primary),
            "tiers":board["rows"][0]["tiers"],"delivery":"BLOCKED","research_export":receipt,
            "consumers":consumers,"independent_validation_tiers":check_board["rows"][0]["tiers"],
            "compile_check_metrics_identical":True,
            "research_zip_sha256":hashlib.sha256(archive).hexdigest(),"reopened":True,
            "loaded_python_modules":modules,"all_loaded_modules_under_target":True,
            "module_inventory_scope":"main workflow process only; not every spawned worker",
            "proj_data_under_target":True,"original_incoming_unchanged":True,
            "jobs":[{k:j[k] for k in ("job_id","operation","state","started_at","finished_at")} for j in states],
            "clean_machine_verified":False,"browser_verified":False,"independent_operator_verified":False,
            "formal_release":False,"system_msvc_still_used":True}
        portable.write_new(target/"surface-workflow-result.json",result)
    finally:
        jobs.close()


def rehearse(project_directory, out, target):
    from mapforge.workbench.project_relocation import _inventory, _directories, _archives, _project, _read
    from scripts.workbench_prepare_offline import verify_bundle
    for p in (project_directory,out,target):portable.no_links(p)
    project_directory,out,target=map(lambda p:Path(p).resolve(),(project_directory,out,target))
    for a,b in ((out,target),(out,project_directory),(target,project_directory),(target,ROOT)):
        if a.is_relative_to(b) or b.is_relative_to(a):raise ValueError("Rehearsal paths must be disjoint")
    if out.exists() or target.exists():raise ValueError("Both evidence and target must be new")
    current=_project(_read(project_directory/"project.json"),project_directory,project_directory.name)
    if current["source_snapshot"]["junction_id"] != "2023062110304177600":raise ValueError("0621 rehearsal only")
    project_files=_inventory(project_directory)
    project_dirs=_directories(project_directory)
    _,archives=_archives(project_directory.parent,project_directory.name)
    wheelhouse=ROOT/"out/workbench/offline-wheelhouse-20261008"
    if verify_bundle(ROOT/"uv.lock",wheelhouse).get("complete") is not True:raise ValueError("Offline media incomplete")
    mapped=portable.inputs(ROOT,wheelhouse)
    mapped={name.replace("data/shp/","application/shp_0222-0326/",1) if name.startswith("data/shp/") else name:p for name,p in mapped.items()}
    mapped.update({"incoming/"+project_directory.name+"/"+name:project_directory/name for name in project_files})
    mapped.update({"incoming/.jobs/"+name:project_directory.parent/".jobs"/name for name in archives})
    original={name:{"size":p.stat().st_size,"sha256":portable.sha(p)} for name,p in mapped.items()}
    archive=ROOT/"out/tools/cpython-3.11.16-complete.tar.gz"
    if portable.sha(archive)!=portable.PYTHON_SHA256 or portable.sha(mapped["tools/uv.exe"])!=portable.UV_SHA256:
        raise ValueError("Publisher runtime/tool identity changed")
    if shutil.disk_usage(target.parent).free < sum(v["size"] for v in original.values())+800_000_000:
        raise ValueError("Insufficient space")
    out.mkdir(parents=True);target.mkdir(parents=True)
    result={"schema":"mapforge/surface-relocation-stage/v1","status":"FAILED","target":str(target),
            "project_id":project_directory.name,"commands":[],"formal_release":False,"clean_machine_verified":False}
    runtime={}
    try:
        for name,p in mapped.items():
            destination=target/name;destination.parent.mkdir(parents=True,exist_ok=True)
            with p.open("rb") as source,destination.open("xb") as dest:shutil.copyfileobj(source,dest)
        for name in project_dirs:
            (target/"incoming"/project_directory.name/name).mkdir(parents=True,exist_ok=True)
        assert not portable.verify_recorded_files(target,original)
        runtime=portable.extract_python(archive,target/"runtime")
        portable.write_new(target/"stage-manifest.json",{"copied_files":original,"runtime_files":runtime})
        portable.write_new(out/"stage-manifest.json",{"copied_files":original,"runtime_files":runtime})
        uv=target/"tools/uv.exe";python=target/".venv/Scripts/python.exe"
        common=[uv,"--offline","--no-cache","--no-config","--no-python-downloads"]
        portable.run_command([*common,"venv",target/".venv","--python",target/"runtime/python/python.exe"],target,out,"01-venv",result["commands"])
        portable.run_command([*common,"pip","sync","--python",python,"--no-index","--find-links",target/"media/wheelhouse",
                              "--require-hashes","--only-binary",":all:","--strict",target/"media/requirements.lock.txt"],target,out,"02-sync",result["commands"])
        portable.run_command([*common,"pip","check","--python",python],target,out,"03-dependencies",result["commands"])
        portable.run_command([python,"-I","-B",target/"application/scripts/workbench_rehearse_surface_relocation.py",
                              "--child",project_directory.name],target,out,"04-workflow",result["commands"],timeout=7600)
        workflow=json.loads((target/"surface-workflow-result.json").read_text(encoding="utf8"))
        assert workflow["status"]=="PASS_RELOCATED_SAME_HOST"
        portable.write_new(out/"surface-workflow-result.json",workflow)
        result.update(status=workflow["status"],staged_files=len(original),runtime_files=len(runtime),
                      source_project_unchanged=_inventory(project_directory)==project_files,
                      copied_inputs_unchanged=not portable.verify_recorded_files(target,original))
    except Exception as exc:
        result["error"]=f"{type(exc).__name__}: {exc}"
    finally:
        changes=[]
        for name,p in mapped.items():
            try:
                portable.no_links(p)
                if portable.sha(p)!=original[name]["sha256"]:
                    changes.append({"file":name,"reason":"bytes-changed"})
            except (OSError,ValueError) as exc:
                changes.append({"file":name,"reason":str(exc)})
        try:
            portable.no_links(archive)
            if portable.sha(archive)!=portable.PYTHON_SHA256:
                changes.append({"file":str(archive),"reason":"runtime-archive-changed"})
            result["source_project_unchanged"]=(_inventory(project_directory)==project_files
                                                 and _directories(project_directory)==project_dirs)
        except (OSError,ValueError) as exc:
            result["source_project_unchanged"]=False
            changes.append({"file":str(project_directory),"reason":str(exc)})
        result["original_input_changes"]=changes
        result["copied_input_changes"]=portable.verify_recorded_files(target,original)
        result["runtime_changes"]=portable.verify_recorded_files(target,runtime)
        if runtime:
            extras={p.relative_to(target).as_posix() for p in (target/"runtime").rglob("*") if p.is_file()}-set(runtime)
            result["runtime_changes"] += [{"file":name,"reason":"unrecorded-runtime-file"} for name in sorted(extras)]
        result["network_downloads_disabled_by_process_flags"]=True
        result["physical_network_disconnected_verified"]=False
        if (changes or result["copied_input_changes"] or result["runtime_changes"]
                or not result["source_project_unchanged"]):
            result["status"]="FAILED_INPUT_DRIFT"
        portable.write_new(out/"report.json",result)
    return result


def main():
    if len(sys.argv)==3 and sys.argv[1]=="--child":
        child(sys.argv[2]);return 0
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--target",type=Path,required=True)
    args=parser.parse_args()
    result=rehearse(args.project,args.out,args.target)
    print(json.dumps({k:result.get(k) for k in ("status","target","error")},ensure_ascii=False))
    return 0 if result["status"]=="PASS_RELOCATED_SAME_HOST" else 1


if __name__=="__main__":
    raise SystemExit(main())
