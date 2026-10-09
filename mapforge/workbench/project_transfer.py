"""Session-owned project packages; import always invalidates old acceptance.

Packages contain saved project/history bytes and terminal job receipts, never
the separately registered source dataset. Historical absolute locators are
opaque evidence. They are never used to locate files on the importing host.
Only this session's single background worker may publish a transfer result.
"""
from __future__ import annotations

import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import threading
import time
from uuid import uuid4
import zipfile

from . import project_relocation as relocation
from .contracts import canonical_bytes, digest

SCHEMA = "mapforge/project-transfer/v1"
RECEIPT_SCHEMA = "mapforge/project-transfer-publication/v1"
MAX_PACKAGE_BYTES = 512 * 1024 * 1024
MAX_MANIFEST_BYTES = 16 * 1024 * 1024
MAX_CHUNK_BYTES = 4 * 1024 * 1024
MAX_UPLOADS = 8
_PROJECT = re.compile(r"[0-9a-f]{32}")
_SHA = re.compile(r"[0-9a-f]{64}")
_REQUEST = re.compile(r"[A-Za-z0-9_.:-]{1,128}")
_FIELDS = {"schema", "project_id", "project_sha256", "source_identity", "files", "empty_dirs",
           "purpose", "recompute_required", "formal_delivery"}


class TransferRejected(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _reject(code, message):
    raise TransferRejected(code, message)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _file_binding(path, budget=MAX_PACKAGE_BYTES):
    return relocation._stream_binding(path, budget)


def _new(path, data):
    relocation._write_new(Path(path), data)


def _source_identity(snapshot):
    return {"snapshot_id": snapshot["snapshot_id"], "content_hash": snapshot["content_hash"],
            "junction_id": snapshot["junction_id"], "profile_sha256": snapshot["profile"]["sha256"]}


def _member(name):
    if (not isinstance(name, str) or not name or len(name) > 1024 or "\\" in name or ":" in name
            or any(ord(c) < 32 for c in name)):
        _reject("unsafe-package-path", "工程包成员路径无效")
    reserved = {"CON", "PRN", "AUX", "NUL", *[f"COM{i}" for i in range(1, 10)],
                *[f"LPT{i}" for i in range(1, 10)]}
    parts = name.split("/")
    if any(p in {"", ".", ".."} or len(p) > 255 or p.rstrip(" .") != p
           or p.split(".")[0].upper() in reserved or any(c in p for c in '<>"|?*') for p in parts):
        _reject("unsafe-package-path", "工程包包含越界或 Windows 不安全路径")
    return parts


def _layout(manifest):
    if (not isinstance(manifest, dict) or set(manifest) != _FIELDS or manifest["schema"] != SCHEMA
            or not isinstance(manifest["project_id"], str) or not _PROJECT.fullmatch(manifest["project_id"])
            or manifest["purpose"] != "SAVED_PROJECT_REQUIRES_IDENTICAL_SOURCE"
            or manifest["recompute_required"] is not True or manifest["formal_delivery"] is not False
            or not isinstance(manifest["project_sha256"], str) or not _SHA.fullmatch(manifest["project_sha256"])
            or not isinstance(manifest["source_identity"], dict)
            or not isinstance(manifest["files"], list) or not manifest["files"]
            or not isinstance(manifest["empty_dirs"], list)
            or len(manifest["files"]) + len(manifest["empty_dirs"]) > relocation.MAX_FILES):
        _reject("invalid-package-manifest", "工程包清单或版本无效")
    prefix = "project/" + manifest["project_id"] + "/"
    files, dirs, canonical_names, total = {}, set(), {}, 0
    for item in manifest["files"]:
        if (not isinstance(item, dict) or set(item) != {"path", "size", "sha256"}
                or type(item["size"]) is not int or not 0 <= item["size"] <= relocation.MAX_FILE_BYTES
                or not isinstance(item["sha256"], str) or not _SHA.fullmatch(item["sha256"])):
            _reject("invalid-package-manifest", "工程包文件大小或哈希无效")
        name = item["path"]
        _member(name)
        if (not name.startswith(prefix) and not re.fullmatch(r"jobs/(?:[0-9a-f]{32}|request-[0-9a-f]{64})\.json", name)):
            _reject("invalid-package-root", "工程包包含未声明的工程或任务根目录")
        if name in files:
            _reject("duplicate-package-path", "工程包清单含重复文件")
        files[name] = item
        total += item["size"]
    if total > relocation.MAX_TOTAL_BYTES:
        _reject("package-too-large", "工程包展开量超出预算")
    for name in manifest["empty_dirs"]:
        _member(name)
        if not name.startswith(prefix) or name in dirs or name in files:
            _reject("invalid-empty-directory", "空目录声明无效或重复")
        dirs.add(name)
    for name in [*files, *dirs]:
        parts = name.split("/")
        for end in range(1, len(parts) + 1):
            ancestor = "/".join(parts[:end])
            previous = canonical_names.setdefault(ancestor.casefold(), ancestor)
            if previous != ancestor:
                _reject("duplicate-package-path", "工程包目录或文件存在大小写冲突")
            if end < len(parts) and (ancestor in files or ancestor in dirs):
                _reject("package-path-conflict", "文件或已声明空目录被用作父目录")
    if prefix + "project.json" not in files:
        _reject("missing-project", "工程包缺少当前工程快照")
    return files, dirs


def _write_package(path, manifest, members, check=lambda: None):
    """Write this protocol's stored ZIP, with no compression or hidden entries."""
    declared, _ = _layout(manifest)
    if set(members) != set(declared):
        _reject("package-binding-mismatch", "待导出文件和清单不一致")
    raw = canonical_bytes(manifest)
    if len(raw) > MAX_MANIFEST_BYTES:
        _reject("manifest-too-large", "工程包清单过大")
    with zipfile.ZipFile(path, "x", compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
        for name in ["manifest.json", *sorted(members)]:
            check()
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED
            info.external_attr = 0o100644 << 16
            info.create_system = 3
            hashed, count = hashlib.sha256(), 0
            with archive.open(info, "w", force_zip64=True) as dest:
                if name == "manifest.json":
                    dest.write(raw)
                else:
                    with relocation._safe(members[name]).open("rb") as source:
                        while chunk := source.read(1024 * 1024):
                            check()
                            count += len(chunk)
                            if count > declared[name]["size"]:
                                _reject("export-input-drift", "导出文件读取期间大小发生变化")
                            hashed.update(chunk)
                            dest.write(chunk)
                    if count != declared[name]["size"] or hashed.hexdigest() != declared[name]["sha256"]:
                        _reject("export-input-drift", "导出文件读取期间字节发生变化")
            if path.stat().st_size > MAX_PACKAGE_BYTES:
                _reject("package-too-large", "工程包超出上传下载预算")
    if path.stat().st_size > MAX_PACKAGE_BYTES:
        _reject("package-too-large", "工程包超出上传下载预算")


def _read_package(path, destination, check=lambda: None):
    """Validate the whole ZIP directory before creating any member files."""
    path, destination = relocation._safe(path), relocation._safe(destination)
    if not path.is_file() or path.stat().st_size > MAX_PACKAGE_BYTES or destination.exists():
        _reject("invalid-package-input", "工程包或独占解包目录无效")
    with zipfile.ZipFile(path, "r") as archive:
        entries = archive.infolist()
        if not entries or len(entries) > relocation.MAX_FILES + 1 or archive.comment:
            _reject("invalid-package-entries", "工程包成员数量或注释不支持")
        by_name, folded, total = {}, set(), 0
        for info in entries:
            _member(info.filename)
            mode = info.external_attr >> 16
            if (info.orig_filename != info.filename or info.filename.casefold() in folded or info.is_dir()
                    or stat.S_IFMT(mode) not in {0, stat.S_IFREG}
                    or info.compress_type != zipfile.ZIP_STORED or info.compress_size != info.file_size
                    or info.flag_bits & 1 or info.comment
                    or info.file_size > (MAX_MANIFEST_BYTES if info.filename == "manifest.json" else relocation.MAX_FILE_BYTES)):
                _reject("unsafe-package-entry", "工程包含重复、链接、压缩、加密或超限成员")
            folded.add(info.filename.casefold())
            by_name[info.filename] = info
            total += info.file_size
        if total > relocation.MAX_TOTAL_BYTES or "manifest.json" not in by_name:
            _reject("invalid-package-entries", "工程包展开量超限或清单缺失")
        manifest = relocation._json(archive.read("manifest.json"))
        files, dirs = _layout(manifest)
        if set(by_name) != {"manifest.json", *files}:
            _reject("unlisted-package-entry", "工程包实际成员与清单不一致")
        if any(by_name[name].file_size != item["size"] for name, item in files.items()):
            _reject("package-size-mismatch", "工程包成员大小与清单不一致")
        check()
        destination.mkdir()
        for name in sorted(dirs):
            (destination / name).mkdir(parents=True, exist_ok=False)
        for name, item in files.items():
            check()
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            hashed, count = hashlib.sha256(), 0
            with archive.open(by_name[name], "r") as source, target.open("xb") as dest:
                while chunk := source.read(1024 * 1024):
                    check()
                    count += len(chunk)
                    if count > item["size"]:
                        _reject("package-size-mismatch", "工程包成员展开超出清单")
                    hashed.update(chunk)
                    dest.write(chunk)
            if count != item["size"] or hashed.hexdigest() != item["sha256"]:
                _reject("package-hash-mismatch", "工程包成员字节与清单不一致")
    project_dir = destination / "project" / manifest["project_id"]
    raw = relocation._read(project_dir / "project.json")
    project = relocation._project(raw, project_dir, manifest["project_id"])
    if _sha(raw) != manifest["project_sha256"] or _source_identity(project["source_snapshot"]) != manifest["source_identity"]:
        _reject("package-project-mismatch", "工程包未绑定实际来源工程")
    # relocate_project expects job archives beside the single project folder.
    jobs = destination / "jobs"
    if jobs.exists():
        os.rename(jobs, destination / "project" / ".jobs")
    archive_tree, selected = relocation._archives(project_dir.parent, manifest["project_id"])
    if set(archive_tree) != set(selected):
        _reject("foreign-project-jobs", "工程包包含其他工程的任务回执")
    return manifest, project_dir


class TransferService:
    """A single session's bounded uploads and one nonqueued background task."""
    def __init__(self, store, source_dir, profile_path):
        self.store = store
        self.source_dir, self.profile_path = relocation._safe(source_dir), relocation._safe(profile_path)
        self.session_id = uuid4().hex
        base = relocation._safe(store.root / ".transfers")
        base.mkdir(exist_ok=True)
        self.root = base / self.session_id
        self.root.mkdir()
        (self.root / "jobs").mkdir()
        (self.root / "uploads").mkdir()
        self._lock = threading.RLock()
        self._closed = False
        self._busy = None
        self._jobs, self._requests, self._uploads = {}, {}, {}
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mapforge-transfer")

    def _open(self):
        with self._lock:
            if self._closed:
                _reject("session-closed", "当前会话已关闭，未发布的迁移不得继续发布")

    def _job_path(self, job):
        return self.root / "jobs" / job["job_id"]

    @staticmethod
    def _public(job):
        return copy.deepcopy({k: v for k, v in job.items() if not k.startswith("_")})

    def _record(self, folder, name, value):
        path = folder / name
        relocation._safe(path)
        temporary = folder / ("." + name + "." + uuid4().hex + ".tmp")
        _new(temporary, canonical_bytes(value))
        os.replace(temporary, path)

    def _persist(self, job):
        self._record(self._job_path(job), "record.json", self._public(job))

    def _start(self, operation, payload, request_id):
        if not isinstance(request_id, str) or not _REQUEST.fullmatch(request_id):
            _reject("invalid-request-id", "请求 ID 无效")
        signature = digest({"operation": operation, "payload": payload})
        with self._lock:
            self._open()
            previous = self._requests.get(request_id)
            if previous:
                if previous["signature"] != signature:
                    _reject("request-conflict", "同一请求 ID 不能用于不同参数")
                return self._public(self._jobs[previous["job_id"]])
            if self._busy:
                _reject("transfer-busy", "本会话已有导出或导入任务，请等待结束")
            job = {"job_id": uuid4().hex, "operation": operation, "state": "running",
                   "project_id": payload.get("project_id"), "started_at": time.time(), "finished_at": None,
                   "result": None, "error": None, "session_id": self.session_id, "request_id": request_id}
            self._job_path(job).mkdir()
            self._jobs[job["job_id"]] = job
            self._requests[request_id] = {"signature": signature, "job_id": job["job_id"]}
            self._busy = job["job_id"]
            try:
                self._persist(job)
                future = self._executor.submit(self._run, job, operation, payload)
                future.add_done_callback(lambda done: self._cancelled_future(job) if done.cancelled() else None)
            except Exception:
                self._busy = None
                job.update(state="failed", finished_at=time.time(), error={"code": "job-start-failed", "message": "迁移任务未能启动"})
                raise
            return self._public(job)

    def _cancelled_future(self, job):
        with self._lock:
            job.update(state="cancelled", finished_at=time.time(), result=None,
                       error={"code": "session-closed", "message": "任务启动前会话已关闭"})
            if self._busy == job["job_id"]:
                self._busy = None
            try:
                self._persist(job)
            except Exception as exc:
                job["record_error"] = str(exc)

    def _run(self, job, operation, payload):
        try:
            self._open()
            if operation == "export":
                self._export(job, payload)
            else:
                self._import(job, payload)
        except Exception as exc:
            with self._lock:
                # A committed result has an immutable publication receipt;
                # do not call an already-published import a failed rollback.
                if job["state"] != "succeeded":
                    job.update(state="cancelled" if self._closed else "failed", result=None,
                        error={"code": getattr(exc, "code", "transfer-failed"), "message": f"{type(exc).__name__}: {exc}"})
        finally:
            with self._lock:
                job["finished_at"] = time.time()
                self._busy = None
                try:
                    self._persist(job)
                except Exception as exc:
                    job["record_error"] = str(exc)

    def get(self, job_id):
        with self._lock:
            if job_id not in self._jobs:
                _reject("unknown-transfer", "任务不属于当前会话")
            return self._public(self._jobs[job_id])

    def start_export(self, project_id, revision, request_id):
        if (not isinstance(project_id, str) or not _PROJECT.fullmatch(project_id)
                or type(revision) is not int or revision < 0):
            _reject("invalid-project-request", "工程或修订号无效")
        return self._start("export", {"project_id": project_id, "revision": revision}, request_id)

    def create_upload(self, name, size, sha256):
        if (not isinstance(name, str) or len(name) > 200 or "/" in name or "\\" in name
                or not name.endswith(".mapforge-project.zip") or any(ord(c) < 32 for c in name)
                or type(size) is not int or not 1 <= size <= MAX_PACKAGE_BYTES
                or not isinstance(sha256, str) or not _SHA.fullmatch(sha256)):
            _reject("invalid-upload", "上传名称、大小或 SHA256 无效")
        _member(name)
        with self._lock:
            self._open()
            if len(self._uploads) >= MAX_UPLOADS:
                _reject("upload-limit", "本会话上传数量已达上限，请关闭后重新打开工作台")
            if shutil.disk_usage(self.root).free < size * 3 + 64 * 1024 * 1024:
                _reject("insufficient-space", "磁盘空间不足以安全暂存工程包")
            upload = {"upload_id": uuid4().hex, "state": "receiving", "name": name,
                      "expected_size": size, "received_size": 0, "sha256": sha256, "error": None}
            folder = self.root / "uploads" / upload["upload_id"]
            folder.mkdir()
            _new(folder / "upload.zip", b"")
            self._uploads[upload["upload_id"]] = {**upload, "_hash": hashlib.sha256()}
            self._record(folder, "record.json", upload)
            return copy.deepcopy(upload)

    def get_upload(self, upload_id):
        with self._lock:
            if upload_id not in self._uploads:
                _reject("unknown-upload", "上传不属于当前会话")
            return self._public(self._uploads[upload_id])

    def append_upload(self, upload_id, offset, data):
        with self._lock:
            self._open()
            upload = self._uploads.get(upload_id)
            if upload is None:
                _reject("unknown-upload", "上传不属于当前会话")
            if (upload["state"] != "receiving" or type(offset) is not int or offset != upload["received_size"]
                    or not isinstance(data, bytes) or not 1 <= len(data) <= MAX_CHUNK_BYTES
                    or offset + len(data) > upload["expected_size"]):
                _reject("invalid-upload-chunk", "上传块顺序、状态或大小不匹配")
            folder = self.root / "uploads" / upload_id
            path = relocation._safe(folder / "upload.zip")
            try:
                if path.stat().st_size != offset:
                    _reject("upload-drift", "暂存上传文件已改变")
                with path.open("ab") as stream:
                    stream.write(data)
                    stream.flush()
                    os.fsync(stream.fileno())
                upload["_hash"].update(data)
                upload["received_size"] += len(data)
                if upload["received_size"] == upload["expected_size"]:
                    if upload["_hash"].hexdigest() != upload["sha256"]:
                        _reject("upload-hash-mismatch", "上传字节与声明 SHA256 不一致")
                    upload["state"] = "sealed"
            except Exception as exc:
                upload.update(state="failed", error={"code": getattr(exc, "code", "upload-failed"), "message": str(exc)})
            self._record(folder, "record.json", self._public(upload))
            return self._public(upload)

    def start_import(self, upload_id, request_id):
        with self._lock:
            if upload_id not in self._uploads:
                _reject("unknown-upload", "上传不属于当前会话")
        return self._start("import", {"upload_id": upload_id}, request_id)

    def _export_inputs(self, project_id, revision):
        base = self.store.project_path(project_id)
        if self.source_dir.is_relative_to(base) or self.profile_path.is_relative_to(base):
            _reject("source-inside-project", "原件或 Profile 位于工程内部，不能作为不含原料的工程包导出")
        tree = relocation._inventory(base)
        directories = relocation._directories(base)
        project_raw = relocation._read(base / "project.json")
        project = relocation._project(project_raw, base, project_id)
        for name in tree:
            if name == "previous.json" or (name.startswith("relocation-history/") and name.endswith("/project.original.json")):
                historic = relocation._project(relocation._read(base / name), base, project_id)
                if historic["revision"] > project["revision"]:
                    _reject("invalid-project-history", "历史工程版本晚于当前工程")
        if project["revision"] != revision:
            _reject("project-revision-conflict", "工程修订已变化，请刷新后导出")
        relocation._fresh_snapshot(self.source_dir, self.profile_path, project["source_snapshot"])
        archive_tree, jobs = relocation._archives(self.store.root, project_id)
        return base, project, project_raw, tree, directories, archive_tree, jobs

    def _publish_export(self, staged, target):
        self._open()
        if target.exists():
            _reject("publication-conflict", "导出包目标已存在")
        os.rename(staged, target)

    def _export(self, job, payload):
        folder = self._job_path(job)
        with self.store._locked():
            inputs = self._export_inputs(payload["project_id"], payload["revision"])
            base, project, raw, tree, directories, _, jobs = inputs
            prefix = "project/" + project["project_id"] + "/"
            members = {prefix + name: base / name for name in tree}
            members.update({"jobs/" + name: self.store.root / ".jobs" / name for name in jobs})
            files = [{"path": prefix + name, **binding} for name, binding in tree.items()]
            files += [{"path": "jobs/" + name, "size": len(data), "sha256": _sha(data)} for name, data in jobs.items()]
            empty = [prefix + name for name in directories
                     if not any(other.startswith(name + "/") for other in [*tree, *directories])]
            manifest = {"schema": SCHEMA, "project_id": project["project_id"], "project_sha256": _sha(raw),
                "source_identity": _source_identity(project["source_snapshot"]), "files": sorted(files, key=lambda item: item["path"]),
                "empty_dirs": sorted(empty), "purpose": "SAVED_PROJECT_REQUIRES_IDENTICAL_SOURCE",
                "recompute_required": True, "formal_delivery": False}
            staged = folder / "export.pending"
            staged.mkdir()
            package = staged / "project.mapforge-project.zip"
            _write_package(package, manifest, members, self._open)
            if self._export_inputs(payload["project_id"], payload["revision"]) != inputs:
                _reject("export-input-drift", "导出期间工程、来源或任务历史发生变化")
            binding = _file_binding(package)
            result = {"project_id": project["project_id"], "revision": project["revision"],
                "filename": project["project_id"] + ".mapforge-project.zip", "size": binding["size"],
                "sha256": binding["sha256"], "recompute_required": True, "formal_delivery": False,
                "source_included": False, "required_source": manifest["source_identity"]}
            _new(staged / "receipt.json", canonical_bytes({"schema": RECEIPT_SCHEMA, "operation": "export",
                "job_id": job["job_id"], "result": result, "manifest_sha256": digest(manifest)}))
            with self._lock:
                self._publish_export(staged, folder / "export")
                job.update(state="succeeded", result=result, _export_inputs=inputs, _manifest=manifest)

    def _publish_import(self, staged, target):
        self._open()
        relocation._publish(staged, target)

    def _import(self, job, payload):
        with self._lock:
            upload = self._public(self._uploads[payload["upload_id"]])
        if upload["state"] != "sealed":
            _reject("upload-not-complete", "工程包尚未完整上传或哈希校验失败")
        package = self.root / "uploads" / upload["upload_id"] / "upload.zip"
        binding = {"size": upload["expected_size"], "sha256": upload["sha256"]}
        if _file_binding(package) != binding:
            _reject("upload-drift", "工程包封存后字节已改变")
        folder = self._job_path(job)
        manifest, incoming = _read_package(package, folder / "unpacked", self._open)
        project_id = manifest["project_id"]
        with self._lock:
            job["project_id"] = project_id
        with self.store._locked():
            if self.store.project_path(project_id).exists():
                _reject("project-already-exists", "当前 workspace 已有同 ID 工程；不重命名或覆盖")
        self._open()
        moved = relocation.relocate_project(incoming, folder / "relocated", self.source_dir, self.profile_path)
        staged = Path(moved["project_directory"])
        tree = relocation._inventory(staged)
        directories = relocation._directories(staged)
        project = moved["project"]
        result = {"project_id": project_id, "revision": project["revision"], "draft_epoch": project["draft_epoch"],
            "candidate_stale": project["status"]["candidate_stale"], "validation_stale": project["status"]["validation_stale"],
            "recompute_required": True, "formal_delivery": False, "relocation_id": moved["relocation_id"],
            "source_identity": manifest["source_identity"]}
        receipt = {"schema": RECEIPT_SCHEMA, "operation": "import", "job_id": job["job_id"],
            "upload_sha256": binding["sha256"], "manifest_sha256": digest(manifest), "result": result,
            "current_project_sha256": tree["project.json"]["sha256"], "preserved_files": tree,
            "preserved_directories": sorted(directories), "destination_project_id": project_id,
            "publication": "visible only after atomic project directory rename", "old_evidence_rewritten": False}
        publication = staged / "transfer-publication" / job["job_id"]
        publication.mkdir(parents=True, exist_ok=False)
        _new(publication / "receipt.json", canonical_bytes(receipt))
        expected = {**tree, "transfer-publication/" + job["job_id"] + "/receipt.json":
                    {"size": len(canonical_bytes(receipt)), "sha256": digest(receipt)}}
        expected_dirs = directories | {"transfer-publication", "transfer-publication/" + job["job_id"]}
        with self.store._locked():
            target = self.store.project_path(project_id)
            if target.exists():
                _reject("project-already-exists", "导入期间出现同 ID 工程；不覆盖")
            relocation._fresh_snapshot(self.source_dir, self.profile_path, project["source_snapshot"])
            if (_file_binding(package) != binding or relocation._inventory(staged) != expected
                    or relocation._directories(staged) != expected_dirs):
                _reject("import-input-drift", "导入发布前包或暂存工程字节发生变化")
            with self._lock:
                self._publish_import(staged, target)
                job.update(state="succeeded", result=result)

    def download(self, job_id):
        with self._lock:
            self._open()
            job = self._jobs.get(job_id)
            if job is None or job["operation"] != "export" or job["state"] != "succeeded":
                _reject("export-unavailable", "当前会话没有此已完成导出包")
            result, inputs = copy.deepcopy(job["result"]), job["_export_inputs"]
            path = self._job_path(job) / "export" / "project.mapforge-project.zip"
        with self.store._locked():
            if self._export_inputs(result["project_id"], result["revision"]) != inputs:
                _reject("export-stale", "导出后工程、来源或任务历史已变化，请重新导出")
            if _file_binding(path) != {"size": result["size"], "sha256": result["sha256"]}:
                _reject("export-drift", "导出包字节已改变")
            data = path.read_bytes()
            if len(data) != result["size"] or _sha(data) != result["sha256"]:
                _reject("export-drift", "读取导出包期间字节发生变化")
            self._open()
            return result["filename"], data

    def close(self):
        with self._lock:
            self._closed = True
        self._executor.shutdown(wait=False, cancel_futures=True)
