"""Loopback-only workbench with registered sources and durable draft APIs."""
from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path
import secrets
import threading

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse

from .jobs import JobManager
from .editing import EditingService
from .checking import CheckingService
from .sources import verify_source_snapshot
from .store import (ProjectStore, StoreConflict, StoreNotFound, StoreReadOnly,
                    StoreValidation)

STATIC = Path(__file__).with_name("static")


def create_app(store: ProjectStore, catalog, token: str, port: int, *, jobs=None, editing=None, checking=None):
    if len(token) < 24:
        raise ValueError("A randomly generated session token is required")
    jobs = jobs or JobManager(archive_dir=store.root / ".jobs")
    editing = editing or EditingService(store, jobs, source_dir=catalog.source_dir,
                                        profile_path=catalog.profile_path)
    checking = checking or CheckingService(store, jobs, source_dir=catalog.source_dir,
                                           profile_path=catalog.profile_path)
    host = f"127.0.0.1:{port}"
    origin = f"http://{host}"
    integrity_issues = {}
    integrity_generation = {}
    job_observations = {}
    integrity_lock = threading.RLock()
    previous_validator = store.source_validator

    def source_guard(snapshot):
        issues = list(previous_validator(snapshot) or []) if previous_validator else []
        if snapshot.get("snapshot_id") != catalog.snapshot_id:
            issues.append({"code": "source-binding-mismatch", "message": "当前登记目录与工程源快照不同"})
        try:
            catalog.assert_unchanged()
        except (ValueError, OSError) as exc:
            issues.append({"code": "source-drift", "message": str(exc)})
        with integrity_lock:
            issues.extend(integrity_issues.get(snapshot.get("snapshot_id"), []))
        return issues

    store.source_validator = source_guard

    def equal_header(value, expected):
        return secrets.compare_digest(value.encode("utf-8"), expected.encode("utf-8"))

    @asynccontextmanager
    async def lifespan(app):
        yield
        jobs.close()

    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)

    @app.middleware("http")
    async def protect(request: Request, call_next):
        if request.headers.get("host") != host:
            return JSONResponse({"detail": "Host rejected"}, status_code=403)
        if request.headers.get("origin") not in (None, origin):
            return JSONResponse({"detail": "Origin rejected"}, status_code=403)
        if request.url.path.startswith("/api/"):
            if not equal_header(request.headers.get("authorization", ""), "Bearer " + token):
                return JSONResponse({"detail": "Session token required"}, status_code=403)
            if request.method not in ("GET", "HEAD"):
                if not equal_header(request.headers.get("x-mapforge-csrf", ""), token):
                    return JSONResponse({"detail": "CSRF token required"}, status_code=403)
                if request.headers.get("content-type", "").split(";")[0] != "application/json":
                    return JSONResponse({"detail": "JSON only"}, status_code=415)
                try:
                    declared = int(request.headers.get("content-length", "0"))
                except ValueError:
                    return JSONResponse({"detail": "Invalid body length"}, status_code=400)
                if declared < 0 or declared > 65536:
                    return JSONResponse({"detail": "Request too large"}, status_code=413)
                size = 0
                chunks = []
                async for chunk in request.stream():
                    size += len(chunk)
                    if size > 65536:
                        return JSONResponse({"detail": "Request too large"}, status_code=413)
                    chunks.append(chunk)
                request._body = b"".join(chunks)
        response = await call_next(request)
        response.headers.update({
            "Cache-Control": "no-store", "X-Content-Type-Options": "nosniff",
            "Referrer-Policy": "no-referrer",
            "Content-Security-Policy": "default-src 'self'; script-src 'self'; style-src 'self'; "
                                      "img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; "
                                      "base-uri 'none'; form-action 'self'; object-src 'none'",
        })
        return response

    async def invalid(request, exc):
        return JSONResponse({"detail": str(exc)}, status_code=422)

    app.add_exception_handler(ValueError, invalid)
    app.add_exception_handler(StoreValidation, invalid)

    @app.exception_handler(StoreConflict)
    @app.exception_handler(StoreReadOnly)
    async def conflict(request, exc):
        return JSONResponse({"detail": str(exc)}, status_code=409)

    @app.exception_handler(StoreNotFound)
    @app.exception_handler(KeyError)
    async def missing(request, exc):
        return JSONResponse({"detail": "工程或任务不存在"}, status_code=404)

    @app.exception_handler(OSError)
    async def disk_error(request, exc):
        return JSONResponse({"detail": "磁盘操作失败，工程未确认保存；请检查空间和权限"}, status_code=507)

    @app.get("/")
    def index():
        return FileResponse(STATIC / "index.html")

    @app.get("/assets/{name}")
    def asset(name: str):
        if name not in {"app.js", "editing.js", "style.css"}:
            return JSONResponse({"detail": "Not found"}, status_code=404)
        return FileResponse(STATIC / name)

    @app.get("/api/catalog")
    def source_catalog():
        return {"junctions": catalog.list_junctions(), "source_name": "已登记 SHP 原件",
                "capabilities": {"source_view": True, "annotation": True,
                                 "geometry_edit": "registered-scope-only", "formal_export": False},
                "notice": "开发版：源工程可保存待办；凤阁路—金剑路已登记一处共享边界修补。正式交付尚未开放。"}

    @app.get("/api/projects")
    def projects():
        return store.list()

    @app.post("/api/projects")
    def create(body: dict):
        junction_id = body.get("junction_id")
        name = body.get("name")
        if not isinstance(junction_id, str) or not isinstance(name, str) or not 1 <= len(name.strip()) <= 120:
            raise ValueError("请选择路口并填写 1–120 字工程名")
        snapshot = catalog.snapshot(junction_id)
        integrity = verify_source_snapshot(snapshot, catalog.source_dir, catalog.profile_path)
        if not integrity["matches"]:
            raise StoreReadOnly("导入期间原件发生变化，未保存混合来源；请重新登记源目录")
        return store.create(snapshot, name.strip())

    @app.get("/api/projects/{project_id}")
    def load(project_id: str):
        project = store.load(project_id)
        verification = verify_source_snapshot(project["source_snapshot"], catalog.source_dir,
                                              catalog.profile_path)
        snapshot_id = project["source_snapshot"]["snapshot_id"]
        with integrity_lock:
            integrity_generation[snapshot_id] = integrity_generation.get(snapshot_id, 0) + 1
            integrity_issues[snapshot_id] = verification["issues"]
        return store.load(project_id)

    @app.post("/api/projects/{project_id}/commands")
    def commit(project_id: str, body: dict):
        return store.commit(project_id, body.get("base_revision"), body.get("command"))

    @app.get("/api/projects/{project_id}/editing")
    def editing_status(project_id: str):
        return editing.describe(store.load(project_id))

    @app.get("/api/projects/{project_id}/editing/geometry")
    def editing_geometry(project_id: str, job_id: str | None = None):
        return editing.geometry(project_id, job_id=job_id)

    @app.post("/api/projects/{project_id}/editing/{operation}")
    def edit(project_id: str, operation: str, body: dict):
        # Only command/transaction identities cross HTTP. Paths, capability
        # specifications, candidate objects and claimed checks stay server-owned.
        allowed = {"enable": {"base_revision", "command_id"},
                   "preview": {"base_revision", "command", "request_id"},
                   "compile": {"base_revision", "request_id"},
                   "accept": {"base_revision", "command_id", "job_id"}}
        if operation in allowed and set(body) - allowed[operation]:
            raise ValueError("编辑请求包含未支持字段；候选、路径和检查结果由服务器生成")
        revision = body.get("base_revision")
        if operation == "enable":
            return editing.enable(project_id, revision, body.get("command_id"))
        if operation == "preview":
            return editing.preview(project_id, revision, body.get("command"), body.get("request_id"))
        if operation == "compile":
            return editing.compile(project_id, revision, body.get("request_id"))
        if operation == "accept":
            return editing.accept(project_id, revision, body.get("command_id"), body.get("job_id"))
        return JSONResponse({"detail": "未支持的编辑操作"}, status_code=404)

    @app.post("/api/projects/{project_id}/{action}")
    def action(project_id: str, action: str, body: dict):
        if action in {"undo", "redo"}:
            return getattr(store, action)(project_id, body.get("base_revision"), body.get("command_id"))
        if action == "export":
            return JSONResponse({"detail": "正式交付未开放：编辑编译、完整验证和发布材料尚未齐备"}, status_code=409)
        if action == "source-check":
            project = store.load(project_id)
            revision = body.get("base_revision")
            if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
                raise ValueError("base_revision 必须为非负整数")
            if revision != project["revision"]:
                raise StoreConflict("工程已变化，请重新载入后检查")
            snapshot_id = project["source_snapshot"]["snapshot_id"]
            with integrity_lock:
                observed = integrity_generation.get(snapshot_id, 0)
            result = jobs.start(project, "source_check", {
                "snapshot": project["source_snapshot"], "source_dir": str(catalog.source_dir),
                "profile_path": str(catalog.profile_path),
            }, body.get("request_id"))
            with integrity_lock:
                job_observations.setdefault(result["job_id"], observed)
            return result
        return JSONResponse({"detail": "未支持的操作"}, status_code=404)

    @app.post("/api/projects/{project_id}/checking/{operation}")
    def check(project_id: str, operation: str, body: dict):
        allowed = {"start": {"base_revision", "request_id"},
                   "attach": {"base_revision", "command_id", "job_id"}}
        if operation not in allowed:
            return JSONResponse({"detail": "未支持的检查操作"}, status_code=404)
        if set(body) - allowed[operation]:
            raise ValueError("检查请求包含未支持字段；检查证据由服务器生成")
        if operation == "start":
            return checking.start(project_id, body.get("base_revision"), body.get("request_id"))
        return checking.attach(project_id, body.get("base_revision"), body.get("command_id"), body.get("job_id"))

    @app.get("/api/projects/{project_id}/jobs/{job_id}")
    def job_status(project_id: str, job_id: str):
        project = store.load(project_id)
        result = jobs.get(job_id, project_id=project_id, revision=project["revision"],
                          content_hash=project["content_hash"])
        if (result["state"] == "succeeded" and result["operation"] == "source_check"
                and not result.get("historical")
                and result["result"]["issues"]):
            # A historical clean result must never clear a later drift finding.
            # Only an explicit current full-hash reopen may clear the cache.
            snapshot_id = project["source_snapshot"]["snapshot_id"]
            with integrity_lock:
                if (job_observations.get(job_id) == integrity_generation.get(snapshot_id, 0)):
                    integrity_issues[snapshot_id] = result["result"]["issues"]
        return result

    @app.get("/api/projects/{project_id}/jobs")
    def project_jobs(project_id: str):
        project = store.load(project_id)
        return jobs.list(project_id=project_id, revision=project["revision"],
                         content_hash=project["content_hash"])

    @app.post("/api/projects/{project_id}/jobs/{job_id}/cancel")
    def cancel(project_id: str, job_id: str, body: dict):
        store.load(project_id)
        return jobs.cancel(job_id, project_id=project_id)

    return app
