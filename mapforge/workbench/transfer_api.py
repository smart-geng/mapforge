"""HTTP contract for session-owned saved-project packages.

Only identities, the declared package size/hash and bounded chunk bytes cross
HTTP. Paths, manifests and publication stay server-owned. A package never
carries the registered source, and an imported project is never opened,
compiled, accepted or formally exported automatically.

Upload protocol: one raw ``application/octet-stream`` PUT per chunk at the
current received offset. It is the only non-JSON mutation the page accepts and
keeps the same Host, Origin, token and CSRF checks; see ``chunk_upload_limit``.
"""
from __future__ import annotations

import re
import threading

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, Response
from starlette.concurrency import run_in_threadpool

from . import project_transfer as transfer
from .project_relocation import RelocationRejected

PURPOSE = "SAVED_PROJECT_REQUIRES_IDENTICAL_SOURCE"
SUFFIX = ".mapforge-project.zip"
_CHUNK_PATH = re.compile(r"/api/transfers/uploads/[0-9a-f]{32}/chunks")
_STATUS = {"unknown-transfer": 404, "unknown-upload": 404, "export-unavailable": 404,
           "transfer-busy": 409, "request-conflict": 409, "session-closed": 409, "upload-limit": 409,
           "export-stale": 409, "export-drift": 409, "project-revision-conflict": 409,
           "insufficient-space": 507}


def chunk_upload_limit(request) -> int | None:
    """Byte limit of the single raw-body route, or None for JSON-only requests."""
    if request.method == "PUT" and _CHUNK_PATH.fullmatch(request.url.path):
        return transfer.MAX_CHUNK_BYTES
    return None


def _fields(body, allowed, message):
    if not isinstance(body, dict) or set(body) != allowed:
        raise transfer.TransferRejected("invalid-transfer-request", message)


def transfer_routes(store, catalog, factory=None):
    """Return the router and a close hook; the session service starts on first use."""
    factory = factory or (lambda: transfer.TransferService(store, catalog.source_dir, catalog.profile_path))
    lock, holder = threading.Lock(), {}
    router = APIRouter()

    def service():
        with lock:
            if holder.get("closed"):
                raise transfer.TransferRejected("session-closed", "当前会话已关闭")
            if "service" not in holder:
                holder["service"] = factory()
            return holder["service"]

    def close():
        with lock:
            holder["closed"] = True
            current = holder.pop("service", None)
        if current is not None:
            current.close()

    def call(action):
        try:
            return action(service())
        except (transfer.TransferRejected, RelocationRejected) as exc:
            return JSONResponse({"detail": str(exc), "code": exc.code}, status_code=_STATUS.get(exc.code, 422))

    @router.get("/api/transfers")
    def describe():
        return {"purpose": PURPOSE, "file_suffix": SUFFIX, "source_included": False,
                "recompute_required": True, "formal_delivery": False,
                "max_package_bytes": transfer.MAX_PACKAGE_BYTES, "chunk_bytes": transfer.MAX_CHUNK_BYTES,
                "max_uploads": transfer.MAX_UPLOADS,
                "notice": "工程包只含已保存工程、草稿历史和终态任务回执，不含原始 SHP 与 Profile；"
                          "导入方须登记同源原件。导入后旧候选和检查一律过期，需重新生成、独立检查并保存。"}

    @router.post("/api/transfers/exports")
    def start_export(body: dict):
        def run(s):
            _fields(body, {"project_id", "base_revision", "request_id"}, "导出只接受工程 ID、当前 base_revision 与请求 ID")
            return s.start_export(body["project_id"], body["base_revision"], body["request_id"])
        return call(run)

    @router.post("/api/transfers/uploads")
    def create_upload(body: dict):
        def run(s):
            _fields(body, {"name", "size", "sha256"}, "上传只接受文件名、字节数与 SHA256")
            return s.create_upload(body["name"], body["size"], body["sha256"])
        return call(run)

    @router.get("/api/transfers/uploads/{upload_id}")
    def get_upload(upload_id: str):
        return call(lambda s: s.get_upload(upload_id))

    @router.put("/api/transfers/uploads/{upload_id}/chunks")
    async def append_chunk(upload_id: str, offset: int, request: Request):
        data = await request.body()
        return await run_in_threadpool(call, lambda s: s.append_upload(upload_id, offset, data))

    @router.post("/api/transfers/imports")
    def start_import(body: dict):
        def run(s):
            _fields(body, {"upload_id", "request_id"}, "导入只接受已封存上传 ID 与请求 ID")
            return s.start_import(body["upload_id"], body["request_id"])
        return call(run)

    @router.get("/api/transfers/jobs/{job_id}")
    def get_job(job_id: str):
        return call(lambda s: s.get(job_id))

    @router.get("/api/transfers/jobs/{job_id}/package")
    def download(job_id: str):
        def run(s):
            name, data = s.download(job_id)
            result = s.get(job_id)["result"]
            return Response(data, media_type="application/zip", headers={
                "Content-Disposition": f'attachment; filename="{name}"',
                "X-Mapforge-Purpose": PURPOSE, "X-Content-SHA256": result["sha256"]})
        return call(run)

    return router, close
