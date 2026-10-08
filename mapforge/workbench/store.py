"""Atomic local draft storage, independent of conversion success and research Project.

Every read/write takes an OS lock shared by all ProjectStore instances/processes.
The previous complete snapshot is preserved. Recovery is explicit, and uncommitted
previews never enter the journal. There is no production export decision here.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from typing import Callable
from uuid import uuid4

from .contracts import (
    StoreConflict, StoreError, StoreNotFound, StoreReadOnly, StoreValidation,
    canonical_bytes, content_hash, context_for, digest, identifier, require_json,
    sha256, validate_capability, validate_command, validate_context, validate_snapshot,
)

SCHEMA = "mapforge-workbench-project-v1"
ENVELOPE = "mapforge-workbench-snapshot-v1"
_LOCKS: dict[str, threading.RLock] = {}
_LOCKS_GUARD = threading.Lock()


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ProjectStore:
    def __init__(self, root: str | Path, *, source_validator: Callable | None = None,
                 lock_timeout: float = 10.0):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.source_validator = source_validator
        self.lock_timeout = lock_timeout
        with _LOCKS_GUARD:
            self._thread_lock = _LOCKS.setdefault(str(self.root).casefold(), threading.RLock())

    @contextmanager
    def _locked(self):
        with self._thread_lock:
            lock_path = self.root / ".writer.lock"
            if lock_path.is_symlink():
                raise StoreValidation("存储锁不得为符号链接")
            with lock_path.open("a+b") as stream:
                stream.seek(0, os.SEEK_END)
                if stream.tell() == 0:
                    stream.write(b"\0")
                    stream.flush()
                deadline = time.monotonic() + self.lock_timeout
                while True:
                    stream.seek(0)
                    try:
                        if os.name == "nt":
                            import msvcrt
                            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
                        else:
                            import fcntl
                            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                        break
                    except OSError as exc:
                        if time.monotonic() >= deadline:
                            raise StoreConflict("另一个进程正在写入工程，请稍后重试") from exc
                        time.sleep(0.02)
                try:
                    yield
                finally:
                    stream.seek(0)
                    if os.name == "nt":
                        import msvcrt
                        msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
                    else:
                        import fcntl
                        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    def project_path(self, project_id: str) -> Path:
        if not isinstance(project_id, str) or not re.fullmatch(r"[0-9a-f]{32}", project_id):
            raise StoreValidation("无效的工程 ID")
        path = self.root / project_id
        if path.is_symlink() or path.resolve().parent != self.root:
            raise StoreValidation("工程路径越界或使用了符号链接")
        return path

    def _read_file(self, path: Path) -> dict:
        if path.is_symlink():
            raise StoreReadOnly("工程快照不得为符号链接")
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(doc, dict) or doc.get("schema") != ENVELOPE:
                raise ValueError("snapshot schema")
            value = doc["payload"]
            if not isinstance(value, dict) or doc.get("sha256") != digest(value) or value.get("schema") != SCHEMA:
                raise ValueError("checksum/schema")
            if value.get("content_hash") != content_hash(value["source_snapshot"], value["intents"]):
                raise ValueError("draft content hash")
            return value
        except (OSError, ValueError, TypeError, KeyError) as exc:
            raise StoreReadOnly(f"工程快照无法读取或校验失败: {path.name}") from exc

    def _read(self, project_id: str) -> tuple[dict, dict | None]:
        directory = self.project_path(project_id)
        if not directory.is_dir():
            raise StoreNotFound("工程不存在")
        current = directory / "project.json"
        try:
            value = self._read_file(current)
            recovery = None
        except StoreReadOnly as exc:
            try:
                value = self._read_file(directory / "previous.json")
            except StoreReadOnly:
                raise StoreReadOnly("工程和上一完整版本均无法读取；保留文件并从备份恢复") from exc
            recovery = {"required": True, "reason": str(exc), "available_revision": value["revision"],
                        "message": "已只读打开上一完整版本；显式恢复后才允许修改，损坏文件将被保留"}
        if value.get("project_id") != project_id:
            raise StoreReadOnly("工程目录与快照身份不一致")
        return value, recovery

    def _source_issues(self, project: dict) -> list:
        if self.source_validator is None:
            return []
        try:
            issues = self.source_validator(copy.deepcopy(project["source_snapshot"]))
            return list(issues or [])
        except Exception as exc:
            return [{"code": "source-check-failed", "message": str(exc)}]

    def _view(self, project: dict, recovery: dict | None = None) -> dict:
        result = copy.deepcopy(project)
        candidate = result.get("candidate")
        candidate_stale = bool(candidate and (candidate["target_content_hash"] != result["content_hash"]
                                             or candidate["context"] != result["context"]
                                             or candidate.get("accepted_epoch") != result["draft_epoch"]))
        validation = result.get("validation")
        validation_stale = bool(validation and (candidate_stale or not candidate
                                                or validation["candidate_hash"] != digest(candidate)
                                                or validation["context"] != result["context"]))
        issues = self._source_issues(result)
        result["status"] = {"read_only": bool(recovery or issues), "recovery": recovery,
                            "source_issues": issues, "candidate_stale": candidate_stale or bool(issues),
                            "validation_stale": validation_stale or bool(issues),
                            "can_undo": result["cursor"] > 0,
                            "can_redo": result["cursor"] < len(result["timeline"]),
                            "formal_export_available": False}
        return result

    def _write_atomic(self, path: Path, data: bytes) -> None:
        if path.is_symlink():
            raise StoreReadOnly("工程文件不得为符号链接")
        temp = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
        try:
            with temp.open("xb") as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, path)
        finally:
            temp.unlink(missing_ok=True)

    def _persist(self, project: dict, *, backup: bool = True) -> None:
        directory = self.project_path(project["project_id"])
        current = directory / "project.json"
        data = canonical_bytes({"schema": ENVELOPE, "payload": project, "sha256": digest(project)})
        # Validate before any mutation. A failed backup/write never replaces current.
        if backup and current.exists():
            self._read_file(current)
            self._write_atomic(directory / "previous.json", current.read_bytes())
        self._write_atomic(current, data)

    def create(self, snapshot: dict, name: str = "未命名工程") -> dict:
        snapshot = validate_snapshot(snapshot)
        if not isinstance(name, str) or not name.strip() or len(name) > 200:
            raise StoreValidation("工程名称需为 1～200 字符")
        with self._locked():
            project_id = uuid4().hex
            now = _now()
            project = {"schema": SCHEMA, "project_id": project_id, "name": name,
                       "revision": 0, "draft_epoch": 0, "created_at": now, "updated_at": now,
                       "source_snapshot": snapshot, "content_hash": content_hash(snapshot, []),
                       "context": context_for(snapshot), "intents": [], "timeline": [], "cursor": 0,
                       "actions": {}, "journal": [], "candidate": None, "candidate_history": [],
                       "validation": None, "validation_history": [], "capabilities": {}}
            directory = self.project_path(project_id)
            directory.mkdir()
            try:
                self._persist(project, backup=False)
            except Exception:
                # Only this newly-created empty directory is removed; no recursive delete.
                if not any(directory.iterdir()):
                    directory.rmdir()
                raise
            return self._view(project)

    def load(self, project_id: str) -> dict:
        with self._locked():
            return self._view(*self._read(project_id))

    def list(self) -> list[dict]:
        with self._locked():
            result = []
            for directory in sorted(self.root.iterdir()):
                if not directory.is_dir() or not re.fullmatch(r"[0-9a-f]{32}", directory.name):
                    continue
                try:
                    project = self._view(*self._read(directory.name))
                    result.append({key: project[key] for key in
                                   ("project_id", "name", "revision", "updated_at", "status")})
                except StoreError as exc:
                    result.append({"project_id": directory.name, "name": "无法读取的工程",
                                   "status": {"read_only": True, "error": str(exc)}})
            return result

    def _begin(self, project_id: str, base_revision: int, command_id: str,
               request: dict) -> tuple[dict, bool]:
        identifier(command_id, "command_id")
        if isinstance(base_revision, bool) or not isinstance(base_revision, int) or base_revision < 0:
            raise StoreValidation("base_revision 必须为非负整数")
        project, recovery = self._read(project_id)
        if recovery or self._source_issues(project):
            raise StoreReadOnly("工程处于恢复或源漂移只读状态")
        previous = project["actions"].get(command_id)
        if previous:
            if previous["request_hash"] != digest(request):
                raise StoreConflict("同一 command_id 不能用于不同请求")
            return project, True
        if project["revision"] != base_revision:
            raise StoreConflict(f"工程已更新：请求版本 {base_revision}，当前版本 {project['revision']}")
        return project, False

    def _finish(self, project: dict, command_id: str, request: dict, kind: str) -> dict:
        project["revision"] += 1
        if kind in ("commit", "undo", "redo", "context", "capabilities"):
            project["draft_epoch"] += 1
        project["updated_at"] = _now()
        project["content_hash"] = content_hash(project["source_snapshot"], project["intents"])
        project["actions"][command_id] = {"request_hash": digest(request), "revision": project["revision"]}
        project["journal"].append({"command_id": command_id, "kind": kind,
                                   "revision": project["revision"], "content_hash": project["content_hash"],
                                   "at": project["updated_at"]})
        self._persist(project)
        return self._view(project)

    def preview(self, project_id: str, base_revision: int, command: dict) -> dict:
        """Pure structural preview; never claims a geometry candidate or mutates disk."""
        if isinstance(base_revision, bool) or not isinstance(base_revision, int) or base_revision < 0:
            raise StoreValidation("base_revision 必须为非负整数")
        with self._locked():
            project, recovery = self._read(project_id)
            if recovery or self._source_issues(project):
                raise StoreReadOnly("工程只读")
            if project["revision"] != base_revision:
                raise StoreConflict("预览基于过期工程版本")
            intent = validate_command(command, project["source_snapshot"], base_revision,
                                      capabilities=project.get("capabilities"), context=project["context"])
            if intent["command_id"] in project["actions"]:
                raise StoreConflict("该命令已提交；请刷新工程或使用新命令 ID")
            target = content_hash(project["source_snapshot"], project["intents"] + [intent])
            return {"project_id": project_id, "base_revision": base_revision,
                    "target_content_hash": target, "context": copy.deepcopy(project["context"]),
                    "intent": intent, "compiled": False}

    def commit(self, project_id: str, base_revision: int, command: dict) -> dict:
        require_json(command)
        if not isinstance(command, dict):
            raise StoreValidation("命令必须为 JSON 对象")
        command = copy.deepcopy(command)
        request = {"operation": "commit", "base_revision": base_revision, "command": command}
        with self._locked():
            project, duplicate = self._begin(project_id, base_revision, command.get("command_id"), request)
            if duplicate:
                return self._view(project)
            intent = validate_command(command, project["source_snapshot"], base_revision,
                                      capabilities=project.get("capabilities"), context=project["context"])
            project["timeline"] = project["timeline"][:project["cursor"]] + [intent]
            project["cursor"] += 1
            project["intents"] = copy.deepcopy(project["timeline"][:project["cursor"]])
            return self._finish(project, intent["command_id"], request, "commit")

    def _travel(self, project_id: str, base_revision: int, command_id: str, step: int) -> dict:
        kind = "undo" if step < 0 else "redo"
        request = {"operation": kind, "base_revision": base_revision}
        with self._locked():
            project, duplicate = self._begin(project_id, base_revision, command_id, request)
            if duplicate:
                return self._view(project)
            cursor = project["cursor"] + step
            if cursor < 0 or cursor > len(project["timeline"]):
                raise StoreConflict("没有可撤销/重做的操作")
            project["cursor"] = cursor
            project["intents"] = copy.deepcopy(project["timeline"][:cursor])
            return self._finish(project, command_id, request, kind)

    def undo(self, project_id: str, base_revision: int, command_id: str) -> dict:
        return self._travel(project_id, base_revision, command_id, -1)

    def redo(self, project_id: str, base_revision: int, command_id: str) -> dict:
        return self._travel(project_id, base_revision, command_id, 1)

    def recover(self, project_id: str) -> dict:
        """Explicitly recover previous snapshot, preserving corrupt current bytes."""
        with self._locked():
            project, recovery = self._read(project_id)
            if recovery is None:
                raise StoreConflict("工程无需恢复")
            directory = self.project_path(project_id)
            current = directory / "project.json"
            if current.is_symlink():
                raise StoreReadOnly("拒绝恢复符号链接快照")
            if current.exists():
                self._write_atomic(directory / f"corrupt-{uuid4().hex}.json", current.read_bytes())
            # Recovery must not reuse the lost transaction's revision. Millisecond
            # epoch remains an exact JS integer and invalidates pre-crash windows.
            project["revision"] = max(project["revision"] + 1, int(time.time() * 1000))
            project["draft_epoch"] += 1
            project["updated_at"] = _now()
            project["journal"].append({"kind": "recovery", "revision": project["revision"],
                                       "at": project["updated_at"], "from_revision": recovery["available_revision"]})
            self._persist(project, backup=False)
            return self._view(project)

    def set_context(self, project_id: str, base_revision: int, command_id: str,
                    compiler_hash: str, policy_hash: str) -> dict:
        sha256(compiler_hash, "compiler_hash")
        sha256(policy_hash, "policy_hash")
        request = {"operation": "context", "base_revision": base_revision,
                   "compiler_hash": compiler_hash, "policy_hash": policy_hash}
        with self._locked():
            project, duplicate = self._begin(project_id, base_revision, command_id, request)
            if duplicate:
                return self._view(project)
            project["context"] = context_for(project["source_snapshot"], compiler_hash, policy_hash)
            return self._finish(project, command_id, request, "context")

    def register_capabilities(self, project_id: str, base_revision: int, command_id: str,
                              capabilities: list[dict], compiler_hash: str, policy_hash: str) -> dict:
        """Server-only registration after compiler capability verification.

        The HTTP draft command API cannot call this method. Metadata validation
        here protects source binding; the compiler owns capability proof and
        must still run real geometry guards for every candidate.
        """
        sha256(compiler_hash, "compiler_hash")
        sha256(policy_hash, "policy_hash")
        require_json(capabilities)
        if not isinstance(capabilities, list) or not capabilities:
            raise StoreValidation("需要至少一个已核验的服务器能力")
        request = {"operation": "register_capabilities", "base_revision": base_revision,
                   "capabilities": copy.deepcopy(capabilities), "compiler_hash": compiler_hash,
                   "policy_hash": policy_hash}
        with self._locked():
            project, duplicate = self._begin(project_id, base_revision, command_id, request)
            if duplicate:
                return self._view(project)
            context = context_for(project["source_snapshot"], compiler_hash, policy_hash)
            registered = project.setdefault("capabilities", {})
            seen = set()
            for spec in request["capabilities"]:
                normalized = validate_capability(spec, project["source_snapshot"], context)
                key = normalized["capability_id"]
                if key in seen:
                    raise StoreValidation("能力登记包含重复 capability_id")
                seen.add(key)
                if key in registered and registered[key]["spec_hash"] != normalized["spec_hash"]:
                    raise StoreConflict("已登记能力 ID 不得原地改变规格；新规格需使用新 ID")
                registered[key] = normalized
            project["context"] = context
            return self._finish(project, command_id, request, "capabilities")

    def _artifact(self, project_id: str, item: dict) -> None:
        if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
            raise StoreValidation("产物必须包含 relative_path 与 sha256")
        relative = item["relative_path"]
        sha256(item["sha256"], "artifact.sha256")
        if (not isinstance(relative, str) or "\\" in relative or ":" in relative
                or not relative.startswith("artifacts/")
                or any(part in ("", ".", "..") for part in relative.split("/"))):
            raise StoreValidation("候选路径必须位于工程 artifacts/ 内")
        base = self.project_path(project_id)
        path = base.joinpath(*PurePosixPath(relative).parts)
        if not path.resolve().is_relative_to(base) or any(p.is_symlink() for p in [path, *path.parents] if p != base.parent):
            raise StoreValidation("候选路径越界或存在符号链接")
        try:
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError as exc:
            raise StoreValidation("候选产物缺失") from exc
        if actual != item["sha256"]:
            raise StoreValidation("候选产物哈希不一致")

    def accept_candidate(self, project_id: str, base_revision: int,
                         command_id: str, candidate: dict) -> dict:
        """Accept actual artifacts only after a compiler has produced matching input hashes."""
        require_json(candidate)
        if not isinstance(candidate, dict):
            raise StoreValidation("候选必须为对象")
        candidate = copy.deepcopy(candidate)
        required = {"candidate_id", "target_content_hash", "context", "artifacts", "affected_ids", "checks"}
        if set(candidate) != required:
            raise StoreValidation("候选字段不完整或包含未支持字段")
        identifier(candidate["candidate_id"], "candidate_id")
        sha256(candidate["target_content_hash"], "target_content_hash")
        validate_context(candidate["context"])
        request = {"operation": "candidate", "base_revision": base_revision, "candidate": candidate}
        with self._locked():
            project, duplicate = self._begin(project_id, base_revision, command_id, request)
            if duplicate:
                return self._view(project)
            if candidate["context"] != project["context"] or candidate["target_content_hash"] != project["content_hash"]:
                raise StoreConflict("候选已过期；不能替换当前草稿的结果")
            if not isinstance(candidate["artifacts"], list) or not candidate["artifacts"]:
                raise StoreValidation("候选没有实际产物")
            if not isinstance(candidate["affected_ids"], list) or not isinstance(candidate["checks"], list):
                raise StoreValidation("候选需提供影响集合与检查清单")
            ids = {x["id"] for x in project["source_snapshot"]["objects"]}
            if any(not isinstance(x, str) or x not in ids for x in candidate["affected_ids"]):
                raise StoreValidation("候选影响集合包含未知对象")
            for item in candidate["artifacts"]:
                self._artifact(project_id, item)
            if project["candidate"]:
                project["candidate_history"].append(project["candidate"])
            project["candidate"] = copy.deepcopy(candidate)
            project["candidate"]["accepted_revision"] = project["revision"] + 1
            project["candidate"]["accepted_epoch"] = project["draft_epoch"]
            return self._finish(project, command_id, request, "candidate")

    def attach_validation(self, project_id: str, base_revision: int,
                          command_id: str, validation: dict) -> dict:
        require_json(validation)
        required = {"bundle_id", "candidate_id", "candidate_hash", "context", "checks", "decision"}
        if not isinstance(validation, dict) or set(validation) != required:
            raise StoreValidation("检查包字段不完整或包含未支持字段")
        validation = copy.deepcopy(validation)
        identifier(validation["bundle_id"], "bundle_id")
        validate_context(validation["context"])
        if not isinstance(validation["checks"], list) or not validation["checks"]:
            raise StoreValidation("检查包不能为空")
        if validation["decision"] not in ("BLOCKED", "REVIEW", "DELIVERABLE"):
            raise StoreValidation("未知检查裁决")
        request = {"operation": "validation", "base_revision": base_revision, "validation": validation}
        with self._locked():
            project, duplicate = self._begin(project_id, base_revision, command_id, request)
            if duplicate:
                return self._view(project)
            candidate = project["candidate"]
            if (not candidate or self._view(project)["status"]["candidate_stale"]
                    or validation["candidate_id"] != candidate["candidate_id"]
                    or validation["candidate_hash"] != digest(candidate)
                    or validation["context"] != project["context"]):
                raise StoreConflict("检查包不属于当前有效候选")
            for item in candidate["artifacts"]:
                self._artifact(project_id, item)
            if project["validation"]:
                project["validation_history"].append(project["validation"])
            # Validation advances the transaction revision, not the draft epoch.
            project["validation"] = copy.deepcopy(validation)
            return self._finish(project, command_id, request, "validation")
