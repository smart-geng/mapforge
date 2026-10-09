"""Copy a closed source project without transferring acceptance to a new site.

Old evidence is copied byte for byte and is never rewritten to hide absolute
paths. Only the new current snapshot changes: location, transaction/epoch,
capabilities and an explicit journal entry. Recompilation and checking remain
separate, server-owned operations. This is not an installer or release gate.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import stat
from uuid import uuid4

from .contracts import (canonical_bytes, content_hash, context_for, digest, identifier, sha256, validate_context,
                        validate_snapshot)
from .jobs import ARCHIVE_SCHEMA, JobManager, _TERMINAL
from .sources import SourceCatalog, verify_source_snapshot
from .store import ENVELOPE, SCHEMA as PROJECT_SCHEMA, ProjectStore

SCHEMA = "mapforge/workbench-project-relocation/v1"
MAX_FILE_BYTES = 128 * 1024 * 1024
MAX_TOTAL_BYTES = 2 * 1024 * 1024 * 1024
MAX_FILES = 20000
_ID = re.compile(r"[0-9a-f]{32}")


class RelocationRejected(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.pending_directory = None


def _reject(code, message):
    raise RelocationRejected(code, message)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _safe(path):
    path = Path(path).absolute()
    for part in (path, *path.parents):
        if part.is_symlink():
            _reject("linked-path", "迁移路径不得使用符号链接")
        try:
            info = part.lstat()
        except FileNotFoundError:
            continue
        if getattr(info, "st_file_attributes", 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            _reject("linked-path", "迁移路径不得使用 Windows junction 或重解析点")
        if not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode)):
            _reject("irregular-file", "迁移路径包含不规则文件")
    return path.resolve()


def _read(path):
    path = _safe(path)
    if not path.is_file() or path.stat().st_size > MAX_FILE_BYTES:
        _reject("file-unavailable-or-too-large", "迁移输入不存在、不是普通文件或超出大小限制")
    raw = path.read_bytes()
    if len(raw) > MAX_FILE_BYTES:
        _reject("file-too-large", "迁移输入读取期间超出大小限制")
    return raw


def _json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                _reject("invalid-json", "JSON 含重复键")
            result[key] = value
        return result
    def invalid(value):
        _reject("invalid-json", "JSON 含非有限数值: " + value)
    try:
        value = json.loads(raw.decode("utf8"), object_pairs_hook=pairs, parse_constant=invalid)
        canonical_bytes(value)
        return value
    except (UnicodeError, ValueError, TypeError) as exc:
        if isinstance(exc, RelocationRejected):
            raise
        _reject("invalid-json", "迁移 JSON 无法严格解析: " + str(exc))


def _stream_binding(path, max_file_bytes):
    """Hash file content in bounded chunks; this never relaxes JSON parsing."""
    path = _safe(path)
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_size > max_file_bytes:
        _reject("file-unavailable-or-too-large", "清单输入不是普通文件或超出该类文件大小限制")
    hashed, count = hashlib.sha256(), 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            count += len(chunk)
            if count > max_file_bytes:
                _reject("file-too-large", "清单输入读取期间超出大小限制")
            hashed.update(chunk)
        if count != info.st_size or os.fstat(stream.fileno()).st_size != count:
            _reject("file-drift", "清单输入读取期间大小发生变化")
    _safe(path)
    return {"size": count, "sha256": hashed.hexdigest()}


def _inventory(root, *, max_file_bytes=None):
    # Source SHP bytes may use the full tree budget; project, JSON and artifact
    # inventory callers retain the smaller default. None is resolved at call
    # time, keeping the two budgets independently testable.
    if max_file_bytes is None:
        max_file_bytes = MAX_FILE_BYTES
    if type(max_file_bytes) is not int or max_file_bytes <= 0:
        _reject("invalid-file-budget", "文件大小预算必须为正整数")
    root = _safe(root)
    if not root.is_dir():
        _reject("missing-directory", "迁移目录不存在")
    result, seen, total, entries = {}, set(), 0, 0
    for parent, dirs, files in os.walk(root, followlinks=False):
        for name in sorted([*dirs, *files]):
            path = _safe(Path(parent) / name)
            entries += 1
            if entries > MAX_FILES or not path.is_relative_to(root):
                _reject("tree-limit-or-escape", "迁移目录超出数量限制或范围")
            relative = path.relative_to(root).as_posix()
            if relative.casefold() in seen:
                _reject("duplicate-path", "迁移目录包含大小写冲突路径")
            seen.add(relative.casefold())
            if path.is_file():
                binding = _stream_binding(path, min(max_file_bytes, MAX_TOTAL_BYTES - total))
                total += binding["size"]
                if total > MAX_TOTAL_BYTES:
                    _reject("tree-too-large", "迁移目录超出总量限制")
                result[relative] = binding
    return result


def _directories(root):
    result = set()
    for parent, dirs, _ in os.walk(root, followlinks=False):
        for name in dirs:
            path = _safe(Path(parent) / name)
            if not path.is_relative_to(root):
                _reject("path-escape", "目录清单越界")
            result.add(path.relative_to(root).as_posix())
    return result


def _write_new(path, raw):
    _safe(path.parent)
    with path.open("xb") as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())


def _copy_tree(source, target, manifest):
    target.mkdir()
    # Preserve empty directories as well as all recorded file bytes.
    for parent, dirs, _ in os.walk(source, followlinks=False):
        for name in sorted(dirs):
            path = _safe(Path(parent) / name)
            if not path.is_relative_to(source):
                _reject("path-escape", "复制目录越界")
            (target / path.relative_to(source)).mkdir()
    for name, expected in manifest.items():
        raw = _read(source / name)
        if {"size": len(raw), "sha256": _sha(raw)} != expected:
            _reject("project-drift", "复制期间源工程文件发生变化")
        _write_new(target / name, raw)
    if _inventory(target) != manifest:
        _reject("copy-mismatch", "工程副本未完整保留原字节")


def _artifact(base, item):
    if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
        _reject("invalid-history", "历史产物清单格式无效")
    name = item["relative_path"]
    sha256(item["sha256"], "artifact.sha256")
    if (not isinstance(name, str) or not name.startswith("artifacts/") or ":" in name or "\\" in name
            or any(part in {"", ".", ".."} for part in name.split("/"))):
        _reject("invalid-history-path", "历史产物路径越界")
    path = _safe(base / name)
    if not path.is_relative_to(base) or _sha(_read(path)) != item["sha256"]:
        _reject("invalid-history-artifact", "历史产物不存在或字节与清单不一致")


def _candidate(base, item, project):
    fields = {"candidate_id", "target_content_hash", "context", "artifacts", "affected_ids", "checks",
              "accepted_revision", "accepted_epoch"}
    if not isinstance(item, dict) or set(item) != fields:
        _reject("invalid-history", "历史候选字段不完整")
    identifier(item["candidate_id"])
    sha256(item["target_content_hash"], "target_content_hash")
    validate_context(item["context"])
    for key, bound in (("accepted_revision", project["revision"]), ("accepted_epoch", project["draft_epoch"])):
        if type(item[key]) is not int or not 0 <= item[key] <= bound:
            _reject("invalid-history", "历史候选版本无效")
    if (not isinstance(item["artifacts"], list) or not item["artifacts"]
            or not isinstance(item["checks"], list) or not isinstance(item["affected_ids"], list)):
        _reject("invalid-history", "历史候选清单无效")
    objects = {o["id"] for o in project["source_snapshot"]["objects"]}
    if any(not isinstance(x, str) or x not in objects for x in item["affected_ids"]):
        _reject("invalid-history", "历史候选引用未知来源对象")
    seen = set()
    for file in item["artifacts"]:
        _artifact(base, file)
        if file["relative_path"] in seen:
            _reject("invalid-history", "历史候选产物重复")
        seen.add(file["relative_path"])


def _validation(base, item):
    fields = {"bundle_id", "candidate_id", "candidate_hash", "context", "checks", "decision"}
    if (not isinstance(item, dict) or set(item) != fields or not isinstance(item["checks"], list)
            or not item["checks"] or item["decision"] not in {"BLOCKED", "REVIEW", "DELIVERABLE"}):
        _reject("invalid-history", "历史检查记录无效")
    identifier(item["bundle_id"])
    identifier(item["candidate_id"])
    sha256(item["candidate_hash"], "candidate_hash")
    validate_context(item["context"])
    for check in item["checks"]:
        if not isinstance(check, dict):
            _reject("invalid-history", "历史检查项无效")
        if check.get("gate") == "validation-byte-binding":
            files = check.get("files")
            if not isinstance(files, list) or not files:
                _reject("invalid-history", "历史检查字节清单为空")
            seen = set()
            for file in files:
                _artifact(base, file)
                if file["relative_path"] in seen:
                    _reject("invalid-history", "历史检查产物重复")
                seen.add(file["relative_path"])


def _project(raw, base, project_id):
    envelope = _json(raw)
    if (not isinstance(envelope, dict) or set(envelope) != {"schema", "payload", "sha256"}
            or envelope["schema"] != ENVELOPE or not isinstance(envelope["payload"], dict)
            or envelope["sha256"] != digest(envelope["payload"])):
        _reject("invalid-project", "工程快照封装或校验和无效")
    project = envelope["payload"]
    fields = {"schema", "project_id", "name", "revision", "draft_epoch", "created_at", "updated_at",
              "source_snapshot", "content_hash", "context", "intents", "timeline", "cursor", "actions",
              "journal", "candidate", "candidate_history", "validation", "validation_history", "capabilities"}
    if set(project) != fields or project["schema"] != PROJECT_SCHEMA or project["project_id"] != project_id:
        _reject("invalid-project", "工程类型、身份或字段不匹配")
    for key in ("revision", "draft_epoch", "cursor"):
        if type(project[key]) is not int or not 0 <= project[key] < 2**53 - 1:
            _reject("invalid-project", "工程版本或游标无效")
    for key in ("intents", "timeline", "journal", "candidate_history", "validation_history"):
        if not isinstance(project[key], list):
            _reject("invalid-project", "工程列表字段无效")
    if (not isinstance(project["actions"], dict) or not isinstance(project["capabilities"], dict)
            or not all(isinstance(project[k], str) and project[k] for k in ("name", "created_at", "updated_at"))
            or project["cursor"] > len(project["timeline"])
            or project["intents"] != project["timeline"][:project["cursor"]]):
        _reject("invalid-project", "工程历史或当前意图不一致")
    validate_snapshot(project["source_snapshot"])
    # ProjectStore.create leaves compiler/policy unbound until editing is
    # enabled; such a project can have no capability or candidate yet.
    never_bound = (project["context"] == context_for(project["source_snapshot"]) and not project["capabilities"]
                   and project["candidate"] is None and not project["candidate_history"])
    if not never_bound:
        validate_context(project["context"])
    if (project["content_hash"] != content_hash(project["source_snapshot"], project["intents"])
            or project["context"].get("source_hash") != digest({k: v for k, v in project["source_snapshot"].items() if k != "locator"})):
        _reject("invalid-project", "工程草稿或来源身份不一致")
    seen = set()
    for intent in project["timeline"]:
        if not isinstance(intent, dict):
            _reject("invalid-history", "工程命令历史无效")
        command = identifier(intent.get("command_id"))
        if command in seen:
            _reject("invalid-history", "工程命令历史 ID 重复")
        seen.add(command)
    if any(not isinstance(row, dict) for row in project["journal"]):
        _reject("invalid-history", "工程日志无效")
    candidates = [*project["candidate_history"], *([project["candidate"]] if project["candidate"] is not None else [])]
    for candidate in candidates:
        _candidate(base, candidate, project)
    for validation in [*project["validation_history"], *([project["validation"]] if project["validation"] is not None else [])]:
        _validation(base, validation)
        if not any(validation["candidate_hash"] == digest(candidate)
                   and validation["candidate_id"] == candidate["candidate_id"]
                   and validation["context"] == candidate["context"] for candidate in candidates):
            _reject("invalid-history", "历史检查没有对应的已保留候选")
    return project


def _archives(workspace, project_id):
    folder = _safe(workspace / ".jobs")
    if not folder.exists():
        return {}, {}
    if not folder.is_dir():
        _reject("invalid-job-archive", "任务回执目录无效，无法确定工程归属")
    inventory = _inventory(folder)
    if any(not re.fullmatch(r"(?:[0-9a-f]{32}|request-[0-9a-f]{64})\.json", name) for name in inventory):
        _reject("invalid-job-archive", "任务目录含无法确认归属的文件")
    reader = object.__new__(JobManager)
    reader.archive_dir = folder
    selected, records, claims, raw_files = {}, {}, {}, {}
    for name in inventory:
        raw = _read(folder / name)
        raw_files[name] = raw
        envelope = _json(raw)
        if (not isinstance(envelope, dict) or set(envelope) != {"schema", "payload", "sha256"}
                or envelope["schema"] != ARCHIVE_SCHEMA or not isinstance(envelope["payload"], dict)
                or envelope["sha256"] != digest(envelope["payload"])):
            _reject("invalid-job-archive", "任务回执损坏，无法可靠确定归属")
        record = envelope["payload"]
        if name.startswith("request-"):
            if (set(record) != {"job_id", "project_id", "input_hash", "request_key"}
                    or any(not isinstance(record[k], str) or not _ID.fullmatch(record[k])
                           for k in ("job_id", "project_id"))
                    or any(not isinstance(record[k], str) or not re.fullmatch(r"[0-9a-f]{64}", record[k])
                           for k in ("input_hash", "request_key"))
                    or record["request_key"] != name[len("request-"):-len(".json")]):
                _reject("invalid-job-index", "请求索引字段或文件身份无效")
            claims[record["request_key"]] = record
            continue
        try:
            reader._read_archive(name[:-5])
        except (OSError, ValueError, TypeError, KeyError) as exc:
            _reject("invalid-job-archive", "任务回执不完整: " + str(exc))
        if not isinstance(record.get("project_id"), str) or not _ID.fullmatch(record["project_id"]):
            _reject("invalid-job-archive", "任务回执工程身份无效")
        records[record["job_id"]] = record
    # The claim has no operation field: its exact input_hash links it to the
    # validated job's operation/input identity. No request payload is invented
    # or recovered from a filename, and no old request becomes executable.
    seen_keys = set()
    for job_id, record in records.items():
        key = record["request_key"]
        if key in seen_keys:
            _reject("conflicting-job-index", "多个任务回执声称同一请求身份")
        seen_keys.add(key)
        claim = claims.get(key)
        expected = {k: record[k] for k in ("job_id", "project_id", "input_hash", "request_key")}
        if claim != expected:
            _reject("missing-or-conflicting-job-index", "任务缺少完整匹配的请求索引")
        if record["project_id"] == project_id:
            if record["state"] not in _TERMINAL:
                _reject("project-job-active", "本工程仍有未确认终止的任务，迁移不会重启或终止其 PID")
            if not isinstance(record.get("finished_at"), (int, float)) or isinstance(record.get("finished_at"), bool):
                _reject("invalid-job-archive", "本工程终态任务缺少完成时间")
            for name in (job_id + ".json", "request-" + key + ".json"):
                selected[name] = raw_files[name]
    if set(claims) != seen_keys:
        _reject("orphaned-job-index", "存在没有对应完整任务回执的孤立请求索引")
    return inventory, selected


def _fresh_snapshot(source, profile, original):
    kwargs = {}
    frame = original.get("frame", {})
    if frame.get("explicit_evidence"):
        kwargs["frame"] = {"kind": frame["kind"], "coordinate_unit": frame["coordinate_unit"],
                           "evidence": frame["explicit_evidence"]}
    snapshot = SourceCatalog(source, profile).snapshot(original["junction_id"], **kwargs)
    if canonical_bytes({k: v for k, v in snapshot.items() if k != "locator"}) != canonical_bytes(
            {k: v for k, v in original.items() if k != "locator"}):
        _reject("source-semantics-mismatch", "新位置原件、对象、框架或来源语义与原工程不一致")
    if verify_source_snapshot(original, source, profile).get("matches") is not True:
        _reject("source-byte-mismatch", "新位置原件或 Profile 字节与原工程不一致")
    return snapshot


def _publish(pending, final):
    _safe(final.parent)
    if final.exists() or final.is_symlink():
        _reject("destination-conflict", "目标工程已存在，不覆盖")
    os.rename(pending, final)


def relocate_project(project_directory, destination_workspace, source_dir, profile_path):
    """Publish a byte-preserving project copy whose old results require recomputation.

    The destination workspace must not exist. Failures after staging starts
    leave a hidden, unopenable pending directory with ``failure.json``. No old
    job is restored into the new active job archive. Source Store locking is
    cooperative; whole-tree and source-byte checks also detect observed drift.
    """
    pending = None
    try:
        origin, destination = _safe(project_directory), _safe(destination_workspace)
        source, profile = _safe(source_dir), _safe(profile_path)
        if not origin.is_dir() or not _ID.fullmatch(origin.name):
            _reject("invalid-project-directory", "源工程目录必须为有效的 32 位工程 ID")
        if destination.exists():
            _reject("destination-exists", "目标 workspace 必须全新，不合并已有目录")
        for path in (origin.parent, source, profile):
            if destination.is_relative_to(path) or path.is_relative_to(destination):
                _reject("nested-destination", "目标 workspace 不得与任何输入互相包含")
        _safe(origin.parent / ".writer.lock")
        source_store = ProjectStore(origin.parent)
        with source_store._locked():
            original_tree = _inventory(origin)
            original_dirs = _directories(origin)
            if any(name.split("/")[0] in {".jobs", ".writer.lock"} or re.fullmatch(r"session-[0-9]+\.json", name)
                   for name in original_tree):
                _reject("unexpected-session-file", "工程目录混入活动会话、任务或锁文件，不能迁移")
            original_raw = _read(origin / "project.json")
            project = _project(original_raw, origin, origin.name)
            for name in original_tree:
                if name == "previous.json" or (name.startswith("relocation-history/") and name.endswith("/project.original.json")):
                    historic = _project(_read(origin / name), origin, origin.name)
                    if historic["revision"] > project["revision"]:
                        _reject("invalid-history", "历史工程版本晚于当前工程")
            source_tree = _inventory(source, max_file_bytes=MAX_TOTAL_BYTES)
            source_dirs = _directories(source)
            profile_raw = _read(profile)
            live_snapshot = _fresh_snapshot(source, profile, project["source_snapshot"])
            jobs_inventory, jobs = _archives(origin.parent, origin.name)
            implementation_sha = _sha(_read(Path(__file__)))
            relocation_id = uuid4().hex
            destination.mkdir(parents=True, exist_ok=False)
            pending = destination / (".relocation-pending-" + relocation_id)
            pending.mkdir()
            staged = pending / origin.name
            _copy_tree(origin, staged, original_tree)
            if _directories(staged) != original_dirs:
                _reject("copy-mismatch", "工程副本目录集合不完整")
            history = staged / "relocation-history" / relocation_id
            history.mkdir(parents=True, exist_ok=False)
            _write_new(history / "project.original.json", original_raw)
            if jobs:
                (history / "jobs").mkdir()
                for name, raw in jobs.items():
                    _write_new(history / "jobs" / name, raw)
            updated = copy.deepcopy(project)
            updated["source_snapshot"]["locator"] = copy.deepcopy(live_snapshot["locator"])
            updated["revision"] += 1
            updated["draft_epoch"] += 1
            updated["capabilities"] = {}
            updated["updated_at"] = datetime.now(timezone.utc).isoformat()
            updated["journal"].append({"kind": "relocation", "relocation_id": relocation_id,
                "revision": updated["revision"], "draft_epoch": updated["draft_epoch"],
                "content_hash": updated["content_hash"], "at": updated["updated_at"],
                "from_revision": project["revision"], "from_draft_epoch": project["draft_epoch"],
                "recompute_required": True})
            current_raw = canonical_bytes({"schema": ENVELOPE, "payload": updated, "sha256": digest(updated)})
            _project(current_raw, staged, origin.name)
            # Only this staged current snapshot is replaced; its exact original
            # has already been closed and saved in relocation-history.
            with (staged / "project.json").open("wb") as stream:
                stream.write(current_raw)
                stream.flush()
                os.fsync(stream.fileno())
            view = source_store._view(updated)
            if updated["candidate"] and not view["status"]["candidate_stale"]:
                _reject("stale-invariant", "迁移未使旧候选过期")
            if updated["validation"] and not view["status"]["validation_stale"]:
                _reject("stale-invariant", "迁移未使旧检查过期")
            if (_inventory(origin) != original_tree or _directories(origin) != original_dirs
                    or _inventory(source, max_file_bytes=MAX_TOTAL_BYTES) != source_tree or _directories(source) != source_dirs
                    or _read(profile) != profile_raw or _archives(origin.parent, origin.name) != (jobs_inventory, jobs)
                    or _sha(_read(Path(__file__))) != implementation_sha):
                _reject("input-drift", "迁移期间工程、原件、回执或实现发生变化")
            _fresh_snapshot(source, profile, project["source_snapshot"])
            receipt = {"schema": SCHEMA, "status": "RELOCATED_RECOMPUTE_REQUIRED", "relocation_id": relocation_id,
                "project_id": origin.name, "source_project_directory": str(origin),
                "destination_workspace": str(destination), "project_directory": str(destination / origin.name),
                "original_project_sha256": _sha(original_raw), "current_project_sha256": _sha(current_raw),
                "original_files": original_tree, "source_files": source_tree, "profile_sha256": _sha(profile_raw),
                "historical_jobs": {name: _sha(raw) for name, raw in jobs.items()},
                "implementation_sha256": implementation_sha, "source_semantics_unchanged": True,
                "old_evidence_rewritten": False, "recompute_required": True,
                "candidate_accepted": False, "formal_delivery": False,
                "history_directory": "relocation-history/" + relocation_id}
            _write_new(history / "receipt.json", canonical_bytes(receipt))
            expected = {**original_tree, "project.json": {"size": len(current_raw), "sha256": _sha(current_raw)},
                (history / "project.original.json").relative_to(staged).as_posix(): {"size": len(original_raw), "sha256": _sha(original_raw)},
                (history / "receipt.json").relative_to(staged).as_posix(): {"size": len(canonical_bytes(receipt)), "sha256": digest(receipt)}}
            for name, raw in jobs.items():
                expected[(history / "jobs" / name).relative_to(staged).as_posix()] = {"size": len(raw), "sha256": _sha(raw)}
            if _inventory(staged) != expected:
                _reject("copy-drift", "发布前工程副本字节发生变化")
            if (_inventory(origin) != original_tree or _directories(origin) != original_dirs
                    or _inventory(source, max_file_bytes=MAX_TOTAL_BYTES) != source_tree or _directories(source) != source_dirs
                    or _read(profile) != profile_raw or _archives(origin.parent, origin.name) != (jobs_inventory, jobs)
                    or _sha(_read(Path(__file__))) != implementation_sha):
                _reject("input-drift", "发布前工程、原件、回执或实现发生变化")
            # Last filesystem operation that publishes a usable project.
            _publish(staged, destination / origin.name)
            return {"status": receipt["status"], "project_id": origin.name,
                    "destination_workspace": str(destination), "project_directory": str(destination / origin.name),
                    "relocation_id": relocation_id, "receipt": receipt, "project": view,
                    "accepted": False, "formal_delivery": False}
    except Exception as exc:
        error = exc if isinstance(exc, RelocationRejected) else RelocationRejected(
            "relocation-unavailable", f"{type(exc).__name__}: {exc}")
        if pending is not None and pending.is_dir():
            error.pending_directory = str(pending)
            try:
                _write_new(pending / "failure.json", canonical_bytes({"schema": SCHEMA, "status": "FAILED",
                    "code": error.code, "message": str(error), "published": False}))
            except (OSError, ValueError):
                pass
        raise error from exc
