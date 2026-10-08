"""Session-owned whole-candidate checking and explicit report attachment.

Checks never rewrite a candidate or open delivery. The stored report is bound
to actual files, the accepted candidate, draft epoch, and validator version.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path, PurePosixPath
import threading

from . import compiler, validation
from .contracts import (StoreConflict, StoreReadOnly, StoreValidation, content_hash,
                        context_for, digest, identifier)
from .sources import verify_source_snapshot


class CheckingService:
    validation_module = validation

    def __init__(self, store, jobs, *, source_dir=None, profile_path=None, baseline_path=None):
        self.store, self.jobs = store, jobs
        self.source_dir = Path(source_dir or compiler.ROOT / "shp_0222-0326").resolve()
        self.profile_path = Path(profile_path or compiler.ROOT / "profiles/shp/ibd-smarteditor-v1.yaml").resolve()
        self.baseline_path = Path(baseline_path or compiler.DEFAULT_BASELINE).resolve()
        self._proofs = {}
        self._lock = threading.RLock()

    @staticmethod
    def _revision(project, revision, *, retry_command_id=None):
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise StoreValidation("base_revision 必须为非负整数")
        if project["revision"] != revision and retry_command_id not in project.get("actions", {}):
            raise StoreConflict("工程已经更新，请刷新后重新检查")
        if project["status"]["read_only"]:
            raise StoreReadOnly("工程只读，不能启动或附加检查")

    @staticmethod
    def _current_candidate(project, fingerprints):
        candidate = project.get("candidate")
        if (not isinstance(candidate, dict) or project["status"].get("candidate_stale")
                or candidate.get("accepted_epoch") != project["draft_epoch"]
                or candidate.get("target_content_hash") != project["content_hash"]
                or project["content_hash"] != content_hash(project["source_snapshot"], project["intents"])
                or candidate.get("context") != project["context"]):
            raise StoreConflict("检查需要当前草稿已接受且未失效的候选")
        runtime = fingerprints["compiler_policy"]
        if project["context"] != context_for(project["source_snapshot"], runtime["compiler_hash"], runtime["policy_hash"]):
            raise StoreConflict("候选的编译器或策略已经变化，需要重新编译并接受")
        return candidate

    def _candidate_files(self, project):
        """Re-read project-relative bytes; never resolve an HTTP-supplied path."""
        directory = self.store.project_path(project["project_id"])
        artifacts = project["candidate"].get("artifacts")
        if not isinstance(artifacts, list) or not artifacts:
            raise StoreValidation("已接受候选没有产物清单")
        xodr_hashes, paths = [], set()
        for item in artifacts:
            if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
                raise StoreValidation("候选产物合同无效")
            relative = item["relative_path"]
            if (not isinstance(relative, str) or "\\" in relative or ":" in relative
                    or not relative.startswith("artifacts/")
                    or any(part in {"", ".", ".."} for part in relative.split("/"))):
                raise StoreValidation("候选产物路径越出工程")
            path = directory.joinpath(*PurePosixPath(relative).parts)
            if (not path.resolve().is_relative_to(directory)
                    or any(p.is_symlink() for p in [path, *path.parents] if p != directory.parent)):
                raise StoreValidation("候选产物越界或使用链接")
            if path in paths:
                raise StoreValidation("候选产物身份重复")
            paths.add(path)
            try:
                with path.open("rb") as stream:
                    actual = hashlib.file_digest(stream, "sha256").hexdigest()
            except OSError as exc:
                raise StoreValidation("候选实际产物缺失") from exc
            if actual != item["sha256"]:
                raise StoreValidation("候选实际产物哈希不一致")
            if path.suffix == ".xodr":
                xodr_hashes.append(actual)
        if len(xodr_hashes) != 1:
            raise StoreValidation("检查需要唯一实际 XODR 候选")
        return xodr_hashes[0]

    @staticmethod
    def _fingerprints():
        try:
            return validation.validation_fingerprints()
        except (OSError, ValueError, KeyError) as exc:
            raise StoreValidation("检查器或检查资产不可用：" + str(exc)) from exc

    def start(self, project_id, base_revision, request_id):
        project = self.store.load(project_id)
        self._revision(project, base_revision)
        fingerprints = self._fingerprints()
        candidate = self._current_candidate(project, fingerprints)
        candidate_sha = self._candidate_files(project)
        payload = {"project": copy.deepcopy(project), "project_directory": str(self.store.project_path(project_id)),
                   "source_dir": str(self.source_dir), "profile_path": str(self.profile_path),
                   "baseline_path": str(self.baseline_path)}
        proof = {"project_id": project_id, "target_revision": project["revision"],
                 "target_draft_epoch": project["draft_epoch"], "target_content_hash": project["content_hash"],
                 "candidate_hash": digest(candidate), "candidate_id": candidate["candidate_id"],
                 "candidate_sha256": candidate_sha, "context": copy.deepcopy(project["context"]),
                 "fingerprints": fingerprints}
        job = self.jobs.start(project, "validate", payload, request_id)
        with self._lock:
            previous = self._proofs.setdefault(job["job_id"], proof)
            if previous != proof:
                raise StoreConflict("检查任务身份已经绑定另一个输入")
        return {"job": job}

    def attach(self, project_id, base_revision, command_id, job_id):
        identifier(command_id, "command_id")
        identifier(job_id, "job_id")
        project = self.store.load(project_id)
        self._revision(project, base_revision, retry_command_id=command_id)
        job = self.jobs.get(job_id, project_id=project_id)
        with self._lock:
            proof = copy.deepcopy(self._proofs.get(job_id))
        if (not proof or proof["project_id"] != project_id or job.get("historical")
                or job.get("operation") != "validate" or job.get("state") != "succeeded"
                or job.get("archive", {}).get("state") != "persisted"
                or job.get("archive", {}).get("recorded_state") != "succeeded"):
            raise StoreConflict("检查未成功持久保存，已取消，或属于历史会话")
        result = job.get("result", {})
        package = result.get("validation")
        if result.get("status") != "VALIDATED" or not isinstance(package, dict):
            error = result.get("error") or {}
            raise StoreValidation("没有可附加的检查结果：" + str(error.get("message", "检查已拒绝")))
        if set(package) != {"bundle_id", "candidate_id", "candidate_hash", "context", "checks", "decision"}:
            raise StoreValidation("检查结果不符合存储合同")
        if package["decision"] not in {"BLOCKED", "REVIEW"}:
            raise StoreValidation("当前工作台研发检查不能开放正式交付")
        fingerprints = self._fingerprints()
        candidate = self._current_candidate(project, fingerprints)
        if fingerprints != proof["fingerprints"]:
            raise StoreConflict("检查器或策略实现已变化，原检查已失效")
        try:
            report = self.validation_module.verify_validation_artifacts(self.store.project_path(project_id), package)
        except (validation.ValidationRejected, OSError, ValueError, KeyError, TypeError) as exc:
            raise StoreValidation("检查产物核验失败：" + str(exc)) from exc
        if report != result.get("report"):
            raise StoreValidation("持久检查报告与任务结果不一致")
        if (job.get("base_revision") != proof["target_revision"]
                or job.get("content_hash") != proof["target_content_hash"]
                or any(report.get(key) != proof[key] for key in
                       ("project_id", "target_revision", "target_draft_epoch", "target_content_hash",
                        "candidate_hash", "candidate_sha256", "context", "fingerprints"))
                or package["candidate_id"] != proof["candidate_id"]
                or package["candidate_hash"] != proof["candidate_hash"]
                or package["context"] != proof["context"]):
            raise StoreConflict("检查报告不属于登记的工程、候选或目标版本")
        previous = project.get("actions", {}).get(command_id, {})
        same_request = previous.get("request_hash") == digest(
            {"operation": "validation", "base_revision": base_revision, "validation": package})
        if (not same_request and project["revision"] != proof["target_revision"]
                or project["draft_epoch"] != proof["target_draft_epoch"]
                or project["content_hash"] != proof["target_content_hash"]
                or digest(candidate) != proof["candidate_hash"] or project["context"] != proof["context"]):
            raise StoreConflict("当前草稿、候选或版本已经变化；请重新检查")
        if self._candidate_files(project) != proof["candidate_sha256"]:
            raise StoreValidation("候选实际字节与检查目标不一致")
        if not verify_source_snapshot(project["source_snapshot"], self.source_dir, self.profile_path)["matches"]:
            raise StoreReadOnly("源原件或 Profile 实际字节已经变化，检查不得附加")
        # Re-read runtime fingerprints after potentially long source hashing.
        if self._fingerprints() != fingerprints:
            raise StoreConflict("附加检查期间检查器或策略发生变化")
        return self.store.attach_validation(project_id, base_revision, command_id, package)
