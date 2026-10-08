"""Session-owned editing adapter for fresh-source auxiliary reconstruction.

The shared editing service owns preview/commit/accept ordering. This adapter
changes the registered capability and actual artifact reader, not those rules.
It never substitutes a historical CLI review for this project's fresh result.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path
import threading

from . import surface_compiler as compiler
from .contracts import StoreConflict, StoreReadOnly, StoreValidation, digest
from .editing import EditingService
from .surface_review import RegisteredSurfaceReview


class SurfaceEditingService(EditingService):
    compile_operation = "compile_surface"

    def __init__(self, store, jobs, *, source_dir=None, profile_path=None):
        self.store, self.jobs = store, jobs
        self.source_dir = Path(source_dir or compiler.ROOT / "shp_0222-0326").resolve()
        self.profile_path = Path(profile_path or compiler.ROOT / "profiles/shp/ibd-smarteditor-v1.yaml").resolve()
        self._registrations = {}
        self._registration_snapshots = {}
        self._proofs = {}
        self._lock = threading.RLock()

    def _registration(self, project, *, fresh=False):
        snapshot = project["source_snapshot"]
        key = digest(snapshot)
        with self._lock:
            if fresh or key not in self._registrations:
                try:
                    registration = compiler.register_source(snapshot, source_dir=self.source_dir,
                                                             profile_path=self.profile_path)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    raise StoreValidation(str(exc)) from exc
                self._registrations[key] = registration
                self._registration_snapshots[registration.snapshot_content_hash] = copy.deepcopy(snapshot)
            return self._registrations[key]

    def _enabled(self, project):
        registration = self._registration(project)
        context = registration.context(project["source_snapshot"])
        stored = project.get("capabilities", {}).get(compiler.CAPABILITY_ID)
        spec = registration.store_spec()
        if (project["context"] != context or not isinstance(stored, dict)
                or stored.get("context") != context
                or any(stored.get(k) != v for k, v in spec.items())
                or stored.get("spec_hash") != digest(spec)):
            raise StoreConflict("辅助铺面操作尚未登记或已过期，请重新核验登记")
        return registration

    @staticmethod
    def _surface_intents(project):
        return [row for row in project.get("intents", [])
                if isinstance(row, dict) and row.get("type") == compiler.INTENT_TYPE]

    def describe(self, project):
        count = len(self._surface_intents(project))
        unavailable = {"supported": False, "enabled": False, "capability": None,
                       "command_template": None, "surface_intent_count": count,
                       "can_preview": False, "can_compile": False}
        if project["source_snapshot"].get("junction_id") != compiler.P.JUNCTION_ID:
            return {**unavailable, "reason": "该辅助铺面操作仅登记于 0621 源路口"}
        try:
            registration = self._registration(project)
            capability = registration.capability()
            template = registration.command_template("template")
            template.pop("command_id")
            try:
                self._enabled(project)
                enabled, reason = True, ""
            except (StoreConflict, StoreValidation, ValueError) as exc:
                enabled, reason = False, str(exc)
            if project["status"]["read_only"]:
                enabled, reason = False, "工程处于来源漂移或恢复只读状态"
            elif enabled and count > 1:
                reason = "草稿包含重复辅助铺面操作，请撤销重复操作后重新编译"
            elif enabled and count == 1:
                reason = "已保存该操作；可重新编译当前草稿，不再追加同一操作"
            return {"supported": True, "enabled": enabled, "reason": reason,
                    "capability": capability, "command_template": template,
                    "surface_intent_count": count, "can_preview": enabled and count == 0,
                    "can_compile": enabled and count == 1}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {**unavailable, "reason": str(exc)}

    def _start(self, project, draft, request_id, *, preview=None):
        payload = {"draft": {k: copy.deepcopy(draft[k]) for k in
                              ("source_snapshot", "revision", "draft_epoch", "intents", "content_hash", "context")},
                   "project_directory": str(self.store.project_path(project["project_id"])),
                   "source_dir": str(self.source_dir), "profile_path": str(self.profile_path)}
        # Refuse empty/duplicate or altered operations before creating a job.
        try:
            compiler._validate_draft(payload["draft"], self._enabled(project))
        except (ValueError, KeyError, TypeError) as exc:
            raise StoreValidation(str(exc)) from exc
        proof = {"project_id": project["project_id"], "base_revision": project["revision"],
                 "base_content_hash": project["content_hash"], "base_draft_epoch": project["draft_epoch"],
                 "target_revision": draft["revision"], "target_draft_epoch": draft["draft_epoch"],
                 "target_content_hash": draft["content_hash"], "context": copy.deepcopy(draft["context"]),
                 "preview": copy.deepcopy(preview)}
        job = self.jobs.start(project, self.compile_operation, payload, request_id)
        with self._lock:
            previous = self._proofs.setdefault(job["job_id"], proof)
            if previous != proof:
                raise StoreConflict("任务身份已经绑定其他草稿")
        return job

    def preview(self, project_id, base_revision, command, request_id):
        project = self.store.load(project_id)
        self._revision(project, base_revision)
        if self._surface_intents(project):
            raise StoreConflict("草稿已保存辅助铺面操作，请编译已保存草稿，不要重复追加")
        if not isinstance(command, dict) or command.get("type") != compiler.INTENT_TYPE:
            raise StoreValidation("该入口只预览登记的辅助铺面操作")
        return super().preview(project_id, base_revision, command, request_id)

    def _artifacts(self, project_id, candidate):
        try:
            verified = compiler.verify_candidate_artifacts(self.store.project_path(project_id), candidate)
            data = Path(verified["primary_path"]).read_bytes()
            evidence = verified["evidence"]
            if hashlib.sha256(data).hexdigest() != evidence["candidate_sha256"]:
                raise StoreValidation("实际主候选在读取期间发生变化")
            return data, evidence
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise StoreValidation(str(exc)) from exc

    def _baseline_bytes(self, registration):
        """Shared accept hook: recheck fresh source, never read a complete baseline."""
        with self._lock:
            snapshot = copy.deepcopy(self._registration_snapshots.get(registration.snapshot_content_hash))
        if snapshot is None:
            raise StoreConflict("该会话缺少操作登记的来源快照")
        try:
            current = compiler.register_source(snapshot, source_dir=self.source_dir, profile_path=self.profile_path)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise StoreReadOnly("来源重新核验失败：" + str(exc)) from exc
        if current.store_spec() != registration.store_spec():
            raise StoreConflict("来源或操作登记发生变化")
        return None

    def geometry(self, project_id, job_id=None):
        project = self.store.load(project_id)
        unavailable = {"available": False, "reason": "当前工程没有本次生成的候选",
                       "candidate_id": None, "candidate_stale": False, "candidate_accepted": False,
                       "preview": job_id is not None, "formal_export_available": False}
        if project["status"]["read_only"]:
            return {**unavailable, "reason": "工程只读或来源待复核，未显示旧候选"}
        try:
            registration = self._enabled(project)
            candidate = project.get("candidate")
            result = None
            if job_id is not None:
                result = self._job_result(project, job_id, allow_uncommitted=True)
                candidate = result["candidate"]
            elif candidate and project["status"]["candidate_stale"]:
                return {**unavailable, "reason": "候选已过期，需重新编译当前草稿",
                        "candidate_id": candidate["candidate_id"], "candidate_stale": True}
            if not candidate:
                return unavailable
            verified = compiler.verify_candidate_artifacts(self.store.project_path(project_id), candidate)
            evidence = verified["evidence"]
            if result is not None and evidence != result["evidence"]:
                raise StoreValidation("任务证据与实际候选证据不一致")
            self._baseline_bytes(registration)
            diagnosis = RegisteredSurfaceReview(verified["run_directory"]).diagnose()
            if diagnosis.get("available") is not True:
                raise StoreValidation(diagnosis.get("reason", "实际候选诊断不可用"))
            # Re-read all artifact bindings and live inputs after rendering work.
            after = compiler.verify_candidate_artifacts(self.store.project_path(project_id), candidate)
            if after != verified or self.store.load(project_id) != project:
                raise StoreConflict("诊断期间工程或候选发生变化，请刷新")
            report = copy.deepcopy(diagnosis["report"])
            accepted = bool(job_id is None and project.get("candidate") == candidate
                            and not project["status"]["candidate_stale"]
                            and candidate.get("accepted_epoch") == project["draft_epoch"])
            context = report.setdefault("context", {})
            context.update(project_id=project_id, candidate_id=candidate["candidate_id"],
                target_revision=evidence["target_revision"], target_draft_epoch=evidence["target_draft_epoch"],
                target_content_hash=evidence["target_content_hash"], candidate_accepted=accepted,
                formal_release_verified=False, preview=job_id is not None,
                meaning="本工程预览目标的新生成候选，尚未接纳" if job_id is not None else
                        "本工程已接纳的研究候选；接纳不代表整图通过或正式交付")
            # The immutable diagnostic proof retains candidate_accepted=false;
            # only session/project context describes the explicit Store action.
            return {"available": True, "reason": "实际新生成候选；正式交付仍阻断", "report": report,
                    "candidate_id": candidate["candidate_id"], "candidate_stale": False,
                    "candidate_accepted": accepted, "preview": job_id is not None,
                    "formal_export_available": False}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {**unavailable, "reason": "本次候选不可用：" + str(exc)}
