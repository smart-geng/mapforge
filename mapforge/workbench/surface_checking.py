"""Session-owned checking service for the current accepted surface candidate."""
from __future__ import annotations

import copy

from . import surface_compiler, surface_validation
from .checking import CheckingService
from .contracts import StoreConflict, StoreValidation, digest


class SurfaceCheckingService(CheckingService):
    validation_module = surface_validation

    @staticmethod
    def _fingerprints():
        try:
            return surface_validation.validation_fingerprints()
        except (OSError, ValueError, KeyError) as exc:
            raise StoreValidation("来源辅助铺面检查实现或资产不可用：" + str(exc)) from exc

    def _candidate_files(self, project):
        try:
            verified = surface_compiler.verify_candidate_artifacts(
                self.store.project_path(project["project_id"]), project["candidate"])
            return surface_validation._sha(surface_validation._read(verified["primary_path"]))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise StoreValidation("来源辅助铺面实际候选核验失败：" + str(exc)) from exc

    def start(self, project_id, base_revision, request_id):
        project = self.store.load(project_id)
        self._revision(project, base_revision)
        fingerprints = self._fingerprints()
        candidate = self._current_candidate(project, fingerprints)
        candidate_sha = self._candidate_files(project)
        payload = {"project": copy.deepcopy(project), "project_directory": str(self.store.project_path(project_id)),
                   "source_dir": str(self.source_dir), "profile_path": str(self.profile_path)}
        proof = {"project_id": project_id, "target_revision": project["revision"],
                 "target_draft_epoch": project["draft_epoch"], "target_content_hash": project["content_hash"],
                 "candidate_hash": digest(candidate), "candidate_id": candidate["candidate_id"],
                 "candidate_sha256": candidate_sha, "context": copy.deepcopy(project["context"]),
                 "fingerprints": fingerprints}
        job = self.jobs.start(project, "validate", payload, request_id)
        with self._lock:
            previous = self._proofs.setdefault(job["job_id"], proof)
            if previous != proof:
                raise StoreConflict("检查任务身份已经绑定另一个来源候选")
        return {"job": job}
