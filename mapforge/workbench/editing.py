"""Server-owned editing sessions for the single registered WB05 capability.

Preview jobs do not write the draft. Their results can be accepted only after
the exact previewed command is committed, at the predicted revision and epoch.
The plotting adapter samples actual immutable XODR bytes; sampling is display
data, never a replacement for the written analytic road geometry.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path, PurePosixPath
import threading

from . import compiler
from .contracts import StoreConflict, StoreReadOnly, StoreValidation, digest, identifier
from .sources import verify_source_snapshot


def _sha(data):
    return hashlib.sha256(data).hexdigest()


class EditingService:
    def __init__(self, store, jobs, *, source_dir=None, profile_path=None, baseline_path=None):
        self.store, self.jobs = store, jobs
        # Only application configuration supplies paths. A project locator is
        # provenance, never authority to open a different client-chosen file.
        self.source_dir = Path(source_dir or compiler.ROOT / "shp_0222-0326").resolve()
        self.profile_path = Path(profile_path or compiler.ROOT / "profiles/shp/ibd-smarteditor-v1.yaml").resolve()
        self.baseline_path = Path(baseline_path or compiler.DEFAULT_BASELINE).resolve()
        self._registrations = {}
        self._proofs = {}
        self._lock = threading.RLock()

    def _registration(self, project, *, fresh=False):
        key = digest(project["source_snapshot"])
        with self._lock:
            if fresh or key not in self._registrations:
                try:
                    self._registrations[key] = compiler.register_verified_baseline(
                        project["source_snapshot"], baseline_path=self.baseline_path,
                        source_dir=self.source_dir, profile_path=self.profile_path)
                except compiler.CompilerRejected as exc:
                    raise StoreValidation(str(exc)) from exc
            return self._registrations[key]

    @staticmethod
    def _revision(project, revision, *, retry_command_id=None):
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise StoreValidation("base_revision 必须为非负整数")
        if project["revision"] != revision and retry_command_id not in project.get("actions", {}):
            raise StoreConflict("工程已更新，请刷新后重试")
        if project["status"]["read_only"]:
            raise StoreReadOnly("工程只读，不能提交修形或接受候选")

    def _enabled(self, project):
        registration = self._registration(project)
        context = registration.context(project["source_snapshot"])
        stored = project.get("capabilities", {}).get(compiler.CAPABILITY_ID)
        spec = registration.store_spec()
        if (project["context"] != context or not stored or stored.get("context") != context
                or any(stored.get(k) != v for k, v in spec.items())):
            raise StoreConflict("修形能力尚未登记或已过期，请先重新核验登记")
        return registration

    def describe(self, project):
        """Read-only and cached; do not rehash the entire source on each UI poll."""
        if project["source_snapshot"].get("junction_id") != compiler.JUNCTION_ID:
            return {"supported": False, "enabled": False,
                    "reason": "当前仅登记凤阁路-金剑路 road 12 的一条共享边界；该工程尚未开放修形", "capability": None}
        try:
            registration = self._registration(project)
            capability = registration.capability()
            try:
                self._enabled(project)
                enabled, reason = True, ""
            except (StoreConflict, StoreValidation, compiler.CompilerRejected) as exc:
                enabled, reason = False, str(exc)
            if project["status"]["read_only"]:
                enabled, reason = False, "工程处于源漂移或恢复只读状态"
            return {"supported": True, "enabled": enabled, "reason": reason, "capability": capability}
        except (StoreValidation, compiler.CompilerRejected, OSError, ValueError) as exc:
            return {"supported": False, "enabled": False, "reason": str(exc), "capability": None}

    def enable(self, project_id, base_revision, command_id):
        identifier(command_id, "command_id")
        project = self.store.load(project_id)
        self._revision(project, base_revision, retry_command_id=command_id)
        registration = self._registration(project, fresh=True)
        context = registration.context(project["source_snapshot"])
        result = self.store.register_capabilities(project_id, base_revision, command_id,
                                                   [registration.store_spec()],
                                                   context["compiler_hash"], context["policy_hash"])
        return {"project": result, "capability": registration.capability()}

    def _start(self, project, draft, request_id, *, preview=None):
        payload = {"draft": {k: copy.deepcopy(draft[k]) for k in
                              ("source_snapshot", "revision", "draft_epoch", "intents", "content_hash", "context")},
                   "project_directory": str(self.store.project_path(project["project_id"])),
                   "baseline_path": str(self.baseline_path), "source_dir": str(self.source_dir),
                   "profile_path": str(self.profile_path)}
        proof = {"project_id": project["project_id"], "base_revision": project["revision"],
                 "base_content_hash": project["content_hash"], "base_draft_epoch": project["draft_epoch"],
                 "target_revision": draft["revision"], "target_draft_epoch": draft["draft_epoch"],
                 "target_content_hash": draft["content_hash"], "context": copy.deepcopy(draft["context"]),
                 "preview": copy.deepcopy(preview)}
        job = self.jobs.start(project, "compile", payload, request_id)
        with self._lock:
            previous = self._proofs.setdefault(job["job_id"], proof)
            if previous != proof:
                raise StoreConflict("任务身份已经绑定其他草稿")
        return job

    def preview(self, project_id, base_revision, command, request_id):
        project = self.store.load(project_id)
        self._revision(project, base_revision)
        self._enabled(project)
        preview = self.store.preview(project_id, base_revision, command)
        # A store mutation between load and preview cannot be used to predict
        # a different epoch: preview enforces the same base revision.
        draft = copy.deepcopy(project)
        draft["intents"].append(preview["intent"])
        draft["revision"] += 1
        draft["draft_epoch"] += 1
        draft["content_hash"] = preview["target_content_hash"]
        proof = {"command": preview["intent"], "base_revision": base_revision,
                 "target_revision": draft["revision"], "target_draft_epoch": draft["draft_epoch"],
                 "target_content_hash": draft["content_hash"]}
        return {"job": self._start(project, draft, request_id, preview=proof), "preview": proof}

    def compile(self, project_id, base_revision, request_id):
        project = self.store.load(project_id)
        self._revision(project, base_revision)
        self._enabled(project)
        return {"job": self._start(project, project, request_id)}

    def _job_result(self, project, job_id, *, allow_uncommitted=False, retry=None):
        identifier(job_id, "job_id")
        job = self.jobs.get(job_id, project_id=project["project_id"])
        with self._lock:
            proof = copy.deepcopy(self._proofs.get(job_id))
        if (not proof or proof["project_id"] != project["project_id"] or job.get("historical")
                or job.get("operation") != "compile" or job.get("state") != "succeeded"
                or job.get("archive", {}).get("state") != "persisted"
                or job.get("archive", {}).get("recorded_state") != "succeeded"):
            raise StoreConflict("任务尚未成功持久保存，已取消，或属于历史会话；不能使用结果")
        result = job.get("result", {})
        if result.get("status") != "COMPILED" or not isinstance(result.get("candidate"), dict):
            error = result.get("error") or {}
            raise StoreValidation("编译未生成可接受候选：" + str(error.get("message", "已拒绝")))
        evidence, candidate = result.get("evidence", {}), result["candidate"]
        if (job.get("base_revision") != proof["base_revision"]
                or job.get("content_hash") != proof["base_content_hash"]
                or any(evidence.get(k) != proof[k] for k in
                       ("target_revision", "target_draft_epoch", "target_content_hash", "context"))
                or candidate.get("target_content_hash") != proof["target_content_hash"]
                or candidate.get("context") != proof["context"]
                or candidate.get("candidate_id") != evidence.get("candidate_id")):
            raise StoreConflict("任务结果与服务器登记的目标不一致")
        before_commit = bool(allow_uncommitted and proof["preview"]
                             and project["revision"] == proof["base_revision"]
                             and project["content_hash"] == proof["base_content_hash"]
                             and project["draft_epoch"] == proof["base_draft_epoch"])
        accepted_retry = False
        if retry:
            previous = project.get("actions", {}).get(retry["command_id"], {})
            accepted_retry = previous.get("request_hash") == digest(
                {"operation": "candidate", "base_revision": retry["base_revision"], "candidate": candidate})
        if not before_commit and not accepted_retry:
            if (project["revision"] != proof["target_revision"]
                    or project["draft_epoch"] != proof["target_draft_epoch"]
                    or project["content_hash"] != proof["target_content_hash"]):
                raise StoreConflict("草稿版本或内容已变化，或预览命令尚未确认保存")
            if proof["preview"]:
                intent = proof["preview"]["command"]
                action = project.get("actions", {}).get(intent["command_id"])
                last = project.get("journal", [])[-1:]
                if (not action or action["revision"] != proof["target_revision"] or not last
                        or last[0].get("command_id") != intent["command_id"] or last[0].get("kind") != "commit"
                        or not project["intents"] or project["intents"][-1] != intent):
                    raise StoreConflict("预览对应的唯一命令没有在预期版本确认")
        if project["context"] != proof["context"]:
            raise StoreConflict("编译上下文已变化")
        return result

    def _artifacts(self, project_id, candidate):
        values = {}
        base = self.store.project_path(project_id)
        items = candidate.get("artifacts")
        if not isinstance(items, list) or not items:
            raise StoreValidation("候选没有产物清单")
        for item in items:
            if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
                raise StoreValidation("候选产物清单格式错误")
            relative = item["relative_path"]
            if (not isinstance(relative, str) or "\\" in relative or ":" in relative
                    or not relative.startswith("artifacts/")
                    or any(p in {"", ".", ".."} for p in relative.split("/"))):
                raise StoreValidation("候选产物路径越界")
            path = base.joinpath(*PurePosixPath(relative).parts)
            if (not path.resolve().is_relative_to(base)
                    or any(p.is_symlink() for p in [path, *path.parents] if p != base.parent)):
                raise StoreValidation("候选产物路径越界或为链接")
            try:
                data = path.read_bytes()
            except OSError as exc:
                raise StoreValidation("候选产物缺失") from exc
            if _sha(data) != item["sha256"]:
                raise StoreValidation("候选产物实际哈希不一致")
            if path.name in values:
                raise StoreValidation("候选产物存在重名")
            values[path.name] = data
        if set(values) != {"candidate.xodr", "evidence.json"}:
            raise StoreValidation("候选产物不符合当前编译器合同")
        try:
            evidence = json.loads(values["evidence.json"])
        except (ValueError, TypeError) as exc:
            raise StoreValidation("候选证据无法解析") from exc
        if (not isinstance(evidence, dict) or evidence.get("candidate_id") != candidate["candidate_id"]
                or evidence.get("candidate_sha256") != _sha(values["candidate.xodr"])
                or evidence.get("target_content_hash") != candidate["target_content_hash"]
                or evidence.get("context") != candidate["context"]):
            raise StoreValidation("候选证据与实际产物或输入不一致")
        return values["candidate.xodr"], evidence

    def accept(self, project_id, base_revision, command_id, job_id):
        identifier(command_id, "command_id")
        project = self.store.load(project_id)
        self._revision(project, base_revision, retry_command_id=command_id)
        registration = self._enabled(project)
        result = self._job_result(project, job_id, retry={"command_id": command_id, "base_revision": base_revision})
        _, evidence = self._artifacts(project_id, result["candidate"])
        if evidence != result["evidence"]:
            raise StoreValidation("任务证据与实际证据文件不一致")
        # Full byte verification prevents same-size/same-mtime source changes
        # from passing acceptance through the lightweight UI source guard.
        if not verify_source_snapshot(project["source_snapshot"], self.source_dir, self.profile_path)["matches"]:
            raise StoreReadOnly("源文件或 Profile 实际字节已经变化")
        self._baseline_bytes(registration)
        if registration.context(project["source_snapshot"]) != project["context"]:
            raise StoreConflict("编译器或策略指纹已变化")
        return self.store.accept_candidate(project_id, base_revision, command_id, result["candidate"])

    def _baseline_bytes(self, registration):
        try:
            data = self.baseline_path.read_bytes()
            manifest = self.baseline_path.with_suffix(".source-lanes.json").read_bytes()
        except OSError as exc:
            raise StoreValidation("登记基线或来源文件缺失") from exc
        if (_sha(data) != compiler.BASELINE_SHA256 or _sha(manifest) != compiler.SOURCE_MANIFEST_SHA256
                or data != registration.prepared.data):
            raise StoreValidation("登记基线或来源文件发生变化")
        return data

    @staticmethod
    def _lines(data, registration):
        from mapforge.repair_web.model import parse
        from scripts.internal_edge_jets import states
        road = parse(data).find("road[@id='12']")
        if road is None:
            raise StoreValidation("候选中缺少已登记道路")
        lo, hi = registration.prepared.knots[0], registration.prepared.knots[-1]
        lo, hi = max(0., lo - 6.), min(float(road.get("length")), hi + 6.)
        # Include exact polynomial cuts in addition to a bounded display grid.
        count = max(2, int((hi - lo) / .25) + 1)
        samples = {lo + (hi - lo) * i / count for i in range(count + 1)}
        samples.update(registration.prepared.knots)
        for section in road.findall("lanes/laneSection"):
            start = float(section.get("s"))
            samples.add(start)
            samples.update(start + float(w.get("sOffset")) for w in section.findall(".//width"))
        stations = sorted(s for s in samples if lo <= s <= hi)
        lines = [{"id": "fixed_inner", "label": "车道 -1 内边缘", "points": []},
                 {"id": "shared", "label": "车道 -1/-2 共享边界", "points": []},
                 {"id": "fixed_outer", "label": "车道 -2 外边缘", "points": []}]
        for s in stations:
            inner = states(road, -1, s, False)
            outer = states(road, -2, s, False)
            for line, point in zip(lines, (inner[0], inner[1], outer[1])):
                line["points"].append([float(point[0]), float(point[1])])
        return {"lines": lines, "stations_m": stations, "sha256": _sha(data)}

    def geometry(self, project_id, job_id=None):
        project = self.store.load(project_id)
        registration = self._registration(project)
        baseline = self._baseline_bytes(registration)
        candidate, data, evidence = project.get("candidate"), None, None
        if job_id is not None:
            self._enabled(project)
            result = self._job_result(project, job_id, allow_uncommitted=True)
            candidate = result["candidate"]
        if candidate:
            data, evidence = self._artifacts(project_id, candidate)
            if job_id is not None and evidence != result["evidence"]:
                raise StoreValidation("任务证据与实际证据文件不一致")
        binding = json.loads(registration.prepared.binding_json)
        source = {"lines": [{"id": key, "label": "原始源边界：" + key, "points": points}
                             for key, points in binding["local_polylines"].items()]}
        coordinate = json.loads(registration.coordinate_evidence_json)
        return {"baseline": self._lines(baseline, registration),
                "candidate": self._lines(data, registration) if data else None, "source": source,
                "coordinate_space": "existing-xodr-local-metres", "coordinate_evidence": coordinate,
                "scope_s_m": [registration.prepared.knots[0], registration.prepared.knots[-1]],
                "candidate_id": candidate["candidate_id"] if candidate else None,
                "candidate_stale": bool(candidate and not job_id and project["status"]["candidate_stale"]),
                "normal_delta_m": evidence.get("normal_delta_m") if evidence else None,
                "local_checks": evidence.get("local_checks") if evidence else None,
                "whole_map_validation": "NOT_RUN", "formal_export_available": False}
