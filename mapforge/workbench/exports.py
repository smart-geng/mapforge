"""Atomic research packages. This module never grants production delivery."""
from __future__ import annotations

import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
from uuid import uuid4
import zipfile

from .contracts import StoreConflict, StoreReadOnly, StoreValidation, canonical_bytes, digest
from .jobs import ARCHIVE_SCHEMA
from .sources import verify_source_snapshot
from .validation import verify_validation_artifacts


def _sha(data):
    return hashlib.sha256(data).hexdigest()


class ResearchExportService:
    """Export a checked, current snapshot with explicit research-only markings.

    ZIP and receipt appear together by renaming a complete staging directory.
    A project change during preparation rejects publication. Downloads check the
    current binding again and return the already verified immutable byte buffer.
    """

    def __init__(self, store, *, source_dir, profile_path):
        self.store = store
        self.source_dir, self.profile_path = Path(source_dir), Path(profile_path)

    @staticmethod
    def _identity(project):
        return {key: project[key] for key in
                ("project_id", "revision", "draft_epoch", "content_hash", "context", "candidate", "validation")}

    def _ready(self, project_id, revision=None):
        project = self.store.load(project_id)
        if revision is not None:
            if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
                raise StoreValidation("base_revision 必须为非负整数")
            if project["revision"] != revision:
                raise StoreConflict("工程版本已变化，请重开后重新导出")
        status = project["status"]
        if status["read_only"]:
            raise StoreReadOnly("工程只读，不能导出当前研究候选")
        if (not project.get("candidate") or not project.get("validation")
                or status["candidate_stale"] or status["validation_stale"]):
            raise StoreConflict("请先接受当前候选并保存与它绑定的完整检查结果")
        verify_validation_artifacts(self.store.project_path(project_id), project["validation"])
        self._record_bytes(project)
        if not verify_source_snapshot(project["source_snapshot"], self.source_dir, self.profile_path)["matches"]:
            raise StoreReadOnly("源文件或 Profile 实际字节已改变，不能导出旧结果")
        return project

    def _record_bytes(self, project):
        """Bind the post-check result file to the successful task's sealed record.

        The result contains the validation object itself, so it cannot include
        its own hash in that object's artifact list. The independently archived
        worker result supplies this missing byte binding.
        """
        bundle = project["validation"]["bundle_id"]
        path = self.store.project_path(project["project_id"]) / "artifacts" / bundle / "validation-result.json"
        data = path.read_bytes()
        archives = self.store.root / ".jobs"
        if archives.is_symlink():
            raise StoreValidation("检查回执目录不得为链接")
        for archive in sorted(archives.glob("*.json")):
            if not re.fullmatch(r"[0-9a-f]{32}\.json", archive.name) or archive.is_symlink():
                continue
            try:
                archived_data = archive.read_bytes()
                envelope = json.loads(archived_data)
                if not isinstance(envelope, dict) or envelope.get("schema") != ARCHIVE_SCHEMA:
                    continue
                record = envelope["payload"]
                if not isinstance(record, dict) or envelope.get("sha256") != digest(record):
                    continue
                result = record.get("result")
                if not isinstance(result, dict) or not isinstance(result.get("report"), dict):
                    continue
                revision = record.get("base_revision")
                if (isinstance(revision, bool) or not isinstance(revision, int) or revision < 0
                        or revision != result["report"].get("target_revision")):
                    continue
                if (record.get("project_id") != project["project_id"] or record.get("operation") != "validate"
                        or record.get("job_id") != archive.stem or record.get("content_hash") != project["content_hash"]
                        or record.get("state") != "succeeded"
                        or result.get("validation") != project["validation"]):
                    continue
                if canonical_bytes(record["result"]) != data:
                    raise StoreValidation("检查结果文件与当时后台任务回执的字节不一致")
                receipt = {"job_id": record["job_id"], "archive_sha256": _sha(archived_data),
                           "validation_result_sha256": _sha(data), "purpose": "RESEARCH_ONLY"}
                return data, canonical_bytes(receipt)
            except (KeyError, TypeError, json.JSONDecodeError):
                continue
        raise StoreValidation("缺少完整检查任务回执，请重新检查并保存后再导出")

    def _read_artifact(self, project_id, item):
        if not isinstance(item, dict) or set(item) != {"relative_path", "sha256"}:
            raise StoreValidation("产物清单格式不正确")
        relative = item["relative_path"]
        if (not isinstance(relative, str) or not relative.startswith("artifacts/")
                or "\\" in relative or ":" in relative
                or any(part in ("", ".", "..") for part in relative.split("/"))):
            raise StoreValidation("产物路径必须位于工程内")
        base = self.store.project_path(project_id)
        path = base.joinpath(*PurePosixPath(relative).parts)
        if (not path.resolve().is_relative_to(base)
                or any(p.is_symlink() for p in [path, *path.parents] if p != base.parent)):
            raise StoreValidation("产物路径越界或使用了链接")
        if path.stat().st_size > 64 * 1024 * 1024:
            raise StoreValidation("单个检查产物超出当前导出范围")
        data = path.read_bytes()
        if _sha(data) != item["sha256"]:
            raise StoreValidation("产物实际字节与检查记录不一致")
        return relative, data

    def _folder(self, project_id):
        base = self.store.project_path(project_id)
        folder = base / ".research-exports"
        if folder.is_symlink() or not folder.resolve().is_relative_to(base):
            raise StoreValidation("导出目录越界或使用了链接")
        folder.mkdir(exist_ok=True)
        return folder

    def _package(self, project_id, export_id):
        if not isinstance(export_id, str) or not re.fullmatch(r"research-[0-9a-f]{32}", export_id):
            raise StoreValidation("无效的研究包 ID")
        folder = self._folder(project_id) / export_id
        if folder.is_symlink():
            raise StoreValidation("研究包不得为链接")
        receipt_path, archive = folder / "receipt.json", folder / "research.zip"
        if receipt_path.is_symlink() or archive.is_symlink():
            raise StoreValidation("研究包文件不得为链接")
        try:
            envelope = json.loads(receipt_path.read_text(encoding="utf8"))
            receipt = envelope["payload"]
            data = archive.read_bytes()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise StoreValidation("研究包尚未完整保存或已损坏") from exc
        if (envelope.get("sha256") != digest(receipt)
                or receipt.get("export_id") != export_id or receipt.get("project_id") != project_id
                or receipt.get("sha256") != _sha(data) or receipt.get("size_bytes") != len(data)
                or receipt.get("purpose") != "RESEARCH_ONLY" or receipt.get("formal_delivery") is not False):
            raise StoreValidation("研究包回读校验失败")
        return receipt, data

    @staticmethod
    def _zip(files):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for name, data in sorted(files.items()):
                info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                archive.writestr(info, data)
        return stream.getvalue()

    def create(self, project_id, base_revision):
        if base_revision is None:
            raise StoreValidation("必须提供 base_revision")
        project = self._ready(project_id, base_revision)
        identity = self._identity(project)
        implementation = _sha(Path(__file__).read_bytes())
        export_id = "research-" + digest({"identity": identity, "exporter": implementation})[:32]
        files = {}
        items = list(project["candidate"]["artifacts"])
        binding = next(c for c in project["validation"]["checks"] if c.get("gate") == "validation-byte-binding")
        items.extend(binding["files"])
        for item in items:
            relative, data = self._read_artifact(project_id, item)
            if relative in files and files[relative] != data:
                raise StoreValidation("研究包存在同名冲突产物")
            files[relative] = data
        bundle_id = project["validation"]["bundle_id"]
        record_data, check_receipt = self._record_bytes(project)
        relative, record_data = self._read_artifact(project_id, {
            "relative_path": f"artifacts/{bundle_id}/validation-result.json", "sha256": _sha(record_data)})
        files[relative] = record_data
        files["research-check-receipt.json"] = check_receipt
        capsule = {**identity, "name": project["name"], "source_snapshot": project["source_snapshot"],
                   "intents": project["intents"], "purpose": "RESEARCH_ONLY", "formal_delivery": False}
        files["research-project.json"] = canonical_bytes(capsule)
        files["README-研究用途.txt"] = (
            "MapForge 研究候选包\n\n仅用于研究、复核和消费端兼容性试验，不是正式交付包。\n"
            f"工程：{project['name']}\n修订：{project['revision']}\n"
            f"原始检查裁决：{project['validation']['decision']}\n"
            "请读取 research-manifest.json 与完整检查报告；不要删除或忽略阻断项。\n"
            "原始数据不随包复制；源快照保留原记录和文件哈希，工程重建仍需相同源数据。\n"
        ).encode("utf8")
        manifest = {"schema": "mapforge/research-package/v1", "export_id": export_id,
                    "purpose": "RESEARCH_ONLY", "formal_delivery": False, "project_id": project_id,
                    "revision": project["revision"], "draft_epoch": project["draft_epoch"],
                    "content_hash": project["content_hash"], "context": project["context"],
                    "candidate": project["candidate"], "validation": project["validation"],
                    "exporter_sha256": implementation,
                    "files": [{"path": name, "sha256": _sha(data), "size_bytes": len(data)}
                              for name, data in sorted(files.items())]}
        files["research-manifest.json"] = canonical_bytes(manifest)
        archive = self._zip(files)
        receipt = {"export_id": export_id, "project_id": project_id, "purpose": "RESEARCH_ONLY",
                   "formal_delivery": False, "revision": project["revision"],
                   "identity_hash": digest(identity), "sha256": _sha(archive), "size_bytes": len(archive),
                   "file_name": f"{export_id}-RESEARCH_ONLY.zip", "decision": project["validation"]["decision"]}
        # Recheck external files and implementation after assembling the bytes.
        if (self._identity(self._ready(project_id, base_revision)) != identity
                or _sha(Path(__file__).read_bytes()) != implementation):
            raise StoreConflict("打包期间工程或实现发生变化，请重新导出")
        folder = self._folder(project_id)
        stage = folder / ("." + export_id + "." + uuid4().hex + ".tmp")
        stage.mkdir()
        try:
            for name, data in (("research.zip", archive), ("receipt.json", canonical_bytes({"payload": receipt, "sha256": digest(receipt)}))):
                with (stage / name).open("xb") as stream:
                    stream.write(data); stream.flush(); os.fsync(stream.fileno())
            # Serialize final publication with project writers. Use the store's
            # internal read while locked rather than recursively acquiring its OS lock.
            with self.store._locked():
                current = self.store._view(*self.store._read(project_id))
                if current["status"]["read_only"] or self._identity(current) != identity:
                    raise StoreConflict("打包期间工程已变化，未发布研究包")
                target = folder / export_id
                if target.exists():
                    previous, data = self._package(project_id, export_id)
                    if previous != receipt or data != archive:
                        raise StoreValidation("已有同身份研究包的字节不同")
                else:
                    os.rename(stage, target)
            stored, data = self._package(project_id, export_id)
            if data != archive or stored != receipt:
                raise StoreValidation("研究包发布后回读不一致")
            return receipt
        finally:
            # Only this newly-created staging directory and its two named files.
            if stage.exists():
                for name in ("research.zip", "receipt.json"):
                    (stage / name).unlink(missing_ok=True)
                stage.rmdir()

    def download(self, project_id, export_id):
        project = self._ready(project_id)
        receipt, data = self._package(project_id, export_id)
        identity = self._identity(project)
        if receipt["identity_hash"] != digest(identity):
            raise StoreConflict("该包对应较早工程版本，请重新导出当前研究候选")
        if self._identity(self._ready(project_id)) != identity:
            raise StoreConflict("读取研究包期间工程发生变化，请重新导出")
        with self.store._locked():
            current = self.store._view(*self.store._read(project_id))
            if current["status"]["read_only"] or self._identity(current) != identity:
                raise StoreConflict("研究包返回前工程发生变化，请重新导出")
        return receipt, data
