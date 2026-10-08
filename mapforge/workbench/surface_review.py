"""Server-registered, read-only surface diagnostics for frozen research runs.

A registration is a local launch argument, never an HTTP-supplied path. Its
historical score remains historical; this service cannot accept a candidate,
change a draft, or authorize delivery. Live source identity and every captured
artifact are checked before and after each diagnosis.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath

from lxml import etree
from scripts.workbench_run_source_tracks import REQUIRED_OUTPUTS

from .contracts import digest
from .sources import verify_source_snapshot

REQUIRED = frozenset((*REQUIRED_OUTPUTS, "runner-child.stdout.txt", "runner-child.stderr.txt"))
MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_BUNDLE_BYTES = 128 * 1024 * 1024


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _json(data):
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError("诊断记录包含重复字段")
            value[key] = item
        return value
    def invalid(value):
        raise ValueError("诊断记录包含非有限数字")
    return json.loads(data, object_pairs_hook=pairs, parse_constant=invalid)


def _snapshot_hash(snapshot):
    return digest({k: v for k, v in snapshot.items() if k not in {"locator", "content_hash"}})


def _local_file(directory, name):
    if (not isinstance(name, str) or not name or "\\" in name or ":" in name
            or PurePosixPath(name).is_absolute()
            or any(part in {"", ".", ".."} for part in name.split("/"))):
        raise ValueError("诊断登记包含越界路径")
    path = directory.joinpath(*name.split("/"))
    if (not path.resolve().is_relative_to(directory) or not path.is_file()
            or any(p.is_symlink() for p in (path, *path.parents))):
        raise ValueError("诊断文件缺失、越界或使用链接")
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError("诊断文件过大")
    return path


def _frame(root):
    refs = root.findall("header/geoReference")
    if len(refs) != 1 or root.find("header/offset") is not None:
        raise ValueError("诊断缺少唯一局部坐标框架或存在额外偏移")
    values = {}
    for token in (refs[0].text or "").split():
        if not token.startswith("+"):
            raise ValueError("诊断局部坐标声明不支持")
        key, _, value = token[1:].partition("=")
        if key in values:
            raise ValueError("诊断坐标声明重复")
        values[key] = value
    if values.get("proj") != "eqc" or values.get("units") != "m":
        raise ValueError("诊断目前仅支持显式米制 eqc 局部框架")
    lon, lat = float(values["lon_0"]), float(values["lat_0"])
    if not (math.isfinite(lon) and math.isfinite(lat) and -180 <= lon <= 180 and -90 <= lat <= 90):
        raise ValueError("诊断坐标原点无效")
    return {"kind": "local-eqc", "unit": "m", "origin": [lon, lat],
            "declaration": refs[0].text, "absolute_crs_status": "unverified",
            "meaning": "候选 XML 的局部坐标声明，仅用于诊断显示；未核验绝对位置"}


def _diagnose(root, evidence):
    from .surface_diagnostics import diagnose_surface
    return diagnose_surface(root, evidence)


class RegisteredSurfaceReview:
    def __init__(self, directory):
        original = Path(directory)
        if any(p.is_symlink() for p in (original, *original.parents)):
            raise ValueError("诊断目录不得使用链接")
        self.directory = original.resolve(strict=True)
        receipt_data = _local_file(self.directory, "runner-result.json").read_bytes()
        receipt = _json(receipt_data)
        if (receipt.get("schema") != "mapforge/wb11-source-tracks-runner/v1"
                or receipt.get("status") != "EXPERIMENT_COMPLETE"
                or receipt.get("report_status") != "EXPERIMENT_EVALUATED"
                or type(receipt.get("exit_code")) is not int or receipt["exit_code"] != 0
                or receipt.get("experiment_complete") is not True
                or receipt.get("candidate_accepted") is not False
                or receipt.get("formal_release_verified") is not False
                or receipt.get("problems") != []):
            raise ValueError("仅能登记证据完整且明确未接受的研究运行")
        bindings = receipt.get("evidence_bindings")
        if not isinstance(bindings, dict) or not REQUIRED <= bindings.keys() or len(bindings) > 128:
            raise ValueError("诊断运行证据清单不完整")
        self.bindings = copy.deepcopy(bindings)
        self.bindings["runner-result.json"] = {"sha256": _sha(receipt_data), "size": len(receipt_data)}
        data = self.read_bound()
        snapshot = _json(data["source-snapshot.json"])
        if (snapshot.get("schema") != "mapforge/workbench-source/v1"
                or snapshot.get("content_hash") != _snapshot_hash(snapshot)):
            raise ValueError("诊断源快照内容不匹配")
        self.snapshot = snapshot
        evidence = _json(data["surface-evidence.json"])
        intent = _json(data["confirmed-probe-intent.json"])
        report = _json(data["report.json"])
        if (intent != evidence.get("intent")
                or intent.get("source_snapshot_id") != snapshot.get("snapshot_id")
                or intent.get("source_content_hash") != snapshot["content_hash"]
                or intent.get("junction_id") != snapshot.get("junction_id")
                or intent.get("confirmed") is not True
                or report.get("candidate_sha256") != _sha(data["candidate.xodr"])
                or report.get("source_snapshot_id") != snapshot.get("snapshot_id")
                or report.get("status") != "EXPERIMENT_EVALUATED"
                or report.get("candidate_accepted") is not False
                or report.get("formal_release_verified") is not False):
            raise ValueError("诊断来源、意图和实际候选绑定不一致")
        board = _json(data["scoreboard.json"])
        rows = board.get("rows")
        if (board.get("schema") != "mapforge/scoreboard/v1" or board.get("files") != 1
                or not isinstance(rows, list) or len(rows) != 1
                or rows[0].get("case") != snapshot.get("junction_id")
                or rows[0].get("artifact") != "candidate.xodr"
                or rows[0].get("pipeline") != "shp"
                or rows[0].get("tiers") != report.get("tiers")
                or set(rows[0].get("tiers", {})) != {"T1", "T2"}
                or board.get("tier_pass") != {tier: int(rows[0]["tiers"][tier].get("status") == "PASS")
                                               for tier in ("T1", "T2")}):
            raise ValueError("历史评分与候选、来源或运行结论不一致")
        decision = _json(data["candidate.delivery-decision.json"])
        if (decision.get("schema") != "mapforge/delivery-decision/v1"
                or decision.get("status") not in {"BLOCKED", "REVIEW_REQUIRED", "DELIVERABLE"}
                or decision != report.get("delivery")):
            raise ValueError("历史交付裁决与运行记录不一致")

    def read_bound(self):
        data, total = {}, 0
        for name, binding in self.bindings.items():
            raw = _local_file(self.directory, name).read_bytes()
            total += len(raw)
            if (total > MAX_BUNDLE_BYTES or not isinstance(binding, dict)
                    or type(binding.get("size")) is not int or binding["size"] != len(raw)
                    or binding.get("sha256") != _sha(raw)):
                raise ValueError("已登记的诊断证据字节发生变化或不完整")
            data[name] = raw
        return data

    def matches(self, snapshot):
        return (snapshot.get("snapshot_id") == self.snapshot["snapshot_id"]
                and snapshot.get("content_hash") == self.snapshot["content_hash"]
                and _snapshot_hash(snapshot) == self.snapshot["content_hash"])

    def diagnose(self):
        data = self.read_bound()
        parser = etree.XMLParser(resolve_entities=False, no_network=True, load_dtd=False)
        root = etree.fromstring(data["candidate.xodr"], parser)
        if root.getroottree().docinfo.doctype or root.tag != "OpenDRIVE":
            raise ValueError("诊断 XML 类型不支持")
        frame = _frame(root)
        result = _diagnose(root, _json(data["surface-evidence.json"]))
        self.read_bound()
        if result.get("status") != "DIAGNOSED":
            return {"available": False, "reason": result.get("reason", "当前表达无法完成严格分区诊断")}
        proof = result.get("source_proof", {})
        if (proof.get("snapshot_id") != self.snapshot["snapshot_id"]
                or proof.get("content_hash") != self.snapshot["content_hash"]):
            raise ValueError("实际诊断图层不属于登记源快照")
        board = _json(data["scoreboard.json"])
        decision = _json(data["candidate.delivery-decision.json"])
        result["context"] = {"candidate_sha256": _sha(data["candidate.xodr"]),
            "source_snapshot_id": self.snapshot["snapshot_id"],
            "source_content_hash": self.snapshot["content_hash"], "frame": frame,
            "candidate_accepted": False, "formal_release_verified": False,
            "historical_score_tiers": [r.get("tiers", {}) for r in board.get("rows", [])],
            "historical_delivery_decision": decision.get("status"),
            "meaning": "外部研究运行的只读诊断，不是当前草稿或已接受候选；原评分与裁决保持"}
        return {"available": True, "reason": "研究候选未接受；正式交付仍阻断", "report": result}


class SurfaceReviewService:
    def __init__(self, catalog, directories=()):
        self.catalog = catalog
        self.registrations = [RegisteredSurfaceReview(p) for p in directories]
        keys = [(r.snapshot["snapshot_id"], r.snapshot["content_hash"]) for r in self.registrations]
        if len(set(keys)) != len(keys):
            raise ValueError("同一来源登记了多个研究诊断，请明确选择一份")

    def describe(self, project):
        snapshot = project["source_snapshot"]
        try:
            matches = [r for r in self.registrations if r.matches(snapshot)]
            if not matches:
                return {"available": False, "reason": "该工程没有登记匹配来源的铺面诊断"}
            if project.get("status", {}).get("read_only"):
                return {"available": False, "reason": "工程当前只读或来源待复核，未显示诊断图层"}
            self.catalog.assert_unchanged()
            verification = verify_source_snapshot(snapshot, self.catalog.source_dir, self.catalog.profile_path)
            if verification.get("matches") is not True:
                return {"available": False, "reason": "来源已变化，未显示旧诊断"}
            result = matches[0].diagnose()
            self.catalog.assert_unchanged()
            if verify_source_snapshot(snapshot, self.catalog.source_dir, self.catalog.profile_path).get("matches") is not True:
                return {"available": False, "reason": "诊断期间来源发生变化，结果未显示"}
            return result
        except (ValueError, KeyError, TypeError, OSError, etree.Error) as exc:
            return {"available": False, "reason": "诊断不可用：" + str(exc)}
