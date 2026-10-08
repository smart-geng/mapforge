"""Read-only freshness of saved checks in HTTP project views.

This adds display state only. It neither rewrites evidence nor replaces the
artifact verification performed when attaching checks or exporting a bundle.
"""
from __future__ import annotations

import copy
import re


def _current_fingerprints():
    from .validation import validation_fingerprints
    return validation_fingerprints()


def _hash(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def project_view(project: dict) -> dict:
    """Fail closed for absent/old validator bindings, preserving prior staleness."""
    if project.get("validation") is None:
        return project
    result = copy.deepcopy(project)
    status = result.setdefault("status", {})
    reasons = status.setdefault("validation_stale_reasons", [])
    if status.get("validation_stale") and not reasons:
        reasons.append({"code": "project-validation-stale",
                        "message": "工程、候选或来源已变化，原检查仅供历史查看"})
    validation = result["validation"]
    checks = validation.get("checks") if isinstance(validation, dict) else None
    bindings = ([item for item in checks if isinstance(item, dict)
                 and item.get("gate") == "validation-byte-binding"]
                if isinstance(checks, list) else [])
    problem = None
    if (len(bindings) != 1 or bindings[0].get("status") != "PASS"
            or not _hash(bindings[0].get("validator_hash"))):
        problem = {"code": "validator-binding-missing",
                   "message": "原检查缺少有效版本绑定，请重新运行整图检查"}
    else:
        try:
            from .surface_routing import is_surface_project, validation_module
            current = (validation_module(result).validation_fingerprints()
                       if is_surface_project(result) else _current_fingerprints())
            files = current.get("implementation_files_sha256")
            compiler_policy = current.get("compiler_policy")
            if (not _hash(current.get("validator_hash")) or not isinstance(files, dict)
                    or not files or not all(_hash(value) for value in files.values())
                    or not isinstance(compiler_policy, dict)
                    or not all(_hash(compiler_policy.get(key))
                               for key in ("compiler_hash", "policy_hash"))):
                raise ValueError("Incomplete live validator fingerprint")
        except Exception:
            problem = {"code": "validator-fingerprint-unavailable",
                       "message": "当前检查版本无法核验，原检查仅供历史查看"}
        else:
            if bindings[0]["validator_hash"] != current["validator_hash"]:
                problem = {"code": "validator-stale",
                           "message": "检查版本已更新，请重新运行整图检查"}
            elif (not isinstance(result.get("context"), dict)
                  or any(result["context"].get(key) != compiler_policy[key]
                         for key in ("compiler_hash", "policy_hash"))):
                problem = {"code": "compiler-policy-stale",
                           "message": "编译器或检查策略已更新，原检查仅供历史查看"}
    if problem:
        status["validation_stale"] = True
        if problem not in reasons:
            reasons.append(problem)
    return result


def project_response(value: dict) -> dict:
    """Decorate a full project or the editing-enable response containing one."""
    if "project" in value:
        return {**value, "project": project_view(value["project"])}
    return project_view(value)
