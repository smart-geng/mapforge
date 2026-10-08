"""Replay a read-only local-shape research comparison into a new directory.

This entry point edits no geometry, runs no optimizer and grants no source,
whole-map, editor or product acceptance. Run from the pinned Python environment.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
EXIT_CODES = {"NO_REGRESSION_FOUND": 0, "REGRESSION": 1, "UNAVAILABLE": 2}
LIMITATION = "仅为冻结合同下的局部形状比较；未验收来源保真、整图门禁或产品交付，未授予编辑能力。"


class ReplayError(ValueError):
    def __init__(self, code, detail):
        super().__init__(detail)
        self.code = code


def _strict_json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ReplayError("duplicate-json-key", "JSON 重复键：" + key)
            result[key] = value
        return result

    def finite_float(value):
        number = float(value)
        if not math.isfinite(number):
            raise ReplayError("nonfinite-json-number", "JSON 含非有限数字")
        return number

    def constant(value):
        raise ReplayError("nonfinite-json-number", "JSON 非法常量：" + value)

    value = json.loads(data.decode("utf-8"), object_pairs_hook=pairs,
                       parse_float=finite_float, parse_constant=constant)
    if not isinstance(value, dict):
        raise ReplayError("invalid-contract-object", "合同必须是 JSON 对象")
    return value


def _metadata(path, data):
    return {"path": str(path), "size": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}


def _unavailable(code, detail):
    return {"schema": "mapforge/local-shape-comparison/v1", "status": "UNAVAILABLE",
            "issues": [{"code": code, "reason": detail}], "curves": [],
            "formal_delivery": False, "editing_capability_granted": False,
            "source_fidelity_evaluated": False, "whole_map_gates_replaced": False,
            "limitations": [LIMITATION]}


def _paths(args):
    inputs = {key: Path(getattr(args, key)).expanduser().resolve()
              for key in ("baseline", "candidate", "contract")}
    requested_out = Path(args.out).expanduser().absolute()
    # lexists also rejects dangling symlinks; resolve follows junction aliases.
    if os.path.lexists(requested_out):
        raise ReplayError("output-already-exists", "输出目录必须全新，已存在路径不写入")
    output = requested_out.resolve()
    if os.path.lexists(output):
        raise ReplayError("output-already-exists", "解析后的输出路径已经存在")
    for key, source in inputs.items():
        if source.is_relative_to(output) or output.is_relative_to(source):
            raise ReplayError("input-output-path-overlap", "输入与输出路径互包含：" + key)
        if source.exists() and not source.is_file():
            raise ReplayError("input-not-file", "输入不是普通文件：" + key)
    return inputs, output


def _load_bound_module(name, path, data):
    # Execute the captured source bytes, bypassing mtime-based .pyc reuse.
    # Relative imports in compare resolve to the similarly bound math module.
    module = types.ModuleType(name)
    module.__file__ = str(path)
    module.__package__ = name.rpartition(".")[0]
    sys.modules[name] = module
    exec(compile(data, str(path), "exec"), module.__dict__)
    return module


def _publish_no_replace(pending, final):
    """Publish a closed report as the last commit point, without replacement."""
    if os.name == "nt":
        # Windows rename is atomic in one directory and refuses an existing target.
        os.rename(pending, final)
    else:
        # POSIX rename replaces an existing target; an exclusive hard link does not.
        os.link(pending, final)
        try:
            pending.unlink()
        except OSError:
            # The complete final report is committed. A leftover private staging
            # link does not turn a completed publication into a failed run.
            pass


def run(args):
    try:
        paths, output = _paths(args)
        output.mkdir(parents=True, exist_ok=False)
    except Exception as exc:
        return _unavailable(getattr(exc, "code", "output-reservation-failed"), str(exc)), None

    snapshots = {}
    inputs = {}
    code = {}
    code_executed = False
    runtime = {"python": platform.python_version(), "python_full": sys.version,
               "python_executable": sys.executable, "implementation": platform.python_implementation()}
    try:
        for key, path in paths.items():
            data = path.read_bytes()
            snapshots[path] = data
            inputs[key] = _metadata(path, data)
        code_paths = {
            "local_shape": ROOT / "mapforge/workbench/local_shape.py",
            "local_shape_math": ROOT / "mapforge/workbench/local_shape_math.py",
            "entrypoint": Path(__file__).resolve(),
        }
        for key, path in code_paths.items():
            data = path.read_bytes()
            snapshots[path] = data
            code[key] = _metadata(path, data)
        runtime.update(numpy=importlib.metadata.version("numpy"),
                       pyclothoids=importlib.metadata.version("pyclothoids"))
        contract = _strict_json(snapshots[paths["contract"]])
        sys.path.insert(0, str(ROOT))
        sys.dont_write_bytecode = True
        _load_bound_module("mapforge.workbench.local_shape_math", code_paths["local_shape_math"],
                           snapshots[code_paths["local_shape_math"]])
        module = _load_bound_module("mapforge.workbench.local_shape", code_paths["local_shape"],
                                    snapshots[code_paths["local_shape"]])
        code_executed = True
        result = module.compare(snapshots[paths["baseline"]], snapshots[paths["candidate"]], contract)
        if not isinstance(result, dict) or result.get("status") not in EXIT_CODES:
            raise ReplayError("invalid-comparison-result", "分析器未返回支持的比较状态")
        # Validate finite output before publishing any evidence file.
        json.dumps(result, ensure_ascii=False, allow_nan=False)
    except Exception as exc:
        result = _unavailable(getattr(exc, "code", "replay-unavailable"), str(exc))

    changed = []
    for path, original in snapshots.items():
        try:
            if path.read_bytes() != original:
                changed.append(str(path))
        except OSError:
            changed.append(str(path))
    if changed:
        result = _unavailable("input-or-code-byte-drift", "输入或分析代码在检查期间变化")
    result["replay"] = {
        "schema": "mapforge/local-shape-replay/v1", "inputs": inputs, "code": code,
        "runtime": runtime, "captured_source_bytes_executed": code_executed,
        "bound_files_unchanged": not changed, "changed_files": changed,
        "input_binding_complete": len(inputs) == 3, "code_binding_complete": len(code) == 3,
        "verification": "byte equality immediately before evidence serialization; mtime not used",
        "research_only": True, "candidate_generated": False,
    }
    status = result["status"]
    summary = ("局部形状研究检查：" + status + "\n" + LIMITATION + "\n"
               + "输入与代码字节绑定：" + ("发生变化，结果不可用" if changed else
                    "完整且未发现变化" if len(inputs) == 3 and len(code) == 3 else "不完整，结果不可用") + "\n"
               + "问题数量：" + str(len(result.get("issues", []))) + "\n")
    try:
        encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False).encode("utf-8") + b"\n"
        # A visible report.json is the commit point. A failure before this point
        # leaves no report that could be mistaken for a complete replay.
        with (output / "summary.txt").open("xb") as handle:
            handle.write(summary.encode("utf-8"))
            handle.flush()
            os.fsync(handle.fileno())
        pending = output / "report.json.pending"
        with pending.open("xb") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        _publish_no_replace(pending, output / "report.json")
    except Exception as exc:
        # Do not link a pre-existing/racing report owned by another actor.
        return _unavailable("evidence-publication-failed", str(exc)), None
    return result, output


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    parser = argparse.ArgumentParser(description="只读重放局部形状研究检查；不编辑候选，不替代来源/整图/产品验收。")
    parser.add_argument("--baseline", required=True, help="原始 OpenDRIVE 文件")
    parser.add_argument("--candidate", required=True, help="待比较 OpenDRIVE 文件，只读")
    parser.add_argument("--contract", required=True, help="预声明的严格 JSON 合同")
    parser.add_argument("--out", required=True, help="全新、不与输入互包含的输出目录")
    args = parser.parse_args(argv)
    result, output = run(args)
    status = result["status"]
    terminal = {"status": status, "exit_code": EXIT_CODES[status],
                "report": str(output / "report.json") if output and (output / "report.json").is_file() else None,
                "formal_delivery": False, "note": LIMITATION}
    if status == "UNAVAILABLE":
        terminal["reason"] = result.get("issues", [])[:1]
    print(json.dumps(terminal, ensure_ascii=False, allow_nan=False))
    return EXIT_CODES[status]


if __name__ == "__main__":
    raise SystemExit(main())
