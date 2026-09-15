# -*- coding: utf-8 -*-
"""金凤固定 7 路口 × SHP/MAP 两管道的 14 个 OpenDRIVE 样本生成。"""
from __future__ import annotations

import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml, parse_map_xml_all  # noqa: E402
from mapforge.ops.map_to_xodr import build_xodr                                   # noqa: E402
from mapforge.ops.shp_to_xodr import build_junction_xodr                          # noqa: E402
from mapforge.report.decision import finalize_opendrive_g8                         # noqa: E402
from mapforge.validate.g8_model import write_json                                  # noqa: E402

CASES = [
    ("node3", "map含金路-金玥路node3.xml"),
    ("node4", "map凤苑路-金玥路node4.xml"),
    ("NODE5", "map凤苑路-金坪路NODE5.xml"),
    ("node13", "map凤阁路-金玥路node13.xml"),
    ("node16", "map凤阁路-金剑路路口node16.xml"),
    ("node17", "map含金路-金剑路路口node17.xml"),
    ("node18", "map凤苑路-金剑路node18.xml"),
]
POLICY = ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _aggregate_hash(paths) -> str:
    h = hashlib.sha256()
    for path in sorted((Path(x) for x in paths), key=lambda p: p.as_posix()):
        h.update(path.name.encode("utf-8"))
        h.update(_sha256(path).encode("ascii"))
    return h.hexdigest()


def shp_source():
    from mapforge.adapters.shp.profile_source import ProfileSource
    try:
        return ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    except FileNotFoundError:
        from mapforge.adapters.shp.ibd_reader import IbdSource
        return IbdSource(str(ROOT / "shp_0222-0326"))


def _validate_inputs():
    expected = {ROOT / "v2x_map_xml" / name for _label, name in CASES}
    actual = set((ROOT / "v2x_map_xml").glob("map*.xml"))
    missing, extra = sorted(expected - actual), sorted(actual - expected)
    if missing or extra:
        raise RuntimeError(f"金凤固定样本集不一致：missing={[x.name for x in missing]} "
                           f"extra={[x.name for x in extra]}")
    if not POLICY.exists():
        raise RuntimeError(f"缺 G8 policy: {POLICY}")
    return [ROOT / "v2x_map_xml" / name for _label, name in CASES]


def _clear_expected_outputs():
    suffixes = (".xodr", ".source-lanes.json", ".g8.json",
                ".quality-report.json", ".delivery-decision.json")
    for folder in (ROOT / "out/m2x", ROOT / "out/direct_xodr"):
        folder.mkdir(parents=True, exist_ok=True)
        for label, _name in CASES:
            for suffix in suffixes:
                (folder / f"{label}{suffix}").unlink(missing_ok=True)


def _entry(label, pipeline, source, target, stats, final, input_hash):
    return {
        "case": label,
        "pipeline": pipeline,
        "source": str(source),
        "source_hash": input_hash,
        "artifact": str(target.relative_to(ROOT)),
        "artifact_sha256": _sha256(target),
        "policy_sha256": final["gate"].get("policy", {}).get("sha256"),
        "gate_status": final["gate"]["status"],
        "delivery_status": final["decision"]["status"],
        "sidecars": {k: str(Path(v).relative_to(ROOT)) for k, v in final["files"].items()},
        "stats": {k: v for k, v in stats.items() if k != "source_lane_manifest"},
    }


def main(report_only=False):
    xmls = _validate_inputs()
    _clear_expected_outputs()
    entries, errors = [], []
    xml_hash = _aggregate_hash(xmls)

    nodes = []
    for path in xmls:
        nodes.extend(parse_map_xml_all(str(path)))
    for (label, _name), path in zip(CASES, xmls):
        try:
            main_node = parse_map_xml(str(path))
            neigh = [n for n in nodes
                     if (n.region, n.node_id) != (main_node.region, main_node.node_id)]
            out = ROOT / "out/m2x" / f"{label}.xodr"
            stats = build_xodr(main_node, out, neighbors=neigh)
            final = finalize_opendrive_g8(
                out, stats["source_lane_manifest"], POLICY, connect_mode="data", raw_map_paths=xmls)
            entries.append(_entry(label, "map-to-opendrive", path, out, stats, final, xml_hash))
            print(f"m2x {label:7s} conn={stats['connections']} real={stats.get('exit_real', 0)} "
                  f"mirror={stats.get('exit_mirror', 0)} G8={final['gate']['status']}")
        except Exception as exc:
            errors.append({"case": label, "pipeline": "map-to-opendrive",
                           "error": f"{type(exc).__name__}: {exc}"})
            print(f"m2x {label:7s} FAIL {errors[-1]['error']}")

    src = shp_source()
    shp_files = list((ROOT / "shp_0222-0326").glob("*.*"))
    shp_hash = _aggregate_hash(shp_files)
    for (label, _name), path in zip(CASES, xmls):
        try:
            ref = parse_map_xml(str(path))
            junc, dist = src.find_junction(ref.ref_lon, ref.ref_lat)
            if junc is None or dist > 50:
                raise RuntimeError(f"无对应 SHP 路口，dist={dist:.1f}m")
            out = ROOT / "out/direct_xodr" / f"{label}.xodr"
            stats = build_junction_xodr(src, junc, out)
            final = finalize_opendrive_g8(
                out, stats["source_lane_manifest"], POLICY, connect_mode="data")
            entries.append(_entry(label, "shp-to-opendrive", ROOT / "shp_0222-0326",
                                  out, stats, final, shp_hash))
            print(f"shp {label:7s} enter={stats['roads_enter']} leave={stats['roads_leave']} "
                  f"conn={stats['connections']} G8={final['gate']['status']}")
        except Exception as exc:
            errors.append({"case": label, "pipeline": "shp-to-opendrive",
                           "error": f"{type(exc).__name__}: {exc}"})
            print(f"shp {label:7s} FAIL {errors[-1]['error']}")

    expected = {(label, pipeline) for label, _name in CASES
                for pipeline in ("map-to-opendrive", "shp-to-opendrive")}
    actual = {(x["case"], x["pipeline"]) for x in entries}
    if actual != expected:
        errors.append({"error": f"输出矩阵不完整: missing={sorted(expected - actual)}"})
    index = {
        "schema": "mapforge/closed-loop-inputs/v1",
        "suite": "jinfeng-opendrive-14",
        "report_only": bool(report_only),
        "entries": sorted(entries, key=lambda x: (x["pipeline"], x["case"])),
        "errors": errors,
    }
    write_json(ROOT / "out/closed-loop-inputs.json", index)
    if errors:
        return 2
    if report_only:
        print("REPORT ONLY：允许 draft policy 生成校准指标，不代表 G8 PASS")
        return 0
    return 0 if all(x["gate_status"] == "PASS" for x in entries) else 1


if __name__ == "__main__":
    sys.exit(main(report_only="--report-only" in sys.argv))
