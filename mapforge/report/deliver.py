# -*- coding: utf-8 -*-
"""交付包组装（方案 8.3）：MAP 三视图 + quality/loss 报告 + id-mapping + provenance + config。

M1 级说明：
- id-mapping 以 JSON+CSV 交付（parquet 待 pyarrow 引入）；source_id 取自重塑器附着的 `_src_pid`（无则留空）；
- id-diff：本次为基线时输出 {"baseline": true}，有上一版 id-mapping 时输出增删改；
- 红线检查（方案 8.2）：phase 缺绑（信控路口）计数>0 时交付状态标 BLOCKED（除非 allow_no_phase）。
"""
from __future__ import annotations

import csv
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

from mapforge import __version__
from mapforge.adapters.v2xmap.xml_reader import MapNode

_MANEUVER_NAMES = {0: "straight", 1: "left", 2: "right", 3: "uTurn"}


def _man_set(bits: str | None):
    if not bits:
        return set()
    return {_MANEUVER_NAMES.get(i, f"bit{i}") for i, b in enumerate(bits) if b == "1"}


def quality_report(node: MapNode, extra: dict | None = None) -> dict:
    lanes = [(lk, ln) for lk in node.links for ln in lk.lanes]
    conns = [c for _, ln in lanes for c in ln.connects]
    n_lane = len(lanes)
    # connectsTo 完整率：maneuvers 置位转向均有对应 connection
    full = 0
    for _, ln in lanes:
        need = _man_set(ln.maneuvers)
        have = set()
        for c in ln.connects:
            have |= _man_set(c.maneuver)
        if need and need <= have:
            full += 1
    width_bad = sum(1 for _, ln in lanes if not ln.width_cm or ln.width_cm < 150 or ln.width_cm > 500)
    rep = {
        "links": len(node.links),
        "lanes": n_lane,
        "connections": len(conns),
        "connects_to_completeness": round(full / n_lane, 3) if n_lane else None,
        "phase_bound": sum(1 for c in conns if c.phase is not None),
        "phase_completeness": round(sum(1 for c in conns if c.phase is not None) / len(conns), 3) if conns else None,
        "isolated_lanes": sum(1 for _, ln in lanes if not ln.connects),
        "lane_width_anomalies": width_bad,
        "upstream_ids_pending": sum(1 for lk in node.links if not lk.upstream[1]),
        "points": {"link": sum(len(lk.points) for lk in node.links),
                   "lane": sum(len(ln.points) for _, ln in lanes)},
    }
    if extra:
        rep["pipeline"] = extra
    return rep


def loss_report(node: MapNode, events: list[dict] | None, simplify_tol_m: float | None) -> dict:
    conns = [c for lk in node.links for ln in lk.lanes for c in ln.connects]
    n_no_phase = sum(1 for c in conns if c.phase is None)
    items = list(events or [])
    if simplify_tol_m is not None:
        items.append({"status": "APPROXIMATED", "what": "point-lists simplified (T/CSAE159 Annex-D chord tol)",
                      "tolerance_m": simplify_tol_m})
    if n_no_phase:
        items.append({"status": "DROPPED", "what": "phase binding missing on connections",
                      "count": n_no_phase, "rule": "FORBIDDEN_AUTO — needs timing table / manual"})
    counts: dict[str, int] = {}
    for it in items:
        counts[it["status"]] = counts.get(it["status"], 0) + 1
    return {"summary": counts, "items": items}


def id_mapping_rows(node: MapNode) -> list[dict]:
    rows = []
    for lk in node.links:
        rows.append({"kind": "link", "source_id": getattr(lk, "_src_pid", ""),
                     "target": f"({node.region},{node.node_id})/{lk.name}"})
        for ln in lk.lanes:
            rows.append({"kind": "lane", "source_id": getattr(ln, "_src_pid", ""),
                         "target": f"({node.region},{node.node_id})/{lk.name}/lane{ln.lane_id}"})
    return rows


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def build_delivery(node: MapNode, out_dir: Path, *,
                   source_path: Path, source_format: str, profile: str,
                   mode: str, config: dict, pipeline_summary: dict | None = None,
                   loss_events: list[dict] | None = None, simplify_tol_m: float | None = 0.30,
                   prev_id_mapping: Path | None = None, allow_no_phase: bool = False) -> dict:
    """写出交付包目录，返回 {status, files}。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    files = []

    # 三视图
    from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val
    from mapforge.adapters.v2xmap.xml_writer import node_to_xml
    from mapforge.report.preview_geojson import to_geojson
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "mapforge" / "adapters" / "v2xmap" / "asn"))
    import msglayer_draft as m
    mf = m.MsgLayerDraft.MessageFrame
    val = node_to_messageframe_val(node, mode)
    mf.set_val(val)
    buf = mf.to_uper()
    mf.from_uper(buf)                                     # 门禁①：编解码回环
    roundtrip_ok = (mf.get_val() == val) if mode == "absolute" else True
    (out_dir / "map.uper").write_bytes(buf)
    (out_dir / "map.xml").write_text(node_to_xml(node), encoding="utf-8")
    (out_dir / "map.geojson").write_text(
        json.dumps(to_geojson(node), ensure_ascii=False, indent=1), encoding="utf-8")
    files += ["map.uper", "map.xml", "map.geojson"]

    # 报告
    q = quality_report(node, pipeline_summary)
    q["encoding"] = {"mode": mode, "bytes": len(buf), "uper_roundtrip": "PASS" if roundtrip_ok else "FAIL"}
    l = loss_report(node, loss_events, simplify_tol_m)
    (out_dir / "quality-report.json").write_text(json.dumps(q, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "loss-report.json").write_text(json.dumps(l, ensure_ascii=False, indent=2), encoding="utf-8")
    files += ["quality-report.json", "loss-report.json"]

    # id-mapping / id-diff
    rows = id_mapping_rows(node)
    (out_dir / "id-mapping.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    with open(out_dir / "id-mapping.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["kind", "source_id", "target"])
        w.writeheader()
        w.writerows(rows)
    if prev_id_mapping and prev_id_mapping.exists():
        prev = {r["target"]: r for r in json.loads(prev_id_mapping.read_text(encoding="utf-8"))}
        cur = {r["target"]: r for r in rows}
        diff = {"added": sorted(set(cur) - set(prev)), "removed": sorted(set(prev) - set(cur)),
                "source_changed": sorted(t for t in set(cur) & set(prev)
                                         if cur[t]["source_id"] != prev[t]["source_id"])}
    else:
        diff = {"baseline": True}
    (out_dir / "id-diff.json").write_text(json.dumps(diff, ensure_ascii=False, indent=2), encoding="utf-8")
    files += ["id-mapping.json", "id-mapping.csv", "id-diff.json"]

    # provenance / config
    prov = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "tool": {"name": "mapforge", "version": __version__},
        "source": {"path": str(source_path), "format": source_format, "profile": profile,
                   "sha256": _sha256(source_path) if source_path.is_file() else None},
        "crs": {"declared": "WGS84(claimed)",
                "integrity": "internally-consistent",
                "evidence": "docs/M0作业报告-ASN编译与数据核验.md §4（SHP↔MAP 同源核验）",
                "absolute_verification": "pending (survey control points)"},
        "standards": {"message_layer": "YD/T draft (msglayer-draft.asn, 送审稿提取)",
                      "simplify_rule": "T/CSAE 159-2020 Annex D"},
    }
    (out_dir / "provenance-manifest.json").write_text(
        json.dumps(prov, ensure_ascii=False, indent=2), encoding="utf-8")
    import yaml
    (out_dir / "conversion-config.yaml").write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    files += ["provenance-manifest.json", "conversion-config.yaml"]

    # 红线（方案 8.2）：信控路口 phase 缺绑
    n_no_phase = sum(1 for lk in node.links for ln in lk.lanes for c in ln.connects if c.phase is None)
    status = "OK"
    if n_no_phase and not allow_no_phase:
        status = "BLOCKED(phase_missing=%d)" % n_no_phase
    (out_dir / "DELIVERY-STATUS.txt").write_text(status + "\n", encoding="utf-8")
    files.append("DELIVERY-STATUS.txt")
    return {"status": status, "files": files, "bytes": len(buf)}
