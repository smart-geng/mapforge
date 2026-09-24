# -*- coding: utf-8 -*-
"""M0 作业②③：金凤 7 路口 MAP XML 解码 → GeoJSON 预览 + 存量 ID 台账清洗。

产出：
- out/geojson/<node>.geojson         每路口预览（refPos/Link 中线/Lane 中线，属性含 phase/maneuvers）
- ledger/jinfeng-2026.draft.yaml     台账草案（决策文件契约：decision 字段留空待人工裁决）
- out/id_cleanup_report.md           ID 交叉核对报告
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.report.preview_geojson import to_geojson  # noqa: F401  已入包，re-export 兼容旧 import

SRC = ROOT / "v2x_map_xml"
OUT = ROOT / "out"
LEDGER = ROOT / "ledger"


def main():
    (OUT / "geojson").mkdir(parents=True, exist_ok=True)
    LEDGER.mkdir(exist_ok=True)
    nodes = []
    for f in sorted(SRC.glob("map*.xml")):
        n = parse_map_xml(str(f))
        nodes.append((f.name, n))
        gj = to_geojson(n)
        out = OUT / "geojson" / (f.stem + ".geojson")
        out.write_text(json.dumps(gj, ensure_ascii=False, indent=1), encoding="utf-8")

    declared = {}
    for fname, n in nodes:
        declared[(n.region, n.node_id)] = fname

    # 收集引用：upstreamNodeId + movements.remote + connectsTo.remote
    refs = {}
    for fname, n in nodes:
        for lk in n.links:
            refs.setdefault(tuple(lk.upstream), set()).add((fname, "upstream", lk.name))
            for m in lk.movements:
                refs.setdefault((m[0], m[1]), set()).add((fname, "movement", lk.name))
            for ln in lk.lanes:
                for c in ln.connects:
                    refs.setdefault((c.region, c.node), set()).add((fname, "connectsTo", f"{lk.name}/lane{ln.lane_id}"))

    matched = {k: v for k, v in refs.items() if k in declared}
    unresolved = {k: v for k, v in refs.items() if k not in declared and k != (None, None)}
    regions = sorted({r for r, _ in list(declared) + list(refs) if r is not None})

    # 台账草案（决策文件契约）
    yaml_lines = [
        "# 金凤示范区 region/node 存量台账草案（自动生成 %d 路口 + 引用图）" % len(nodes),
        "# 决策规则：decision 为空的条目属 FORBIDDEN_AUTO（node ID 裁决），须人工填写后此文件方可转正",
        "region_candidates: %s   # 观测到的 region 取值（混用实锤）" % regions,
        "nodes:",
    ]
    for fname, n in nodes:
        ref_by = sorted({f for f, _, _ in refs.get((n.region, n.node_id), set()) if f != fname})
        hint = "".join(ch for ch in fname if ch.isdigit())
        yaml_lines += [
            "  - file: %s" % fname,
            "    declared: {region: %s, id: %s}" % (n.region, n.node_id),
            "    filename_hint: %s" % (hint[-2:] if hint else "null"),
            "    referenced_by_declared_id: %s" % (ref_by if ref_by else "[]"),
            "    decision: null        # 人工：canonical {region, id}",
        ]
    yaml_lines.append("unresolved_refs:      # 被引用但没有任何文件以此自称——引用失配/缺文件")
    for (r, i), who in sorted(unresolved.items(), key=lambda x: (str(x[0][0]), str(x[0][1]))):
        files = sorted({f for f, _, _ in who})
        yaml_lines.append("  - {ref: {region: %s, id: %s}, referenced_by: %s, decision: null}" % (r, i, files))
    (LEDGER / "jinfeng-2026.draft.yaml").write_text("\n".join(yaml_lines), encoding="utf-8")

    # 报告
    rep = ["# 金凤 7 路口解码与 ID 清洗报告", "",
           "## 解码统计", ""]
    for fname, n in nodes:
        n_lane = sum(len(lk.lanes) for lk in n.links)
        n_pts = sum(len(lk.points) + sum(len(ln.points) for ln in lk.lanes) for lk in n.links)
        phases = sorted({m[2] for lk in n.links for m in lk.movements if m[2] is not None} |
                        {c.phase for lk in n.links for ln in lk.lanes for c in ln.connects if c.phase is not None})
        rep.append(f"- {fname}: 自称 ({n.region},{n.node_id})  {len(n.links)} Link / {n_lane} 车道 / {n_pts} 点  phase={phases}")
    rep += ["", "## ID 交叉核对", "",
            f"- region 取值（混用）：{regions}",
            f"- 引用可解析（有文件自称该 ID）：{len(matched)} 个；**引用失配：{len(unresolved)} 个**", ""]
    for (r, i), who in sorted(unresolved.items(), key=lambda x: (str(x[0][0]), str(x[0][1]))):
        files = sorted({f.split("map")[-1].split(".xml")[0] for f, _, _ in who})
        rep.append(f"  - 被引用 ({r},{i}) ← {files}（无对应文件）")
    rep += ["", f"台账草案：ledger/jinfeng-2026.draft.yaml（{len(nodes)} 节点 + {len(unresolved)} 待决引用，decision 全部待人工）",
            "GeoJSON 预览：out/geojson/ ×%d" % len(nodes)]
    (OUT / "id_cleanup_report.md").write_text("\n".join(rep), encoding="utf-8")
    print("\n".join(rep))


if __name__ == "__main__":
    main()
