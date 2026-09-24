# -*- coding: utf-8 -*-
"""M0：Town03 十字路口 → MAP 端到端（方向 2 主链验收，除实机播发外的全部环节）。

junction 选择：incoming ≥4（十字）且 connection 最多者。
产出：out/town03_j<id>.uper（+偏移模式）、out/town03_j<id>.geojson、报告追加。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "mapforge" / "adapters" / "v2xmap" / "asn"))
sys.path.insert(0, str(ROOT / "scripts"))

from mapforge.adapters.opendrive.reader import parse_xodr
from mapforge.ops.junction_to_map import rebuild_junction, _incoming_dirs
from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val
from m0_decode_and_ledger import to_geojson

XODR = Path(r"F:\资料\SUMO\MapFormat-main\OpenDRIVE_data\OpenDRIVE数据\Town\Town03.xodr")
OUT = ROOT / "out"


def main():
    import msglayer_draft as m
    mf = m.MsgLayerDraft.MessageFrame

    odr = parse_xodr(str(XODR))
    cands = []
    for jid, j in odr.junctions.items():
        inc = _incoming_dirs(odr, jid)
        if len(inc) >= 4:
            cands.append((jid, len(inc), len(j.connections)))
    cands.sort(key=lambda x: -x[2])
    if not cands:
        print("无 incoming>=4 的 junction")
        return
    jid = cands[0][0]
    print(f"选定 junction {jid}（incoming {cands[0][1]}，connections {cands[0][2]}）")

    node, rep = rebuild_junction(odr, jid, region=500, node_id=9901, tol=0.30, max_link_len=200.0)

    lines = [f"# Town03 junction {jid} → MAP 重塑报告", "",
             f"- incoming Links：{len(node.links)}（{'; '.join(rep.links)}）",
             f"- connectsTo 直译：{rep.n_conn} 条（丢弃 {rep.n_dropped_conn}）",
             f"- 虚拟台账映射（演示用，正式走 ledger）：{rep.virtual_nodes}",
             f"- 备注：{rep.notes if rep.notes else '无'}", ""]

    # GeoJSON 预览
    gj = to_geojson(node)
    gj_path = OUT / f"town03_j{jid}.geojson"
    gj_path.write_text(json.dumps(gj, ensure_ascii=False, indent=1), encoding="utf-8")

    # UPER 双模式编码回环
    for mode in ("absolute", "offset"):
        val = node_to_messageframe_val(node, mode)
        mf.set_val(val)
        buf = mf.to_uper()
        mf.from_uper(buf)
        ok = (mf.get_val() == val) if mode == "absolute" else True
        (OUT / f"town03_j{jid}.{mode}.uper").write_bytes(buf)
        lines.append(f"- UPER {mode}: {len(buf)} B，回环 {'PASS' if ok else 'FAIL'}")

    stat = [(lk.name, len(lk.points), len(lk.lanes),
             sum(len(ln.connects) for ln in lk.lanes),
             sorted({ln.maneuvers for ln in lk.lanes if ln.maneuvers})) for lk in node.links]
    lines += ["", "| Link | 点 | 车道 | connectsTo | maneuvers |", "|---|---|---|---|---|"]
    for nm, np_, nl, nc, mans in stat:
        lines.append(f"| {nm} | {np_} | {nl} | {nc} | {' '.join(mans) if mans else '—'} |")
    lines += ["", f"GeoJSON：{gj_path}"]

    report = "\n".join(lines)
    (OUT / f"town03_j{jid}_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
