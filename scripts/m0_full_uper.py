# -*- coding: utf-8 -*-
"""M0：7 路口整帧 UPER 编码回环 + 大小统计（绝对坐标 vs 偏移编码实测对比，喂方案 5.5 预算）。

回环校验：absolute 模式解码回读值 == 编码值；offset 模式解码后坐标还原与原值差 ≤1 LSB（1e-7°）。
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "mapforge" / "adapters" / "v2xmap" / "asn"))

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val, _i7, _OFFSET_TIERS

SRC = ROOT / "v2x_map_xml"
OUT = ROOT / "out"


def restore_points(val):
    """从 MessageFrame 值提取所有点的绝对 1e-7° 坐标（按遍历序），offset 分支加回 refPos。"""
    md = val[1]
    out = []
    for nd in md["nodes"]:
        rlat, rlon = nd["refPos"]["lat"], nd["refPos"]["long"]
        for lk in nd.get("inLinks", []):
            conts = [lk.get("points", [])] + [ln.get("points", []) for ln in lk.get("lanes", [])]
            for pts in conts:
                for p in pts:
                    br, xy = p["posOffset"]["offsetLL"]
                    if br == "position-LatLon":
                        out.append((xy["lon"], xy["lat"]))
                    else:
                        out.append((rlon + xy["lon"], rlat + xy["lat"]))
    return out


def main():
    import msglayer_draft as m
    mf_t = m.MsgLayerDraft.MessageFrame
    rows = []
    tier_used = {}
    for f in sorted(SRC.glob("map*.xml")):
        node = parse_map_xml(str(f))
        xml_bytes = f.stat().st_size

        v_abs = node_to_messageframe_val(node, "absolute")
        mf_t.set_val(v_abs)
        b_abs = mf_t.to_uper()
        mf_t.from_uper(b_abs)
        ok_abs = mf_t.get_val() == v_abs

        v_off = node_to_messageframe_val(node, "offset")
        mf_t.set_val(v_off)
        b_off = mf_t.to_uper()
        mf_t.from_uper(b_off)
        pts_orig = restore_points(v_abs)
        pts_rest = restore_points(mf_t.get_val())
        ok_off = len(pts_orig) == len(pts_rest) and all(
            abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1 for a, b in zip(pts_orig, pts_rest))
        for nd in v_off[1]["nodes"]:
            for lk in nd.get("inLinks", []):
                for pts in [lk.get("points", [])] + [ln.get("points", []) for ln in lk.get("lanes", [])]:
                    for p in pts:
                        br = p["posOffset"]["offsetLL"][0]
                        tier_used[br] = tier_used.get(br, 0) + 1

        n_pts = len(pts_orig)
        rows.append((f.stem.replace("map", ""), n_pts, xml_bytes, len(b_abs), len(b_off),
                     "PASS" if ok_abs else "FAIL", "PASS" if ok_off else "FAIL"))

    rep = ["# 7 路口整帧 UPER 编码与大小统计", "",
           "| 路口 | 点数 | XML 明文 B | UPER 绝对 B | UPER 偏移 B | 偏移省 | 绝对回环 | 偏移回环(≤1LSB) |",
           "|---|---|---|---|---|---|---|---|"]
    for name, n, xb, ab, ob, ra, ro in rows:
        rep.append(f"| {name} | {n} | {xb} | {ab} | {ob} | {100*(ab-ob)/ab:.0f}% | {ra} | {ro} |")
    tot_ab = sum(r[3] for r in rows)
    tot_ob = sum(r[4] for r in rows)
    rep += ["",
            f"合计：XML {sum(r[2] for r in rows)/1024:.0f} KB → UPER 绝对 {tot_ab} B → UPER 偏移 {tot_ob} B"
            f"（偏移编码整体再省 {100*(tot_ab-tot_ob)/tot_ab:.0f}%）",
            f"偏移档使用分布：{tier_used}", "",
            "口径：XML 明文为现网文件大小（含缩进）；UPER 绝对=现网同语义编码；UPER 偏移=mapforge 优化项（逐点最小档）。",
            "结论供方案 5.5 大小预算：单路口 UPER 帧量级与偏移收益以此为基线，实链路预算待 RSU 标定。"]
    report = "\n".join(rep)
    (OUT / "uper_size_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
