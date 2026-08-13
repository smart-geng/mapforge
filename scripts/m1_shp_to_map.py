# -*- coding: utf-8 -*-
"""M1：金凤 7 路口批量真回归对拍（重塑核心已入包：mapforge.ops.shp_to_map）。

本脚本职责：批量调用 rebuild_from_ibd + 与现网 XML 对拍（横向口径）+ 汇总报告。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, r"F:\MapFactory")
sys.path.insert(0, r"F:\MapFactory\mapforge\adapters\v2xmap\asn")

from mapforge.adapters.shp.ibd_reader import IbdSource
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val
from mapforge.ops.shp_to_map import rebuild_from_ibd, phase_table_from_xml, id_table_from_xml
from mapforge.report.preview_geojson import to_geojson

SHP_DIR = r"F:\MapFactory\shp_0222-0326"
SRC = Path(r"F:\MapFactory\v2x_map_xml")
OUT = Path(r"F:\MapFactory\out")
R_EARTH = 6378137.0


def proj(pts_deg, lat0, lon0):
    pts_deg = np.asarray(pts_deg, dtype=float)
    x = np.radians(pts_deg[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts_deg[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def polyline_dist(query_xy, line_xy):
    """(横向距离中位, 覆盖率)：仅统计投影落在线段内部的点。"""
    lateral, covered = [], 0
    for p in query_xy:
        best, inside = 1e18, False
        for i in range(line_xy.shape[0] - 1):
            a, b = line_xy[i], line_xy[i + 1]
            ab = b - a
            L2 = float(ab @ ab)
            if L2 < 1e-12:
                continue
            t = float((p - a) @ ab / L2)
            if 0.0 <= t <= 1.0:
                d = float(np.linalg.norm(a + t * ab - p))
                if d < best:
                    best, inside = d, True
        if inside:
            lateral.append(best)
            covered += 1
    med = float(np.median(lateral)) if lateral else float("nan")
    return med, covered / max(1, query_xy.shape[0])


def process(xml_path: Path, src: IbdSource, gj_dir: Path):
    ref = parse_map_xml(str(xml_path))
    lat0, lon0 = ref.ref_lat, ref.ref_lon
    node, s, lane_geo = rebuild_from_ibd(
        src, lon0, lat0, region=ref.region, node_id=ref.node_id,
        phase_table=phase_table_from_xml(ref), id_table=id_table_from_xml(ref))

    # 对拍（Link 按方位名匹配——已证一致）
    xml_links = {lk.name: lk for lk in ref.links}
    dists, covers = [], []
    for gi, glk in enumerate(node.links):
        xlk = xml_links.get(glk.name)
        if xlk is None:
            continue
        for xln in xlk.lanes:
            if not xln.points:
                continue
            q = proj(xln.points, lat0, lon0)
            cands = [polyline_dist(q, lane_geo[(gi, ln.lane_id)]) for ln in glk.lanes
                     if (gi, ln.lane_id) in lane_geo]
            cands = [c for c in cands if not math.isnan(c[0])]
            if cands:
                best = min(cands, key=lambda c: c[0])
                dists.append(best[0])
                covers.append(best[1])

    gj = to_geojson(node)
    (gj_dir / (xml_path.stem + ".gen.geojson")).write_text(
        json.dumps(gj, ensure_ascii=False, indent=1), encoding="utf-8")
    import msglayer_draft as m
    mf = m.MsgLayerDraft.MessageFrame
    sizes = {}
    for mode in ("absolute", "offset"):
        val = node_to_messageframe_val(node, mode)
        mf.set_val(val)
        buf = mf.to_uper()
        mf.from_uper(buf)
        sizes[mode] = len(buf)

    n_xml_lanes = sum(len(l.lanes) for l in ref.links)
    x_out = sum(len(ln.connects) for lk in ref.links for ln in lk.lanes)
    return node, {
        "name": xml_path.stem.replace("map", ""), "junc_name": s["junction"], "dist": s["ref_dist_m"],
        "links": f"{s['links']}/{len(ref.links)}", "lanes": f"{s['lanes']}/{n_xml_lanes}",
        "lat_med": float(np.nanmedian(dists)) if dists else float("nan"),
        "lat_p90": float(np.nanpercentile(dists, 90)) if dists else float("nan"),
        "cover": float(np.mean(covers)) if covers else 0.0,
        "conn": f"{s['connects']}/{x_out}", "phase": f"{s['phase_bound']}/{s['phase_bound']+s['phase_missing']}",
        "uper": f"{sizes['absolute']}/{sizes['offset']}",
        "broken": [(b["lane"][-5:], b["len_m"], b["nearest_gap_m"]) for b in s["broken_chains"][:4]],
    }


def main():
    OUT.mkdir(exist_ok=True)
    gj_dir = OUT / "m1_gen_geojson"
    gj_dir.mkdir(exist_ok=True)
    src = IbdSource(SHP_DIR)
    rows, notes = [], []
    for f in sorted(SRC.glob("map*.xml")):
        try:
            _, r = process(f, src, gj_dir)
            rows.append(r)
            if r["broken"]:
                notes.append(f"- {r['name']}: 上游断链样本（车道尾号, 已拼长度m, 最近gap m）: {r['broken']}")
        except Exception as e:  # noqa
            rows.append({"name": f.stem.replace("map", ""), "junc_name": f"FAIL: {e}", "dist": float("nan"),
                         "links": "-", "lanes": "-", "lat_med": float("nan"), "lat_p90": float("nan"),
                         "cover": 0.0, "conn": "-", "phase": "-", "uper": "-", "broken": []})
    rep = ["# M1 批量对拍：金凤 7 路口 SHP→MAP vs 现网 XML", "",
           "重塑核心：mapforge.ops.shp_to_map；横向口径（覆盖段内）；phase 通道=各路口现网 XML 提取的配时表。", "",
           "| 路口 | IBD 路口面 | 面距m | Link 生/现 | 车道 生/现 | 横向中位m | p90 | 覆盖率 | connectsTo 生/现 | phase | UPER 绝/偏 B |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        rep.append(f"| {r['name']} | {r['junc_name']} | {r['dist']:.0f} | {r['links']} | {r['lanes']} | "
                   f"{r['lat_med']:.2f} | {r['lat_p90']:.2f} | {r['cover']*100:.0f}% | {r['conn']} | {r['phase']} | {r['uper']} |")
    ok = [r for r in rows if not math.isnan(r["lat_med"])]
    if ok:
        rep += ["", f"汇总：{len(ok)}/7 路口跑通；横向中位的中位 {np.median([r['lat_med'] for r in ok]):.2f} m；"
                    f"平均覆盖率 {100*np.mean([r['cover'] for r in ok]):.0f}%"]
    if notes:
        rep += ["", "断链诊断："] + notes
    report = "\n".join(rep)
    (OUT / "m1_batch_compare.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
