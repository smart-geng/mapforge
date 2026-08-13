# -*- coding: utf-8 -*-
"""Spike-B：金凤 node16（凤阁路-金剑路）端到端——MAP XML → 拟合 → scenariogeneration → xodr。

验证点：① XML 方言解析；② 稀疏点列（3-5 点/Link）拟合为整条参考线（对照 xml2xodr 每两点一条 road）；
③ pyclothoids SolveG2 生成 G2 连接路；④ 连续性检查器 + XSD 校验。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, r"F:\MapFactory")
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops.refline_fit import fit_polyline, eval_planview, lateral_deviation, PlanView, PlanSeg
from mapforge.validate.planview_check import check_file

from scenariogeneration import xodr

XML = Path(r"F:\MapFactory\v2x_map_xml\map凤阁路-金剑路路口node16.xml")
OUT = Path(r"F:\MapFactory\out")
R_EARTH = 6378137.0


def project(pts_deg, lat0, lon0):
    """局部等距圆柱投影（路口尺度 <1 km，球面近似误差可忽略；正式版换 pyproj sterea）。"""
    lat0r = math.radians(lat0)
    out = []
    for lon, lat in pts_deg:
        x = math.radians(lon - lon0) * R_EARTH * math.cos(lat0r)
        y = math.radians(lat - lat0) * R_EARTH
        out.append((x, y))
    return np.asarray(out)


def planview_to_sg(pv: PlanView):
    """mapforge PlanView → scenariogeneration 固定几何列表 [(geom, x, y, h)]。"""
    geoms = []
    x, y, h = pv.x0, pv.y0, pv.hdg
    for seg in pv.segs:
        if seg.kind == "line" or abs(seg.curvature) < 1e-12:
            geoms.append((xodr.Line(seg.length), x, y, h))
            x += seg.length * math.cos(h)
            y += seg.length * math.sin(h)
        else:
            k = seg.curvature
            geoms.append((xodr.Arc(k, length=seg.length), x, y, h))
            x = x + (math.sin(h + k * seg.length) - math.sin(h)) / k
            y = y - (math.cos(h + k * seg.length) - math.cos(h)) / k
            h = h + k * seg.length
    return geoms


def build_road(road_id, pv: PlanView, n_right, lane_w, road_type=-1):
    plan = xodr.PlanView()
    for geom, x, y, h in planview_to_sg(pv):
        plan.add_fixed_geometry(geom, x, y, h)
    center = xodr.Lane(a=0)
    ls = xodr.LaneSection(0, center)
    for _ in range(max(1, n_right)):
        ls.add_right_lane(xodr.Lane(lane_type=xodr.LaneType.driving, a=lane_w))
    lanes = xodr.Lanes()
    lanes.add_lanesection(ls)
    return xodr.Road(road_id, plan, lanes, road_type=road_type)


def end_pose(pv: PlanView):
    x, y, h = pv.x0, pv.y0, pv.hdg
    for seg in pv.segs:
        if seg.kind == "line" or abs(seg.curvature) < 1e-12:
            x += seg.length * math.cos(h)
            y += seg.length * math.sin(h)
        else:
            k = seg.curvature
            x = x + (math.sin(h + k * seg.length) - math.sin(h)) / k
            y = y - (math.cos(h + k * seg.length) - math.cos(h)) / k
            h += k * seg.length
    return x, y, h


def g2_connect(p0, p1, road_id):
    """pyclothoids SolveG2 → 三段 Spiral 的连接路 PlanView 几何列表。"""
    from pyclothoids import SolveG2
    clothoids = SolveG2(p0[0], p0[1], p0[2], 0.0, p1[0], p1[1], p1[2], 0.0)
    geoms = []
    for c in clothoids:
        k0 = c.KappaStart
        k1 = c.KappaEnd
        geoms.append((xodr.Spiral(k0, k1, length=c.length), c.XStart, c.YStart, c.ThetaStart))
    total = sum(c.length for c in clothoids)
    plan = xodr.PlanView()
    for geom, x, y, h in geoms:
        plan.add_fixed_geometry(geom, x, y, h)
    center = xodr.Lane(a=0)
    ls = xodr.LaneSection(0, center)
    ls.add_right_lane(xodr.Lane(lane_type=xodr.LaneType.driving, a=3.5))
    lanes = xodr.Lanes()
    lanes.add_lanesection(ls)
    return xodr.Road(road_id, plan, lanes), total, [(c.KappaStart, c.KappaEnd, c.length) for c in clothoids]


def _strip_empty_profiles(path: Path):
    """scenariogeneration 会写空的 elevationProfile/lateralProfile，1.5M XSD 不允许——删除空元素。"""
    import xml.etree.ElementTree as ET
    tree = ET.parse(str(path))
    for road in tree.getroot().findall("road"):
        for tag in ("elevationProfile", "lateralProfile"):
            el = road.find(tag)
            if el is not None and len(el) == 0:
                road.remove(el)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def main():
    OUT.mkdir(exist_ok=True)
    node = parse_map_xml(str(XML))
    rep = [f"# Spike-B 报告：{XML.name} 端到端", "",
           f"Node ({node.region},{node.node_id}) name={node.name} refPos=({node.ref_lat:.7f},{node.ref_lon:.7f})", ""]

    odr = xodr.OpenDrive("node16")
    link_pv = {}
    total_src_pts = 0
    rep.append("## Link 参考线拟合（Link.points 中线，稀疏点列）")
    for i, lk in enumerate(node.links):
        pts = project(lk.points, node.ref_lat, node.ref_lon)
        total_src_pts += len(lk.points)
        if pts.shape[0] < 2:
            rep.append(f"- Link {lk.name}: 点数不足，跳过")
            continue
        pv, pieces = fit_polyline(pts, kappa_th=1 / 600, min_seg_len=6.0, smooth_win=3, refine=True)
        recon = eval_planview(pv, step=0.5)
        dmax, dmean = lateral_deviation(recon, pts)
        segstr = " ".join(("A" if s.kind == "arc" else "L") + f"{s.length:.0f}m" for s in pv.segs)
        lane_ws = [ln.width_cm for ln in lk.lanes if ln.width_cm]
        lane_w = (sum(lane_ws) / len(lane_ws) / 100.0) if lane_ws else 3.5
        road = build_road(10 + i, pv, n_right=len(lk.lanes), lane_w=lane_w)
        odr.add_road(road)
        link_pv[lk.name] = pv
        rep.append(f"- Link {lk.name}（{len(lk.points)} 点，{len(lk.lanes)} 车道，宽 {lane_w:.2f} m）"
                   f"→ [{segstr}] 横向偏差 max {dmax*100:.1f} cm / mean {dmean*100:.1f} cm；"
                   f"movements phase={[m[2] for m in lk.movements]}")

    # 两条示范 G2 连接路：west→north（左转类）与 west→east 对向直行类（按几何位姿连接）
    rep += ["", "## G2 连接路（pyclothoids SolveG2 → 三段 spiral）"]
    conns = []
    names = [lk.name for lk in node.links]
    pairs = [("west", "north"), ("south", "west")]
    rid = 50
    for a, b in pairs:
        if a not in link_pv or b not in link_pv:
            continue
        pa = end_pose(link_pv[a])                       # 进口道末点（停止线，朝路口）
        pvb = link_pv[b]
        xb, yb, hb = end_pose(pvb)
        p1 = (xb, yb, hb + math.pi)                     # 出口 = 对方 Link 末点反向驶出
        road, total, segs = g2_connect(pa, p1, rid)
        odr.add_road(road)
        conns.append((a, b, total))
        segtxt = "; ".join(f"k:{k0:+.4f}->{k1:+.4f} L={L:.1f}m" for k0, k1, L in segs)
        rep.append(f"- {a}→{b}: 总长 {total:.1f} m，[{segtxt}]")
        rid += 1

    out_path = OUT / "spike_b_node16.xodr"
    odr.write_xml(str(out_path), prettyprint=True)
    _strip_empty_profiles(out_path)
    rep += ["", f"输出：{out_path}"]

    # 门禁：连续性 + XSD
    chk = check_file(str(out_path))
    rep += ["", "## 门禁", "",
            f"- planView 连续性：检查 {chk['pairs_checked']} 对相邻段，超阈值 road 数 {len(chk['violations'])}，"
            f"最差位置差 {chk['worst_pos_gap_m']*1000:.3f} mm / 最差航向差 {chk['worst_hdg_gap_rad']*1000:.3f} mrad"]
    try:
        from lxml import etree
        schema = etree.XMLSchema(etree.parse(r"F:\MapFactory\OpenDRIVE_1.5M.xsd"))
        doc = etree.parse(str(out_path))
        ok = schema.validate(doc)
        rep.append(f"- XSD 1.5M：{'PASS' if ok else 'FAIL'}"
                   + ("" if ok else f"（前 3 错：{[str(e) for e in schema.error_log[:3]]}）"))
    except Exception as e:  # noqa
        rep.append(f"- XSD 校验异常：{e}")

    # 与 xml2xodr“每两点一条 road”的结构对比
    old_style_roads = sum(max(0, len(lk.points) - 1) + sum(max(0, len(ln.points) - 1) for ln in lk.lanes)
                          for lk in node.links)
    rep += ["", "## 结构对比（vs xml2xodr 逐段拼接）", "",
            f"- mapforge：{len(node.links)} 条 Link road + {len(conns)} 条连接路 = {len(node.links)+len(conns)} 条 road，"
            f"road 内多段参数几何，G1/G2 连续",
            f"- xml2xodr 方式推算：每两点一条 road ≈ {old_style_roads} 条独立 road，逐段 G0 拼接、无连接路"]

    report = "\n".join(rep)
    (OUT / "spike_b_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
