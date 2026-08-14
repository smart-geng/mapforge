# -*- coding: utf-8 -*-
"""MAP→xodr junction 重建门禁（快测，不依赖 IBD 索引）。

- 出口路两级来源：多节点帧邻居真实 inLink 优先（real），单节点帧镜像兜底（mirror/INFERRED）；
- 已知 ID 失配引用（NODE5 west→20701，资料盘点三.3）必须诚实 skip，不得脑补出口；
- 平滑门禁：全部连接路与进口/出口路**车道级曲率连续**（SolveG2 传入端部曲率，
  含 κ/(1−tκ) 偏移修正）；稀疏抽稀点列由 G1 clothoid spline 档处理（顶点偏差度量）。
"""
from pathlib import Path

import numpy as np
import pytest
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]


def _convert(xml_name, tmp_path):
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops.map_to_xodr import build_xodr
    node = parse_map_xml(str(ROOT / "v2x_map_xml" / xml_name))
    out = tmp_path / "o.xodr"
    st = build_xodr(node, out)
    n_xml = sum(len(ln.connects) for lk in node.links for ln in lk.lanes)
    return st, out, n_xml


def _validate(out):
    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    assert schema.validate(etree.parse(str(out))), schema.error_log
    from mapforge.validate.planview_check import check_file
    assert not check_file(str(out))["violations"]
    from mapforge.validate.smoothness import audit_file
    assert audit_file(out)["kappa_step_max"] < 1e-6      # 全网 G2 门禁


def _worst_seam_kappa_gap(out) -> float:
    """连接路起点曲率 vs 前驱路末段曲率（允许车道偏移修正 κ/(1−tκ)，|t|≤8m）。"""
    root = etree.parse(str(out)).getroot()
    roads = {rd.get("id"): rd for rd in root.findall("road")}

    def last_kappa(rd):
        g = rd.findall("planView/geometry")[-1]
        if g.find("arc") is not None:
            return float(g.find("arc").get("curvature"))
        if g.find("spiral") is not None:
            return float(g.find("spiral").get("curvEnd"))
        return 0.0

    worst = 0.0
    for rd in root.findall("road"):
        if rd.get("junction") in (None, "-1") or rd.get("name") == "junction_paving":
            continue
        k_conn = float(rd.findall("planView/geometry")[0].find("spiral").get("curvStart"))
        k_ref = last_kappa(roads[rd.find("link/predecessor").get("elementId")])
        if abs(k_ref) < 1e-12:
            gap = abs(k_conn)
        else:
            gap = min(abs(k_conn - k_ref / (1 - t * k_ref)) for t in np.linspace(-8, 8, 65))
        worst = max(worst, gap)
    return worst


@pytest.mark.parametrize("xml", ["map凤阁路-金剑路路口node16.xml",
                                 "map凤苑路-金玥路node4.xml"])
def test_junction_synthesized_full_coverage(xml, tmp_path):
    st, out, n_xml = _convert(xml, tmp_path)
    assert st["skipped"] == 0
    assert st["connections"] == st["conn_roads"] == n_xml    # connectsTo 全覆盖
    assert st["exit_roads"] >= 3 and st["exit_real"] == 0    # 单节点帧：全镜像
    root = etree.parse(str(out)).getroot()
    assert len(root.findall("junction")) == 1
    assert len(root.findall("junction/connection")) == n_xml
    # 双向 leg 模型：出口并入 leg road 左侧，road 数 = 进口 leg + 连接路 + 铺面
    n_pave = 1 if st.get("paving") == "hull" else 0
    assert len(root.findall("road")) == st["links"] + st["conn_roads"] + n_pave
    assert root.findall("road/lanes/laneSection/left/lane")  # 左侧车道确实存在
    _validate(out)
    assert _worst_seam_kappa_gap(out) < 1e-6                 # 车道级曲率连续（G2 接缝）


def test_known_id_mismatch_skipped_not_invented(tmp_path):
    st, _out, n_xml = _convert("map凤苑路-金坪路NODE5.xml", tmp_path)
    assert st["skipped"] == 1                                # west lane2 → 20701（无此上游）
    assert st["connections"] == n_xml - 1


def test_real_exits_from_multinode_frame(tmp_path):
    """合帧（node4+node18+node3）：node4 的 west/south 出口用邻居真实 inLink 几何。"""
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all
    from mapforge.ops.map_to_xodr import build_xodr
    base = etree.parse(str(ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"))
    nodes_el = base.getroot().find("mapFrame/nodes")
    for f in ("map凤苑路-金剑路node18.xml", "map含金路-金玥路node3.xml"):
        nodes_el.append(etree.parse(str(ROOT / "v2x_map_xml" / f))
                        .getroot().find("mapFrame/nodes/Node"))
    merged = tmp_path / "merged.xml"
    base.write(str(merged), encoding="utf-8", xml_declaration=True)

    nodes = parse_map_xml_all(str(merged))
    assert len(nodes) == 3
    node = next(n for n in nodes if n.node_id == 4)
    out = tmp_path / "m.xodr"
    st = build_xodr(node, out, neighbors=[n for n in nodes if n.node_id != 4])
    assert st["exit_real"] == 2 and st["exit_mirror"] == 2
    assert st["skipped"] == 0
    # west 链 s255-298 有数字化噪声（R=10m 级振荡），曲率封顶主动平滑——
    # 对原始（含噪）顶点的偏差 0.68m 是修复的代价而非回归；封顶行为本身锁死
    assert st.get("refit_smoothed", 0) >= 1
    assert st["fit_dev_max"] < 1.0
    _validate(out)
    assert _worst_seam_kappa_gap(out) < 1e-6
