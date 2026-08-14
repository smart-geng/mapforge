# -*- coding: utf-8 -*-
"""SHP→xodr 直转门禁（node4）：junction 结构完整 + 连接路实测几何 + XSD + 连续性。

历史教训：曾经 CLI 的 SHP→xodr 借道 SHP→MAP→xodr（空口消息窄门），产物无 junction、
单一平均车道宽——本测试锁死直转路径的结构底线，防止回退。
"""
from pathlib import Path

import pytest
from lxml import etree

pytestmark = pytest.mark.slow

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def ibd():
    from mapforge.adapters.shp.ibd_reader import IbdSource
    return IbdSource(str(ROOT / "shp_0222-0326"))


def test_direct_xodr_node4(ibd, tmp_path):
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops.shp_to_xodr import build_junction_xodr
    from mapforge.validate.planview_check import check_file

    ref = parse_map_xml(str(ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"))
    junc, dist = ibd.find_junction(ref.ref_lon, ref.ref_lat)
    assert dist < 30
    out = tmp_path / "node4.xodr"
    st = build_junction_xodr(ibd, junc, out)

    # 结构：进/出口路全建；连接路全部实测几何（node4 无 TOPO 直连缺口）；laneLink 全覆盖
    assert st["roads_enter"] == 4 and st["roads_leave"] == 4
    assert st["connections"] >= 20 and st["conn_g2"] == 0
    assert st["lanelinks"] >= st["connections"]
    assert st["fit_dev_max"] < 0.5

    root = etree.parse(str(out)).getroot()
    assert len(root.findall("junction")) == 1
    assert len(root.findall("junction/connection")) == st["connections"]
    conn_roads = [r for r in root.findall("road")
                  if r.get("junction") not in (None, "-1")
                  and r.get("name") != "junction_paving"]
    assert len(conn_roads) == st["conn_via"] + st["conn_g2"]
    assert st.get("paving") == "polygon"                 # IBD 交叉口面铺面（路口不露底）
    # 每车道真实宽度落盘（node4 存在 3.45/3.5/3.65 等多值，均分宽会退化成单值）
    widths = {w.get("a") for w in root.findall("road/lanes/laneSection/right/lane/width")}
    assert len(widths) >= 3

    # 门禁：XSD 1.5M + planView 连续性
    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    assert schema.validate(etree.parse(str(out))), schema.error_log
    chk = check_file(str(out))
    assert not chk["violations"]

    # 门禁：虚拟行车换乘连续（进口车道→连接路→出口车道，两端 G2 桥构造性归零）
    import math
    from mapforge.validate.smoothness import audit_file, route_continuity
    rc = route_continuity(etree.parse(str(out)).getroot())
    assert rc
    assert max(r["gap_in"] for r in rc) < 0.01
    assert max(r["gap_out"] for r in rc if not math.isnan(r["gap_out"])) < 0.01
    assert max(r["dh_in_deg"] for r in rc) < 0.1

    # 门禁：全网 G2（参考线结点曲率连续——SolveG2 过渡后应为机器精度）
    au = audit_file(out)
    assert au["kappa_step_max"] < 1e-6
    assert au["lane_edge_step_max"] < 0.05
