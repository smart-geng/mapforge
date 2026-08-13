# -*- coding: utf-8 -*-
"""黄金回归：金凤 7 路口 SHP→MAP 对拍门禁（方案 11 节回归判定的可执行化）。

判定不看字节相同，看：Link/车道结构、几何横向容差、phase 绑定、connectsTo 覆盖。
标记 slow：IBD 索引全量加载（首路口 ~40s，后续复用）。
"""
import math
from pathlib import Path

import pytest

pytestmark = pytest.mark.slow

SRC = Path(r"F:\MapFactory\v2x_map_xml")
SHP = r"F:\MapFactory\shp_0222-0326"


@pytest.fixture(scope="module")
def ibd():
    from mapforge.adapters.shp.ibd_reader import IbdSource
    return IbdSource(SHP)


@pytest.mark.parametrize("xml", sorted(SRC.glob("map*.xml")), ids=lambda x: x.stem[-8:])
def test_shp_to_map_matches_live(xml, ibd, tmp_path):
    import sys
    sys.path.insert(0, r"F:\MapFactory\scripts")
    from m1_shp_to_map import process
    node, r = process(xml, ibd, tmp_path)
    # 结构：Link 4/4；车道差 ≤1（node16 已知版本差异）
    g_l, x_l = map(int, r["links"].split("/"))
    g_n, x_n = map(int, r["lanes"].split("/"))
    assert g_l == x_l == 4
    assert abs(g_n - x_n) <= 1
    # 几何：横向中位 ≤0.5 m（实测 0.00–0.13）
    assert not math.isnan(r["lat_med"]) and r["lat_med"] <= 0.5
    # phase：绑定率 ≥65%（node18 实测 25/36 为下限；其余 100%）
    ph_b, ph_t = map(int, r["phase"].split("/"))
    assert ph_t == 0 or ph_b / ph_t >= 0.65
    # connectsTo：生成侧不少于现网（TOPO 更全）
    c_g, c_x = map(int, r["conn"].split("/"))
    assert c_g >= c_x
