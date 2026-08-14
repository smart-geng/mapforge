# -*- coding: utf-8 -*-
"""SHP Profile 引擎门禁：
- 快测：profile 校验规则、模板可加载、边界中线合成；
- 慢测（真实数据）：YAML 引擎与内置 IbdSource 等价（去时间戳逐字节）、
  路线 B 边界推宽 vs WIDTH 字段（恒宽车道厘米级；变宽车道差异是数据真相，允许少量离群）。
"""
import re
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


# ---------- 快测 ----------

def test_validate_profile_rules():
    from mapforge.adapters.shp.profile_source import validate_profile
    assert validate_profile({}) != []
    assert any("layers.lane" in e for e in validate_profile({"layers": {}}))
    p = {"layers": {"lane": {"file": "L", "fields": {"id": "A"}}}}
    assert any("fields.road" in e for e in validate_profile(p))
    p = {"layers": {"lane": {"file": "L", "geometry": "boundaries",
                             "fields": {"id": "A", "road": "B"}}}}
    assert any("boundaries 路线" in e for e in validate_profile(p))


def test_template_is_valid_profile():
    from mapforge.adapters.shp.profile_source import TEMPLATE_YAML, validate_profile
    assert validate_profile(yaml.safe_load(TEMPLATE_YAML)) == []


def test_midline_of_parallel_lines():
    from mapforge.adapters.shp.profile_source import midline_of
    x = np.linspace(0, 20, 21)
    left = np.column_stack([x, np.zeros_like(x)])
    right = np.column_stack([x, np.full_like(x, -3.5)])
    mid = midline_of(left, right)
    assert abs(float(np.median(mid[:, 1])) + 1.75) < 0.02


# ---------- 慢测（金凤真实数据） ----------

pytestmark_slow = pytest.mark.slow


@pytest.fixture(scope="module")
def node4_ref():
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    return parse_map_xml(str(ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"))


@pytest.mark.slow
def test_profile_engine_supersedes_builtin(node4_ref, tmp_path):
    """超集契约：YAML 引擎与内置读取器结构全等、非宽度内容逐字节全等；
    宽度只允许在**内置缺数据处**分歧（WIDTH=0 脏数据由 boundaries 阶梯救回——
    金凤实锤 6 条车道，引擎行为优于内置）。"""
    from mapforge.adapters.shp.ibd_reader import IbdSource
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.ops.shp_to_xodr import build_junction_xodr

    a = IbdSource(str(ROOT / "shp_0222-0326"))
    ja, _ = a.find_junction(node4_ref.ref_lon, node4_ref.ref_lat)
    sa = build_junction_xodr(a, ja, tmp_path / "builtin.xodr")

    b = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    jb, _ = b.find_junction(node4_ref.ref_lon, node4_ref.ref_lat)
    sb = build_junction_xodr(b, jb, tmp_path / "profile.xodr")

    for k in ("roads_enter", "roads_leave", "sections", "conn_via", "conn_g2",
              "connections", "lanelinks"):
        assert sa[k] == sb[k], k

    def norm(p, drop_width):
        txt = re.sub(r'date="[^"]*"', 'date=""', p.read_text(encoding="utf-8"))
        return re.sub(r"<width [^/]*/>", "", txt) if drop_width else txt

    # 非宽度内容逐字节全等
    assert norm(tmp_path / "builtin.xodr", True) == norm(tmp_path / "profile.xodr", True)
    # 宽度分歧有界：仅内置缺数据的车道（金凤 node4 拼链范围实测 ≤8 处）
    wa = re.findall(r"<width [^/]*/>", norm(tmp_path / "builtin.xodr", False))
    wb = re.findall(r"<width [^/]*/>", norm(tmp_path / "profile.xodr", False))
    n_diff = sum(1 for x, y in zip(wa, wb) if x != y)
    assert len(wa) == len(wb) and 0 < n_diff <= 8
    assert b.derivation_stats["width"]["boundaries"] > 0     # 阶梯确实启用了


@pytest.mark.slow
def test_route_b_width_from_boundaries(node4_ref):
    from mapforge.adapters.shp.ibd_reader import IbdSource
    from mapforge.adapters.shp.profile_source import ProfileSource

    truth_src = IbdSource(str(ROOT / "shp_0222-0326"))
    junc, _ = truth_src.find_junction(node4_ref.ref_lon, node4_ref.ref_lat)
    c = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-boundary-demo")
    diffs = []
    for lpid in junc.enter_roads + junc.leave_roads:
        truth = {l.lane_pid: l.width_mm for l in truth_src.lanes_of(lpid)}
        for l in c.lanes_of(lpid):
            if l.lane_pid in truth:
                diffs.append(abs(l.width_mm - truth[l.lane_pid]))
    assert len(diffs) >= 20
    assert float(np.median(diffs)) <= 50                 # 恒宽车道：厘米级
    assert sum(d > 100 for d in diffs) <= 3              # 离群=口部展宽变宽车道（数据真相非推导误差）
    assert c.derivation_stats["width"]["boundaries"] > 0
    assert c.derivation_stats["width"]["field"] == 0
