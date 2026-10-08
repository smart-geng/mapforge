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
    """超集契约：Profile 保持道路/拓扑结构，同时可消费内置 reader 看不到的真实边界。

    WIDTH=0 仍只允许由 boundary 阶梯救回；v1.28 起 Profile 还会用 LANE_BOUNDARY
    锚定道路物理外缘，所以 laneOffset/width/provenance 不再要求与内置 reader 逐字节相同。
    """
    from mapforge.adapters.shp.ibd_reader import IbdSource
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.ops.shp_to_xodr import build_junction_xodr

    if not (ROOT / "shp_0222-0326" / "IBD_LANE_LINK.shp").exists():
        pytest.skip("original SHP delivery not installed")
    a = IbdSource(str(ROOT / "shp_0222-0326"))
    ja, _ = a.find_junction(node4_ref.ref_lon, node4_ref.ref_lat)
    sa = build_junction_xodr(a, ja, tmp_path / "builtin.xodr")

    b = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    jb, _ = b.find_junction(node4_ref.ref_lon, node4_ref.ref_lat)
    sb = build_junction_xodr(b, jb, tmp_path / "profile.xodr")

    for k in ("roads_enter", "roads_leave", "conn_via", "conn_g2",
              "connections", "lanelinks"):
        assert sa[k] == sb[k], k
    assert sb["sections"] >= sa["sections"]

    from lxml import etree
    ra_xml = etree.parse(str(tmp_path / "builtin.xodr")).getroot()
    rb_xml = etree.parse(str(tmp_path / "profile.xodr")).getroot()
    # 参考线和 junction 拓扑不受 Profile 边界增强影响；变化只在横断面表达。
    legs_a = [x for x in ra_xml.findall("road") if x.get("junction") in (None, "-1")]
    legs_b = [x for x in rb_xml.findall("road") if x.get("junction") in (None, "-1")]
    assert [etree.tostring(x.find("planView")) for x in legs_a] == [
        etree.tostring(x.find("planView")) for x in legs_b]
    assert [(x.get("incomingRoad"), x.get("connectingRoad"), x.get("contactPoint"))
            for x in ra_xml.findall("junction/connection")] == [
        (x.get("incomingRoad"), x.get("connectingRoad"), x.get("contactPoint"))
        for x in rb_xml.findall("junction/connection")]
    assert sb.get("source_boundary_endpoints", 0) > 0
    assert sb.get("boundary_section_step_m") == 10.0
    assert sb.get("physical_edge_fill_max_m", 0.0) > 0.0
    # 语义契约：Profile 只救回内置 reader 中 WIDTH=0 的记录，绝不改写已有宽度；
    # 输出差异可传播到同 section 的相邻边界/median，以及调和后的相邻 section。
    rescued = set()
    for pid in a.roadlinks:
        aa = {x.lane_pid: x for x in a.lanes_of(pid)}
        bb = {x.lane_pid: x for x in b.lanes_of(pid)}
        for sid, old in aa.items():
            new = bb[sid]
            if old.width_mm == new.width_mm:
                continue
            assert old.width_mm == 0 and new.width_mm > 0
            assert (old.s_width_mm, old.e_width_mm) == (new.s_width_mm, new.e_width_mm)
            rescued.add(sid)
    assert rescued

    def lane_rows(path):
        root = etree.parse(str(path)).getroot()
        rows = {}
        for rd in root.findall("road"):
            for si, sec in enumerate(rd.findall("lanes/laneSection")):
                for ln in sec.findall("left/lane") + sec.findall("right/lane"):
                    ud = ln.find("userData[@code='mapforge.source_lane']")
                    sid = ud.get("value") if ud is not None else None
                    widths = [tuple(w.get(k) for k in ("sOffset", "a", "b", "c", "d"))
                              for w in ln.findall("width")]
                    rows[(rd.get("id"), si, ln.get("id"))] = (sid, widths)
        return rows

    ra = lane_rows(tmp_path / "builtin.xodr")
    rb = lane_rows(tmp_path / "profile.xodr")
    # 边界增强会增加 laneSection 控制站，不能再按 section 索引逐项对拍；
    # 但来源 lane 集合必须守恒，且确实产生不同的横断面表达。
    src_a = {sid for sid, _widths in ra.values() if sid}
    src_b = {sid for sid, _widths in rb.values() if sid}
    assert src_a == src_b
    assert (tmp_path / "builtin.xodr").read_bytes() != (tmp_path / "profile.xodr").read_bytes()
    assert b.derivation_stats["width"]["boundaries"] > 0     # 阶梯确实启用了


@pytest.mark.slow
def test_route_b_width_from_boundaries(node4_ref):
    from mapforge.adapters.shp.ibd_reader import IbdSource
    from mapforge.adapters.shp.profile_source import ProfileSource

    if not (ROOT / "shp_0222-0326" / "IBD_LANE_LINK.shp").exists():
        pytest.skip("original SHP delivery not installed")
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
