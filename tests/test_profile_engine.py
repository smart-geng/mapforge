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
        if not drop_width:
            return txt
        # laneOffset 与 width 同源：v1.19 起车道边界由实测轮廓导出（边界=相邻中心中点、
        # laneOffset=最左车道左缘），故宽度救回**必然**传导到 laneOffset——两者一起剔除后
        # 再比"其余内容逐字节全等"，laneOffset 的分歧幅度另行有界检查
        txt = re.sub(r"<width [^/]*/>", "", txt)
        return re.sub(r"<laneOffset [^/]*/>", "", txt)

    # 非宽度/非 laneOffset 内容逐字节全等
    assert norm(tmp_path / "builtin.xodr", True) == norm(tmp_path / "profile.xodr", True)
    # laneOffset 分歧有界：只应源自被救回的宽度（半个车道宽以内）
    oa = [float(m) for m in re.findall(r'<laneOffset s="[^"]*" a="([^"]*)"',
                                       (tmp_path / "builtin.xodr").read_text(encoding="utf-8"))]
    ob = [float(m) for m in re.findall(r'<laneOffset s="[^"]*" a="([^"]*)"',
                                       (tmp_path / "profile.xodr").read_text(encoding="utf-8"))]
    assert len(oa) == len(ob)
    assert max((abs(x - y) for x, y in zip(oa, ob)), default=0.0) < 2.5
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

    from lxml import etree

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
    assert ra.keys() == rb.keys()
    affected = {(road, si) for (road, si, _lid), (sid, _w) in rb.items()
                if sid in rescued}
    affected |= {(road, si + d) for road, si in list(affected) for d in (-1, 1)}
    changed = [key for key in ra if ra[key][1] != rb[key][1]]
    assert changed
    assert all((road, si) in affected for road, si, _lid in changed)
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
