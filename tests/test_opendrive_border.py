# -*- coding: utf-8 -*-
"""ASAM OpenDRIVE border writer/consumer spike。

真实测量边界路线必须使用绝对 border，不能再被 laneOffset + 累加 width 改形。
"""
from pathlib import Path

import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]


def test_border_writer_xsd_and_geometry(tmp_path):
    from mapforge.adapters.opendrive.writer import Lane, LaneSection, Road, XodrDoc
    from mapforge.validate.smoothness import lane_edges_at

    doc = XodrDoc("border-spike", geo_reference="+proj=eqc +units=m")
    road = Road(1, "measured-boundaries")
    road.add_geometry("line", 0.0, 0.0, 0.0, 100.0)
    sec = LaneSection(0.0)
    r1 = Lane(-1); r1.add_border(-3.0)
    r2 = Lane(-2); r2.add_border(-7.0, b=0.01)
    l1 = Lane(1); l1.add_border(3.5)
    sec.right.extend([r1, r2]); sec.left.append(l1)
    road.sections.append(sec); doc.add_road(road)
    out = tmp_path / "border.xodr"
    doc.write(out)

    tree = etree.parse(str(out))
    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    assert schema.validate(tree), schema.error_log
    root = tree.getroot()
    assert not root.findall("road/lanes/laneOffset")
    assert not root.findall("road/lanes/laneSection/right/lane/width")
    assert len(root.findall("road/lanes/laneSection/right/lane/border")) == 2
    road_el = root.find("road")
    assert lane_edges_at(road_el, 50.0, "right") == [0.0, -3.0, -6.5]
    assert lane_edges_at(road_el, 50.0, "left") == [0.0, 3.5]


def test_writer_rejects_border_with_lane_offset(tmp_path):
    import pytest
    from mapforge.adapters.opendrive.writer import Lane, LaneSection, Road, XodrDoc

    doc = XodrDoc("invalid-border")
    road = Road(1); road.add_geometry("line", 0, 0, 0, 10); road.add_offset(0, 1)
    sec = LaneSection(0); lane = Lane(-1); lane.add_border(-3); sec.right.append(lane)
    road.sections.append(sec); doc.add_road(road)
    with pytest.raises(ValueError, match="laneOffset cannot coexist"):
        doc.write(tmp_path / "invalid.xodr")


def test_absolute_borders_materialize_to_equivalent_widths(tmp_path):
    """内部共享边界模型必须能无损转成消费端普遍支持的 lane.width。"""
    from mapforge.adapters.opendrive.writer import Lane, LaneSection, Road, XodrDoc
    from mapforge.ops.lane_family_border import _materialize_widths
    from mapforge.validate.smoothness import lane_edges_at

    doc = XodrDoc("border-materialize", geo_reference="+proj=eqc +units=m")
    road = Road(1, "measured-boundaries")
    road.add_geometry("line", 0.0, 0.0, 0.0, 100.0)
    sec = LaneSection(0.0)
    inner = Lane(-1)
    inner.add_border(-3.0)
    outer = Lane(-2)
    outer.add_border(-7.0, b=0.01)
    sec.right.extend([inner, outer])
    road.sections.append(sec)
    doc.add_road(road)
    out = tmp_path / "before.xodr"
    doc.write(out)

    tree = etree.parse(str(out))
    road_el = tree.getroot().find("road")
    stations = np.linspace(0.0, 100.0, 21)
    expected = [lane_edges_at(road_el, float(s), "right") for s in stations]
    result = _materialize_widths(
        road_el, road_el.findall("lanes/laneSection"), [0.0], [100.0])

    assert result == {"lanes": 2, "width_records_max": 1}
    assert not road_el.findall(".//border")
    assert len(road_el.findall(".//right/lane/width")) == 2
    actual = [lane_edges_at(road_el, float(s), "right") for s in stations]
    assert np.allclose(actual, expected, atol=1e-9)


def test_exact_point_to_polyline_distance_has_no_sampling_quantization():
    from mapforge.validate.shp_boundary_fidelity import _point_polyline_distance

    line = np.array([[0.0, 0.0], [10.0, 0.0]])
    points = np.array([[5.123, 2.0], [-1.0, 0.0], [11.0, 0.0]])
    distance, inside = _point_polyline_distance(points, line)
    assert np.allclose(distance, [2.0, 1.0, 1.0], atol=1e-12)
    assert inside.tolist() == [True, False, False]


def test_boundary_gate_clips_same_object_continuation_after_leg_end():
    from mapforge.validate.shp_boundary_fidelity import (
        _clip_source_to_target_support,
    )

    target = np.column_stack([np.arange(0.0, 101.0), np.zeros(101)])
    # 同一源边界先覆盖目标走廊，随后掉头并沿平行线返回；返回段仍会投影到
    # 目标线段内部，不能用普通端点外投影规则排除。
    source = np.vstack([
        target,
        np.array([[100.0, -float(y)] for y in np.arange(1.0, 11.0)]),
        np.column_stack([np.arange(99.0, -1.0, -1.0), np.full(100, -10.0)]),
    ])
    clipped = _clip_source_to_target_support(source, target)
    assert len(clipped) < len(source) / 2
    assert np.max(np.abs(clipped[:, 1])) <= 1.0


def test_edge_shape_gate_rejects_snaking_width(tmp_path):
    """参考线完全笔直、section 端点完全闭合，也不能掩盖道路边缘蛇形。"""
    from mapforge.adapters.opendrive.writer import Lane, LaneSection, Road, XodrDoc
    from mapforge.validate.smoothness import audit_file

    doc = XodrDoc("edge-shape-fault")
    road = Road(1, "straight-reference-snaking-edge")
    road.add_geometry("line", 0.0, 0.0, 0.0, 20.0)
    values = [3.5, 10.0, 3.5, 10.0, 3.5]
    for i in range(4):
        length = 5.0
        v0, v1 = values[i], values[i + 1]
        # 两端斜率均为 0：位置和切向都闭合，但 5m 内反复收放，二阶形态失真。
        c = 3.0 * (v1 - v0) / length ** 2
        d = -2.0 * (v1 - v0) / length ** 3
        sec = LaneSection(i * length)
        lane = Lane(-1)
        lane.add_width(v0, 0.0, c, d)
        sec.right.append(lane)
        road.sections.append(sec)
    doc.add_road(road)
    out = tmp_path / "snake.xodr"
    doc.write(out)

    audit = audit_file(out)
    assert audit["kappa_step_max"] == 0.0                 # 旧参考线门禁会放过
    assert audit["lane_edge_step_max"] < 1e-6            # 旧位置门禁也会放过
    assert audit["edge_lateral_second_max"] > 0.40       # 新世界边缘门禁抓住
