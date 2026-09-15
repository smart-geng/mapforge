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


def test_continuous_polyline_distance_has_no_sampling_phase_floor():
    from mapforge.ops.map_to_xodr import _directed_point_to_polyline

    # 同一条直线用 1m 与错相的 0.5m 采样；点云 KDTree 会误报约 0.5m，
    # 连续点到线段距离必须严格为零。
    coarse = np.column_stack([np.arange(0.0, 10.1, 1.0), np.zeros(11)])
    shifted = np.column_stack([np.arange(0.5, 10.0, 1.0), np.zeros(10)])
    assert np.max(_directed_point_to_polyline(shifted, coarse)) < 1e-12


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
    # A nominal MAP width cannot be reused after source-centre boundary fitting.
    # Verify actual written size at both ends, not just the centerline seam.
    from mapforge.validate.junction_edges import endpoint_edges
    root = etree.parse(str(out)).getroot()
    roads = {r.get('id'): r for r in root.findall('road')}
    for r in roads.values():
        if r.get('junction') == '-1' or r.get('name') == 'junction_paving':
            continue
        lane = r.find("lanes/laneSection/right/lane[@id='-1']")
        for role, contact in (('predecessor', 'start'), ('successor', 'end')):
            link = r.find('link/' + role)
            other_id = int(lane.find('link/' + role).get('id'))
            a = endpoint_edges(r, -1, contact, True)
            b = endpoint_edges(roads[link.get('elementId')], other_id, link.get('contactPoint'), True)
            span = lambda edges: np.hypot(edges['left']['x']-edges['right']['x'],
                                         edges['left']['y']-edges['right']['y'])
            assert span(a) == pytest.approx(span(b), abs=1e-7)
    from mapforge.validate.planview_check import check_file
    assert not check_file(str(out))["violations"]
    from mapforge.validate.smoothness import audit_file, curvature_audit
    audit = audit_file(out)
    assert audit["kappa_step_max"] < 1e-6                # 全网 G2 门禁
    # 最终世界坐标边缘门禁：参考线连续不等于道路边缘平滑。MAP 分段断面必须
    # 共用站点导数，禁止“位置接上、切向仍折”的假连续。
    assert audit["edge_lateral_second_max"] <= 0.40
    assert audit["outer_curvature_max"] <= 0.25
    assert audit["outer_edge_heading_step_max_deg"] <= 5.0
    cq = curvature_audit(etree.parse(str(out)).getroot())["leg"]
    assert cq["seg_min_len"] >= 3.0                       # 禁止极小段假平滑
    assert cq["sharpness_max"] <= 0.0045
    assert cq["flips_per_100m_max"] <= 8.0


def _worst_seam_kappa_gap(out) -> float:
    """检查真实 laneLink 两端，禁止在任意偏移范围内搜索一个能通过的曲率。"""
    from mapforge.validate.smoothness import junction_lane_interfaces
    root = etree.parse(str(out)).getroot()
    rows = junction_lane_interfaces(root)
    assert len(rows) == 2*len(root.findall('junction/connection/laneLink'))
    assert not any('error' in x for x in rows)
    assert max(x['position_m'] for x in rows) < .01
    assert max(x['heading_deg'] for x in rows) < .1
    return max(x['curvature_per_m'] for x in rows)


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
    n_pave = st.get("paving_roads", 1) if st.get("paving") in ("mouth-apron", "hull") else 0
    assert len(root.findall("road")) == st["links"] + st["conn_roads"] + n_pave
    assert root.findall("road/lanes/laneSection/left/lane")  # 左侧车道确实存在
    provenance = [x.get("value") for x in root.findall(
        ".//userData[@code='mapforge.provenance/v1']")]
    assert any('"exclusion_code":"mirror-no-source-geometry"' in x for x in provenance)
    assert st.get("paving") == "mouth-apron"
    assert st.get("paving_roads") == 2
    assert any('"support_kind":"mouth-envelope-rounded"' in x for x in provenance)
    # 单节点帧缺出口几何时，整幅出口必须与同 leg 进口逐车道严格左右镜像；
    # 不得再按 connectsTo 的最大目标车道号裁少或补多车道。
    for rd in root.findall("road"):
        if not 10 <= int(rd.get("id")) < 30:
            continue
        for sec in rd.findall("lanes/laneSection"):
            left = {int(x.get("id")): x for x in sec.findall("left/lane")
                    if x.get("type") != "median"}
            right = {int(x.get("id")): x for x in sec.findall("right/lane")}
            assert len(left) == len(right)
            for k in range(1, len(right) + 1):
                lw, rw = left[k].find("width"), right[-k].find("width")
                assert all(abs(float(lw.get(a)) - float(rw.get(a))) < 1e-12
                           for a in "abcd")
    manifest_ids = [x["source_lane_id"] for x in st["source_lane_manifest"]["lanes"]]
    assert manifest_ids and len(manifest_ids) == len(set(manifest_ids))
    _validate(out)
    from mapforge.validate.smoothness import surface_continuity
    surf = surface_continuity(root)
    assert surf["paving_road_components_max"] == 1
    assert surf["paving_components"] == 1
    assert surf["paving_holes_gt1cm2"] == 0
    assert surf["paving_leg_overlap_min"] >= 0.2
    assert _worst_seam_kappa_gap(out) < 1e-6                 # 车道级曲率连续（G2 接缝）


def test_known_id_mismatch_skipped_not_invented(tmp_path):
    st, _out, n_xml = _convert("map凤苑路-金坪路NODE5.xml", tmp_path)
    assert st["skipped"] == 1                                # west lane2 → 20701（无此上游）
    assert st["connections"] == n_xml - 1


def test_lane_profile_clips_only_reference_endpoint_overhang():
    from mapforge.ops.map_to_xodr import _lane_profile

    ref = np.column_stack([np.arange(0.0, 101.0, 0.5), np.zeros(202)])
    tang = np.tile([1.0, 0.0], (len(ref), 1))
    points = np.array([[-20.0, 3.0], [0.0, 3.0], [50.0, 3.0],
                       [100.0, 3.0], [120.0, 3.0]])

    (stations, offsets), support = _lane_profile(
        ref, tang, points, return_points=True)

    assert stations.tolist() == [0.0, 50.0, 100.0]
    assert np.allclose(offsets, 3.0)
    assert np.allclose(support[:, 0], [0.0, 50.0, 100.0])


def test_missing_exit_is_exact_left_right_mirror():
    """镜像必须复制同 leg 进口断面的逐车道宽度和变化率，而非中位宽堆叠。"""
    from mapforge.ops.map_to_xodr import _mirror_bounds

    # 右侧内缘为 0，两个进口车道宽分别 3.2m、3.6m；边界斜率也不相同。
    left_b, left_m = _mirror_bounds(
        ([0.0, -3.2, -6.8], [0.1, 0.2, 0.15]), lane_count=2,
        fallback_width=3.5,
    )
    assert np.allclose(left_b, [0.0, 3.2, 6.8])
    assert np.allclose(left_m, [0.1, 0.0, 0.05])

    # 目标编号多出一条时，仅最外侧额外车道使用名义宽；已有两条仍为精确镜像。
    extra_b, extra_m = _mirror_bounds(
        ([1.0, -2.2, -5.8], [0.0, 0.1, -0.05]), lane_count=3,
        fallback_width=3.5,
    )
    assert np.allclose(extra_b, [1.0, 4.2, 7.8, 11.3])
    assert np.allclose(extra_m, [0.0, -0.1, 0.05, 0.05])


def test_rounded_mouth_apron_keeps_all_road_openings():
    from shapely.geometry import Point, Polygon
    from mapforge.ops.map_to_xodr import _rounded_mouth_apron, _shift

    mouths = [
        {"pose": (-20.0, 0.0, 0.0), "right_t": -7.0, "left_t": 7.0},
        {"pose": (0.0, -20.0, np.pi / 2), "right_t": -7.0, "left_t": 7.0},
        {"pose": (20.0, 0.0, np.pi), "right_t": -7.0, "left_t": 7.0},
        {"pose": (0.0, 20.0, -np.pi / 2), "right_t": -7.0, "left_t": 7.0},
    ]
    poly = Polygon(_rounded_mouth_apron(mouths))
    assert poly.is_valid and poly.area > 500.0
    assert poly.covers(Point(0.0, 0.0))
    # 每个道路口部的左右角点都由 overlap cap 保住，不会因圆角切出三角洞。
    for m in mouths:
        for t in (m["right_t"], m["left_t"]):
            p = _shift(m["pose"], t)
            assert poly.buffer(1e-8).covers(Point(p[0], p[1]))


def test_offcenter_mouth_apron_keeps_road_tangents():
    """偏心十字口：朝几何中心的向量不是道路切线，旧实现会产生 5--20° 尖接。"""
    from mapforge.ops.map_to_xodr import _rounded_mouth_apron, _shift
    mouths = [
        {"pose": (-30.0, -5.0, 0.0), "right_t": -5., "left_t": 5.},
        {"pose": (30.0, 8.0, np.pi), "right_t": -5., "left_t": 5.},
        {"pose": (7.0, 30.0, -np.pi/2), "right_t": -5., "left_t": 5.},
        {"pose": (-9.0, -30.0, np.pi/2), "right_t": -5., "left_t": 5.},
    ]
    poly = _rounded_mouth_apron(mouths)
    assert poly is not None
    ring = poly[:-1] if np.allclose(poly[0], poly[-1]) else poly
    for m in mouths:
        tangent = np.array([np.cos(m["pose"][2]), np.sin(m["pose"][2])])
        for side in ("right", "left"):
            p = np.asarray(_shift(m["pose"], m[side + "_t"])[:2])
            i = int(np.argmin(np.linalg.norm(ring - p, axis=1)))
            assert np.linalg.norm(ring[i] - p) < 1e-8
            vectors = ring[[(i-1) % len(ring), (i+1) % len(ring)]] - p
            vectors /= np.linalg.norm(vectors, axis=1)[:, None]
            assert np.max(np.abs(vectors @ tangent)) > np.cos(np.deg2rad(0.6))


def test_written_mouth_includes_width_and_offset_derivatives():
    from mapforge.adapters.opendrive.writer import Road, Lane, LaneSection
    from mapforge.ops.map_to_xodr import _written_road_mouth
    road = Road(1).add_geometry("line", 0, 0, 0, 10).add_offset(0, 0, 0.1)
    sec = LaneSection(0)
    sec.right.append(Lane(-1).add_width(3.5, 0.2, 0.01))
    sec.left.append(Lane(1).add_width(3.0))
    road.sections.append(sec)
    got = _written_road_mouth(road)
    assert got["right_t"] == pytest.approx(-5.5)
    assert got["left_t"] == pytest.approx(4.0)
    assert got["right_heading"] == pytest.approx(np.arctan(-0.3))
    assert got["left_heading"] == pytest.approx(np.arctan(0.1))
    assert got["right_curvature"] == pytest.approx(-0.02 / (1 + 0.3**2)**1.5)


def test_paving_axis_uses_opposite_mouths_not_square_pca_diagonal():
    from mapforge.ops.map_to_xodr import _mouth_apron_axis, _mouth_apron_axes

    mouths = [
        {"pose": (-30.0, 0.0, 0.0)}, {"pose": (30.0, 0.0, np.pi)},
        {"pose": (0.0, -20.0, np.pi / 2)}, {"pose": (0.0, 20.0, -np.pi / 2)},
    ]
    axis = _mouth_apron_axis(mouths)
    axis = axis / np.linalg.norm(axis)
    assert abs(axis[0]) > 1 - 1e-12 and abs(axis[1]) < 1e-12
    axes = _mouth_apron_axes(mouths)
    assert len(axes) == 2
    unit = [x / np.linalg.norm(x) for x in axes]
    assert abs(float(np.dot(unit[0], unit[1]))) < 1e-12


def test_real_exits_from_multinode_frame(tmp_path):
    """合帧包含同号异 region 节点时，只绑定完整 (region,node) 与 upstream 均匹配的出口。"""
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
    node = next(n for n in nodes if (n.region, n.node_id) == (500, 4))
    out = tmp_path / "m.xodr"
    st = build_xodr(node, out, neighbors=[n for n in nodes if n is not node])
    assert st["exit_real"] == 1 and st["exit_mirror"] == 3
    contexts = [x for x in st["source_lane_manifest"]["source_contexts"]
                if x["role"] == "neighbor-real-exit"]
    assert contexts == [{"region": 500, "node_id": 18, "role": "neighbor-real-exit"}]
    manifest_ids = [x["source_lane_id"] for x in st["source_lane_manifest"]["lanes"]]
    assert all(":from:" in x for x in manifest_ids)
    assert not any(x.startswith("map:3:3:") for x in manifest_ids)
    root = etree.parse(str(out)).getroot()
    real_departures = [x.get("value") for x in root.findall(
        ".//userData[@code='mapforge.provenance/v1']")
        if '"policy_class":"map.point-list-departure"' in x.get("value", "")]
    assert real_departures and all('"support_s":[' in x for x in real_departures)
    assert st["skipped"] == 0
    # west 链 s255-298 有数字化噪声（R=10m 级振荡），曲率封顶主动平滑——
    # 对原始（含噪）顶点的偏差 0.68m 是修复的代价而非回归；封顶行为本身锁死
    assert st.get("refit_smoothed", 0) >= 1
    assert st["fit_dev_max"] <= 1.5                          # v1.25 来源偏差硬上限
    _validate(out)
    assert _worst_seam_kappa_gap(out) < 1e-6
