# -*- coding: utf-8 -*-
"""SHP→xodr 直转门禁（node4）：junction 结构完整 + 连接路实测几何 + XSD + 连续性。

历史教训：曾经 CLI 的 SHP→xodr 借道 SHP→MAP→xodr（空口消息窄门），产物无 junction、
单一平均车道宽——本测试锁死直转路径的结构底线，防止回退。
"""
from pathlib import Path

import json
import pytest
from lxml import etree

pytestmark = pytest.mark.slow

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def ibd():
    # 与 CLI 正式路径一致：ProfileSource 会按用户字段映射读取，并在 WIDTH=0
    # 等脏数据处使用可信边界证据补宽；绕过 Profile 的 IbdSource 只保留作底层读入器。
    if not (ROOT / "shp_0222-0326" / "IBD_LANE_LINK.shp").exists():
        pytest.skip("original SHP delivery not installed")
    from mapforge.adapters.shp.profile_source import ProfileSource
    return ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")


def test_direct_xodr_node4(ibd, tmp_path):
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops.shp_to_xodr import build_junction_xodr
    from mapforge.validate.planview_check import check_file

    ref = parse_map_xml(str(ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"))
    junc, dist = ibd.find_junction(ref.ref_lon, ref.ref_lat)
    assert dist < 30
    out = tmp_path / "node4.xodr"
    st = build_junction_xodr(ibd, junc, out)
    manifest = st["source_lane_manifest"]
    source_ids = [x["source_lane_id"] for x in manifest["lanes"]]
    assert len(source_ids) == len(set(source_ids)) and len(source_ids) > 50
    assert any(x["policy_class"].endswith("-approach") for x in manifest["lanes"])
    assert any(x["role"] == "junction-via" for x in manifest["lanes"])

    # 结构：进/出口路全建；连接路全部实测几何（node4 无 TOPO 直连缺口）；laneLink 全覆盖
    assert st["roads_enter"] == 4 and st["roads_leave"] == 4
    assert st["connections"] >= 20 and st["conn_g2"] == 0
    assert st["lanelinks"] >= st["connections"]
    # 参考线偏差**不是**保真判据（v1.25：参考线容差放到 1.5m 换曲率不跟噪声抖，
    # 车道由实测横距相对它写出、差量被 laneOffset/width 吸收）——保真改由
    # validate/lane_fidelity 直接比对"写出车道中心 vs 源车道点列"
    assert st["fit_dev_max"] <= 1.5

    root = etree.parse(str(out)).getroot()
    # Raw node4 fork: full-width continuation is not the nearer zero-width
    # branch path. Lock the generated topology/widths, not only matcher units.
    split_lanes = {}
    for lane in root.findall('.//right/lane'):
        source = lane.find("userData[@code='mapforge.source_lane']")
        if source is not None:
            split_lanes.setdefault(source.get('value'), lane)
    continuing = split_lanes['2023081117251325687']
    newborn = split_lanes['2023081117251327605']
    assert continuing.find('link/predecessor').get('id') == '-1'
    assert float(continuing.find('width').get('a')) > 3.5
    assert newborn.find('link/predecessor') is None
    assert float(newborn.find('width').get('a')) < .001
    # 共享物理边界在内部求解，最终无损物化为消费端普遍支持的 lane.width。
    # esmini 3.6 对 lane.border 求值为零，交付文件不得再包含 border。
    assert not root.findall(".//lane/border")
    assert root.findall(".//lane/width")
    assert st.get("border_family_materialized_width_lanes", 0) > 0
    assert len(root.findall("junction")) == 1
    assert len(root.findall("junction/connection")) == st["connections"]
    conn_roads = [r for r in root.findall("road")
                  if r.get("junction") not in (None, "-1")
                  and r.get("name") != "junction_paving"]
    assert len(conn_roads) == st["conn_via"] + st["conn_g2"]
    assert st.get("paving") == "polygon"                 # IBD 交叉口面铺面（路口不露底）
    assert st.get("paving_roads") == 2
    paving = [r for r in root.findall("road") if r.get("name") == "junction_paving"]
    assert all(len(r.findall("lanes/laneSection")) == 1 for r in paving)
    from mapforge.validate.smoothness import surface_continuity
    surf = surface_continuity(root)
    assert surf["paving_road_components_max"] == 1
    assert surf["paving_components"] == 1
    assert surf["paving_holes_gt1cm2"] == 0
    assert surf["paving_leg_overlap_min"] >= 0.2
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
    from mapforge.validate.smoothness import audit_file, curvature_audit, route_continuity
    rc = route_continuity(etree.parse(str(out)).getroot())
    assert rc
    assert max(r["gap_in"] for r in rc) < 0.01
    assert max(r["gap_out"] for r in rc if not math.isnan(r["gap_out"])) < 0.01
    assert max(r["dh_in_deg"] for r in rc) < 0.1

    # 门禁：全网 G2（参考线结点曲率连续——SolveG2 过渡后应为机器精度）
    au = audit_file(out)
    assert au["kappa_step_max"] < 1e-6
    assert au["lane_edge_step_max"] < 0.05
    assert au["edge_lateral_second_max"] <= 0.40
    assert au["outer_curvature_max"] <= 0.25
    assert au["outer_edge_heading_step_max_deg"] <= 5.0
    from mapforge.validate.shp_boundary_fidelity import evaluate_shp_outer_edges
    bf = evaluate_shp_outer_edges(ROOT / "shp_0222-0326", out)
    # Four main legs plus any legitimate directional carriageway splits caused
    # by asymmetric source coverage must all participate in fidelity checks.
    assert bf["paired_roads"] == 4 + st.get("directional_carriageway_splits", 0)
    assert bf["source_to_target"]["median_m"] <= 0.30
    assert bf["source_to_target"]["p95_m"] <= 0.65
    assert bf["target_to_source"]["median_m"] <= 0.30
    assert bf["target_to_source"]["p95_m"] <= 0.65
    assert "excluded_unsupported_by_code" in bf

    # source limits remain unchanged, even when generated geometry fails G11-D.
    assert st.get("border_family_routeable_speed_caps", 0) == 0
    checked = 0
    for lane in root.findall(".//lane"):
        item = lane.find("userData[@code='mapforge.provenance/v1']")
        if item is None:
            continue
        provenance = json.loads(item.get("value", "{}"))
        assert provenance.get("speed_adjustment") != "dynamic-safety-cap"
        sid = lane.find("userData[@code='mapforge.source_lane']")
        if sid is not None:
            source = ibd.lane(sid.get('value'))
            if source is not None and source.max_speed_kmh:
                assert all(float(s.get('max'))*3.6 == pytest.approx(source.max_speed_kmh, abs=1e-5)
                           for s in lane.findall('speed'))
                assert lane.findall('speed')
                checked += 1
    assert checked > 20

    from mapforge.validate.g11 import audit_file as audit_g11
    g11 = audit_g11(out, ROOT / "profiles/validation/g11-opendrive-v1.draft.yaml")
    # The current default is still a rejected geometry candidate, not a
    # release. Fixing geometry must eventually replace this negative baseline.
    assert g11['groups']['G11-D']['status'] == 'FAIL'
    cq = curvature_audit(root)
    assert cq["leg"]["seg_min_len"] >= 3.0             # 禁止 0.x–2.xm 碎段假平滑
    assert cq["leg"]["sharpness_max"] <= 0.0045
    assert cq["leg"]["flips_per_100m_max"] <= 8.0


def test_connect_mode_fill(ibd, tmp_path):
    """转向补全三档：data 只用源 TOPO（默认不发明拓扑）；full 补出可行车连接且门禁不降级。

    动机：源数据的车道拓扑可能整片缺录/失配（IBD TOPO 上游缺录、MAP connectsTo ID 失配），
    需要按几何补全的模式；补出的连接必须与数据连接同等平滑（同一套 G2 机制）。"""
    import math
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops.shp_to_xodr import build_junction_xodr
    from mapforge.validate.smoothness import audit_file, route_continuity

    ref = parse_map_xml(str(ROOT / "v2x_map_xml" / "map凤苑路-金玥路node4.xml"))
    junc, _d = ibd.find_junction(ref.ref_lon, ref.ref_lat)
    st_d = build_junction_xodr(ibd, junc, tmp_path / "d.xodr", connect_mode="data")
    st_f = build_junction_xodr(ibd, junc, tmp_path / "f.xodr", connect_mode="full")
    assert st_d.get("conn_filled", 0) == 0                   # 默认档绝不发明拓扑
    assert st_f["conn_filled"] >= 20                         # 全连接实补
    assert st_f["connections"] > st_d["connections"]

    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    root = etree.parse(str(tmp_path / "f.xodr"))
    assert schema.validate(root), schema.error_log
    au = audit_file(tmp_path / "f.xodr")
    assert au["kappa_step_max"] < 1e-6                       # 补出的连接同样全网 G2
    rc = route_continuity(etree.parse(str(tmp_path / "f.xodr")).getroot())
    assert max(r["gap_in"] for r in rc) < 0.01
    assert max(r["gap_out"] for r in rc if not math.isnan(r["gap_out"])) < 0.01


def test_parallel_source_links_form_one_physical_leg(ibd, tmp_path):
    """node13 南口的两个并行 2-lane 出口组必须合并为一条双向 leg。

    旧的一对一 ROADLINK 配对把第二组写成 road 30，画面成为一条独立单边细路；
    同时较长出口覆盖到约 125m、进口只覆盖约 42m，缺测侧应以稳定断面补齐，
    不能把 12m 的交叉零宽 transition 放大成连续鼓包。
    """
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops.shp_to_xodr import build_junction_xodr

    ref = parse_map_xml(str(ROOT / "v2x_map_xml" / "map凤阁路-金玥路node13.xml"))
    junc, _dist = ibd.find_junction(ref.ref_lon, ref.ref_lat)
    out = tmp_path / "node13.xodr"
    st = build_junction_xodr(ibd, junc, out)
    assert st.get("parallel_leave_groups") == 2
    assert st.get("reference_extended_from_opposite_m", 0.0) > 70.0
    assert st.get("source_transition_links_bypassed") == 1

    root = etree.parse(str(out)).getroot()
    assert root.find("road[@id='30']") is None
    south = root.find("road[@id='10']")
    assert south is not None and float(south.get("length")) > 120.0
    first = south.find("lanes/laneSection")
    assert len(first.findall("right/lane[@type='driving']")) == 3
    assert len(first.findall("left/lane[@type='driving']")) == 4

    # 真实回归：旧三段解虽过宽门限，via 2024010415270546470 的 P95
    # 仍达 1.1m。四段长回旋线能保留原始转弯，不得再以少一段为由选回旧解。
    from mapforge.validate.lane_fidelity import evaluate_g8
    gate = evaluate_g8(out, st["source_lane_manifest"],
                       ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml")
    row = next(x for x in gate["per_lane"]
               if x["source_lane_id"] == "2024010415270546470")
    assert row["source_to_target"]["p95_m"] < 0.40
    assert row["target_to_source"]["p95_m"] < 0.40
    geoms = root.find(f"road[@id='{row['target']['road_id']}']").findall("planView/geometry")
    assert len(geoms) <= 5
    assert min(float(g.get("length")) for g in geoms) >= 6.0 - 1e-6


def test_lane_fidelity_and_no_hairpin(ibd, tmp_path):
    """真正的保真/平滑判据（替代"参考线贴得紧"这个间接指标）：
    ① 写出车道中心 vs 源车道点列的横向偏差；② leg 参考线不得出现发夹弯。

    背景：v1.25 把参考线容差放到 1.5m 以让曲率不跟数字化噪声抖（node18 蛇行
    10.3→4.9 次/100m），代价必须由这两条直接判据兜住——否则就是在悄悄挪路。"""
    import numpy as np
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.ops.shp_to_xodr import build_junction_xodr, _proj
    from mapforge.validate.lane_fidelity import paired_deviation
    from mapforge.validate.smoothness import _geoms, curvature_audit

    ref = parse_map_xml(str(ROOT / "v2x_map_xml" / "map凤苑路-金剑路node18.xml"))
    junc, _d = ibd.find_junction(ref.ref_lon, ref.ref_lat)
    out = tmp_path / "n18.xodr"
    st = build_junction_xodr(ibd, junc, out)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    src_lanes = {l.lane_pid: _proj(l.geometry, lat0, lon0)
                 for pid in set(junc.enter_roads) | set(junc.leave_roads)
                 for l in ibd.lanes_of(pid) if l.geometry.shape[0] >= 2}

    root = etree.parse(str(out)).getroot()
    fid = paired_deviation(root, src_lanes)
    assert fid["matched"] == len(src_lanes) and not fid["missing"]
    assert fid["source_to_target"]["median"] < 0.6
    assert fid["source_to_target"]["p95"] < 1.5
    eligible_source_ids = {
        item["source_lane_id"] for item in st["source_lane_manifest"]["lanes"]
        if item.get("comparison", {}).get("eligible")
    }
    eligible_medians = [
        fid["per_lane"][source_id]["source_to_target"]["median"]
        for source_id in eligible_source_ids if source_id in fid["per_lane"]
    ]
    assert eligible_medians
    assert max(eligible_medians) < 0.6

    # 正式 G8：桥接 apron 不冒充实测 via；无法保留来源的拓扑冲突必须显式排除并进入 review。
    from mapforge.validate.lane_fidelity import evaluate_g8
    gate = evaluate_g8(out, st["source_lane_manifest"],
                       ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml")
    via_rows = [x for x in gate["per_lane"] if x["policy_class"] == "shp.field-via"]
    assert via_rows
    # 正常弯线的 7m 弦方向并不是端点切线。已通过少段 G2 + 来源容差的
    # 候选必须进入 G8，禁止以弦方向差为由绕过源数据验收。
    assert st.get("conn_source_chord_heading_warning", 0) > 0
    assert not any(x["code"] == "source-end-state-conflict" for x in gate["exclusions"])
    assert gate["status"] == "PASS"
    assert max(x["source_to_target"]["median_m"] for x in via_rows) < 0.6
    # node18 的最坏可比转弯原 P95 约 1.36m；优先在严格来源容差内
    # 选少段解后约 0.21m。既锁定保真改善，也防止用米级碎段实现。
    improved = next(x for x in via_rows
                    if x["source_lane_id"] == "2024010516362469683")
    assert improved["source_to_target"]["p95_m"] < 0.35
    assert improved["target_to_source"]["p95_m"] < 0.35
    geoms = root.find(f"road[@id='{improved['target']['road_id']}']").findall("planView/geometry")
    assert len(geoms) <= 5
    assert min(float(g.get("length")) for g in geoms) >= 6.0 - 1e-6
    assert any(x["code"] == "minimal-chain-source-fidelity-unmet"
               and x.get("support_kind") == "source-topology-gap"
               for x in gate["exclusions"])
    assert not gate["issues"]["missing_source_ids"]

    # 故障注入：把一条来源 lane 的 provenance 绑到错误对象，G8 必须报告缺失；
    # 旧“到全路面最近距离”会被相邻车道掩盖，无法发现这种 lane 绑错。
    victim = next(iter(src_lanes))
    for ud in root.findall(f".//userData[@code='mapforge.source_lane'][@value='{victim}']"):
        ud.set("value", "fault-injected-wrong-lane")
    bad = paired_deviation(root, src_lanes)
    assert victim in bad["missing"]

    kmax = max((max(abs(g[5]), abs(g[6])) for rd in root.findall("road")
                if rd.get("junction") in (None, "-1") for g in _geoms(rd)), default=0.0)
    assert kmax < 1 / 25                                  # leg 无发夹弯（R≥25m）
    cq = curvature_audit(root)["leg"]
    assert cq["seg_min_len"] >= 3.0
    assert cq["sharpness_max"] <= 0.0045
    assert cq["flips_per_100m_max"] <= 8.0
