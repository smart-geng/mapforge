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
    assert max(x["source_to_target"]["median"] for x in fid["per_lane"].values()) < 0.6

    # 正式 G8：桥接 apron 不冒充实测 via；无法保留来源的拓扑冲突必须显式排除并进入 review。
    from mapforge.validate.lane_fidelity import evaluate_g8
    gate = evaluate_g8(out, st["source_lane_manifest"],
                       ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml")
    via_rows = [x for x in gate["per_lane"] if x["policy_class"] == "shp.field-via"]
    assert via_rows
    assert max(x["source_to_target"]["median_m"] for x in via_rows) < 0.6
    assert any(x["code"] == "source-topology-gap-bridge" for x in gate["exclusions"])
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
