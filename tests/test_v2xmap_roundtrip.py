# -*- coding: utf-8 -*-
"""v2xmap 编解码回环：XML 读写全等 + 7 路口 UPER 双模式。"""
from pathlib import Path

import pytest

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.adapters.v2xmap.xml_writer import node_to_xml
from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val

SRC = Path(r"F:\MapFactory\v2x_map_xml")
XMLS = sorted(SRC.glob("map*.xml"))


@pytest.mark.parametrize("xml", XMLS, ids=[x.stem[-8:] for x in XMLS])
def test_xml_write_read_roundtrip(xml, tmp_path):
    n1 = parse_map_xml(str(xml))
    p = tmp_path / "rt.xml"
    p.write_text(node_to_xml(n1), encoding="utf-8")
    assert parse_map_xml(str(p)) == n1


@pytest.mark.parametrize("xml", XMLS, ids=[x.stem[-8:] for x in XMLS])
def test_uper_roundtrip_absolute(xml):
    import msglayer_draft as m
    mf = m.MsgLayerDraft.MessageFrame
    val = node_to_messageframe_val(parse_map_xml(str(xml)), "absolute")
    mf.set_val(val)
    buf = mf.to_uper()
    mf.from_uper(buf)
    assert mf.get_val() == val
    assert 300 < len(buf) < 2000                                  # 大小基线量级（v1.6 实测 678–1373）


def test_uper_offset_within_1lsb():
    import msglayer_draft as m
    mf = m.MsgLayerDraft.MessageFrame
    node = parse_map_xml(str(XMLS[0]))
    val = node_to_messageframe_val(node, "offset")
    mf.set_val(val)
    mf.from_uper(mf.to_uper())
    out = mf.get_val()[1]["nodes"][0]
    ref = out["refPos"]
    for lk_o, lk_i in zip(out["inLinks"], val[1]["nodes"][0]["inLinks"]):
        for p_o, p_i in zip(lk_o.get("points", []), lk_i.get("points", [])):
            br_o, xy_o = p_o["posOffset"]["offsetLL"]
            br_i, xy_i = p_i["posOffset"]["offsetLL"]
            lon_o = xy_o["lon"] + (ref["long"] if br_o != "position-LatLon" else 0)
            lon_i = xy_i["lon"] + (ref["long"] if br_i != "position-LatLon" else 0)
            assert abs(lon_o - lon_i) <= 1
