# -*- coding: utf-8 -*-
"""M0 作业①收尾：编译产物生成 Python 模块 + 最小 MessageFrame(mapFrame) UPER 编码回环。

用 node16 真实数据填最小 MapData → UPER 编码 → 解码 → 值比对。
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

ASN = ROOT / "mapforge" / "adapters" / "v2xmap" / "asn" / "msglayer-draft.asn"
GEN = ROOT / "mapforge" / "adapters" / "v2xmap" / "asn" / "msglayer_draft.py"
OUT = ROOT / "out"


def main():
    from pycrate_asn1c.asnproc import compile_text, generate_modules, PycrateGenerator

    compile_text(ASN.read_text(encoding="utf-8"))
    generate_modules(PycrateGenerator, str(GEN))
    print("生成运行时模块：", GEN)

    sys.path.insert(0, str(GEN.parent))
    import msglayer_draft  # noqa

    mf = msglayer_draft.MsgLayerDraft.MessageFrame
    # node16 真实数据的最小 MapData（refPos 1e-7°，含一条含 movements 的 Link 骨架）
    val = ("mapFrame", {
        "msgCnt": 18,
        "nodes": [{
            "name": "node_21801",
            "id": {"region": 500, "id": 21801},
            "refPos": {"lat": 295161415, "long": 1063183590},
            "inLinks": [{
                "name": "west",
                "upstreamNodeId": {"region": 500, "id": 21301},
                "linkWidth": 650,
                "movements": [{"remoteIntersection": {"region": 500, "id": 21803},
                               "phaseId": 25}],
                "lanes": [{"laneID": 1, "laneWidth": 350,
                           "maneuvers": (int("010000000000", 2), 12)}],
            }],
        }],
    })
    mf.set_val(val)
    buf = mf.to_uper()
    print("UPER 编码成功：%d 字节  hex[:32]=%s" % (len(buf), buf[:32].hex()))

    mf2 = msglayer_draft.MsgLayerDraft.MessageFrame
    mf2.from_uper(buf)
    val2 = mf2.get_val()
    ok = val2 == mf.get_val()
    print("解码回读比对：", "PASS" if ok else "FAIL")
    if not ok:
        print("回读值：", val2)

    with open(OUT / "asn_extract_report.md", "a", encoding="utf-8") as f:
        f.write("\n\n## UPER 回环烟雾测试\n\n")
        f.write(f"- 运行时模块：{GEN.name}（pycrate 生成）\n")
        f.write(f"- 最小 MessageFrame(mapFrame, node16 数据) → UPER {len(buf)} 字节 → 解码比对 "
                f"{'PASS' if ok else 'FAIL'}\n")


if __name__ == "__main__":
    main()
