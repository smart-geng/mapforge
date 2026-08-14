# -*- coding: utf-8 -*-
"""金凤 7 路口两条 xodr 主线批量再生成（真实数据全量）。

- out/direct_xodr/<node>.xodr：SHP(IBD)→xodr 直转（Profile 引擎默认源；
  路口用对应 MAP XML 的参考点定位）；
- out/m2x/<node>.xodr：MAP XML→xodr（全部 7 帧合帧共享邻居 → 真实出口优先，镜像兜底）。

用法：.venv/Scripts/python scripts/gen_all.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml, parse_map_xml_all  # noqa: E402
from mapforge.ops.map_to_xodr import build_xodr                                   # noqa: E402
from mapforge.ops.shp_to_xodr import build_junction_xodr                          # noqa: E402


def shp_source():
    from mapforge.adapters.shp.profile_source import ProfileSource
    try:
        return ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    except FileNotFoundError:
        from mapforge.adapters.shp.ibd_reader import IbdSource
        return IbdSource(str(ROOT / "shp_0222-0326"))


def main():
    xmls = sorted((ROOT / "v2x_map_xml").glob("map*.xml"))
    names = {p: re.search(r"(node\d+|NODE\d+)", p.stem, re.I).group(1) for p in xmls}

    # —— MAP→xodr：全帧互为邻居（真实出口优先） ——
    nodes = []
    for p in xmls:
        nodes.extend(parse_map_xml_all(str(p)))
    (ROOT / "out/m2x").mkdir(parents=True, exist_ok=True)
    for p in xmls:
        main_node = parse_map_xml(str(p))
        neigh = [n for n in nodes if n.node_id != main_node.node_id]
        st = build_xodr(main_node, ROOT / "out/m2x" / f"{names[p]}.xodr", neighbors=neigh)
        print(f"m2x {names[p]:7s} conn={st['connections']} real={st.get('exit_real', 0)} "
              f"mirror={st.get('exit_mirror', 0)} skip={st['skipped']}")

    # —— SHP→xodr 直转 ——
    src = shp_source()
    (ROOT / "out/direct_xodr").mkdir(parents=True, exist_ok=True)
    for p in xmls:
        ref = parse_map_xml(str(p))
        junc, dist = src.find_junction(ref.ref_lon, ref.ref_lat)
        if junc is None or dist > 50:
            print(f"shp {names[p]:7s} SKIP（无对应路口，dist={dist:.0f}m）")
            continue
        st = build_junction_xodr(src, junc, ROOT / "out/direct_xodr" / f"{names[p]}.xodr")
        print(f"shp {names[p]:7s} enter={st['roads_enter']} leave={st['roads_leave']} "
              f"conn={st['connections']} bridged={st.get('conn_bridged', 0)} "
              f"tapers={st.get('tapers', 0)} dev={st['fit_dev_max']:.2f}")


if __name__ == "__main__":
    main()
