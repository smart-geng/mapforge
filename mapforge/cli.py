# -*- coding: utf-8 -*-
"""mapforge CLI 骨架（M0）：preview / encode / decode / validate-xodr。

用法（venv 内）：
  python -m mapforge.cli preview  map.xml  [-o out.geojson]
  python -m mapforge.cli encode   map.xml  [-o out.uper] [--mode absolute|offset]
  python -m mapforge.cli decode   map.uper
  python -m mapforge.cli validate-xodr file.xodr [--xsd OpenDRIVE_1.5M.xsd]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import typer

app = typer.Typer(add_completion=False, help="mapforge — 地图格式转换工厂（M0 骨架）")

_ROOT = Path(__file__).resolve().parent.parent
_ASN_DIR = _ROOT / "mapforge" / "adapters" / "v2xmap" / "asn"


def _msgframe_type():
    sys.path.insert(0, str(_ASN_DIR))
    import msglayer_draft  # noqa
    return msglayer_draft.MsgLayerDraft.MessageFrame


@app.command()
def preview(xml_path: Path, out: Path = typer.Option(None, "-o", help="输出 GeoJSON 路径")):
    """MAP XML → GeoJSON 预览（refPos/Link/Lane，含 phase、maneuvers 属性）。"""
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.report.preview_geojson import to_geojson
    node = parse_map_xml(str(xml_path))
    gj = to_geojson(node)
    out = out or xml_path.with_suffix(".geojson")
    out.write_text(json.dumps(gj, ensure_ascii=False, indent=1), encoding="utf-8")
    typer.echo(f"OK ({node.region},{node.node_id}) {len(gj['features'])} features -> {out}")


@app.command()
def encode(xml_path: Path,
           out: Path = typer.Option(None, "-o", help="输出 .uper 路径"),
           mode: str = typer.Option("absolute", help="点列编码：absolute | offset")):
    """MAP XML → UPER 帧（含编码回环自校验）。"""
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val
    node = parse_map_xml(str(xml_path))
    val = node_to_messageframe_val(node, mode)
    mf = _msgframe_type()
    mf.set_val(val)
    buf = mf.to_uper()
    mf.from_uper(buf)                          # 回环自校验（结构级）
    out = out or xml_path.with_suffix(".uper")
    out.write_bytes(buf)
    typer.echo(f"OK ({node.region},{node.node_id}) mode={mode} {len(buf)} bytes -> {out}")


@app.command()
def decode(uper_path: Path):
    """UPER 帧 → 结构摘要（消息类型/节点/Link/车道/点数）。"""
    mf = _msgframe_type()
    mf.from_uper(uper_path.read_bytes())
    kind, body = mf.get_val()
    if kind != "mapFrame":
        typer.echo(f"{kind}（非 MAP，摘要略）")
        raise typer.Exit()
    for nd in body["nodes"]:
        n_link = len(nd.get("inLinks", []))
        n_lane = sum(len(lk.get("lanes", [])) for lk in nd.get("inLinks", []))
        n_pts = sum(len(lk.get("points", [])) + sum(len(ln.get("points", [])) for ln in lk.get("lanes", []))
                    for lk in nd.get("inLinks", []))
        typer.echo(f"mapFrame node=({nd['id'].get('region')},{nd['id']['id']}) "
                   f"links={n_link} lanes={n_lane} points={n_pts} msgCnt={body['msgCnt']}")


@app.command()
def convert(input_path: Path,
            to: str = typer.Option(..., "--to", help="目标格式: geojson | uper | map-xml | map(三视图) | xodr"),
            out: Path = typer.Option(None, "-o", help="输出路径/目录"),
            mode: str = typer.Option("absolute", help="UPER 点列编码 absolute|offset"),
            like: Path = typer.Option(None, help="IBD SHP 源必填：现网参考 XML（定位路口/配对/phase 演示通道）"),
            junction: str = typer.Option(None, help="xodr 源：junction id（缺省自动选最大十字）"),
            region: int = typer.Option(500), node_id: int = typer.Option(9901),
            connect: str = typer.Option(None, help="MAP→xodr 附加 G2 连接路对，如 west-north,south-west"),
            allow_no_phase: bool = typer.Option(False, "--allow-no-phase",
                                                help="显式降级：允许交付无 phase 的信控路口 MAP（默认红线阻断）")):
    """统一转换入口：MAP XML / OpenDRIVE / IBD SHP 目录 → geojson / uper / map-xml / map / xodr。"""
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.report.preview_geojson import to_geojson

    # —— 源 → MapNode ——
    fmt, profile, pipe = "v2xmap-xml", "jinfeng-dialect", None
    if input_path.is_dir():                                  # IBD SHP 目录
        if like is None:
            typer.echo("IBD SHP 源需要 --like <现网参考XML>")
            raise typer.Exit(1)
        from mapforge.adapters.shp.ibd_reader import IbdSource
        from mapforge.ops.shp_to_map import rebuild_from_ibd, phase_table_from_xml, id_table_from_xml
        ref = parse_map_xml(str(like))
        node, summary, _ = rebuild_from_ibd(
            IbdSource(str(input_path)), ref.ref_lon, ref.ref_lat,
            region=ref.region, node_id=ref.node_id,
            phase_table=phase_table_from_xml(ref),
            id_table=id_table_from_xml(ref))     # 台账 inherit-as-is（ledger/jinfeng-2026.yaml 裁决）
        fmt, profile, pipe = "shp", "ibd-smarteditor-v1", summary
        typer.echo(f"SHP 重塑: {summary['junction']} links={summary['links']} lanes={summary['lanes']} "
                   f"connects={summary['connects']} phase={summary['phase_bound']}/{summary['phase_bound']+summary['phase_missing']}")
    elif input_path.suffix.lower() == ".xodr":
        from mapforge.adapters.opendrive.reader import parse_xodr
        from mapforge.ops.junction_to_map import rebuild_junction, _incoming_dirs
        odr = parse_xodr(str(input_path))
        jid = junction
        if jid is None:
            cands = [(j, len(_incoming_dirs(odr, j)), len(odr.junctions[j].connections))
                     for j in odr.junctions if len(_incoming_dirs(odr, j)) >= 4]
            if not cands:
                typer.echo("未找到 incoming>=4 的 junction，请用 --junction 指定")
                raise typer.Exit(1)
            jid = max(cands, key=lambda x: x[2])[0]
        node, rep = rebuild_junction(odr, jid, region=region, node_id=node_id)
        fmt, profile = "opendrive", "generic"
        pipe = {"junction": jid, "connects": rep.n_conn, "virtual_nodes": rep.virtual_nodes}
        typer.echo(f"xodr junction {jid} 重塑: {len(node.links)} links, "
                   f"connectsTo {rep.n_conn}（虚拟台账 {rep.virtual_nodes}）")
    else:                                                    # MAP XML
        node = parse_map_xml(str(input_path))

    # —— MapNode → 目标 ——
    base = out if out else (_ROOT / "out" / "convert" / (Path(input_path).stem if not input_path.is_dir()
                                                         else f"ibd_{node.node_id}"))
    base.parent.mkdir(parents=True, exist_ok=True)

    def _write_uper(p: Path):
        from mapforge.adapters.v2xmap.to_asn import node_to_messageframe_val
        mf = _msgframe_type()
        val = node_to_messageframe_val(node, mode)
        mf.set_val(val)
        buf = mf.to_uper()
        mf.from_uper(buf)
        p.write_bytes(buf)
        typer.echo(f"  {p.name}: {len(buf)} B ({mode})")

    def _write_xml(p: Path):
        from mapforge.adapters.v2xmap.xml_writer import node_to_xml
        p.write_text(node_to_xml(node), encoding="utf-8")
        typer.echo(f"  {p.name}")

    def _write_geojson(p: Path):
        p.write_text(json.dumps(to_geojson(node), ensure_ascii=False, indent=1), encoding="utf-8")
        typer.echo(f"  {p.name}: {len(to_geojson(node)['features'])} features")

    if to == "geojson":
        _write_geojson(base.with_suffix(".geojson"))
    elif to == "uper":
        _write_uper(base.with_suffix(".uper"))
    elif to == "map-xml":
        _write_xml(base.with_suffix(".map.xml"))
    elif to == "map":
        from mapforge.report.deliver import build_delivery
        out_dir = out if out else (_ROOT / "out" / "deliver" / base.name)
        res = build_delivery(
            node, out_dir, source_path=input_path, source_format=fmt, profile=profile,
            mode=mode, pipeline_summary=pipe, allow_no_phase=allow_no_phase,
            config={"command": "convert", "input": str(input_path), "to": to, "mode": mode,
                    "like": str(like) if like else None, "junction": junction,
                    "region": region, "node_id": node_id, "allow_no_phase": allow_no_phase})
        typer.echo(f"  交付包 {out_dir}（{len(res['files'])} 文件，UPER {res['bytes']} B）")
        typer.echo(f"  DELIVERY-STATUS: {res['status']}")
        if res["status"].startswith("BLOCKED"):
            raise typer.Exit(2)
    elif to == "xodr":
        from mapforge.ops.map_to_xodr import build_xodr
        pairs = [tuple(p.split("-", 1)) for p in connect.split(",")] if connect else []
        stats = build_xodr(node, base.with_suffix(".xodr"), pairs)
        typer.echo(f"  {base.with_suffix('.xodr').name}: {stats}")
    else:
        typer.echo(f"未知目标 {to}")
        raise typer.Exit(1)
    typer.echo("OK")


@app.command("validate-xodr")
def validate_xodr(xodr_path: Path,
                  xsd: Path = typer.Option(_ROOT / "OpenDRIVE_1.5M.xsd", help="XSD 路径")):
    """planView 连续性检查 + XSD 校验。"""
    from mapforge.validate.planview_check import check_file
    chk = check_file(str(xodr_path))
    typer.echo(f"continuity: pairs={chk['pairs_checked']} violations={len(chk['violations'])} "
               f"worst_pos={chk['worst_pos_gap_m']*1000:.3f}mm worst_hdg={chk['worst_hdg_gap_rad']*1000:.3f}mrad")
    from lxml import etree
    schema = etree.XMLSchema(etree.parse(str(xsd)))
    ok = schema.validate(etree.parse(str(xodr_path)))
    typer.echo(f"xsd({xsd.name}): {'PASS' if ok else 'FAIL'}")
    if not ok:
        for e in schema.error_log[:5]:
            typer.echo(f"  - {e}")
        raise typer.Exit(1)


if __name__ == "__main__":
    app()
