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


def _finalize_xodr(path: Path, stats: dict, connect_mode: str):
    from mapforge.report.decision import finalize_opendrive_g8
    policy = _ROOT / "profiles" / "validation" / "g8-opendrive-jinfeng-v1.yaml"
    result = finalize_opendrive_g8(
        path, stats.get("source_lane_manifest"), policy, connect_mode=connect_mode)
    gate, decision = result["gate"], result["decision"]
    typer.echo(f"  G8: {gate['status']}（matched {gate.get('scope', {}).get('matched_source_lanes', 0)}，"
               f"exclusions {len(gate.get('exclusions', []))}）")
    typer.echo(f"  DELIVERY-STATUS: {decision['status']}")
    if decision["status"] != "DELIVERABLE":
        raise typer.Exit(2)
    return result


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
            like: Path = typer.Option(None, help="IBD SHP 源：现网参考 XML（定位路口；--to map 还提供配对/phase 通道）"),
            at: str = typer.Option(None, "--at", help="IBD SHP 源定位路口的备选方式: lon,lat（如 106.51,29.60）"),
            shp_profile: str = typer.Option(None, "--profile",
                                            help="SHP Profile YAML（路径或 profiles/shp/ 下的名字）；"
                                                 "缺省用内置 ibd-smarteditor-v1 直读器"),
            junction: str = typer.Option(None, help="xodr 源：junction id（缺省自动选最大十字）"),
            connect_mode: str = typer.Option("data", "--connect-mode",
                                             help="→xodr 路口转向来源：data=仅源数据（默认，不发明"
                                                  "拓扑）| default=补每条车道位置应有的转向 | "
                                                  "full=全连接（源拓扑缺录时铺满路口，标 INFERRED）"),
            allow_uturn: bool = typer.Option(False, "--allow-uturn",
                                             help="补全时允许掉头（默认排除）"),
            region: int = typer.Option(500), node_id: int = typer.Option(9901),
            allow_no_phase: bool = typer.Option(False, "--allow-no-phase",
                                                help="显式降级：生成无 phase 的诊断产物，生产状态仍 BLOCKED")):
    """统一转换入口：MAP XML / OpenDRIVE / IBD SHP 目录 → geojson / uper / map-xml / map / xodr。"""
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
    from mapforge.report.preview_geojson import to_geojson

    # —— 源 → MapNode ——
    fmt, profile, pipe = "v2xmap-xml", "jinfeng-dialect", None
    neighbors: list = []
    def _shp_source(shp_dir):
        """SHP 源：默认走 Profile 引擎（ibd-smarteditor-v1）——其宽度阶梯能从边界横距
        救回 WIDTH=0 脏数据；YAML 缺失时退回内置直读器。"""
        from mapforge.adapters.shp.profile_source import ProfileSource
        try:
            s = ProfileSource(str(shp_dir), shp_profile or "ibd-smarteditor-v1")
            typer.echo(f"profile: {s.p.get('profile')}（{s.p.get('_path', '内嵌')}）")
            return s
        except FileNotFoundError:
            if shp_profile:
                raise
            from mapforge.adapters.shp.ibd_reader import IbdSource
            return IbdSource(str(shp_dir))

    if input_path.is_dir() and to == "xodr":                 # SHP → xodr：直转（不过 MAP 窄门）
        from mapforge.ops.shp_to_xodr import build_junction_xodr
        src = _shp_source(input_path)
        if like is not None:
            ref = parse_map_xml(str(like))
            lon_, lat_ = ref.ref_lon, ref.ref_lat
        elif at:
            lon_, lat_ = (float(v) for v in at.split(","))
        else:
            names = "、".join((j.name or j.pid) for j in src.junctions[:20])
            typer.echo(f"SHP→xodr 需定位路口：--like <现网XML> 或 --at lon,lat。可选路口：{names}")
            raise typer.Exit(1)
        junc, dist = src.find_junction(lon_, lat_)
        base = out if out else (_ROOT / "out" / "convert" / f"ibd_{junc.pid[-8:]}")
        base.parent.mkdir(parents=True, exist_ok=True)
        st = build_junction_xodr(src, junc, base.with_suffix(".xodr"),
                                 connect_mode=connect_mode, allow_uturn=allow_uturn)
        typer.echo(f"直转 {st['junction']}（ref 距 {dist:.0f}m）: 进口路 {st['roads_enter']} + "
                   f"出口路 {st['roads_leave']} + 连接路 {st['conn_via'] + st['conn_g2']}"
                   f"（实测几何 {st['conn_via']} / G2 合成 {st['conn_g2']}），"
                   f"junction connections {st['connections']} laneLinks {st['lanelinks']}，"
                   f"拟合偏差峰值 {st['fit_dev_max']:.2f}m")
        if getattr(src, "derivation_stats", None):
            typer.echo(f"  推导统计: {src.derivation_stats}")
        typer.echo(f"  {base.with_suffix('.xodr').name}")
        _finalize_xodr(base.with_suffix(".xodr"), st, connect_mode)
        typer.echo("OK")
        return
    if input_path.is_dir():                                  # SHP 目录 → MAP 系目标
        if like is None:
            typer.echo("SHP 源转 MAP 需要 --like <现网参考XML>")
            raise typer.Exit(1)
        from mapforge.ops.shp_to_map import rebuild_from_ibd, phase_table_from_xml, id_table_from_xml
        ref = parse_map_xml(str(like))
        node, summary, _ = rebuild_from_ibd(
            _shp_source(input_path), ref.ref_lon, ref.ref_lat,
            region=ref.region, node_id=ref.node_id,
            phase_table=phase_table_from_xml(ref),
            id_table=id_table_from_xml(ref))     # 台账 inherit-as-is（ledger/jinfeng-2026.yaml 裁决）
        fmt, profile, pipe = "shp", (shp_profile or "ibd-smarteditor-v1"), summary
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
    else:                                                    # MAP XML（可为多节点帧）
        from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all
        all_nodes = parse_map_xml_all(str(input_path))
        sel = [n for n in all_nodes if n.node_id == node_id]
        node = sel[0] if sel else all_nodes[0]
        neighbors = [n for n in all_nodes if n is not node]
        if neighbors:
            typer.echo(f"多节点帧: 共 {len(all_nodes)} 节点，主节点 ({node.region},{node.node_id})，"
                       f"邻居 {[n.node_id for n in neighbors]} 作为真实出口数据源")

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
        stats = build_xodr(node, base.with_suffix(".xodr"), neighbors=neighbors or None,
                           connect_mode=connect_mode, allow_uturn=allow_uturn)
        typer.echo(f"  进口路 {stats['links']} + 出口路 {stats['exit_roads']}"
                   f"（真实 {stats['exit_real']} / 镜像 INFERRED {stats['exit_mirror']}）+ "
                   f"连接路 {stats['conn_roads']}(G2)，junction connections {stats['connections']}"
                   f"，拟合偏差峰值 {stats['fit_dev_max']:.2f}m"
                   + (f"，skipped {stats['skipped']}" if stats["skipped"] else ""))
        typer.echo(f"  {base.with_suffix('.xodr').name}")
        _finalize_xodr(base.with_suffix(".xodr"), stats, connect_mode)
    else:
        typer.echo(f"未知目标 {to}")
        raise typer.Exit(1)
    typer.echo("OK")


@app.command("profile-init")
def profile_init(out: Path = typer.Argument(Path("my-vendor.yaml"), help="输出模板路径")):
    """生成带注释的 SHP Profile 模板（新图商字段映射从这里开始改）。"""
    from mapforge.adapters.shp.profile_source import TEMPLATE_YAML
    if out.exists():
        typer.echo(f"{out} 已存在，不覆盖")
        raise typer.Exit(1)
    out.write_text(TEMPLATE_YAML, encoding="utf-8")
    typer.echo(f"模板已生成: {out}\n下一步: 按交付改字段名 → python -m mapforge.cli profile-check {out} <shp目录>")


@app.command("profile-check")
def profile_check(profile: str = typer.Argument(..., help="Profile YAML（路径或 profiles/shp/ 名字）"),
                  shp_dir: Path = typer.Argument(..., help="图商 SHP 交付目录")):
    """映射体检：图层/字段存在性、值抽样、宽度量级、路线与降级预告。发现硬伤 exit 1。"""
    import shapefile as _sf
    from mapforge.adapters.shp.profile_source import load_profile, _LEN_TO_MM
    p = load_profile(profile)
    typer.echo(f"profile: {p.get('profile')}  encoding={p.get('encoding', 'utf-8')}  "
               f"units.length={(p.get('units') or {}).get('length', 'm')}")
    crs = p.get("crs") or {}
    typer.echo(f"crs: declared={crs.get('declared')!r} verified={crs.get('verified')!r}"
               + ("" if crs.get("verified") not in (False, None, "false") else
                  "  <-- 未核验：核验通过前禁止生产转换（硬约束3）"))
    errors, warns = [], []
    lane_spec = p["layers"]["lane"]
    for lname, spec in p["layers"].items():
        f = shp_dir / f"{spec['file']}.shp"
        if not f.exists():
            errors.append(f"[{lname}] 缺文件 {f.name}")
            continue
        r = _sf.Reader(str(shp_dir / spec["file"]), encoding=p.get("encoding", "utf-8"))
        have = {x[0] for x in r.fields[1:]}
        mapped = {k: v for k, v in spec.get("fields", {}).items()}
        miss = {k: v for k, v in mapped.items() if v not in have}
        for k, v in miss.items():
            errors.append(f"[{lname}] 字段 {k}->{v!r} 不存在（可用: {sorted(have)[:12]}...）")
        ok = {k: v for k, v in mapped.items() if v in have}
        if ok:
            rec = r.record(0)
            fields = [x[0] for x in r.fields[1:]]
            m = dict(zip(fields, rec))
            sample = {k: str(m.get(v))[:18] for k, v in ok.items()}
            typer.echo(f"  [{lname}] {spec['file']} {r.numRecords} 条  样例: {sample}")
    # 宽度量级体检
    wf = (lane_spec.get("fields") or {}).get("width")
    if wf and (shp_dir / f"{lane_spec['file']}.shp").exists():
        r = _sf.Reader(str(shp_dir / lane_spec["file"]), encoding=p.get("encoding", "utf-8"))
        fields = [x[0] for x in r.fields[1:]]
        if wf in fields:
            i = fields.index(wf)
            vals = [float(str(rec[i]).strip() or 0) for rec in list(r.iterRecords())[:200]]
            scale = _LEN_TO_MM[(p.get("units") or {}).get("length", "m")]
            med_mm = sorted(v * scale for v in vals)[len(vals) // 2]
            typer.echo(f"  宽度量级: 抽样中位 {med_mm:.0f}mm（按 units.length 换算）")
            if not 1500 <= med_mm <= 6500:
                warns.append(f"宽度中位 {med_mm:.0f}mm 不像车道宽——units.length 可能配错")
    # 路线与降级预告
    geom = lane_spec.get("geometry", "field")
    typer.echo(f"  车道几何路线: {'A 车道中心线字段' if geom == 'field' else 'B 边界线合成'}；"
               f"宽度阶梯: {lane_spec.get('width_from', ['field', 'boundaries', 'spacing', 'default'])}")
    for opt, msg in (("junction", "无路口层：需 --at lon,lat 定位，端点聚类推断进 REVIEW"),
                     ("topo", "无拓扑层：junction 连接将为空（几何推断路线未实装）"),
                     ("road", "无道路层：按车道 road 字段合成归组，路名/拼链不可用"),
                     ("road_center", "无道路中心线层：参考线用中间车道替代")):
        if opt not in p["layers"]:
            warns.append(msg)
    for w in warns:
        typer.echo(f"  警告: {w}")
    if errors:
        for e in errors:
            typer.echo(f"  错误: {e}")
        raise typer.Exit(1)
    typer.echo("profile-check PASS")


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
