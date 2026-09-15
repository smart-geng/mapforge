# -*- coding: utf-8 -*-
"""把源地图与生成的 OpenDRIVE 叠加到真实在线底图上。

输出为持久化的 Leaflet HTML，可在浏览器中切换卫星/道路底图及源/目标图层。
源身份取本次 manifest；SHP 几何回查未经裁剪的原始图层，目标取最终 XODR。
不能只展示已被生成器裁过的来源支持域，否则会隐藏提前转弯等丢失部分。

用法：
  python scripts/basemap_overlay.py
  python scripts/basemap_overlay.py --case node4
"""
from __future__ import annotations

import argparse
import html
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all  # noqa: E402
from mapforge.validate.smoothness import (                     # noqa: E402
    _sections,
    lane_edges_at,
    sample_road_ref,
)
from scripts.xodr_topdown import road_patches                  # noqa: E402


CASES = ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")


def _feature(geometry_type, coordinates, **properties):
    return {
        "type": "Feature",
        "geometry": {"type": geometry_type, "coordinates": coordinates},
        "properties": properties,
    }


def _fc(features):
    return {"type": "FeatureCollection", "features": features}


def _round_coords(values, digits=8):
    a = np.asarray(values, float)
    return np.round(a, digits).tolist()


class _EqcInverse:
    def __init__(self, geo_reference):
        params = dict(re.findall(r"\+([A-Za-z0-9_]+)=([^\s]+)", geo_reference))
        if params.get("proj") != "eqc":
            raise ValueError(f"当前底图工具仅支持生成器写出的 +proj=eqc：{geo_reference}")
        try:
            self.lat0 = float(params["lat_0"])
            self.lon0 = float(params["lon_0"])
            self.lat_ts = float(params.get("lat_ts", self.lat0))
            self.radius = float(params.get("R", 6378137.0))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"无效 eqc geoReference：{geo_reference}") from exc

    def transform(self, x, y):
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        lon = self.lon0 + np.degrees(x / (
            self.radius * math.cos(math.radians(self.lat_ts))))
        lat = self.lat0 + np.degrees(y / self.radius)
        return lon, lat


def _transformer(root):
    geo = (root.findtext("header/geoReference") or "").strip()
    if not geo:
        raise ValueError("XODR 缺少 header/geoReference，不能打到真实底图")
    return _EqcInverse(geo), geo


def _to_wgs84(points, transformer):
    p = np.asarray(points, float)
    lon, lat = transformer.transform(p[:, 0], p[:, 1])
    return _round_coords(np.column_stack([lon, lat]))


def _road_provenance(road):
    records = []
    for lane in road.findall("lanes/laneSection/right/lane") + road.findall(
            "lanes/laneSection/left/lane"):
        ud = lane.find("userData[@code='mapforge.provenance/v1']")
        if ud is None or not ud.get("value"):
            continue
        try:
            records.append(json.loads(ud.get("value")))
        except json.JSONDecodeError:
            continue
    return records


def _road_kind(road):
    if road.get("name") == "junction_paving":
        return "junction-paving"
    if road.get("junction") not in (None, "-1"):
        return "connecting-road"
    if any(x.get("status") == "INFERRED" or x.get("support_kind") == "mirror"
           for x in _road_provenance(road)):
        return "mirrored-exit"
    return "ordinary-road"


def _lane_lines(road, ds=0.75):
    """按 laneSection 返回最终 XODR 的所有车道边缘与车道中心。"""
    points, stations, headings = sample_road_ref(road, ds)
    from mapforge.validate.smoothness import _geoms
    from mapforge.ops.refline_fit import _prim_pose_at
    primitives=_geoms(road)
    starts=np.r_[0.,np.cumsum([g[4] for g in primitives])]
    edges, centers = [], []
    sections = _sections(road)
    for index, (s0, right, left) in enumerate(sections):
        s1 = sections[index + 1][0] if index + 1 < len(sections) else float(
            road.get("length"))
        local_s=np.unique(np.r_[s0,stations[(stations>s0)&(stations<s1)],s1])
        local_points=np.c_[np.interp(local_s,stations,points[:,0]),np.interp(local_s,stations,points[:,1])]
        local_h=np.interp(local_s,stations,np.unwrap(headings))
        # Explicit section endpoints avoid display gaps and inflated source
        # distances caused by a global sample grid missing section boundaries.
        for j in (0,-1):
            idx=min(len(primitives)-1,max(0,int(np.searchsorted(starts,local_s[j],side='right')-1)))
            x,y,h,_=_prim_pose_at(primitives[idx],local_s[j]-starts[idx])
            local_points[j]=[x,y];local_h[j]=h
        normals=np.c_[-np.sin(local_h),np.cos(local_h)]
        eval_s=np.clip(local_s,s0+min(1e-7,(s1-s0)/4),s1-min(1e-7,(s1-s0)/4))
        for side, lanes in (("right", right), ("left", left)):
            if not lanes:
                continue
            offsets = np.asarray([lane_edges_at(road, s, side=side)
                                  for s in eval_s], float)
            for edge_index in range(offsets.shape[1]):
                edges.append((local_points + offsets[:, edge_index, None] * normals,
                              side, index, edge_index))
            for lane_index in range(offsets.shape[1] - 1):
                middle = 0.5 * (offsets[:, lane_index] + offsets[:, lane_index + 1])
                centers.append((local_points + middle[:, None] * normals,
                                side, index, lane_index + 1))
    return edges, centers


def _xodr_layers(root, transformer, raw_review=None):
    surface, refline, edges, centers = [], [], [], []
    surface_polygons = []
    counts = {"ordinary-road": 0, "mirrored-exit": 0,
              "connecting-road": 0, "junction-paving": 0}
    for road in root.findall("road"):
        kind = _road_kind(road)
        counts[kind] += 1
        props = {
            "road_id": road.get("id"),
            "road_name": road.get("name") or "",
            "road_kind": kind,
            "junction": road.get("junction") or "-1",
        }
        if kind == "connecting-road":
            props["review_reasons"] = sorted({
                p["exclusion_code"] for p in _road_provenance(road)
                if p.get("exclusion_code") in {
                    "minimal-chain-source-fidelity-unmet", "source-end-state-conflict"}})
            props["source_lane_ids"] = sorted({
                ud.get("value") for ud in road.findall(
                    ".//userData[@code='mapforge.source_lane']") if ud.get("value")})
            review = (raw_review or {}).get(road.get("id"))
            if review:
                props["review_reasons"].append("unclipped-source-route-deviation")
                props["raw_route_error_m"] = review["full_raw_source_to_linked_route"]
        for polygon, _is_junction in road_patches(road, ds=0.75):
            surface_polygons.append(np.asarray(polygon, float))
            ring = _to_wgs84(np.vstack([polygon, polygon[:1]]), transformer)
            surface.append(_feature("Polygon", [ring], **props))
        p, _s, _h = sample_road_ref(road, 1.0)
        refline.append(_feature("LineString", _to_wgs84(p, transformer), **props))
        lane_edges, lane_centers = _lane_lines(road)
        for line, side, section, ordinal in lane_edges:
            edges.append(_feature(
                "LineString", _to_wgs84(line, transformer),
                **props, side=side, lane_section=section, edge_ordinal=ordinal))
        for line, side, section, ordinal in lane_centers:
            centers.append(_feature(
                "LineString", _to_wgs84(line, transformer),
                **props, side=side, lane_section=section, lane_ordinal=ordinal))
    outlines, holes = [], []
    if surface_polygons:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        # road_patches 在 laneSection 边界为避免取错断面会留下约 1e-4m 的数值缝。
        # 先以 2cm 形态学闭运算消除纯数值缝，再画“可见道路面并集”；大于该容差的
        # 真孔洞仍保留并进入 surfaceHoles 诊断层。
        union = unary_union([Polygon(p).buffer(0) for p in surface_polygons]) \
            .buffer(0.02, join_style=2).buffer(-0.02, join_style=2)
        polygons = list(union.geoms) if union.geom_type == "MultiPolygon" else [union]
        for index, polygon in enumerate(polygons):
            if polygon.is_empty:
                continue
            outlines.append(_feature(
                "LineString",
                _to_wgs84(np.asarray(polygon.exterior.coords), transformer),
                component=index,
                support_kind="xodr-rendered-surface-union"))
            from shapely.geometry import Polygon as ShapelyPolygon
            for hole_index, ring in enumerate(polygon.interiors):
                ring_xy = np.asarray(ring.coords)
                area = float(ShapelyPolygon(ring_xy).area)
                holes.append(_feature(
                    "Polygon", [[*_to_wgs84(ring_xy, transformer)]],
                    component=index, hole=hole_index, area_m2=area,
                    support_kind="xodr-rendered-surface-hole"))
    return {
        "surface": _fc(surface),
        "surface_outline": _fc(outlines),
        "surface_holes": _fc(holes),
        "refline": _fc(refline),
        "edges": _fc(edges),
        "centers": _fc(centers),
    }, counts


def _source_center_layers(manifest, transformer):
    lines, points = [], []
    for lane in manifest.get("lanes", []):
        geom = lane.get("geometry") or {}
        coords = geom.get("coordinates") or []
        if geom.get("type") != "LineString" or len(coords) < 2:
            continue
        wgs = _to_wgs84(coords, transformer)
        compare = lane.get("comparison") or {}
        props = {
            "source_lane_id": lane.get("source_lane_id") or "",
            "role": lane.get("role") or "",
            "status": lane.get("status") or "",
            "support_kind": lane.get("support_kind") or "",
            "eligible": bool(compare.get("eligible")),
        }
        lines.append(_feature("LineString", wgs, **props))
        for index, point in enumerate(wgs):
            points.append(_feature("Point", point, **props, point_index=index))
    return _fc(lines), _fc(points)


def _shp_source_layers(manifest):
    source = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    features, seen = [], set()
    raw_lines, raw_points = [], []
    link_ids = set()
    for lane in manifest.get("lanes", []):
        lane_id = str(lane.get("source_lane_id") or "")
        owner = lane.get("owner") or {}
        if owner.get("link"):
            link_ids.add(str(owner["link"]))
        if not lane_id:
            continue
        record = source.lane(lane_id)
        if record is not None and len(record.geometry) >= 2:
            props = {"source_lane_id": lane_id, "role": lane.get("role"),
                     "source_extent": "full-unclipped-SHP"}
            points = _round_coords(record.geometry)
            raw_lines.append(_feature("LineString", points, **props))
            for i, pt in enumerate(points):
                raw_points.append(_feature("Point", pt, **props, point_index=i))
        for boundary in source.lane_boundary_geometries(lane_id):
            values = np.asarray(boundary, float)
            if len(values) < 2:
                continue
            key = (len(values), tuple(np.round(values[0], 8)),
                   tuple(np.round(values[-1], 8)))
            reverse = (key[0], key[2], key[1])
            if key in seen or reverse in seen:
                continue
            seen.add(key)
            features.append(_feature(
                "LineString", _round_coords(values),
                source_lane_id=lane_id, support_kind="shp-physical-boundary"))
    road_centers = []
    centers = source.roadcenters
    for link_id in sorted(link_ids):
        values = centers.get(link_id)
        if values is not None and len(values) >= 2:
            road_centers.append(_feature(
                "LineString", _round_coords(values), source_link_id=link_id,
                support_kind="shp-road-center"))
    junctions = []
    junction_ids = {str(x.get("junction")) for x in manifest.get("source_contexts", [])
                    if x.get("junction") is not None}
    for junction in source.junctions:
        if junction.pid not in junction_ids or len(junction.polygon) < 3:
            continue
        ring = np.asarray(junction.polygon, float)
        if float(np.linalg.norm(ring[0] - ring[-1])) > 1e-12:
            ring = np.vstack([ring, ring[:1]])
        junctions.append(_feature(
            "Polygon", [_round_coords(ring)], source_junction_id=junction.pid,
            name=junction.name, support_kind="shp-intersection-surface"))
    return (_fc(features), _fc(road_centers), _fc(junctions),
            _fc(raw_lines), _fc(raw_points))


def _map_road_center_layer(manifest):
    """读取实际参与本次转换的 MAP Link.points（道路中心/Link 参考线）。"""
    nodes = {}
    for path in sorted((ROOT / "v2x_map_xml").glob("map*.xml")):
        for node in parse_map_xml_all(str(path)):
            nodes[(node.region, node.node_id)] = node
    wanted = set()
    for lane in manifest.get("lanes", []):
        owner = lane.get("owner") or {}
        if owner.get("format") != "map" or not owner.get("link"):
            continue
        wanted.add((owner.get("region"), owner.get("node"), str(owner.get("link"))))
    features = []
    for region, node_id, link_name in sorted(wanted, key=lambda x: tuple(map(str, x))):
        node = nodes.get((region, node_id))
        if node is None:
            continue
        for link in node.links:
            if link.name != link_name or len(link.points) < 2:
                continue
            features.append(_feature(
                "LineString", _round_coords(link.points), region=region, node=node_id,
                link=link.name, upstream=list(link.upstream),
                support_kind="map-link-road-center"))
    return _fc(features)


def _json_for_script(value):
    # 防止数据中的字符串意外闭合 script 标签。
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")) \
        .replace("</", "<\\/")


def _popup(properties):
    return "<br>".join(
        f"<b>{html.escape(str(key))}</b>: {html.escape(str(value))}"
        for key, value in properties.items() if value not in (None, ""))


def _html_page(case, pipeline, xodr_path, manifest, layers, counts, geo_ref,
               source_boundaries=None, source_road_centers=None,
               source_junctions=None):
    source_name = "原始 SHP" if pipeline == "shp" else "原始 MAP XML"
    warning = "绝对 CRS 尚未用外部控制点核验；底图用于人工发现整体平移/旋转及局部形态差异。"
    data = {
        "sourceLines": layers["source_lines"],
        "sourcePoints": layers["source_points"],
        "sourceBoundaries": source_boundaries or _fc([]),
        "sourceRoadCenters": source_road_centers or _fc([]),
        "sourceJunctions": source_junctions or _fc([]),
        "surface": layers["surface"],
        "surfaceOutline": layers["surface_outline"],
        "surfaceHoles": layers["surface_holes"],
        "refline": layers["refline"],
        "edges": layers["edges"],
        "centers": layers["centers"],
    }
    source_count = len(data["sourceLines"]["features"])
    boundary_count = len(data["sourceBoundaries"]["features"])
    road_center_count = len(data["sourceRoadCenters"]["features"])
    title = f"{case} {source_name} / OpenDRIVE 底图叠加"
    return f"""<!doctype html>
<html lang=\"zh-CN\">
<head>
  <meta charset=\"utf-8\">
  <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">
  <title>{html.escape(title)}</title>
  <link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\">
  <style>
    html,body,#map{{height:100%;margin:0}}
    body{{font-family:Segoe UI,Microsoft YaHei,sans-serif}}
    .info{{padding:10px 12px;background:rgba(20,24,30,.90);color:#fff;
      max-width:380px;line-height:1.45;border-radius:5px;box-shadow:0 1px 8px #0008}}
    .info h1{{font-size:16px;margin:0 0 6px;font-weight:600}}
    .info .small{{font-size:12px;color:#ddd}}
    .legend{{padding:8px 10px;background:rgba(20,24,30,.90);color:#fff;
      line-height:1.7;border-radius:5px}}
    .legend i{{display:inline-block;width:24px;height:4px;margin-right:7px;vertical-align:middle}}
    .leaflet-control-layers{{font-size:13px}}
  </style>
</head>
<body>
<div id=\"map\"></div>
<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>
<script>
const DATA={_json_for_script(data)};
const map=L.map('map',{{preferCanvas:true}});
const satellite=L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
  {{maxZoom:20,attribution:'Tiles &copy; Esri'}}).addTo(map);
const osm=L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
  {{maxZoom:20,attribution:'&copy; OpenStreetMap contributors'}});

function popup(feature,layer){{
  const p=feature.properties||{{}};
  const escape=s=>String(s).replace(/[&<>"']/g,c=>({{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}}[c]));
  layer.bindPopup(Object.entries(p).filter(x=>x[1]!==''&&x[1]!==null)
    .map(x=>'<b>'+escape(x[0])+'</b>: '+escape(typeof x[1]==='object'?JSON.stringify(x[1]):x[1])).join('<br>'));
}}
function lineLayer(data,style,pointStyle){{
  return L.geoJSON(data,{{style:()=>style,onEachFeature:popup,
    pointToLayer:(feature,latlng)=>L.circleMarker(latlng,pointStyle||{{radius:2}})}});
}}
function surfaceStyle(feature){{
  const k=(feature.properties||{{}}).road_kind;
  if((feature.properties.review_reasons||[]).length)
    return {{color:'#ff1744',weight:1.2,fillColor:'#ff1744',fillOpacity:.23}};
  if(k==='mirrored-exit') return {{color:'#ff3bf2',weight:1,fillColor:'#ff3bf2',fillOpacity:.28}};
  if(k==='connecting-road') return {{color:'#a855f7',weight:.6,fillColor:'#a855f7',fillOpacity:.20}};
  if(k==='junction-paving') return {{color:'#22c55e',weight:.5,fillColor:'#22c55e',fillOpacity:.12}};
  return {{color:'#1689ff',weight:.5,fillColor:'#1689ff',fillOpacity:.16}};
}}

const surface=L.geoJSON(DATA.surface,{{style:surfaceStyle,onEachFeature:popup}}).addTo(map);
const surfaceOutline=L.geoJSON(DATA.surfaceOutline,
  {{style:()=>({{color:'#ffffff',weight:4,opacity:.95,fillOpacity:0}}),
    onEachFeature:popup}}).addTo(map);
const surfaceHoles=L.geoJSON(DATA.surfaceHoles,
  {{style:feature=>({{color:'#ff1744',weight:2,opacity:.9,fillColor:'#ff1744',
    fillOpacity:(feature.properties.area_m2||0)>.05?.35:.08}}),onEachFeature:popup}});
const sourceBoundary=lineLayer(DATA.sourceBoundaries,
  {{color:'#ff8a00',weight:3,opacity:.95}}).addTo(map);
const sourceRoadCenter=lineLayer(DATA.sourceRoadCenters,
  {{color:'#60ff64',weight:3.2,opacity:.98,dashArray:'10 5'}}).addTo(map);
const sourceJunction=L.geoJSON(DATA.sourceJunctions,
  {{style:()=>({{color:'#ff8a00',weight:3,fillColor:'#ff8a00',fillOpacity:.06}}),
    onEachFeature:popup}}).addTo(map);
const sourceLines=lineLayer(DATA.sourceLines,
  {{color:'#ffd400',weight:3,opacity:.95,dashArray:'7 5'}}).addTo(map);
const sourcePoints=lineLayer(DATA.sourcePoints,{{}},
  {{radius:2.8,color:'#fff2a8',fillColor:'#ff8a00',fillOpacity:1,weight:1}});
const xodrEdges=lineLayer(DATA.edges,
  {{color:'#00e5ff',weight:1.8,opacity:.92}}).addTo(map);
const xodrCenters=lineLayer(DATA.centers,
  {{color:'#2979ff',weight:1.4,opacity:.85,dashArray:'5 4'}});
const refline=lineLayer(DATA.refline,
  {{color:'#ff2d55',weight:2.2,opacity:.9,dashArray:'10 6'}});
// 独立置顶的审查线：不能被源中心线或普通边缘覆盖后只剩一个红色图例。
const reviewRoutes=lineLayer({{type:'FeatureCollection',features:
  DATA.refline.features.filter(f=>(f.properties.review_reasons||[]).length)}},
  {{color:'#ff1744',weight:4,opacity:1}}).addTo(map);

const bounds=L.geoJSON(DATA.surface).getBounds();
if(bounds.isValid()) map.fitBounds(bounds.pad(.08)); else map.setView([29.52,106.32],18);
L.control.layers(
  {{'Esri 卫星影像':satellite,'OpenStreetMap 道路图':osm}},
  {{'XODR 道路面':surface,
    'XODR 最终道路面并集外轮廓':surfaceOutline,
    'XODR 道路面内部孔洞（诊断）':surfaceHoles,
    '{source_name}物理边界':sourceBoundary,
    '{source_name}道路中心/Link.points':sourceRoadCenter,
    '{source_name}原始路口面':sourceJunction,
    '{source_name}车道点列/中心线':sourceLines,
    '{source_name}原始采样点':sourcePoints,
    'XODR 车道边缘':xodrEdges,
    'XODR 车道中心':xodrCenters,
    'XODR 参考线':refline,
    '未裁剪来源待修连接（置顶）':reviewRoutes}},
  {{collapsed:false}}).addTo(map);
L.control.scale({{imperial:false,maxWidth:180}}).addTo(map);

const info=L.control({{position:'topleft'}});
info.onAdd=function(){{const d=L.DomUtil.create('div','info');d.innerHTML=
  '<h1>{html.escape(title)}</h1>'+
  '<div>{source_name}：{road_center_count} 条道路中心，{source_count} 条车道线；物理边界：{boundary_count} 条</div>'+
  '<div>XODR：普通路 {counts['ordinary-road']}，镜像出口 {counts['mirrored-exit']}，连接路 {counts['connecting-road']}，铺装面 {counts['junction-paving']}</div>'+
  '<div class=\"small\">橙/黄＝源数据；青/蓝＝XODR；洋红＝左右镜像；紫＝路口连接路；红＝来源保真/端状态待复核。点选红色连接可查看源车道 ID 与原因。</div>'+
  '<div class=\"small\">{warning}</div>';return d;}};info.addTo(map);
const legend=L.control({{position:'bottomright'}});
legend.onAdd=function(){{const d=L.DomUtil.create('div','legend');d.innerHTML=
  '<i style=\"background:#ff8a00\"></i>源物理边界<br>'+
  '<i style=\"background:#60ff64\"></i>源道路中心 / Link.points<br>'+
  '<i style=\"background:#ffd400\"></i>源车道线/点列<br>'+
  '<i style=\"background:#00e5ff\"></i>XODR 车道边缘<br>'+
  '<i style=\"background:#ff3bf2\"></i>镜像出口<br>'+
  '<i style=\"background:#a855f7\"></i>连接路<br>'+
  '<i style=\"background:#ff1744\"></i>来源保真/端状态待复核';return d;}};legend.addTo(map);
</script>
</body>
</html>
"""


def render_paths(case, pipeline, xodr_path, manifest_path, output_dir, label=None):
    xodr_path = Path(xodr_path)
    manifest_path = Path(manifest_path)
    root = ET.parse(xodr_path).getroot()
    transformer, geo_ref = _transformer(root)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    raw_review = {}
    raw_path = ROOT / "out/unclipped-via-audit-v135.json"
    if pipeline == "shp" and raw_path.exists():
        report = json.loads(raw_path.read_text(encoding="utf-8"))
        current_hash = hashlib.sha256(xodr_path.read_bytes()).hexdigest()
        matches = any(x["case"] == case and x["sha256"] == current_hash
                      for x in report.get("artifacts", []))
        if matches:
            raw_review = {r["road_id"]: r for r in report["rows"]
                          if r["case"] == case and r["needs_shape_review"]}
    xodr_layers, counts = _xodr_layers(root, transformer, raw_review)
    source_lines, source_points = _source_center_layers(manifest, transformer)
    xodr_layers["source_lines"] = source_lines
    xodr_layers["source_points"] = source_points
    if pipeline == "shp":
        source_boundaries, source_road_centers, source_junctions, raw_lines, raw_points = \
            _shp_source_layers(manifest)
        xodr_layers["source_lines"] = raw_lines
        xodr_layers["source_points"] = raw_points
    else:
        source_boundaries = None
        source_road_centers = _map_road_center_layer(manifest)
        source_junctions = None
    page = _html_page(case, pipeline, xodr_path, manifest, xodr_layers, counts,
                      geo_ref, source_boundaries, source_road_centers,
                      source_junctions)
    output_dir.mkdir(parents=True, exist_ok=True)
    suffix = "shp-vs-xodr" if pipeline == "shp" else "map-vs-xodr"
    output = output_dir / f"{label or case}-{suffix}-basemap.html"
    output.write_text(page, encoding="utf-8")
    return output


def render_case(case, pipeline, output_dir):
    folder = ROOT / ("out/direct_xodr" if pipeline == "shp" else "out/m2x")
    return render_paths(case, pipeline, folder / f"{case}.xodr",
                        folder / f"{case}.source-lanes.json", output_dir)


def _index(outputs, output_dir):
    rows = []
    for case in CASES:
        shp = next((p for p in outputs if p.name.startswith(case + "-shp-")), None)
        map_ = next((p for p in outputs if p.name.startswith(case + "-map-")), None)
        rows.append(
            f"<tr><td>{case}</td><td><a href='{shp.name}'>SHP vs XODR</a></td>"
            f"<td><a href='{map_.name}'>MAP XML vs XODR</a></td></tr>")
    page = """<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>mapforge 真实底图叠加检查</title><style>
body{font-family:Segoe UI,Microsoft YaHei,sans-serif;max-width:900px;margin:36px auto;padding:0 20px}
table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid #ccc;padding:12px;text-align:left}
a{color:#1565c0}p{line-height:1.6;color:#444}</style></head><body>
<h1>mapforge 真实底图叠加检查</h1>
<p>每个路口分开提供 SHP→XODR 与 MAP XML→XODR。进入页面后可切换卫星/道路底图及源/目标图层。</p>
<table><thead><tr><th>路口</th><th>SHP 管线</th><th>MAP XML 管线</th></tr></thead><tbody>""" \
        + "".join(rows) + "</tbody></table></body></html>"
    path = output_dir / "index.html"
    path.write_text(page, encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=CASES)
    parser.add_argument("--pipeline", choices=("shp", "map"))
    parser.add_argument("--xodr", type=Path,
                        help="审计任意 XODR；须同时给 --case/--pipeline，可省略同名 manifest")
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--label", help="任意产物输出文件标签")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "out/preview/basemap-overlay")
    args = parser.parse_args()
    if args.xodr:
        if not args.case or not args.pipeline:
            parser.error("--xodr 必须同时给 --case 和 --pipeline")
        manifest = args.manifest or args.xodr.with_suffix(".source-lanes.json")
        path = render_paths(args.case, args.pipeline, args.xodr, manifest,
                            args.output_dir, label=args.label)
        print(path)
        return
    cases = (args.case,) if args.case else CASES
    outputs = []
    for case in cases:
        for pipeline in ((args.pipeline,) if args.pipeline else ("shp", "map")):
            path = render_case(case, pipeline, args.output_dir)
            outputs.append(path)
            print(path)
    if not args.case:
        print(_index(outputs, args.output_dir))


if __name__ == "__main__":
    main()
