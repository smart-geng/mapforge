# -*- coding: utf-8 -*-
"""MAP → GeoJSON 预览（交付包三视图之一；方案 report/preview_geojson）。"""
from __future__ import annotations

from mapforge.adapters.v2xmap.xml_reader import MapNode


def to_geojson(node: MapNode) -> dict:
    feats = [{
        "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [node.ref_lon, node.ref_lat]},
        "properties": {"kind": "refPos", "region": node.region, "node": node.node_id, "name": node.name},
    }]
    for lk in node.links:
        if lk.points:
            feats.append({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[lon, lat] for lon, lat in lk.points]},
                "properties": {"kind": "link", "name": lk.name,
                               "upstream": list(lk.upstream), "width_cm": lk.width_cm,
                               "movements": [{"to": [m[0], m[1]], "phase": m[2]} for m in lk.movements]},
            })
        for ln in lk.lanes:
            if ln.points:
                feats.append({
                    "type": "Feature",
                    "geometry": {"type": "LineString", "coordinates": [[lon, lat] for lon, lat in ln.points]},
                    "properties": {"kind": "lane", "link": lk.name, "laneID": ln.lane_id,
                                   "width_cm": ln.width_cm, "maneuvers": ln.maneuvers,
                                   "connects": [{"to": [c.region, c.node], "lane": c.lane, "phase": c.phase}
                                                for c in ln.connects]},
                })
    return {"type": "FeatureCollection",
            "properties": {"crs_note": "坐标为源数据原值（度），坐标系状态见 provenance-manifest"},
            "features": feats}
