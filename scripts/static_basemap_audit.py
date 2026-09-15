# -*- coding: utf-8 -*-
"""把 basemap_overlay 的 GeoJSON 渲染成可直接目检的卫星底图静态审计图。"""
from __future__ import annotations

import argparse
import io
import json
import math
import re
import urllib.request
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
CASES = ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")
ZOOM = 18
TILE = 256


def _read_data(path):
    text = path.read_text(encoding="utf-8")
    match = re.search(r"const DATA=(.*?);\nconst map=", text, re.S)
    if not match:
        raise ValueError(f"HTML 中未找到 DATA: {path}")
    return json.loads(match.group(1))


def _coord_sequences(geometry):
    kind = geometry.get("type")
    coordinates = geometry.get("coordinates") or []
    if kind == "Point":
        return [coordinates]
    if kind == "LineString":
        return [coordinates]
    if kind == "Polygon":
        return coordinates
    return []


def _all_points(data, keys):
    out = []
    for key in keys:
        for feature in data[key]["features"]:
            for seq in _coord_sequences(feature["geometry"]):
                out.extend(seq)
    return np.asarray(out, float)


def _global_pixel(points, zoom=ZOOM):
    p = np.asarray(points, float)
    n = 2 ** zoom
    x = (p[:, 0] + 180.0) / 360.0 * n * TILE
    lat = np.radians(np.clip(p[:, 1], -85.05112878, 85.05112878))
    y = (1.0 - np.arcsinh(np.tan(lat)) / math.pi) / 2.0 * n * TILE
    return np.column_stack([x, y])


def _tile_url(x, y, z):
    return ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            f"World_Imagery/MapServer/tile/{z}/{y}/{x}")


def _tile(cache, x, y, z):
    path = cache / str(z) / str(x) / f"{y}.jpg"
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        request = urllib.request.Request(
            _tile_url(x, y, z), headers={"User-Agent": "mapforge-visual-audit/1.0"})
        with urllib.request.urlopen(request, timeout=30) as response:
            path.write_bytes(response.read())
    return Image.open(path).convert("RGB")


def _mosaic(points, cache, zoom=ZOOM):
    pix = _global_pixel(points, zoom)
    pad = 48
    xmin = int(math.floor((pix[:, 0].min() - pad) / TILE))
    xmax = int(math.floor((pix[:, 0].max() + pad) / TILE))
    ymin = int(math.floor((pix[:, 1].min() - pad) / TILE))
    ymax = int(math.floor((pix[:, 1].max() + pad) / TILE))
    image = Image.new("RGB", ((xmax - xmin + 1) * TILE,
                              (ymax - ymin + 1) * TILE))
    for x in range(xmin, xmax + 1):
        for y in range(ymin, ymax + 1):
            image.paste(_tile(cache, x, y, zoom), ((x - xmin) * TILE,
                                                    (y - ymin) * TILE))
    return image, np.array([xmin * TILE, ymin * TILE], float)


def _local_pixels(sequence, origin):
    return _global_pixel(sequence) - origin


def _draw_fc(ax, fc, origin, *, color, linewidth=1.2, alpha=0.9,
             fill=False, fill_alpha=0.12, linestyle="-"):
    for feature in fc["features"]:
        for seq in _coord_sequences(feature["geometry"]):
            if len(seq) < 2:
                continue
            p = _local_pixels(seq, origin)
            if fill and feature["geometry"]["type"] == "Polygon":
                ax.fill(p[:, 0], p[:, 1], color=color, alpha=fill_alpha,
                        linewidth=0)
            ax.plot(p[:, 0], p[:, 1], color=color, lw=linewidth,
                    alpha=alpha, linestyle=linestyle)


def _draw_source(ax, data, origin):
    _draw_fc(ax, data["sourceJunctions"], origin, color="#ff7a00",
             linewidth=2.4, fill=True, fill_alpha=0.12)
    _draw_fc(ax, data["sourceBoundaries"], origin, color="#ff7a00",
             linewidth=1.8)
    _draw_fc(ax, data["sourceRoadCenters"], origin, color="#5cff45",
             linewidth=2.4, linestyle="--")
    _draw_fc(ax, data["sourceLines"], origin, color="#ffd400",
             linewidth=1.2, alpha=0.82, linestyle="--")


def _draw_target(ax, data, origin):
    # 低透明道路面只用于辨别覆盖范围，车道边缘才是形态对拍主线。
    for feature in data["surface"]["features"]:
        kind = feature.get("properties", {}).get("road_kind")
        color = {"mirrored-exit": "#ff2eea", "connecting-road": "#a855f7",
                 "junction-paving": "#20d875"}.get(kind, "#1089ff")
        if feature.get("properties", {}).get("review_reasons"):
            color = "#ff1744"
        _draw_fc(ax, {"features": [feature]}, origin, color=color,
                 linewidth=0.35, alpha=0.55, fill=True, fill_alpha=0.10)
    _draw_fc(ax, data["edges"], origin, color="#00e5ff", linewidth=1.15)
    _draw_fc(ax, data["surfaceOutline"], origin, color="#ffffff",
             linewidth=2.8, alpha=0.95)
    _draw_fc(ax, data["refline"], origin, color="#ff315b", linewidth=1.6,
             alpha=0.85, linestyle="--")


def _junction_center(data):
    points = []
    for feature in data["surface"]["features"]:
        if feature.get("properties", {}).get("road_kind") != "junction-paving":
            continue
        for seq in _coord_sequences(feature["geometry"]):
            points.extend(seq)
    if not points:
        points = _all_points(data, ("surface",)).tolist()
    return np.mean(np.asarray(points, float), axis=0)


def render(html_path, output, cache):
    data = _read_data(html_path)
    points = _all_points(data, ("surface", "sourceRoadCenters", "sourceLines",
                                "sourceBoundaries", "sourceJunctions"))
    image, origin = _mosaic(points, cache)
    width, height = image.size
    fig, axes = plt.subplots(2, 2, figsize=(20, 13), dpi=150)
    panels = (("Source on satellite", True, False),
              ("OpenDRIVE on satellite", False, True),
              ("Full overlay", True, True),
              ("Junction center zoom", True, True))
    center = _local_pixels([_junction_center(data)], origin)[0]
    center_lat = _junction_center(data)[1]
    resolution = 156543.033928 * math.cos(math.radians(center_lat)) / (2 ** ZOOM)
    radius = 80.0 / resolution
    for index, (ax, (title, source, target)) in enumerate(zip(axes.flat, panels)):
        ax.imshow(image, extent=(0, width, height, 0))
        if target:
            _draw_target(ax, data, origin)
        if source:
            _draw_source(ax, data, origin)
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.set_axis_off()
        if index == 3:
            ax.set_xlim(center[0] - radius, center[0] + radius)
            ax.set_ylim(center[1] + radius, center[1] - radius)
        else:
            ax.set_xlim(0, width)
            ax.set_ylim(height, 0)
    fig.suptitle(html_path.stem +
                 " | orange=SHP boundary/surface, green=source road center/Link.points, "
                 "yellow=source lane center, cyan=XODR edges, red=XODR reference")
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)
    print(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=CASES)
    parser.add_argument("--pipeline", choices=("shp", "map"))
    parser.add_argument("--html", type=Path, help="直接渲染任意 basemap overlay HTML")
    parser.add_argument("--output", type=Path, help="与 --html 配合指定 PNG")
    args = parser.parse_args()
    output_root = ROOT / "out/preview/basemap-static"
    cache = output_root / "tile-cache"
    if args.html:
        render(args.html, args.output or output_root / f"{args.html.stem}.png", cache)
        return
    cases = (args.case,) if args.case else CASES
    pipes = (args.pipeline,) if args.pipeline else ("shp", "map")
    source = ROOT / "out/preview/basemap-overlay"
    for case in cases:
        for pipeline in pipes:
            suffix = "shp-vs-xodr" if pipeline == "shp" else "map-vs-xodr"
            html_path = source / f"{case}-{suffix}-basemap.html"
            render(html_path, output_root / f"{case}-{pipeline}-satellite-audit.png", cache)


if __name__ == "__main__":
    main()
