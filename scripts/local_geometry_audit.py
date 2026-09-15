# -*- coding: utf-8 -*-
"""真实源几何与最终 OpenDRIVE 的局部误差审计。

整体中位数/P95 会掩盖一小段 1~3m 的错位。本脚本把误差落到每个源采样点，
同时单独检查：

* MAP ``Link.points``（方向道路参考中心）到最终 planView；
* MAP/SHP 车道中心到同身份 OpenDRIVE 车道中心；
* SHP 物理外边界到同身份 OpenDRIVE 道路外缘；
* SHP 路口面边界到 OpenDRIVE ``junction_paving`` 并集边界。

输出 JSON 与 PNG；PNG 中黄/红点就是需要人工逐处查看的局部偏差，不能被全局统计
平均掉。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import BoundaryNorm, ListedColormap

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all  # noqa: E402
from mapforge.ops.map_to_xodr import _project as project_map  # noqa: E402
from mapforge.validate.lane_fidelity import extract_target_components  # noqa: E402
from mapforge.validate.shp_boundary_fidelity import (  # noqa: E402
    _densify,
    _origin,
    _point_polyline_distance,
    _project,
    _target_outer_edges,
)
from mapforge.validate.smoothness import road_surface_polygon, sample_road_ref  # noqa: E402


ERROR_BINS = np.asarray([0.0, 0.20, 0.50, 0.75, 1.00, 1.50, 3.00])
ERROR_COLORS = ["#1a9850", "#91cf60", "#fee08b", "#fdae61", "#f46d43", "#a50026"]
ERROR_CMAP = ListedColormap(ERROR_COLORS)
ERROR_NORM = BoundaryNorm(ERROR_BINS, len(ERROR_COLORS), clip=True)
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def _stats(values) -> dict:
    a = np.asarray(values, float)
    if not len(a):
        return {"count": 0, "median_m": None, "p95_m": None, "max_m": None,
                "gt050_count": 0, "gt100_count": 0}
    return {
        "count": int(len(a)),
        "median_m": float(np.median(a)),
        "p95_m": float(np.percentile(a, 95)),
        "max_m": float(np.max(a)),
        "gt050_count": int(np.count_nonzero(a > 0.50)),
        "gt100_count": int(np.count_nonzero(a > 1.00)),
    }


def _densify_xy(points, ds=0.5):
    g = np.asarray(points, float)
    if len(g) < 2:
        return g
    seg = np.linalg.norm(np.diff(g, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-9])
    g = g[keep]
    if len(g) < 2:
        return g
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))])
    q = np.arange(0.0, s[-1], ds)
    if not len(q) or s[-1] - q[-1] > 1e-9:
        q = np.append(q, s[-1])
    return np.column_stack([np.interp(q, s, g[:, 0]), np.interp(q, s, g[:, 1])])


def _distance(points, line):
    return _point_polyline_distance(np.asarray(points, float), np.asarray(line, float))[0]


def _runs(points, distances, threshold=0.50):
    p = np.asarray(points, float)
    d = np.asarray(distances, float)
    if len(p) < 2:
        return []
    mask = d > threshold
    idx = np.flatnonzero(mask)
    if not len(idx):
        return []
    split = np.flatnonzero(np.diff(idx) > 1) + 1
    rows = []
    for run in np.split(idx, split):
        if not len(run):
            continue
        i0, i1 = int(run[0]), int(run[-1])
        length = float(np.linalg.norm(np.diff(p[i0:i1 + 1], axis=0), axis=1).sum()) \
            if i1 > i0 else 0.0
        rows.append({"start_index": i0, "end_index": i1,
                     "length_m": length, "max_m": float(d[run].max())})
    return rows


def _segments(points):
    p = np.asarray(points, float)
    return np.stack([p[:-1], p[1:]], axis=1) if len(p) >= 2 else np.zeros((0, 2, 2))


def _plot_error_line(ax, points, distances, lw=2.2, alpha=0.95):
    p = np.asarray(points, float)
    d = np.asarray(distances, float)
    if len(p) < 2:
        return
    colors = 0.5 * (d[:-1] + d[1:])
    ax.add_collection(LineCollection(_segments(p), cmap=ERROR_CMAP, norm=ERROR_NORM,
                                     array=colors, linewidths=lw, alpha=alpha, zorder=5))
    ax.scatter(p[:, 0], p[:, 1], c=d, cmap=ERROR_CMAP, norm=ERROR_NORM,
               s=12, edgecolors="none", zorder=6)


def _surface_union(root):
    from shapely.ops import unary_union

    polygons = []
    for road in root.findall("road"):
        pg = road_surface_polygon(road, ds=0.2)
        if not pg.is_empty:
            polygons.append(pg)
    return unary_union(polygons).buffer(0) if polygons else None


def _paving_union(root):
    from shapely.ops import unary_union

    polygons = [road_surface_polygon(road, ds=0.1) for road in root.findall("road")
                if road.get("name") == "junction_paving"]
    polygons = [p for p in polygons if not p.is_empty]
    return unary_union(polygons).buffer(0) if polygons else None


def _plot_surface(ax, root):
    union = _surface_union(root)
    if union is None or union.is_empty:
        return
    geoms = list(getattr(union, "geoms", [union]))
    for pg in geoms:
        xy = np.asarray(pg.exterior.coords)
        ax.fill(xy[:, 0], xy[:, 1], color="#6f746f", alpha=0.28, zorder=0)
        ax.plot(xy[:, 0], xy[:, 1], color="#31363b", lw=0.75, alpha=0.8, zorder=1)


def _source_manifest(path):
    path = Path(path)
    if not path.exists():
        return {"schema": "mapforge/source-lane-geometry-manifest/v1",
                "source_contexts": [], "lanes": []}
    return json.loads(path.read_text(encoding="utf-8"))


def _target_by_source(root):
    out = defaultdict(list)
    extracted = extract_target_components(root, ds=0.25, include_excluded=True)
    for row in extracted["components"]:
        item = dict(row)
        item["points"] = np.asarray(row["points"], float)
        out[row["source_lane_id"]].append(item)
    return dict(out)


def _lane_center_records(manifest, targets):
    rows = []
    for lane in manifest.get("lanes", []):
        comparable = bool((lane.get("comparison") or {}).get("eligible"))
        # 来源端状态与道路口部冲突的 connector 会被正式 G8 排除，但它们仍是
        # 用户目检最需要看到的差异；局部热图不得因“排除”而把红段藏掉。
        source_conflict = lane.get("source_conflict")
        role = lane.get("role") or "unknown"
        support = lane.get("support") or {}
        exclusion = support.get("reason") or lane.get("exclusion_code")
        if exclusion == "minimal-chain-source-fidelity-unmet":
            # 求解器未找到来源合格少段解只证明当前建模/拟合未达标，
            # 不能据此认定原始拓扑或几何错误。保留原排除码供复核。
            review_class = "minimal-chain-source-fidelity-unmet"
        elif exclusion == "source-end-state-conflict":
            review_class = "source-end-state-conflict"
        elif exclusion == "lane-transition-taper":
            review_class = "lane-transition-taper"
        elif role == "junction-via":
            review_class = "junction-via-comparable" if comparable else "junction-via-excluded"
        elif comparable:
            review_class = "ordinary-comparable"
        else:
            review_class = "other-excluded"
        sid = lane.get("source_lane_id")
        geom = lane.get("geometry") or {}
        source = np.asarray(geom.get("coordinates") or [], float)
        candidates = list(targets.get(sid) or [])
        if len(source) < 2 or not candidates:
            continue
        source_dense = _densify_xy(source, 0.5)
        # 同一 source id 在车道生灭/连接路场景可能对应多个 target occurrence。
        # 先按 provenance role/support_kind 缩小到同语义 occurrence，再在该 source
        # id 内选与来源几何最近的一支；绝不跨 source id 做全图最近邻。
        same_role = [x for x in candidates if x.get("role") == role]
        if same_role:
            candidates = same_role
        same_support = [x for x in candidates
                        if x.get("support_kind") == lane.get("support_kind")]
        if same_support:
            candidates = same_support
        dense_candidates = [_densify_xy(x["points"], 0.25) for x in candidates]
        dense_candidates = [x for x in dense_candidates if len(x) >= 2]
        if not dense_candidates:
            continue
        target_dense = min(
            dense_candidates,
            key=lambda x: float(np.median(_distance(source_dense, x))))
        d = _distance(source_dense, target_dense)
        runs = _runs(source_dense, d)
        # MAP 各车道终点可能沿斜停止线前后错开，而一个 OpenDRIVE road 的所有
        # 连接车道共享同一 contact plane。仅当超差只占末端不足 2m、主体 P95
        # 仍小于 0.30m 时标为表达冲突；这不会从总误差中删除，只是避免误判为
        # 整条道路拟合失败。
        if (review_class == "ordinary-comparable" and role == "approach"
                and d[-1] > 0.50 and float(np.percentile(d, 95)) < 0.30
                and runs and all(x["end_index"] == len(source_dense) - 1
                                 and x["length_m"] <= 2.0 for x in runs)):
            review_class = "staggered-stopline-endpoint"
        rows.append({"id": sid, "source": source_dense, "target": target_dense,
                     "distances": d, "stats": _stats(d), "runs_gt050": runs,
                     "comparison_eligible": comparable,
                     "role": role, "status": lane.get("status"),
                     "support_kind": lane.get("support_kind"),
                     "exclusion_code": exclusion,
                     "review_class": review_class,
                     "source_conflict": source_conflict})
    return rows


def _all_map_nodes():
    nodes = {}
    for path in sorted((ROOT / "v2x_map_xml").glob("map*.xml")):
        for node in parse_map_xml_all(str(path)):
            nodes[(node.region, node.node_id)] = node
    return nodes


def _map_link_records(case, root, manifest):
    contexts = [x for x in manifest.get("source_contexts", []) if x.get("role") == "main"]
    # A case filename is not an operational node identity: node13 contains
    # (21901,21901), and node16 contains (500,21801). Never silently report
    # zero measured reference points after looking up an invented filename ID.
    if len(contexts) != 1:
        raise ValueError(f'{case}: expected exactly one manifest main MAP identity')
    region, node_id = int(contexts[0]['region']), int(contexts[0]['node_id'])
    node = _all_map_nodes().get((region, node_id))
    if node is None:
        raise ValueError(f'{case}: source MAP identity {(region, node_id)} not found')
    roads = {road.get("name"): road for road in root.findall("road")
             if road.get("junction") in (None, "-1")}
    rows = []
    conflicts = {}
    for ctx in manifest.get("source_contexts", []):
        for item in ctx.get("refline_source_conflicts", []):
            conflicts[item.get("link_name")] = item
    for link in node.links:
        road = roads.get(link.name)
        if road is None or len(link.points) < 2:
            continue
        source_raw = project_map(link.points, node.ref_lat, node.ref_lon)
        target = sample_road_ref(road, 0.10)[0]
        d = _distance(source_raw, target)
        rows.append({"name": link.name, "source": source_raw, "target": target,
                     "distances": d, "stats": _stats(d), "runs_gt050": _runs(source_raw, d),
                     "review_class": ("source-impulse-conflict" if link.name in conflicts
                                      else "ordinary-comparable"),
                     "source_conflict": conflicts.get(link.name)})
    return rows


def _shp_boundary_records(root, src):
    lat0, lon0 = _origin(root)
    grouped = defaultdict(list)
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        for record in _target_outer_edges(road, ds=0.25):
            if record.get("boundary_evidence_trusted") is False:
                continue
            unsupported = record.get("exclusion_code") in {
                "source-extension", "source-support-unavailable",
                "source-support-too-short",
            }
            if unsupported and record.get("boundary_evidence_trusted") is not True:
                continue
            exclusion = record.get("exclusion_code")
            review_class = ("lane-transition-taper" if exclusion == "lane-transition-taper"
                            else "ordinary-comparable"
                            if record.get("eligibility") == "comparable"
                            else "other-excluded")
            grouped[(road.get("id"), record["source_lane_id"], review_class,
                     exclusion, record.get("support_kind"))].append(
                np.asarray(record["points"], float))

    rows = []
    for (road_id, source_id, review_class, exclusion, support_kind), target_parts in grouped.items():
            target = np.vstack(target_parts)
            candidates = [_densify(_project(g, lat0, lon0), ds=0.25)
                          for g in src.lane_boundary_geometries(source_id)]
            candidates = [g for g in candidates if len(g) >= 2]
            if not candidates:
                continue
            source = min(candidates, key=lambda g: float(np.median(_distance(target, g))))
            d, interior = _point_polyline_distance(source, target)
            # 只比较双方共同覆盖域；超出目标首末端的长源尾巴由正式 G8
            # endpoint/coverage 负责，不能在局部横向误差图里伪造几十米红线。
            keep = interior | (d <= 1.5)
            idx = np.flatnonzero(keep)
            if not len(idx):
                continue
            cuts = np.flatnonzero(np.diff(idx) > 1) + 1
            for part_i, run in enumerate(np.split(idx, cuts)):
                if len(run) < 2:
                    continue
                sp = source[run]
                sd = d[run]
                rows.append({"road_id": road_id, "id": source_id,
                             "part": part_i, "source": sp, "target": target,
                             "distances": sd, "stats": _stats(sd),
                             "runs_gt050": _runs(sp, sd),
                             "review_class": review_class,
                             "exclusion_code": exclusion,
                             "support_kind": support_kind})
    return rows


def _group_stats(rows, key="review_class"):
    groups = defaultdict(list)
    for row in rows:
        groups[str(row.get(key) or "unclassified")].extend(
            float(x) for x in row.get("distances", []))
    return {name: _stats(values) for name, values in sorted(groups.items())}


def _shp_junction_record(root, src, manifest):
    contexts = manifest.get("source_contexts", [])
    wanted = {str(x.get("junction")) for x in contexts if x.get("junction") is not None}
    junction = next((x for x in src.junctions if str(x.pid) in wanted), None)
    if junction is None:
        lat0, lon0 = _origin(root)
        junction, _distance_m = src.find_junction(lon0, lat0)
    paving = _paving_union(root)
    if junction is None or paving is None or paving.is_empty:
        return None
    lat0, lon0 = _origin(root)
    source = _densify(_project(junction.polygon, lat0, lon0), ds=0.20)
    boundary = paving.boundary
    parts = list(getattr(boundary, "geoms", [boundary]))
    target = max((np.asarray(g.coords) for g in parts), key=len)
    d = _distance(source, target)
    td = _distance(_densify_xy(target, 0.20), source)
    return {"id": str(junction.pid), "source": source, "target": target,
            "distances": d, "stats": _stats(d), "target_to_source": _stats(td),
            "runs_gt050": _runs(source, d)}


def _serializable_record(row):
    return {key: value for key, value in row.items()
            if key not in {"source", "target", "distances"}}


def render(case, pipeline, xodr_path, manifest_path, output, shp_dir=None,
           profile="ibd-smarteditor-v1"):
    root = ET.parse(str(xodr_path)).getroot()
    manifest = _source_manifest(manifest_path)
    targets = _target_by_source(root)
    lanes = _lane_center_records(manifest, targets)
    links, boundaries, junction = [], [], None
    if pipeline == "map":
        links = _map_link_records(case, root, manifest)
    else:
        src = ProfileSource(str(shp_dir), profile)
        boundaries = _shp_boundary_records(root, src)
        junction = _shp_junction_record(root, src, manifest)

    fig, axes = plt.subplots(2, 2, figsize=(18, 14))
    ax_all, ax_center, ax_ref, ax_detail = axes.ravel()
    for ax in axes.ravel():
        _plot_surface(ax, root)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.18)
        ax.set_xlabel("local x (m)")
        ax.set_ylabel("local y (m)")

    for row in lanes:
        ax_all.plot(row["target"][:, 0], row["target"][:, 1], color="#2166ac",
                    lw=0.55, alpha=0.35)
        _plot_error_line(ax_all, row["source"], row["distances"], lw=1.1, alpha=0.7)
        _plot_error_line(ax_center, row["source"], row["distances"], lw=1.3, alpha=0.8)
    ax_all.set_title("全图：源车道中心按到目标同身份车道中心的误差着色")
    ax_center.set_title("路口中心 +/-50m：源车道中心局部误差")
    ax_center.set_xlim(-50, 50)
    ax_center.set_ylim(-50, 50)

    reference_rows = links if pipeline == "map" else boundaries
    for row in reference_rows:
        ax_ref.plot(row["target"][:, 0], row["target"][:, 1], color="#2166ac",
                    lw=1.0, alpha=0.7)
        _plot_error_line(ax_ref, row["source"], row["distances"], lw=2.2)
    ax_ref.set_title("MAP Link.points→planView" if pipeline == "map"
                     else "SHP 物理外边界→OpenDRIVE 同身份外缘")

    detail_rows = sorted(reference_rows or lanes,
                         key=lambda x: x["stats"]["max_m"] or -1, reverse=True)
    if detail_rows:
        worst = detail_rows[0]
        ax_detail.plot(worst["target"][:, 0], worst["target"][:, 1], color="#2166ac",
                       lw=1.5, label="OpenDRIVE target")
        _plot_error_line(ax_detail, worst["source"], worst["distances"], lw=3.0)
        allp = np.vstack([worst["source"], worst["target"]])
        lo, hi = allp.min(axis=0), allp.max(axis=0)
        pad = max(5.0, 0.08 * float(np.max(hi - lo)))
        ax_detail.set_xlim(lo[0] - pad, hi[0] + pad)
        ax_detail.set_ylim(lo[1] - pad, hi[1] + pad)
        label = worst.get("name") or worst.get("road_id") or worst.get("id")
        ax_detail.set_title(f"最坏局部：{label}\nmax={worst['stats']['max_m']:.3f}m，"
                            f"P95={worst['stats']['p95_m']:.3f}m", loc='right', fontsize=10)
    if junction is not None:
        ax_center.plot(junction["source"][:, 0], junction["source"][:, 1],
                       color="#762a83", lw=2.0, label="SHP junction polygon")
        ax_center.plot(junction["target"][:, 0], junction["target"][:, 1],
                       color="#1b7837", lw=1.5, label="XODR paving union")
        ax_center.legend(loc="best")

    sm = plt.cm.ScalarMappable(norm=ERROR_NORM, cmap=ERROR_CMAP)
    sm.set_array([])
    fig.colorbar(sm, ax=axes.ravel().tolist(), shrink=0.72, pad=0.02, extend="max",
                 label="源点到同身份 OpenDRIVE 几何距离 (m)")
    fig.suptitle(f"{case} {pipeline.upper()}→OpenDRIVE 局部保真审计（不使用全图最近邻掩盖）")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(fig)

    report = {
        "schema": "mapforge/local-geometry-audit/v1",
        "case": case,
        "pipeline": pipeline,
        "artifact": str(Path(xodr_path)),
        "lane_centers": {
            "count": len(lanes),
            "aggregate": _stats([float(v) for row in lanes for v in row["distances"]]),
            "by_review_class": _group_stats(lanes),
            "worst": [_serializable_record(x) for x in sorted(
                lanes, key=lambda q: q["stats"]["max_m"] or -1, reverse=True)[:20]],
        },
        "map_link_points" if pipeline == "map" else "shp_outer_boundaries": {
            "count": len(reference_rows),
            "aggregate": _stats([float(v) for row in reference_rows for v in row["distances"]]),
            "by_review_class": _group_stats(reference_rows),
            "worst": [_serializable_record(x) for x in detail_rows[:20]],
        },
        "shp_junction_boundary": _serializable_record(junction) if junction else None,
    }
    report_path = output.with_suffix(".json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(output)
    print(report_path)
    print(json.dumps({
        "lane_centers": report["lane_centers"]["aggregate"],
        "lane_centers_by_class": report["lane_centers"]["by_review_class"],
        "reference": report[
            "map_link_points" if pipeline == "map" else "shp_outer_boundaries"
        ]["aggregate"],
        "reference_by_class": report[
            "map_link_points" if pipeline == "map" else "shp_outer_boundaries"
        ]["by_review_class"],
        "shp_junction_boundary": (
            report["shp_junction_boundary"]["stats"]
            if report["shp_junction_boundary"] else None),
    }, ensure_ascii=False))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("case")
    parser.add_argument("pipeline", choices=("map", "shp"))
    parser.add_argument("xodr")
    parser.add_argument("manifest")
    parser.add_argument("output")
    parser.add_argument("--shp-dir", default=str(ROOT / "shp_0222-0326"))
    parser.add_argument("--profile", default="ibd-smarteditor-v1")
    args = parser.parse_args()
    render(args.case, args.pipeline, args.xodr, args.manifest, args.output,
           shp_dir=args.shp_dir, profile=args.profile)
