"""路口物理面证据：源多边形与移动口部的真实车道尾带。"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from shapely.affinity import translate


def written_mouth_specs(root, decisions):
    """口部身份来自转换决策；几何、median 来自最终写出的 lane.width。"""
    from mapforge.validate.smoothness import sample_road_ref, lane_edges_kinematics_at
    result = []
    for i, decision in enumerate(decisions):
        rid = str(decision.get("road_id", 10+i))  # historical isolated probes
        road = root.find(f"road[@id='{rid}']")
        if road is None:
            raise ValueError(f"missing physical road {rid}")
        pts, _, hh = sample_road_ref(road, .25)
        spec = {**decision, "road_id":rid, "pose":[*pts[-1], float(hh[-1])]}
        sec = road.findall("lanes/laneSection")[-1]
        if any(x.get("id") == "1" and x.get("type") == "median" for x in sec.findall("left/lane")):
            edges = lane_edges_kinematics_at(road, float(road.get("length"))-1e-7, "left")
            spec["median_interval"] = [edges[0][0], edges[1][0]]
        result.append(spec)
    return result


def replace_source_paving(root, src, junction, project, decisions):
    """替换候选辅助铺面，不修改普通道路、connecting road、laneLink 或 signals。"""
    from mapforge.adapters.opendrive import writer as W
    from mapforge.ops.map_to_xodr import _mouth_apron_axes
    mouths = written_mouth_specs(root, decisions)
    doc = W.XodrDoc("candidate-source-surface")
    retained = [rd for rd in root.findall("road") if rd.get("name") != "junction_paving"]
    for rd in retained:
        doc.add_road(W.Road(int(rd.get("id"))))  # reserve IDs only
    original_count = len(doc.roads)
    stats = append_source_paving(doc, src, junction, project, mouths,
                                  _mouth_apron_axes(mouths) or [None])
    for rd in list(root.findall("road")):
        if rd.get("name") == "junction_paving":
            root.remove(rd)
    j = root.find("junction")
    at = list(root).index(j) if j is not None else len(root)
    for rd in doc.roads[original_count:]:
        temporary = ET.Element("OpenDRIVE")
        doc._road_el(temporary, rd)
        root.insert(at, temporary.find("road"))
        at += 1
    stats["source_surface_status"] = ("FAIL" if any(stats[k] for k in
        ("mouth_tail_source_issues", "mouth_tail_bridge_issues", "median_tail_issues")) else "GENERATED")
    return stats


def _polygons(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    return [p for p in geometry.geoms if p.geom_type == "Polygon" and p.area > 1e-8]


def directional_sweep(polygon, direction, length):
    """沿指定道路方向扫出局部缝隙，绝不是全局凸包/buffer 扩边。"""
    if polygon.interiors:
        raise ValueError("directional sweep must not fill source islands")
    direction = np.asarray(direction, float)
    norm = np.linalg.norm(direction)
    if not np.isfinite(norm) or norm < 1e-9 or not math.isfinite(length) or length < 0:
        raise ValueError("invalid directional sweep")
    shift = direction/norm*float(length)
    moving = translate(polygon, xoff=shift[0], yoff=shift[1])
    pieces = [polygon, moving]
    for ring in [polygon.exterior]:
        coords = np.asarray(ring.coords)
        for a, b in zip(coords[:-1], coords[1:]):
            quad = Polygon([a, b, b+shift, a+shift])
            if quad.area > 1e-10:
                pieces.append(quad)
    return unary_union(pieces)


def source_tail_regions(src, junction, project, mouths):
    """纯几何采集，不做凸包或洞填充，保留每块源身份用于诊断。"""
    base = Polygon(project(junction.polygon))
    if not base.is_valid:
        raise ValueError("invalid source junction polygon")
    pieces, issues = [], []
    getter = getattr(src, "lane_boundary_geometries", None)
    for spec in mouths:
        records = [lane for pid in [spec["enter_link"], *spec.get("leave_links", [])]
                   for lane in src.lanes_of(pid) if len(lane.geometry) >= 2]
        pose = spec["pose"]
        point = np.array(pose[:2], float)
        along = np.array([math.cos(pose[2]), math.sin(pose[2])])
        normal = np.array([-along[1], along[0]])
        end = max([1.] + [float(((project(r.geometry)-point) @ along).max())+1.
                         for r in records])
        slab = Polygon([point+s*along+t*normal for s, t in
                        ((-.75, -100.), (end, -100.), (end, 100.), (-.75, 100.))])
        for lane in records:
            pair = getter(lane.lane_pid) if getter else []
            if len(pair) != 2:
                issues.append({"source_lane": lane.lane_pid, "reason": "missing-boundaries"})
                continue
            a, b = [project(x) for x in pair]
            if np.dot(a[-1]-a[0], b[-1]-b[0]) < 0:
                b = b[::-1]
            polygon = Polygon(np.vstack([a, b[::-1]]))
            if not polygon.is_valid:
                issues.append({"source_lane": lane.lane_pid, "reason": "invalid-boundaries"})
                continue
            tail = polygon.intersection(slab)
            if not tail.is_empty:
                pieces.append({"source_lane": lane.lane_pid, "road_id": spec["road_id"],
                               "geometry": tail, "gap_to_junction_m": float(tail.distance(base))})
    union = unary_union([base, *[p["geometry"] for p in pieces]])
    components = list(union.geoms) if union.geom_type == "MultiPolygon" else [union]
    summary = {"type": union.geom_type, "source_junction_area_m2": float(base.area),
               "components": [{"area_m2": float(p.area), "bounds": list(p.bounds),
                               "gap_to_junction_m": float(p.distance(base)),
                               "holes": len(p.interiors)} for p in components],
               "source_tail_count": len(pieces), "issues": issues}
    return base, pieces, union, summary


def bridge_source_tails(base, pieces, mouths, *, max_sweep_m=1.5, overlap_m=.25):
    """按每条 physical leg 的真实边界带分块，仅补短的沿路缝隙。"""
    result, issues = [], []
    for spec in mouths:
        shape = unary_union([p["geometry"] for p in pieces if p["road_id"] == spec["road_id"]])
        direction = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        for polygon in _polygons(shape):
            if polygon.interiors:
                issues.append({"road_id":spec["road_id"], "reason":"source-tail-has-island"})
                continue
            # 一个角碰到路口面不等于整条横向端帽接通。沿所有朝前端边的
            # 点发射短射线，取整端面所需距离，避免斜端帽留下狭缝。
            forward_gaps = []
            coords = np.asarray(orient(polygon, sign=1).exterior.coords)
            missing = False
            for a, b in zip(coords[:-1], coords[1:]):
                edge = b-a
                edge_len = np.linalg.norm(edge)
                if edge_len < 1e-8 or np.dot([edge[1], -edge[0]], direction)/edge_len < .5:
                    continue
                for w in np.linspace(0., 1., max(2, int(edge_len/.25)+2)):
                    point = a+w*edge
                    if base.covers(Point(point)):
                        forward_gaps.append(0.)
                        continue
                    hit = base.intersection(LineString([point, point+direction*max_sweep_m]))
                    if hit.is_empty:
                        missing = True
                    else:
                        forward_gaps.append(float(Point(point).distance(hit)))
            shift = 0.
            if missing or not forward_gaps:
                issues.append({"road_id":spec["road_id"], "reason":"end-cap-outside-repair-budget",
                               "gap_m":float(polygon.distance(base))})
                continue
            if max(forward_gaps) > 1e-6:
                if directional_sweep(polygon, direction, max_sweep_m).intersection(base).area < .05:
                    issues.append({"road_id":spec["road_id"], "reason":"gap-outside-repair-budget",
                                   "gap_m":float(polygon.distance(base))})
                    continue
                shift = min(max_sweep_m, max(forward_gaps)+overlap_m)
            patched = directional_sweep(polygon, direction, shift) if shift else polygon
            result.append({"road_id":spec["road_id"], "geometry":patched,
                           "axis": direction, "sweep_m":shift,
                           "added_area_m2":float(patched.area-polygon.area),
                           "gap_m":float(polygon.distance(base))})
    return result, issues


def append_source_paving(doc, src, junction, project, mouths, axes, junction_id=1):
    """中心面与实际 carriageway 尾带独立建面，避免跨中央带取 min/max。"""
    from mapforge.adapters.opendrive.writer import add_paving_road
    base, pieces, _, summary = source_tail_regions(src, junction, project, mouths)
    tails, issues = bridge_source_tails(base, pieces, mouths)
    medians, median_issues = source_median_tails(base, tails, mouths)
    stats = {"mouth_tail_source_components":summary["components"],
             "mouth_tail_source_issues":summary["issues"], "mouth_tail_bridge_issues":issues,
             "mouth_tail_bridges":[{k:v for k,v in p.items() if k not in {"geometry","axis"}}
                                   for p in tails]}
    stats["median_tail_issues"] = median_issues
    stats["median_tail_areas_m2"] = [p["geometry"].area for p in medians]
    regions = [(base, axis, "source-polygon", "restricted") for axis in axes[:2]]
    used = {r.road_id for r in doc.roads}
    ids = iter(i for i in range(50, 100) if i not in used)
    count = 0
    for polygon, axis, support, lane_type in regions:
        for part in _polygons(polygon):
            if part.interiors:
                raise ValueError("paving region with an island cannot be reduced to exterior")
            rid = next(ids)
            if add_paving_road(doc, np.asarray(part.exterior.coords), junction_id,
                               road_id=rid, smooth_profile=True, preferred_axis=axis,
                               lane_type=lane_type,
                               overlap_m=.25,
                               provenance={"eligibility":"excluded", "role":"paving",
                                           "status":"APPROXIMATED" if support != "source-polygon" else "TRANSFORMED",
                                           "support_kind":support, "travel_direction":"with_s",
                                           "exclusion_code":"source-polygon-paving"}):
                count += 1
    for spec in mouths:
        supported = [x["geometry"] for x in tails if x["road_id"] == spec["road_id"]]
        islands = [x["geometry"] for x in medians if x["road_id"] == spec["road_id"]]
        if not supported:
            continue
        if spec.get("median_interval") is not None and not islands:
            stats["mouth_tail_bridge_issues"].append({"road_id":spec["road_id"],
                                                    "reason":"median-tail-unresolved"})
            continue
        family = unary_union(supported+islands)
        axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        # 只在原始 INTERSECTION 内增加端帽重叠；既不向街角扩大，也不
        # 延长 median。斜切口不能只靠两个独立多项式端点相碰。
        caps = [directional_sweep(p, axis, .75).intersection(base)
                for p in _polygons(family) if not p.interiors]
        family = unary_union([family, *caps])
        _append_surface_family(doc, family, unary_union(islands),
                               spec, next(ids), junction_id,
                               [p["source_lane"] for p in pieces if p["road_id"] == spec["road_id"]])
        count += 1
    stats.update(paving="source-boundary-regions", paving_roads=count)
    return stats


def _append_surface_family(doc, surface, median, spec, road_id, junction_id, source_ids):
    """一个物理尾带共用参考线和边界系数：沥青 / median / 沥青。

    三条辅助带仅供 sim Profile 的路面显示，全部不带 laneLink。restricted
    并非通用禁行标志，消费端必须排除这些 road；strict AD Profile 不允许。
    共享边界之差生成 width，杜绝独立扫掠造成的厘米级细缝。
    """
    from scipy.interpolate import PchipInterpolator, CubicHermiteSpline
    from mapforge.adapters.opendrive import writer as W
    point = np.asarray(spec["pose"][:2], float)
    axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
    normal = np.array([-axis[1], axis[0]])
    coords = np.vstack([np.asarray(p.exterior.coords) for p in _polygons(surface)])
    s = (coords-point)@axis
    s0, s1 = float(s.min())-.25, float(s.max())+.25
    us = np.linspace(0., s1-s0, max(3, int(s1-s0)+2))
    reference = point+s0*axis
    rows = []
    for u in us:
        p = reference+u*axis
        ray = LineString([p-100*normal, p+100*normal])

        def intervals(geometry):
            cut = geometry.intersection(ray)
            result = []
            for line in getattr(cut, "geoms", [cut]):
                if line.geom_type == "LineString" and not line.is_empty:
                    t = (np.asarray(line.coords)-p)@normal
                    result.append((float(t.min()), float(t.max())))
            return sorted(result)

        ranges = intervals(surface)
        if not ranges:
            rows.append(None)
            continue
        # 此 min/max 只跨有身份的 median；未知孔洞不能被当成路面扫过。
        if any(b[0]-a[1] > .02 for a,b in zip(ranges[:-1], ranges[1:])):
            raise ValueError(f"unresolved gap inside source surface family: road={spec['road_id']}, "
                             f"s={u+s0:.3f}, ranges={ranges}")
        low, high = ranges[0][0], ranges[-1][1]
        mid = intervals(median)
        if len(mid) > 1:
            raise ValueError("multiple islands need separate surface families")
        if mid:
            m0, m1 = mid[0]
        else:
            anchor = sum(spec.get("median_interval", [low, low]))/2
            m0 = m1 = float(np.clip(anchor, low, high))
        rows.append([low, m0, m1, high])
    valid = [i for i,r in enumerate(rows) if r is not None]
    if not valid:
        raise ValueError("empty source surface family")
    for i,r in enumerate(rows):
        if r is None:
            if valid[0] < i < valid[-1]:
                raise ValueError("unsupported interior surface interval")
            rows[i] = rows[min(valid, key=lambda j:abs(i-j))]
    values = np.asarray(rows)
    widths = np.diff(values, axis=1)
    if np.min(widths) < -1e-8:
        raise ValueError("unordered source surface samples")
    slopes = np.column_stack([PchipInterpolator(us, values[:,i]).derivative()(us)
                              for i in range(4)])
    scales = np.ones(len(us))
    # 共同缩放每个站点的全部边界切线；保证 width 的两个中间 Bezier
    # 控制点非负，从构造上禁止边界交叉，且相邻区间共用同一切线。
    for i, h in enumerate(np.diff(us)):
        dw0, dw1 = np.diff(slopes[i]), np.diff(slopes[i+1])
        for j in range(3):
            if dw0[j] < 0:
                scales[i] = min(scales[i], max(0., 3*widths[i,j]/(-h*dw0[j])))
            if dw1[j] > 0:
                scales[i+1] = min(scales[i+1], max(0., 3*widths[i+1,j]/(h*dw1[j])))
    curves = [CubicHermiteSpline(us, values[:,i], slopes[:,i]*scales) for i in range(4)]
    road = W.Road(road_id, name="junction_paving", junction=junction_id)
    road.add_geometry("line", *reference, spec["pose"][2], s1-s0)
    sec = W.LaneSection(0.)
    for j, kind in enumerate(("restricted", "median", "restricted")):
        lane = W.Lane(j+1, kind, provenance={"eligibility":"excluded", "role":"paving",
                      "status":"APPROXIMATED", "support_kind":"source-boundary-family",
                      "source_ids":source_ids, "physical_road":spec["road_id"],
                      "travel_direction":"against_s", "exclusion_code":"source-polygon-paving"})
        for i in range(len(us)-1):
            coeff = curves[j+1].c[:,i]-curves[j].c[:,i]
            roots = np.roots(np.trim_zeros(np.polyder(coeff), "f"))
            probes = [0., us[i+1]-us[i]] + [r.real for r in roots
                       if abs(r.imag) < 1e-9 and 0 < r.real < us[i+1]-us[i]]
            if min(np.polyval(coeff, probes)) < -1e-7:
                raise ValueError("crossing source surface boundaries")
            lane.add_width(*coeff[::-1], s_offset=float(us[i]))
        sec.left.append(lane)
    for i in range(len(us)-1):
        road.add_offset(float(us[i]), *curves[0].c[:,i][::-1])
    road.sections.append(sec)
    doc.add_road(road)


def source_median_tails(base, tails, mouths):
    """仅延续已存在的 median；左右界取对应两侧实测车道尾带，非补车道。

    口部必须带最终写出 median 的横向范围作为身份锚点。不跨越新岔口、
    不填独立 source polygon 的孔，也不把未知间隙自动认成中央分隔带。
    """
    result, issues = [], []
    for spec in mouths:
        interval = spec.get("median_interval")
        if interval is None or interval[1]-interval[0] < .05:
            continue
        origin = np.asarray(spec["pose"][:2], float)
        axis = np.array([math.cos(spec["pose"][2]), math.sin(spec["pose"][2])])
        normal = np.array([-axis[1], axis[0]])
        parts = [t["geometry"] for t in tails if t["road_id"] == spec["road_id"]]
        geometry = unary_union(parts)
        if geometry.is_empty:
            continue
        max_s = max(float(((np.asarray(p.exterior.coords)-origin)@axis).max())
                    for p in _polygons(geometry))
        previous = sum(interval)/2
        rows = []
        for s in np.linspace(-.5, max_s, max(3, int((max_s+.5)/.25)+2)):
            point = origin+s*axis
            cut = geometry.intersection(LineString([point-100*normal, point+100*normal]))
            ranges = []
            for line in getattr(cut, "geoms", [cut]):
                if line.geom_type != "LineString" or line.is_empty:
                    continue
                v = (np.asarray(line.coords)-point)@normal
                ranges.append((float(v.min()), float(v.max())))
            ranges.sort()
            gaps = [(a[1], b[0]) for a, b in zip(ranges[:-1], ranges[1:])
                    if b[0]-a[1] > .01 and a[1]-.5 <= previous <= b[0]+.5]
            if len(gaps) != 1:
                if not rows and s <= 1.:
                    # 斜边界的共同支持区可能比口部稍短；最多查找 1m，
                    # 后面以已写 median 端态补短缺测带，并显式留痕。
                    continue
                if not rows:
                    issues.append({"road_id":spec["road_id"], "reason":"median-gap-ambiguous",
                                   "s":float(s), "expected":interval, "ranges":ranges})
                break
            lo, hi = gaps[0]
            if not rows and (abs(lo-interval[0]) > .5 or abs(hi-interval[1]) > .5):
                issues.append({"road_id":spec["road_id"], "reason":"median-mouth-source-mismatch"})
                break
            if not rows and s > -.49:
                anchor = origin-.5*axis
                rows.append((anchor+normal*interval[0], anchor+normal*interval[1]))
            rows.append((point+normal*lo, point+normal*hi))
            previous = (lo+hi)/2
            # 已完全进入原始路口面则终止，不延长分隔带去切断转向。
            if base.covers(LineString([rows[-1][0], rows[-1][1]])):
                break
        if len(rows) < 2:
            issues.append({"road_id":spec["road_id"], "reason":"median-insufficient-boundary-support"})
            continue
        polygon = Polygon(np.vstack([[x[0] for x in rows], [x[1] for x in rows[::-1]]]))
        # 去除与路口面重叠，岛头以源 INTERSECTION 开始处为上限。
        # 和车道尾带一样，只允许有预算的沿路延伸来接源路口面。
        median_parts, join_issues = bridge_source_tails(base,
            [{"road_id":spec["road_id"], "geometry":polygon}], [spec])
        if join_issues:
            issues.extend(join_issues)
            continue
        polygon = median_parts[0]["geometry"].difference(base)
        for part in _polygons(polygon):
            if not part.interiors and part.area > .05:
                result.append({"road_id":spec["road_id"], "geometry":part, "axis":axis})
    return result, issues
