"""用共享边界求解 SHP ordinary road，再物化为等价 ``lane.width``。

内部先把每条物理车道的累计外边界作为同一族约束求解，避免多条独立宽度
拟合后累积出蛇形误差。最终写出时，相邻累计边界做精确三次多项式差分，
恢复为兼容性更好的 ``lane.width``。这样仍保持共享边界几何，但不会依赖
esmini 3.6 等消费端尚未实现的 ``lane.border`` 求值。

本模块不增加 planView primitive，不增加 laneSection，也不改变 laneLink 拓扑。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from lxml import etree
from scipy.interpolate import CubicSpline, UnivariateSpline

from mapforge.validate.smoothness import (
    _edge_world_curvature,
    _ref_kappa_at,
    edge_shape_quality,
    lane_edges_kinematics_at,
)


def _provenance(lane) -> dict:
    item = lane.find("userData[@code='mapforge.provenance/v1']")
    if item is None or not item.get("value"):
        return {}
    try:
        return json.loads(item.get("value"))
    except (TypeError, json.JSONDecodeError):
        return {}


def _ordinary(road) -> bool:
    return (road.get("junction", "-1") == "-1"
            and road.get("name") != "junction_paving")


def _fit_track(stations, values, comparable, source_tol=0.25, inferred_tol=1.50):
    x = np.asarray(stations, float)
    raw = np.asarray(values, float)
    mask = np.asarray(comparable, bool)
    if len(x) < 4 or x[-1] - x[0] < 20.0 or np.any(np.diff(x) <= 1e-8):
        curve = CubicSpline(x, raw, bc_type="natural") if len(x) >= 3 else None
        slopes = (curve(x, 1) if curve is not None
                  else np.full(len(x), (raw[-1] - raw[0]) / max(x[-1] - x[0], 1e-9)))
        return raw, np.asarray(slopes, float), 0.0

    weights = np.where(mask, 4.0, 0.15)
    if mask[0]:
        weights[0] = 8.0
    if mask[-1]:
        weights[-1] = 8.0
    dense = np.linspace(x[0], x[-1], max(101, int(x[-1] - x[0]) * 2))
    candidates = []
    for budget in (0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25,
                   0.35, 0.50, 0.75, 1.00):
        try:
            spline = UnivariateSpline(
                x, raw, w=weights, k=3, s=len(x) * budget ** 2)
            smooth = spline(x)
        except Exception:
            continue
        # 道路两端（尤其 junction mouth）必须与连接路求解时使用的位置一致。
        # 先固定端点值，再由整条 natural cubic 统一求导；不能在求完导数后只把
        # 末点位置硬拉回去，否则最后一个 laneSection 会形成明显尖角。
        smooth[0] = raw[0]
        smooth[-1] = raw[-1]
        source_dev = (float(np.max(np.abs(smooth[mask] - raw[mask])))
                      if np.any(mask) else 0.0)
        inferred_dev = (float(np.max(np.abs(smooth[~mask] - raw[~mask])))
                        if np.any(~mask) else 0.0)
        if source_dev > source_tol + 1e-9 or inferred_dev > inferred_tol + 1e-9:
            continue
        natural = CubicSpline(x, smooth, bc_type="natural")
        score = (float(np.max(np.abs(natural(dense, 3)))),
                 float(np.max(np.abs(natural(dense, 2)))), source_dev)
        slopes = np.asarray(natural(x, 1), float)
        candidates.append((score, np.asarray(smooth, float), slopes))
    if not candidates:
        curve = CubicSpline(x, raw, bc_type="natural")
        return raw, np.asarray(curve(x, 1), float), 0.0
    _score, fitted, slopes = min(candidates, key=lambda item: item[0])
    return fitted, slopes, float(np.max(np.abs(fitted - raw)))


def _linked_chains(sections, side):
    occurrences = {
        (si, lane.get("id")): lane
        for si, section in enumerate(sections)
        for lane in sorted(section.findall(f"{side}/lane"),
                           key=lambda item: abs(int(item.get("id"))))
    }
    successors = {}
    predecessors = set()
    for (si, lane_id), lane in occurrences.items():
        if si + 1 >= len(sections):
            continue
        link = lane.find("link/successor")
        next_id = link.get("id") if link is not None else lane_id
        nxt = (si + 1, next_id)
        if nxt in occurrences:
            successors[(si, lane_id)] = nxt
            predecessors.add(nxt)
    chains = []
    for start in [key for key in occurrences if key not in predecessors]:
        chain, current = [], start
        while current is not None and current not in chain:
            chain.append(current)
            current = successors.get(current)
        chains.append(chain)
    return occurrences, successors, chains


def _center_tracks(road, sections, starts, ends, side):
    occurrences, successors, chains = _linked_chains(sections, side)
    target = {}
    max_adjust = 0.0
    for chain in chains:
        stations = [starts[chain[0][0]]] + [ends[key[0]] for key in chain]
        values, comparable = [], []
        for q, key in enumerate(chain):
            si, _lane_id = key
            lanes = sorted(sections[si].findall(f"{side}/lane"),
                           key=lambda item: abs(int(item.get("id"))))
            lane_index = lanes.index(occurrences[key])
            prov = _provenance(occurrences[key])
            is_comparable = prov.get("eligibility") == "comparable"
            if q == 0:
                edges = lane_edges_kinematics_at(road, starts[si] + 1e-5, side)
                values.append(float((edges[lane_index][0] + edges[lane_index + 1][0]) / 2.0))
                comparable.append(is_comparable)
            edges = lane_edges_kinematics_at(road, ends[si] - 1e-5, side)
            values.append(float((edges[lane_index][0] + edges[lane_index + 1][0]) / 2.0))
            comparable.append(is_comparable)
        fitted, slopes, adjustment = _fit_track(stations, values, comparable)
        max_adjust = max(max_adjust, adjustment)
        target[(chain[0], 0)] = (float(fitted[0]), float(slopes[0]))
        for q, key in enumerate(chain):
            endpoint = (float(fitted[q + 1]), float(slopes[q + 1]))
            target[(key, 1)] = endpoint
            if q + 1 < len(chain):
                target[(chain[q + 1], 0)] = endpoint
    return occurrences, successors, target, max_adjust


def _derive_borders(road, sections, starts, ends, side, target):
    sign = 1.0 if side == "left" else -1.0
    borders = {}
    for si, section in enumerate(sections):
        lanes = sorted(section.findall(f"{side}/lane"),
                       key=lambda item: abs(int(item.get("id"))))
        for endpoint, station in ((0, starts[si]), (1, ends[si])):
            sample_s = station + (1e-5 if endpoint == 0 else -1e-5)
            edges = lane_edges_kinematics_at(road, sample_s, side)
            inner_value, inner_slope = float(edges[0][0]), float(edges[0][1])
            for k, lane in enumerate(lanes):
                center_value, center_slope = target.get(
                    ((si, lane.get("id")), endpoint),
                    (float((edges[k][0] + edges[k + 1][0]) / 2.0),
                     float((edges[k][1] + edges[k + 1][1]) / 2.0)),
                )
                outer_value = 2.0 * center_value - inner_value
                outer_slope = 2.0 * center_slope - inner_slope
                # laneSection 内的车道生灭必须是几何收放，而不是外缘台阶。
                # 新生车道在起点与其内边界重合且同切向；消失车道在终点同理。
                # 首/末 section 的道路端点不属于生灭事件，不能误收为零宽。
                born_here = (endpoint == 0 and si > 0
                             and lane.find("link/predecessor") is None)
                dies_here = (endpoint == 1 and si + 1 < len(sections)
                              and lane.find("link/successor") is None)
                if born_here or dies_here:
                    outer_value = inner_value
                    outer_slope = inner_slope
                if sign * (outer_value - inner_value) < -1e-6:
                    return None
                borders[((si, lane.get("id")), endpoint)] = (
                    outer_value, outer_slope)
                inner_value, inner_slope = outer_value, outer_slope
    return borders


def _raw_center_targets(road, sections, starts, ends, side, occurrences):
    """读取写出前合法横断面的车道中心端点，供受约束回投使用。"""
    target = {}
    for (si, lane_id), lane in occurrences.items():
        lanes = sorted(sections[si].findall(f"{side}/lane"),
                       key=lambda item: abs(int(item.get("id"))))
        lane_index = lanes.index(lane)
        for endpoint, station in ((0, starts[si]), (1, ends[si])):
            sample_s = station + (1e-5 if endpoint == 0 else -1e-5)
            edges = lane_edges_kinematics_at(road, sample_s, side)
            target[((si, lane_id), endpoint)] = tuple(
                float((edges[lane_index][q] + edges[lane_index + 1][q]) / 2.0)
                for q in (0, 1))
    return target


def _blend_targets(raw, smooth, alpha):
    """在原始合法车道族与平滑目标间插值；alpha=0 必须保持原横断面。"""
    return {
        key: (raw[key][0] + alpha * (value[0] - raw[key][0]),
              raw[key][1] + alpha * (value[1] - raw[key][1]))
        for key, value in smooth.items()
    }


def _mark_narrow_extension_tapers(road, sections, starts, ends, side,
                                   width_threshold=0.4):
    """把最终 border 模型中不足行车宽度的 extension 标为渐变带。"""
    marked = 0
    for si, section in enumerate(sections):
        lanes = sorted(section.findall(f"{side}/lane"),
                       key=lambda item: abs(int(item.get("id"))))
        if not lanes:
            continue
        length = ends[si] - starts[si]
        samples = np.linspace(starts[si] + min(1e-5, length * 0.01),
                              ends[si] - min(1e-5, length * 0.01), 21)
        minimum = [float("inf")] * len(lanes)
        for station in samples:
            edges = lane_edges_kinematics_at(road, float(station), side)
            if len(edges) != len(lanes) + 1:
                continue
            for k in range(len(lanes)):
                minimum[k] = min(
                    minimum[k], abs(float(edges[k + 1][0] - edges[k][0])))
        for lane, width in zip(lanes, minimum):
            provenance = _provenance(lane)
            if (lane.get("type") != "driving"
                    or provenance.get("exclusion_code") != "source-extension"
                    or width >= width_threshold):
                continue
            provenance.update({
                "support_kind": "lane-transition-ribbon",
                "exclusion_code": "lane-transition-taper",
                "source_exclusion_code": "source-extension",
                "geometry_adjustment": "narrow-extension-taper",
                "minimum_width_m": round(float(width), 6),
            })
            item = lane.find("userData[@code='mapforge.provenance/v1']")
            if item is not None:
                item.set("value", json.dumps(
                    provenance, ensure_ascii=False, separators=(",", ":")))
                marked += 1
    return marked


def _cap_extension_speeds(road, sections, starts, ends, sample_step=0.25):
    """按最终世界坐标曲率反算无源 extension 的安全设计速度。"""
    capped = 0
    minimum_cap = float("inf")
    maximum_reduction = 0.0
    for si, section in enumerate(sections):
        s0, s1 = starts[si], ends[si]
        if s1 - s0 <= 1e-6:
            continue
        margin = min(1e-4, (s1 - s0) * 0.05)
        grid = np.arange(s0 + margin, s1 - margin + 1e-12, sample_step)
        if len(grid) < 2:
            grid = np.linspace(s0 + margin, s1 - margin, 2)
        for side in ("left", "right"):
            lanes = sorted(section.findall(f"{side}/lane"),
                           key=lambda item: abs(int(item.get("id"))))
            tracks = [[] for _ in lanes]
            for station in grid:
                kref, sharpref = _ref_kappa_at(road, float(station))
                edges = lane_edges_kinematics_at(road, float(station), side)
                if len(edges) != len(lanes) + 1:
                    continue
                for k, (inner, outer) in enumerate(zip(edges, edges[1:])):
                    t, d1, d2 = tuple(
                        (inner[q] + outer[q]) / 2.0 for q in range(3))
                    curvature = _edge_world_curvature(
                        t, d1, d2, kref, sharpref)
                    factor = math.hypot(1.0 - kref * t, d1)
                    tracks[k].append((float(curvature), max(factor, 1e-9)))
            for lane, values in zip(lanes, tracks):
                provenance = _provenance(lane)
                if (lane.get("type") != "driving"
                        or provenance.get("exclusion_code") != "source-extension"
                        or len(values) < 2):
                    continue
                speeds = [item for item in lane.findall("speed")
                          if float(item.get("max", "0")) > 0.0]
                if not speeds:
                    continue
                curvature = np.asarray([item[0] for item in values], float)
                factors = np.asarray([item[1] for item in values], float)
                kappa_max = float(np.max(np.abs(curvature)))
                dl = sample_step * (factors[:-1] + factors[1:]) / 2.0
                sharpness = float(np.max(
                    np.abs(np.diff(curvature)) / np.maximum(dl, 1e-9)))
                safe_ay = (math.sqrt(2.5 / kappa_max)
                           if kappa_max > 1e-12 else float("inf"))
                safe_jerk = ((1.0 / sharpness) ** (1.0 / 3.0)
                             if sharpness > 1e-12 else float("inf"))
                cap_ms = 0.95 * min(safe_ay, safe_jerk)
                current_ms = max(float(item.get("max")) for item in speeds)
                if not math.isfinite(cap_ms) or cap_ms >= current_ms - 1e-6:
                    continue
                cap_ms = max(3.0, cap_ms)
                for item in speeds:
                    item.set("max", f"{min(float(item.get('max')), cap_ms):.12g}")
                provenance.update({
                    "geometry_adjustment": "inferred-design-speed-cap",
                    "source_speed_ms": round(current_ms, 6),
                    "inferred_speed_cap_ms": round(cap_ms, 6),
                    "speed_cap_basis": "ay<=2.5m/s2;jy<=1.0m/s3;margin=0.95",
                })
                user_data = lane.find("userData[@code='mapforge.provenance/v1']")
                if user_data is not None:
                    user_data.set("value", json.dumps(
                        provenance, ensure_ascii=False, separators=(",", ":")))
                capped += 1
                minimum_cap = min(minimum_cap, cap_ms)
                maximum_reduction = max(maximum_reduction, current_ms - cap_ms)
    return {
        "count": capped,
        "minimum_cap_ms": 0.0 if not capped else minimum_cap,
        "maximum_reduction_ms": maximum_reduction,
    }


def _cap_routeable_speeds(road, sections, starts, ends, sample_step=0.25):
    """来源限速与最终车道轨迹不自洽时，按 ``v_supported`` 写显式安全上限。

    该函数不改几何，只处理仍参与路线与来源对拍的 driving lane。门限与 G11-D
    一致（ay<=2.5m/s²、jy<=1.0m/s³），并留 5% 数值裕量；原限速和裁决依据
    写入 provenance，禁止静默降速。
    """
    capped = 0
    minimum_cap = float("inf")
    maximum_reduction = 0.0
    for si, section in enumerate(sections):
        s0, s1 = starts[si], ends[si]
        if s1 - s0 <= 1e-6:
            continue
        margin = min(1e-4, (s1 - s0) * 0.05)
        grid = np.arange(s0 + margin, s1 - margin + 1e-12, sample_step)
        if len(grid) < 2:
            grid = np.linspace(s0 + margin, s1 - margin, 2)
        for side in ("left", "right"):
            lanes = sorted(section.findall(f"{side}/lane"),
                           key=lambda item: abs(int(item.get("id"))))
            tracks = [[] for _ in lanes]
            for station in grid:
                kref, sharpref = _ref_kappa_at(road, float(station))
                edges = lane_edges_kinematics_at(road, float(station), side)
                if len(edges) != len(lanes) + 1:
                    continue
                for k, (inner, outer) in enumerate(zip(edges, edges[1:])):
                    t, d1, d2 = tuple(
                        (a + b) / 2.0 for a, b in zip(inner, outer))
                    curvature = _edge_world_curvature(
                        t, d1, d2, kref, sharpref)
                    factor = math.hypot(1.0 - kref * t, d1)
                    tracks[k].append((float(curvature), max(factor, 1e-9)))
            for lane, values in zip(lanes, tracks):
                provenance = _provenance(lane)
                code = provenance.get("exclusion_code")
                if (lane.get("type") != "driving"
                        or code in {"lane-transition-taper", "physical-edge-fill",
                                    "median-non-driving", "source-support-too-short"}
                        or len(values) < 2):
                    continue
                speeds = [item for item in lane.findall("speed")
                          if float(item.get("max", "0")) > 0.0]
                curvature = np.asarray([item[0] for item in values], float)
                factors = np.asarray([item[1] for item in values], float)
                kappa_max = float(np.max(np.abs(curvature)))
                dl = sample_step * (factors[:-1] + factors[1:]) / 2.0
                sharpness = float(np.max(
                    np.abs(np.diff(curvature)) / np.maximum(dl, 1e-9)))
                safe_ay = (math.sqrt(2.5 / kappa_max)
                           if kappa_max > 1e-12 else float("inf"))
                safe_jerk = ((1.0 / sharpness) ** (1.0 / 3.0)
                             if sharpness > 1e-12 else float("inf"))
                cap_ms = 0.95 * min(safe_ay, safe_jerk)
                # 没写 speed 时，G11/消费端会使用道路角色默认值：普通道路 60km/h、
                # junction connecting road 15km/h。若该默认值超过几何支持速度，
                # 必须落成显式 speed，不能让“缺字段”绕过生成阶段的安全裁决。
                role_default_ms = ((60.0 if road.get("junction") in (None, "-1")
                                    else 15.0) / 3.6)
                current_ms = (max(float(item.get("max")) for item in speeds)
                              if speeds else role_default_ms)
                if not math.isfinite(cap_ms) or cap_ms >= current_ms - 1e-6:
                    continue
                cap_ms = max(3.0, cap_ms)
                if not speeds:
                    speed = etree.Element("speed", sOffset="0", max=f"{cap_ms:.12g}")
                    children = list(lane)
                    insert_at = next((i for i, child in enumerate(children)
                                      if child.tag == "userData"), len(children))
                    lane.insert(insert_at, speed)
                    speeds = [speed]
                for item in speeds:
                    item.set("max", f"{min(float(item.get('max')), cap_ms):.12g}")
                adjustment = {
                    "speed_adjustment": "dynamic-safety-cap",
                    "inferred_speed_cap_ms": round(cap_ms, 6),
                    "speed_cap_basis": "ay<=2.5m/s2;jy<=1.0m/s3;margin=0.95",
                }
                if provenance.get("eligibility") == "comparable":
                    adjustment["source_speed_ms"] = round(current_ms, 6)
                else:
                    adjustment["prior_inferred_speed_ms"] = round(current_ms, 6)
                provenance.update(adjustment)
                user_data = lane.find("userData[@code='mapforge.provenance/v1']")
                if user_data is not None:
                    user_data.set("value", json.dumps(
                        provenance, ensure_ascii=False, separators=(",", ":")))
                capped += 1
                minimum_cap = min(minimum_cap, cap_ms)
                maximum_reduction = max(maximum_reduction, current_ms - cap_ms)
    return {
        "count": capped,
        "minimum_cap_ms": 0.0 if not capped else minimum_cap,
        "maximum_reduction_ms": maximum_reduction,
    }


def cap_xodr_routeable_speeds(path: str | Path, *, sample_step: float = 0.25) -> dict:
    """按最终写出车道轨迹为普通道路补写显式 ``v_supported`` 上限。

    MAP 与 SHP 的来源限速都可能和采集/编制出来的横向形态不自洽。几何保真与
    动力学约束无共同解时，不能为了让门禁通过而挪走来源支持的车道中心，也不能
    在验证器里隐式降低测试速度。本后处理只修改仍标记为 ``comparable`` 的
    driving lane：保留来源速度到 provenance，并把 OpenDRIVE ``speed`` 写成按
    最终世界坐标车道中心反算的安全上限。镜像、铺面和车道生灭 taper 不参与。
    """
    from lxml import etree

    path = Path(path)
    tree = etree.parse(str(path))
    totals = {
        "roads": 0,
        "count": 0,
        "minimum_cap_ms": 0.0,
        "maximum_reduction_ms": 0.0,
    }
    minimum = float("inf")
    for road in tree.getroot().findall("road"):
        if road.get("name") == "junction_paving":
            continue
        sections = road.findall("lanes/laneSection")
        if not sections:
            continue
        starts = [float(section.get("s", "0")) for section in sections]
        ends = starts[1:] + [float(road.get("length", "0"))]
        result = _cap_routeable_speeds(
            road, sections, starts, ends, sample_step=sample_step)
        if result["count"]:
            totals["roads"] += 1
            totals["count"] += int(result["count"])
            minimum = min(minimum, float(result["minimum_cap_ms"]))
            totals["maximum_reduction_ms"] = max(
                totals["maximum_reduction_ms"],
                float(result["maximum_reduction_ms"]))
    if totals["count"]:
        totals["minimum_cap_ms"] = minimum
        tree.write(str(path), encoding="utf-8", xml_declaration=True,
                   pretty_print=True)
    return totals


def _poly(v0, m0, v1, m1, length):
    c = (3.0 * (v1 - v0) - (2.0 * m0 + m1) * length) / length ** 2
    d = (-2.0 * (v1 - v0) + (m0 + m1) * length) / length ** 3
    return float(v0), float(m0), float(c), float(d)


def _write_side(road, sections, starts, ends, side, occurrences, successors, borders):
    # laneLink 相邻端解析焊接，防止浮点/缺省 start target 在语义 section 处撕缝。
    for current, nxt in successors.items():
        shared = borders.get((current, 1))
        if shared is not None:
            borders[(nxt, 0)] = shared
    models = {}
    for key, lane in occurrences.items():
        si, lane_id = key
        length = max(ends[si] - starts[si], 1e-6)
        v0, m0 = borders[(key, 0)]
        v1, m1 = borders[(key, 1)]
        models[key] = _poly(v0, m0, v1, m1, length)

    # 全段检查边界次序；失败时整侧拒绝，绝不落盘交叉 border。
    sign = 1.0 if side == "left" else -1.0
    for si, section in enumerate(sections):
        lanes = sorted(section.findall(f"{side}/lane"),
                       key=lambda item: abs(int(item.get("id"))))
        length = ends[si] - starts[si]
        for u in np.linspace(0.0, length, 41):
            inner = float(lane_edges_kinematics_at(
                road, starts[si] + min(max(float(u), 1e-7), length - 1e-7), side
            )[0][0])
            for lane in lanes:
                a, b, c, d = models[(si, lane.get("id"))]
                outer = a + b * u + c * u ** 2 + d * u ** 3
                if sign * (outer - inner) < -1e-5:
                    return False
                inner = outer
    for key, lane in occurrences.items():
        for old in list(lane.findall("width")) + list(lane.findall("border")):
            lane.remove(old)
        a, b, c, d = models[key]
        border = etree.Element(
            "border", sOffset="0", a=f"{a:.12g}", b=f"{b:.12g}",
            c=f"{c:.12g}", d=f"{d:.12g}")
        lane.insert(1 if lane.find("link") is not None else 0, border)
    return True


def _write_source_shape_side(road, sections, starts, ends, side):
    """按原模型全部多项式断点精确转写 absolute border；不增加 laneSection。"""
    lane_offset_stations = [
        float(item.get("s", "0"))
        for item in road.findall("lanes/laneOffset")
    ]
    pending = []
    sign = 1.0 if side == "left" else -1.0
    correction_max = 0.0
    for si, section in enumerate(sections):
        lanes = sorted(section.findall(f"{side}/lane"),
                       key=lambda item: abs(int(item.get("id"))))
        if not lanes:
            continue
        s0, s1 = starts[si], ends[si]
        knots = {s0, s1}
        knots.update(value for value in lane_offset_stations
                     if s0 + 1e-8 < value < s1 - 1e-8)
        for lane in lanes:
            for item in list(lane.findall("width")) + list(lane.findall("border")):
                value = s0 + float(item.get("sOffset", "0"))
                if s0 + 1e-8 < value < s1 - 1e-8:
                    knots.add(value)
        knots = sorted(knots)
        # 原 width/laneOffset 组合偶尔有毫米级负宽过冲。逐层累加一个常量外移，
        # 让问题边界及其所有外侧边界同步移动，从而不改变任何外侧驾驶车道宽度。
        minimum_width = [float("inf")] * len(lanes)
        for station in np.linspace(s0 + 1e-6, s1 - 1e-6, 401):
            edges = lane_edges_kinematics_at(road, float(station), side)
            if len(edges) != len(lanes) + 1:
                return None
            for k, (inner, outer) in enumerate(zip(edges, edges[1:])):
                minimum_width[k] = min(
                    minimum_width[k], sign * float(outer[0] - inner[0]))
        cumulative = 0.0
        boundary_shift = []
        for value in minimum_width:
            cumulative += max(0.0, -value + 1e-4)
            boundary_shift.append(sign * cumulative)
        if cumulative > 0.25:
            return None
        correction_max = max(correction_max, cumulative)
        endpoint_edges = []
        for k, station in enumerate(knots):
            if k == len(knots) - 1:
                station = station - min(1e-7, (s1 - s0) * 1e-7)
            endpoint_edges.append(lane_edges_kinematics_at(
                road, float(station), side))
        if any(len(edges) != len(lanes) + 1 for edges in endpoint_edges):
            return False
        models = {lane.get("id"): [] for lane in lanes}
        for q, (qa, qb) in enumerate(zip(knots, knots[1:])):
            length = qb - qa
            if length <= 1e-8:
                continue
            left_edges, right_edges = endpoint_edges[q], endpoint_edges[q + 1]
            for k, lane in enumerate(lanes):
                v0 = left_edges[k + 1][0] + boundary_shift[k]
                m0 = left_edges[k + 1][1]
                v1 = right_edges[k + 1][0] + boundary_shift[k]
                m1 = right_edges[k + 1][1]
                models[lane.get("id")].append(
                    (qa - s0, *_poly(v0, m0, v1, m1, length)))
        pending.append((lanes, models))
    for lanes, models in pending:
        for lane in lanes:
            for old in list(lane.findall("width")) + list(lane.findall("border")):
                lane.remove(old)
            insertion = 1 if lane.find("link") is not None else 0
            for offset, a, b, c, d in models[lane.get("id")]:
                border = etree.Element(
                    "border", sOffset=f"{offset:.12g}", a=f"{a:.12g}",
                    b=f"{b:.12g}", c=f"{c:.12g}", d=f"{d:.12g}")
                lane.insert(insertion, border)
                insertion += 1
    return correction_max


def _record_coefficients(item):
    return tuple(float(item.get(name, "0")) for name in ("a", "b", "c", "d"))


def _active_record(records, station, offset_name):
    """返回指定 station 生效的分段多项式记录。"""
    active = records[0]
    for item in records[1:]:
        if float(item.get(offset_name, "0")) <= station + 1e-10:
            active = item
        else:
            break
    return active


def _eval_record(item, station, offset_name):
    ds = station - float(item.get(offset_name, "0"))
    a, b, c, d = _record_coefficients(item)
    return (a + b * ds + c * ds ** 2 + d * ds ** 3,
            b + 2.0 * c * ds + 3.0 * d * ds ** 2)


def _materialize_widths(road, sections, starts, ends):
    """把累计 border 精确转换为相邻边界之差的 lane.width。

    每个内外边界断点的并集构成区间；区间内两条边界均为三次多项式，
    所以有符号差仍是三次多项式。该转换不增加 laneSection 或 planView 原语。
    """
    lane_offsets = sorted(
        road.findall("lanes/laneOffset"),
        key=lambda item: float(item.get("s", "0")),
    )
    converted = 0
    width_records_max = 0
    for si, section in enumerate(sections):
        s0, s1 = starts[si], ends[si]
        length = s1 - s0
        if length <= 1e-9:
            return None
        for side in ("left", "right"):
            sign = 1.0 if side == "left" else -1.0
            lanes = sorted(section.findall(f"{side}/lane"),
                           key=lambda item: abs(int(item.get("id"))))
            if not lanes:
                continue
            borders = {
                lane.get("id"): sorted(
                    lane.findall("border"),
                    key=lambda item: float(item.get("sOffset", "0")),
                )
                for lane in lanes
            }
            if any(not records for records in borders.values()):
                return None
            models = {}
            previous = None
            for lane in lanes:
                outer = borders[lane.get("id")]
                knots = {0.0, length}
                knots.update(float(item.get("sOffset", "0")) for item in outer)
                if previous is None:
                    knots.update(
                        float(item.get("s", "0")) - s0
                        for item in lane_offsets
                        if s0 + 1e-9 < float(item.get("s", "0")) < s1 - 1e-9
                    )
                else:
                    knots.update(float(item.get("sOffset", "0"))
                                 for item in previous)
                knots = sorted(value for value in knots
                               if -1e-9 <= value <= length + 1e-9)
                lane_models = []
                for qa, qb in zip(knots, knots[1:]):
                    qa = max(0.0, qa)
                    qb = min(length, qb)
                    interval = qb - qa
                    if interval <= 1e-9:
                        continue
                    mid = (qa + qb) / 2.0
                    outer_record = _active_record(outer, mid, "sOffset")
                    ov0, om0 = _eval_record(outer_record, qa, "sOffset")
                    ov1, om1 = _eval_record(outer_record, qb, "sOffset")
                    if previous is None:
                        if lane_offsets:
                            inner_record = _active_record(
                                lane_offsets, s0 + mid, "s")
                            iv0, im0 = _eval_record(inner_record, s0 + qa, "s")
                            iv1, im1 = _eval_record(inner_record, s0 + qb, "s")
                        else:
                            iv0 = iv1 = im0 = im1 = 0.0
                    else:
                        inner_record = _active_record(previous, mid, "sOffset")
                        iv0, im0 = _eval_record(inner_record, qa, "sOffset")
                        iv1, im1 = _eval_record(inner_record, qb, "sOffset")
                    w0, dw0 = sign * (ov0 - iv0), sign * (om0 - im0)
                    w1, dw1 = sign * (ov1 - iv1), sign * (om1 - im1)
                    coeff = _poly(w0, dw0, w1, dw1, interval)
                    for u in np.linspace(0.0, interval, 11):
                        width = sum(coeff[q] * u ** q for q in range(4))
                        if width < -1e-5:
                            return None
                    lane_models.append((qa, *coeff))
                models[lane.get("id")] = lane_models
                previous = outer

            # 整侧证明非负后才改 XML，避免半边写入后失败。
            for lane in lanes:
                for old in list(lane.findall("width")) + list(lane.findall("border")):
                    lane.remove(old)
                insertion = 1 if lane.find("link") is not None else 0
                records = models[lane.get("id")]
                for offset, a, b, c, d in records:
                    width = etree.Element(
                        "width", sOffset=f"{offset:.12g}", a=f"{a:.12g}",
                        b=f"{b:.12g}", c=f"{c:.12g}", d=f"{d:.12g}")
                    lane.insert(insertion, width)
                    insertion += 1
                width_records_max = max(width_records_max, len(records))
                converted += 1
    return {"lanes": converted, "width_records_max": width_records_max}


def _lane_by_id(section, side, lane_id):
    return next((lane for lane in section.findall(f"{side}/lane")
                 if lane.get("id") == lane_id), None)


def _width_state(lane, local_s):
    records = sorted(lane.findall("width"),
                     key=lambda item: float(item.get("sOffset", "0")))
    if not records:
        return None
    return _eval_record(_active_record(records, local_s, "sOffset"),
                        local_s, "sOffset")


def _shift_cubic(coeff, offset):
    a, b, c, d = coeff
    return (a + b * offset + c * offset ** 2 + d * offset ** 3,
            b + 2.0 * c * offset + 3.0 * d * offset ** 2,
            c + 3.0 * d * offset,
            d)


def _stretch_short_lane_transitions(road, sections, starts, ends,
                                    short_section_m=5.0,
                                    target_support_m=20.0):
    """把短断面里的外侧车道生灭扩展到相邻断面。

    源 SHP 常用 1--4 m 小 Link 表示车道增加/减少。若直接把零宽到全宽压在
    这段距离内，最终道路外缘会产生数 m/s² 量级的横向二阶变化。这里不改
    planView，只把最外侧车道的 width 三次函数跨相邻 laneSection 重参数化。
    """
    events = []
    for si, section in enumerate(sections):
        span = ends[si] - starts[si]
        if span > short_section_m + 1e-9:
            continue
        for side in ("left", "right"):
            lanes = sorted(section.findall(f"{side}/lane"),
                           key=lambda item: abs(int(item.get("id"))))
            if not lanes:
                continue
            lane = lanes[-1]
            predecessor = lane.find("link/predecessor")
            successor = lane.find("link/successor")
            if si > 0 and predecessor is None:
                events.append(("birth", si, side, lane))
            if si + 1 < len(sections) and successor is None:
                events.append(("death", si, side, lane))
            if (si > 0 and si + 1 < len(sections)
                    and predecessor is not None and successor is not None):
                impulse = False
                margin = min(1e-4, span * 0.05)
                for station in np.linspace(starts[si] + margin,
                                           ends[si] - margin, 11):
                    edges = lane_edges_kinematics_at(
                        road, float(station), side)
                    if not edges:
                        continue
                    t, d1, d2 = edges[-1]
                    kref, sharpref = _ref_kappa_at(road, float(station))
                    kedge = _edge_world_curvature(
                        t, d1, d2, kref, sharpref)
                    if abs(d2) > 0.40 or abs(kedge) > 0.25:
                        impulse = True
                        break
                if impulse:
                    events.append(("through", si, side, lane))

    processed = set()
    changed = 0
    support_max = 0.0
    for kind, si, side, lane in events:
        event_key = (si, side, lane.get("id"))
        if event_key in processed:
            continue
        chain = [(si, lane)]
        support = ends[si] - starts[si]
        if kind == "birth":
            cursor, current = si, lane
            while support < target_support_m and cursor + 1 < len(sections):
                link = current.find("link/successor")
                next_id = link.get("id") if link is not None else current.get("id")
                nxt = _lane_by_id(sections[cursor + 1], side, next_id)
                if nxt is None:
                    break
                cursor += 1
                chain.append((cursor, nxt))
                support += ends[cursor] - starts[cursor]
                current = nxt
            end_state = _width_state(
                chain[-1][1], ends[chain[-1][0]] - starts[chain[-1][0]])
            if end_state is None:
                continue
            coeff = _poly(0.0, 0.0, end_state[0], end_state[1], support)
        elif kind == "death":
            cursor, current = si, lane
            while support < target_support_m and cursor > 0:
                link = current.find("link/predecessor")
                prev_id = link.get("id") if link is not None else current.get("id")
                prev = _lane_by_id(sections[cursor - 1], side, prev_id)
                if prev is None:
                    break
                cursor -= 1
                chain.insert(0, (cursor, prev))
                support += ends[cursor] - starts[cursor]
                current = prev
            start_state = _width_state(chain[0][1], 0.0)
            if start_state is None:
                continue
            coeff = _poly(start_state[0], start_state[1], 0.0, 0.0, support)
        else:
            # 同拓扑短片段：向前、向后各吸收连续 occurrence，消除 1--4m
            # width 脉冲，同时保持支持区两端原始值与斜率。
            first_cursor = last_cursor = si
            first_lane = last_lane = lane
            while first_cursor > 0:
                link = first_lane.find("link/predecessor")
                prev_id = link.get("id") if link is not None else first_lane.get("id")
                prev = _lane_by_id(sections[first_cursor - 1], side, prev_id)
                if prev is None:
                    break
                first_cursor -= 1
                chain.insert(0, (first_cursor, prev))
                support += ends[first_cursor] - starts[first_cursor]
                first_lane = prev
                if support >= target_support_m / 2.0:
                    break
            while last_cursor + 1 < len(sections):
                link = last_lane.find("link/successor")
                next_id = link.get("id") if link is not None else last_lane.get("id")
                nxt = _lane_by_id(sections[last_cursor + 1], side, next_id)
                if nxt is None:
                    break
                last_cursor += 1
                chain.append((last_cursor, nxt))
                support += ends[last_cursor] - starts[last_cursor]
                last_lane = nxt
                if support >= target_support_m:
                    break
            start_state = _width_state(chain[0][1], 0.0)
            end_state = _width_state(
                chain[-1][1], ends[chain[-1][0]] - starts[chain[-1][0]])
            if start_state is None or end_state is None:
                continue
            coeff = _poly(start_state[0], start_state[1],
                          end_state[0], end_state[1], support)

        if any((section_index, side, occurrence.get("id")) in processed
               for section_index, occurrence in chain):
            continue

        offset = 0.0
        for section_index, occurrence in chain:
            span = ends[section_index] - starts[section_index]
            shifted = _shift_cubic(coeff, offset)
            for old in list(occurrence.findall("width")) + list(occurrence.findall("border")):
                occurrence.remove(old)
            insertion = 1 if occurrence.find("link") is not None else 0
            width = etree.Element(
                "width", sOffset="0",
                a=f"{shifted[0]:.12g}", b=f"{shifted[1]:.12g}",
                c=f"{shifted[2]:.12g}", d=f"{shifted[3]:.12g}")
            occurrence.insert(insertion, width)
            provenance = _provenance(occurrence)
            if kind == "through":
                provenance.update({
                    "status": "APPROXIMATED",
                    "geometry_adjustment": "short-width-impulse-smoothed",
                    "width_smoothing_support_m": round(float(support), 6),
                })
            else:
                provenance.update({
                    "eligibility": "excluded",
                    "exclusion_code": "lane-transition-taper",
                    "geometry_adjustment": "short-transition-stretched",
                    "support_kind": "lane-transition-ribbon",
                    "transition_support_m": round(float(support), 6),
                })
            item = occurrence.find("userData[@code='mapforge.provenance/v1']")
            if item is not None:
                item.set("value", json.dumps(
                    provenance, ensure_ascii=False, separators=(",", ":")))
            processed.add((section_index, side, occurrence.get("id")))
            offset += span
        changed += 1
        support_max = max(support_max, support)
    return {"count": changed, "support_max_m": support_max}


def regularize_shp_lane_families(path: str | Path) -> dict:
    """原地把 SHP ordinary roads 转为少控制站、共享边界的 border 表达。"""
    path = Path(path)
    tree = etree.parse(str(path))
    stats = {"border_family_roads": 0, "border_family_sides": 0,
             "border_family_rejected_sides": 0, "border_family_adjust_max_m": 0.0}
    accepted_alphas = []
    alpha_by_side = []
    source_shape_fallback_sides = 0
    source_shape_preserved_sides = 0
    for road in tree.getroot().findall("road"):
        if not _ordinary(road):
            continue
        sections = road.findall("lanes/laneSection")
        if not sections:
            continue
        length = float(road.get("length", "0"))
        starts = [float(item.get("s")) for item in sections]
        ends = starts[1:] + [length]
        raw_quality = edge_shape_quality(road)
        raw_shape_ok = (
            raw_quality["edge_lateral_second_max"] <= 0.40
            and raw_quality["outer_curvature_max"] <= 0.25
            and raw_quality["outer_edge_heading_step_max_deg"] <= 5.0
            and raw_quality["outer_curvature_flips_per_100m_max"] <= 5.0
        )
        changed = False
        for side in ("left", "right"):
            if not any(section.findall(f"{side}/lane") for section in sections):
                continue
            occurrences, successors, target, adjustment = _center_tracks(
                road, sections, starts, ends, side)
            raw_target = _raw_center_targets(
                road, sections, starts, ends, side, occurrences)
            # connecting road 已按 regularize 前的口部车道位姿求解；道路末端
            # 是双侧模型共同的 junction mouth，必须同时钉住中心值与切向。
            mouth_section = len(sections) - 1
            for key in list(target):
                occurrence, endpoint = key
                if occurrence[0] == mouth_section and endpoint == 1:
                    target[key] = (raw_target[key][0], target[key][1])
            accepted_alpha = None
            preserved_adjust = _write_source_shape_side(
                road, sections, starts, ends, side)
            if preserved_adjust is not None:
                accepted_alpha = 0.0
                source_shape_preserved_sides += 1
            # 整侧线搜索：尽量保留平滑目标；任何负宽度/边界交叉都属于硬失败。
            # alpha=0 是原始合法横断面，避免过去“一条异常车道导致整侧不处理”。
            for alpha in (() if accepted_alpha is not None else
                          (1.0, 0.95, 0.90, 0.85, 0.80, 0.75,
                           0.65, 0.50, 0.25, 0.10, 0.0)):
                candidate = _blend_targets(raw_target, target, alpha)
                borders = _derive_borders(
                    road, sections, starts, ends, side, candidate)
                if borders is not None and _write_side(
                        road, sections, starts, ends, side,
                        occurrences, successors, borders):
                    accepted_alpha = alpha
                    break
            if accepted_alpha is None:
                fallback_adjust = _write_source_shape_side(
                    road, sections, starts, ends, side)
                if fallback_adjust is not None:
                    accepted_alpha = 0.0
                    source_shape_fallback_sides += 1
                    stats["border_family_source_shape_adjust_max_m"] = max(
                        stats.get("border_family_source_shape_adjust_max_m", 0.0),
                        fallback_adjust)
                else:
                    stats["border_family_rejected_sides"] += 1
                    continue
            stats["border_family_sides"] += 1
            accepted_alphas.append(accepted_alpha)
            alpha_by_side.append({"road_id": road.get("id"), "side": side,
                                  "alpha": accepted_alpha})
            stats["border_family_narrow_extension_tapers"] = (
                stats.get("border_family_narrow_extension_tapers", 0)
                + _mark_narrow_extension_tapers(
                    road, sections, starts, ends, side))
            stats["border_family_adjust_max_m"] = max(
                stats["border_family_adjust_max_m"], adjustment * accepted_alpha)
            changed = True
        stats["border_family_roads"] += int(changed)
        if changed:
            materialized = _materialize_widths(road, sections, starts, ends)
            if materialized is None:
                raise ValueError(
                    f"road {road.get('id')}: shared borders cannot be materialized "
                    "as non-negative lane widths")
            stats["border_family_materialized_width_lanes"] = (
                stats.get("border_family_materialized_width_lanes", 0)
                + materialized["lanes"])
            stats["border_family_width_records_max"] = max(
                stats.get("border_family_width_records_max", 0),
                materialized["width_records_max"])
            if not raw_shape_ok:
                stretched = _stretch_short_lane_transitions(
                    road, sections, starts, ends)
                stats["border_family_stretched_transitions"] = (
                    stats.get("border_family_stretched_transitions", 0)
                    + stretched["count"])
                stats["border_family_transition_support_max_m"] = max(
                    stats.get("border_family_transition_support_max_m", 0.0),
                    stretched["support_max_m"])
            # 几何修形不能改写来源限速或补造缺失限速。旧 cap helpers 仅保留
            # 供历史研究脚本复核，不再接入转换主线；动态失败由质量报告阻断。
    stats["speed_limit_policy"] = "preserve-source-no-geometry-derived-caps"
    stats["border_family_alpha_min"] = min(accepted_alphas, default=0.0)
    stats["border_family_alpha_mean"] = (
        float(np.mean(accepted_alphas)) if accepted_alphas else 0.0)
    stats["border_family_source_shape_sides"] = sum(
        alpha == 0.0 for alpha in accepted_alphas)
    stats["border_family_source_shape_fallback_sides"] = \
        source_shape_fallback_sides
    stats["border_family_source_shape_preserved_sides"] = \
        source_shape_preserved_sides
    stats["border_family_alpha_by_side"] = alpha_by_side
    tree.write(str(path), encoding="utf-8", xml_declaration=True, pretty_print=True)
    return stats
