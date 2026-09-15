# -*- coding: utf-8 -*-
"""MAP(MapNode) → OpenDRIVE（方向 6："MAP 还原"），junction 完整重建。

道路模型 = OpenDRIVE 规范做法：**一条街一条 leg road，参考线沿进口 Link 中心线，
双侧展开**——进口车道挂右侧（-1..-n），通往该 Link 上游节点的出口挂左侧（+1..+n，
行车方向沿 s 递减），中线双黄。写出走自研规范级 writer（弃 scenariogeneration）。

出口两级来源（有真实数据用真实，缺才脑补）：
1. **real**：多节点 MAP 帧里，邻居节点中"上游=本节点"的 inLink 就是本路口的真实出口
   （车道点列实测横距放置到本 leg 参考线左侧，EXACT/TRANSFORMED 语义）；
2. **mirror**：单节点帧无出口数据时，进口车道组左侧对称镜像（对向车行道，INFERRED）。

平滑（全接缝 G2）：
- 参考线升级档拟合（fit_polyline_auto，偏差>0.3m 自动换细档）；
- 连接路起止位姿取**路模型端部位姿**（拟合参考线端点 + 车道横向偏移）；
- SolveG2 传两端**车道级曲率**（κ/(1−tκ) 偏移修正；左侧行车逆 s，曲率取负）。
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from mapforge.adapters.opendrive import writer as W
from mapforge.adapters.opendrive.writer import std_mark
from mapforge.adapters.v2xmap.xml_reader import MapNode
from mapforge.ops.refline_fit import (ReflineFitError, eval_planview,
                                      fit_leg_refline, planview_prims,
                                      seg_kappa, solve_g2_balanced)
from mapforge.validate.g8_model import make_manifest, source_lane

R_EARTH = 6378137.0


def _project(pts_deg, lat0, lon0):
    pts = np.asarray(pts_deg, dtype=float)
    x = np.radians(pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def _georef(lat0, lon0):
    """本地平面投影的 PROJ 管线（球面 eqc，与 _project 数学一致）——CRS 硬约束 3。"""
    return (f"+proj=eqc +lat_ts={lat0:.8f} +lat_0={lat0:.8f} +lon_0={lon0:.8f} "
            f"+R={R_EARTH:.0f} +units=m +no_defs")


def _lane_speed_kmh(ln) -> float | None:
    """MAP 车道限速（RegulatorySpeedLimit，0.02 m/s 步长）→ km/h。"""
    for tname, raw in (ln.speed_limits or []):
        if tname == "vehicleMaxSpeed" and raw:
            return raw * 0.02 * 3.6
    return None


def _legacy_source_lane_key(link, lane) -> str:
    reg, nid = link.upstream or (None, None)
    return f"map:{reg}:{nid}:{link.name}:lane:{lane.lane_id}"


def _source_lane_key(owner: MapNode, link, lane) -> str:
    """包含 Link 所属节点的稳定来源键；冲突由 manifest collector 阻断。"""
    reg, nid = link.upstream or (None, None)
    return (f"map:{owner.region}:{owner.node_id}:from:{reg}:{nid}:"
            f"{link.name}:lane:{lane.lane_id}")


def _reference_support(link_points, lane_point_sets):
    """补足 Link 首点以前的参考支撑，绝不裁剪 Lane 原始观测。

    Link 与 Lane 点列的覆盖长度可以不同。选取有共同断面、方向一致的最长
    车道前缀，按 Link 首断面实测横距平移；已有 Link 点（尤其停止线端）不变。
    前缀只是 INFERRED 参考支撑，不是测得的道路中心线。整个支撑交给长原语
    拟合器，不能把这些点逐段直出。路口端错位另报冲突，不能移动停止线迁就。
    """
    link = np.asarray(link_points, float)
    if len(link) < 2 or not np.isfinite(link).all():
        raise ValueError("reference support requires a finite Link point list")
    nonzero = np.flatnonzero(np.linalg.norm(link[1:] - link[0], axis=1) > 1e-6)
    if not len(nonzero):
        raise ValueError("reference support requires a nonzero Link tangent")
    tangent = link[int(nonzero[0]) + 1] - link[0]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    candidates, rejected = [], []
    for lane_id, points in lane_point_sets:
        pts = np.asarray(points, float)
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
            continue
        if not np.isfinite(pts).all():
            rejected.append({"lane_id": lane_id, "reason": "nonfinite-lane-points"})
            continue
        u = (pts - link[0]) @ tangent
        if u.min() >= -1.5:
            continue
        # A backwards or disconnected lane is not authority to lengthen this leg.
        crossings = np.flatnonzero((u[:-1] < 0) & (u[1:] >= 0))
        if u[0] >= -1.5 or not len(crossings):
            rejected.append({"lane_id": lane_id, "reason": "no-forward-common-section"})
            continue
        j = int(crossings[0])
        if np.any(np.diff(u[:j + 2]) < -1.5):
            rejected.append({"lane_id": lane_id, "reason": "backtracking-prefix"})
            continue
        alpha = -u[j] / (u[j + 1] - u[j])
        anchor = pts[j] + alpha * (pts[j + 1] - pts[j])
        lateral = float((anchor - link[0]) @ normal)
        if abs(lateral) > 30.0:
            rejected.append({"lane_id": lane_id, "reason": "disconnected-common-section"})
            continue
        prefix = pts[:j + 1] - lateral * normal
        prefix = prefix[u[:j + 1] < -0.5]
        if len(prefix):
            candidates.append((float(-u[0]), lane_id, prefix, lateral))
    meta = {"status": "EXACT", "prefix_point_count": 0, "rejected": rejected}
    if not candidates:
        return link.copy(), meta
    extent, lane_id, prefix, lateral = max(candidates, key=lambda item: item[0])
    meta.update({"status": "INFERRED", "method": "lane-prefix-common-section-offset",
                 "source_lane_id": lane_id, "prefix_point_count": len(prefix),
                 "longitudinal_extension_m": extent, "common_section_offset_m": lateral,
                 "link_points_unchanged": True,
                 "reference_support_xy": prefix.tolist()})
    return np.vstack([prefix, link]), meta


def _lane_kappa(k_ref: float, t: float) -> float:
    """参考线曲率 → 横向偏移 t 处车道中心曲率：κ/(1−tκ)。"""
    den = 1.0 - t * k_ref
    return k_ref / den if abs(den) > 1e-6 else k_ref


def _shift(pose, t):
    x, y, h = pose
    return (x - t * math.sin(h), y + t * math.cos(h), h)


def _densify_polyline(pts_xy, step=0.5):
    """沿 MAP 原始点列分段线性加密；不平滑、不外推。"""
    pts = np.asarray(pts_xy, float)
    if len(pts) < 2:
        return pts.copy()
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    ss = np.concatenate([[0.0], np.cumsum(seg)])
    if ss[-1] <= 1e-9:
        return pts[:1].copy()
    q = np.arange(0.0, ss[-1], max(float(step), 1e-3))
    q = np.append(q, ss[-1]) if not len(q) or ss[-1] - q[-1] > 1e-9 else q
    return np.column_stack([np.interp(q, ss, pts[:, 0]),
                            np.interp(q, ss, pts[:, 1])])


def _adaptive_section_stations(length, knot_sets=(), *, base_step=20.0,
                               min_gap=5.0):
    """生成横断面控制站；源采样位置优先，且不产生密集 laneSection。

    固定 10m 网格会把稀疏 MAP 点列中的展宽/横移控制点错开数米。反过来直接把
    0.5m 加密点全部写成 laneSection 又会制造横断面碎片。本函数只消费原始点列
    投影得到的 ``s``，先保留间距足够的源控制点，再只在大空档中补均匀站点。
    这不会改变 planView 的 line/arc/spiral 段数。
    """
    L = max(0.0, float(length))
    if L <= 1e-9:
        return [0.0, L]
    candidates = [0.0, L]
    for values in knot_sets:
        if values is None:
            continue
        candidates.extend(float(x) for x in np.asarray(values, float).ravel()
                          if math.isfinite(float(x)) and 0.0 < float(x) < L)
    candidates = sorted(set(round(min(max(x, 0.0), L), 6) for x in candidates))

    essential = [0.0]
    for value in candidates[1:-1]:
        if value - essential[-1] >= min_gap and L - value >= min_gap:
            essential.append(value)
    if L - essential[-1] < min_gap and len(essential) > 1:
        essential.pop()
    essential.append(L)

    out = [essential[0]]
    for a, b in zip(essential, essential[1:]):
        span = b - a
        pieces = max(1, int(math.ceil(span / max(base_step, min_gap))))
        while pieces > 1 and span / pieces < min_gap:
            pieces -= 1
        out.extend(a + span * j / pieces for j in range(1, pieces + 1))
    return [float(x) for x in out]


def _lane_profile(ref, tang, lp, max_snap=30.0, return_points=False,
                  densify_profile=False):
    """车道点列 → 沿参考线的 (s, d) 实测轮廓。

    最近参考点只给出初值；再叠加切向残量得到真实纵向位置。这样参考线端点外仍在
    ``max_snap`` 半径内的点不会被压到同一个 s 并误计入可比较来源域。
    """
    lp = np.asarray(lp, float)
    if float(np.dot(lp[-1] - lp[0], ref[-1] - ref[0])) < 0:
        lp = lp[::-1]
    ref_s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))])

    def _project_profile(points):
        dist = np.linalg.norm(ref[None, :, :] - points[:, None, :], axis=2)
        idx = np.argmin(dist, axis=1)
        longitudinal = np.sum((points - ref[idx]) * tang[idx], axis=1)
        keep = dist[np.arange(len(points)), idx] < max_snap
        keep &= ~((idx == 0) & (longitudinal < -1.5))
        keep &= ~((idx == len(ref) - 1) & (longitudinal > 1.5))
        if keep.sum() < 2:
            return None
        points2, idx2 = points[keep], idx[keep]
        # 最近采样点只用于锁定投影邻域；真实纵向坐标还要加回切向残量。
        # 旧实现遗漏了这一步，使稀疏 MAP 点在 0.5m 参考线采样格上量化，随后
        # laneOffset/width 控制站会被放到错误的 s，口部展宽与局部换道形状随之错位。
        s2 = np.clip(ref_s[idx2] + longitudinal[keep], 0.0, ref_s[-1])
        d2 = (tang[idx2, 0] * (points2[:, 1] - ref[idx2, 1])
              - tang[idx2, 1] * (points2[:, 0] - ref[idx2, 0]))
        order2 = np.argsort(s2)
        return (s2[order2], d2[order2]), points2[order2]

    raw = _project_profile(lp)
    if raw is None:
        return None
    prof, support_points = raw
    if densify_profile and len(support_points) <= 12:
        # 这里只返回拟合支持点，不是来源 manifest。manifest 必须从完整原件
        # 单独建立；不能用此处的投影裁剪/重排结果缩小验收范围。
        dense = _project_profile(_densify_polyline(support_points, 0.5))
        if dense is not None:
            prof = dense[0]
    return (prof, support_points) if return_points else prof


def _prof_eval(prof, u, win=15.0):
    """轮廓在 s=u 处的 (值, 斜率)：±win 窗局部线性回归；窗内点不足取最近点保持。"""
    ps, pd = prof
    ups = np.unique(ps)
    dense_linearized = (len(ups) >= 2 and
                        float(np.median(np.diff(ups))) <= 0.75)
    if ((2 <= len(ups) <= 12 or dense_linearized)
            and float(ups[-1] - ups[0]) > 1.0):
        upd = np.asarray([np.median(pd[np.isclose(ps, value)]) for value in ups])
        if u <= ups[0]:
            return float(upd[0]), 0.0
        if u >= ups[-1]:
            return float(upd[-1]), 0.0
        i = int(np.searchsorted(ups, u))
        du = max(float(ups[i] - ups[i - 1]), 1e-9)
        slope = float((upd[i] - upd[i - 1]) / du)
        value = float(upd[i - 1] + (u - ups[i - 1]) * slope)
        return value, float(min(max(slope, -0.25), 0.25))
    m = np.abs(ps - u) <= win
    if m.sum() >= 3 and float(ps[m].max() - ps[m].min()) > 1.0:
        b, a = np.polyfit(ps[m] - u, pd[m], 1)
        return float(a), float(min(max(b, -0.25), 0.25))
    k = np.argsort(np.abs(ps - u))[:3]
    return float(np.median(pd[k])), 0.0


def _dv(lane, u):
    """车道在 s=u 的 (中心横距, 斜率)：有轮廓走回归，无点列取常量（斜率 0）。"""
    if lane.get("prof") is None:
        return lane.get("const", 0.0), 0.0
    return _prof_eval(lane["prof"], u)


def _bounds(lanes, u, sign, base=None):
    """一侧在站点 u 的边界 (b, mb)：**仅该站点有点列覆盖的车道**参与堆叠，覆盖外写 0 宽
    （相邻站点间自然收放 = 口部展宽/远端加车道的数据本义；强行外推会把只存在于远端的
    车道摆进对向幅）。实测中心 → 相邻中点为界，外缘 ±半宽。
    sign=-1 右侧（b 递减）/ +1 左侧（b[0]=内缘）；base=(v,m) 时 rel 车道贴进口幅左缘。
    轮廓交叉守卫：活动车道宽 <0.4m → 本站点改按名义宽从内缘堆叠。"""
    act = [k for k, x in enumerate(lanes)
           if x.get("cov", (-1e18, 1e18))[0] <= u <= x.get("cov", (-1e18, 1e18))[1]]
    if not act:                                          # 全无覆盖：退回全员（保持值）
        act = list(range(len(lanes)))
    vs, ms, ws = [], [], []
    for k in act:
        x = lanes[k]
        if x.get("rel") is not None and base is not None:
            v, m = base[0] + x["rel"], base[1]
        else:
            v, m = _dv(x, u)
        vs.append(v)
        ms.append(m)
        ws.append(x["w"])
    # OpenDRIVE 车道中心由相邻边界的均值决定。旧算法把相邻源中心的中点直接
    # 当边界；当实际车道间距不等或发生展宽时，写出的中心会变成
    # ``(3*v_i+v_{i+1})/4``，可持续偏离源中心近 1m。这里直接求
    # ``boundary0 + widths``：中心残差为主目标，MAP nominal width 只作弱先验，
    # 并硬约束所有活动车道宽度 >=0.4m。这样仍是标准 reference-line +
    # laneOffset/width 表达，不是把单边道路贴成多条独立 road。
    n = len(act)
    try:
        from scipy.optimize import lsq_linear

        # z=[b0,w0,...]；center_i=b0+sign*(sum(w[:i])+0.5*w_i)
        A_center = np.zeros((n, n + 1), float)
        A_center[:, 0] = 1.0
        for i in range(n):
            if i:
                A_center[i, 1:i + 1] = sign
            A_center[i, i + 1] = 0.5 * sign
        center_weight = 100.0
        width_weight = 1.0
        A = np.vstack([center_weight * A_center,
                       width_weight * np.column_stack([
                           np.zeros(n), np.eye(n)])])
        y = np.concatenate([center_weight * np.asarray(vs, float),
                            width_weight * np.asarray(ws, float)])
        upper_width = np.maximum(15.0, 3.0 * np.asarray(ws, float))
        sol = lsq_linear(A, y,
                         bounds=(np.concatenate([[-np.inf], np.full(n, 0.4)]),
                                 np.concatenate([[np.inf], upper_width])),
                         lsmr_tol="auto", max_iter=100)
        b0, widths = float(sol.x[0]), np.asarray(sol.x[1:], float)
        ba = [b0]
        for width in widths:
            ba.append(ba[-1] + sign * float(width))

        # 同一线性模型求边界导数；宽度导数以 0 为弱先验。后续全 road C1
        # 正则化会从这些站点值重新求唯一导数，此处主要保证镜像前语义一致。
        Ad = np.vstack([center_weight * A_center,
                        width_weight * np.column_stack([
                            np.zeros(n), np.eye(n)])])
        yd = np.concatenate([center_weight * np.asarray(ms, float),
                             np.zeros(n)])
        dz, *_ = np.linalg.lstsq(Ad, yd, rcond=None)
        ma = [float(dz[0])]
        for slope in dz[1:]:
            ma.append(ma[-1] + sign * float(slope))
    except Exception:
        ba = [vs[0] - sign * ws[0] / 2]
        ma = [ms[0]]
        for i in range(1, n):
            ba.append((vs[i - 1] + vs[i]) / 2)
            ma.append((ms[i - 1] + ms[i]) / 2)
        ba.append(vs[-1] + sign * ws[-1] / 2)
        ma.append(ms[-1])
    act_set, b, mb, i = set(act), [ba[0]], [ma[0]], 0
    for k in range(len(lanes)):
        if k in act_set:                                 # 活动车道：取本段边界
            i += 1
            b.append(ba[i])
            mb.append(ma[i])
        else:                                            # 未覆盖：零宽（与前一边界重合）
            b.append(b[-1])
            mb.append(mb[-1])
    return b, mb


def _mirror_bounds(right_bounds, lane_count, fallback_width=3.5):
    """把同一 leg 的右侧进口断面严格左右镜像为左侧出口断面。

    镜像轴取右侧车道组内边界 ``b[0]``；不仅镜像边界位置，也镜像边界
    对 s 的导数，因此进口侧的逐段变宽、收口和车道生灭会原样反射到出口侧。
    若 MAP 的目标车道号要求比可镜像的进口车道更多，只有超出的外侧车道才
    使用名义宽度兜底，已有车道绝不退化为“中位宽度整齐堆叠”。
    """
    b, mb = right_bounds
    if not b or not mb:
        return [0.0] * (lane_count + 1), [0.0] * (lane_count + 1)
    axis, axis_m = float(b[0]), float(mb[0])
    source_count = min(len(b), len(mb)) - 1
    out_b, out_m = [axis], [axis_m]
    for k in range(lane_count):
        if k < source_count:
            out_b.append(2.0 * axis - float(b[k + 1]))
            out_m.append(2.0 * axis_m - float(mb[k + 1]))
        else:
            out_b.append(out_b[-1] + float(fallback_width))
            out_m.append(out_m[-1])
    return out_b, out_m


def _c1_boundary_tracks(stations, bounds, sign, *, smoothing_lam=0.006):
    """把整条 leg 的横断面站点转换为一组共享 C1 边界导数。

    旧实现对每个 laneSection 分别调用 ``fc_clamp``。同一个站点作为前段终点和
    后段起点时会被两套区间独立限幅，位置虽然相等，斜率却可能相差十余度，正是
    查看器里“接得上但不流畅”的来源。这里按道路全长处理：内缘与每条车道宽度
    分别用形态保持 PCHIP 求唯一站点导数，再累加回绝对边界。这样既保持所有原始
    站点位置，也保证 laneOffset、每条 width 及最终世界边缘在 section 间 C1。
    """
    from scipy.interpolate import CubicSpline, make_smoothing_spline

    x = np.asarray(stations, float)
    values = np.asarray([b[0] for b in bounds], float)
    if values.ndim != 2 or len(values) != len(x) or len(x) < 2:
        return bounds
    if len(x) == 2:
        slopes = np.repeat(((values[1] - values[0]) /
                            max(float(x[1] - x[0]), 1e-9))[None, :], 2, axis=0)
        return [(values[j].tolist(), slopes[j].tolist()) for j in range(len(x))]

    # 不直接让 PCHIP 穿过每个 20m 站点。MAP 点列常把缺测车道表现成
    # 0→3.5m 的一步跳变；逐点插值会制造 0.03–0.10/m 的车道中心曲率。
    # 对“内缘 + 各车道宽度”分别做全路惩罚样条，随后用 natural cubic 的
    # 唯一导数写回 Hermite。归一化弧长使 lam 不随道路长度改变；端点用线性
    # 校正精确保持，避免路口接缝漂移。宽度始终非负，且不独立平滑绝对边界，
    # 因而不会出现边界交叉。
    u = (x - x[0]) / max(float(x[-1] - x[0]), 1e-9)

    def regularized(y, *, nonnegative=False):
        y = np.asarray(y, float)
        spread = float(np.percentile(y, 90) - np.percentile(y, 10))
        if spread < 0.08:
            out = np.full_like(y, float(np.median(y)))
        elif len(u) < 5:
            # scipy.make_smoothing_spline 至少需要 5 点。短道路只有 3–4 个
            # laneSection 时直接用全路二次最小二乘，仍是一条低频轨迹，不能
            # 退回逐段 PCHIP。
            degree = min(2, len(u) - 1)
            out = np.polyval(np.polyfit(u, y, degree), u)
            out += (y[0] - out[0]) * (1.0 - u) + (y[-1] - out[-1]) * u
        else:
            # 默认 lam=0.006 对应约 45–60m 的车道收放尺度；真实值是否仍被保留
            # 由随后的 G8 双向车道保真门禁决定。
            spl = make_smoothing_spline(u, y, lam=float(smoothing_lam))
            out = np.asarray(spl(u), float)
            out += (y[0] - out[0]) * (1.0 - u) + (y[-1] - out[-1]) * u
        if nonnegative:
            out = np.maximum(out, 0.0)
        return out

    inner = regularized(values[:, 0])
    widths = sign * np.diff(values, axis=1)
    widths = np.column_stack([
        regularized(widths[:, k], nonnegative=True)
        for k in range(widths.shape[1])
    ]) if widths.shape[1] else np.zeros((len(x), 0), float)
    smooth = np.zeros_like(values)
    smooth[:, 0] = inner
    for k in range(widths.shape[1]):
        smooth[:, k + 1] = smooth[:, k] + sign * widths[:, k]

    slopes = np.zeros_like(smooth)
    for k in range(smooth.shape[1]):
        slopes[:, k] = CubicSpline(x, smooth[:, k], bc_type="natural").derivative()(x)
    return [(smooth[j].tolist(), slopes[j].tolist()) for j in range(len(x))]


def _boundary_track_eval(stations, bounds, column, queries):
    """按最终写出的分段 Hermite 语义求边界横距。"""
    x = np.asarray(stations, float)
    q = np.clip(np.asarray(queries, float), x[0], x[-1])
    out = np.empty_like(q)
    for n, u in enumerate(q):
        j = min(max(int(np.searchsorted(x, u, side="right") - 1), 0), len(x) - 2)
        x0, x1 = float(x[j]), float(x[j + 1])
        L = max(x1 - x0, 1e-9)
        z = (float(u) - x0) / L
        y0, m0 = float(bounds[j][0][column]), float(bounds[j][1][column])
        y1, m1 = float(bounds[j + 1][0][column]), float(bounds[j + 1][1][column])
        out[n] = ((2*z**3 - 3*z**2 + 1)*y0
                  + (z**3 - 2*z**2 + z)*L*m0
                  + (-2*z**3 + 3*z**2)*y1
                  + (z**3 - z**2)*L*m1)
    return out


def _directed_point_to_polyline(points, polyline):
    """每个点到连续折线的精确最近距离，而不是到折线采样点的距离。

    横断面平滑强度选择曾用 KDTree 比较 1m 与 0.5m 两组离散点。即使两条
    曲线完全重合，错开的采样相位也会制造约 0.5m 的误差地板，迫使选择器
    退回近似插值（lambda=1e-6），进而把 MAP 点列抖动写进道路边缘。
    """
    pts = np.asarray(points, float)
    line = np.asarray(polyline, float)
    if not len(pts):
        return np.zeros(0, float)
    if len(line) == 0:
        return np.full(len(pts), float("inf"), float)
    if len(line) == 1:
        return np.linalg.norm(pts - line[0], axis=1)
    a, ab = line[:-1], np.diff(line, axis=0)
    den = np.sum(ab * ab, axis=1)
    out = np.empty(len(pts), float)
    # 限制临时矩阵大小；真实 MAP 单 lane 通常仅几十到数百采样点。
    for i in range(0, len(pts), 256):
        block = pts[i:i + 256]
        ap = block[:, None, :] - a[None, :, :]
        t = np.divide(np.sum(ap * ab[None, :, :], axis=2),
                      den[None, :], out=np.zeros((len(block), len(ab))),
                      where=den[None, :] > 1e-12)
        t = np.clip(t, 0.0, 1.0)
        nearest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
        out[i:i + len(block)] = np.sqrt(
            np.min(np.sum((block[:, None, :] - nearest) ** 2, axis=2), axis=1))
    return out


def _boundary_source_metrics(stations, bounds, lanes, sign, *, base_bounds=None,
                             ref=None, tang=None):
    """在局部 ``(s,t)`` 平面复现 G8 的双向最近曲线距离。"""
    all_err = []
    lane_rows = []
    for k, lane in enumerate(lanes):
        prof = lane.get("prof")
        if prof is None:
            continue
        ps, pd = (np.asarray(prof[0], float), np.asarray(prof[1], float))
        mask = ((ps >= float(stations[0]) - 1e-6)
                & (ps <= float(stations[-1]) + 1e-6))
        if np.count_nonzero(mask) < 2:
            continue
        ps, pd = ps[mask], pd[mask]
        q = np.arange(float(ps.min()), float(ps.max()), 0.5)
        if not len(q) or float(ps.max()) - float(q[-1]) > 1e-9:
            q = np.append(q, float(ps.max()))
        a = _boundary_track_eval(stations, bounds, k, q)
        b = _boundary_track_eval(stations, bounds, k + 1, q)
        pred = (a + b) / 2.0
        if sign > 0 and base_bounds is not None:
            own0 = _boundary_track_eval(stations, bounds, 0, q)
            base0 = _boundary_track_eval(stations, base_bounds, 0, q)
            pred += np.maximum(base0 - own0, 0.0)
        source_points = lane.get("source_points")
        if (source_points is not None and ref is not None and tang is not None):
            source_curve = _densify_polyline(np.asarray(source_points, float), 0.5)
            ref_arr, tang_arr = np.asarray(ref, float), np.asarray(tang, float)
            ref_s = np.concatenate([[0.0], np.cumsum(
                np.linalg.norm(np.diff(ref_arr, axis=0), axis=1))])
            rx = np.interp(q, ref_s, ref_arr[:, 0])
            ry = np.interp(q, ref_s, ref_arr[:, 1])
            tx = np.interp(q, ref_s, tang_arr[:, 0])
            ty = np.interp(q, ref_s, tang_arr[:, 1])
            nn = np.hypot(tx, ty)
            tx, ty = tx / np.maximum(nn, 1e-12), ty / np.maximum(nn, 1e-12)
            target_curve = np.column_stack([rx - pred * ty, ry + pred * tx])
        else:
            source_curve = np.column_stack([ps, pd])
            target_curve = np.column_stack([q, pred])
        # 比较连续折线，不让不同采样相位制造固定的伪误差。
        s2t = _directed_point_to_polyline(source_curve, target_curve)
        t2s = _directed_point_to_polyline(target_curve, source_curve)
        err = np.concatenate([s2t, t2s])
        all_err.extend(err.tolist())
        endpoint = max(float(np.linalg.norm(source_curve[0] - target_curve[0])),
                       float(np.linalg.norm(source_curve[-1] - target_curve[-1])))
        lane_rows.append({
            "lane": int(lane.get("lid", getattr(lane.get("ln"), "lane_id", k + 1))),
            "median_m": max(float(np.median(s2t)), float(np.median(t2s))),
            "p95_m": max(float(np.percentile(s2t, 95)),
                           float(np.percentile(t2s, 95))),
            "max_m": max(float(np.max(s2t)), float(np.max(t2s))),
            "coverage_min": min(float(np.mean(s2t <= 1.5)),
                                  float(np.mean(t2s <= 1.5))),
            "endpoint_m": endpoint,
        })
    if not all_err:
        return {"median_m": 0.0, "p95_m": 0.0, "max_m": 0.0,
                "lanes": lane_rows}
    arr = np.asarray(all_err, float)
    return {"median_m": float(np.median(arr)),
            "p95_m": float(np.percentile(arr, 95)),
            "max_m": float(np.max(arr)), "lanes": lane_rows}


_BOUNDARY_LAMS = (0.250, 0.100, 0.060, 0.040, 0.025, 0.015, 0.010,
                  0.006, 0.004, 0.002, 0.001, 0.0003, 1e-6)


def _boundary_metrics_ok(metrics):
    return all(
        # MAP Lane.points 是车道中心实测/编制点，不是仅供视觉平滑的提示线。
        # 旧阈值允许局部偏离 3m，会让“全局 P95 合格”掩盖一整段车道错位。
        # 这里约束候选选择；找不到可行正则强度时仍返回最保真的候选，随后由
        # 正式 G8 明确失败，不静默放宽或增加 planView 碎片。
        row["median_m"] <= 0.20 + 1e-9
        and row["p95_m"] <= 0.45 + 1e-9
        and row["max_m"] <= 0.75 + 1e-9
        and row["coverage_min"] >= 0.98 - 1e-9
        and row["endpoint_m"] <= 0.50 + 1e-9
        for row in metrics["lanes"])


def _regularize_measured_boundaries(stations, bounds, lanes, sign, *,
                                     base_bounds=None, ref=None, tang=None):
    """选择满足来源保真的最大横断面平滑强度。"""
    fallback = None
    trials = []
    for lam in _BOUNDARY_LAMS:
        fitted = _c1_boundary_tracks(
            stations, bounds, sign, smoothing_lam=lam)
        metrics = _boundary_source_metrics(
            stations, fitted, lanes, sign, base_bounds=base_bounds,
            ref=ref, tang=tang)
        fallback = (fitted, lam, metrics)
        trials.append({
            "lambda": lam,
            "median_max_m": max((r["median_m"] for r in metrics["lanes"]), default=0.0),
            "p95_max_m": max((r["p95_m"] for r in metrics["lanes"]), default=0.0),
            "max_m": max((r["max_m"] for r in metrics["lanes"]), default=0.0),
            "coverage_min": min((r["coverage_min"] for r in metrics["lanes"]), default=1.0),
            "endpoint_max_m": max((r["endpoint_m"] for r in metrics["lanes"]), default=0.0),
        })
        if _boundary_metrics_ok(metrics):
            metrics["selection_trials"] = trials
            return fitted, lam, metrics
    fallback[2]["selection_trials"] = trials
    return fallback


def _regularize_measured_pair(stations, right_bounds, right_lanes,
                              left_bounds, left_lanes, *, ref, tang):
    """联合选择真实双向道路两侧的正则化强度。

    左侧 written 位置依赖右侧 laneOffset/median 基准，独立贪心会出现“一侧很平、
    另一侧被迫追点”的病态组合。这里先枚举右侧可行候选，再在同一基准下评估
    左侧；按 ``min(lambda_r, lambda_l)`` 优先，防止把复杂度转嫁给任一侧。
    """
    right_candidates = []
    left_shapes = [(lam, _c1_boundary_tracks(
        stations, left_bounds, +1, smoothing_lam=lam))
        for lam in _BOUNDARY_LAMS]
    for rlam in _BOUNDARY_LAMS:
        rb = _c1_boundary_tracks(stations, right_bounds, -1,
                                 smoothing_lam=rlam)
        rm = _boundary_source_metrics(
            stations, rb, right_lanes, -1, ref=ref, tang=tang)
        if not _boundary_metrics_ok(rm):
            continue
        for llam, lb in left_shapes:
            lm = _boundary_source_metrics(
                stations, lb, left_lanes, +1, base_bounds=rb,
                ref=ref, tang=tang)
            if _boundary_metrics_ok(lm):
                right_candidates.append((
                    min(rlam, llam), math.sqrt(rlam * llam), rlam + llam,
                    rb, rlam, rm, lb, llam, lm))
    if right_candidates:
        best = max(right_candidates, key=lambda x: x[:3])
        return best[3], best[4], best[5], best[6], best[7], best[8]
    # 数据本身无联合可行解时仍返回两侧各自最保真的候选，让正式 G8 明确 FAIL，
    # 绝不静默丢侧或改成镜像。
    rb, rlam, rm = _regularize_measured_boundaries(
        stations, right_bounds, right_lanes, -1, ref=ref, tang=tang)
    lb, llam, lm = _regularize_measured_boundaries(
        stations, left_bounds, left_lanes, +1, base_bounds=rb,
        ref=ref, tang=tang)
    return rb, rlam, rm, lb, llam, lm


def _infer_mirror_speeds(lanes, stations, bounds, pv, *, ceiling_kmh=60.0):
    """按镜像后实际 XY 轨迹反求各车道安全限速。

    MAP 缺出口时，几何和限速都属于 INFERRED。短路段中移动的镜像轴可能使
    外侧轨迹曲率高于进口；照抄 60km/h 会把一个几何上平顺、但速度不匹配的
    结果交给自动驾驶。这里按 ay≤2.5m/s²、jerk≤1.0m/s³ 计算，并留 10% 裕量。
    """
    L = float(stations[-1])
    q = np.arange(0.0, L, 0.20)
    if not len(q) or L - q[-1] > 1e-9:
        q = np.append(q, L)
    ref = eval_planview(pv, 0.15)
    rs = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))])
    rx, ry = np.interp(q, rs, ref[:, 0]), np.interp(q, rs, ref[:, 1])
    dx, dy = np.gradient(rx, q), np.gradient(ry, q)
    dn = np.hypot(dx, dy)
    nx, ny = -dy / np.maximum(dn, 1e-12), dx / np.maximum(dn, 1e-12)
    out = []
    for k, lane in enumerate(lanes):
        a = _boundary_track_eval(stations, bounds, k, q)
        b = _boundary_track_eval(stations, bounds, k + 1, q)
        t = (a + b) / 2.0
        xy = np.column_stack([rx + t * nx, ry + t * ny])
        ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        h = np.unwrap(np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0])))
        sm = (np.concatenate([[0.0], np.cumsum(ds)])[:-1]
              + np.concatenate([[0.0], np.cumsum(ds)])[1:]) / 2.0
        kap = np.diff(h) / np.maximum(np.diff(sm), 1e-9)
        ks = (sm[1:] + sm[:-1]) / 2.0
        sharp = np.diff(kap) / np.maximum(np.diff(ks), 1e-9) if len(kap) > 1 else np.zeros(0)
        sharp_s = (ks[1:] + ks[:-1]) / 2.0 if len(ks) > 1 else np.zeros(0)
        support = lane.get("transition_support_s")
        if isinstance(support, list) and len(support) == 2 and len(kap):
            kmask = ((sm[1:] >= float(support[0]) + 0.5)
                     & (sm[1:] <= float(support[1]) - 0.5))
            if np.any(kmask):
                kap = kap[kmask]
            if len(sharp):
                smask = ((sharp_s >= float(support[0]) + 0.75)
                         & (sharp_s <= float(support[1]) - 0.75))
                if np.any(smask):
                    sharp = sharp[smask]
        kmax = float(np.max(np.abs(kap))) if len(kap) else 0.0
        jmax = float(np.percentile(np.abs(sharp), 99.5)) if len(sharp) else 0.0
        v_ay = math.sqrt(2.5 / kmax) if kmax > 1e-12 else float("inf")
        v_j = (1.0 / jmax) ** (1.0 / 3.0) if jmax > 1e-12 else float("inf")
        kmh = min(float(ceiling_kmh), 0.90 * 3.6 * v_ay, 0.90 * 3.6 * v_j)
        lane["kmh"] = max(10.0, kmh)
        out.append({"lane_index": k + 1, "speed_kmh": lane["kmh"],
                    "kappa_max": kmax, "sharpness_p995": jmax})
    return out


def _written_jet(records, s):
    rec = max((r for r in records if r[0] <= s + 1e-9),
              key=lambda r: r[0], default=(0, 0, 0, 0, 0))
    so, a, b, c, d = rec
    u = s - so
    return np.array([a + b*u + c*u*u + d*u*u*u,
                     b + 2*c*u + 3*d*u*u, 2*c + 6*d*u], float)


def _written_lane_end(road, lane_id, *, reverse=False):
    """最终车道中心末端状态，不能用参考线切向及常偏移曲率代替。

    包含 laneOffset、累积 width（或绝对 border）的一、二阶导数。
    reverse 用于从同一 leg 末端驶出的左侧车道：位置不变，航向加 π、曲率取反。
    """
    sec = road.sections[-1]
    side = sec.left if lane_id > 0 else sec.right
    sign = 1 if lane_id > 0 else -1
    inner = _written_jet(road.offsets, road.length)
    for lane in sorted(side, key=lambda x: abs(x.lane_id)):
        outer = (_written_jet(lane.borders, road.length-sec.s) if lane.borders
                 else inner + sign*_written_jet(lane.widths, road.length-sec.s))
        if lane.lane_id == lane_id:
            t, dt, ddt = (inner + outer)/2
            pose = road.end_pose()
            kind, _, _, _, length, k0, k1 = road.geoms[-1]
            k = k1 if kind == 'spiral' else k0 if kind == 'arc' else 0.
            dk = (k1-k0)/length if kind == 'spiral' else 0.
            a, b = 1-k*t, dt
            norm2 = a*a + b*b
            if norm2 <= 1e-12:
                raise ValueError('singular written lane endpoint')
            curvature = (a*(k*a+ddt)+b*(dk*t+2*k*dt))/norm2**1.5
            x, y, _ = _shift(pose, float(t))
            heading = pose[2]+math.atan2(b, a)+(math.pi if reverse else 0.)
            return (x, y, heading), float(-curvature if reverse else curvature)
        inner = outer
    raise ValueError(f'road {road.road_id}: missing endpoint lane {lane_id}')


def _written_road_mouth(road):
    """最终道路末端的左右外缘位姿；包含 width/laneOffset 的一、二阶导数。"""
    jet = _written_jet

    L = road.length
    pose = road.end_pose()
    sec = road.sections[-1]
    offset = jet(road.offsets, L)
    kind, _x, _y, _h, glen, k0, k1 = road.geoms[-1]
    k = k1 if kind == "spiral" else k0 if kind == "arc" else 0.0
    dk = (k1 - k0) / glen if kind == "spiral" else 0.0
    mouth = {"pose": pose}
    for side, lanes, sign in (("right", sec.right, -1), ("left", sec.left, 1)):
        edge = offset.copy()
        for lane in sorted(lanes, key=lambda x: abs(x.lane_id)):
            if lane.borders:
                edge = jet(lane.borders, L - sec.s)
            else:
                edge += sign * jet(lane.widths, L - sec.s)
        t, dt, ddt = edge
        a, b = 1.0 - k*t, dt
        norm2 = a*a + b*b
        if norm2 <= 1e-12:
            raise ValueError("junction mouth has singular written outer edge")
        mouth[side + "_t"] = float(t)
        mouth[side + "_heading"] = float(pose[2] + math.atan2(b, a))
        mouth[side + "_curvature"] = float(
            (a * (k*a + ddt) + b * (dk*t + 2*k*dt)) / norm2**1.5)
    return mouth


def _rounded_mouth_apron(mouths):
    """由道路口部真实横断面构造曲率连续的 junction curb-return 包络。

    同一口部的左右角点保持直线开口；相邻道路之间不用凸包斜边或经验 Bézier，
    而用两端实际外缘状态的 SolveG2 三回旋线构造路缘回转曲线。这样道路边缘
    与 curb-return 在位置、切向和曲率上都连续，避免 Bézier 控制柄在不规则路口
    产生尖点。病态解才退回三次 Bézier，并在返回前统一做多边形有效性检查。
    """
    if len(mouths) < 3:
        return None
    from shapely.geometry import Polygon

    centers = np.asarray([[m["pose"][0], m["pose"][1]] for m in mouths], float)
    c = centers.mean(axis=0)
    entries = []
    for idx, m in enumerate(mouths):
        pose = m["pose"]
        right = np.asarray(_shift(pose, m["right_t"])[:2], float)
        left = np.asarray(_shift(pose, m["left_t"])[:2], float)
        # 中心位置只用于判断方向正反，不能代替道路真实切线；偏心路口两者
        # 可相差数十度。左右外缘还必须包含变宽/偏移导数，不能仅复用参考线航向。
        along = np.array([math.cos(pose[2]), math.sin(pose[2])], float)
        sign = 1 if np.dot(along, c - np.asarray(pose[:2])) >= 0 else -1
        for side, point in (("right", right), ("left", left)):
            h = m.get(side + "_heading", pose[2]) + (math.pi if sign < 0 else 0.0)
            entries.append({"mouth": idx, "p": point,
                            "forward": np.array([math.cos(h), math.sin(h)]),
                            "kappa": sign * m.get(side + "_curvature", 0.0)})

    ordered = sorted(entries, key=lambda e: math.atan2(e["p"][1] - c[1],
                                                        e["p"][0] - c[0]))
    outline = [ordered[0]["p"]]
    for i, a in enumerate(ordered):
        b = ordered[(i + 1) % len(ordered)]
        p0, p3 = a["p"], b["p"]
        if a["mouth"] == b["mouth"]:                    # 道路完整开口，保持直线
            outline.append(p3)
            continue
        chord = float(np.linalg.norm(p3 - p0))
        corner = None
        try:
            from pyclothoids import SolveG2
            h0 = math.atan2(a["forward"][1], a["forward"][0])
            # 曲线抵达下一道路口部时应从路口中心指向道路外侧。
            h1 = math.atan2(-b["forward"][1], -b["forward"][0])
            cls = SolveG2(p0[0], p0[1], h0, a["kappa"],
                          p3[0], p3[1], h1, -b["kappa"])
            total = sum(x.length for x in cls)
            if (total <= 4.0 * chord + 2.0
                    and all(max(abs(x.KappaStart), abs(x.KappaEnd)) <= 0.5
                            for x in cls)):
                sampled = []
                for j, cl in enumerate(cls):
                    n = max(4, int(cl.length / 0.35) + 1)
                    xs, ys = cl.SampleXY(n)
                    q = np.column_stack([xs, ys])
                    sampled.extend(q[1 if j else 0:])
                corner = sampled[1:]                     # p0 已在 outline
        except Exception:
            corner = None
        if corner is None:                               # 仅作病态数据兜底
            alpha = min(15.0, max(1.0, chord * 0.40))
            p1 = p0 + a["forward"] * alpha
            p2 = p3 + b["forward"] * alpha
            corner = []
            for t in np.linspace(0.0, 1.0, max(8, int(chord / 1.0)) + 1)[1:]:
                corner.append((1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1
                              + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3)
        outline.extend(corner)

    # 解析回旋线终点与下一口部可能仅相差 1e-14m；先规范闭合点，
    # 避免多边形留下零长度边，使下游边缘切线/三角化受浮点残差影响。
    clean = [outline[0]]
    for point in outline[1:]:
        if np.linalg.norm(np.asarray(point) - clean[-1]) > 1e-8:
            clean.append(point)
    if np.linalg.norm(np.asarray(clean[-1]) - clean[0]) < 1e-8:
        clean.pop()
    apron = Polygon(clean).buffer(0)
    if apron.geom_type == "MultiPolygon":
        apron = max(apron.geoms, key=lambda g: g.area)
    if apron.is_empty or apron.area < 10.0 or not apron.is_valid:
        return None
    return np.asarray(apron.exterior.coords, float)


def _mouth_apron_axis(mouths):
    """铺面扫掠轴锁定到最远口部对，避免近方形多边形 PCA 随机选成对角线。"""
    axes = _mouth_apron_axes(mouths)
    return axes[0] if axes else None


def _mouth_apron_axes(mouths):
    """返回最多两条近正交口部轴，用重叠 road 表达二维 junction 面。

    单条 OpenDRIVE road 的横断面本质是关于一个 s 轴的带状函数；十字路口在另一
    对道路的垂直切向处会形成表示奇异点。第一轴取最远口部对；第二轴从其余口部
    对中选与第一轴夹角最大且长度足够的一对。三腿/斜交数据若没有可靠第二轴，
    保留单轴退化路径。
    """
    if len(mouths) < 2:
        return []
    pts = np.asarray([[m["pose"][0], m["pose"][1]] for m in mouths], float)
    pairs = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            v = pts[j] - pts[i]
            L = float(np.linalg.norm(v))
            if L > 1e-6:
                pairs.append((L, i, j, v))
    if not pairs:
        return []
    L0, i0, j0, a0 = max(pairs, key=lambda x: x[0])
    u0 = a0 / L0
    axes = [a0]
    # 优先用与主轴端点不重合的另一对；角度至少 45°，避免近共线重复铺面。
    secondary = []
    for L, i, j, v in pairs:
        if i in (i0, j0) or j in (i0, j0):
            continue
        u = v / L
        cross = abs(float(u0[0] * u[1] - u0[1] * u[0]))
        if cross >= math.sin(math.radians(45.0)):
            secondary.append((L * cross, v))
    if secondary:
        axes.append(max(secondary, key=lambda x: x[0])[1])
    return axes


def _emit_lanes(sec, lanes, B, sign, j, n_sec, Ls, med_off, stats):
    """一侧一段的车道对象：宽度 = 全 road 共享 C1 边界差的 Hermite。"""
    for k, x in enumerate(lanes):
        w0 = sign * (B[j][0][k + 1] - B[j][0][k])
        w1 = sign * (B[j + 1][0][k + 1] - B[j + 1][0][k])
        mw0 = sign * (B[j][1][k + 1] - B[j][1][k])
        mw1 = sign * (B[j + 1][1][k + 1] - B[j + 1][1][k])
        lane_id = sign * (k + 1 + (med_off if sign > 0 else 0))
        provenance = dict(x.get("provenance") or {})
        support = x.get("transition_support_s") or provenance.get("support_s")
        u0, u1 = float(sec.s), float(sec.s) + float(Ls)
        if (isinstance(support, list) and len(support) == 2
                and (u1 <= float(support[0]) + 1e-6
                     or u0 >= float(support[1]) - 1e-6)):
            provenance.update({
                "eligibility": "excluded", "status": "APPROXIMATED",
                "exclusion_code": "lane-transition-taper",
                "support_kind": "lane-transition-ribbon",
            })
        lane = W.Lane(lane_id, source_id=x.get("source_id"),
                      provenance=provenance)
        lane.add_width(w0, mw0,
                       (3 * (w1 - w0) - (2 * mw0 + mw1) * Ls) / Ls ** 2,
                       (-2 * (w1 - w0) + (mw0 + mw1) * Ls) / Ls ** 3)
        lane.mark = std_mark("outer" if k == len(lanes) - 1 else "inner")
        if x.get("kmh"):
            lane.speed_ms = x["kmh"] / 3.6
            stats["speeds"] = stats.get("speeds", 0) + 1
        # 零宽端不写衔接：车道在此**不存在**（口部展宽的上游端/远端加车道的路口端），
        # 与 SHP 侧生灭车道同语义——消费端据此并线，而非骑着收拢的车道横移
        if j > 0 and w0 > 0.05:
            lane.pred = lane_id
        if j < n_sec - 1 and w1 > 0.05:
            lane.succ = lane_id
        (sec.left if sign > 0 else sec.right).append(lane)


def build_xodr(node: MapNode, out_path: str | Path,
               neighbors: list[MapNode] | None = None, *,
               connect_mode: str = "data", allow_uturn: bool = False) -> dict:
    """MapNode → .xodr（双向 leg road）。neighbors：同帧其他节点（真实出口数据）。

    connect_mode：data=仅 connectsTo（默认，不发明拓扑）/ default=补无出口车道 /
    full=全连接（治 connectsTo 缺录失配，同时把路口铺满行车带）——见 ops/junction_fill。"""
    lat0, lon0 = node.ref_lat, node.ref_lon
    proj = lambda p: _project(p, lat0, lon0)                 # noqa: E731
    doc = W.XodrDoc(f"node{node.node_id}", geo_reference=_georef(lat0, lon0))
    JID = 1
    stats = {"links": 0, "exit_roads": 0, "exit_real": 0, "exit_mirror": 0,
             "conn_roads": 0, "connections": 0, "lanelinks": 0, "skipped": 0,
             "segs": 0, "fit_dev_max": 0.0}
    manifest_lanes: dict[str, dict] = {}
    source_contexts: dict[tuple, dict] = {
        (node.region, node.node_id): {"region": node.region, "node_id": node.node_id,
                                      "role": "main"}}

    def _register_map_source(owner, link, lane, *, role, direction, stopline=False):
        sid = _source_lane_key(owner, link, lane)
        points = (_project(lane.points, lat0, lon0)
                  if len(lane.points) else np.zeros((0, 2)))
        stop = ({"availability": "anchor", "coordinates": [points[-1].tolist()],
                 "semantic": "MAP lane last point is the stop-line end"}
                if stopline and len(points) else
                {"availability": "unavailable", "reason": "lane-point-list-missing"}
                if stopline else {"availability": "not-applicable"})
        rec = source_lane(
            sid, points,
            owner={"format": "map", "region": owner.region, "node": owner.node_id,
                   "upstream": list(link.upstream or (None, None)),
                   "link": link.name, "lane": lane.lane_id},
            role=role, status="TRANSFORMED", support_kind="map-lane-point-list",
            policy_class=f"map.point-list-{role}", travel_direction=direction,
            stop_line=stop, legacy_source_key=_legacy_source_lane_key(link, lane),
        )
        if sid in manifest_lanes:
            old = manifest_lanes[sid]
            if old["geometry"]["geometry_sha256"] != rec["geometry"]["geometry_sha256"]:
                raise ValueError(f"MAP source key 冲突: {sid}")
        else:
            manifest_lanes[sid] = rec
        context = source_contexts.setdefault((owner.region, owner.node_id), {})
        context.update({
            "region": owner.region, "node_id": owner.node_id,
            "role": "main" if owner is node else "neighbor-real-exit"})
        return sid, rec

    # 先登记全部原始车道。缺少可建模 Link 的车道也不能从 G8 分母消失。
    for source_link in node.links:
        for source_ln in source_link.lanes:
            _register_map_source(node, source_link, source_ln, role="approach",
                                 direction="with_s", stopline=True)
    links = [lk for lk in node.links if len(lk.points) >= 2]
    stats["unmodeled_link_count"] = len(node.links) - len(links)
    lk_by_name = {lk.name: lk for lk in links}

    # —— 出口需求归结（remote (region,node) → 目标车道数） ——
    by_up = {}
    for lk in links:
        remote = lk.upstream or (None, None)
        if remote[1] is not None:
            by_up[remote] = lk.name
    need = {}
    for lk in links:
        for ln in lk.lanes:
            for c in ln.connects:
                if c.node is not None:
                    remote = (c.region, c.node)
                    need[remote] = max(need.get(remote, 1), c.lane or 1)

    def _real_exit_link(remote, ref, tang):
        """只接受身份和空间都属于当前 leg 的邻居真实出口。"""
        candidates = []
        for nb in (neighbors or []):
            if (nb.region, nb.node_id) != remote:
                continue
            for candidate in nb.links:
                if candidate.upstream != (node.region, node.node_id) or len(candidate.points) < 2:
                    continue
                # Selection uses raw Link geometry, not a fitted/clipped target.
                # Densification here tests overlapping corridors only; never
                # replace the source inventory with these derived samples.
                points = proj(candidate.points)
                if float((points[-1]-points[0]) @ (ref[-1]-ref[0])) >= 0:
                    continue
                link_prof = _lane_profile(ref, tang, _densify_polyline(points, 2.))
                if link_prof is None or float(np.median(link_prof[1])) <= 0:
                    continue
                candidates.append((abs(float(np.median(link_prof[1]))), nb, candidate))
        if not candidates:
            return None
        _score, owner, link = min(candidates, key=lambda item: item[0])
        return owner, link

    # 出口 → 所属 leg（完整 remote identity 必须与进口 Link 的 upstream 一致）
    exit_of_leg = {}                                         # leg name → (region, node)
    for remote in need:
        dname = by_up.get(remote)
        if dname is not None:
            exit_of_leg[dname] = remote

    in_info, exits, pave_mouths = {}, {}, []
    # —— 逐 leg：进口右侧 + 出口左侧（实测轮廓跟踪 + 站点网格多 laneSection） ——
    for i, lk in enumerate(links):
        rid = 10 + i
        remote = exit_of_leg.get(lk.name)
        raw_ref = _densify_polyline(proj(lk.points), .5)
        raw_tang = np.gradient(raw_ref, axis=0)
        raw_tang /= np.linalg.norm(raw_tang, axis=1, keepdims=True) + 1e-12
        real_hit = _real_exit_link(remote, raw_ref, raw_tang) if remote is not None else None
        support_lanes = [(_source_lane_key(node, lk, ln), proj(ln.points)) for ln in lk.lanes
                         if len(ln.points) >= 2]
        if real_hit is not None:
            real_owner, real = real_hit
            # Reverse only the geometric support to increasing reference s.
            # Source manifest retains the original with-travel order.
            support_lanes.extend((_source_lane_key(real_owner, real, ln), proj(ln.points)[::-1])
                                 for ln in real.lanes if len(ln.points) >= 2)
        support, support_meta = _reference_support(
            proj(lk.points), support_lanes)
        if support_meta["prefix_point_count"] or support_meta["rejected"]:
            rec = {"link_name": lk.name, "upstream": list(lk.upstream or (None, None)),
                   **support_meta}
            stats.setdefault("reference_support_extensions", []).append(rec)
            source_contexts[(node.region, node.node_id)].setdefault(
                "reference_support_extensions", []).append(rec)
        try:
            # MAP 车道常离参考线 5–15m；偏移曲线会放大参考线 dκ/ds。这里允许
            # 0.00025 以免把真实缓弯压离 Link.points；最终由 written lane path
            # 反算可行速度并在需要时降速，不能为保留来源 60km/h 而牺牲几何。
            pv, dev, smoothed = fit_leg_refline(
                support, sharp_cap=0.00025)
        except ReflineFitError as exc:
            raise ReflineFitError(
                f"MAP node={node.region}/{node.node_id} link={lk.name!r} "
                f"upstream={lk.upstream} points={len(lk.points)}: {exc}"
            ) from exc
        impulse = pv.fit_meta.get("impulse_filter", {})
        if impulse.get("removed_count"):
            conflict = {
                "link_name": lk.name,
                "upstream": list(lk.upstream) if lk.upstream is not None else None,
                "reason": "isolated-impulse-outlier",
                "status": "APPROXIMATED",
                "action": "excluded-from-reference-line-fit",
                "removed_indices": [int(x) for x in impulse.get("removed_indices", [])],
                "removed_reference_support_xy": [support[int(j)].tolist()
                    for j in impulse.get("removed_indices", []) if 0 <= int(j) < len(support)],
                "index_domain": "extended-reference-support-xy",
                "max_residual_m": float(impulse.get("max_residual_m", 0.0)),
            }
            stats["refline_outlier_points_removed"] = (
                stats.get("refline_outlier_points_removed", 0)
                + int(impulse["removed_count"]))
            stats["refline_outlier_max_residual_m"] = max(
                stats.get("refline_outlier_max_residual_m", 0.0),
                float(impulse.get("max_residual_m", 0.0)))
            stats.setdefault("refline_source_conflicts", []).append(conflict)
            source_contexts[(node.region, node.node_id)].setdefault(
                "refline_source_conflicts", []).append(conflict)
        if smoothed:
            stats["refit_smoothed"] = stats.get("refit_smoothed", 0) + 1
        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
        prims, ep = planview_prims(pv)
        stats["segs"] += len(prims)
        L_leg = sum(p[4] for p in prims)
        end_tangent = np.asarray([math.cos(ep[2]), math.sin(ep[2])], float)
        end_offsets = []
        for source_ln in lk.lanes:
            if not source_ln.points:
                continue
            q = proj([source_ln.points[-1]])[0]
            longitudinal = float((q - np.asarray(ep[:2], float)) @ end_tangent)
            end_offsets.append({"lane_id": int(source_ln.lane_id),
                                "longitudinal_m": longitudinal})
        if len(end_offsets) >= 2:
            values = [x["longitudinal_m"] for x in end_offsets]
            spread = float(max(values) - min(values))
            if spread > 1.0:
                contact_conflict = {
                    "link_name": lk.name,
                    "reason": "lane-end-stagger-vs-shared-road-contact-plane",
                    "status": "APPROXIMATED",
                    "action": "kept-link-reference-end",
                    "lane_end_longitudinal_offsets_m": end_offsets,
                    "spread_m": spread,
                    "representation_limit": (
                        "OpenDRIVE road lanes share one longitudinal contact plane"),
                }
                stats.setdefault("lane_contact_plane_conflicts", []).append(
                    contact_conflict)
                source_contexts[(node.region, node.node_id)].setdefault(
                    "lane_contact_plane_conflicts", []).append(contact_conflict)
        ref = eval_planview(pv, 0.5)
        tang = np.gradient(ref, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        # —— 右侧：进口车道实测轮廓（任一条缺点列则整幅回退居中堆叠常量） ——
        rlanes = []
        for ln in lk.lanes:
            pdata = (_lane_profile(ref, tang, proj(ln.points), return_points=True,
                                   densify_profile=True)
                     if len(ln.points) >= 2 else None)
            prof, support_points = pdata if pdata is not None else (None, None)
            raw_prof = (_lane_profile(ref, tang, support_points)
                        if support_points is not None else None)
            sid, sm = _register_map_source(node, lk, ln, role="approach",
                                           direction="with_s", stopline=True)
            support_s = [float(prof[0].min()), float(prof[0].max())] if prof is not None else None
            x = {"ln": ln, "prof": prof,
                 "profile_knots": raw_prof[0] if raw_prof is not None else None,
                 "w": (ln.width_cm or 350) / 100.0,
                 "source_points": support_points,
                 "source_id": sid,
                 "provenance": {
                     "eligibility": "comparable", "role": "approach",
                     "status": sm["status"], "support_kind": sm["support_kind"],
                     "policy_class": sm["policy_class"], "travel_direction": "with_s",
                     "support_s": support_s,
                 },
                 "kmh": _lane_speed_kmh(ln)}
            if prof is not None:                          # 进口车道按 MAP 语义必抵停止线，
                lo = float(prof[0].min())                 # 远端起点按数据（口部展宽车道锥形展开）
                x["cov"] = (-1e18 if lo < 20.0 else lo - 10.0, 1e18)
                if lo >= 20.0:
                    stats["flare_lanes"] = stats.get("flare_lanes", 0) + 1
            rlanes.append(x)
        if any(x["prof"] is None for x in rlanes):
            total_w = sum(x["w"] for x in rlanes)
            cum = 0.0
            for x in rlanes:
                x["prof"], x["const"] = None, total_w / 2 - cum - x["w"] / 2
                cum += x["w"]
        rlanes.sort(key=lambda x: -_dv(x, L_leg)[0])     # 左→右
        stations = _adaptive_section_stations(
            L_leg, [x.get("profile_knots") for x in rlanes])
        n_sec = len(stations) - 1
        Br = [_bounds(rlanes, u, -1) for u in stations]

        # —— 左侧：本 leg 的出口（real 实测轮廓 / real 无点列堆叠 / mirror 镜像） ——
        llanes, Bl, med, has_med, kind = [], None, None, False, None
        if remote is not None:
            lane_ws = [x.width_cm for x in lk.lanes if x.width_cm]
            def_w = (sorted(lane_ws)[len(lane_ws) // 2] / 100.0) if lane_ws else 3.5
            if real_hit is not None:
                real_owner, real = real_hit
                for source_ln in real.lanes:
                    _register_map_source(real_owner, real, source_ln, role="departure",
                                         direction="against_s")
                point_lanes = [ln for ln in real.lanes if len(ln.points) >= 2]
                for ln in point_lanes:
                    pdata = _lane_profile(ref, tang, proj(ln.points), return_points=True,
                                          densify_profile=True)
                    if pdata is None:
                        continue
                    prof, support_points = pdata
                    raw_prof = _lane_profile(ref, tang, support_points)
                    # Physical left/right is relative to laneOffset, not t=0.
                    # A selected true departure may have negative t on a valid
                    # coordinate spine. Keep it; joint width/fidelity checks
                    # must report a real overlap instead of dropping its ID.
                    sid, sm = _register_map_source(real_owner, real, ln, role="departure",
                                                   direction="against_s")
                    lo, hi = float(prof[0].min()), float(prof[0].max())
                    llanes.append({"prof": prof,
                                   "profile_knots": (
                                       raw_prof[0] if raw_prof is not None else None),
                                   "lid": ln.lane_id,
                                   "source_points": support_points,
                                   "source_id": sid,
                                   "provenance": {
                                       "eligibility": "comparable", "role": "departure",
                                       "status": sm["status"], "support_kind": sm["support_kind"],
                                       "policy_class": sm["policy_class"],
                                       "travel_direction": "against_s",
                                       "support_s": [lo, hi],
                                   },
                                   "w": (ln.width_cm or 350) / 100.0,
                                   "kmh": _lane_speed_kmh(ln),
                                   # 出口两端均按数据覆盖（远端才加出的车道不得外推进口部）
                                   "cov": (-1e18 if lo < 20.0 else lo - 10.0,
                                           1e18 if hi > L_leg - 20.0 else hi + 10.0)})
                llanes.sort(key=lambda x: _dv(x, L_leg)[0])   # 内→外
                if not llanes and not point_lanes:          # 源 lane 无点列：保留不可测事实
                    cum = 0.0
                    for ln in real.lanes:
                        sid, sm = _register_map_source(real_owner, real, ln, role="departure",
                                                       direction="against_s")
                        w = (ln.width_cm or 350) / 100.0
                        llanes.append({"prof": None, "rel": cum + w / 2, "w": w,
                                       "source_id": sid,
                                       "provenance": {
                                           "eligibility": "comparable", "role": "departure",
                                           "status": sm["status"],
                                           "support_kind": sm["support_kind"],
                                           "policy_class": sm["policy_class"],
                                           "travel_direction": "against_s",
                                       },
                                       "kmh": _lane_speed_kmh(ln)})
                        cum += w
                if llanes:
                    kind = "real"
                else:
                    stats["exit_real_rejected"] = stats.get("exit_real_rejected", 0) + 1
            if not llanes:                                  # 无严格匹配实测出口：同 leg 全断面镜像兜底
                mirror_count = len(rlanes)
                if need[remote] != mirror_count:
                    stats["mirror_lane_count_from_approach"] = \
                        stats.get("mirror_lane_count_from_approach", 0) + 1
                llanes = [{"prof": None, "rel": None, "w": rlanes[k]["w"],
                           "kmh": None,
                           "transition_support_s":
                               rlanes[k].get("provenance", {}).get("support_s"),
                           "provenance": {
                               "eligibility": "excluded", "role": "departure",
                               "status": "INFERRED", "support_kind": "mirror",
                               "travel_direction": "against_s",
                               "exclusion_code": "mirror-no-source-geometry",
                           }} for k in range(mirror_count)]
                kind = "mirror"
        if llanes and kind == "real":
            refined = _adaptive_section_stations(
                L_leg, [x.get("profile_knots") for x in rlanes + llanes])
            if len(refined) != len(stations) or not np.allclose(refined, stations):
                stations = refined
                n_sec = len(stations) - 1
                Br = [_bounds(rlanes, u, -1) for u in stations]
        stats["adaptive_section_min_gap_m"] = min(
            stats.get("adaptive_section_min_gap_m", float("inf")),
            min(np.diff(stations), default=L_leg))
        stats["adaptive_section_max_gap_m"] = max(
            stats.get("adaptive_section_max_gap_m", 0.0),
            max(np.diff(stations), default=L_leg))
        if llanes:
            if kind == "mirror":
                # MAP 缺出口几何：以同一进口方向的车道组内边界为轴做严格左右镜像。
                # 这保留每条进口车道的实际宽度轮廓，而不是按路口中心或中位宽复制。
                Bl = [_mirror_bounds(Br[j], len(llanes), def_w)
                      for j in range(len(stations))]
            else:
                Bl = [_bounds(llanes, stations[j], +1,
                              base=(Br[j][0][0], Br[j][1][0]))
                      for j in range(len(stations))]
            # 先按全 road 求一次共享站点导数，再由同一组导数写入前后 section。
            # 必须在 median 之前做，因为 median 也是左右内缘之差。
            if kind == "mirror":
                Br, lam_r, met_r = _regularize_measured_boundaries(
                    stations, Br, rlanes, -1, ref=ref, tang=tang)
                # 镜像必须来自同一进口方向的最终物理断面。先把真实进口正则化，
                # 再连同唯一导数一起左右镜像；若先镜像原始缺测阶跃、再独立平滑，
                # 两侧会在路口端产生不同的收敛速度和 jerk 尖峰。
                Bl = [_mirror_bounds(Br[j], len(llanes), def_w)
                      for j in range(len(stations))]
                # 左右镜像只补几何；源中没有出口限速就保持缺失。
                # 设计速度属于验收 Profile，不能由待验曲线反推后写成限速。
            else:
                Br, lam_r, met_r, Bl, lam_l, met_l = _regularize_measured_pair(
                    stations, Br, rlanes, Bl, llanes, ref=ref, tang=tang)
                stats.setdefault("boundary_smoothing", []).append({
                    "road": lk.name, "side": "left", "lambda": lam_l,
                    "source_error": {
                        k: met_l[k] for k in ("median_m", "p95_m", "max_m")},
                    "selection_trials": met_l.get("selection_trials", []),
                })
            stats.setdefault("boundary_smoothing", []).append({
                "road": lk.name, "side": "right", "lambda": lam_r,
                "source_error": {k: met_r[k] for k in ("median_m", "p95_m", "max_m")},
                "selection_trials": met_r.get("selection_trials", []),
            })
            med = []
            for j in range(len(stations)):
                g = Bl[j][0][0] - Br[j][0][0]
                mg = Bl[j][1][0] - Br[j][1][0]
                med.append((g, mg) if g > 0 else (0.0, 0.0))
            has_med = any(g > 0.05 for g, _m in med)
        else:
            Br, lam_r, met_r = _regularize_measured_boundaries(
                stations, Br, rlanes, -1, ref=ref, tang=tang)
            stats.setdefault("boundary_smoothing", []).append({
                "road": lk.name, "side": "right", "lambda": lam_r,
                "source_error": {k: met_r[k] for k in ("median_m", "p95_m", "max_m")},
                "selection_trials": met_r.get("selection_trials", []),
            })
        two_way = bool(llanes)
        med_off = 1 if has_med else 0

        # —— 写 road：planView + 逐 section laneOffset/median/左右车道（全 Hermite+FC 限幅） ——
        road = W.Road(rid, name=lk.name)
        for pr in prims:
            road.add_geometry(*pr)
        for j in range(n_sec):
            u0, u1 = stations[j], stations[j + 1]
            Ls = max(u1 - u0, 1e-3)
            sec = W.LaneSection(u0, center_mark=std_mark("center2" if two_way else "center"))
            y0, my0 = Br[j][0][0], Br[j][1][0]            # laneOffset = 进口幅左缘实测
            y1, my1 = Br[j + 1][0][0], Br[j + 1][1][0]
            road.add_offset(u0, y0, my0,
                            (3 * (y1 - y0) - (2 * my0 + my1) * Ls) / Ls ** 2,
                            (-2 * (y1 - y0) + (my0 + my1) * Ls) / Ls ** 3)
            if has_med:                                   # median 恒 +1（宽随 s 变化）
                g0, mg0 = med[j]
                g1, mg1 = med[j + 1]
                mlane = W.Lane(1, "median", provenance={
                    "eligibility": "excluded", "role": "median",
                    "status": "TRANSFORMED", "support_kind": "median",
                    "travel_direction": "against_s", "exclusion_code": "median-non-driving",
                })
                mlane.add_width(g0, mg0,
                                (3 * (g1 - g0) - (2 * mg0 + mg1) * Ls) / Ls ** 2,
                                (-2 * (g1 - g0) + (mg0 + mg1) * Ls) / Ls ** 3)
                if j > 0:
                    mlane.pred = 1
                if j < n_sec - 1:
                    mlane.succ = 1
                sec.left.append(mlane)
            for lanes, B, sign in ((llanes, Bl, +1), (rlanes, Br, -1)):
                if not lanes:
                    continue
                _emit_lanes(sec, lanes, B, sign, j, n_sec, Ls, med_off, stats)
            road.sections.append(sec)
            stats["sections"] = stats.get("sections", 0) + 1
        road.add_link("successor", "junction", JID)
        doc.add_road(road)

        # —— 路口端车道位姿：按 written 末站边界取中（连接路 G2 精确瞄准） ——
        xid, tmap = {}, {}
        for k, x in enumerate(rlanes):
            xid[x["ln"].lane_id] = -(k + 1)
            tmap[x["ln"].lane_id] = (Br[-1][0][k] + Br[-1][0][k + 1]) / 2
        in_info[lk.name] = {"rid": rid, "xid": xid, "t": tmap, "pose": ep,
                            "kappa": seg_kappa(pv, at_end=True),
                            "lane_states": {lid: _written_lane_end(road, xid[lid])
                                            for lid in xid}}
        pave_mouths.append(_written_road_mouth(road))
        stats["links"] += 1
        if llanes:
            exid, etmap = {}, {}
            # 起点取 written 链（laneOffset + 已钳 median）——出口与进口幅横向重叠时
            # median 被钳到 0，写出堆叠会整体外移，瞄准点必须跟着 written 走
            cum = Br[-1][0][0] + (med[-1][0] if has_med else 0.0)
            if abs(cum - Bl[-1][0][0]) > 0.05:
                stats["exit_overlap_shift_m"] = round(abs(cum - Bl[-1][0][0]), 2)
            ctr, wid = [], []                             # 路口端各出口车道 written 中心/宽
            for k in range(len(llanes)):
                w = Bl[-1][0][k + 1] - Bl[-1][0][k]
                ctr.append(cum + w / 2)
                wid.append(w)
                cum += w
            for k, x in enumerate(llanes):
                kk = k                                    # 口部零宽（远端才加出的车道）：
                if wid[k] < 0.5:                          # 连接改瞄最近的实际存在车道
                    cand = [j for j in range(len(wid)) if wid[j] >= 0.5]
                    if cand:
                        kk = min(cand, key=lambda j: abs(j - k))
                        stats["exit_lane_remap"] = stats.get("exit_lane_remap", 0) + 1
                key = x.get("lid", k + 1)                 # 目标车道号（MAP 1 起）→ xodr id
                exid[key] = kk + 1 + med_off
                etmap[key] = ctr[kk]
            exits[remote] = {"rid": rid, "xid": exid, "t": etmap, "pose": ep,
                             "kappa": seg_kappa(pv, at_end=True), "n": len(llanes),
                             "kind": kind, "contact": "end",
                             "lane_states": {lid: _written_lane_end(road, exid[lid], reverse=True)
                                             for lid in exid}}
            stats["exit_real" if kind == "real" else "exit_mirror"] += 1
            stats["exit_roads"] += 1

    # —— 连接路：模型端部位姿 + 车道级曲率 G2 ——
    junction = W.Junction(JID, f"node{node.node_id}")
    rid_c = 100
    pave_pts = []
    def _make_conn(cls, w, in_rid, in_xid, out_rid, out_xid, contact,
                   exclusion_code="inferred-connector-no-source-geometry",
                   fit_diagnostics=None):
        """一条连接路（G2 回旋链 + 单车道 + junction connection）。"""
        nonlocal rid_c
        road = W.Road(rid_c, junction=JID)
        for cl in cls:
            road.add_geometry("spiral", cl.XStart, cl.YStart, cl.ThetaStart,
                              cl.length, cl.KappaStart, cl.KappaEnd)
        from mapforge.ops.connector_width import written_lane_width, centred_width_records
        in_road = next(r for r in doc.roads if r.road_id == in_rid)
        out_road = next(r for r in doc.roads if r.road_id == out_rid)
        w_in = written_lane_width(in_road, in_xid, 'end')
        w_out = written_lane_width(out_road, out_xid, contact)
        width_records = centred_width_records(w_in, w_out, road.length)
        for station, *coefficients in width_records:
            road.add_offset(station, *[v / 2 for v in coefficients])
        csec = W.LaneSection(0.0)
        provenance = {
            "eligibility": "excluded", "role": "connector", "status": "INFERRED",
            "support_kind": "synthetic-connector", "travel_direction": "with_s",
            "exclusion_code": exclusion_code,
        }
        if fit_diagnostics is not None:
            provenance["curve_fit"] = fit_diagnostics
        provenance["width_basis"] = "final-linked-lane-endpoints"
        provenance["width_transition"] = "fixed-three-span-C2-zero-end-jets"
        provenance["width_endpoints_m"] = [w_in, w_out]
        provenance["nominal_incoming_width_m"] = w
        provenance["edge_jet_continuity"] = "not-guaranteed-by-width-size-matching"
        clane = W.Lane(-1, provenance=provenance)
        for station, *coefficients in width_records:
            clane.add_width(*coefficients, s_offset=station)
        # MAP connectsTo 没有连接道路限速；缺失原样保留。v_supported 只可
        # 出现在质量报告，不能回写 speed 再让同一曲线通过动力学门禁。
        provenance["speed_limit_basis"] = "absent-in-source"
        clane.pred, clane.succ = in_xid, out_xid
        csec.right.append(clane)
        road.sections.append(csec)
        road.add_link("predecessor", "road", in_rid, "end")
        road.add_link("successor", "road", out_rid, contact)
        doc.add_road(road)
        pave_pts.append(road.end_pose()[:2])
        pave_pts.extend((g[1], g[2]) for g in road.geoms)
        conn = W.Connection(in_rid, rid_c, "start")
        conn.add_lanelink(in_xid, -1)
        junction.connections.append(conn)
        rid_c += 1
        stats["conn_roads"] += 1
        stats["connections"] += 1
        stats["lanelinks"] += 1

    filled_pairs = set()                                 # 已建连接对（补全去重用）
    for lk in links:
        ii = in_info[lk.name]
        for ln in lk.lanes:
            for c in ln.connects:
                remote = (c.region, c.node)
                e = exits.get(remote)
                if e is None or ln.lane_id not in ii["t"]:
                    stats["skipped"] += 1
                    continue
                p0, k0 = ii["lane_states"][ln.lane_id]
                kk = c.lane or 1                         # 目标车道号（键=出口 Link 车道号）
                if kk not in e["t"]:                     # 号不在册：退最近可用号（不发明拓扑）
                    kk = min(e["t"], key=lambda j: abs(j - kk))
                p1, k1 = e["lane_states"][kk]
                try:
                    cls, tuning = solve_g2_balanced(p0, p1, k0, k1)
                except Exception:
                    stats["skipped"] += 1
                    continue
                filled_pairs.add(((lk.name, ln.lane_id), (remote, kk)))
                _make_conn(cls, (ln.width_cm or 350) / 100.0, ii["rid"],
                           ii["xid"][ln.lane_id], e["rid"], e["xid"].get(kk, kk),
                           e["contact"], fit_diagnostics=tuning)

    # —— 转向补全（connect_mode≠data）：connectsTo 缺录/失配时按几何补，标 INFERRED ——
    if connect_mode != "data" and exits:
        from mapforge.ops.junction_fill import plan_fill
        leg_of_exit = {nid: name for name, nid in exit_of_leg.items()}
        ent_meta, exit_meta = [], []
        for lk in links:
            ii = in_info[lk.name]
            ids = sorted(ii["t"], key=lambda i: -ii["xid"][i])    # -1 最左 → 断面序
            for i, lid in enumerate(ids):
                ent_meta.append({"key": (lk.name, lid), "leg": lk.name, "idx": i,
                                 "n": len(ids), "pose": ii["lane_states"][lid][0],
                                 "rid": ii["rid"], "xid": ii["xid"][lid],
                                 "kappa": ii["lane_states"][lid][1],
                                 "w": next((x.width_cm or 350) / 100.0
                                           for x in lk.lanes if x.lane_id == lid)})
        for nid, e in exits.items():
            ks = sorted(e["t"])                                   # 号小=靠中线
            for i, kk in enumerate(ks):
                q, kappa = e["lane_states"][kk]
                exit_meta.append({"key": (nid, kk), "leg": leg_of_exit.get(nid, nid),
                                  "idx": i, "n": len(ks),
                                  "pose": q,
                                  "rid": e["rid"], "xid": e["xid"].get(kk, kk),
                                  "kappa": kappa,
                                  "contact": e["contact"]})
        for en, ex in plan_fill(ent_meta, exit_meta, filled_pairs,
                                mode=connect_mode, allow_uturn=allow_uturn):
            try:
                cls, tuning = solve_g2_balanced(
                    en["pose"], ex["pose"], en["kappa"], ex["kappa"])
            except Exception:
                cls = None
                tuning = None
            if cls is None or any(max(abs(c.KappaStart), abs(c.KappaEnd)) > 0.125
                                  for c in cls):
                stats["conn_fill_skipped"] = stats.get("conn_fill_skipped", 0) + 1
                continue                                  # 无解或 R<8m：判为不可行转向
            _make_conn(cls, en["w"], en["rid"], en["xid"], ex["rid"], ex["xid"],
                       ex["contact"], "filled-connector-no-source-geometry",
                       fit_diagnostics=tuning)
            stats["conn_filled"] = stats.get("conn_filled", 0) + 1

    # —— junction 铺面：MAP 无面数据 → 道路口部边界圆角包络（INFERRED） ——
    if pave_pts:
        try:
            from shapely.geometry import MultiPoint
            from mapforge.adapters.opendrive.writer import add_paving_road
            apron = _rounded_mouth_apron(pave_mouths)
            support_kind = "mouth-envelope-rounded"
            if apron is None:                             # 少于 3 个有效口部才退旧兜底
                hull = MultiPoint(pave_pts).convex_hull.buffer(2.5)
                apron = np.asarray(hull.exterior.coords)
                support_kind = "connector-hull-fallback"
            axes = _mouth_apron_axes(pave_mouths)
            if not axes:
                axes = [None]
            paved = 0
            for patch_i, axis in enumerate(axes):
                if add_paving_road(doc, apron, JID, road_id=90 + patch_i,
                                   smooth_profile=True, preferred_axis=axis,
                                   overlap_m=0.75, provenance={
                        "eligibility": "excluded", "role": "paving", "status": "INFERRED",
                        "support_kind": support_kind, "travel_direction": "with_s",
                        "exclusion_code": "inferred-paving"}):
                    paved += 1
            if paved:
                stats["paving"] = "mouth-apron" if support_kind == "mouth-envelope-rounded" else "hull"
                stats["paving_roads"] = paved
        except Exception:
            stats["paving"] = "skip"

    doc.add_junction(junction)
    doc.write(out_path)
    stats["speed_limit_policy"] = "preserve-source-no-geometry-derived-caps"
    stats["source_lane_manifest"] = make_manifest(
        source_format="map", source_profile="jinfeng-map-xml",
        comparison_crs={
            "id": "local-eqc", "units": "m", "axis_order": ["x", "y"],
            "origin": {"lon": lon0, "lat": lat0}, "proj_string": doc.geo_reference,
            "integrity": "internally-consistent",
        },
        source_contexts=[source_contexts[k] for k in sorted(source_contexts,
                                                             key=lambda x: (str(x[0]), str(x[1])))],
        lanes=[manifest_lanes[k] for k in sorted(manifest_lanes)],
    )
    return stats
