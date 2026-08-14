# -*- coding: utf-8 -*-
"""SHP(IBD) → OpenDRIVE 直转（方向 3；不过 MAP 窄门，junction 一等公民）。

道路模型 = OpenDRIVE 规范做法：**一条街一条 road，参考线沿进口幅中心，双侧展开**——
进口车道挂右侧（-1..-n），对向出口车道按实测横距挂左侧（+1..+n），
两幅间实测间隙写 median 车道（宽可为 0），中线双黄。进/出口按端点位姿+方向自动配对；
配不上的单向链退化为单侧 road（物理分隔/数据缺失时的合法形态）。

写出走自研规范级 writer（adapters/opendrive/writer.py，弃 scenariogeneration）：
车道 <speed>/<roadMark> 原生落盘、geoReference 记录 PROJ 管线、无后处理补丁。

断面：**多 laneSection**——进口链每源 Link 一段、对向链边界投影到参考线后与进口边界
合并（1.5m 内吸附）；每车道 S_WIDTH→E_WIDTH smoothstep 变宽、laneOffset C1 单调样条、
生/灭车道 20m 锥形收放、断面重划分处起宽衔接上段末宽（Σ宽连续）。
路口：<junction> + 连接路（路口内车道实测几何 + 两端 G2 桥）+ Connection/laneLink；
出口连接路接同一条 leg road 的 END 接触点（左侧车道，行车方向沿 s 递减）。

已知简化（正式化清单）：高程平面输出（SLOPE/BANKING 字段未核验，资料盘点）；
标线为通行画法默认（MARK_TYPE 权威枚举待消费）；lane_type 除 median 外全 driving。
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from mapforge.adapters.opendrive import writer as W
from mapforge.adapters.opendrive.writer import std_mark
from mapforge.adapters.shp.ibd_reader import IbdSource, JunctionRec
from mapforge.ops.refline_fit import (eval_planview, fc_clamp, fit_leg_refline,
                                      fit_polyline_auto, g2ify_planview,
                                      planview_prims, seg_kappa, simplify_planview,
                                      weld_g2)

R_EARTH = 6378137.0
_DEG_EPS = 1e-5          # 端点衔接容差（度，≈1m）
_TAPER = 20.0            # 生/灭车道锥形收放长度（m，APPROXIMATED：数据以整 Link 粒度生灭）
_SNAP = 1.5              # 对向链边界向进口边界吸附距离（m）


def _smooth_w(w0, w1, L):
    """smoothstep 三次系数（两端零斜率）：宽度过渡 C1，边缘无折角。"""
    L = max(L, 1e-3)
    return (w0, 0.0, 3 * (w1 - w0) / L ** 2, -2 * (w1 - w0) / L ** 3)


def _width_pieces(sw, ew, L, born, dying, sw_join=None):
    """车道宽度分段多项式 [(sOffset, a, b, c, d)]。
    生车道从 0 张开、灭车道收拢到 0、sw_join 覆盖起宽与上段末宽衔接——
    数据按 Link 粒度瞬现/瞬失/横断面重划分，直写会在边缘留矩形缺口（查看器显形）；
    锥形/衔接段 ≤_TAPER 后回归数据值，为记录在案的修复。所有过渡 smoothstep（两端零斜率）。"""
    if born and dying:                                   # 仅存在于本 section 的短车道
        T = L / 2
        wm = max(sw, ew)
        return [(0.0, *_smooth_w(0.0, wm, T)), (T, *_smooth_w(wm, 0.0, L - T))]
    w0 = 0.0 if born else sw
    if not born and sw_join is not None and abs(sw_join - sw) > 0.01:
        w0 = sw_join
    if dying:
        T = min(_TAPER, L)
        if T < L - 1e-6:
            w_t = sw + (ew - sw) * (L - T) / L
            return [(0.0, *_smooth_w(w0, w_t, L - T)), (L - T, *_smooth_w(w_t, 0.0, T))]
        return [(0.0, *_smooth_w(w0, 0.0, L))]
    T = min(_TAPER, L)
    if born:                                             # 生车道锥形：必须张满到 ew——
        w_t = sw + (ew - sw) * T / L                     # 坡度上限只属于衔接分支
        out = [(0.0, *_smooth_w(0.0, w_t, T))]           # （误入会钳住张开量，下段跳台阶）
        if T < L - 1e-6:
            out.append((T, *_smooth_w(w_t, ew, L - T)))
        return out
    if abs(w0 - sw) > 1e-9:                              # 起宽被覆盖（衔接上段末宽）
        w_end = ew
        max_dw = 0.10 * L                                # 坡度上限：车道中心横摆可驾驶
        if abs(ew - w0) > max_dw:                        # 短 section 消化不完 → 残量传下段
            w_end = w0 + math.copysign(max_dw, ew - w0)
        if abs(w_end - ew) < 1e-9 and T < L - 1e-6:
            w_t = sw + (ew - sw) * T / L
            return [(0.0, *_smooth_w(w0, w_t, T)), (T, *_smooth_w(w_t, ew, L - T))]
        return [(0.0, *_smooth_w(w0, w_end, L))]
    return [(0.0, *_smooth_w(w0, ew, L))]


_fc = fc_clamp                                           # Hermite 斜率单调限幅（共享）


def _pieces_end(pieces, L):
    """分段宽度多项式在 section 末端的实际值（written 末宽——衔接链的真值）。"""
    so, a, b, c, d = pieces[-1]
    t = max(L - so, 0.0)
    return a + b * t + c * t ** 2 + d * t ** 3


def _proj(pts_deg, lat0, lon0):
    pts = np.asarray(pts_deg, dtype=float)
    x = np.radians(pts[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = np.radians(pts[:, 1] - lat0) * R_EARTH
    return np.column_stack([x, y])


def _georef(lat0, lon0):
    """本地平面投影的 PROJ 管线（球面 eqc，与 _proj 数学一致）——CRS 硬约束 3。"""
    return (f"+proj=eqc +lat_ts={lat0:.8f} +lat_0={lat0:.8f} +lon_0={lon0:.8f} "
            f"+R={R_EARTH:.0f} +units=m +no_defs")


def _polylen(pts_xy):
    return float(np.linalg.norm(np.diff(pts_xy, axis=0), axis=1).sum()) if len(pts_xy) >= 2 else 0.0


def _center_of(src: IbdSource, pid: str):
    g = src.roadcenters.get(pid)
    if g is None or g.shape[0] < 2:                      # 缺中心线：中间车道近似
        lanes = [l for l in src.lanes_of(pid) if l.geometry.shape[0] >= 2]
        if not lanes:
            return None
        g = lanes[len(lanes) // 2].geometry
    return np.asarray(g, float)


def _chained_links(src: IbdSource, seed_pid: str, is_enter: bool, proj, cj,
                   max_len: float = 160.0, hops: int = 5):
    """同名 ROADLINK 端点拼链（**跨车道数**——车道数变化由多 laneSection 表达）。
    返回 [(link_pid, center_lonlat)]，顺序=行车方向（enter: 上游→路口；leave: 路口→下游）。"""
    g0 = _center_of(src, seed_pid)
    if g0 is None:
        return None
    if (np.linalg.norm(proj(g0[:1])[0] - cj) < np.linalg.norm(proj(g0[-1:])[0] - cj)) == is_enter:
        g0 = g0[::-1]
    chain, used = [(seed_pid, g0)], {seed_pid}
    rl0 = src.roadlinks[seed_pid]
    total = _polylen(proj(g0))
    for _ in range(hops):
        if total >= max_len:
            break
        far = chain[0][1][0] if is_enter else chain[-1][1][-1]
        best = None
        for pid, c in src.roadcenters.items():
            if pid in used:
                continue
            rl = src.roadlinks.get(pid)
            if rl is None or rl.name != rl0.name:
                continue
            if not [l for l in src.lanes_of(pid) if l.geometry.shape[0] >= 2]:
                continue                                 # 无车道数据的段建不了 section
            d0 = float(np.linalg.norm(c[0] - far))
            d1 = float(np.linalg.norm(c[-1] - far))
            if min(d0, d1) < _DEG_EPS:
                cand = (c if d1 <= d0 else c[::-1]) if is_enter else (c if d0 <= d1 else c[::-1])
                best = (pid, cand) if best is None else "AMBIG"
        if best is None or best == "AMBIG":              # 无候选或分叉即停
            break
        pid, cand = best
        used.add(pid)
        chain.insert(0, (pid, cand)) if is_enter else chain.append((pid, cand))
        total += _polylen(proj(cand))
    return chain


def _fit_ref(pts_xy, resample_step=2.0, min_seg_len=6.0):
    return fit_polyline_auto(pts_xy, resample_step, min_seg_len)   # 升级档拟合（refline_fit 共用）


def _shift(pose, t):
    x, y, h = pose
    return (x - t * math.sin(h), y + t * math.cos(h), h)


def _g2_prims(p0, p1):
    """两端位姿 → 单条 G2 回旋链（SolveG2 三段）的几何原语；病态解返回 None。"""
    from pyclothoids import SolveG2
    try:
        cls = SolveG2(p0[0], p0[1], p0[2], 0.0, p1[0], p1[1], p1[2], 0.0)
    except Exception:
        return None
    if any(max(abs(c.KappaStart), abs(c.KappaEnd)) > 0.5 for c in cls):
        return None                                      # 猪尾巴（R<2m 回环）
    return [("spiral", c.XStart, c.YStart, c.ThetaStart, c.length,
             c.KappaStart, c.KappaEnd) for c in cls]


def _prims_dev(prims, pts):
    """几何原语链对实测点列的最大横向偏差（每 0.5m 采样最近距）。"""
    from mapforge.ops.refline_fit import PlanSeg, PlanView
    segs = [PlanSeg("spiral" if p[0] == "spiral" else p[0], p[4], p[5],
                    p[6] if p[0] == "spiral" else None) for p in prims]
    ref = eval_planview(PlanView(prims[0][1], prims[0][2], prims[0][3], segs), 0.5)
    return float(max(np.min(np.linalg.norm(ref - q[None, :], axis=1)) for q in pts))


def _bridged_geoms(vg, p0, p1, fit_fn, scale=1.0):
    """路口内车道实测折线 + 两端 G2 桥：起点精确接进口车道模型位姿 p0、
    末端精确接出口车道模型位姿 p1（换乘跳变构造性归零）。中段保留实测几何。
    守卫：桥长 >40m 或桥内 |κ|>0.5（R<2m"猪尾巴"回环，位姿近退化时 SolveG2
    的病态解）即拒绝——调用侧按 scale 放大切口重试。返回 (prims, dev) 或 None。"""
    from pyclothoids import SolveG2
    d = np.linalg.norm(np.diff(vg, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    L = float(s[-1])
    # 切口下限 8m：SolveG2 桥是 3 段回旋线，切口太小会把桥压成 1–2m 的碎段
    # （消费端读到高频曲率锯齿）；8m 切口 ⇒ 桥段 ≈2.5–4m，仍是合理的缓和过渡长度
    cut0 = min(max(8.0, 2.0 * float(np.linalg.norm(vg[0] - p0[:2]))) * scale, L / 3)
    cut1 = min(max(8.0, 2.0 * float(np.linalg.norm(vg[-1] - p1[:2]))) * scale, L / 3)
    mid = vg[(s >= cut0) & (s <= L - cut1)]
    if mid.shape[0] < 4:
        return None
    pv, dev = fit_fn(mid)
    got = simplify_planview(pv, mid, dev_tol=min(0.55, max(0.4, dev * 1.5)),
                            min_seg_len=12.0)
    if got is not None:                                  # 中段曲率域精简（消碎段）
        pv, dev = got[0], got[1]
        pv = weld_g2(pv)                                 # 残差焊平（不增段）
    else:
        pv = weld_g2(g2ify_planview(pv)[0])              # 未精简：插过渡段 + 焊平兜底
    prims_mid, ep = planview_prims(pv)
    try:
        b0 = SolveG2(p0[0], p0[1], p0[2], 0.0,
                     pv.x0, pv.y0, pv.hdg, seg_kappa(pv, at_end=False))
        b1 = SolveG2(ep[0], ep[1], ep[2], seg_kappa(pv, at_end=True),
                     p1[0], p1[1], p1[2], 0.0)
    except Exception:
        return None
    bridge = list(b0) + list(b1)
    if (sum(c.length for c in b0) > 40 or sum(c.length for c in b1) > 40
            or any(max(abs(c.KappaStart), abs(c.KappaEnd)) > 0.5 for c in bridge)):
        return None                                      # 桥失控/猪尾巴：由调用侧重试
    prims = [("spiral", c.XStart, c.YStart, c.ThetaStart, c.length,
              c.KappaStart, c.KappaEnd) for c in b0]
    prims += prims_mid
    prims += [("spiral", c.XStart, c.YStart, c.ThetaStart, c.length,
               c.KappaStart, c.KappaEnd) for c in b1]
    return prims, dev


def _pair_legs(src: IbdSource, junc: JunctionRec, proj, cj):
    """进/出口链按路口侧端点+方向配对成"腿"：同腿 = 端点近（<45m）且行车方向相反。
    返回 ([(enter_pid, leave_pid|None)], [unpaired_leave_pid])。"""
    def seed_info(pid, is_enter):
        g = _center_of(src, pid)
        if g is None or g.shape[0] < 2:
            return None
        gx = proj(g)
        if (np.linalg.norm(gx[0] - cj) < np.linalg.norm(gx[-1] - cj)) == is_enter:
            gx = gx[::-1]                                # 统一为行车方向
        if is_enter:
            pt, dv = gx[-1], gx[-1] - gx[-2]
        else:
            pt, dv = gx[0], gx[1] - gx[0]
        n = np.linalg.norm(dv)
        return (pt, dv / n) if n > 1e-9 else None

    enters = {p: seed_info(p, True) for p in junc.enter_roads}
    leaves = {p: seed_info(p, False) for p in junc.leave_roads}
    cands = []
    for e, ei in enters.items():
        for l, li in leaves.items():
            if ei is None or li is None:
                continue
            if float(np.dot(ei[1], li[1])) > -0.3:       # 同腿的对向：方向必须相反
                continue
            cands.append((float(np.linalg.norm(ei[0] - li[0])), e, l))
    used_e, used_l, pair_of = set(), set(), {}
    for dist, e, l in sorted(cands):
        if dist > 45 or e in used_e or l in used_l:
            continue
        used_e.add(e)
        used_l.add(l)
        pair_of[e] = l
    pairs = [(e, pair_of.get(e)) for e in junc.enter_roads if enters.get(e) is not None]
    single = [l for l in junc.leave_roads if l not in used_l]
    return pairs, single


def build_junction_xodr(src: IbdSource, junc: JunctionRec, out_path: str | Path,
                        *, max_len: float = 160.0) -> dict:
    """IBD 路口 → 完整 OpenDRIVE（双侧 leg road + junction 连接路 + laneLink）。"""
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: _proj(p, lat0, lon0)                # noqa: E731
    cj = proj(junc.polygon).mean(axis=0)
    doc = W.XodrDoc(f"ibd_{junc.pid[-8:]}", geo_reference=_georef(lat0, lon0))
    JID = 1

    stats = {"roads_enter": 0, "roads_leave": 0, "legs_two_way": 0, "sections": 0,
             "multi_section_roads": 0, "conn_via": 0, "conn_g2": 0, "connections": 0,
             "lanelinks": 0, "skipped": 0, "fit_dev_max": 0.0, "speeds": 0,
             "junction": junc.name, "junction_pid": junc.pid}
    rid_of, xid_of = {}, {}                              # 均以 link_pid 为键
    exit_contact = {}                                    # leave_pid → "start"|"end"
    end_pose, lane_end, lane_start = {}, {}, {}

    # ---------------------------------------------------------------- 侧向机械
    def _span_recs(pid, ref, tang, u0, u1, seed_tail):
        """一个源 Link 的车道实测：横向偏移 d + 数据首末宽 + **全程横距轮廓**
        （lg 每点投影到参考线的 (s, d) 序列——边缘贴合实测形状的原料）。"""
        i0 = min(int(u0 / 0.5), len(ref) - 2)
        i1 = min(int(u1 / 0.5) + 1, len(ref))
        rwin, twin = ref[i0:i1], tang[i0:i1]
        recs = []
        for l in [x for x in src.lanes_of(pid) if x.geometry.shape[0] >= 2]:
            lg = proj(l.geometry)
            flip = float(np.dot(lg[-1] - lg[0], rwin[-1] - rwin[0])) < 0
            if flip:
                lg = lg[::-1]
            sw = (l.s_width_mm or l.width_mm or 3500)
            ew = (l.e_width_mm or l.width_mm or 3500)
            if flip:
                sw, ew = ew, sw
            lg_m = lg[int(lg.shape[0] * 0.7):] if seed_tail else lg
            if lg_m.shape[0] < 2:
                lg_m = lg
            sub = lg_m[:: max(1, lg_m.shape[0] // 25)]
            idx = np.argmin(np.linalg.norm(rwin[None, :, :] - sub[:, None, :], axis=2), axis=1)
            d = float(np.median(twin[idx, 0] * (sub[:, 1] - rwin[idx, 1])
                                - twin[idx, 1] * (sub[:, 0] - rwin[idx, 0])))
            # 全程轮廓：逐点投影（用全参考线，防 span 窗截断）
            gi = np.argmin(np.linalg.norm(ref[None, :, :] - lg[:, None, :], axis=2), axis=1)
            dd = (tang[gi, 0] * (lg[:, 1] - ref[gi, 1])
                  - tang[gi, 1] * (lg[:, 0] - ref[gi, 0]))
            order = np.argsort(gi * 0.5)
            recs.append({"d": d, "l": l, "sw": sw / 1000.0, "ew": ew / 1000.0, "lg": lg,
                         "ps": (gi[order] * 0.5).astype(float), "pd": dd[order]})
        return recs

    def _prof_eval(rec, u, win=10.0):
        """轮廓在 s=u 处的 (值, 斜率)：±win 窗**局部线性回归**在 u 点取值——
        中位数在扇形段有一阶偏差（span 两侧窗互不重叠会撕开边界），回归无此偏差。"""
        ps, pd = rec["ps"], rec["pd"]
        m = np.abs(ps - u) <= win
        if m.sum() >= 3 and float(ps[m].max() - ps[m].min()) > 1.0:
            b, a = np.polyfit(ps[m] - u, pd[m], 1)
            return float(a), float(min(max(b, -0.25), 0.25))
        k = np.argsort(np.abs(ps - u))[:5]
        return float(np.median(pd[k])), 0.0

    def _side_specs(spans, sections):
        """逐合并 section 取本侧活动 span 的车道，宽度按 span 内数据斜坡插值到 section 端点。"""
        out = []
        for (u0, u1) in sections:
            sp = next((s for s in spans
                       if s["s0"] - 1e-3 <= u0 < s["s1"] - 1e-3 or
                       (u0 >= s["s1"] - 1e-3 and u1 <= s["s1"] + 1e-3)), None)
            if sp is None:
                out.append(None)
                continue
            lanes = []
            for r in sp["recs"]:
                a, b = sp["s0"], sp["s1"]
                f0 = min(max((u0 - a) / max(b - a, 1e-6), 0.0), 1.0)
                f1 = min(max((u1 - a) / max(b - a, 1e-6), 0.0), 1.0)
                v0, m0 = _prof_eval(r, u0)
                v1, m1 = _prof_eval(r, u1)
                lanes.append({"d": r["d"], "l": r["l"],
                              "w0": r["sw"] + (r["ew"] - r["sw"]) * f0,
                              "w1": r["sw"] + (r["ew"] - r["sw"]) * f1,
                              "v0": v0, "v1": v1, "m0": m0, "m1": m1})
            out.append({"pid": sp["pid"], "span": sp, "lanes": lanes})
        return out

    def _reconcile(specs, matches):
        """交界调和：匹配车道两侧端点的值/斜率/宽度取平均——边界机器级 C0/C1。"""
        for si, mp in enumerate(matches):
            a, b = specs[si], specs[si + 1]
            if a is None or b is None:
                continue
            for ia, ib in mp.items():
                va, vb = a["lanes"][ia], b["lanes"][ib]
                for ka, kb in (("v1", "v0"), ("m1", "m0"), ("w1", "w0")):
                    mid = (va[ka] + vb[kb]) / 2
                    va[ka] = vb[kb] = mid

    def _bounds_at(spec, e, sign):
        """一侧某端点的边界数组 (b[0..n], mb[0..n])：实测中心轮廓 → 相邻中点为界，
        外缘 = 端车道中心 ± 半数据宽。sign=-1 右侧（b 递减）、+1 左侧（b[0]=内缘）。"""
        lanes = spec["lanes"]
        v = [(ln["v0"] if e == 0 else ln["v1"]) for ln in lanes]
        mm = [(ln["m0"] if e == 0 else ln["m1"]) for ln in lanes]
        w = [(ln["w0"] if e == 0 else ln["w1"]) for ln in lanes]
        b = [v[0] - sign * w[0] / 2]
        mb = [mm[0]]
        for k in range(1, len(lanes)):
            b.append((v[k - 1] + v[k]) / 2)
            mb.append((mm[k - 1] + mm[k]) / 2)
        b.append(v[-1] + sign * w[-1] / 2)
        mb.append(mm[-1])
        return b, mb

    def _side_match(specs):
        """相邻合并 section 的车道配对：同 span 恒等，跨 span 按横向位置贪心 1:1。"""
        ms = []
        for sa, sb in zip(specs, specs[1:]):
            if sa is None or sb is None:
                ms.append({})
                continue
            if sa["span"] is sb["span"]:
                ms.append({i: i for i in range(len(sa["lanes"]))})
                continue
            pairs = sorted((abs(x["d"] - y["d"]), i, j)
                           for i, x in enumerate(sa["lanes"])
                           for j, y in enumerate(sb["lanes"]))
            ua, ub, mp = set(), set(), {}
            for gap, i, j in pairs:
                if gap >= 2.0 or i in ua or j in ub:
                    continue
                ua.add(i)
                ub.add(j)
                mp[i] = j
            ms.append(mp)
        return ms

    # ---------------------------------------------------------------- leg 主构建
    def build_leg(e_pid: str, l_pid: str | None, rid: int) -> bool:
        chain = _chained_links(src, e_pid, True, proj, cj, max_len=max_len)
        if not chain:
            return False
        pts = chain[0][1]
        for _, c in chain[1:]:
            pts = np.vstack([pts, c[1:]])
        gxy = proj(pts)
        # —— 路口侧延伸到车道实际端点（ROADCENTER 常铺不到停止线） ——
        seed_lanes = [l for l in src.lanes_of(e_pid) if l.geometry.shape[0] >= 2]
        if seed_lanes:
            dvec = gxy[-1] - gxy[-2]
            dvec = dvec / (np.linalg.norm(dvec) + 1e-12)
            deltas = []
            for l in seed_lanes:
                lg = proj(l.geometry)
                lp = lg[-1] if np.dot(lg[-1] - lg[0], dvec) > 0 else lg[0]
                deltas.append(float(np.dot(lp - gxy[-1], dvec)))
            delta = float(np.median(deltas))
            if delta > 0.2:
                gxy = np.vstack([gxy, (gxy[-1] + dvec * delta)[None, :]])
        seg_lens_all = np.linalg.norm(np.diff(gxy, axis=0), axis=1)
        chain_raw = [_polylen(proj(c)) for _, c in chain]
        seg_lens = list(chain_raw)
        seg_lens[-1] += max(0.0, float(seg_lens_all.sum()) - sum(chain_raw))
        pv, dev, smoothed = fit_leg_refline(gxy)         # 曲率封顶（双侧模型 1−tκ 护栏）
        if smoothed:
            stats["refit_smoothed"] = stats.get("refit_smoothed", 0) + 1
        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
        L_fit = sum(s.length for s in pv.segs)
        scale = L_fit / max(sum(seg_lens), 1e-6)
        e_bounds = [round(float(x) * scale, 4)
                    for x in np.concatenate([[0.0], np.cumsum(seg_lens)])]
        e_bounds[-1] = round(L_fit, 4)                   # 边界一次圆整（span 与 section 同源）
        ref = eval_planview(pv, 0.5)
        tang = np.gradient(ref, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12

        def s_of(pt):
            return 0.5 * int(np.argmin(np.linalg.norm(ref - pt[None, :], axis=1)))

        # —— 进口 span（每源 Link 一段，覆盖 [0, L]） ——
        spans_e = []
        for i, (pid, _c) in enumerate(chain):
            u0, u1 = e_bounds[i], e_bounds[i + 1]
            recs = _span_recs(pid, ref, tang, u0, u1, seed_tail=(i == len(chain) - 1))
            if not recs:
                return False
            recs.sort(key=lambda r: -r["d"])             # 右侧：左→右
            off = recs[0]["d"] + recs[0]["sw"] / 2
            spans_e.append({"pid": pid, "s0": u0, "s1": u1, "recs": recs, "off": off})

        # —— 对向 span：leave 链边界投影到参考线（吸附/去碎） ——
        # 预算对齐进口参考线长度：对向覆盖不足会在数据尽头留"全幅漏斗"（собранная
        # 车道收拢楔），能用真实数据补就不该用漏斗
        spans_l = []
        if l_pid is not None:
            chain_l = _chained_links(src, l_pid, False, proj, cj,
                                     max_len=max(max_len, L_fit + 20.0))
            for pid, c in (chain_l or []):
                cxy = proj(c)
                sa, sb = sorted((s_of(cxy[0]), s_of(cxy[-1])))
                sa, sb = max(sa, 0.0), min(sb, L_fit)
                if sb - sa < 3.0:
                    continue
                if L_fit - sb < 8.0:                     # 路口侧放宽吸附：对向幅物理到达路口，
                    sb = L_fit                           # 差值=停止线错位/投影斜量（±数米）
                for eb in e_bounds:                      # 吸附到进口边界/端点
                    if abs(sa - eb) < _SNAP:
                        sa = eb
                    if abs(sb - eb) < _SNAP:
                        sb = eb
                recs = _span_recs(pid, ref, tang, sa, sb,
                                  seed_tail=(pid == l_pid))
                recs = [r for r in recs if r["d"] > 0]   # 左侧车道必在参考线左
                if not recs:
                    continue
                recs.sort(key=lambda r: r["d"])          # 左侧：内→外（+1 靠中线）
                # 内缘两端各测（median 随 s 变化：横摆沿 span 分布而非集中在接缝）；
                # 端窗取 15%——拼链相邻 Link 端点物理同点，边界处两侧实测自然收敛
                lg = recs[0]["lg"]
                n40 = max(2, lg.shape[0] * 3 // 20)
                i0 = min(int(sa / 0.5), len(ref) - 2)
                i1 = min(int(sb / 0.5) + 1, len(ref))
                rwin, twin = ref[i0:i1], tang[i0:i1]

                def _dwin(part):
                    sub = part[:: max(1, part.shape[0] // 25)]
                    ix = np.argmin(np.linalg.norm(rwin[None, :, :] - sub[:, None, :],
                                                  axis=2), axis=1)
                    return float(np.median(twin[ix, 0] * (sub[:, 1] - rwin[ix, 1])
                                           - twin[ix, 1] * (sub[:, 0] - rwin[ix, 0])))
                spans_l.append({"pid": pid, "s0": sa, "s1": sb, "recs": recs,
                                "din0": _dwin(lg[:n40]), "din1": _dwin(lg[-n40:])})
            spans_l.sort(key=lambda s: s["s0"])
            for a, b in zip(spans_l, spans_l[1:]):       # 内部空档/重叠全部中点缝合——
                if abs(b["s0"] - a["s1"]) > 1e-9:        # 对向幅物理连续，覆盖不得开天窗
                    mid = (a["s1"] + b["s0"]) / 2
                    a["s1"] = b["s0"] = mid
            spans_l = [sp for sp in spans_l if sp["s1"] - sp["s0"] > 1.0]
            if spans_l:                                  # 两端补齐：断面保持外推（APPROXIMATED）
                far = spans_l[0]
                if far["s0"] > 1.0:
                    stats["left_extended_m"] = round(
                        stats.get("left_extended_m", 0.0) + far["s0"], 1)
                    spans_l.insert(0, {"pid": far["pid"], "s0": 0.0, "s1": far["s0"],
                                       "recs": far["recs"], "din0": far["din0"],
                                       "din1": far["din0"]})
                near = spans_l[-1]
                if near["s1"] < L_fit - 1e-6:
                    stats["left_extended_m"] = round(
                        stats.get("left_extended_m", 0.0) + (L_fit - near["s1"]), 1)
                    spans_l.append({"pid": near["pid"], "s0": near["s1"], "s1": L_fit,
                                    "recs": near["recs"], "din0": near["din1"],
                                    "din1": near["din1"]})

        # —— 合并 section 边界（leave 边界 6m 稀疏化：微 section 会把锥形压成陡坡） ——
        bounds = set(np.round(e_bounds, 4))
        for sp in spans_l:
            for v in (sp["s0"], sp["s1"]):
                if all(abs(v - b) > 6.0 for b in bounds) and 1.0 < v < L_fit - 1.0:
                    bounds.add(round(float(v), 4))
        B = sorted(bounds)
        sections = [(B[i], B[i + 1]) for i in range(len(B) - 1) if B[i + 1] - B[i] > 0.5]
        rs = _side_specs(spans_e, sections)
        lspec = _side_specs(spans_l, sections)
        m_r = _side_match(rs)
        m_l = _side_match(lspec)
        _reconcile(rs, m_r)
        _reconcile(lspec, m_l)
        two_way = any(x is not None for x in lspec)

        # —— 边界轮廓（实测跟踪）：每 section 端点各车道实测横距 → 相邻中点为界 ——
        # 边缘/车道线由此贴合 SHP 实测形状（此前为标量宽度堆叠的合成边缘）
        rbounds = [(_bounds_at(sp, 0, -1), _bounds_at(sp, 1, -1)) if sp else None
                   for sp in rs]
        lbounds = [(_bounds_at(sp, 0, +1), _bounds_at(sp, 1, +1)) if sp else None
                   for sp in lspec]
        off_vals = []                                    # laneOffset 每 section (y0,m0,y1,m1)
        for rb in rbounds:
            (b0, mb0), (b1, mb1) = rb
            off_vals.append((b0[0], mb0[0], b1[0], mb1[0]))
        for i in range(len(off_vals) - 1):               # 交界硬连续（值取上段末、斜率平均）
            y0a, m0a, y1a, m1a = off_vals[i]
            y0b, m0b, y1b, m1b = off_vals[i + 1]
            mm = (m1a + m0b) / 2
            off_vals[i] = (y0a, m0a, y1a, mm)
            off_vals[i + 1] = (y1a, mm, y1b, m1b)

        # —— 中央分隔（median）：对向内缘实测 − laneOffset（同源 Hermite，全程 C1） ——
        med_w = []                                       # 每 section: (g0, m0, g1, m1)|None
        for i, lb in enumerate(lbounds):
            if lb is None:
                med_w.append(None)
                continue
            (bl0, mbl0), (bl1, mbl1) = lb
            y0, my0, y1, my1 = off_vals[i]
            g0, mg0 = bl0[0] - y0, mbl0[0] - my0
            g1, mg1 = bl1[0] - y1, mbl1[0] - my1
            if g0 < 0.0:
                g0, mg0 = 0.0, 0.0
            if g1 < 0.0:
                g1, mg1 = 0.0, 0.0
            med_w.append((g0, mg0, g1, mg1))
        for i in range(len(med_w) - 1):                  # 交界硬连续
            if med_w[i] is None or med_w[i + 1] is None:
                continue
            g0a, mg0a, g1a, mg1a = med_w[i]
            g0b, mg0b, g1b, mg1b = med_w[i + 1]
            mm = (mg1a + mg0b) / 2
            med_w[i] = (g0a, mg0a, g1a, mm)
            med_w[i + 1] = (g1a, mm, g1b, mg1b)
        has_median = any(g is not None and max(g[0], g[2]) > 0.05 for g in med_w)
        med_off = 1 if has_median else 0

        # —— 逐 section 生成车道对象（宽度 = 实测边界差 Hermite；生灭仍锥形收放；
        #    交界硬连续：续接车道起宽/起斜率 := 上段 written 末值，Σ堆叠零台阶） ——
        def _emit(side_specs, side_match, bounds, sign):
            all_objs, all_xid, all_endw = [], [], []
            prev_w, prev_m = None, None
            for si, spec in enumerate(side_specs):
                objs, xid, end_w, end_m = [], {}, [], []
                if spec is not None:
                    L_sec = sections[si][1] - sections[si][0]
                    lanes = spec["lanes"]
                    (b0a, mb0a), (b1a, mb1a) = bounds[si]
                    for k, ln_rec in enumerate(lanes):
                        dying = si < len(side_match) and k not in side_match[si]
                        born = si > 0 and (side_specs[si - 1] is None or
                                           k not in side_match[si - 1].values())
                        if sign < 0:
                            w0, w1 = b0a[k] - b0a[k + 1], b1a[k] - b1a[k + 1]
                            mw0, mw1 = mb0a[k] - mb0a[k + 1], mb1a[k] - mb1a[k + 1]
                        else:
                            w0, w1 = b0a[k + 1] - b0a[k], b1a[k + 1] - b1a[k]
                            mw0, mw1 = mb0a[k + 1] - mb0a[k], mb1a[k + 1] - mb1a[k]
                        if w0 < 0.4 or w1 < 0.4:         # 轮廓交叉/噪声：退数据标量宽
                            w0, w1 = ln_rec["w0"], ln_rec["w1"]
                            mw0 = mw1 = None
                            stats["profile_fallback"] = stats.get("profile_fallback", 0) + 1
                        if si > 0 and prev_w is not None and not born:
                            srcs = [i2 for i2, j2 in side_match[si - 1].items() if j2 == k]
                            if srcs and srcs[0] < len(prev_w):
                                w0 = prev_w[srcs[0]]     # 交界硬连续
                                if mw0 is not None:
                                    mw0 = prev_m[srcs[0]]
                        lane_id = sign * (k + 1 + (med_off if sign > 0 else 0))
                        ln = W.Lane(lane_id)
                        if born or dying:
                            for so, a, b, c, dd in _width_pieces(w0, w1, L_sec, born, dying):
                                ln.add_width(a, b, c, dd, s_offset=so)
                            end_w.append(0.0 if dying else w1)
                            end_m.append(0.0)
                            stats["tapers"] = stats.get("tapers", 0) + 1
                        elif mw0 is None:
                            ln.add_width(*_smooth_w(w0, w1, L_sec))
                            end_w.append(w1)
                            end_m.append(0.0)
                        else:                            # Hermite：值+斜率贴实测轮廓
                            mw0, mw1 = _fc(w0, mw0, w1, mw1, L_sec)
                            c = (3 * (w1 - w0) - (2 * mw0 + mw1) * L_sec) / L_sec ** 2
                            d = (-2 * (w1 - w0) + (mw0 + mw1) * L_sec) / L_sec ** 3
                            ln.add_width(w0, mw0, c, d)
                            end_w.append(w1)
                            end_m.append(mw1)
                        ln.mark = std_mark("outer" if k == len(lanes) - 1 else "inner")
                        if ln_rec["l"].max_speed_kmh:
                            ln.speed_ms = float(ln_rec["l"].max_speed_kmh) / 3.6
                            stats["speeds"] += 1
                        objs.append(ln)
                        xid[ln_rec["l"].seq] = lane_id
                all_objs.append(objs)
                all_xid.append(xid)
                all_endw.append(end_w)
                prev_w = end_w if spec is not None else None
                prev_m = end_m if spec is not None else None
            return all_objs, all_xid, all_endw

        r_objs, r_xid, r_endw = _emit(rs, m_r, rbounds, -1)
        l_objs, l_xid, l_endw = _emit(lspec, m_l, lbounds, +1)
        # 跨 section 车道衔接
        for si, mp in enumerate(m_r):
            for ia, ib in mp.items():
                if ia < len(r_objs[si]) and ib < len(r_objs[si + 1]):
                    r_objs[si][ia].succ = -(ib + 1)
                    r_objs[si + 1][ib].pred = -(ia + 1)
        for si, mp in enumerate(m_l):
            for ia, ib in mp.items():
                if ia < len(l_objs[si]) and ib < len(l_objs[si + 1]):
                    l_objs[si][ia].succ = ib + 1 + med_off
                    l_objs[si + 1][ib].pred = ia + 1 + med_off

        rl = src.roadlinks.get(e_pid)
        road = W.Road(rid, name=(rl.name if rl else None))
        prims, ep = planview_prims(pv)
        for p in prims:
            road.add_geometry(*p)
        for i, (u0, u1) in enumerate(sections):
            y0, my0, y1, my1 = off_vals[i]               # Hermite：贴实测左缘
            Ls = max(u1 - u0, 1e-3)
            my0, my1 = _fc(y0, my0, y1, my1, Ls)
            c = (3 * (y1 - y0) - (2 * my0 + my1) * Ls) / Ls ** 2
            d = (-2 * (y1 - y0) + (my0 + my1) * Ls) / Ls ** 3
            road.add_offset(u0, y0, my0, c, d)
        med_end_last = 0.0
        for si, (u0, u1) in enumerate(sections):
            sec = W.LaneSection(u0, center_mark=std_mark("center2" if two_way else "center"))
            if lspec[si] is not None and has_median:     # median 车道恒 +1（宽可为 0）
                g0, m0, g1, m1 = med_w[si]
                Ls = max(u1 - u0, 1e-3)
                m0, m1 = _fc(g0, m0, g1, m1, Ls)
                c = (3 * (g1 - g0) - (2 * m0 + m1) * Ls) / Ls ** 2
                d = (-2 * (g1 - g0) + (m0 + m1) * Ls) / Ls ** 3
                mln = W.Lane(1, "median")
                mln.add_width(g0, m0, c, d)              # Hermite：值+斜率双侧衔接（全程 C1）
                if si == len(sections) - 1:
                    med_end_last = g1                    # 路口端中隔宽（堆叠用）
                if si > 0 and lspec[si - 1] is not None:
                    mln.pred = 1
                if si < len(sections) - 1 and lspec[si + 1] is not None:
                    mln.succ = 1
                sec.left.append(mln)
            sec.left.extend(l_objs[si])
            sec.right.extend(r_objs[si])
            road.sections.append(sec)
            stats["sections"] += 1
        if len(sections) > 1:
            stats["multi_section_roads"] += 1
        road.add_link("successor", "junction", JID)
        doc.add_road(road)

        # —— 路口端车道位姿（文件语义堆叠：laneOffset ± written 宽度累计） ——
        last = len(sections) - 1
        off_end = off_vals[-1][2]
        cum = 0.0
        for k, ln_rec in enumerate(rs[last]["lanes"] if rs[last] else []):
            w = r_endw[last][k]
            t = off_end - cum - w / 2
            lane_end[ln_rec["l"].lane_pid] = _shift(ep, t)
            cum += w
        rid_of[e_pid], xid_of[e_pid] = rid, r_xid[last]
        end_pose[e_pid] = ep
        stats["roads_enter"] += 1
        if l_pid is not None and lspec[last] is not None:
            cum = med_end_last if has_median else 0.0
            for k, ln_rec in enumerate(lspec[last]["lanes"]):
                w = l_endw[last][k]
                t = off_end + cum + w / 2
                p = _shift(ep, t)
                lane_start[(l_pid, ln_rec["l"].seq)] = (p[0], p[1], ep[2] + math.pi)
                cum += w
            rid_of[l_pid], xid_of[l_pid] = rid, l_xid[last]
            exit_contact[l_pid] = "end"
            stats["roads_leave"] += 1
            stats["legs_two_way"] += 1
        return True

    # ---------------------------------------------------------------- 单侧出口回退
    def build_single_leave(lpid: str, rid: int) -> bool:
        chain = _chained_links(src, lpid, False, proj, cj, max_len=max_len)
        if not chain:
            return False
        pts = chain[0][1]
        for _, c in chain[1:]:
            pts = np.vstack([pts, c[1:]])
        gxy = proj(pts)
        pv, dev, smoothed = fit_leg_refline(gxy)
        if smoothed:
            stats["refit_smoothed"] = stats.get("refit_smoothed", 0) + 1
        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
        L_fit = sum(s.length for s in pv.segs)
        ref = eval_planview(pv, 0.5)
        tang = np.gradient(ref, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        recs = _span_recs(lpid, ref, tang, 0.0, min(40.0, L_fit), seed_tail=False)
        if not recs:
            return False
        recs.sort(key=lambda r: -r["d"])
        off = recs[0]["d"] + recs[0]["sw"] / 2
        sec = W.LaneSection(0.0, center_mark=std_mark("center"))
        xid, cum = {}, 0.0
        for k, r in enumerate(recs):
            ln = W.Lane(-(k + 1))
            ln.add_width(*_smooth_w(r["sw"], r["ew"], L_fit))
            ln.mark = std_mark("outer" if k == len(recs) - 1 else "inner")
            if r["l"].max_speed_kmh:
                ln.speed_ms = float(r["l"].max_speed_kmh) / 3.6
                stats["speeds"] += 1
            sec.right.append(ln)
            xid[r["l"].seq] = -(k + 1)
            t = off - cum - r["sw"] / 2
            p = _shift((pv.x0, pv.y0, pv.hdg), t)
            lane_start[(lpid, r["l"].seq)] = (p[0], p[1], pv.hdg)
            cum += r["sw"]
        rl = src.roadlinks.get(lpid)
        road = W.Road(rid, name=(rl.name if rl else None))
        prims, _ep = planview_prims(pv)
        for p in prims:
            road.add_geometry(*p)
        road.add_offset(0.0, off)
        road.sections.append(sec)
        road.add_link("predecessor", "junction", JID)
        doc.add_road(road)
        rid_of[lpid], xid_of[lpid] = rid, xid
        exit_contact[lpid] = "start"
        stats["roads_leave"] += 1
        stats["sections"] += 1
        return True

    pairs, single_leaves = _pair_legs(src, junc, proj, cj)
    for i, (e_pid, l_pid) in enumerate(pairs):
        build_leg(e_pid, l_pid, 10 + i)
        if l_pid is not None and l_pid not in rid_of:    # 腿建成但对向没挂上：单侧回退
            single_leaves.append(l_pid)
    for j, lpid in enumerate(single_leaves):
        build_single_leave(lpid, 30 + j)

    # —— junction 连接路：TOPO 两跳（进口车道→路口内车道→出口车道）——
    enter_set = [p for p in junc.enter_roads if p in rid_of]
    leave_set = {p for p in junc.leave_roads if p in rid_of}
    junction = W.Junction(JID, f"junc_{junc.pid[-8:]}")
    conn_roads, conn_objs = {}, {}
    rid_c = 100

    def connecting_road(key, prims, width_m, e_pid, x_pid, in_xid, out_xid):
        nonlocal rid_c
        if key in conn_roads:
            rid0, owner = conn_roads[key]
            if owner == e_pid:                           # 同进口路复用：仅补 laneLink
                conn_objs[key].add_lanelink(in_xid, -1)
                stats["lanelinks"] += 1
            else:
                stats["skipped"] += 1
            return
        road = W.Road(rid_c, junction=JID)
        for p in prims:
            road.add_geometry(*p)
        road.add_offset(0.0, width_m / 2)
        sec = W.LaneSection(0.0)
        ln = W.Lane(-1)
        ln.add_width(width_m)
        ln.pred, ln.succ = in_xid, out_xid
        sec.right.append(ln)
        road.sections.append(sec)
        road.add_link("predecessor", "road", rid_of[e_pid], "end")
        road.add_link("successor", "road", rid_of[x_pid],
                      exit_contact.get(x_pid, "start"))
        doc.add_road(road)
        conn = W.Connection(rid_of[e_pid], rid_c, "start")
        conn.add_lanelink(in_xid, -1)
        junction.connections.append(conn)
        conn_roads[key], conn_objs[key] = (rid_c, e_pid), conn
        stats["connections"] += 1
        stats["lanelinks"] += 1
        rid_c += 1

    for e_pid in enter_set:
        for l in [x for x in src.lanes_of(e_pid) if x.geometry.shape[0] >= 2]:
            in_xid = xid_of[e_pid].get(l.seq)
            if in_xid is None:
                continue
            for out1 in src.topo_out.get(l.lane_pid, []):
                mid = src.lane(out1)
                if mid is None:
                    stats["skipped"] += 1
                    continue
                if mid.link_pid in leave_set:            # TOPO 直连（无路口内几何）→ G2 合成
                    targets = [(mid, None)]
                else:
                    targets = [(src.lane(o2), mid) for o2 in src.topo_out.get(out1, [])
                               if src.lane(o2) is not None
                               and src.lane(o2).link_pid in leave_set]
                for out_rec, via in targets:
                    out_xid = xid_of[out_rec.link_pid].get(out_rec.seq)
                    if out_xid is None:
                        stats["skipped"] += 1
                        continue
                    if via is not None and via.geometry.shape[0] >= 3:
                        vg = proj(via.geometry)
                        a = np.asarray(lane_end.get(l.lane_pid, end_pose[e_pid])[:2])
                        if np.linalg.norm(vg[-1] - a) < np.linalg.norm(vg[0] - a):
                            vg = vg[::-1]                # 起点靠进口侧
                        p0 = lane_end.get(l.lane_pid)
                        p1 = lane_start.get((out_rec.link_pid, out_rec.seq))
                        br = None
                        # ① 最少段优先：整条转弯若能用一条 G2 回旋链（3 段）表达且
                        #    对实测中心线偏差 ≤0.5m，就用它——路口内 40m 转弯用 3 段
                        #    远优于"桥+中段"的 10+ 段（消费端读到的曲率更干净）
                        if p0 is not None and p1 is not None:
                            g2 = _g2_prims(p0, p1)
                            if g2 is not None and _prims_dev(g2, vg) <= 0.5:
                                br = (g2, _prims_dev(g2, vg))
                                stats["conn_g2_direct"] = stats.get("conn_g2_direct", 0) + 1
                        # ② 否则桥+实测中段（切口放大阶梯避猪尾巴病态解）
                        if br is None and p0 is not None and p1 is not None:
                            for sc in (1.0, 2.0, 3.0):
                                br = _bridged_geoms(vg, np.asarray(p0), np.asarray(p1),
                                                    lambda m: _fit_ref(m, resample_step=1.0,
                                                                       min_seg_len=4.0),
                                                    scale=sc)
                                if br is not None:
                                    break
                        if br is not None:               # 两端 G2 桥接（换乘零跳变）
                            prims, dev = br
                            stats["conn_bridged"] = stats.get("conn_bridged", 0) + 1
                        else:
                            prims, dev = None, 0.0
                            if p0 is not None and p1 is not None:
                                from pyclothoids import SolveG2
                                try:                     # 次级：纯 G2 合成（端点仍精确）
                                    cls = SolveG2(p0[0], p0[1], p0[2], 0.0,
                                                  p1[0], p1[1], p1[2], 0.0)
                                    if all(max(abs(c.KappaStart), abs(c.KappaEnd)) <= 0.5
                                           for c in cls):
                                        prims = [("spiral", c.XStart, c.YStart,
                                                  c.ThetaStart, c.length,
                                                  c.KappaStart, c.KappaEnd) for c in cls]
                                        stats["conn_g2_synth"] = \
                                            stats.get("conn_g2_synth", 0) + 1
                                except Exception:
                                    prims = None
                            if prims is None:            # 最后：原始拟合（弃端点精确）
                                pv, dev = _fit_ref(vg, resample_step=1.0, min_seg_len=4.0)
                                got2 = simplify_planview(pv, vg, dev_tol=max(0.35, dev * 1.5),
                                                         min_seg_len=8.0)
                                pv = weld_g2(got2[0] if got2 else g2ify_planview(pv)[0])
                                prims, _ = planview_prims(pv)
                        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
                        connecting_road(("via", via.lane_pid), prims,
                                        (via.width_mm or 3500) / 1000.0,
                                        e_pid, out_rec.link_pid, in_xid, out_xid)
                        stats["conn_via"] += 1
                    else:                                # 直连：车道端位姿 SolveG2
                        p0 = lane_end.get(l.lane_pid)
                        p1 = lane_start.get((out_rec.link_pid, out_rec.seq))
                        if p0 is None or p1 is None:
                            stats["skipped"] += 1
                            continue
                        from pyclothoids import SolveG2
                        try:
                            cls = SolveG2(p0[0], p0[1], p0[2], 0.0, p1[0], p1[1], p1[2], 0.0)
                        except Exception:
                            stats["skipped"] += 1
                            continue
                        prims = [("spiral", c.XStart, c.YStart, c.ThetaStart,
                                  c.length, c.KappaStart, c.KappaEnd) for c in cls]
                        connecting_road(("g2", l.lane_pid, out_rec.lane_pid), prims,
                                        (out_rec.width_mm or 3500) / 1000.0,
                                        e_pid, out_rec.link_pid, in_xid, out_xid)
                        stats["conn_g2"] += 1

    # —— junction 铺面：IBD 交叉口面实测轮廓（type=none 无标线沥青面，不入拓扑） ——
    from mapforge.adapters.opendrive.writer import add_paving_road
    try:
        if add_paving_road(doc, proj(junc.polygon), JID):
            stats["paving"] = "polygon"
    except Exception:
        stats["paving"] = "skip"

    doc.add_junction(junction)
    doc.write(out_path)
    return stats
