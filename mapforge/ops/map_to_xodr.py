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
from mapforge.ops.refline_fit import (eval_planview, fc_clamp, fit_leg_refline,
                                      planview_prims, seg_kappa)

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


def _lane_kappa(k_ref: float, t: float) -> float:
    """参考线曲率 → 横向偏移 t 处车道中心曲率：κ/(1−tκ)。"""
    den = 1.0 - t * k_ref
    return k_ref / den if abs(den) > 1e-6 else k_ref


def _shift(pose, t):
    x, y, h = pose
    return (x - t * math.sin(h), y + t * math.cos(h), h)


def _lane_profile(ref, tang, lp, max_snap=30.0):
    """车道点列 → 沿参考线的 (s, d) 实测轮廓（超出参考线覆盖的点剔除）。
    抽稀点列逐点投影即可——MAP 附录 D 抽稀后每条 Link 车道通常十几到几十点。"""
    if float(np.dot(lp[-1] - lp[0], ref[-1] - ref[0])) < 0:
        lp = lp[::-1]
    dist = np.linalg.norm(ref[None, :, :] - lp[:, None, :], axis=2)
    idx = np.argmin(dist, axis=1)
    keep = dist[np.arange(len(lp)), idx] < max_snap
    if keep.sum() < 2:
        return None
    lp2, idx2 = lp[keep], idx[keep]
    s = (idx2 * 0.5).astype(float)
    d = (tang[idx2, 0] * (lp2[:, 1] - ref[idx2, 1])
         - tang[idx2, 1] * (lp2[:, 0] - ref[idx2, 0]))
    order = np.argsort(s)
    return s[order], d[order]


def _prof_eval(prof, u, win=15.0):
    """轮廓在 s=u 处的 (值, 斜率)：±win 窗局部线性回归；窗内点不足取最近点保持。"""
    ps, pd = prof
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
    ba = [vs[0] - sign * ws[0] / 2]
    ma = [ms[0]]
    for i in range(1, len(act)):
        ba.append((vs[i - 1] + vs[i]) / 2)
        ma.append((ms[i - 1] + ms[i]) / 2)
    ba.append(vs[-1] + sign * ws[-1] / 2)
    ma.append(ms[-1])
    if any(sign * (ba[i + 1] - ba[i]) < 0.4 for i in range(len(act))):
        ba = [ba[0]]
        for w in ws:
            ba.append(ba[-1] + sign * w)
        ma = [ma[0]] * (len(act) + 1)
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


def _emit_lanes(sec, lanes, B, sign, j, n_sec, Ls, med_off, stats):
    """一侧一段的车道对象：宽度 = 相邻实测边界差的 Hermite（FC 限幅）。"""
    for k, x in enumerate(lanes):
        w0 = sign * (B[j][0][k + 1] - B[j][0][k])
        w1 = sign * (B[j + 1][0][k + 1] - B[j + 1][0][k])
        mw0 = sign * (B[j][1][k + 1] - B[j][1][k])
        mw1 = sign * (B[j + 1][1][k + 1] - B[j + 1][1][k])
        mw0, mw1 = fc_clamp(w0, mw0, w1, mw1, Ls)
        lane_id = sign * (k + 1 + (med_off if sign > 0 else 0))
        lane = W.Lane(lane_id)
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
               neighbors: list[MapNode] | None = None) -> dict:
    """MapNode → .xodr（双向 leg road）。neighbors：同帧其他节点（真实出口数据）。"""
    lat0, lon0 = node.ref_lat, node.ref_lon
    proj = lambda p: _project(p, lat0, lon0)                 # noqa: E731
    doc = W.XodrDoc(f"node{node.node_id}", geo_reference=_georef(lat0, lon0))
    JID = 1
    stats = {"links": 0, "exit_roads": 0, "exit_real": 0, "exit_mirror": 0,
             "conn_roads": 0, "connections": 0, "lanelinks": 0, "skipped": 0,
             "segs": 0, "fit_dev_max": 0.0}

    links = [lk for lk in node.links if len(lk.points) >= 2]
    lk_by_name = {lk.name: lk for lk in links}

    # —— 出口需求归结（remote node id → 目标车道数） ——
    by_up, by_up_id = {}, {}
    for lk in links:
        reg, nid = (lk.upstream or (None, None))
        if nid is not None:
            by_up[(reg, nid)] = lk.name
            by_up_id.setdefault(nid, lk.name)
    need = {}
    for lk in links:
        for ln in lk.lanes:
            for c in ln.connects:
                if c.node is not None:
                    need[c.node] = max(need.get(c.node, 1), c.lane or 1)

    def _real_exit_link(remote_nid):
        for nb in (neighbors or []):
            if nb.node_id == remote_nid:
                cands = [lk for lk in nb.links
                         if (lk.upstream or (None, None))[1] == node.node_id
                         and len(lk.points) >= 2]
                exact = [lk for lk in cands if lk.upstream[0] == node.region]
                if exact or cands:
                    return (exact or cands)[0]
        return None

    # 出口 → 所属 leg（该出口通往的节点 = 某进口 Link 的上游 ⇒ 同一条街）
    exit_of_leg = {}                                         # leg name → remote nid
    for nid in need:
        dname = by_up.get((node.region, nid)) or by_up_id.get(nid)
        if dname is not None:
            exit_of_leg[dname] = nid

    in_info, exits = {}, {}
    # —— 逐 leg：进口右侧 + 出口左侧（实测轮廓跟踪 + 站点网格多 laneSection） ——
    for i, lk in enumerate(links):
        rid = 10 + i
        pv, dev, smoothed = fit_leg_refline(proj(lk.points))
        if smoothed:
            stats["refit_smoothed"] = stats.get("refit_smoothed", 0) + 1
        stats["fit_dev_max"] = max(stats["fit_dev_max"], dev)
        prims, ep = planview_prims(pv)
        stats["segs"] += len(prims)
        L_leg = sum(p[4] for p in prims)
        ref = eval_planview(pv, 0.5)
        tang = np.gradient(ref, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        # 站点网格（≈30m）：相邻 laneSection 共享站点边界 ⇒ 断面天然连续，无需事后调和
        n_sec = max(1, int(round(L_leg / 30.0)))
        stations = [L_leg * j / n_sec for j in range(n_sec + 1)]

        # —— 右侧：进口车道实测轮廓（任一条缺点列则整幅回退居中堆叠常量） ——
        rlanes = []
        for ln in lk.lanes:
            prof = _lane_profile(ref, tang, proj(ln.points)) if len(ln.points) >= 2 else None
            x = {"ln": ln, "prof": prof, "w": (ln.width_cm or 350) / 100.0,
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
        Br = [_bounds(rlanes, u, -1) for u in stations]

        # —— 左侧：本 leg 的出口（real 实测轮廓 / real 无点列堆叠 / mirror 镜像） ——
        nid = exit_of_leg.get(lk.name)
        llanes, Bl, med, has_med, kind = [], None, None, False, None
        if nid is not None:
            real = _real_exit_link(nid)
            lane_ws = [x.width_cm for x in lk.lanes if x.width_cm]
            def_w = (sorted(lane_ws)[len(lane_ws) // 2] / 100.0) if lane_ws else 3.5
            if real is not None:
                for ln in real.lanes:
                    if len(ln.points) < 2:
                        continue
                    prof = _lane_profile(ref, tang, proj(ln.points))
                    if prof is None or float(np.median(prof[1])) <= 0:
                        continue                          # 左侧车道必在参考线左
                    lo, hi = float(prof[0].min()), float(prof[0].max())
                    llanes.append({"prof": prof, "lid": ln.lane_id,
                                   "w": (ln.width_cm or 350) / 100.0,
                                   "kmh": _lane_speed_kmh(ln),
                                   # 出口两端均按数据覆盖（远端才加出的车道不得外推进口部）
                                   "cov": (-1e18 if lo < 20.0 else lo - 10.0,
                                           1e18 if hi > L_leg - 20.0 else hi + 10.0)})
                llanes.sort(key=lambda x: _dv(x, L_leg)[0])   # 内→外
                if not llanes:                            # 无逐车道点列：贴进口幅堆叠
                    cum = 0.0                             # （车道数/宽/限速仍为真实数据）
                    for ln in real.lanes:
                        w = (ln.width_cm or 350) / 100.0
                        llanes.append({"prof": None, "rel": cum + w / 2, "w": w,
                                       "kmh": _lane_speed_kmh(ln)})
                        cum += w
                kind = "real"
            else:                                         # 单节点帧：镜像兜底（INFERRED）
                llanes = [{"prof": None, "rel": def_w * (k + 0.5), "w": def_w,
                           "kmh": None} for k in range(need[nid])]
                kind = "mirror"
        if llanes:
            Bl = [_bounds(llanes, stations[j], +1,
                          base=(Br[j][0][0], Br[j][1][0])) for j in range(len(stations))]
            med = []
            for j in range(len(stations)):
                g = Bl[j][0][0] - Br[j][0][0]
                mg = Bl[j][1][0] - Br[j][1][0]
                med.append((g, mg) if g > 0 else (0.0, 0.0))
            has_med = any(g > 0.05 for g, _m in med)
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
            my0, my1 = fc_clamp(y0, my0, y1, my1, Ls)
            road.add_offset(u0, y0, my0,
                            (3 * (y1 - y0) - (2 * my0 + my1) * Ls) / Ls ** 2,
                            (-2 * (y1 - y0) + (my0 + my1) * Ls) / Ls ** 3)
            if has_med:                                   # median 恒 +1（宽随 s 变化）
                g0, mg0 = med[j]
                g1, mg1 = med[j + 1]
                mg0, mg1 = fc_clamp(g0, mg0, g1, mg1, Ls)
                mlane = W.Lane(1, "median")
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
                            "kappa": seg_kappa(pv, at_end=True)}
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
            exits[nid] = {"rid": rid, "xid": exid, "t": etmap, "pose": ep,
                          "kappa": seg_kappa(pv, at_end=True), "n": len(llanes),
                          "kind": kind, "contact": "end"}
            stats["exit_real" if kind == "real" else "exit_mirror"] += 1
            stats["exit_roads"] += 1

    # —— 连接路：模型端部位姿 + 车道级曲率 G2 ——
    junction = W.Junction(JID, f"node{node.node_id}")
    rid_c = 100
    pave_pts = []
    from pyclothoids import SolveG2
    for lk in links:
        ii = in_info[lk.name]
        for ln in lk.lanes:
            for c in ln.connects:
                e = exits.get(c.node)
                if e is None or ln.lane_id not in ii["t"]:
                    stats["skipped"] += 1
                    continue
                t0 = ii["t"][ln.lane_id]
                p0 = _shift(ii["pose"], t0)
                k0 = _lane_kappa(ii["kappa"], t0)
                kk = c.lane or 1                         # 目标车道号（键=出口 Link 车道号）
                if kk not in e["t"]:                     # 号不在册：退最近可用号（不发明拓扑）
                    kk = min(e["t"], key=lambda j: abs(j - kk))
                t1 = e["t"][kk]
                q = _shift(e["pose"], t1)
                p1 = (q[0], q[1], e["pose"][2] + math.pi)    # 左侧出口：行车逆 s
                k1 = -_lane_kappa(e["kappa"], t1)
                try:
                    cls = SolveG2(p0[0], p0[1], p0[2], k0, p1[0], p1[1], p1[2], k1)
                except Exception:
                    stats["skipped"] += 1
                    continue
                w = (ln.width_cm or 350) / 100.0
                road = W.Road(rid_c, junction=JID)
                for cl in cls:
                    road.add_geometry("spiral", cl.XStart, cl.YStart, cl.ThetaStart,
                                      cl.length, cl.KappaStart, cl.KappaEnd)
                road.add_offset(0.0, w / 2)
                csec = W.LaneSection(0.0)
                clane = W.Lane(-1)
                clane.add_width(w)
                clane.pred = ii["xid"][ln.lane_id]
                clane.succ = e["xid"].get(kk, kk)
                csec.right.append(clane)
                road.sections.append(csec)
                road.add_link("predecessor", "road", ii["rid"], "end")
                road.add_link("successor", "road", e["rid"], e["contact"])
                doc.add_road(road)
                pave_pts += [(g[1], g[2]) for g in road.geoms]
                pave_pts.append(road.end_pose()[:2])
                conn = W.Connection(ii["rid"], rid_c, "start")
                conn.add_lanelink(ii["xid"][ln.lane_id], -1)
                junction.connections.append(conn)
                rid_c += 1
                stats["conn_roads"] += 1
                stats["connections"] += 1
                stats["lanelinks"] += 1

    # —— junction 铺面：MAP 无面数据 → 连接路几何凸包 +2.5m（INFERRED，type=none） ——
    if pave_pts:
        try:
            from shapely.geometry import MultiPoint
            from mapforge.adapters.opendrive.writer import add_paving_road
            hull = MultiPoint(pave_pts).convex_hull.buffer(2.5)
            if add_paving_road(doc, np.asarray(hull.exterior.coords), JID):
                stats["paving"] = "hull"
        except Exception:
            stats["paving"] = "skip"

    doc.add_junction(junction)
    doc.write(out_path)
    return stats
