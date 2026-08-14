# -*- coding: utf-8 -*-
"""esmini RoadManager 独立验证（第三方消费端实证，不是我们自己的审计器自说自话）。

两层检查：
1) 换乘缝隙（确定性全覆盖）：对文件里每条 junction 连接，让 esmini 自己求
   进口车道末端 / 连接路起终点 / 出口车道起点的世界坐标，逐对量缝隙——
   esmini 的车道求值器是独立实现，能双盲验证 laneOffset/width/laneLink 语义；
2) 随机行驶冒烟：从每条进口车道 1m 步进穿路口。esmini 的 junctionSelector 会
   "借道"（随机挑中别的车道的连接路，横向跳一个车道宽入场）——那是选路策略
   不是地图缺陷，故只对 from 车道匹配的连接计跳变。

签名对齐 esmini v3.6.0（id_t=uint32、double 参数、出参取车道 id——旧版 float
签名喂进去是垃圾值，还会让 DLL 段错误）。
用法：.venv/Scripts/python scripts/esmini_rm_check.py out/direct_xodr/node4.xodr [...]
"""
import ctypes
import math
import sys
import xml.etree.ElementTree as ET
from ctypes import POINTER, byref, c_bool, c_char_p, c_double, c_int, c_uint32
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DLL = ROOT / "esmini" / "bin" / "esminiRMLib.dll"


class RMPos(ctypes.Structure):
    _fields_ = [("x", c_double), ("y", c_double), ("z", c_double),
                ("h", c_double), ("p", c_double), ("r", c_double),
                ("hRelative", c_double), ("roadId", c_uint32),
                ("junctionId", c_uint32), ("laneId", c_int),
                ("laneOffset", c_double), ("s", c_double)]


def _bind(rm):
    rm.RM_Init.argtypes = [c_char_p]
    rm.RM_SetLanePosition.argtypes = [c_int, c_uint32, c_int, c_double, c_double, c_bool]
    rm.RM_PositionMoveForward.argtypes = [c_int, c_double, c_double]
    rm.RM_GetPositionData.argtypes = [c_int, POINTER(RMPos)]
    rm.RM_GetIdOfRoadFromIndex.argtypes = [c_uint32]
    rm.RM_GetIdOfRoadFromIndex.restype = c_uint32
    rm.RM_GetRoadLength.argtypes = [c_uint32]
    rm.RM_GetRoadLength.restype = c_double
    rm.RM_GetRoadNumberOfDrivableLanes.argtypes = [c_uint32, c_double]
    rm.RM_GetDrivableLaneIdByIndex.argtypes = [c_uint32, c_int, c_double, POINTER(c_int)]
    rm.RM_GetLaneWidthByRoadId.argtypes = [c_uint32, c_int, c_double, POINTER(c_double)]


def _start_s(rm, road, lane, L, step=5.0, need=1.0):
    """该车道真正存在（宽 ≥need）的最小 s——口部展宽车道在远端零宽，
    从零宽处起步会被 esmini 归到邻道，量出的是探针起点错误而非地图缺陷。"""
    w = c_double(0.0)
    s = 1.0
    while s < L:
        if rm.RM_GetLaneWidthByRoadId(road, lane, c_double(s), byref(w)) == 0 and w.value >= need:
            return s
        s += step
    return None


def _parse_connections(path):
    """[{inc, from, cid, exit, to}]：连接路的车道级换乘对（file 语义）。"""
    root = ET.parse(path).getroot()
    roads = {r.get("id"): r for r in root.findall("road")}
    out = []
    for c in root.findall("junction/connection"):
        cr = roads.get(c.get("connectingRoad"))
        if cr is None:
            continue
        succ = cr.find("link/successor")
        lk = cr.find("lanes/laneSection/right/lane/link/successor")
        for ll in c.findall("laneLink"):
            out.append({"inc": int(c.get("incomingRoad")), "from": int(ll.get("from")),
                        "cid": int(cr.get("id")),
                        "exit": int(succ.get("elementId")) if succ is not None else None,
                        "exit_end": (succ is not None and
                                     succ.get("contactPoint", "start") == "end"),
                        "to": int(lk.get("id")) if lk is not None else -1})
    return out


def _parse_merge_ends(path):
    """车道并线点 {(road_id, boundary_s_round, lane_id)}：
    右侧无后继（沿 s 行驶收拢归零）与左侧无前驱（逆 s 行驶到出生点）的车道端——
    数据本义：车道终止，车辆应并线。RM 的朴素探针不会并线，会按同 ID 映射到
    新断面横移——那是消费策略不是地图缺陷。"""
    root = ET.parse(path).getroot()
    out = set()                                          # (rid, lo, hi, lane_id)：锥形带
    T = 22.0
    for rd in root.findall("road"):
        rid = int(rd.get("id"))
        secs = rd.findall("lanes/laneSection")
        for sec, nxt in zip(secs, secs[1:]):
            b = float(nxt.get("s"))
            for ln in sec.findall("right/lane"):         # 右侧灭：边界前收拢
                if ln.find("link/successor") is None:
                    out.add((rid, b - T, b + 2.0, int(ln.get("id"))))
            for ln in nxt.findall("left/lane"):          # 左侧生：边界后张开（逆行到头）
                if ln.find("link/predecessor") is None:
                    out.add((rid, b - 2.0, b + T, int(ln.get("id"))))
    return out


def _lane_pos(rm, h, road, lane, s):
    if rm.RM_SetLanePosition(h, road, lane, c_double(0.0), c_double(s), True) < 0:
        return None
    pd = RMPos()
    rm.RM_GetPositionData(h, byref(pd))
    return pd.x, pd.y


def check(rm, path: str) -> bool:
    if rm.RM_Init(path.encode()) != 0:
        print(f"{path}: RM_Init FAIL")
        return False
    name = Path(path).name
    conns = _parse_connections(path)
    merge_ends = _parse_merge_ends(path)
    h = rm.RM_CreatePosition()

    # —— 1) 换乘缝隙全覆盖（esmini 求值器为独立标尺） ——
    worst_a = worst_b = 0.0
    n_seams = 0
    for c in conns:
        L_inc = rm.RM_GetRoadLength(c["inc"])
        L_c = rm.RM_GetRoadLength(c["cid"])
        pa1 = _lane_pos(rm, h, c["inc"], c["from"], max(L_inc, 0.0))
        pa2 = _lane_pos(rm, h, c["cid"], -1, 0.0)
        if pa1 and pa2:
            worst_a = max(worst_a, math.hypot(pa1[0] - pa2[0], pa1[1] - pa2[1]))
            n_seams += 1
        if c["exit"] is None:
            continue
        pb1 = _lane_pos(rm, h, c["cid"], -1, max(L_c, 0.0))
        s_exit = rm.RM_GetRoadLength(c["exit"]) if c["exit_end"] else 0.0
        pb2 = _lane_pos(rm, h, c["exit"], c["to"], max(float(s_exit), 0.0))
        if pb1 and pb2:
            worst_b = max(worst_b, math.hypot(pb1[0] - pb2[0], pb1[1] - pb2[1]))

    # —— 2) 随机行驶冒烟（只对 from 车道匹配的连接计跳变） ——
    from_of = {c["cid"]: (c["inc"], c["from"]) for c in conns}
    lanes_of = {}
    for i in range(rm.RM_GetNumberOfRoads()):
        rid = rm.RM_GetIdOfRoadFromIndex(i)
        if not (10 <= rid < 30):                         # 只从进口路出发
            continue
        n = rm.RM_GetRoadNumberOfDrivableLanes(rid, c_double(1.0))
        ids = []
        for k in range(max(n, 0)):
            lid = c_int(0)
            if rm.RM_GetDrivableLaneIdByIndex(rid, k, c_double(1.0), byref(lid)) == 0:
                ids.append(lid.value)
        lanes_of[rid] = [x for x in ids if x < 0]
    ok_routes, jumps, hops, merges = 0, 0, 0, 0
    for road, lane_ids in sorted(lanes_of.items()):
        for lane in lane_ids:
            s0 = _start_s(rm, road, lane, rm.RM_GetRoadLength(road))
            if s0 is None:                               # 该车道全程零宽：无处起步
                continue
            for _trial in range(3):
                hh = rm.RM_CreatePosition()
                if rm.RM_SetLanePosition(hh, road, lane, c_double(0.0),
                                         c_double(s0), True) < 0:
                    break
                pd = RMPos()
                rm.RM_GetPositionData(hh, byref(pd))
                prev, seq, worst, cur_lane = (pd.x, pd.y, pd.s), [int(pd.roadId)], 0.0, lane
                lane_matched, merged = True, False
                for _ in range(400):
                    if rm.RM_PositionMoveForward(hh, c_double(1.0), c_double(-1.0)) < 0:
                        break
                    rm.RM_GetPositionData(hh, byref(pd))
                    rid_now = int(pd.roadId)
                    step = math.hypot(pd.x - prev[0], pd.y - prev[1])
                    if seq[-1] != rid_now:
                        if rid_now in from_of and from_of[rid_now] != (seq[-1], cur_lane):
                            lane_matched = False          # selector 借道：本次不计
                        seq.append(rid_now)
                    elif step > 1.5:
                        # 同路内大步：落在车道锥形带（收拢/张开）→ 数据本义并线，豁免
                        lo = min(prev[2], pd.s)
                        hi = max(prev[2], pd.s)
                        if any(r == rid_now and l2 == cur_lane and lo <= b_hi and hi >= b_lo
                               for (r, b_lo, b_hi, l2) in merge_ends):
                            merged = True
                            step = 0.0
                    worst = max(worst, step)
                    prev, cur_lane = (pd.x, pd.y, pd.s), pd.laneId
                # 穿越 = 进过连接路（≥100）且之后回到普通路（双向 leg 模型：出口在 10..29 左侧）
                first_conn = next((ix for ix, r in enumerate(seq) if r >= 100), None)
                crossed = first_conn is not None and any(r < 100 for r in seq[first_conn + 1:])
                if not lane_matched:
                    hops += 1
                elif merged:
                    merges += 1
                elif worst > 1.5:
                    jumps += 1
                    print(f"  road{road} lane{lane}: 跳变 {worst:.2f}m 路径 {seq}")
                elif crossed:
                    ok_routes += 1
    rm.RM_Close()
    ok = worst_a < 0.15 and worst_b < 0.15 and jumps == 0 and ok_routes > 0
    print(f"{name}: 换乘缝隙 max 进侧 {worst_a * 100:.1f}cm / 出侧 {worst_b * 100:.1f}cm"
          f"（{n_seams} 对）；行驶穿越 {ok_routes} 次，跳变 {jumps}，"
          f"借道弃计 {hops}，灭车道并线 {merges}"
          f"  -> {'PASS' if ok else 'CHECK'}")
    return ok


def main():
    rm = ctypes.CDLL(str(DLL))
    _bind(rm)
    all_ok = True
    for p in sys.argv[1:]:
        all_ok &= check(rm, str(Path(p).resolve()))
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
