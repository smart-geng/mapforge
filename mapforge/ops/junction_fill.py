# -*- coding: utf-8 -*-
"""路口转向补全（connect-mode）：源数据的车道拓扑可能缺录/失配，此模块按**几何**
推断缺失的进口车道→出口车道转向，供两条 xodr 管道共用。

三档（默认 data，即不发明拓扑——这是项目硬约束的默认立场）：
- ``data``    只用源数据（IBD LANE_TOPO_DETAIL / MAP connectsTo）；
- ``default`` 只补"完全没有任何出口"的进口车道：按车道在断面中的位置给一条
              最合理的转向（最左→左转、最右→右转、其余→直行）——治"个别车道漏录"；
- ``full``    全连接：每条进口车道 → 每个非掉头出口腿的**每条**车道——治"整片缺录"，
              同时把路口内部铺满行车带。

补出的连接一律标 INFERRED（stats.conn_filled），并受曲率守卫约束：
合成的 G2 回旋链若 |κ| 超限（默认 R<8m）判为不可行转向，丢弃并计入 conn_fill_skipped。
"""
from __future__ import annotations

import math

MODES = ("data", "default", "full")
_STRAIGHT = math.radians(45.0)       # |Δh| < 45° 视为直行
_UTURN = math.radians(135.0)         # |Δh| > 135° 视为掉头


def turn_of(h_in: float, h_out: float) -> str:
    """进口末端航向 → 出口起始航向的转向分类（left/right/straight/uturn）。"""
    d = (h_out - h_in + math.pi) % (2 * math.pi) - math.pi
    if abs(d) > _UTURN:
        return "uturn"
    if abs(d) < _STRAIGHT:
        return "straight"
    return "left" if d > 0 else "right"


def plan_fill(entries, exits, existing, mode: str = "full", allow_uturn: bool = False):
    """规划需要补的 (entry, exit) 对。

    entries: [{key, leg, idx, n, pose(x,y,h)}]——idx=断面内序号（0=最左）、n=本腿车道数；
    exits:   [{key, leg, idx, n, pose(x,y,h)}]——idx=0 为最靠中线的出口车道；
    existing: 已有连接的 {(entry_key, exit_key)} 集合。
    返回 [(entry, exit)]，不含 existing 中已有的对。"""
    if mode not in MODES:
        raise ValueError(f"connect mode must be one of {MODES}: {mode!r}")
    if mode == "data" or not entries or not exits:
        return []
    by_leg = {}
    for x in exits:
        by_leg.setdefault(x["leg"], []).append(x)
    for lst in by_leg.values():
        lst.sort(key=lambda x: x["idx"])

    def _cands(e):
        """本进口车道可达的 (turn, exit) 候选（按出口腿分组）。"""
        out = []
        for leg, lanes in by_leg.items():
            if leg == e["leg"]:                          # 同腿=掉头
                if not allow_uturn:
                    continue
                turn = "uturn"
            else:
                turn = turn_of(e["pose"][2], lanes[0]["pose"][2])
                if turn == "uturn" and not allow_uturn:
                    continue
            out.append((turn, lanes))
        return out

    by_key = {x["key"]: x for x in exits}
    plan = []
    for e in entries:
        # 本车道**已有**的转向集合（按几何分类，非按数据字段——数据字段正是可疑的那个）
        have = set()
        for (ek, xk) in existing:
            if ek != e["key"]:
                continue
            x = by_key.get(xk)
            if x is not None:
                have.add("uturn" if x["leg"] == e["leg"]
                         else turn_of(e["pose"][2], x["pose"][2]))
        want = _default_turn(e)
        for turn, lanes in _cands(e):
            if mode == "full":
                targets = lanes                          # 全连接：该腿所有出口车道
            else:                                        # default：只补该车道位置应有的转向
                if turn != want or want in have:
                    continue
                targets = [_position_match(e, lanes, turn)]
            for x in targets:
                if (e["key"], x["key"]) not in existing:
                    plan.append((e, x))
    return plan


def _default_turn(e) -> str:
    """按车道在断面中的位置推断唯一转向：最左→左转、最右→右转、其余→直行。"""
    if e["n"] <= 1:
        return "straight"
    if e["idx"] == 0:
        return "left"
    if e["idx"] == e["n"] - 1:
        return "right"
    return "straight"


def _position_match(e, lanes, turn):
    """目标车道：左转取最靠中线、右转取最外、直行按断面序号对位。"""
    if turn == "left":
        return lanes[0]
    if turn == "right":
        return lanes[-1]
    return lanes[min(e["idx"], len(lanes) - 1)]
