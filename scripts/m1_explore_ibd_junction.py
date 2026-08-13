# -*- coding: utf-8 -*-
"""M1 探索：node16 refPos → IBD 路口面 → ENTER_ROAD → 车道 → 拓扑，验证直读链完整性。"""
from __future__ import annotations

import math
import sys

import numpy as np
import shapefile

sys.path.insert(0, r"F:\MapFactory")

SHP = r"F:\MapFactory\shp_0222-0326"
REF = (106.3183590, 29.5161415)     # node16 refPos (lon, lat)


def rd(name, fields=None):
    # IBD 的 DBF LDID=0x57、中文实为 GBK（资料盘点 2.1）
    return shapefile.Reader(SHP + "\\" + name, encoding="gbk")


def main():
    # 1) 最近路口面
    r = rd("IBD_OBJECT_INTERSECTION_SURFACE")
    fields = [f[0] for f in r.fields[1:]]
    best = None
    for sr in r.iterShapeRecords():
        pts = np.asarray(sr.shape.points)
        c = pts.mean(axis=0)
        d = math.hypot(c[0] - REF[0], c[1] - REF[1])
        if best is None or d < best[0]:
            best = (d, dict(zip(fields, sr.record)), pts)
    d, rec, poly = best
    print("最近路口面: dist=%.1f m  NAME=%s  PID=%s" % (d * 111000, rec.get("NAME"), rec.get("OBJECT_PID")))
    enter = [x for x in str(rec.get("ENTER_ROAD", "")).split(";") if x.strip()]
    leave = [x for x in str(rec.get("LEAVE_ROAD", "")).split(";") if x.strip()]
    print("ENTER_ROAD(%d): %s" % (len(enter), enter))
    print("LEAVE_ROAD(%d): %s" % (len(leave), leave))

    # 2) ROADLINK 索引
    rl = rd("IBD_ROADLINK")
    rl_fields = [f[0] for f in rl.fields[1:]]
    roadlinks = {}
    for sr in rl.iterShapeRecords():
        rec2 = dict(zip(rl_fields, sr.record))
        roadlinks[str(rec2["LINK_PID"]).strip()] = (rec2, np.asarray(sr.shape.points))
    for pid in enter:
        if pid in roadlinks:
            rec2, pts = roadlinks[pid]
            print("  enter %s: name=%s lanes=%s len=%sm pts=%d" %
                  (pid, rec2.get("ROADNAME"), rec2.get("LANE_NUM"),
                   (rec2.get("LENGTH") or 0) // 1000 if rec2.get("LENGTH") else "?", len(pts)))
        else:
            print("  enter %s: !! ROADLINK 缺失" % pid)

    # 3) LANE_LINK：进口 Link 的车道
    ll = rd("IBD_LANE_LINK")
    ll_fields = [f[0] for f in ll.fields[1:]]
    lanes_by_link = {}
    lane_geo = {}
    for sr in ll.iterShapeRecords():
        rec3 = dict(zip(ll_fields, sr.record))
        lp = str(rec3["LINK_PID"]).strip()
        lanes_by_link.setdefault(lp, []).append(rec3)
        lane_geo[str(rec3["LANE_PID"]).strip()] = np.asarray(sr.shape.points)
    for pid in enter:
        lns = sorted(lanes_by_link.get(pid, []), key=lambda x: int(x.get("SEQ_NUM") or 0))
        print("  link %s lanes: %s" % (pid, [(str(l["LANE_PID"])[-5:], l.get("SEQ_NUM"), l.get("WIDTH"),
                                              l.get("LANE_TYPE")) for l in lns]))

    # 4) TOPO：进口车道出度（IN→OUT）
    tp = rd("IBD_LANE_TOPO_DETAIL")
    tp_fields = [f[0] for f in tp.fields[1:]]
    topo_out = {}
    for rec4 in tp.iterRecords():
        m = dict(zip(tp_fields, rec4))
        topo_out.setdefault(str(m["IN_PID"]).strip(), []).append(str(m["OUT_PID"]).strip())
    # MERGE 层（路口内虚拟车道）
    mg = rd("IBD_LANE_LINK_MERGE")
    mg_fields = [f[0] for f in mg.fields[1:]]
    merge_ids = set()
    merge_link = {}
    for sr in mg.iterShapeRecords():
        m = dict(zip(mg_fields, sr.record))
        mid = str(m["LANE_PID"]).strip()
        merge_ids.add(mid)
        merge_link[mid] = str(m.get("LINK_PID", "")).strip()
        lane_geo.setdefault(mid, np.asarray(sr.shape.points))
    n_via_merge = n_direct = 0
    for pid in enter:
        for l in lanes_by_link.get(pid, []):
            lp = str(l["LANE_PID"]).strip()
            outs = topo_out.get(lp, [])
            via = [o for o in outs if o in merge_ids]
            n_via_merge += len(via)
            n_direct += len(outs) - len(via)
            if outs:
                ends = []
                for o in via:
                    outs2 = topo_out.get(o, [])
                    ends += outs2
                print("    lane ..%s -> outs=%d (经虚拟车道 %d, 虚拟车道再出 %s)" %
                      (lp[-5:], len(outs), len(via), [e[-5:] for e in ends]))
    print("进口车道出边合计：经 MERGE 虚拟车道 %d 条 / 直连 %d 条" % (n_via_merge, n_direct))


if __name__ == "__main__":
    main()
