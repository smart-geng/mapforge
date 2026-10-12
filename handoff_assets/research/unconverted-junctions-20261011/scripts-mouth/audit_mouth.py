"""Dataset-wide pre-check of the source-envelope mouth guard (read-only, no fitting).

For every junction of the SHP delivery, replicates the first part of shp_to_xodr.build_leg (pairing incl. the
directional-carriageway split, _chained_links, the mouth points of seed + opposite seed lanes) and evaluates the
_source_envelope_axis guard. Also records the source facts used in the diagnosis: listed enter/leave roads that do
not exist (dangling), listed roads whose ROADCENTER is flagged IS_JUNC=1 / whose ROADLINK MEMO is the junction pid,
and listed roads chained head-to-tail through lane topology.

usage (cwd = experiment tree): python audit_mouth.py <out.json> [scoreboard]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import shapefile

sys.path.insert(0, str(Path.cwd()))
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402

SHP = Path("shp_0222-0326")


def polylen(g):
    g = np.asarray(g, float)
    return float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()) if len(g) > 1 else 0.0


def raw_flags():
    isj, memo = {}, {}
    r = shapefile.Reader(str(SHP / "IBD_ROADCENTER"), encoding="gbk")
    f = [x[0] for x in r.fields[1:]]
    for rec in r.iterRecords():
        m = dict(zip(f, rec))
        isj.setdefault(str(m["LINK_PID"]).strip(), set()).add(int(m["IS_JUNC"] or 0))
    r = shapefile.Reader(str(SHP / "IBD_ROADLINK"), encoding="gbk")
    f = [x[0] for x in r.fields[1:]]
    for rec in r.iterRecords():
        m = dict(zip(f, rec))
        memo[str(m["LINK_PID"]).strip()] = str(m.get("MEMO") or "").strip()
    return isj, memo


def head_to_tail(src, ids):
    """[(upstream, downstream)] among ids where a lane of upstream continues (TOPO) into a lane of downstream."""
    out = set()
    for a in ids:
        for l in src.lanes_of(a):
            for q in src.topo_out.get(l.lane_pid, []):
                ql = src.lane(q)
                if ql is not None and ql.link_pid in ids and ql.link_pid != a:
                    out.add((a, ql.link_pid))
    return sorted(out)


def leg_check(src, junc, proj, cj, e_pid, l_pids, margin=3.0, max_len=160.0):
    chain = S._chained_links(src, e_pid, True, proj, cj, max_len=max_len)
    if not chain:
        return {"enter": e_pid, "status": "no-chain"}
    pts = chain[0][1]
    for _, c in chain[1:]:
        pts = np.vstack([pts, c[1:]])
    gxy = proj(pts)
    L = polylen(gxy)
    seed_len = polylen(proj(chain[-1][1]))
    seed_lanes = [l for l in src.lanes_of(e_pid) if l.geometry.shape[0] >= 2]
    opp = [(lp, l) for lp in l_pids for l in src.lanes_of(lp) if l.geometry.shape[0] >= 2]
    if not seed_lanes:
        return {"enter": e_pid, "status": "no-seed-lanes"}
    dv = gxy[-1] - gxy[-2]
    dv = dv / (np.linalg.norm(dv) + 1e-12)
    nv = np.array([-dv[1], dv[0]])
    rows = []
    for owner, l in [(e_pid, l) for l in seed_lanes] + opp:
        lg = proj(l.geometry)
        lp = lg[0] if np.linalg.norm(lg[0] - cj) < np.linalg.norm(lg[-1] - cj) else lg[-1]
        rows.append((float((lp - gxy[-1]) @ dv), float((lp - gxy[-1]) @ nv), owner, l.lane_pid))
    d_enter = min(r[0] for r in rows if r[2] == e_pid)
    d_all = min(r[0] for r in rows)
    worst = min(rows)
    min_keep = max(6.0, L - seed_len + 0.5)
    station = L + d_all - margin
    return {"enter": e_pid, "leaves": list(l_pids), "status": "fail" if station < min_keep else "ok",
            "chain": [p for p, _ in chain], "axis_m": round(L, 3), "seed_m": round(seed_len, 3),
            "seed_lanes": len(seed_lanes), "seed_has_centre": e_pid in src.roadcenters,
            "station": round(station, 3), "min_keep": round(min_keep, 3),
            "seed_left_m": round(station - (L - seed_len), 3),
            "delta_enter_only_m": round(d_enter, 3), "delta_all_m": round(d_all, 3),
            "earliest_side": "enter" if worst[2] == e_pid else "leave",
            "earliest_lateral_m": round(worst[1], 3), "earliest_lane": worst[3],
            "station_enter_only": round(L + d_enter - margin, 3),
            "fail_enter_only": (L + d_enter - margin) < min_keep}


def main(out_json, only=None):
    src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
    isj, memo = raw_flags()
    rl = src.roadlinks
    juncs = src.junctions
    if only:
        juncs = [j for j in juncs if j.pid in only]
    report = []
    for junc in juncs:
        lon0, lat0 = float(junc.center[0]), float(junc.center[1])
        proj = lambda p, lat0=lat0, lon0=lon0: S._proj(p, lat0, lon0)  # noqa: E731
        cj = proj(junc.polygon).mean(axis=0)
        row = {"junction": junc.pid, "kind": f"{len(junc.enter_roads)}x{len(junc.leave_roads)}",
               "dangling": [p for p in junc.enter_roads + junc.leave_roads if p not in rl],
               "listed_is_junc": [p for p in junc.enter_roads + junc.leave_roads if 1 in isj.get(p, set())],
               "listed_memo_self": [p for p in junc.enter_roads + junc.leave_roads if memo.get(p) == junc.pid],
               "enter_no_centre": [p for p in junc.enter_roads if p in rl and p not in src.roadcenters],
               "leave_no_centre": [p for p in junc.leave_roads if p in rl and p not in src.roadcenters],
               "enter_head_to_tail": head_to_tail(src, set(junc.enter_roads)),
               "leave_head_to_tail": head_to_tail(src, set(junc.leave_roads)), "legs": []}
        try:
            pairs, single = S._pair_legs(src, junc, proj, cj)
            for e_pid, l_pids in pairs:
                l_pids = list(l_pids or [])
                if len(l_pids) == 1:      # the converter's directional-carriageway split (build_junction_xodr)
                    ch_e = S._chained_links(src, e_pid, True, proj, cj, max_len=160.0)
                    ch_l = S._chained_links(src, l_pids[0], False, proj, cj, max_len=160.0)
                    le = sum(polylen(proj(c)) for _p, c in (ch_e or []))
                    ll = sum(polylen(proj(c)) for _p, c in (ch_l or []))
                    dl = abs(le - ll)
                    if min(le, ll) >= 60.0 and dl > 35.0 and dl / max(le, ll, 1e-9) > 0.15:
                        l_pids = []
                try:
                    row["legs"].append(leg_check(src, junc, proj, cj, e_pid, l_pids))
                except Exception as exc:  # noqa: BLE001
                    row["legs"].append({"enter": e_pid, "status": f"error: {exc}"})
        except Exception as exc:  # noqa: BLE001
            row["error"] = str(exc)
        report.append(row)
        fails = [g for g in row["legs"] if g.get("status") == "fail"]
        if fails or row["dangling"] or row["enter_head_to_tail"] or row["listed_is_junc"]:
            print(junc.pid, row["kind"], "fails", [(g["enter"], g["station"], g["min_keep"], g["earliest_side"],
                                                   g["earliest_lateral_m"]) for g in fails],
                  "dangling", row["dangling"], "h2t", row["enter_head_to_tail"], row["leave_head_to_tail"],
                  "is_junc", row["listed_is_junc"], flush=True)
    Path(out_json).write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    legs = [g for r in report for g in r["legs"]]
    print("junctions", len(report), "legs", len(legs), "fail", sum(g.get("status") == "fail" for g in legs),
          "fail_enter_only", sum(bool(g.get("fail_enter_only")) for g in legs))


if __name__ == "__main__":
    only = None
    if len(sys.argv) > 2:
        only = set(sys.argv[2].split(","))
    main(sys.argv[1], only)
