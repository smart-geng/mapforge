"""For failing legs: the lane-topology upstream of the seed and why _chained_links did (not) chain it (read-only).

usage (cwd = experiment tree): python why_no_chain.py <audit.json> <out.json>
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.cwd()))
from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402


def polylen(g):
    g = np.asarray(g, float)
    return float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()) if len(g) > 1 else 0.0


def main(audit_json, out_json):
    src = ProfileSource("shp_0222-0326", "ibd-smarteditor-v1")
    rep = json.load(open(audit_json, encoding="utf-8"))
    juncs = {j.pid: j for j in src.junctions}
    out = []
    for r in rep:
        for g in r["legs"]:
            if g.get("status") != "fail":
                continue
            junc = juncs[r["junction"]]
            lon0, lat0 = float(junc.center[0]), float(junc.center[1])
            proj = lambda p, lat0=lat0, lon0=lon0: S._proj(p, lat0, lon0)  # noqa: E731
            cj = proj(junc.polygon).mean(axis=0)
            far_pid = g["chain"][0]
            g0 = S._center_of(src, far_pid)
            if (np.linalg.norm(proj(g0[:1])[0] - cj) < np.linalg.norm(proj(g0[-1:])[0] - cj)):
                g0 = g0[::-1]
            far = g0[0]
            ups = {}
            for l in src.lanes_of(far_pid):
                for q in src.topo_in.get(l.lane_pid, []):
                    ql = src.lane(q)
                    if ql is not None:
                        ups.setdefault(ql.link_pid, []).append((l.seq, ql.seq))
            rows = []
            rl0 = src.roadlinks[far_pid]
            for up, seqs in ups.items():
                rlu = src.roadlinks.get(up)
                c = src.roadcenters.get(up)
                row = {"upstream": up, "lane_pairs": seqs, "name": rlu.name if rlu else None,
                       "same_name": bool(rlu and rlu.name == rl0.name), "has_centre": c is not None,
                       "lanes": len(src.lanes_of(up)), "upstream_lanes_total": len(src.lanes_of(up)),
                       "is_listed_enter": up in junc.enter_roads, "is_listed_leave": up in junc.leave_roads}
                if c is not None:
                    d0 = float(np.linalg.norm(c[0] - far))
                    d1 = float(np.linalg.norm(c[-1] - far))
                    cand = c if d1 <= d0 else c[::-1]
                    row["end_gap_m"] = round(polylen(proj(np.vstack([cand[-1], far]))), 3)
                    row["end_gap_deg_ok"] = min(d0, d1) < S._DEG_EPS
                    row["join_angle_deg"] = round(math.degrees(S._join_angle(proj, [(far_pid, g0)], cand, True)), 1)
                    row["corridor_turn_deg"] = round(math.degrees(S._corridor_turn(proj(cand))), 1)
                    row["length_m"] = round(polylen(proj(c)), 3)
                rows.append(row)
            # all same-named centres whose end is within 1e-5 deg of far (the converter's candidates)
            cands = []
            for pid, c in src.roadcenters.items():
                if pid in g["chain"]:
                    continue
                rl = src.roadlinks.get(pid)
                if rl is None or rl.name != rl0.name:
                    continue
                if min(float(np.linalg.norm(c[0] - far)), float(np.linalg.norm(c[-1] - far))) < S._DEG_EPS:
                    cands.append(pid)
            out.append({"junction": r["junction"], "enter": g["enter"], "chain": g["chain"],
                        "far_link": far_pid, "far_link_has_centre": far_pid in src.roadcenters,
                        "far_link_name": rl0.name, "topology_upstream": rows, "converter_candidates": cands,
                        "seed_m": g["seed_m"], "earliest_side": g["earliest_side"],
                        "earliest_lateral_m": g["earliest_lateral_m"]})
            print(r["junction"], g["enter"], "chain", len(g["chain"]), "far", far_pid, "centre",
                  far_pid in src.roadcenters, "cands", cands)
            for row in rows:
                print("    ", row)
    Path(out_json).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
