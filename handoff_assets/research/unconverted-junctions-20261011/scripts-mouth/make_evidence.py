"""Assemble the small numeric evidence files from the probe outputs (no conversion).

usage: python make_evidence.py <diag-out/mouth>
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def lane_ends(topo, pid):
    info = topo["links"].get(pid)
    if not info:
        return None
    return [{"lane": l["lane"], "seq": l["seq"], "start_xy": l["xy"][0], "end_xy": l["xy"][-1], "len_m": l["len_m"]}
            for l in info["lanes"]]


def main(d):
    d = Path(d)
    probe = d / "probe"
    # ---------------------------------------------------------------- case 1
    leg1, topo1 = load(probe / "case1-leg.json"), load(probe / "case1-topo.json")
    fl = leg1["failing_leg"]
    case1 = {
        "junction": leg1["junction"], "name": leg1["name"],
        "error": fl["error"],
        "listed_enter": leg1["enter_roads"], "listed_leave": leg1["leave_roads"],
        "converter_pairs": leg1["pairs"],
        "failing_leg": {"enter_link": fl["enter_link"], "leave_links": fl["leave_links"],
                        "chain": [{k: r[k] for k in ("link", "length_m", "has_centre", "station_range")}
                                  for r in fl["chain_upstream_to_junction"]],
                        "axis_length_m": fl["axis_length_m"], "mouth_station_m": fl["mouth_station"],
                        "min_keep_m": fl["min_keep_m"],
                        "axis_enters_source_surface_at_station_m": fl["axis_surface_station_range"][0],
                        "substitute_centre": "middle lane (seq 2) of the seed, 5.167 m",
                        "earliest_lane_end": {k: fl["earliest_lane_end"][k] for k in
                                              ("lane", "seq", "junction_end_offset", "junction_end_in_surface",
                                               "junction_end_dist_to_surface_m")}},
        "why_no_upstream_chain": {
            "upstream_by_lane_topology": "2023063010392607255 (listed enter, 141.6 m, lanes 1:1)",
            "seed_roadcenter": "exists but IS_JUNC=1 -> skipped by the reader",
            "gap_middle_lane_start_to_upstream_centre_end_m": 1.883,
            "converter_join_tolerance": "1e-5 deg (~1 m)"},
        "source_flags": {
            "2023063010392607255": {"IS_JUNC": 0, "MEMO": ""},
            "2023063010394548043": {"IS_JUNC": 1, "MEMO": "2023070118182048481", "listed": "enter"},
            "2023063010394551273": {"IS_JUNC": 1, "MEMO": "2023070118182048481", "listed": None},
            "2023063010394022483": {"IS_JUNC": 1, "MEMO": "2023070118182048481", "listed": "leave"},
            "2023063010391836450": {"IS_JUNC": 0, "MEMO": "", "listed": "leave"},
            "2023063010375642074": {"IS_JUNC": 1, "MEMO": "2023070118182048481", "listed": None}},
        "westbound_lane_chain_x_m": {"2023063010392607255": [154.02, 12.45], "2023063010394548043": [12.45, 7.28],
                                     "2023063010394551273": [7.28, -5.12], "2023063010394022483": [-5.12, -10.41],
                                     "2023063010391836450": [-10.41, -13.0]},
        "eastbound_lane_chain_x_m": {"2023063010375108825": [-35.96, -9.72], "2023063010375642074": [-9.72, 12.48],
                                     "2023063010375644870": [12.48, 33.87]},
        "roadlink_nodes": "392607255.E_NODE = 394548043.S_NODE; 394548043.E = 394551273.S; 394551273.E = "
                          "394022483.S; 394022483.E = 391836450.S",
        "lane_ends": {pid: lane_ends(topo1, pid) for pid in ("2023063010392607255", "2023063010394548043",
                                                             "2023063010394022483", "2023063010391836450")},
    }
    (d / "case1-evidence.json").write_text(json.dumps(case1, ensure_ascii=False, indent=1), encoding="utf-8")
    # ---------------------------------------------------------------- case 2
    leg2, topo2 = load(probe / "case2-leg.json"), load(probe / "case2-topo.json")
    fl = leg2["failing_leg"]
    case2 = {
        "junction": leg2["junction"], "name": leg2["name"], "error": fl["error"],
        "listed_enter": leg2["enter_roads"], "listed_leave": leg2["leave_roads"],
        "dangling_listed_enter": ["2023063016133426805", "2023062916401270486"],
        "dangling_note": "these two ids occur only in IBD_OBJECT_INTERSECTION_SURFACE.dbf (no ROADLINK/LANE/CENTER)",
        "unlisted_topological_enter": {"2024022014091095228": "east approach main carriageway, 4 lanes, ends x=43.6-43.7",
                                       "2024022013530728003": "west approach main carriageway, 3 lanes"},
        "converter_pairs": leg2["pairs"], "single_leaves": leg2["single_leaves"],
        "failing_leg": {"enter_link": fl["enter_link"], "leave_links": fl["leave_links"],
                        "seed": "1-lane right-turn slip road fed by lane 5 of 2023070819384191610 (lanes 1-4 -> "
                                "2024022014091094027 -> 2024022014091095228)",
                        "chain": [{k: r[k] for k in ("link", "lane_num", "length_m", "station_range")}
                                  for r in fl["chain_upstream_to_junction"]],
                        "axis_length_m": fl["axis_length_m"], "seed_station_range": fl["seed_station_range"],
                        "mouth_station_m": fl["mouth_station"], "min_keep_m": round(fl["min_keep_m"], 3),
                        "own_lanes_mouth_station_m": round(fl["axis_length_m"] - 3.0, 3),
                        "mouth_points": [{k: m[k] for k in ("owner_link", "side", "lane", "seq",
                                                            "end_proj_along_axis_end_dir_m", "junction_end_offset",
                                                            "junction_end_dist_to_surface_m")}
                                         for m in fl["mouth_points"]]},
        "west_slip_leg_passing_but_pulled_back": {"enter_link": "2023062916465136085",
                                                 "leave_link": "2023062917143657253", "axis_shift_m": -23.498},
    }
    (d / "case2-evidence.json").write_text(json.dumps(case2, ensure_ascii=False, indent=1), encoding="utf-8")
    # ---------------------------------------------------------------- dataset audit
    audit = load(probe / "audit-all.json")
    why = {(w["junction"], w["enter"]): w for w in load(probe / "why-no-chain.json")}
    rows = []
    for r in audit:
        for g in r["legs"]:
            if g.get("status") != "fail":
                continue
            w = why.get((r["junction"], g["enter"]), {})
            if len(g["chain"]) == 1:
                ups = w.get("topology_upstream", [])
                if not g["seed_has_centre"]:
                    cls = "A-no-chain:seed-has-no-usable-centre"
                elif any(not u.get("same_name") for u in ups) and ups:
                    cls = "A-no-chain:upstream-other-roadname"
                elif len(w.get("converter_candidates", [])) > 1:
                    cls = "A-no-chain:ambiguous-upstream"
                else:
                    cls = "A-no-chain:slip-diverging-from-wide-road"
            elif not g["fail_enter_only"]:
                cls = "C-shared-mouth:opposite-lanes-far-to-the-side"
            else:
                cls = "B-chained:seed-link-shorter-than-margin"
            rows.append({"junction": r["junction"], "kind": r["kind"], "enter": g["enter"], "class": cls,
                         "seed_m": g["seed_m"], "seed_lanes": g["seed_lanes"], "chain_links": len(g["chain"]),
                         "station_m": g["station"], "min_keep_m": g["min_keep"], "earliest_side": g["earliest_side"],
                         "earliest_lateral_m": g["earliest_lateral_m"],
                         "listed_head_to_tail": bool(r["enter_head_to_tail"]),
                         "dangling_listed": r["dangling"]})
    summary = {"junctions": len(audit), "legs": sum(len(r["legs"]) for r in audit), "failing_legs": len(rows),
               "failing_junctions": len({x["junction"] for x in rows}),
               "by_class": dict(Counter(x["class"] for x in rows)),
               "junctions_with_dangling_listed_roads": [r["junction"] for r in audit if r["dangling"]],
               "junctions_with_listed_head_to_tail": [r["junction"] for r in audit
                                                     if r["enter_head_to_tail"] or r["leave_head_to_tail"]],
               "scoreboard_legs_with_opposite_lanes_setting_the_mouth": {
                   "node4": 1, "node13": 1, "node16": 2, "node17": 2, "node18": 2,
                   "note": "opposite lanes 5-15.7 m to the side move the shared mouth by up to 3.36 m there; "
                           "none of the 7 scoreboard junctions fails the guard"},
               "rows": rows}
    (d / "audit-mouth-guard.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "rows"}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
