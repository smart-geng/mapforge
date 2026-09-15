"""直接从原始 SHP 复核一个未达标 via，区分原始端点和 written mouth。"""
import argparse
import json
import hashlib
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from mapforge.validate.shp_boundary_fidelity import _point_polyline_distance
from mapforge.ops.shp_to_xodr import _densify_polyline
from mapforge.validate.smoothness import sample_road_ref, lane_edges_at


def _lane_tail(road, lane_id, at_end, distance=30.0):
    """沿真实 laneLink 反查普通车道，返回口部→远端；绝不跨车道取最近线。"""
    pts, ss, hh = sample_road_ref(road, .1)
    sections = road.findall("lanes/laneSection")
    order = list(range(len(sections)))
    if at_end:
        order.reverse()
    lid, result = int(lane_id), []
    L = float(ss[-1])
    for index in order:
        sec = sections[index]
        s0 = float(sec.get("s"))
        s1 = float(sections[index+1].get("s")) if index+1 < len(sections) else L
        if (at_end and s1 < L-distance) or (not at_end and s0 > distance):
            break
        lane = sec.find(f".//lane[@id='{lid}']")
        if lane is None:
            raise ValueError(f"laneLink references missing lane {lid}")
        lo, hi = (max(s0, L-distance), s1) if at_end else (s0, min(s1, distance))
        grid = np.linspace(lo, hi, max(2, int((hi-lo)/.1)+1))
        if at_end:
            grid = grid[::-1]
        side = "left" if lid > 0 else "right"
        ids = sorted((int(x.get("id")) for x in sec.findall(f"{side}/lane")), key=abs)
        edge_index = ids.index(lid)
        for s in grid:
            t_edges = lane_edges_at(road, min(float(s), s1-1e-7), side)
            t = .5*(t_edges[edge_index]+t_edges[edge_index+1])
            h = np.interp(s, ss, hh)
            point = np.array([np.interp(s, ss, pts[:, k]) for k in range(2)])
            result.append(point + t*np.array([-np.sin(h), np.cos(h)]))
        link = lane.find("link/predecessor" if at_end else "link/successor")
        if link is None:
            break
        lid = int(link.get("id"))
    return np.asarray(result)


def render(path, source_id, output, baseline=None):
    root = ET.parse(path).getroot()
    lat0, lon0 = _origin(root)
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    via = src.lane(source_id)
    if via is None:
        raise ValueError(f"missing source lane {source_id}")
    road = next(r for r in root.findall("road")
                if r.get("junction") != "-1" and r.find(
                    f".//userData[@code='mapforge.source_lane'][@value='{source_id}']") is not None)
    target, _ss, _hh = sample_road_ref(road, 0.1)
    # 当前 connecting road 的 reference 正好位于恒宽车道中心；若此约定改变则拒绝。
    lane = road.find("lanes/laneSection/right/lane[@id='-1']")
    widths = lane.findall("width")
    offsets = road.findall("lanes/laneOffset")
    assert len(widths) == len(offsets) == 1
    assert all(abs(float(e.get(k, 0))) < 1e-10 for e in widths + offsets for k in ("b", "c", "d"))
    assert abs(float(offsets[0].get("a")) - float(widths[0].get("a"))/2) < 1e-6
    raw = _project(via.geometry, lat0, lon0)
    if np.linalg.norm(raw[-1] - target[0]) < np.linalg.norm(raw[0] - target[0]):
        raw = raw[::-1]
    neighbors = []
    for role, ids in (("incoming", src.topo_in.get(source_id, [])),
                      ("outgoing", src.topo_out.get(source_id, []))):
        for sid in ids:
            rec = src.lane(sid)
            if rec is None or len(rec.geometry) < 2:
                continue
            g = _project(rec.geometry, lat0, lon0)
            endpoint = raw[0] if role == "incoming" else raw[-1]
            gap = min(np.linalg.norm(g[0] - endpoint), np.linalg.norm(g[-1] - endpoint))
            neighbors.append({"role": role, "id": sid, "points": g,
                              "raw_endpoint_gap_m": float(gap)})
    fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
    colors = {"incoming": "#2b8cbe", "outgoing": "#2ca25f"}
    for n in neighbors:
        ax.plot(*n["points"].T, color=colors[n["role"]], label=n["role"] + " raw lane")
    if baseline is not None:
        old_root = ET.parse(baseline).getroot()
        assert np.allclose(_origin(old_root), (lat0, lon0), rtol=0, atol=1e-9)
        old_road = next(r for r in old_root.findall("road")
                        if r.get("junction") != "-1" and r.find(
                            f".//userData[@code='mapforge.source_lane'][@value='{source_id}']") is not None)
        old_target, _, _ = sample_road_ref(old_road, .1)
        ax.plot(*old_target.T, "--", color="#666666", lw=2, label="baseline XODR lane center")
    ax.plot(*raw.T, "o-", color="#e69f00", ms=3, lw=2, label="raw SHP via (unclipped)")
    ax.plot(*target.T, color="#cc2288", lw=2, label="written XODR lane center")
    for i, label in ((0, "in mouth"), (-1, "out mouth")):
        ax.scatter(*target[i], color="#cc2288", marker="x", s=90, zorder=10)
        ax.annotate(label, target[i], xytext=(8, -12), textcoords="offset points")
        ax.plot([raw[i, 0], target[i, 0]], [raw[i, 1], target[i, 1]], "k--", lw=1)
    bounds = np.vstack([raw, target])
    ax.set_xlim(bounds[:, 0].min()-8, bounds[:, 0].max()+8)
    ax.set_ylim(bounds[:, 1].min()-8, bounds[:, 1].max()+8)
    ax.set_aspect("equal")
    ax.set_xlabel("local x (m)")
    ax.set_ylabel("local y (m)")
    ax.set_title(f"{path.stem} / road {road.get('id')} / source {source_id}", pad=15)
    ax.legend(loc="best", fontsize=9)
    ax.grid(alpha=.2)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output)
    plt.close(fig)
    report = {"source_lane_id": source_id, "road_id": road.get("id"),
              "raw_to_written_start_m": float(np.linalg.norm(raw[0]-target[0])),
              "raw_to_written_end_m": float(np.linalg.norm(raw[-1]-target[-1])),
              "raw_topology_endpoints": [{k:v for k,v in n.items() if k != "points"} for n in neighbors]}
    output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report))
    print(output)


def audit_suite(folder, output, cases=None):
    """全部真实 via 的未裁剪证据，不采用 G8 eligibility 作为过滤器。"""
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    rows, artifacts = [], []
    for path in sorted(folder.glob("*.xodr")):
        if cases is not None and path.stem not in cases:
            continue
        artifacts.append({"case": path.stem,
                          "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
        root = ET.parse(path).getroot()
        roads = {r.get("id"):r for r in root.findall("road")}
        lat0, lon0 = _origin(root)
        for road in root.findall("road"):
            if road.get("junction") == "-1" or road.get("name") == "junction_paving":
                continue
            lane = road.find("lanes/laneSection/right/lane[@id='-1']")
            sid_el = lane.find("userData[@code='mapforge.source_lane']") if lane is not None else None
            if sid_el is None:
                continue
            sid = sid_el.get("value")
            via = src.lane(sid)
            if via is None:
                raise ValueError(f"unknown source id {sid}")
            widths, offsets = lane.findall("width"), road.findall("lanes/laneOffset")
            assert len(widths) == len(offsets) == 1
            assert all(abs(float(e.get(k, 0))) < 1e-10
                       for e in widths + offsets for k in ("b", "c", "d"))
            assert abs(float(offsets[0].get("a")) - float(widths[0].get("a"))/2) < 1e-6
            target, _, _ = sample_road_ref(road, 0.1)
            raw = _project(via.geometry, lat0, lon0)
            if np.linalg.norm(raw[-1]-target[0]) < np.linalg.norm(raw[0]-target[0]):
                raw = raw[::-1]
            raw_gaps = []
            for ids, endpoint in ((src.topo_in.get(sid, []), raw[0]),
                                  (src.topo_out.get(sid, []), raw[-1])):
                gaps = []
                for nid in ids:
                    rec = src.lane(nid)
                    if rec is None:
                        continue
                    ng = _project(rec.geometry, lat0, lon0)
                    gaps.append(min(np.linalg.norm(ng[0]-endpoint),
                                    np.linalg.norm(ng[-1]-endpoint)))
                raw_gaps.append(float(min(gaps)) if gaps else None)
            dist, _ = _point_polyline_distance(_densify_polyline(raw, .25), target)
            p95, mx = float(np.percentile(dist, 95)), float(np.max(dist))
            route_parts = []
            for role in ("predecessor", "successor"):
                road_link = road.find(f"link/{role}")
                lane_link = lane.find(f"link/{role}")
                tail = _lane_tail(roads[road_link.get("elementId")],
                                  int(lane_link.get("id")),
                                  road_link.get("contactPoint") == "end")
                route_parts.append(tail)
            seam_gaps = [float(np.linalg.norm(route_parts[0][0]-target[0])),
                         float(np.linalg.norm(route_parts[1][0]-target[-1]))]
            if max(seam_gaps) > .15:
                raise ValueError(f"{path.stem}/{road.get('id')} broken lane route: {seam_gaps}")
            route = np.vstack([route_parts[0][::-1], target, route_parts[1]])
            route_dist, _ = _point_polyline_distance(_densify_polyline(raw, .25), route)
            route_p95, route_max = float(np.percentile(route_dist, 95)), float(route_dist.max())
            prov_el = lane.find("userData[@code='mapforge.provenance/v1']")
            prov = json.loads(prov_el.get("value", "{}")) if prov_el is not None else {}
            rows.append({"case": path.stem, "road_id": road.get("id"),
                         "source_lane_id": sid, "raw_topology_endpoint_gaps_m": raw_gaps,
                         "written_mouth_to_raw_endpoint_m": [
                             float(np.linalg.norm(raw[0]-target[0])),
                             float(np.linalg.norm(raw[-1]-target[-1]))],
                         "full_raw_source_to_target": {
                             "median_m": float(np.median(dist)), "p95_m": p95, "max_m": mx},
                         "full_raw_source_to_linked_route": {
                             "median_m": float(np.median(route_dist)),
                             "p95_m": route_p95, "max_m": route_max},
                         "raw_coverage_within_0_75m": float(np.mean(dist <= .75)),
                         "needs_shape_review": route_p95 > .75 or route_max > 1.5,
                         "written_exclusion_code": prov.get("exclusion_code")})
    if not artifacts or (cases is not None and set(cases)-{x["case"] for x in artifacts}):
        raise ValueError("requested XODR audit cases missing")
    report = {"schema": "mapforge/unclipped-via-audit/v1", "folder": str(folder),
              "includes_excluded_vias": True, "source_cropping": False,
              "review_thresholds_m": {"p95": .75, "max": 1.5},
              "note": "未裁剪来源到 laneLink 确定的完整行车链（含两侧各30m）的距离；另列单连接路距离。不是车型可行性验收。",
              "count": len(rows), "needs_shape_review": sum(r["needs_shape_review"] for r in rows),
              "raw_topology_closed_count": sum(
                  all(g is not None and g <= .01 for g in r["raw_topology_endpoint_gaps_m"])
                  for r in rows), "artifacts": artifacts, "rows": rows}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
                      encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k != "rows"}, ensure_ascii=False))
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("xodr", type=Path, help="xodr file, or directory with --suite")
    p.add_argument("source_lane", nargs="?")
    p.add_argument("output", type=Path)
    p.add_argument("--suite", action="store_true")
    p.add_argument("--baseline", type=Path)
    p.add_argument("--case", action="append", help="explicit case filter for --suite")
    args = p.parse_args()
    if args.suite:
        audit_suite(args.xodr, args.output, args.case)
    else:
        if args.source_lane is None:
            p.error("source_lane required for single rendering")
        render(args.xodr, args.source_lane, args.output, args.baseline)
