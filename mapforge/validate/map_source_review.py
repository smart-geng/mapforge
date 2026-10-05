"""Raw MAP point-list review (read-only): points that look like source errors, reported for a human decision.

Both rules work on the raw integer coordinates of each lane point list, in local metres, with the same
helpers map_source_corrections uses to recompute the evidence of an approved decision:

- ``isolated-detour``: the point lies more than 100 m off the chord of its neighbours, which are at most
  100 m apart, with a detour ratio above 10 (map-node18 north lane 1 point 1: 1.72 km). Same thresholds as
  scripts/map_raw_geometry_review.py, which is bound to frozen evidence and stays unchanged.
- ``lateral-spike``: the lane turns at least 15 deg at the point and, with the point removed, turns at most
  4 deg at its neighbours, while the point lies at least 1 m off their chord: the lane steps out and back at
  one point (map-node4 west lanes 2 and 3 point 5: 2.56 m and 2.38 m off a 30 m chord, 22 and 21 deg; without
  them the neighbours turn 2.9 and 2.5 deg). Real bends in the seven junctions turn at most 12 deg at a point;
  the neighbour condition alone is not enough (a real 8.8 deg bend, map-node18 east, leaves 3.2 deg), and a
  lane that shifts to make room for a turn bay stays shifted (map-node4 east lanes 2-4: 0.7-2.0 m).

A finding is REVIEW_REQUIRED only. Dropping a point needs an approved, versioned decision
(profiles/source-corrections, applied by mapforge.ops.map_source_corrections); the original file is never
modified.

    python -m mapforge.validate.map_source_review --out out/map-source-review-<date>.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

from lxml import etree

from mapforge.ops import map_source_corrections as C

ROOT = Path(__file__).resolve().parents[2]
DETOUR_OFFSET_M, DETOUR_RATIO_MIN, DETOUR_SPAN_MAX_M = 100.0, 10.0, 100.0
SPIKE_TURN_MIN_DEG, SPIKE_NEIGHBOUR_TURN_MAX_DEG, SPIKE_OFFSET_MIN_M = 15.0, 4.0, 1.0


def _turn(u, v):
    """Unsigned turning angle [deg] from direction u to direction v."""
    return math.degrees(abs(math.atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1])))


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def point_findings(xy):
    """Findings for the interior points of one lane point list (local metres)."""
    out = []
    for i in range(1, len(xy) - 1):
        offset, span, ratio = C._detour(xy, i)
        turn = _turn(_sub(xy[i], xy[i - 1]), _sub(xy[i + 1], xy[i]))
        chord = _sub(xy[i + 1], xy[i - 1])
        after = [_turn(_sub(xy[i - 1], xy[i - 2]), chord)] if i >= 2 else []
        after += [_turn(chord, _sub(xy[i + 2], xy[i + 1]))] if i + 2 < len(xy) else []
        rule = None
        if offset > DETOUR_OFFSET_M and ratio > DETOUR_RATIO_MIN and span <= DETOUR_SPAN_MAX_M:
            rule = "isolated-detour"
        elif (turn >= SPIKE_TURN_MIN_DEG and after and max(after) <= SPIKE_NEIGHBOUR_TURN_MAX_DEG
              and offset >= SPIKE_OFFSET_MIN_M):
            rule = "lateral-spike"
        if rule:
            out.append({"rule": rule, "point_index_0based": i, "distance_to_neighbor_chord_m": offset,
                        "neighbor_chord_length_m": span, "detour_ratio": ratio, "turn_deg": turn,
                        "neighbour_turn_without_point_deg": max(after) if after else None})
    return out


def review_file(path) -> dict:
    """REVIEW_REQUIRED findings of one raw MAP XML (every node, inbound link and lane)."""
    path = Path(path)
    root = etree.parse(str(path)).getroot()
    findings = []
    for node in root.findall("mapFrame/nodes/Node"):
        nid = (int(node.findtext("id/region", "-1")), int(node.findtext("id/id", "-1")))
        for link in node.findall("inLinks/Link"):
            name = (link.findtext("name") or "").strip()
            up = (int(link.findtext("upstreamNodeId/region", "-1")), int(link.findtext("upstreamNodeId/id", "-1")))
            for lane in link.findall("lanes/Lane"):
                lane_id = int(lane.findtext("laneID", "-1"))
                raw = [C._raw(rp) for rp in lane.findall("points/RoadPoint")]
                for f in _lane(raw):
                    i = f["point_index_0based"]
                    f.update(status="REVIEW_REQUIRED", source_action="NONE; original point retained",
                             source_lane_id=f"map:{nid[0]}:{nid[1]}:from:{up[0]}:{up[1]}:{name}:lane:{lane_id}",
                             target={"node": {"region": nid[0], "id": nid[1]},
                                     "link": {"name": name, "upstream": {"region": up[0], "id": up[1]}},
                                     "lane": lane_id, "point_index_0based": i,
                                     "raw": {"lon": raw[i][0], "lat": raw[i][1]}})
                    findings.append(f)
    return {"source": {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()},
            "findings": findings}


def _lane(raw):
    """Findings of one lane; each point is measured in metres around its previous point (as the decisions)."""
    out = []
    for f in point_findings([C._local_xy(lon, lat, raw[0][0], raw[0][1]) for lon, lat in raw]):
        i = f["point_index_0based"]
        xy = [C._local_xy(lon, lat, raw[i - 1][0], raw[i - 1][1]) for lon, lat in raw]
        f["distance_to_neighbor_chord_m"], f["neighbor_chord_length_m"], f["detour_ratio"] = C._detour(xy, i)
        out.append(f)
    return out


def main(argv=None) -> int:
    from mapforge.validate.scoreboard import CASES
    ap = argparse.ArgumentParser(prog="python -m mapforge.validate.map_source_review", description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, help="JSON report path")
    a = ap.parse_args(argv)
    report = {"schema": "mapforge/map-source-review/v1",
              "scope": "raw MAP point-list review; findings need a human decision, sources are never modified",
              "rules": {"isolated-detour": {"offset_m_gt": DETOUR_OFFSET_M, "detour_ratio_gt": DETOUR_RATIO_MIN,
                                            "neighbor_chord_m_le": DETOUR_SPAN_MAX_M},
                        "lateral-spike": {"turn_deg_ge": SPIKE_TURN_MIN_DEG,
                                          "neighbour_turn_without_point_deg_le": SPIKE_NEIGHBOUR_TURN_MAX_DEG,
                                          "offset_m_ge": SPIKE_OFFSET_MIN_M}},
              "cases": [{"case": label, **review_file(ROOT / "v2x_map_xml" / name)} for label, name in CASES]}
    text = json.dumps(report, ensure_ascii=False, indent=1)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(text, encoding="utf-8")
    for case in report["cases"]:
        for f in case["findings"]:
            print(f"{case['case']}: {f['rule']} {f['source_lane_id']} point {f['point_index_0based']} "
                  f"{f['distance_to_neighbor_chord_m']:.2f} m off a {f['neighbor_chord_length_m']:.1f} m chord")
    return 0


if __name__ == "__main__":
    sys.exit(main())
