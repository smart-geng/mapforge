"""Approved, versioned corrections of raw MAP XML sources (never silent, never in place).

A correction decision (profiles/source-corrections/*.yaml) is bound to the exact
source bytes (SHA256), to the point identity (node / link / lane / index) and to
the raw integer coordinates. Applying it writes a *derived* XML next to the run;
the original file is never modified. Evidence is recomputed with the same
isolated-detour rule and must agree with the decision, otherwise nothing is
applied. Every dropped point is reported with status DROPPED, so loss reports and
scoreboards show exactly what the conversion did not use.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import yaml
from lxml import etree

ROOT = Path(__file__).resolve().parents[2]
DECISIONS = ROOT / "profiles" / "source-corrections"
SCHEMA = "mapforge/source-corrections/v1"
EVIDENCE_REL_TOL = 0.01


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def applicable(xml_path, decisions_dir: Path = DECISIONS) -> list[dict]:
    """Decisions whose source file and SHA256 match ``xml_path`` exactly."""
    xml_path = Path(xml_path)
    digest = _sha256(xml_path)
    found = []
    for path in sorted(Path(decisions_dir).glob("*.yaml")):
        decision = yaml.safe_load(path.read_text(encoding="utf-8"))
        if Path(decision["source"]["file"]).name == xml_path.name and decision["source"]["sha256"] == digest:
            found.append({**decision, "_path": str(path.relative_to(ROOT))})
    return found


def _local_xy(lon_e7, lat_e7, lon0, lat0):
    r = 6378137.0
    x = math.radians((lon_e7 - lon0) * 1e-7) * r * math.cos(math.radians(lat0 * 1e-7))
    y = math.radians((lat_e7 - lat0) * 1e-7) * r
    return x, y


def _detour(points, i):
    """Distance to the neighbour chord, chord length and detour ratio of point ``i``."""
    (ax, ay), (px, py), (bx, by) = points[i - 1], points[i], points[i + 1]
    bxa, bya = bx - ax, by - ay
    span = math.hypot(bxa, bya)
    t = max(0.0, min(1.0, ((px - ax) * bxa + (py - ay) * bya) / (span * span)))
    offset = math.hypot(px - (ax + t * bxa), py - (ay + t * bya))
    detour = math.hypot(px - ax, py - ay) + math.hypot(bx - px, by - py)
    return offset, span, detour / span


def _find_lane(root, target):
    for node in root.findall("mapFrame/nodes/Node"):
        if (int(node.findtext("id/region", "-1")), int(node.findtext("id/id", "-1"))) != (
                target["node"]["region"], target["node"]["id"]):
            continue
        for link in node.findall("inLinks/Link"):
            up = (int(link.findtext("upstreamNodeId/region", "-1")), int(link.findtext("upstreamNodeId/id", "-1")))
            if (link.findtext("name") or "").strip() != target["link"]["name"] or up != (
                    target["link"]["upstream"]["region"], target["link"]["upstream"]["id"]):
                continue
            for lane in link.findall("lanes/Lane"):
                if int(lane.findtext("laneID", "-1")) == target["lane"]:
                    return lane
    raise ValueError("correction target lane not found in source")


def _raw(rp):
    ll = rp.find("posOffset/offsetLL/position-LatLon")
    return int(ll.findtext("lon").strip()), int(ll.findtext("lat").strip())


def apply(xml_path, decisions: list[dict], out_path) -> dict:
    """Write the corrected derivative of ``xml_path`` to ``out_path``; return the DROPPED record."""
    xml_path, out_path = Path(xml_path), Path(out_path)
    original = _sha256(xml_path)
    tree = etree.parse(str(xml_path), etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    items = []
    for decision in decisions:
        if decision["source"]["sha256"] != original:
            raise ValueError(f"{decision['id']}: source bytes changed; correction not applied")
        for c in decision["corrections"]:
            if c["action"] != "drop_point":
                raise ValueError(f"{decision['id']}: unsupported action {c['action']}")
            target = c["target"]
            lane = _find_lane(root, target)
            points = lane.findall("points/RoadPoint")
            i = target["point_index_0based"]
            if not 0 < i < len(points) - 1:
                raise ValueError(f"{decision['id']}: only interior points can be dropped")
            raw = [_raw(rp) for rp in points]
            if raw[i] != (target["raw"]["lon"], target["raw"]["lat"]):
                raise ValueError(f"{decision['id']}: raw coordinates differ from the decision")
            xy = [_local_xy(lon, lat, raw[i - 1][0], raw[i - 1][1]) for lon, lat in raw]
            offset, span, ratio = _detour(xy, i)
            expected = c["evidence"]
            for got, key in ((offset, "distance_to_neighbor_chord_m"), (span, "neighbor_chord_length_m"),
                             (ratio, "detour_ratio")):
                if abs(got - expected[key]) > EVIDENCE_REL_TOL * abs(expected[key]):
                    raise ValueError(f"{decision['id']}: recomputed {key} {got:.3f} disagrees with decision")
            points[i].getparent().remove(points[i])
            items.append({
                "status": c["status"], "what": "raw MAP RoadPoint dropped by an approved source correction",
                "decision": decision["id"], "decision_file": decision["_path"], "approved": decision["approved"],
                "source_lane_id": (f"map:{target['node']['region']}:{target['node']['id']}:from:"
                                   f"{target['link']['upstream']['region']}:{target['link']['upstream']['id']}:"
                                   f"{target['link']['name']}:lane:{target['lane']}"),
                "point_index_0based": i, "raw": target["raw"],
                "evidence": {"distance_to_neighbor_chord_m": offset, "neighbor_chord_length_m": span,
                             "detour_ratio": ratio, "rule": expected.get("rule")},
                "reason": c.get("reason"),
            })
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(out_path), xml_declaration=True, encoding="UTF-8")
    summary = {}
    for item in items:
        summary[item["status"]] = summary.get(item["status"], 0) + 1
    return {"schema": SCHEMA, "source": {"file": xml_path.name, "sha256": original},
            "derived": {"file": str(out_path), "sha256": _sha256(out_path)},
            "summary": summary, "items": items}


def write_loss_report(record: dict, path) -> None:
    """Loss report in the delivery-package shape: {summary, items}."""
    Path(path).write_text(json.dumps({"summary": record["summary"], "items": record["items"],
                                      "source": record["source"], "derived": record["derived"]},
                                     ensure_ascii=False, indent=2), encoding="utf-8")
