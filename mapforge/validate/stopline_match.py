"""SHP approach lanes whose source stop line is not linked: nearest match, or a recorded source gap.

Two user decisions (2026-10-09), graded by T1 from policy 0.10-draft:

* Nearest match. An at-stopline approach lane without a LANE_REL stop line (manifest reason
  ``source-stopline-not-linked``) takes the source stop line nearest to its source downstream end, when that stop
  line passes within MATCH_RADIUS_M (2 m) of the end: ``stopline-nearest-match``, INFERRED. G8 then measures the
  lane end against it like a linked one (stopline delta <= 1.5 m).
* Source absent. With no source stop line that close (0621: the nearest are 110-120 m away on other roads) the
  lane is recorded ``source-stopline-absent`` and its G8 stop-line requirement is listed apart, not counted.

G8, its policy, the converter and the stored G8 sidecar are unchanged: the original ``g8_status`` and the delivery
decision still report the unlinked lanes. The graded ``g8_status_stopline_resolved`` comes from the same
``evaluate_g8`` run on a copy of the manifest in which matched lanes carry their stop line; an absent lane's
``<id>:stopline`` unavailability is then left out (only that one, never a lane unavailable for another reason) and
the status is derived by G8's own rule (UNAVAILABLE, else FAIL, else PASS). The match uses only the source
stop-line layer and source lane geometry, so stored conversions are scored without converting again.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
G8_POLICY = ROOT / "profiles/validation/g8-opendrive-jinfeng-v1.yaml"
RULE_MATCH = "stopline-nearest-match"
RULE_ABSENT = "source-stopline-absent"
UNLINKED = "source-stopline-not-linked"
MATCH_RADIUS_M = 2.0
R_EARTH = 6378137.0
DECISION = ("nearest-source-stopline (user, 2026-10-09): an approach lane without a linked source stop line takes "
            "the nearest source stop line within 2 m of its source downstream end (INFERRED); with none that close "
            "the source has no stop line for it, recorded, and its G8 stop-line requirement is not graded")


def _proj(points, origin):
    """The manifest's comparison frame (spherical eqc about its origin), as the converter projects."""
    p = np.asarray(points, float)[:, :2]
    lon0, lat0 = float(origin["lon"]), float(origin["lat"])
    return np.column_stack([np.radians(p[:, 0] - lon0) * R_EARTH * math.cos(math.radians(lat0)),
                            np.radians(p[:, 1] - lat0) * R_EARTH])


def source_stoplines(src) -> list[dict]:
    """Every record of the profile's stop-line layer, linked or not (``stoplines_by_lane`` keeps linked ones only)."""
    spec = src.L.get("stop_line")
    if not spec:
        return []
    fields, sep = spec["fields"], spec.get("list_sep", ";")
    out = []
    for index, (record, shape) in enumerate(src._iter(spec)):
        if shape is None or len(shape.points) < 2:
            continue
        out.append({"index": index, "id": str(record.get(fields.get("id"), "") or "").strip(),
                    "lane_refs": [x for x in str(record.get(fields.get("lane_refs"), "") or "").split(sep) if x],
                    "points": np.asarray(shape.points, float)[:, :2]})
    return out


def _downstream_end(lane, raw_xy):
    """The source lane end on the travel-end side of the manifest window."""
    from shapely.geometry import LineString, Point
    line = LineString(raw_xy)
    window = (lane.get("geometry") or {}).get("coordinates") or []
    if len(window) >= 2:
        forward = line.project(Point(window[-1][:2])) >= line.project(Point(window[0][:2]))
    else:
        forward = (lane.get("travel") or {}).get("coordinate_order", "with-travel") == "with-travel"
    return raw_xy[-1] if forward else raw_xy[0]


def findings(manifest: dict, src) -> list[dict]:
    """One finding per unlinked at-stopline lane: the matched stop line, or the recorded absence."""
    from shapely.geometry import LineString, Point
    lanes = [ln for ln in manifest.get("lanes", [])
             if (ln.get("stop_line") or {}).get("availability") == "unavailable"
             and (ln.get("stop_line") or {}).get("reason") == UNLINKED]
    if not lanes:
        return []
    origin = manifest["comparison_crs"]["origin"]
    stops = [dict(s, xy=_proj(s["points"], origin)) for s in source_stoplines(src)]
    for s in stops:
        s["line"] = LineString(s["xy"])
    out = []
    for lane in lanes:
        sid = lane["source_lane_id"]
        rec = src.lane(sid)
        base = {"source_lane_id": sid, "role": lane.get("role"),
                "g8_comparable": bool((lane.get("comparison") or {}).get("eligible"))}
        if rec is None or len(rec.geometry) < 2:
            out.append({"rule": RULE_ABSENT, "status": "RECORDED", **base, "reason": "source-lane-geometry-missing",
                        "resolution": "stopline-requirement-not-graded"})
            continue
        end = Point(_downstream_end(lane, _proj(rec.geometry, origin)))
        ranked = sorted(((s["line"].distance(end), s["index"], s) for s in stops), key=lambda x: x[:2])
        if ranked and ranked[0][0] <= MATCH_RADIUS_M + 1e-9:
            d, _i, s = ranked[0]
            out.append({"rule": RULE_MATCH, "status": "INFERRED", **base, "stopline_id": s["id"],
                        "stopline_record_index": s["index"], "stopline_lane_refs": s["lane_refs"],
                        "distance_m": round(float(d), 3), "resolution": "nearest-source-stopline",
                        "geometry": {"type": "LineString", "coordinates": s["xy"].tolist()}})
        else:
            near = ranked[0] if ranked else None
            out.append({"rule": RULE_ABSENT, "status": "RECORDED", **base,
                        "nearest_stopline_id": near[2]["id"] if near else None,
                        "nearest_distance_m": round(float(near[0]), 3) if near else None,
                        "resolution": "stopline-requirement-not-graded"})
    return out


def resolved_manifest(manifest: dict, found: list[dict]) -> dict:
    """A copy whose matched lanes carry their stop line; every other lane is left as it was."""
    out = copy.deepcopy(manifest)
    by_id = {f["source_lane_id"]: f for f in found if f["rule"] == RULE_MATCH}
    for lane in out.get("lanes", []):
        f = by_id.get(lane["source_lane_id"])
        if f is not None:
            lane["stop_line"] = {"availability": "available", "source_id": f["stopline_id"],
                                 "association": "INFERRED", "rule": RULE_MATCH, "distance_m": f["distance_m"],
                                 "geometry": f["geometry"]}
    return out


def resolved_status(gate: dict, absent: set[str], policy_errors: list[str]) -> tuple[str, list[str]]:
    """G8's status rule after leaving out the stop-line unavailability of source-absent lanes."""
    issues = gate.get("issues") or {}
    unmeasurable = set(issues.get("unmeasurable_source_ids") or [])
    unknown = {x.get("source_lane_id") for x in issues.get("unknown_policy_classes") or []}
    waived = sorted(sid for sid in absent
                    if f"{sid}:stopline" in unmeasurable and sid not in unmeasurable and sid not in unknown)
    left_out = {f"unavailable:{sid}" for sid in waived}
    reasons = [r for r in gate.get("failure_reasons") or [] if r not in left_out]
    if policy_errors or any(r.startswith("unavailable:") for r in reasons):
        return "UNAVAILABLE", waived
    return ("FAIL" if reasons else "PASS"), waived


def audit(xodr, manifest: dict | None, g8: dict | None, src, policy=G8_POLICY) -> dict:
    """G8 status with unlinked stop lines resolved, plus counts. Unchanged inputs give the stored G8 status."""
    if not manifest or not g8:
        return {}
    found = [f for f in findings(manifest, src) if f["g8_comparable"]]
    matched = [f for f in found if f["rule"] == RULE_MATCH]
    absent = {f["source_lane_id"] for f in found if f["rule"] == RULE_ABSENT}
    out = {"stopline_matched": len(matched),
           "stopline_source_absent": len(absent),
           "stopline_match_distance_max_m": max((f["distance_m"] for f in matched), default=None),
           "stopline_matched_delta_max_m": None}
    if not found:
        out.update(g8_status_stopline_resolved=g8.get("status"), g8_stopline_requirements_left_out=0)
        return out
    from mapforge.validate.g8_model import load_policy, validate_policy
    from mapforge.validate.lane_fidelity import evaluate_g8
    loaded = load_policy(policy)
    if (g8.get("policy") or {}).get("sha256") != loaded.get("policy_sha256"):
        out["g8_status_stopline_resolved"] = "UNAVAILABLE"      # stored G8 was run under another policy
        out["stopline_resolution_error"] = "g8-policy-mismatch"
        return out
    gate = evaluate_g8(xodr, resolved_manifest(manifest, found), loaded)
    ids = {f["source_lane_id"] for f in matched}
    deltas = [r["stopline"]["delta_m"] for r in gate.get("per_lane", [])
              if r["source_lane_id"] in ids and r["stopline"].get("delta_m") is not None]
    out["stopline_matched_delta_max_m"] = max(deltas, default=None)
    out["g8_status_stopline_resolved"], waived = resolved_status(gate, absent, validate_policy(loaded))
    out["g8_stopline_requirements_left_out"] = len(waived)
    return out
