"""SHP source consistency review (read-only): places where the source contradicts itself, recorded as source issues.

User decision (2026-10-04, boundary priority): lane edges follow the SHP lane boundaries and a lane centre is the
midpoint of its edges. Where the SHP lane centreline or neighbouring lane objects disagree with that, the
difference is recorded here; nothing is corrected and nothing blocks. The G8 lane-centre comparison uses the
SHP centreline and the boundary gate uses the boundaries, so where a finding lies the two cannot both be met.
Rules, in the conversion's local comparison frame (metres):

- ``centreline-off-midpoint``: the lane centreline lies more than CENTRE_OFFSET_MIN_M off the midpoint of its
  two boundaries, where the lane is at least FULL_WIDTH_MIN_M and FULL_WIDTH_SHARE of its own width wide and the
  centreline point sees both boundaries
  inside their extent. shp-node18 road 11 lane 4: 0.43-0.48 m on a 3.43 m lane whose converted edges match the
  boundaries within 4 cm. Narrower stretches are left out: the centreline of a lane born (ending) at zero width
  runs from (to) its neighbour's centre while its boundaries meet at the edge, a convention of this data, and
  such tapers are not G8-comparable.
- ``lane-object-step``: a lane object and its TOPO successor meet (end points within JOINT_MAX_M) but their
  centrelines, or their boundaries on the same side, are more than STEP_MIN_M apart across the travel direction
  at the joint. Joints where either lane is narrower than ZERO_WIDTH_M (a birth or a death) are left out.
- ``lane-object-kink``: a lane object and its TOPO successor meet at an angle of KINK_MIN_DEG or more (centreline
  headings extrapolated to the joint from 4 m on either side, joint_heading). At a junction a connector has to leave and reach its
  linked lanes along their own direction, so a via line that meets them at an angle cannot be followed there:
  shp-node4 via ..698050 reaches its departure lane at 11 deg, the connector, which ends tangent to the
  departure road 1.1 m before that joint, lies 0.22 m off the via at its end and 0.66 m before it.
- ``shared-boundary-mismatch``: two lanes the conversion puts side by side (adjacent lanes of an XODR lane
  section; without the XODR, adjacent SEQ of one link) do not share their common boundary: the nearer of their
  boundaries lie more than SHARED_GAP_MIN_M apart (a gap or an overlap). The common edge can only lie between
  them. shp-node13 road 10 left lanes 3/4 belong to two parallel links with a 1 m gap between them: the edge
  lies in the middle, both lanes come out about 0.5 m wider, lane 3's centre 0.26 m off its centreline and so
  are the connectors ending on it.
- ``mouth-curb-flare`` (with the XODR; 2026-10-05): at a junction mouth the source lane widens towards the curb
  return and its centreline swings outward with it: at the lane's junction end it is at least FLARE_WIDEN_MIN_M
  wider than its regular width and its centreline at least FLARE_SWING_MIN_M farther out from its inner boundary
  than its regular position (medians FLARE_REGULAR_M from the mouth; both measured from the inner boundary, so a
  road bend or a lateral shift of the whole road counts for nothing; a lane object too short for that is
  continued by its TOPO neighbour away from the junction). The via lines start or end on the swung-out point.
  The converted road keeps a regular cross-section at the mouth (straight run, small edge slope: the connector
  edges have to meet it in G2) and a connector leaves and reaches it tangentially, so neither follows the swing:
  shp-node17 road 10 lane -3 is 1.95 m wider at its end, its centreline 0.98 m farther out, and connectors 103 to
  106 leaving it lie 0.3-0.6 m off their via windows in their first metres. User decision (2026-10-05): recorded as
  a source characteristic, not followed; the findings carry the mouth point and the connector lanes attached
  there, and the scoreboard reports lane-centre statistics without the FLARE_ZONE_M around such mouths
  (mapforge.validate.lane_centre_flare), not graded.
- ``tight-turn-source-conflict`` (2026-10-09): a comparable junction via whose source window turns tighter than a
  5 m radius within any 3 m (mapforge.validate.tight_turn_conflict). User decision: a source conflict, recorded,
  not followed; SHP T2 grades the lane centres without these connector lanes (0.8-draft). Only added when found,
  so a review without such a via keeps its bytes.

    python -m mapforge.validate.shp_source_review --manifest out/.../shp-node4.source-lanes.json --out review.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from mapforge.validate.shp_boundary_fidelity import _densify, _point_polyline_distance, _project

ROOT = Path(__file__).resolve().parents[2]
CODE = "mapforge.shp-source-review/v1"
CENTRE_OFFSET_MIN_M = 0.15   # G8 lane-centre P95 threshold: a larger source offset alone takes it
STEP_MIN_M = 0.10            # lateral step at a lane-object joint
JOINT_MAX_M = 5.0            # end points farther apart than this are a gap, not a joint
FULL_WIDTH_MIN_M = 2.5       # centreline offsets are judged where the lane is at least this wide
FULL_WIDTH_SHARE = 0.9       # ... and at least this share of its own (90th percentile) width
ZERO_WIDTH_M = 0.3           # a lane end narrower than this is a birth/death, not a joint
SHARED_GAP_MIN_M = 0.10      # neighbouring lanes' common boundaries further apart than this
KINK_MIN_DEG = 5.0           # lane objects meeting at this angle or more
TANGENT_BASE_M = 2.0         # direction at a joint from this much of the lane's end
SAMPLE_M = 0.5
FLARE_WIDEN_MIN_M = 0.30     # a lane end this much wider than the lane's regular width ...
FLARE_SWING_MIN_M = 0.15     # ... whose centreline lies this much farther out (the G8 lane-centre P95 threshold)
FLARE_REGULAR_M = (6.0, 20.0)  # regular width and centre position: medians over this distance from the mouth
FLARE_REGULAR_MIN = 6        # ... from at least this many samples
FLARE_FROM_M = 0.05          # the flare starts where the centreline is this much farther out than regular
FLARE_NEAR_M = 25.0          # (lane points farther than the regular stretch plus this from the mouth are not looked at)
FLARE_ZONE_M = 10.0          # lane-centre samples this close to a flared mouth are left out of the flare-free
                             # report statistics (the connector end zone of the user decision of 2026-10-05)


def _local(points_deg, origin):
    return _project(np.asarray(points_deg, float)[:, :2], origin["lat"], origin["lon"])


def _lonlat(xy, origin):
    lat0, lon0 = origin["lat"], origin["lon"]
    r = 6378137.0
    return {"lon": lon0 + math.degrees(xy[0] / (r * math.cos(math.radians(lat0)))),
            "lat": lat0 + math.degrees(xy[1] / r)}


def _at(xy, origin):
    return {"x": round(float(xy[0]), 3), "y": round(float(xy[1]), 3),
            **{k: round(v, 9) for k, v in _lonlat(xy, origin).items()}}


def _foot(points, line):
    """Nearest points on a polyline and whether they lie inside it (not on its first/last end point)."""
    p = np.asarray(points, float)
    q = np.asarray(line, float)
    a, v = q[:-1], q[1:] - q[:-1]
    vv = np.maximum(np.einsum("ij,ij->i", v, v), 1e-16)
    d = p[:, None, :] - a[None, :, :]
    t = np.clip(np.einsum("bsi,si->bs", d, v) / vv[None, :], 0.0, 1.0)
    f = a[None, :, :] + t[:, :, None] * v[None, :, :]
    idx = np.argmin(np.einsum("bsi,bsi->bs", p[:, None, :] - f, p[:, None, :] - f), axis=1)
    row = np.arange(len(p))
    inside = ~(((idx == 0) & (t[row, idx] <= 1e-9)) | ((idx == len(a) - 1) & (t[row, idx] >= 1 - 1e-9)))
    return f[row, idx], inside


def centreline_offset(centre, boundaries):
    """(offsets, centre samples, widths) where the samples see both boundaries inside their extent; the offset
    is the distance of the centreline from the midpoint of the nearest points on the two boundaries (also where
    the centreline runs outside the lane: some taper centrelines go straight while the boundaries converge)."""
    c = _densify(centre, SAMPLE_M)
    f1, in1 = _foot(c, boundaries[0])
    f2, in2 = _foot(c, boundaries[1])
    keep = in1 & in2
    off = np.linalg.norm(c - 0.5 * (f1 + f2), axis=1)
    return off[keep], c[keep], np.linalg.norm(f1 - f2, axis=1)[keep]


def off_midpoint(rec, centre, bnds):
    """The centreline-off-midpoint judgement of one lane: (offsets, centre samples, widths) on the stretch where the
    lane is at least FULL_WIDTH_MIN_M and FULL_WIDTH_SHARE of its own width wide, or None where the rule does not
    apply (no field centreline, not two boundaries) or no offset reaches CENTRE_OFFSET_MIN_M."""
    if centre is None or len(bnds) != 2 or rec is None or rec.geometry_source != "field":
        return None
    off, pts, width = centreline_offset(centre, bnds)
    full = width >= max(FULL_WIDTH_MIN_M, FULL_WIDTH_SHARE * float(np.percentile(width, 90))) if len(width) else width > 0
    off, pts, width = off[full], pts[full], width[full]
    if not len(off) or float(off.max()) < CENTRE_OFFSET_MIN_M:
        return None
    return off, pts, width


def _end(line, at_start):
    """End point and outward unit tangent (from TANGENT_BASE_M of the line) of one end."""
    g = line if not at_start else line[::-1]
    p = g[-1]
    seg = np.linalg.norm(np.diff(g, axis=0), axis=1)
    back = np.concatenate([[0.0], np.cumsum(seg[::-1])])
    j = int(np.searchsorted(back, TANGENT_BASE_M))
    q = g[max(len(g) - 1 - min(j, len(g) - 1), 0)]
    if np.linalg.norm(p - q) < 1e-9:
        q = g[-2]
    u = (p - q) / max(np.linalg.norm(p - q), 1e-12)
    return p, u


def joint_heading(line, at_start, base=4.0):
    """Travel heading [rad] of ``line`` at one end (its start or end), extrapolated to the end point: segment
    headings within ``base`` metres of that end, fitted linearly against the distance from it, so a steady curve
    right at a joint is not taken for a kink."""
    g = np.asarray(line, float)
    if at_start:
        g = g[::-1]                       # walk from the far side towards the end in question
    seg = np.diff(g, axis=0)
    length = np.linalg.norm(seg, axis=1)
    keep = length > 1e-6
    seg, length = seg[keep], length[keep]
    h = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    dist = np.concatenate([np.cumsum(length[::-1])[::-1][1:], [0.0]]) + 0.5 * length   # segment middle to the end
    near = dist <= base
    if near.sum() >= 2 and np.ptp(dist[near]) > 1e-6:
        slope, intercept = np.polyfit(dist[near], h[near], 1)
        heading = intercept
    else:
        heading = h[-1]
    return heading + (math.pi if at_start else 0.0)


def joint_step(a, b):
    """Lateral and longitudinal offset of b's end nearest to a, against a's direction at that end."""
    best = None
    for a_start in (False, True):
        for b_start in (True, False):
            pa = a[0] if a_start else a[-1]
            pb = b[0] if b_start else b[-1]
            d = float(np.linalg.norm(pb - pa))
            if best is None or d < best[0]:
                best = (d, a_start, b_start)
    d, a_start, b_start = best
    pa, u = _end(a, a_start)
    pb = b[0] if b_start else b[-1]
    v = pb - pa
    return {"distance_m": d, "lateral_m": float(u[0] * v[1] - u[1] * v[0]), "longitudinal_m": float(u @ v),
            "a_start": a_start, "b_start": b_start, "point": pa}


def neighbour_pairs(root):
    """(source lane, source lane) of lanes side by side in the ordinary roads of an XODR (inner first)."""
    pairs = []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        for sec in road.findall("lanes/laneSection"):
            for side in ("left", "right"):
                ids = []
                for ln in sorted(sec.findall(f"{side}/lane"), key=lambda x: abs(int(x.get("id")))):
                    u = ln.find("userData[@code='mapforge.source_lane']")
                    ids.append(u.get("value") if u is not None else None)
                for a, b in zip(ids, ids[1:]):
                    if a and b and a != b and (a, b) not in pairs:
                        pairs.append((a, b))
    return pairs


def _shared_boundaries(src, lane_ids, geoms, origin, meta, pairs=None):
    """shared-boundary-mismatch findings for the lane pairs ``pairs`` (default: adjacent SEQ of one link)."""
    if pairs is None:
        by_link, pairs = {}, []
        for sid in lane_ids:
            rec = src.lane(sid)
            if rec is not None and getattr(rec, "seq", None) is not None:
                by_link.setdefault(rec.link_pid, []).append((rec.seq, sid))
        for members in by_link.values():
            members.sort()
            pairs += [(a, b) for (sa, a), (sb, b) in zip(members, members[1:]) if sb == sa + 1]
    out = []
    for a, b in pairs:
        if a not in geoms or b not in geoms or len(geoms[a][1]) != 2 or len(geoms[b][1]) != 2:
            continue
        best = None
        for ga in geoms[a][1]:
            pts = _densify(ga, SAMPLE_M)
            for gb in geoms[b][1]:
                d, inside = _point_polyline_distance(pts, gb)
                if inside.sum() < 4:
                    continue
                med = float(np.median(d[inside]))
                if best is None or med < best[0]:
                    best = (med, d[inside], pts[inside])
        if best is None or float(np.max(best[1])) < SHARED_GAP_MIN_M:
            continue
        d, pts = best[1], best[2]
        i = int(np.argmax(d))
        rec_a, rec_b = src.lane(a), src.lane(b)
        out.append({"rule": "shared-boundary-mismatch", "status": "RECORDED", "resolution": "boundary-priority",
                    "source_lane_id": a, "neighbour": b, "link": rec_a.link_pid if rec_a is not None else None,
                    "neighbour_link": rec_b.link_pid if rec_b is not None else None, **meta.get(a, {}),
                    "neighbour_roles": meta.get(b, {}).get("roles", []),
                    "max_m": round(float(d[i]), 3), "median_m": round(best[0], 3),
                    "p95_m": round(float(np.percentile(d, 95)), 3),
                    "over_threshold_m": round(float((d >= SHARED_GAP_MIN_M).sum() * SAMPLE_M), 1),
                    "at": _at(pts[i], origin)})
    return out


def review(src, lane_ids, origin, meta=None, pairs=None) -> dict:
    """Findings for the source lanes ``lane_ids`` (and their TOPO joints among them). ``meta`` {lane id: {"roles",
    "g8_comparable"}} from the conversion's manifest is copied into the findings: findings on junction-via lanes
    (connectors) do not bear on the road-side boundaries, findings on G8-comparable lanes bear on G8."""
    lane_ids = sorted(set(lane_ids))
    meta = meta or {}
    findings = []
    geoms = {}
    for sid in lane_ids:
        rec = src.lane(sid)
        bnds = [_local(g, origin) for g in src.lane_boundary_geometries(sid)]
        centre = _local(rec.geometry, origin) if rec is not None and len(rec.geometry) >= 2 else None
        geoms[sid] = (centre, bnds)
        judged = off_midpoint(rec, centre, bnds)
        if judged is None:
            continue
        off, pts, width = judged
        i = int(np.argmax(off))
        findings.append({
            "rule": "centreline-off-midpoint", "status": "RECORDED", "resolution": "boundary-priority",
            "source_lane_id": sid, "link": rec.link_pid, **meta.get(sid, {}),
            "max_m": round(float(off[i]), 3), "p95_m": round(float(np.percentile(off, 95)), 3),
            "over_threshold_m": round(float((off >= CENTRE_OFFSET_MIN_M).sum() * SAMPLE_M), 1),
            "lane_width_m": round(float(width[i]), 3), "at": _at(pts[i], origin)})
    findings += _shared_boundaries(src, lane_ids, geoms, origin, meta, pairs)
    topo = src.topo_out
    for sid in lane_ids:
        ca, ba = geoms[sid]
        for nxt in topo.get(sid, []):
            if nxt not in geoms:
                continue
            cb, bb = geoms[nxt]
            if ca is None or cb is None:
                continue
            j = joint_step(ca, cb)
            if j["distance_m"] > JOINT_MAX_M:
                continue
            # a's heading into the joint and b's heading out of it (either may be digitized the other way)
            ha = joint_heading(ca, j["a_start"]) + (math.pi if j["a_start"] else 0.0)
            hb = joint_heading(cb, j["b_start"]) + (0.0 if j["b_start"] else math.pi)
            kink = math.degrees((hb - ha + math.pi) % (2 * math.pi) - math.pi)
            if abs(kink) >= KINK_MIN_DEG:
                findings.append({
                    "rule": "lane-object-kink", "status": "RECORDED", "resolution": "lane-priority",
                    "source_lane_id": sid, "successor": nxt, **meta.get(sid, {}),
                    "successor_roles": meta.get(nxt, {}).get("roles", []),
                    "kink_deg": round(kink, 2), "at": _at(j["point"], origin)})
            bnd_lateral = []
            if len(ba) == 2 and len(bb) == 2:
                ends_a = [min((g[0], g[-1]), key=lambda x: float(np.linalg.norm(x - j["point"]))) for g in ba]
                pb_ = cb[0] if j["b_start"] else cb[-1]
                ends_b = [min((g[0], g[-1]), key=lambda x: float(np.linalg.norm(x - pb_))) for g in bb]
                if (float(np.linalg.norm(ends_a[0] - ends_a[1])) < ZERO_WIDTH_M
                        or float(np.linalg.norm(ends_b[0] - ends_b[1])) < ZERO_WIDTH_M):
                    continue             # a death or a birth at the joint
                _, u = _end(ca, j["a_start"])
                for g in ba:
                    # a's boundary end at the joint, against the nearer of b's boundaries (its nearest point)
                    p = min((g[0], g[-1]), key=lambda x: float(np.linalg.norm(x - j["point"])))
                    feet = [_foot(p[None, :], h)[0][0] for h in bb]
                    q = min(feet, key=lambda x: float(np.linalg.norm(x - p)))
                    v = q - p
                    if float(np.linalg.norm(v)) <= JOINT_MAX_M:
                        bnd_lateral.append(float(u[0] * v[1] - u[1] * v[0]))
            worst = max([abs(j["lateral_m"])] + [abs(x) for x in bnd_lateral])
            if worst < STEP_MIN_M:
                continue
            findings.append({
                "rule": "lane-object-step", "status": "RECORDED", "resolution": "boundary-priority",
                "source_lane_id": sid, "successor": nxt, **meta.get(sid, {}),
                "successor_roles": meta.get(nxt, {}).get("roles", []),
                "centreline_lateral_m": round(j["lateral_m"], 3),
                "centreline_longitudinal_m": round(j["longitudinal_m"], 3),
                "boundary_lateral_m": [round(x, 3) for x in bnd_lateral],
                "max_m": round(worst, 3), "at": _at(j["point"], origin)})
    counts = {}
    for f in findings:
        on_road = any(r != "junction-via" for r in f.get("roles", [])) or any(
            r != "junction-via" for r in f.get("successor_roles", []) + f.get("neighbour_roles", []))
        key = f"{f['rule']}/{'road' if on_road else 'junction-via'}"
        counts[key] = counts.get(key, 0) + 1
    return {"schema": CODE, "decision": "boundary-priority (user, 2026-10-04): edges follow the SHP boundaries, "
                                        "lane centre = edge midpoint, differences recorded here, nothing corrected",
            "thresholds": {"centre_offset_min_m": CENTRE_OFFSET_MIN_M, "step_min_m": STEP_MIN_M,
                           "joint_max_m": JOINT_MAX_M, "shared_gap_min_m": SHARED_GAP_MIN_M,
                           "kink_min_deg": KINK_MIN_DEG},
            "lanes_reviewed": len(lane_ids), "counts": counts,
            # every finding falls under the boundary-priority decision: recorded, none open
            "open": 0, "recorded": len(findings), "findings": findings}


def _road_frame(road):
    """Projector of local points onto ``road``'s reference line: (s, t) per point, past either end along the end
    heading (s < 0 before the start, s > length past the end); and the sampled reference (points, s, heading)."""
    from mapforge.validate.smoothness import sample_road_ref
    pts, ss, hh = sample_road_ref(road, 0.25)
    a, ab = pts[:-1], pts[1:] - pts[:-1]
    l2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-18)

    def project(points):
        out = []
        for p in np.asarray(points, float):
            u = np.einsum("ij,ij->i", p - a, ab) / l2
            uc = np.clip(u, 0.0, 1.0)
            q = a + uc[:, None] * ab
            k = int(np.argmin(np.linalg.norm(p - q, axis=1)))
            if (k == 0 and u[k] < 0.0) or (k == len(a) - 1 and u[k] > 1.0):
                j = 0 if (k == 0 and u[k] < 0.0) else -1
                e = np.array([math.cos(hh[j]), math.sin(hh[j])])
                v = p - pts[j]
                out.append((ss[j] + float(v @ e), float(e[0] * v[1] - e[1] * v[0])))
            else:
                v = p - q[k]
                out.append((ss[k] + uc[k] * (ss[k + 1] - ss[k]),
                            float(math.cos(hh[k]) * v[1] - math.sin(hh[k]) * v[0])))
        return np.asarray(out, float).reshape(-1, 2)
    return project, pts, ss, hh


def _mouth_centre(road, contact, lane_id, pts, ss, hh):
    """Written lane-centre point of lane ``lane_id`` at a road end (local frame)."""
    from mapforge.validate.smoothness import lane_edges_at
    at = 0.0 if contact == "start" else float(ss[-1]) - 1e-7
    edges = lane_edges_at(road, at, "right" if lane_id < 0 else "left")
    k = abs(lane_id)
    t = 0.5 * (edges[k - 1] + edges[k])
    j = 0 if contact == "start" else -1
    return pts[j] + t * np.array([-math.sin(hh[j]), math.cos(hh[j])])


def _attached(root, road_id, contact, lane_id):
    """Connectors leaving or reaching lane ``lane_id`` at that road end: [(road id, source id of its lane -1)]."""
    out = []
    for conn in root.findall("road"):
        if conn.get("junction") in (None, "-1") or conn.get("name") == "junction_paving":
            continue
        lane = conn.find("lanes/laneSection/right/lane[@id='-1']")
        if lane is None:
            continue
        for tag in ("predecessor", "successor"):
            link, lane_link = conn.find(f"link/{tag}"), lane.find(f"link/{tag}")
            if (link is not None and lane_link is not None and link.get("elementType") == "road"
                    and link.get("elementId") == road_id and link.get("contactPoint") == contact
                    and int(lane_link.get("id")) == lane_id):
                u = lane.find("userData[@code='mapforge.source_lane']")
                out.append((conn.get("id"), u.get("value") if u is not None else None))
    return out


def _along(line, d):
    """Lateral offset of a (distance, offset) sample set at distance ``d`` (nan outside it)."""
    o = np.argsort(line[:, 0])
    x, y = line[o, 0], line[o, 1]
    if d < x[0] - SAMPLE_M or d > x[-1] + SAMPLE_M:
        return float("nan")
    return float(np.interp(d, x, y))


def mouth_flares(root, src, origin, meta=None) -> list:
    """mouth-curb-flare findings at the junction ends of the converted roads (module doc)."""
    meta = meta or {}
    findings = []
    lo, hi = FLARE_REGULAR_M
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        frame = None
        for contact, tag in (("start", "predecessor"), ("end", "successor")):
            link = road.find(f"link/{tag}")
            if link is None or link.get("elementType") != "junction":
                continue
            if frame is None:
                frame = _road_frame(road)
            project, pts, ss, hh = frame
            length = float(ss[-1])
            mouth_ref = pts[0] if contact == "start" else pts[-1]
            sec = road.findall("lanes/laneSection")[0 if contact == "start" else -1]
            for lane in sec.iter("lane"):
                u = lane.find("userData[@code='mapforge.source_lane']")
                rec = src.lane(u.get("value")) if u is not None else None
                if rec is None or len(rec.geometry) < 2:
                    continue
                sid, lane_id = u.get("value"), int(lane.get("id"))
                side = -1.0 if lane_id < 0 else 1.0
                # right lanes run along s: they reach an end mouth (approach) and leave a start mouth (departure)
                approach = (lane_id < 0) == (contact == "end")

                def lines(lane_pid, project=project, contact=contact, length=length, side=side, mouth_ref=mouth_ref):
                    """(centreline, inner boundary, outer boundary) as (distance from the mouth, offset) samples
                    (only points near the mouth: the regular stretch ends FLARE_REGULAR_M[1] into the road)"""
                    r = src.lane(lane_pid)
                    if r is None or len(r.geometry) < 2:
                        return None
                    out = []
                    for g in [r.geometry] + list(src.lane_boundary_geometries(lane_pid)):
                        q = _densify(_local(g, origin), SAMPLE_M)
                        q = q[np.linalg.norm(q - mouth_ref, axis=1) <= hi + FLARE_NEAR_M]
                        if len(q) < 2:
                            return None
                        st = project(q)
                        out.append(np.column_stack([st[:, 0] if contact == "start" else length - st[:, 0], st[:, 1]]))
                    if len(out) != 3:
                        return None
                    inner, outer = sorted(out[1:], key=lambda b: side * float(np.median(b[:, 1])))
                    return out[0], inner, outer

                own = lines(sid)
                if own is None:
                    continue
                parts = [own]
                if int(((own[0][:, 0] >= lo) & (own[0][:, 0] <= hi)).sum()) < FLARE_REGULAR_MIN:
                    # continued by the neighbour away from the junction; of several (a split or a merge there),
                    # the one whose centreline carries on from this one's at the joint
                    far = float(own[0][:, 0].max())
                    t_far = _along(own[0], far)
                    nexts = [x for x in (lines(n) for n in (src.topo_in if approach else src.topo_out).get(sid, []))
                             if x is not None and not np.isnan(_along(x[0], far))]
                    if nexts:
                        # (a lane born at the joint starts on this one's centre too: compare 5 m on)
                        parts.append(min(nexts, key=lambda x: abs(_along(x[0], min(far + 5.0, float(x[0][:, 0].max())))
                                                                  - t_far)))
                centre, inner, outer = (np.vstack([x[i] for x in parts]) for i in range(3))
                reg = [d for d in np.arange(lo, hi + 1e-9, SAMPLE_M) if centre[:, 0].min() <= d <= centre[:, 0].max()]
                width = np.array([side * (_along(outer, d) - _along(inner, d)) for d in reg])
                rel = np.array([side * (_along(centre, d) - _along(inner, d)) for d in reg])
                if len(reg) < FLARE_REGULAR_MIN or np.isnan(width).all() or np.isnan(rel).all():
                    continue
                w_reg, c_reg = float(np.nanmedian(width)), float(np.nanmedian(rel))
                # the source lane's junction end (where its centreline and both boundaries still reach)
                d_end = max(float(own[i][:, 0].min()) for i in range(3))
                widen = side * (_along(outer, d_end) - _along(inner, d_end)) - w_reg
                swing = side * (_along(centre, d_end) - _along(inner, d_end)) - c_reg
                if not (widen >= FLARE_WIDEN_MIN_M and swing >= FLARE_SWING_MIN_M):
                    continue
                grid = np.arange(d_end, lo + 1e-9, SAMPLE_M)
                out_by = np.array([side * (_along(centre, d) - _along(inner, d)) - c_reg for d in grid])
                over = grid[out_by >= FLARE_FROM_M]
                mouth_w = side * (_along(outer, 0.0) - _along(inner, 0.0)) - w_reg
                mouth_c = side * (_along(centre, 0.0) - _along(inner, 0.0)) - c_reg
                attached = _attached(root, road.get("id"), contact, lane_id)
                geom = _local(rec.geometry, origin)
                end_xy = geom[-1] if approach else geom[0]
                findings.append({
                    "rule": "mouth-curb-flare", "status": "RECORDED", "resolution": "regular-mouth",
                    "source_lane_id": sid, "link": rec.link_pid, **meta.get(sid, {}),
                    "travel": "approach" if approach else "departure",
                    "road": road.get("id"), "contact": contact, "lane": lane_id,
                    "connectors": [c for c, _ in attached], "connector_lanes": [v for _, v in attached if v],
                    "widening_m": round(widen, 3), "swing_m": round(swing, 3),
                    "mouth_widening_m": None if math.isnan(mouth_w) else round(mouth_w, 3),
                    "mouth_swing_m": None if math.isnan(mouth_c) else round(mouth_c, 3),
                    "lane_end_from_mouth_m": round(d_end, 2),
                    "flare_from_mouth_m": round(float(over.max()), 2) if len(over) else None,
                    "mouth": _at(_mouth_centre(road, contact, lane_id, pts, ss, hh), origin),
                    "at": _at(end_xy, origin)})
    return findings


def review_manifest(manifest: dict, shp_dir=None, profile="ibd-smarteditor-v1", src=None, xodr=None) -> dict:
    """Review of the source lanes one conversion used (its source manifest), in its comparison frame; with the
    converted ``xodr`` (path or root) the lane pairs it puts side by side are checked for a shared boundary."""
    if src is None:
        from mapforge.adapters.shp.profile_source import ProfileSource
        src = ProfileSource(str(shp_dir or ROOT / "shp_0222-0326"), profile)
    origin = manifest["comparison_crs"]["origin"]
    meta = {}
    for ln in manifest["lanes"]:
        m = meta.setdefault(ln["source_lane_id"], {"roles": [], "g8_comparable": False})
        if ln.get("role") and ln["role"] not in m["roles"]:
            m["roles"].append(ln["role"])
        m["g8_comparable"] = m["g8_comparable"] or bool((ln.get("comparison") or {}).get("eligible"))
    pairs, root = None, None
    if xodr is not None:
        from lxml import etree
        root = xodr if hasattr(xodr, "findall") else etree.parse(str(xodr)).getroot()
        pairs = neighbour_pairs(root)
    out = review(src, list(meta) + sorted({x for pair in (pairs or []) for x in pair} - set(meta)), origin, meta,
                 pairs)
    if root is not None:
        flares = mouth_flares(root, src, origin, meta)
        out["findings"] += flares
        if flares:
            out["counts"]["mouth-curb-flare/road"] = len(flares)
        out["recorded"] = len(out["findings"])
        out["thresholds"].update({"flare_widen_min_m": FLARE_WIDEN_MIN_M, "flare_swing_min_m": FLARE_SWING_MIN_M,
                                  "flare_regular_m": list(FLARE_REGULAR_M), "flare_zone_m": FLARE_ZONE_M})
        out["decision_flares"] = ("regular-mouth (user, 2026-10-05): curb-return flares at junction mouths are "
                                  "recorded, not followed; lane-centre statistics without the flare zones are "
                                  "reported apart, not graded")
    from mapforge.validate import tight_turn_conflict as T
    conflicts = T.findings(manifest)
    if conflicts:
        out["findings"] += conflicts
        out["counts"][T.RULE + "/lane"] = len(conflicts)
        out["recorded"] = len(out["findings"])
        out["thresholds"].update({"tight_turn_radius_min_m": T.RADIUS_MIN_M, "tight_turn_window_m": T.WINDOW_M})
        out["decision_tight_turns"] = T.DECISION
    out["comparison_crs"] = manifest["comparison_crs"].get("id")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--manifest", required=True, nargs="+", help="source-lanes.json of SHP conversions")
    ap.add_argument("--out", help="JSON output (one review per manifest); default stdout")
    args = ap.parse_args(argv)
    from mapforge.adapters.shp.profile_source import ProfileSource
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    reviews = {}
    for m in args.manifest:
        x = Path(m.replace(".source-lanes.json", ".xodr"))
        reviews[Path(m).name] = review_manifest(json.loads(Path(m).read_text(encoding="utf-8")), src=src,
                                                xodr=x if x.exists() else None)
    text = json.dumps(reviews, ensure_ascii=False, indent=1)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
