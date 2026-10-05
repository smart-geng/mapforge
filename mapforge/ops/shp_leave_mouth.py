"""SHP departure carriageways: the mouth moved past the via joints and the curb-return flare (2026-10-05).

The converter builds a separate departure carriageway (a leave link without an approach on its leg: node3 roads
30 and 31, node4 road 30) from its source link and extends it 1 m into the junction, so that road and paving
overlap (shp_to_xodr.build_single_leave). The via lines of the connectors ending there meet the departure lanes
at the link start, so each joint lies about 1.1 m inside the road: a connector has to reach the departure
heading before the mouth and cannot round the joint's kink (10-20 deg) on both sides; node3 connectors 105,
106, 113, 114 and node4 105 lay 0.5-0.9 m off their via lines. On the approaches the envelope mouth sits 3 m
before the earliest real lane end, so every joint there already lies about 3 m inside its connector.

Here such a road starts ROOM_M past the last irregular point of the source paths ending on it instead: its farthest
via joint, or the end of the curb-return flare of its source lanes where that lies further (plan, flare_end). The
outer departure lane of node3 roads 30 and 31 widens by 0.4-0.9 m towards the junction over its first 3.5-4.5 m
(node4 road 30: about 0.2 m over 1.5 m). With the mouth 3 m past the joints, inside the flare, the connectors
ending on that lane had to end where its source centreline still swings across (node3 connectors 104 and 125:
0.19-0.25 m off); with the mouth right at the flare end they had no room left to straighten (0.14-0.29 m).
3 m further the road starts on a regular cross-section, and every connector keeps 3 m of regular source lane past
its last kink, as on the approaches, where the envelope mouth sits 3 m before the earliest real lane end.

* its first metres are cut off (reference line, lane records, map_stop_line_mouth.cut_start; the lanes'
  support_s move with it);
* every connector ending there is lengthened by as much along its end heading (mouth_frame_align then refits
  it onto the new mouth and its source, as any connector);
* the comparison windows follow the targets (the source manifest says which source part each target is compared
  with): a connector's via window runs on along its via line and the departure lane up to the new mouth, a
  departure lane's window starts there; their geometry hashes are renewed and the change is recorded on them;
* the junction paving is rebuilt afterwards with the moved roads as mouths of their own (rebuild_paving), so the
  strip between the old and the new road start is paved as the other mouth tails are.
"""
from __future__ import annotations

import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

CODE = "mapforge.shp_leave_mouth/v1"
ROOM_M = 3.0          # regular source lane kept inside each connector past its last kink (as the envelope margin)
MAX_SHIFT_M = 10.0
MIN_ROAD_M = 20.0     # a road keeps at least this length
FLARE_SEARCH_M = 15.0  # a departure lane's boundaries are looked at over this length from their start
FLARE_STEP_M = 0.5
FLARE_SLOPE = 0.05    # a boundary flares while its slope across the road differs from its steady slope by more


def _f(x):
    return repr(float(x))


def _source_id(lane):
    u = lane.find("userData[@code='mapforge.source_lane']")
    return u.get("value") if u is not None else None


def _local(points_deg, origin):
    from mapforge.validate.shp_boundary_fidelity import _project
    return _project(np.asarray(points_deg, float)[:, :2], origin[0], origin[1])


def _ending_on(root, road_id):
    """Connectors whose end is linked to the start of road ``road_id``."""
    out = []
    for conn in root.findall("road"):
        if conn.get("junction") in (None, "-1") or conn.get("name") == "junction_paving":
            continue
        succ = conn.find("link/successor")
        if (succ is not None and succ.get("elementType") == "road" and succ.get("elementId") == road_id
                and succ.get("contactPoint") == "start"):
            out.append(conn)
    return out


def _station(pts, ss, hh, point):
    """Signed station of ``point`` along a sampled reference (negative: before its start)."""
    from mapforge.ops.lane_refit import _project
    s, _, clamped = _project(np.asarray([point]), pts, ss, hh)[0]
    if clamped and s <= ss[0] + 1e-9:
        return float((np.asarray(point) - pts[0]) @ np.array([math.cos(hh[0]), math.sin(hh[0])]))
    return float(s)


def flare_end(st):
    """Station where a boundary's curb-return flare ends: ``st`` is the boundary as (s, t) in the road frame.
    The flare is the run of FLARE_STEP_M steps from the boundary's start whose slope dt/ds differs from the
    boundary's steady slope (the median over FLARE_SEARCH_M) by more than FLARE_SLOPE; None without one."""
    st = np.asarray(st, float)
    st = st[np.argsort(st[:, 0])]
    s0 = float(st[0, 0])
    grid = np.arange(s0, min(float(st[-1, 0]), s0 + FLARE_SEARCH_M) + 1e-9, FLARE_STEP_M)
    if len(grid) < 4:
        return None
    slope = np.diff(np.interp(grid, st[:, 0], st[:, 1])) / FLARE_STEP_M
    off = np.abs(slope - np.median(slope)) > FLARE_SLOPE
    if not off[0]:
        return None
    k = int(np.argmin(off)) if not off.all() else len(off)
    return float(grid[k])


def _flare(road, src, origin, ref, start_max):
    """Farthest flare end over the source boundaries of the road's first lanes (road stations), or None. Only
    boundaries starting by ``start_max`` (where the departure lanes begin, at the via joints) count: a lane taper
    further on is no curb return."""
    from mapforge.ops.lane_refit import _project
    pts, ss, hh = ref
    ends = []
    for lane in road.findall("lanes/laneSection")[0].iter("lane"):
        sid = _source_id(lane)
        if not sid or src.lane(sid) is None:
            continue
        for bnd in src.lane_boundary_geometries(sid):
            st = [(s, t) for s, t, clamped in _project(_local(bnd, origin), pts, ss, hh)
                  if not (clamped and s <= ss[0] + 1e-9)]          # (points before the road start)
            end = flare_end(st) if len(st) >= 2 and min(x[0] for x in st) <= start_max else None
            if end is not None:
                ends.append(end)
    return max(ends, default=None)


def plan(root, src, origin):
    """[{"road", "shift_m", "joints": {connector: station}, "flare_end_m"}] for the roads that start at a junction
    and whose connectors' via joints lie less than ROOM_M inside the connector (or inside the road). Such a road
    then starts ROOM_M past its farthest joint or past the end of its source lanes' curb-return flare, whichever
    lies further (stations on the road as it is; a flare ending beyond MAX_SHIFT_M - ROOM_M is not followed)."""
    from mapforge.validate.smoothness import sample_road_ref
    out = []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        pred = road.find("link/predecessor")
        if pred is None or pred.get("elementType") != "junction":
            continue
        conns = _ending_on(root, road.get("id"))
        if not conns:
            continue
        pts, ss, hh = sample_road_ref(road, 0.25)
        joints = {}
        for conn in conns:
            lane = conn.find("lanes/laneSection/right/lane[@id='-1']")
            via = _source_id(lane) if lane is not None else None
            rec = src.lane(via) if via else None
            if rec is None or len(rec.geometry) < 2:
                continue
            joints[conn.get("id")] = _station(pts, ss, hh, _local(rec.geometry, origin)[-1])
        if not joints:
            continue
        last = max(joints.values())
        if last + ROOM_M <= 0.5:
            continue
        flare = _flare(road, src, origin, (pts, ss, hh), last + 1.0)
        if flare is not None and flare + ROOM_M <= MAX_SHIFT_M:
            last = max(last, flare)
        shift = last + ROOM_M
        length = float(road.get("length"))
        if shift > MAX_SHIFT_M or length - shift < MIN_ROAD_M:
            continue
        out.append({"road": road.get("id"), "shift_m": round(shift, 3),
                    "joints": {k: round(v, 3) for k, v in joints.items()},
                    "flare_end_m": None if flare is None else round(flare, 3)})
    return out


def _road_pose(road, s):
    """Reference point and heading of ``road`` at station ``s``."""
    from mapforge.ops.map_stop_line_mouth import _pose_at
    geom = [g for g in road.findall("planView/geometry") if float(g.get("s")) <= s + 1e-9][-1]
    x, y, h, _ = _pose_at(geom, s - float(geom.get("s")))
    return np.array([x, y]), float(h)


def _end_pose(conn):
    from mapforge.ops.map_stop_line_mouth import _pose_at
    last = conn.findall("planView/geometry")[-1]
    return _pose_at(last, float(last.get("length")))


def _extend_end(conn, d):
    """Lengthen a connector at its end by ``d`` along its end heading (a line; lane records held)."""
    from lxml import etree
    x, y, h, _ = _end_pose(conn)
    length = float(conn.get("length"))
    pv = conn.find("planView")
    g = etree.SubElement(pv, "geometry", s=_f(length), x=_f(x), y=_f(y), hdg=_f(h), length=_f(d))
    etree.SubElement(g, "line")
    conn.set("length", _f(length + d))
    lanes = conn.find("lanes")
    sec = lanes.findall("laneSection")[-1]
    local = length - float(sec.get("s"))

    def hold(parent, tag, key, at):
        recs = sorted(parent.findall(tag), key=lambda el: float(el.get(key)))
        if not recs:
            return
        last_rec = recs[-1]
        u = at - float(last_rec.get(key))
        a, b, c, dd = (float(last_rec.get(k, 0.0)) for k in ("a", "b", "c", "d"))
        new = copy.deepcopy(last_rec)
        new.set(key, _f(at))
        new.set("a", _f(a + u * (b + u * (c + u * dd))))
        for k in ("b", "c", "d"):
            new.set(k, "0")
        parent.insert(list(parent).index(last_rec) + 1, new)
    hold(lanes, "laneOffset", "s", length)
    for lane in sec.iter("lane"):
        hold(lane, "width", "sOffset", local)
    # the source support that reached the old end reaches the new one (G8 clips the target to support_s)
    for prov in conn.iter("userData"):
        if prov.get("code") != "mapforge.provenance/v1" or not prov.get("value"):
            continue
        rec = json.loads(prov.get("value"))
        sup = rec.get("support_s")
        if isinstance(sup, list) and len(sup) == 2 and float(sup[1]) >= length - 1e-3:
            rec["support_s"] = [float(sup[0]), length + d]
            prov.set("value", json.dumps(rec, ensure_ascii=False, sort_keys=True, separators=(",", ":")))


def _shift_support(road, d):
    for prov in road.iter("userData"):
        if prov.get("code") != "mapforge.provenance/v1" or not prov.get("value"):
            continue
        rec = json.loads(prov.get("value"))
        sup = rec.get("support_s")
        if isinstance(sup, list) and len(sup) == 2:
            rec["support_s"] = [max(0.0, float(sup[0]) - d), max(0.0, float(sup[1]) - d)]
            prov.set("value", json.dumps(rec, ensure_ascii=False, sort_keys=True, separators=(",", ":")))


def _crossing(path, point, heading):
    """Index and point where ``path`` first reaches the mouth line through ``point`` (heading into the road),
    or None."""
    f = (np.asarray(path) - point) @ np.array([math.cos(heading), math.sin(heading)])
    ahead = np.flatnonzero(f >= 0.0)
    if not len(ahead):
        return None
    j = int(ahead[0])
    if j == 0:
        return 0, np.asarray(path[0], float)
    w = -f[j - 1] / (f[j] - f[j - 1])
    return j, np.asarray(path[j - 1]) + w * (np.asarray(path[j]) - np.asarray(path[j - 1]))


def _record(lane, d):
    from mapforge.validate.g8_model import geometry_sha256
    coords = lane["geometry"]["coordinates"]
    lane["geometry"]["geometry_sha256"] = geometry_sha256(coords)
    lane["travel"]["start"], lane["travel"]["end"] = coords[0], coords[-1]
    lane.setdefault("window_adjustments", []).append({"code": CODE, "mouth_shift_m": round(d, 3)})


def _extend_window(lane, path, point, heading, d):
    """Via window run on along ``path`` (via line, then the departure lane) up to the new mouth line."""
    coords = np.asarray(lane["geometry"]["coordinates"], float)
    tail = coords[-1]
    a, b = path[:-1], path[1:]
    u = np.clip(((tail - a) * (b - a)).sum(axis=1) / np.maximum(((b - a) ** 2).sum(axis=1), 1e-18), 0.0, 1.0)
    k = int(np.argmin(np.linalg.norm(a + u[:, None] * (b - a) - tail, axis=1)))   # the segment the window ends on
    hit = _crossing(path[k:], point, heading)
    if hit is None or hit[0] == 0:
        return False
    j, p = hit
    ext, last = [], tail
    for q in [*path[k + 1:k + j], p]:
        if np.linalg.norm(q - last) > 1e-6:            # (the via end and the lane start coincide)
            ext.append(q)
            last = q
    if not ext:
        return False
    lane["geometry"]["coordinates"] = np.vstack([coords, ext]).tolist()
    _record(lane, d)
    return True


def _crop_window(lane, point, heading, d):
    """Departure lane window started at the new mouth line."""
    coords = np.asarray(lane["geometry"]["coordinates"], float)
    hit = _crossing(coords, point, heading)
    if hit is None:
        return False
    j, p = hit
    if j == 0:
        return False
    lane["geometry"]["coordinates"] = np.vstack([p[None, :], coords[j:]]).tolist()
    _record(lane, d)
    return True


def apply(root, manifest, src, origin):
    """Move the planned mouths (lxml root and manifest in place); returns the report."""
    from mapforge.ops.map_stop_line_mouth import cut_start
    from mapforge.validate.g8_model import object_sha256
    lanes_by_id = {lane["source_lane_id"]: lane for lane in manifest["lanes"]}
    report = []
    for item in plan(root, src, origin):
        road = root.find(f"road[@id='{item['road']}']")
        d = item["shift_m"]
        point, heading = _road_pose(road, d)
        first = road.findall("lanes/laneSection")[0]
        dep_of = {int(ln.get("id")): _source_id(ln) for ln in first.iter("lane")}
        windows = {"extended": [], "cropped": []}
        for conn in _ending_on(root, road.get("id")):
            lane = conn.find("lanes/laneSection/right/lane[@id='-1']")
            via = _source_id(lane) if lane is not None else None
            succ = lane.find("link/successor") if lane is not None else None
            dep = dep_of.get(int(succ.get("id"))) if succ is not None else None
            _extend_end(conn, d)
            if via in lanes_by_id and dep and src.lane(via) is not None and src.lane(dep) is not None:
                path = np.vstack([_local(src.lane(via).geometry, origin), _local(src.lane(dep).geometry, origin)])
                if _extend_window(lanes_by_id[via], path, point, heading, d):
                    windows["extended"].append(via)
        for sid in sorted({x for x in dep_of.values() if x}):
            if sid in lanes_by_id and _crop_window(lanes_by_id[sid], point, heading, d):
                windows["cropped"].append(sid)
        cut_start(road, d)
        _shift_support(road, d)
        report.append({**item, "new_mouth": [round(float(point[0]), 3), round(float(point[1]), 3)],
                       "windows": windows})
    if report:
        manifest.pop("manifest_sha256", None)
        manifest["manifest_sha256"] = object_sha256(manifest)
    return report


def rebuild_paving(root, src, junction, project, decisions, moved):
    """Junction paving rebuilt (ElementTree root, in place) with the moved departure roads as mouths of their own
    (pose at the road start, heading into the junction; their source lanes give the tails); the paving axes stay
    those of the converter's mouths."""
    from mapforge.adapters.opendrive import writer as W
    from mapforge.ops.envelope_surface import append_source_paving
    from mapforge.ops.junction_surface import written_mouth_specs
    from mapforge.ops.map_to_xodr import _mouth_apron_axes
    from mapforge.validate.smoothness import sample_road_ref
    mouths = written_mouth_specs(root, decisions)
    axes = _mouth_apron_axes(mouths) or [None]
    for item in moved:
        road = root.find(f"road[@id='{item['road']}']")
        pts, _, hh = sample_road_ref(road, 0.25)
        links = sorted({src.lane(_source_id(ln)).link_pid for ln in road.iter("lane")
                        if _source_id(ln) and src.lane(_source_id(ln)) is not None})
        if not links:
            continue
        mouths.append({"road_id": item["road"], "enter_link": links[0], "leave_links": links[1:],
                       "pose": [float(pts[0][0]), float(pts[0][1]), float(hh[0]) + math.pi],
                       "decision": "leave-mouth-shift"})
    doc = W.XodrDoc("candidate-source-surface")
    for rd in root.findall("road"):
        if rd.get("name") != "junction_paving":
            doc.add_road(W.Road(int(rd.get("id"))))
    count = len(doc.roads)
    stats = append_source_paving(doc, src, junction, project, mouths, axes)
    for rd in list(root.findall("road")):
        if rd.get("name") == "junction_paving":
            root.remove(rd)
    j = root.find("junction")
    at = list(root).index(j) if j is not None else len(root)
    for rd in doc.roads[count:]:
        temporary = ET.Element("OpenDRIVE")
        doc._road_el(temporary, rd)
        root.insert(at, temporary.find("road"))
        at += 1
    stats["source_surface_status"] = ("FAIL" if any(stats[k] for k in
                                      ("mouth_tail_source_issues", "mouth_tail_bridge_issues", "median_tail_issues"))
                                      else "GENERATED")
    return stats


def apply_file(xodr, manifest, shp_dir, profile, locate, decisions):
    """Raw SHP conversion ``xodr`` (written in place) and its ``manifest`` (dict, in place); ``locate``: (lon, lat)
    of the junction. Returns the report (empty when nothing moved)."""
    from lxml import etree
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.ops.shp_to_xodr import _proj
    from mapforge.validate.shp_boundary_fidelity import _origin
    xodr = Path(xodr)
    src = ProfileSource(str(shp_dir), profile)
    tree = etree.parse(str(xodr), etree.XMLParser(strip_cdata=False, remove_blank_text=False))
    root = tree.getroot()
    moved = apply(root, manifest, src, _origin(root))
    if not moved:
        return {"schema": CODE, "moved": []}
    tree.write(str(xodr), xml_declaration=True, encoding="UTF-8")
    junc, _ = src.find_junction(*locate)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    et_root = ET.parse(xodr).getroot()
    surface = rebuild_paving(et_root, src, junc, lambda p: _proj(p, lat0, lon0), decisions, moved)
    if surface["source_surface_status"] != "GENERATED":
        raise ValueError(f"leave-mouth paving not generated: {surface}")
    ET.indent(et_root)
    ET.ElementTree(et_root).write(xodr, encoding="utf-8", xml_declaration=True)
    return {"schema": CODE, "moved": moved,
            "surface": {k: v for k, v in surface.items() if k not in ("mouth_tail_source_components",)}}
