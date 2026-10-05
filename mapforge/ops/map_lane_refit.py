"""MAP road-side refit: lane boundaries fitted to the MAP lane point lists (2026-10-04).

The MAP converter places the lane boundaries at a few control stations (5-20 m apart) and smooths them
(C1, lambda 0.006); its lane centres leave the MAP points between the stations by up to 0.6 m, which is
where the MAP lane-centre P95 of 0.17-0.44 m came from. MAP gives lane centre point lists (thinned by chord
tolerance, T/CSAE 159 Appendix D) and nominal widths, not boundaries. Here, per road:

1. free far ends are extended to the MAP lane end points beyond them (map_far_end);
2. the per-section pieces of a turn-bay taper are linked into one lane (link_tapers);
3. the boundaries are sampled every 0.5 m so that each lane centre lies on its point list (densified
   piecewise linearly) and the current widths change as little as possible (make_observe);
4. the SHP road-side fitter (lane_refit.refit_road: C2 boundaries, G2 corners, births/deaths, mouth
   conditions) fits them at MAP_LEVEL, a lane that is narrow somewhere as an offset from its inner boundary
   (narrow_chain), so zero-width stretches and opening tapers never cross;
5. the departure side mirrors the approach side (INFERRED, no source geometry): its mirrored boundaries are
   the refitted approach boundaries reflected about the centre line, and its mirrored lanes take the
   approach lanes' widths again, so the mirror stays exact.
Junction mouths keep their place (the stop lines are the last MAP points and the converter meets them).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from lxml import etree

from mapforge.ops import lane_refit as LR

CODE = "mapforge.map_lane_refit/v1"
STEP_M = 0.5
# Narrow lane (a turn-bay taper, a zero-width lane outside its point list): without a point list there it
# keeps its current width (NARROW_WIDTH_WEIGHT), its outer boundary follows the fitted inner boundary, and its
# boundary chain is fitted as a width (narrow_chain).
NARROW_WIDTH_M = 2.0
# Least-squares weights of the boundary samples (squared residuals): a lane centre on its MAP point list, the
# current width of a lane (narrow lanes: a taper keeps its shape), the centre boundary where no lane is seen.
CENTRE_WEIGHT = 100.0
NARROW_WIDTH_WEIGHT = 100.0
ANCHOR_WEIGHT = 1e-6
# A lane continuing before its first MAP point is pulled to that point's offset over this distance (a turn
# bay opens over it: 25 m is about 1:7 for 3.5 m; 12 m left the outer edge at up to 0.15 /m, map-node18).
SOURCE_RAMP_M = 25.0
# Corner/curvature level for MAP boundaries: level A (corner curvature 0.04 /m, RDP 5 cm) with 5 m runs, so a
# MAP point 5.9 m after the first one still gets its corner (map-node3 road 10 lane -1: 0.42 m off with 6 m).
# Level B (0.02 /m, 12 m runs, 10 cm) dropped such points: 0.6 m off at a 1.15 m lane shift over 9 m
# (map-NODE5 road 12 lane -1), and its 10 cm tolerance let narrow lanes cross by up to 0.27 m (map-node18).
MAP_LEVEL = {"kappa_cap": 0.04, "seg_min": 5.0, "rdp_tol": 0.05}


def _densify(points, step=STEP_M):
    pts = np.asarray(points, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 1e-9:
        return pts[:1]
    q = np.append(np.arange(0.0, s[-1], step), s[-1])
    return np.column_stack([np.interp(q, s, pts[:, 0]), np.interp(q, s, pts[:, 1])])


def load_centres(manifest_path):
    """Source lane centre point lists (XODR local frame) keyed by source lane id."""
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    out = {}
    for lane in manifest.get("lanes", []):
        geom = (lane.get("geometry") or {}).get("coordinates")
        if geom and len(geom) >= 2 and (lane.get("comparison") or {}).get("eligible", True):
            out[lane["source_lane_id"]] = np.asarray(geom, float)[:, :2]
    return out


def _smoothstep(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def make_observe(centres):
    """observe(road, side, i, k, fitted) for lane_refit.refit_road: boundary samples from the MAP lane centres.

    At every grid station of a section the boundaries of both sides are solved together (least squares): each
    lane centre the MAP point list covers lies on it (CENTRE_WEIGHT), every lane keeps its current width as far
    as the centres allow, and the centre boundary stays put where no lane is seen. Where the MAP centre
    spacing disagrees with the current widths (P95 0.2-0.4 m, up to 0.6 m in the seven junctions) the widths
    give way, not the centres. A lane without a point list at a station keeps its current width much more
    strongly when it is narrow (a taper keeps its shape). Without any source in a section (departure side, a
    lane outside its point list) this is the current geometry: a boundary is never bridged or extrapolated
    over such a section (map-node4 road 12: the centre line was extrapolated 2.7 m over a 57 m turn-bay taper).

    Where a lane continues before its first MAP point (a turn bay whose taper starts earlier), its centre is
    pulled to the first point's offset over SOURCE_RAMP_M before it, with a weight that blends the two
    solutions smoothly: switching the centre on at the first point stepped the samples by up to 1.1 m
    (map-node4 road 12 lane -1, whose first point already lies at full width). Past the last point (the stop
    line) nothing is held: the converter's mouth meets it, and holding it moved the mouths by 1-2 cm, which
    made every connector re-align (five times slower, map-node3/node4).
    """
    projected, solved = {}, {}

    def lane_profile(road, sid):
        """(s, t) of the densified point list sorted by s, and whether its first point is the low-s end."""
        key = (road.el.get("id"), sid)
        if key not in projected:
            st = [(s, t) for s, t, clamped in LR._project(_densify(centres[sid]), *road.ref) if not clamped]
            projected[key] = (np.asarray(sorted(st)), st[0][0] <= st[-1][0]) if len(st) >= 2 else None
        return projected[key]

    def centre_at(road, lane, x):
        """(target offset, blend 0..1) of the lane centre at x, or None.

        The blend ramp lies only before the first point (travel start): the last point is the stop line,
        where the converter's mouth already meets it and the connectors start."""
        sid = lane.find("userData[@code='mapforge.source_lane']")
        if sid is None or sid.get("value") not in centres:
            return None
        found = lane_profile(road, sid.get("value"))
        if found is None:
            return None
        prof, first_low = found
        lo, hi = prof[0, 0], prof[-1, 0]
        if lo <= x <= hi:
            return float(np.interp(x, prof[:, 0], prof[:, 1])), 1.0
        if (x < lo) != first_low:
            return None
        gap, held = (lo - x, prof[0, 1]) if x < lo else (x - hi, prof[-1, 1])
        if gap >= SOURCE_RAMP_M:
            return None
        return float(held), _smoothstep(1.0 - gap / SOURCE_RAMP_M)

    def solve(road, i):
        key = (road.el.get("id"), i)
        if key in solved:
            return solved[key]
        s0, s1 = road.s[i], road.s[i + 1]
        grid = np.linspace(s0, s1, max(3, int((s1 - s0) / LR.GRID) + 1))
        sides = {side: road.lanes(i, side) for side in ("right", "left")}
        n = len(sides["right"])
        size = 1 + n + len(sides["left"])

        def index(side, k):   # unknowns: centre boundary, right boundaries 1..n, left boundaries 1..m
            return 0 if k == 0 else (k if side == "right" else n + k)

        out = {side: np.zeros((len(grid), len(lanes) + 1)) for side, lanes in sides.items()}
        for g, x in enumerate(grid):
            rows, rhs = [], []

            def add(coef, value, weight):
                r = np.zeros(size)
                for j, c in coef:
                    r[j] += c
                rows.append(r * math.sqrt(weight))
                rhs.append(value * math.sqrt(weight))

            edges = {side: road.lane_edges_at(road.el, x, side) for side in sides}
            add([(0, 1.0)], edges["right"][0], ANCHOR_WEIGHT)
            for side, lanes in sides.items():
                outward = 1.0 if side == "left" else -1.0
                e = edges[side]
                for k, lane in enumerate(lanes, 1):
                    inner, outer = index(side, k - 1), index(side, k)
                    w = abs(e[k - 1] - e[k])
                    seen = centre_at(road, lane, x)
                    held = w < NARROW_WIDTH_M and seen is None
                    add([(outer, outward), (inner, -outward)], w, NARROW_WIDTH_WEIGHT if held else 1.0)
                    if seen is not None:
                        c, blend = seen
                        # weight b/(1-b) against a unit width prior blends the two solutions about linearly
                        weight = CENTRE_WEIGHT if blend >= 1.0 else min(CENTRE_WEIGHT, blend / (1.0 - blend))
                        if weight > 0.0:
                            add([(inner, 0.5), (outer, 0.5)], c, weight)
            u = np.linalg.lstsq(np.asarray(rows), np.asarray(rhs), rcond=None)[0]
            for side, lanes in sides.items():
                out[side][g] = [u[index(side, k)] for k in range(len(lanes) + 1)]
        solved[key] = grid, out
        return solved[key]

    def observe(road, side, i, k, fitted=None):
        if k == 0 and side == "left":
            return None   # the centre boundary is solved once, for both sides
        grid, out = solve(road, i)
        t = out[side][:, k].copy()
        inner_fit = (fitted or {}).get((side, i, k - 1)) if k >= 1 else None
        if inner_fit:
            # Where lane k (just inside this boundary) is narrow, this boundary follows the already fitted inner
            # boundary minus the lane's solved width: the two boundaries of a zero-width or opening lane must
            # not be fitted independently (they crossed by 0.1-0.4 m, map-node4 roads 10/11/13).
            inward = 1.0 if side == "right" else -1.0
            width = np.abs(out[side][:, k - 1] - out[side][:, k])
            for g in np.flatnonzero(width < NARROW_WIDTH_M):
                t[g] = LR._at(inner_fit, grid[g]) - inward * width[g]
        return np.column_stack([grid, t])

    return observe


TAPER_CODE = "lane-transition-taper"
TAPER_JOIN_STEP_M = 0.05


def _width_at(lane, ds):
    rec = [w for w in lane.findall("width") if float(w.get("sOffset")) <= ds + 1e-9]
    if not rec:
        return 0.0
    w = rec[-1]
    u = ds - float(w.get("sOffset"))
    a, b, c, d = (float(w.get(k)) for k in ("a", "b", "c", "d"))
    return a + u * (b + u * (c + u * d))


def _taper(lane):
    prov = lane.find("userData[@code='mapforge.provenance/v1']")
    return prov is not None and prov.get("value") and json.loads(prov.get("value")).get("exclusion_code") == TAPER_CODE


def _source_id(lane):
    sid = lane.find("userData[@code='mapforge.source_lane']")
    return sid.get("value") if sid is not None else None


def _add_link(lane, tag, lid):
    link = lane.find("link")
    if link is None:
        link = etree.Element("link")
        link.tail = lane.text
        lane.insert(0, link)
    el = etree.Element(tag, id=lid)
    if tag == "predecessor":
        link.insert(0, el)
    else:
        link.append(el)


def link_tapers(road_el):
    """Link the per-section pieces of a turn-bay taper into one lane; returns [(section, side, lane id)].

    The MAP converter writes a taper (lane-transition-taper, zero width until the bay opens) as one lane per
    section without lane links, so the refit started the bay as a birth in the section where it opens and
    opened it within the few metres before the first MAP point (outer edge curvature up to 0.23 /m, map-node17
    road 11: 3.3 m in 11.6 m). Linked, the bay can open gently before its first point (SOURCE_RAMP_M).
    Only approach lanes (with a MAP point list) are linked: a departure lane mirrored from a bay narrows to zero
    away from the junction and must end there, or traffic following it is moved sideways at zero width (esmini:
    nine 2 m jumps, map-NODE5); the departure side still mirrors the linked approach geometry (mirror_fitted).
    """
    secs = road_el.findall("lanes/laneSection")
    s = [float(x.get("s")) for x in secs] + [float(road_el.get("length"))]
    linked = []
    for i in range(len(secs) - 1):
        for side in ("left", "right"):
            nxt = {ln.get("id"): ln for ln in secs[i + 1].findall(f"{side}/lane")}
            for la in secs[i].findall(f"{side}/lane"):
                lb = nxt.get(la.get("id"))
                if (lb is None or la.find("link/successor") is not None or lb.find("link/predecessor") is not None
                        or not (_taper(la) and _taper(lb)) or _source_id(la) is None or _source_id(la) != _source_id(lb)
                        or abs(_width_at(la, s[i + 1] - s[i]) - _width_at(lb, 0.0)) > TAPER_JOIN_STEP_M):
                    continue
                _add_link(la, "successor", lb.get("id"))
                _add_link(lb, "predecessor", la.get("id"))
                linked.append((i, side, la.get("id")))
    return linked


def _widths(lane):
    return [tuple(float(w.get(k)) for k in ("sOffset", "a", "b", "c", "d")) for w in lane.findall("width")]


def _same_widths(a, b, tol=1e-6):
    wa, wb = _widths(a), _widths(b)
    return len(wa) == len(wb) and all(abs(x - y) <= tol for ra, rb in zip(wa, wb) for x, y in zip(ra, rb))


def mirrored_lanes(road_el):
    """(section index, left lane id) of left lanes whose width records equal the right lane of the same |id|.

    The MAP converter mirrors the approach side onto the departure side (lanes it labels mirror or
    lane-transition-ribbon); these are recorded before the refit so they can follow it exactly.
    """
    out = set()
    for i, sec in enumerate(road_el.findall("lanes/laneSection")):
        right = {abs(int(ln.get("id"))): ln for ln in sec.findall("right/lane")}
        for lane in sec.findall("left/lane"):
            twin = right.get(abs(int(lane.get("id"))))
            if (twin is not None and lane.find("userData[@code='mapforge.source_lane']") is None
                    and _same_widths(lane, twin)):
                out.add((i, lane.get("id")))
    return out


def mirror_fitted(road, fitted, mirrored):
    """Mirrored departure boundaries = the refitted approach boundaries reflected about the centre line.

    Fitting them separately from the current geometry crossed zero-width lanes (map-node4 road 10, -0.37 m);
    a left boundary k is mirrored only where left lanes 1..k all mirror their approach twins.
    """
    count = 0
    for i in range(len(road.sections)):
        s0, s1 = road.s[i], road.s[i + 1]
        centre = fitted.get(("right", i, 0))
        if not centre:
            continue
        twice = LR._combine(centre, centre, s0, s1, 1.0)
        for lane in road.lanes(i, "left"):
            k = abs(int(lane.get("id")))
            if (i, lane.get("id")) not in mirrored or not fitted.get(("right", i, k)):
                break
            fitted[("left", i, k)] = LR._combine(twice, fitted[("right", i, k)], s0, s1, -1.0)
            count += 1
    return count


def _remirror(road_el, mirrored):
    """The recorded mirrored departure lanes take the refitted approach lane's width records again."""
    changed = 0
    for i, sec in enumerate(road_el.findall("lanes/laneSection")):
        right = {abs(int(ln.get("id"))): ln for ln in sec.findall("right/lane")}
        for lane in sec.findall("left/lane"):
            twin = right.get(abs(int(lane.get("id"))))
            if (i, lane.get("id")) not in mirrored or twin is None:
                continue
            old = lane.findall("width")
            at = list(lane).index(old[0]) if old else len(lane.findall("link"))
            tail = old[-1].tail if old else "\n"
            for el in old:
                lane.remove(el)
            for j, w in enumerate(twin.findall("width")):
                el = etree.Element("width", **{key: w.get(key) for key in ("sOffset", "a", "b", "c", "d")})
                el.tail = tail
                lane.insert(at + j, el)
            changed += 1
    return changed


def narrow_chain(road, side, chain):
    """A boundary chain whose lane is narrower than NARROW_WIDTH_M somewhere (a taper, a zero-width stretch):
    fitted as an offset from its inner boundary (lane_refit.refit_road ``relative``)."""
    for i, k in chain["members"].items():
        for x in np.linspace(road.s[i], road.s[i + 1], 9):
            edges = road.lane_edges_at(road.el, x, side)
            if k < len(edges) and abs(edges[k - 1] - edges[k]) < NARROW_WIDTH_M:
                return True
    return False


def refit_tree(root, manifest_path, params=None, far_ends=True, stop_line_mouths=True):
    """Refit every ordinary MAP road with source lanes; mirrored sides follow the refitted approach side.

    ``stop_line_mouths``: arms are first extended to their farthest stop line, attached connectors cut back
    and, after the refit, the junction paving rebuilt on the final mouths (map_stop_line_mouth).
    ``far_ends``: free road ends are extended to the MAP lane end points beyond them (map_far_end).
    """
    from mapforge.ops import map_far_end, map_stop_line_mouth
    from mapforge.validate.smoothness import sample_road_ref
    params = dict(params or {})
    params["curb_shoulder"] = False   # no physical curb in MAP: a flare at the mouth stays a driving lane
    params.update(MAP_LEVEL)
    centres = load_centres(manifest_path)
    mouths = map_stop_line_mouth.apply(root, centres) if stop_line_mouths else None
    observe = make_observe(centres)
    rows, skipped = [], []
    for road_el in root.findall("road"):
        if road_el.get("junction") not in (None, "-1") or road_el.get("name") == "junction_paving":
            continue
        rid = road_el.get("id")
        if road_el.find("lanes/laneSection/*/lane/border") is not None:
            skipped.append({"road": rid, "reason": "border records"})
            continue
        if road_el.find(f"userData[@code='{CODE}']") is not None:
            skipped.append({"road": rid, "reason": "already refitted"})
            continue
        sources = {u.get("value") for u in road_el.iter("userData") if u.get("code") == "mapforge.source_lane"}
        if not sources & set(centres):
            skipped.append({"road": rid, "reason": "no MAP source lanes"})
            continue
        far = None
        try:
            if far_ends:
                far = map_far_end.extend(road_el, centres, sample_road_ref(road_el, 0.25))
            tapers = link_tapers(road_el)
            mirrored = mirrored_lanes(road_el)
            road, fitted, report = LR.refit_road(road_el, None, None, params, observe=observe, relative=narrow_chain)
            report["mirrored_boundaries"] = mirror_fitted(road, fitted, mirrored)
            report["params"] = params
            report["taper_links"] = len(tapers)
            if far:
                report["far_end"] = far
            if LR.write_road(road, fitted, report):
                report["mirrored_lanes"] = _remirror(road_el, mirrored)
                road_el.append(etree.Element("userData", code=CODE, value=json.dumps(report, ensure_ascii=False)))
                rows.append(report)
            else:
                skipped.append({"road": rid, "reason": report.get("skipped"), "far_end": far})
        except (ValueError, KeyError, StopIteration, IndexError, np.linalg.LinAlgError) as exc:
            skipped.append({"road": rid, "reason": f"{type(exc).__name__}: {exc}", "far_end": far})
    out = {"schema": CODE, "rewritten": len(rows), "skipped": skipped, "rows": rows}
    if mouths and mouths["roads"]:
        mouths["paving"] = map_stop_line_mouth.rebuild_paving(root, mouths["junctions"])
        out["stop_line_mouths"] = mouths
    return out
