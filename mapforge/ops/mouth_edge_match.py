"""Junction-mouth lane-edge consistency for one-lane connecting roads (Phase 2, step 0).

Connecting roads are written with a constant lane width (SHP) or widths that do
not reproduce the linked lanes at the mouths. Lane centres meet, but world lane
edges step by up to 1.8 m / 9 deg at contacts (2026-10-03 baseline). This
post-process keeps every reference line, link, lane identity and source record.
It only replaces the connecting road's laneOffset and lane -1 width with cubic
Hermite edge offsets that reproduce both linked lanes' edge position and
heading (G1, decision D2). Edge curvature is not matched; the scoreboard
reports the remaining jump. Unsupported shapes are reported, never guessed.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from lxml import etree
from pyclothoids import Clothoid

from mapforge.validate.junction_edges import endpoint_edges
from mapforge.validate.smoothness import _geoms

CODE = "mapforge.mouth_edge_match/v1"


def _hermite(t0, m0, t1, m1, length):
    """Cubic a+b*s+c*s^2+d*s^3 with value/slope (t0, m0) at 0 and (t1, m1) at length."""
    delta = (t1 - t0) / length
    return (t0, m0, (3 * delta - 2 * m0 - m1) / length, (m0 + m1 - 2 * delta) / length ** 2)


def _poly(c, s):
    return c[0] + s * (c[1] + s * (c[2] + s * c[3]))


# Width bulge/pinch budget beyond the end widths (below the draft T1 limit of 0.30 m).
WIDTH_OVERSHOOT_TOL_M = 0.25


def _monotone_slopes(w0, m0, w1, m1, length):
    """Fritsch-Carlson limiter for one cubic interval: no overshoot beyond the end values."""
    delta = (w1 - w0) / length
    if abs(delta) < 1e-12:
        return 0.0, 0.0
    a, b = m0 / delta, m1 / delta
    a, b = max(a, 0.0), max(b, 0.0)
    if a * a + b * b > 9.0:
        tau = 3.0 / math.sqrt(a * a + b * b)
        a, b = tau * a, tau * b
    return a * delta, b * delta


def edge_offsets(local, length, tol=WIDTH_OVERSHOOT_TOL_M):
    """Left/right edge offset cubics from end targets {(contact, side): (t, dt/ds, along)}.

    The lane centre always matches position and slope (smooth lane-centre path).
    The width matches slope too, unless that cubic would bulge or pinch more than
    ``tol`` beyond its end widths (strongly flared lanes at a mouth). Then the width
    slopes are Fritsch-Carlson limited and the resulting edge kink is reported.
    """
    def ends(side):
        return local[("start", side)][:2], local[("end", side)][:2]
    (l0, dl0), (l1, dl1) = ends("left")
    (r0, dr0), (r1, dr1) = ends("right")
    centre = _hermite((l0 + r0) / 2, (dl0 + dr0) / 2, (l1 + r1) / 2, (dl1 + dr1) / 2, length)
    w0, w1, m0, m1 = l0 - r0, l1 - r1, dl0 - dr0, dl1 - dr1
    width = _hermite(w0, m0, w1, m1, length)
    samples = [_poly(width, length * i / 200) for i in range(201)]
    bulge = max(max(samples) - max(w0, w1), min(w0, w1) - min(samples), 0.0)
    limited = bulge > tol
    if limited:
        # Blend from the monotone slopes towards the target slopes as far as the bulge budget allows.
        n0, n1 = _monotone_slopes(w0, m0, w1, m1, length)

        def bulge_at(lam):
            c = _hermite(w0, n0 + lam * (m0 - n0), w1, n1 + lam * (m1 - n1), length)
            v = [_poly(c, length * i / 200) for i in range(201)]
            return max(max(v) - max(w0, w1), min(w0, w1) - min(v), 0.0)

        lo, hi = 0.0, 1.0
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if bulge_at(mid) <= tol else (lo, mid)
        s0, s1 = n0 + lo * (m0 - n0), n1 + lo * (m1 - n1)
        width = _hermite(w0, s0, w1, s1, length)
        kink = max(abs(math.degrees(math.atan((s0 - m0) / 2))), abs(math.degrees(math.atan((s1 - m1) / 2))))
    else:
        kink = 0.0
    left = tuple(c + w / 2 for c, w in zip(centre, width))
    right = tuple(c - w / 2 for c, w in zip(centre, width))
    return left, right, {"width_slope_limited": limited, "unlimited_width_bulge_m": bulge,
                         "edge_heading_kink_deg": kink}


def _frame(road, contact):
    """Reference pose (x, y, heading, curvature) of ``road`` at its start or end."""
    geoms = _geoms(road)
    if not geoms or len(geoms) != len(road.findall("planView/geometry")):
        raise ValueError("unsupported reference geometry")
    _, x, y, h, length, k0, k1 = geoms[0 if contact == "start" else -1]
    if contact == "start":
        return x, y, h, k0
    c = Clothoid.StandardParams(x, y, h, k0, (k1 - k0) / length, length)
    return c.XEnd, c.YEnd, c.ThetaEnd, c.KappaEnd


def _local(frame, edge):
    """Lateral offset, its s-derivative, and along-track residual of a world edge."""
    x, y, h, k = frame
    dx, dy = edge["x"] - x, edge["y"] - y
    t = -dx * math.sin(h) + dy * math.cos(h)
    along = dx * math.cos(h) + dy * math.sin(h)
    rel = (edge["heading"] - h + math.pi) % (2 * math.pi) - math.pi
    if abs(rel) >= math.pi / 2:
        raise ValueError("edge heading opposes connector direction")
    return t, (1 - k * t) * math.tan(rel), along


def _targets(roads, cr, connection):
    lane = cr.find("lanes/laneSection/right/lane[@id='-1']")
    out = {}
    for role, contact in (("predecessor", "start"), ("successor", "end")):
        link = cr.find("link/" + role)
        if link is None or link.get("elementType") != "road":
            raise ValueError("missing endpoint road")
        other = roads[link.get("elementId")]
        oc = link.get("contactPoint")
        if role == "predecessor":
            pairs = [p for p in connection.findall("laneLink") if p.get("to") == "-1"]
            if len(pairs) != 1:
                raise ValueError("connector lane -1 needs exactly one incoming laneLink")
            other_id = int(pairs[0].get("from"))
            forward = oc == "end"
        else:
            nxt = lane.find("link/successor")
            if nxt is None:
                raise ValueError("connector lane -1 has no successor lane")
            other_id = int(nxt.get("id"))
            forward = oc == "start"
        out[contact] = endpoint_edges(other, other_id, oc, forward)
    return out


def _supported(cr):
    sections = cr.findall("lanes/laneSection")
    if len(sections) != 1:
        return "connector has more than one laneSection"
    right = sections[0].findall("right/lane")
    if [lane.get("id") for lane in right] != ["-1"] or sections[0].findall("left/lane"):
        return "connector is not a single right lane -1"
    if sections[0].find("right/lane[@id='-1']").findall("border"):
        return "connector lane uses border records"
    return None


def _fmt(v):
    return format(float(v), ".15g")


def _replace(parent, tag, new_elements):
    """Replace all ``tag`` children of ``parent`` in place (schema order kept)."""
    old = parent.findall(tag)
    if not old:
        raise ValueError(f"no {tag} records to replace")
    index = list(parent).index(old[0])
    tail = old[-1].tail
    for el in old:
        parent.remove(el)
    for i, el in enumerate(new_elements):
        el.tail = tail
        parent.insert(index + i, el)


def match_tree(root, only=None):
    """Rewrite connector offsets/widths in ``root`` in place; return a per-connector report.

    ``only``: optional set of connecting-road ids to process (others untouched).
    """
    roads = {r.get("id"): r for r in root.findall("road")}
    connections = {c.get("connectingRoad"): c for c in root.findall("junction/connection")}
    rows, skipped = [], []
    for cr in root.findall("road"):
        if cr.get("junction") in (None, "-1") or cr.get("name") == "junction_paving":
            continue
        rid = cr.get("id")
        if only is not None and rid not in only:
            continue
        reason = _supported(cr)
        connection = connections.get(rid)
        if reason is None and (connection is None or connection.get("contactPoint") != "start"):
            reason = "no start-contact junction connection"
        if reason:
            skipped.append({"connecting_road": rid, "reason": reason})
            continue
        try:
            targets = _targets(roads, cr, connection)
            length = float(cr.get("length"))
            frames = {"start": _frame(cr, "start"), "end": _frame(cr, "end")}
            local = {(c, side): _local(frames[c], targets[c][side])
                     for c in ("start", "end") for side in ("left", "right")}
        except (KeyError, ValueError, ZeroDivisionError) as exc:
            skipped.append({"connecting_road": rid, "reason": f"{type(exc).__name__}: {exc}"})
            continue
        left, right, shape = edge_offsets(local, length)
        width = tuple(a - b for a, b in zip(left, right))
        samples = [length * i / 200 for i in range(201)]
        min_width = min(_poly(width, s) for s in samples)
        if min_width <= 0.5:
            skipped.append({"connecting_road": rid, "reason": f"Hermite width would drop to {min_width:.3f} m"})
            continue
        lanes = cr.find("lanes")
        lane = lanes.find("laneSection/right/lane[@id='-1']")
        before = [{k: float(w.get(k)) for k in ("sOffset", "a", "b", "c", "d")} for w in lane.findall("width")]
        _replace(lanes, "laneOffset", [etree.Element(
            "laneOffset", s="0", a=_fmt(left[0]), b=_fmt(left[1]), c=_fmt(left[2]), d=_fmt(left[3]))])
        _replace(lane, "width", [etree.Element(
            "width", sOffset="0", a=_fmt(width[0]), b=_fmt(width[1]), c=_fmt(width[2]), d=_fmt(width[3]))])
        record = {"before_width": before, "start_width_m": width[0], "end_width_m": _poly(width, length),
                  "min_width_m": min_width, **shape,
                  "along_track_residual_m": max(abs(v[2]) for v in local.values()),
                  "edge_curvature": "not matched (G1 plus bounded jump)"}
        lane.append(etree.Element("userData", code=CODE, value=json.dumps(record, ensure_ascii=False)))
        rows.append({"connecting_road": rid, **{k: v for k, v in record.items() if k != "before_width"},
                     "before_constant_width_m": before[0]["a"] if len(before) == 1 else None})
    return {"schema": CODE, "rewritten": len(rows), "skipped": skipped, "rows": rows}


def apply(xodr_in, xodr_out):
    tree = etree.parse(str(xodr_in), etree.XMLParser(strip_cdata=False, remove_blank_text=False))
    report = match_tree(tree.getroot())
    Path(xodr_out).parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(xodr_out), xml_declaration=True, encoding="UTF-8")
    return report
