"""Analytic, source-faithful auxiliary surfaces with holes and separated cuts.

Only original polygon vertex stations are sweep events. One-to-one sections
with identical one-sided limits continue the same cell; splits, merges and
vertical boundary steps start new cells. Each cell is one line road, one
non-routable lane, and piecewise linear widths/offsets. There is no sampling
grid, endpoint padding, convex hull, or min/max envelope across separate cuts.

``written_geometry`` reconstructs the actual XML at every width/offset event;
it does not miss narrow events by sampling the reference line at a fixed step.
Short cells and real holes remain visible to the existing whole-map gates.
"""
from __future__ import annotations

from decimal import Context, Decimal
import hashlib
import json
import math
from typing import Iterable
import xml.etree.ElementTree as ET

import numpy as np
from shapely import wkb
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
from shapely.ops import unary_union

from mapforge.adapters.opendrive import writer as W

SCHEMA = "mapforge/source-surface-cells/v1"


class SurfaceCellError(ValueError):
    def __init__(self, code: str, **details):
        self.code, self.details = code, details
        super().__init__(code + (": " + json.dumps(details, ensure_ascii=False) if details else ""))


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _canonical(road):
    return ET.canonicalize(ET.tostring(road, encoding="unicode"), strip_text=True).encode("utf8")


def _finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise SurfaceCellError("nonfinite-number")
    return value


def _quantize(value):
    return _finite(W._f(_finite(value)))


def _polygons(geometry):
    if not isinstance(geometry, (Polygon, MultiPolygon)):
        raise SurfaceCellError("polygon-or-multipolygon-required")
    if not geometry.is_valid:
        raise SurfaceCellError("invalid-source-polygon")
    if geometry.has_z:
        raise SurfaceCellError("explicit-2d-source-required")
    if geometry.is_empty:
        return []
    return [geometry] if isinstance(geometry, Polygon) else list(geometry.geoms)


def _at(edge, s):
    if s == edge["s0"]:
        return edge["t0"]
    if s == edge["s1"]:
        return edge["t1"]
    return edge["t0"] + (s-edge["s0"])/(edge["s1"]-edge["s0"])*(edge["t1"]-edge["t0"])


def _same_limits(previous, following):
    return previous["low1"] == following["low0"] and previous["high1"] == following["high0"]


def _projection_roundoff(xy, origin, axis):
    """Outward bound for binary64 input/subtraction/products/addition.

    Each supplied binary64 coordinate/direction is enclosed by half an ulp;
    each arithmetic operation contributes half an ulp. Twice the accumulated
    bound keeps its own arithmetic outward. This is a numerical representation
    bound, not a geometric merge tolerance or a minimum supported feature size.
    """
    products, errors = [], []
    for value, reference, direction in zip(xy, origin, axis):
        delta = float(value-reference)
        product = float(delta*direction)
        delta_error = .5*(math.ulp(float(value))+math.ulp(float(reference))+math.ulp(delta))
        errors.append(abs(float(direction))*delta_error + abs(delta)*.5*math.ulp(float(direction))
                      + .5*math.ulp(product))
        products.append(product)
    return 2*(sum(errors)+.5*math.ulp(sum(products)))


def _normalize_stations(vertices):
    """Group only events whose independently bounded intervals intersect.

    Intersection, rather than transitive proximity, bounds the complete group.
    Source vertices/identities stay in evidence, including every original s.
    """
    groups, current = [], []
    lower, upper = -math.inf, math.inf
    for vertex in sorted(vertices,key=lambda v:v["s_m"]):
        value, error = vertex["s_m"], vertex["projection_roundoff_m"]
        lo,hi = value-error,value+error
        if current and max(lower,lo)>min(upper,hi):
            groups.append((current,lower,upper)); current=[]; lower=-math.inf; upper=math.inf
        current.append(vertex); lower=max(lower,lo); upper=min(upper,hi)
    if current:
        groups.append((current,lower,upper))
    adjustments=[]
    for group,lo,hi in groups:
        candidates=sorted({v["s_m"] for v in group if lo<=v["s_m"]<=hi})
        station=candidates[len(candidates)//2] if candidates else lo+(hi-lo)/2
        for vertex in group:
            vertex["normalized_s_m"]=station
        raw=sorted({v["s_m"] for v in group})
        if len(raw)>1:
            adjustments.append({"original_stations_m":raw,"normalized_station_m":station,
                "common_roundoff_interval_m":[lo,hi],
                "max_station_change_m":max(abs(s-station) for s in raw),
                "vertex_count":len(group)})
    return adjustments


def _add_piece(cell, piece):
    pieces = cell["pieces"]
    # A vertex on an unrelated boundary is not a new width event for this cell.
    if (pieces and pieces[-1]["edges"] == piece["edges"]
            and pieces[-1]["end_s_m"] == piece["start_s_m"]
            and _same_limits(pieces[-1], piece)):
        pieces[-1].update(end_s_m=piece["end_s_m"], low1=piece["low1"], high1=piece["high1"])
    else:
        pieces.append(dict(piece))


def _sweep_polygon(polygon, component_index, origin, axis, normal):
    edges, vertices, events, ring_vertices = [], [], set(), []
    for ring_index, ring in enumerate([polygon.exterior, *polygon.interiors]):
        xy = np.asarray(ring.coords, dtype=float)
        if xy.shape[1] != 2 or not np.isfinite(xy).all():
            raise SurfaceCellError("explicit-finite-2d-source-required")
        local = np.column_stack(((xy-origin)@axis, (xy-origin)@normal))
        for vertex_index, (world, point) in enumerate(zip(xy[:-1], local[:-1])):
            vertices.append({"component_index": component_index, "ring_index": ring_index,
                "vertex_index": vertex_index, "xy": world.tolist(),
                "s_m": float(point[0]), "t_m": float(point[1]),
                "projection_roundoff_m":_projection_roundoff(world,origin,axis)})
        ring_vertices.append(vertices[-(len(xy)-1):])
    normalizations=_normalize_stations(vertices)
    normalized_rings=[]
    for ring_index,ring in enumerate(ring_vertices):
        local=[np.array([v["normalized_s_m"],v["t_m"]]) for v in ring]
        events.update(float(p[0]) for p in local)
        local.append(local[0])
        normalized_rings.append(local)
        for edge_index, (p, q) in enumerate(zip(local[:-1], local[1:])):
            edges.append({"s0": float(p[0]), "t0": float(p[1]),
                "s1": float(q[0]), "t1": float(q[1]),
                "ref": {"component_index": component_index, "ring_index": ring_index,
                         "edge_index": edge_index}})
    normalized_polygon=Polygon(normalized_rings[0],normalized_rings[1:])
    if not normalized_polygon.is_valid or normalized_polygon.area<=0:
        raise SurfaceCellError("source-topology-not-resolvable-within-roundoff")
    stations = sorted(events)
    cells, topology, previous = [], [], []
    for start, end in zip(stations[:-1], stations[1:]):
        active = []
        for edge in edges:
            # No midpoint sampling: this remains valid when adjacent source
            # events have no representable floating-point midpoint.
            if min(edge["s0"], edge["s1"]) <= start and max(edge["s0"], edge["s1"]) >= end:
                if edge["s0"] == edge["s1"]:
                    continue
                t0, t1 = _at(edge, start), _at(edge, end)
                active.append((0.5*t0+0.5*t1, t0, t1, edge["ref"]))
        active.sort(key=lambda item: item[0])
        if len(active) % 2:
            raise SurfaceCellError("odd-open-section-intersection-count", station=start, count=len(active))
        following = []
        for low, high in zip(active[::2], active[1::2]):
            if not low[0] < high[0] or low[1] > high[1] or low[2] > high[2]:
                raise SurfaceCellError("unresolvable-or-crossing-source-interval", start=start, end=end)
            following.append({"start_s_m": start, "end_s_m": end,
                "low0": low[1], "low1": low[2], "high0": high[1], "high1": high[2],
                "edges": [low[3], high[3]]})
        links = {(i,j) for i,p in enumerate(previous) for j,n in enumerate(following)
                 if min(p["high1"], n["high0"]) > max(p["low1"], n["low0"])}
        pred = {j: [i for i,_j in links if _j == j] for j in range(len(following))}
        succ = {i: [j for _i,j in links if _i == i] for i in range(len(previous))}
        continued = 0
        vertical_steps = 0
        for j, piece in enumerate(following):
            parents = pred[j]
            keep = len(parents) == 1 and len(succ[parents[0]]) == 1
            if keep and not _same_limits(previous[parents[0]], piece):
                keep = False
                vertical_steps += 1
            if keep:
                index = previous[parents[0]]["cell_index"]
                continued += 1
            else:
                index = len(cells)
                cells.append({"component_index": component_index, "pieces": []})
            _add_piece(cells[index], piece)
            piece["cell_index"] = index
        topology.append({"station_m": start, "before_intervals": len(previous),
            "after_intervals": len(following), "continued_cells": continued,
            "split_intervals": sum(len(v)>1 for v in succ.values()),
            "merge_intervals": sum(len(v)>1 for v in pred.values()),
            "vertical_boundary_steps": vertical_steps,
            "born_intervals": sum(not v for v in pred.values()),
            "ended_intervals": sum(not v for v in succ.values())})
        previous = following
    if previous:
        topology.append({"station_m": stations[-1], "before_intervals":len(previous),
            "after_intervals":0, "continued_cells":0, "split_intervals":0,
            "merge_intervals":0, "vertical_boundary_steps":0,
            "born_intervals":0, "ended_intervals":len(previous)})
    return cells, vertices, topology, normalizations


def _quad(origin, axis, normal, start, end, low0, high0, low1, high1):
    return Polygon([origin+start*axis+low0*normal,
                    origin+start*axis+high0*normal,
                    origin+end*axis+high1*normal,
                    origin+end*axis+low1*normal])


def _width_slope(a, desired_end, span):
    slope = _quantize((_quantize(desired_end)-a)/span)
    adjustment = None
    if a+slope*span < 0:
        # %.10g rounding of a descending width can otherwise write a negative
        # width at a zero-width source tip. Round that one decimal quantum
        # toward zero, and report it as serialization loss, never as source
        # editing or endpoint padding.
        before = slope
        slope = float(Decimal(str(slope)).next_plus(Context(prec=10)))
        slope = _quantize(slope)
        if a+slope*span < 0:
            raise SurfaceCellError("writer-cannot-preserve-nonnegative-width")
        adjustment = {"kind":"decimal-width-rounding-toward-zero",
                      "before_slope":before, "after_slope":slope,
                      "endpoint_change_m":float((slope-before)*span)}
    return slope, adjustment


def _road_from_cell(cell, origin, axis, normal, road_id, junction_id, lane_type, provenance):
    pieces = cell["pieces"]
    start, end = pieces[0]["start_s_m"], pieces[-1]["end_s_m"]
    length = _quantize(end-start)
    if length <= 0:
        raise SurfaceCellError("nonpositive-cell-length")
    natural = [p["start_s_m"]-start for p in pieces] + [end-start]
    written_stations = [_quantize(s) for s in natural]
    if any(b <= a for a,b in zip(written_stations[:-1], written_stations[1:])):
        raise SurfaceCellError("source-events-collapse-at-writer-precision", source_stations=natural,
                               written_stations=written_stations)
    xy = origin+start*axis
    road = W.Road(road_id, name="junction_paving", junction=junction_id)
    road.add_geometry("line", float(xy[0]), float(xy[1]), math.atan2(axis[1], axis[0]), length)
    lane = W.Lane(-1, lane_type, provenance=provenance)
    adjustments = []
    for index,piece in enumerate(pieces):
        s0,s1 = written_stations[index:index+2]
        span = s1-s0
        high0 = _quantize(piece["high0"])
        high_slope = _quantize((_quantize(piece["high1"])-high0)/span)
        width0 = _quantize(piece["high0"]-piece["low0"])
        slope, correction = _width_slope(width0, piece["high1"]-piece["low1"], span)
        if correction:
            adjustments.append({"record_index":index, **correction})
        road.add_offset(s0, high0, high_slope)
        lane.add_width(width0, slope, s_offset=s0)
    section = W.LaneSection(0.0)
    section.right.append(lane)
    road.sections.append(section)
    holder = ET.Element("OpenDRIVE")
    W.XodrDoc("numeric-roundtrip")._road_el(holder, road)
    xml_road = holder.find("road")
    actual, metadata = _read_road(xml_road)
    planned_quads = [_quad(origin,axis,normal,p["start_s_m"],p["end_s_m"],
                     p["low0"],p["high0"],p["low1"],p["high1"]) for p in pieces]
    vertex_errors = []
    for source_quad, actual_quad in zip(planned_quads, metadata.pop("piece_polygons")):
        a = np.asarray(source_quad.exterior.coords)[:4]
        b = np.asarray(actual_quad.exterior.coords)[:4]
        vertex_errors.extend(np.linalg.norm(a-b,axis=1).tolist())
    return road, {"road_id":str(road_id), "component_index":cell["component_index"],
        "source_start_s_m":start, "source_end_s_m":end, "length_m":length,
        "source_pieces":pieces, "natural_record_stations_m":natural,
        "written_record_stations_m":written_stations,
        "expected_xml_sha256":_sha(_canonical(xml_road)),
        "serialization_adjustments":adjustments,
        "expected_max_vertex_serialization_error_m":max(vertex_errors,default=0.0),
        "expected_written_metadata":metadata}, actual, planned_quads


def append_polygon(doc, geometry, junction_id, road_ids: Iterable[int], *, preferred_axis,
                   lane_type="restricted", provenance=None):
    """Append auxiliary line roads, atomically, and return JSON source evidence.

    ``geometry`` includes interior rings. ``preferred_axis`` is an explicit
    nonzero 2-vector. No axis search or smoothing takes place. ID exhaustion,
    collisions, unsupported precision, or invalid source leave ``doc`` intact.
    """
    polygons = _polygons(geometry)
    axis = np.asarray(preferred_axis,dtype=float)
    if axis.shape != (2,) or not np.isfinite(axis).all() or np.linalg.norm(axis) == 0:
        raise SurfaceCellError("explicit-finite-nonzero-axis-required")
    axis = axis/np.linalg.norm(axis)
    normal = np.array([-axis[1],axis[0]])
    if lane_type not in {"restricted","median"}:
        raise SurfaceCellError("auxiliary-lane-type-required")
    if not isinstance(junction_id,int) or isinstance(junction_id,bool) or junction_id < 0:
        raise SurfaceCellError("nonnegative-integer-junction-id-required")
    try:
        provenance = json.loads(json.dumps(provenance or {},allow_nan=False))
    except (TypeError,ValueError) as exc:
        raise SurfaceCellError("json-provenance-required") from exc
    if not isinstance(provenance,dict):
        raise SurfaceCellError("object-provenance-required")
    if provenance.get("eligibility","excluded") != "excluded":
        raise SurfaceCellError("auxiliary-surface-must-be-excluded")
    source_hash = _sha(geometry.wkb)
    origin = np.asarray(polygons[0].exterior.coords[0],float) if polygons else np.array([0.,0.])
    cells,vertices,topology,normalizations = [],[],[],[]
    for component_index,polygon in enumerate(polygons):
        c,v,t,n = _sweep_polygon(polygon,component_index,origin,axis,normal)
        cells.extend(c)
        vertices.extend(v)
        topology.extend({"component_index":component_index,**item} for item in t)
        normalizations.extend({"component_index":component_index,**item} for item in n)
    ids = iter(road_ids)
    used = {r.road_id for r in doc.roads}
    planned_roads, rows, written_shapes, source_quads = [],[],[],[]
    for index,cell in enumerate(cells):
        try:
            road_id = next(ids)
        except StopIteration as exc:
            raise SurfaceCellError("insufficient-road-ids") from exc
        if not isinstance(road_id,int) or isinstance(road_id,bool) or road_id < 0 or road_id in used:
            raise SurfaceCellError("invalid-or-colliding-road-id",road_id=road_id)
        used.add(road_id)
        prov = {**provenance, "eligibility":"excluded", "role":"paving",
            "status":provenance.get("status","TRANSFORMED"),
            "support_kind":provenance.get("support_kind","source-polygon-cells"),
            "source_geometry_sha256":source_hash, "representation":SCHEMA,
            "source_cell_index":index, "source_component_index":cell["component_index"],
            "exclusion_code":"source-polygon-paving"}
        road,row,shape,quads = _road_from_cell(cell,origin,axis,normal,road_id,junction_id,lane_type,prov)
        planned_roads.append(road); rows.append(row); written_shapes.append(shape); source_quads.extend(quads)
    planned_union = unary_union(source_quads)
    scale = max([1.]+[abs(float(c)) for p in polygons for ring in [p.exterior,*p.interiors]
                      for xy in ring.coords for c in xy])
    arithmetic_roundoff = 64*np.finfo(float).eps*scale
    if (not planned_union.difference(geometry.buffer(arithmetic_roundoff)).is_empty
            or not geometry.difference(planned_union.buffer(arithmetic_roundoff)).is_empty):
        raise SurfaceCellError("analytic-partition-does-not-cover-source",
            symmetric_difference_m2=float(planned_union.symmetric_difference(geometry).area))
    max_error = max((r["expected_max_vertex_serialization_error_m"] for r in rows),default=0.0)
    # Derived solely from the expected %.10g encoding, before an external XML
    # is examined. Altered XML cannot enlarge its own admissible error band.
    numeric_band = arithmetic_roundoff + 4*max_error
    for road in planned_roads:
        doc.add_road(road)
    spans = [b-a for r in rows for a,b in zip(r["written_record_stations_m"][:-1],
                                             r["written_record_stations_m"][1:])]
    return {"schema":SCHEMA,"road_ids":[str(r.road_id) for r in planned_roads],
        "source_geometry_sha256":source_hash,"source_geometry_wkb_hex":geometry.wkb.hex(),
        "source_area_m2":float(geometry.area),"source_components":len(polygons),
        "source_holes":sum(len(p.interiors) for p in polygons),
        "sweep_origin_xy":origin.tolist(),"sweep_axis":axis.tolist(),
        "source_vertices":vertices,"natural_topology_events":topology,"cells":rows,
        "numerical_station_normalizations":normalizations,
        "complexity":{"line_roads":len(rows),"lane_sections":len(rows),
            "width_records":sum(len(r["source_pieces"]) for r in rows),
            "source_vertices":len(vertices),
            "natural_station_events":sum(len({v["normalized_s_m"] for v in vertices if v["component_index"]==i})
                                           for i in range(len(polygons))),
            "minimum_width_event_span_m":min(spans,default=None),
            "minimum_road_length_m":min((r["length_m"] for r in rows),default=None),
            "sampling_grid_events":0,"endpoint_padding_m":0.0},
        "numeric_arithmetic_roundoff_m":arithmetic_roundoff,
        "expected_max_vertex_serialization_error_m":max_error,
        "numeric_serialization_band_m":numeric_band,
        "analytic_partition_symmetric_difference_m2":float(planned_union.symmetric_difference(geometry).area),
        "production_qualification":"NOT_EVALUATED"}


def _read_road(road):
    if road is None or road.get("name") != "junction_paving" or road.find("link") is not None:
        raise SurfaceCellError("unlinked-auxiliary-road-required")
    geoms = road.findall("planView/geometry")
    sections = road.findall("lanes/laneSection")
    if (len(geoms)!=1 or len(list(geoms[0]))!=1 or geoms[0][0].tag!="line"
            or len(sections)!=1 or _finite(sections[0].get("s"))!=0
            or _finite(geoms[0].get("s"))!=0):
        raise SurfaceCellError("single-line-single-section-required")
    if road.find("elevationProfile/elevation") is not None or road.find("lateralProfile/*") is not None:
        raise SurfaceCellError("planar-auxiliary-surface-required")
    lanes = sections[0].findall("right/lane")
    if (len(lanes)!=1 or sections[0].findall("left/lane") or lanes[0].get("id")!="-1"
            or lanes[0].get("type") not in {"restricted","median"} or lanes[0].find("link") is not None):
        raise SurfaceCellError("one-unlinked-auxiliary-right-lane-required")
    length = _finite(road.get("length"))
    if length<=0 or _finite(geoms[0].get("length"))!=length:
        raise SurfaceCellError("invalid-written-cell-length")
    widths = lanes[0].findall("width")
    offsets = road.findall("lanes/laneOffset")
    if not widths or len(widths)!=len(offsets) or lanes[0].find("border") is not None:
        raise SurfaceCellError("paired-linear-width-offset-records-required")
    stations = [_finite(w.get("sOffset")) for w in widths]
    if (stations[0]!=0 or stations!=[_finite(o.get("s")) for o in offsets]
            or any(b<=a for a,b in zip(stations[:-1],stations[1:])) or stations[-1]>=length):
        raise SurfaceCellError("duplicate-or-invalid-written-stations")
    origin=np.array([_finite(geoms[0].get("x")),_finite(geoms[0].get("y"))])
    heading=_finite(geoms[0].get("hdg")); axis=np.array([math.cos(heading),math.sin(heading)])
    normal=np.array([-axis[1],axis[0]])
    shapes,minimum_width=[],math.inf
    for i,(width,offset) in enumerate(zip(widths,offsets)):
        if any(_finite(r.get(k,"0"))!=0 for r in (width,offset) for k in ("c","d")):
            raise SurfaceCellError("nonlinear-written-record-not-supported")
        start=stations[i]; end=stations[i+1] if i+1<len(stations) else length
        wa,wb=_finite(width.get("a")),_finite(width.get("b"))
        oa,ob=_finite(offset.get("a")),_finite(offset.get("b"))
        w0,w1=wa,wa+wb*(end-start); h0,h1=oa,oa+ob*(end-start)
        minimum_width=min(minimum_width,w0,w1)
        if min(w0,w1)<0:
            raise SurfaceCellError("negative-written-width",width_m=min(w0,w1))
        shape=_quad(origin,axis,normal,start,end,h0-w0,h0,h1-w1,h1)
        if not shape.is_valid or shape.area<=0:
            raise SurfaceCellError("invalid-written-width-cell")
        shapes.append(shape)
    return unary_union(shapes), {"minimum_written_width_m":minimum_width,
        "duplicate_station_count":0,"width_records":len(widths),"piece_polygons":shapes}


def written_geometry(root, road_ids):
    """Exact piecewise-linear footprint from actual XML, not viewer sampling."""
    ids = [str(i) for i in road_ids]
    if len(ids)!=len(set(ids)):
        raise SurfaceCellError("duplicate-requested-road-id")
    for connection in root.findall("junction/connection"):
        if any(connection.get(key) in ids for key in ("incomingRoad","connectingRoad")):
            raise SurfaceCellError("auxiliary-cell-referenced-by-routing")
    shapes=[]
    for rid in ids:
        matches=[r for r in root.findall("road") if r.get("id")==rid]
        if len(matches)!=1:
            raise SurfaceCellError("missing-or-duplicate-xml-road",road_id=rid)
        shapes.append(_read_road(matches[0])[0])
    return unary_union(shapes)


def audit_written(root, evidence):
    """Return a PASS/FAIL for this source family; whole-map gates stay separate."""
    report={"status":"FAIL","passed":False,"per_cell":[],"duplicate_station_count":0,
            "minimum_written_width_m":None}
    try:
        if evidence.get("schema")!=SCHEMA:
            raise SurfaceCellError("unknown-source-cell-evidence")
        source=wkb.loads(bytes.fromhex(evidence["source_geometry_wkb_hex"]))
        if _sha(source.wkb)!=evidence["source_geometry_sha256"]:
            raise SurfaceCellError("source-geometry-binding-mismatch")
        actual=written_geometry(root,evidence["road_ids"])
        band=_finite(evidence["numeric_serialization_band_m"])
        if band<0:
            raise SurfaceCellError("invalid-numeric-serialization-band")
        for cell in evidence["cells"]:
            road=next(r for r in root.findall("road") if r.get("id")==cell["road_id"])
            _shape,meta=_read_road(road)
            meta.pop("piece_polygons")
            report["per_cell"].append({"road_id":cell["road_id"],
                "expected_xml_unchanged":_sha(_canonical(road))==cell["expected_xml_sha256"],**meta})
        outward=actual.difference(source)
        missing=source.difference(actual)
        within=(actual.difference(source.buffer(band)).is_empty
                and source.difference(actual.buffer(band)).is_empty)
        unchanged=all(r["expected_xml_unchanged"] for r in report["per_cell"])
        report.update(status="PASS" if within and unchanged else "FAIL",passed=within and unchanged,
            source_minus_written_m2=float(missing.area),written_minus_source_m2=float(outward.area),
            symmetric_difference_m2=float(actual.symmetric_difference(source).area),
            numeric_serialization_band_m=band,within_numeric_serialization_band=within,
            minimum_written_width_m=min((r["minimum_written_width_m"] for r in report["per_cell"]),default=None),
            source_holes=evidence["source_holes"],source_components=evidence["source_components"],
            actual_area_m2=float(actual.area),source_area_m2=float(source.area),
            whole_map_gates_evaluated=False)
    except (SurfaceCellError,ValueError,KeyError,TypeError,StopIteration) as exc:
        report.update(error=str(exc),error_code=getattr(exc,"code","invalid-source-cell-evidence"))
        if getattr(exc,"code","")=="duplicate-or-invalid-written-stations":
            report["duplicate_station_count"]=1
    return report
