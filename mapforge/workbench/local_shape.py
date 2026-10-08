"""Frozen-domain shape comparison, not a replacement for map/source gates.

This deliberately conservative contract can reject every nonzero correction of
a straight baseline. NO_REGRESSION_FOUND grants neither an editing capability
nor release approval. Callers must still establish source fidelity and all map
gates. No production converter or existing editor imports this opt-in checker.
"""
from __future__ import annotations

import copy
from fractions import Fraction
import hashlib
import json
import math
from numbers import Real
import xml.etree.ElementTree as ET

from numpy.polynomial import Polynomial as P

SCHEMA = "mapforge/local-shape-comparison/v1"
CONTRACT_SCHEMA = "mapforge/local-shape-contract/v1"
# Floating-point comparison bands, not permitted geometric regression budgets.
NUMERIC_BAND = 1e-10
JET_BAND = 1e-9


class Unavailable(ValueError):
    pass


def _number(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise Unavailable("non-finite-or-nonnumeric-contract")
    return float(value)


def make_contract(baseline: bytes, road_id: str, domain, comparison_stations_m,
                  *, editable_width_lane_ids, allow_lane_offset=False):
    contract = {"schema": CONTRACT_SCHEMA,
                "baseline_sha256": hashlib.sha256(baseline).hexdigest(),
                "road_id": road_id, "domain": list(domain),
                "comparison_stations_m": list(comparison_stations_m),
                "editable_width_lane_ids": list(editable_width_lane_ids),
                "allow_lane_offset": allow_lane_offset}
    _contract(contract, baseline)
    return contract


def _contract(value, baseline):
    fields = {"schema", "baseline_sha256", "road_id", "domain", "comparison_stations_m",
              "editable_width_lane_ids", "allow_lane_offset"}
    if not isinstance(value, dict) or set(value) != fields or value.get("schema") != CONTRACT_SCHEMA:
        raise Unavailable("unsupported-or-incomplete-contract")
    if value["baseline_sha256"] != hashlib.sha256(baseline).hexdigest():
        raise Unavailable("baseline-binding-mismatch")
    if not isinstance(value["road_id"], str) or not value["road_id"]:
        raise Unavailable("invalid-road-identity")
    if not isinstance(value["domain"], list) or len(value["domain"]) != 2:
        raise Unavailable("invalid-domain")
    lo, hi = map(_number, value["domain"])
    cells = value["comparison_stations_m"]
    if not isinstance(cells, list) or len(cells) < 2 or len(cells) > 100:
        raise Unavailable("invalid-comparison-partition")
    cells = list(map(_number, cells))
    if not 0 <= lo < hi or cells[0] != lo or cells[-1] != hi or any(b <= a for a,b in zip(cells,cells[1:])):
        raise Unavailable("comparison-partition-does-not-cover-domain")
    ids = value["editable_width_lane_ids"]
    if not isinstance(ids, list) or any(type(i) is not int or i == 0 for i in ids) or len(ids) != len(set(ids)):
        raise Unavailable("invalid-editable-width-identities")
    if type(value["allow_lane_offset"]) is not bool:
        raise Unavailable("invalid-offset-permission")
    return lo, hi, cells


def _xml(data):
    if not isinstance(data, bytes) or not data or len(data) > 32 * 1024 * 1024:
        raise Unavailable("invalid-xml-bytes")
    # Only UTF-8 is supported, so UTF-16/32 cannot hide declarations from the
    # lexical DTD/entity rejection. The original bytes remain the binding.
    source = data.decode("utf-8-sig")
    if "\x00" in source:
        raise Unavailable("unsupported-xml-encoding")
    if "<!DOCTYPE" in source.upper() or "<!ENTITY" in source.upper():
        raise Unavailable("xml-dtd-or-entity")
    root = ET.fromstring(source)
    if root.tag != "OpenDRIVE":
        raise Unavailable("unsupported-xml-root")
    ids = [r.get("id") for r in root.findall("road")]
    if None in ids or len(ids) != len(set(ids)):
        raise Unavailable("duplicate-or-missing-road-identity")
    return root


def _semantic(e):
    return (e.tag, tuple(sorted(e.attrib.items())), (e.text or "").strip(),
            tuple(_semantic(c) for c in e))


def _road(root, rid):
    return next((r for r in root.findall("road") if r.get("id") == rid), None)


def _section(r, lo, hi):
    secs = r.findall("lanes/laneSection")
    starts = [float(s.get("s")) for s in secs]
    if not starts or starts[0] != 0 or any(b <= a for a,b in zip(starts, starts[1:])):
        raise Unavailable("invalid-lane-section-order")
    length = float(r.get("length"))
    for i, sec in enumerate(secs):
        end = starts[i+1] if i+1 < len(starts) else length
        if starts[i] <= lo < hi <= end:
            return sec
    raise Unavailable("domain-crosses-section-or-road-end")


def _lanes(sec):
    ls = sec.findall("left/lane") + sec.findall("right/lane")
    result = {int(l.get("id")): l for l in ls}
    if len(result) != len(ls) or 0 in result:
        raise Unavailable("invalid-lane-identities")
    return result


def _poly(records, attr, origin, at, anchor):
    if not records:
        return P([0.])
    starts = [Fraction.from_float(float(origin) + float(e.get(attr))) for e in records]
    if any(not math.isfinite(s) for s in starts) or any(b <= a for a,b in zip(starts,starts[1:])):
        raise Unavailable("invalid-polynomial-order")
    eligible = [i for i,s in enumerate(starts) if s <= at]
    if not eligible:
        raise Unavailable("polynomial-does-not-cover-domain")
    i = eligible[-1]
    c = [float(records[i].get(k,"0")) for k in "abcd"]
    if not all(math.isfinite(v) for v in c):
        raise Unavailable("nonfinite-polynomial")
    return P(c)(P([float(anchor-starts[i]), 1.]))


def _different_on_interval(delta, span):
    """Conservative bounds in metres and station derivatives, not coefficients.

    On 0 <= ds <= span the sum of |c_i| span**i bounds the whole
    polynomial. Rational arithmetic prevents a long interval overflowing or
    hiding a small coefficient's large displacement. The comparison bands
    apply separately to position, slope and second derivative; they are not
    an allowed edit outside the declared domain.
    """
    for order in range(3):
        coefficients = delta.deriv(order).coef
        if not all(math.isfinite(v) for v in coefficients):
            raise Unavailable("nonfinite-polynomial-difference")
        bound = sum((abs(Fraction.from_float(float(v))) * span**i
                     for i, v in enumerate(coefficients)), Fraction(0))
        if bound > Fraction.from_float(NUMERIC_BAND):
            return True
    return False


def _editable_record(record, station_attribute):
    if (set(record.attrib) != {station_attribute, "a", "b", "c", "d"}
            or list(record) or (record.text or "").strip()):
        raise Unavailable("unsupported-editable-polynomial-metadata")


def _scope(before, after, contract):
    rid = contract["road_id"]; lo, hi = contract["domain"]
    old = _road(before,rid)
    new = _road(after,rid)
    if old is None or new is None:
        raise Unavailable("road-not-found")
    a, b = _section(old,lo,hi), _section(new,lo,hi)
    old_lanes, new_lanes = _lanes(a), _lanes(b)
    allowed = contract["editable_width_lane_ids"]
    if not set(allowed).issubset(old_lanes) or set(old_lanes) != set(new_lanes):
        raise Unavailable("lane-scope-mismatch")
    for root in (before,after):
        for road in root.findall("road"):
            # Border and width semantics cannot be mixed by this checker.
            if road.get("id") == rid and road.findall(".//border"):
                raise Unavailable("border-representation-not-supported")
    masked = []
    for root in (before,after):
        root = copy.deepcopy(root)
        road = _road(root,rid); sec = _section(road,lo,hi)
        if contract["allow_lane_offset"]:
            for e in list(road.findall("lanes/laneOffset")):
                _editable_record(e,"s")
                road.find("lanes").remove(e)
        for lid, lane in _lanes(sec).items():
            if lid in allowed:
                for e in list(lane.findall("width")):
                    _editable_record(e,"sOffset")
                    lane.remove(e)
        masked.append(_semantic(root))
    if masked[0] != masked[1]:
        raise Unavailable("outside-editable-xml-structure-changed")
    # Rational midpoints retain every distinct binary64 global record station,
    # including adjacent floats. This is the same explicit station model as
    # local_shape_math, not a formal real-arithmetic reading of decimal XML.
    f = lambda v: Fraction.from_float(float(v))
    length = f(old.get("length"));lo,hi=f(lo),f(hi)
    cuts = {Fraction(0),length,lo,hi}
    for r in (old,new):
        cuts.update(f(e.get("s")) for e in r.findall("lanes/laneOffset"))
        for sec in r.findall("lanes/laneSection"):
            s0 = float(sec.get("s"));cuts.add(f(s0))
            cuts.update(f(s0+float(w.get("sOffset"))) for w in sec.findall(".//width"))
    cuts = sorted(cuts); changed = False
    for x,y in zip(cuts,cuts[1:]):
        if x == y: continue
        middle=(x+y)/2
        if not x < middle < y:
            raise Unavailable("unresolvable-record-interval")
        sections = [max((sec for sec in r.findall("lanes/laneSection") if f(sec.get("s"))<=middle),
                        key=lambda sec:f(sec.get("s"))) for r in (old,new)]
        pairs = [(old.findall("lanes/laneOffset"),new.findall("lanes/laneOffset"),"s",Fraction(0))]
        aa,bb = map(_lanes,sections)
        if set(aa) != set(bb): raise Unavailable("outside-lane-layout-changed")
        for lid in aa:
            pairs.append((aa[lid].findall("width"),bb[lid].findall("width"),"sOffset",f(sections[0].get("s"))))
        for p,q,attr,origin in pairs:
            pp,qq = [_poly(rr,attr,origin,middle,x) for rr in (p,q)]
            delta = qq-pp
            different = _different_on_interval(delta, y-x)
            if y <= lo or x >= hi:
                if different: raise Unavailable("outside-domain-polynomial-changed")
            else:
                changed |= different
                if attr == "sOffset":
                    for width in (pp,qq):
                        roots = width.deriv().roots()
                        span=float(y-x)
                        points = [0.,span]+[float(z.real) for z in roots if abs(z.imag)<1e-12 and 0<z.real<span]
                        if min(float(width(t)) for t in points) < -NUMERIC_BAND:
                            raise Unavailable("negative-width-inside-domain")
    if not changed: raise Unavailable("no-geometric-change")
    return old,new


def _curve_map(report):
    return {(lane["lane_id"],name):curve for lane in report["lanes"]
            for name,curve in lane["curves"].items()}


def _subsequence(a,b):
    it=iter(b)
    return all(any(item == candidate for candidate in it) for item in a)


def compare(baseline: bytes, candidate: bytes, contract: dict):
    """Inspect immutable bytes under a baseline-bound, predeclared contract.

    Caller-authored contracts are not source authorization. The report explicitly
    retains that missing acceptance layer, even for NO_REGRESSION_FOUND.
    """
    from .local_shape_math import analyze_road, UnavailableError
    result={"schema":SCHEMA,"status":"UNAVAILABLE","formal_delivery":False,
            "editing_capability_granted":False,"source_fidelity_evaluated":False,
            "whole_map_gates_replaced":False,"baseline_sha256":hashlib.sha256(baseline).hexdigest() if isinstance(baseline,bytes) else None,
            "candidate_sha256":hashlib.sha256(candidate).hexdigest() if isinstance(candidate,bytes) else None,"contract":None,
            "numeric_comparison_band":NUMERIC_BAND,"issues":[],"curves":[],
            "limitations":["conservative frozen-domain comparison, not universal repair acceptance",
                           "zero-curvature baseline can reject all bending; no relaxed floor",
                           "joint rate jumps are not compared; no C3 continuity guarantee",
                           "source contract, whole-map checks and delivery remain independent"]}
    try:
        if not isinstance(baseline,bytes) or not isinstance(candidate,bytes):
            raise Unavailable("invalid-xml-bytes")
        lo,hi,cells=_contract(contract,baseline)
        # Never echo an invalid/nonfinite caller object into a JSON receipt.
        result["contract"]=json.loads(json.dumps(contract,allow_nan=False))
        before,after=_xml(baseline),_xml(candidate)
        old,new=_scope(before,after,contract)
        computation_cuts=set(cells)
        for road in (old,new):
            computation_cuts.update(float(e.get("s")) for e in road.findall("planView/geometry")+road.findall("lanes/laneOffset"))
            sec=_section(road,lo,hi);origin=float(sec.get("s"))
            computation_cuts.update(origin+float(e.get("sOffset")) for e in sec.findall(".//width"))
        computation_cuts=sorted(x for x in computation_cuts if lo<=x<=hi)
        analyses=[analyze_road(r,domain=(lo,hi),extra_cuts=computation_cuts) for r in (old,new)]
        maps=list(map(_curve_map,analyses))
        if maps[0].keys()!=maps[1].keys() or not maps[0]:
            raise Unavailable("incomplete-physical-curve-coverage")
        issues=[];rows=[]
        for key,bc in maps[0].items():
            cc=maps[1][key]; row={"lane_id":key[0],"curve":key[1],"cells":[],
                "before_sign_sequence":bc["sign_sequence"],"after_sign_sequence":cc["sign_sequence"],
                "before_roots":bc["roots"],"after_roots":cc["roots"]}
            for x,y in zip(cells,cells[1:]):
                peaks=[]
                for curve in (bc,cc):
                    pieces=[p for p in curve["pieces"] if x<=p["domain"][0] and p["domain"][1]<=y]
                    if not pieces or pieces[0]["domain"][0]!=x or pieces[-1]["domain"][1]!=y:
                        raise Unavailable("incomplete-frozen-cell")
                    peaks.append({m:{
                        **max((p[m]["max_abs"] for p in pieces),key=lambda v:abs(v["value"])),
                        "max_abs_lower_bound":max(p[m]["max_abs_lower_bound"] for p in pieces),
                        "max_abs_upper_bound":max(p[m]["max_abs_upper_bound"] for p in pieces),
                    } for m in ("kappa","rate")})
                row["cells"].append({"domain":[x,y],"before":peaks[0],"after":peaks[1]})
                for metric in ("kappa","rate"):
                    base, candidate_peak = peaks[0][metric], peaks[1][metric]
                    band = Fraction.from_float(NUMERIC_BAND)
                    if (Fraction.from_float(candidate_peak["max_abs_lower_bound"])
                            > Fraction.from_float(base["max_abs_upper_bound"])+band):
                        issues.append({"code":"local-"+metric+"-regression","lane_id":key[0],"curve":key[1],"domain":[x,y],
                                       "before":peaks[0][metric],"after":peaks[1][metric]})
                    elif (Fraction.from_float(candidate_peak["max_abs_upper_bound"])
                            > Fraction.from_float(base["max_abs_lower_bound"])+band):
                        raise Unavailable("extremum-enclosure-crosses-comparison-band")
            if not _subsequence(cc["sign_sequence"],bc["sign_sequence"]):
                issues.append({"code":"new-curvature-reversal-or-direction","lane_id":key[0],"curve":key[1]})
            # Both endpoints stay at their original local/world C2 state.
            for label,jb,jc in [("start",bc["pieces"][0]["start_jet"],cc["pieces"][0]["start_jet"]),
                                ("end",bc["pieces"][-1]["end_jet"],cc["pieces"][-1]["end_jet"])]:
                for component in ("x","y","heading","kappa","t","dt_ds","d2t_ds2"):
                    if abs(jb[component]-jc[component])>JET_BAND:
                        issues.append({"code":"domain-port-changed","lane_id":key[0],"curve":key[1],"end":label,"component":component})
            bj={v["s_m"]:v for v in bc["breakpoints"]}
            cj={v["s_m"]:v for v in cc["breakpoints"]}
            if bj.keys()!=cj.keys():raise Unavailable("incomplete-common-joint-evaluation")
            for station in bj:
                if bj[station]["left_jet"] is None or bj[station]["right_jet"] is None:
                    continue
                # The reference geometry is immutable. Local jets therefore
                # retain real discontinuities even when adding a large world
                # origin rounds both Cartesian positions to the same float.
                for component in ("x","y","heading","kappa","t","dt_ds","d2t_ds2"):
                    jumps=[abs(j[station]["left_jet"][component]-j[station]["right_jet"][component]) for j in (bj,cj)]
                    if jumps[1]>jumps[0]+JET_BAND:
                        issues.append({"code":"joint-"+component+"-regression","lane_id":key[0],"curve":key[1],"s_m":station,
                                       "before":jumps[0],"after":jumps[1]})
            rows.append(row)
        result.update(status="REGRESSION" if issues else "NO_REGRESSION_FOUND",issues=issues,curves=rows,
                      comparison_partition=cells,computation_partition=computation_cuts,
                      numerical_method=analyses[0].get("method",analyses[0].get("schema")),
                      mathematical_analysis=analyses)
    except (Unavailable,UnavailableError,ValueError,KeyError,TypeError,OverflowError,ET.ParseError) as exc:
        result.update(status="UNAVAILABLE",issues=[{"code":"incomplete-local-shape-comparison","reason":str(exc)}],curves=[])
    return result
