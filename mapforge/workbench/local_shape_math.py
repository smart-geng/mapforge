"""Read-only planar Frenet analysis of the actual written OpenDRIVE model.

``analyze_road(road, domain, lane_ids=None, extra_cuts=())`` returns extrema of
curvature and d(curvature)/d(lane arclength), signs, and one-sided world jets.
Line/arc/spiral, width records and one laneSection are supported. There is no
quality threshold or PASS in this module. Unsupported/uncertain mathematics
raises UnavailableError; no dense sample grid is used to admit a curve.

Root isolation operates on exact rational Frenet polynomials constructed from
the parsed/translated floating-point lateral/reference parameters. Derivative roots partition [0,1]
into monotone intervals; rational Horner interval evaluation verifies that
critical-root brackets hide no further root. Exact repeated factors are proved
and removed by a rational polynomial gcd; unresolved near-multiple roots are
rejected. This is a complete root accounting
for that float model, not a formal certificate of decimal XML/source data.
No curvature or rate floor changes small nonzero curves into straight lines.
"""
from __future__ import annotations

from bisect import bisect_right
from fractions import Fraction as F
from functools import lru_cache
import math

import numpy as np
from numpy.polynomial import Polynomial as P


class UnavailableError(ValueError):
    """The supported finite model cannot be completely analysed."""


def _fail(reason):
    raise UnavailableError("UNAVAILABLE: " + reason)


def _finite(value, name):
    if isinstance(value, bool):
        _fail(name + " must be finite numeric")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        _fail(name + " must be finite numeric")
    if not math.isfinite(number):
        _fail(name + " must be finite numeric")
    return number


def _attr(element, name):
    if name not in element.attrib:
        _fail("missing " + element.tag + "." + name)
    return _finite(element.get(name), element.tag + "." + name)


def _coefficients(poly):
    co = tuple(float(v) for v in poly.coef)
    if not co or len(co) > 25 or not all(math.isfinite(v) for v in co):
        _fail("nonfinite or unsupported polynomial degree")
    while len(co) > 1 and co[-1] == 0:
        co = co[:-1]
    return co


def _rtrim(co):
    co=tuple(co)
    while len(co)>1 and co[-1]==0:
        co=co[:-1]
    return co


def _rcoeff(poly):
    if isinstance(poly,P):
        return tuple(F(v) for v in _coefficients(poly))
    co=_rtrim(tuple(F(v) for v in poly))
    if not co or len(co)>25:
        _fail("unsupported rational polynomial degree")
    return co


def _radd(*polys):
    result=[F(0)]*max(map(len,polys))
    for p in polys:
        for i,v in enumerate(p):result[i]+=v
    return _rtrim(result)


def _rscale(p,c):
    return _rtrim(tuple(v*c for v in p))


def _rmul(a,b):
    out=[F(0)]*(len(a)+len(b)-1)
    for i,x in enumerate(a):
        for j,y in enumerate(b):out[i+j]+=x*y
    return _rtrim(out)


def _rder(p):
    return _rtrim(tuple(i*p[i] for i in range(1,len(p))) or (F(0),))


def _primitive_integer(poly):
    """Primitive integer representative, reducing exact Euclidean bit growth."""
    den=math.lcm(*(v.denominator for v in poly))
    values=[v.numerator*(den//v.denominator) for v in poly]
    content=math.gcd(*values)
    if not content:return (F(0),)
    if values[-1]<0:content=-content
    return _rtrim(tuple(F(v//content) for v in values))


def _divrem(a,b):
    remainder=list(a); quotient=[F(0)]*max(1,len(a)-len(b)+1)
    while any(remainder) and len(remainder)>=len(b):
        shift=len(remainder)-len(b);factor=remainder[-1]/b[-1]
        quotient[shift]+=factor
        for i,v in enumerate(b):remainder[i+shift]-=factor*v
        remainder=list(_rtrim(remainder))
    return _rtrim(quotient),_rtrim(remainder)


def _squarefree(poly):
    a=_primitive_integer(poly);b=_primitive_integer(_rder(a))
    while any(b):
        _,remainder=_divrem(a,b)
        a,b=b,_primitive_integer(remainder)
    if len(a)==1:
        return None
    quotient,remainder=_divrem(poly,a)
    if any(remainder) or len(quotient)>=len(poly):
        _fail("exact repeated-factor division failed")
    return _primitive_integer(quotient)


def _bernstein_nonzero(controls):
    """Exact sign cover, subdividing proof intervals, never road geometry.

    This can certify a sign-definite polynomial whose *derivative* has an
    arbitrarily ill-conditioned root. Failure to prove a sign is not success:
    the complete root isolator must then account for every root instead.
    """
    target=1 if controls[0]>0 else -1
    if controls[0]==0 or target*controls[-1]<=0:
        return False
    stack=[(controls,0)];visited=0
    while stack:
        values,depth=stack.pop();visited+=1
        if visited>256:return False
        if all(target*v>=0 for v in values):continue
        if target*values[0]<=0 or target*values[-1]<=0 or depth>=40:return False
        levels=[values]
        while len(levels[-1])>1:
            previous=levels[-1]
            levels.append([(a+b)/2 for a,b in zip(previous,previous[1:])])
        if target*levels[-1][0]<=0:return False
        stack.append(([level[0] for level in levels],depth+1))
        stack.append(([level[-1] for level in reversed(levels)],depth+1))
    return True


def _rat_value(co, x):
    value = F(0)
    for a in reversed(co):
        value = value*x + a
    return value


def _rat_interval(co, lo, hi):
    low = high = F(0)
    for a in reversed(co):
        products = (low*lo, low*hi, high*lo, high*hi)
        low, high = min(products)+a, max(products)+a
    return low, high


@lru_cache(maxsize=8192)
def _root_brackets(co):
    """Return disjoint rational enclosures of ALL roots in [0,1].

    Exact endpoint factors (including multiplicity) are removed algebraically.
    Exact repeated factors are removed only after a rational gcd proves them. A
    derivative recursion, not companion-root filtering, establishes completeness.
    """
    original = tuple(F(v) for v in co)
    if not any(original):
        return (), True
    work = [0]

    def solve(coeff):
        work[0] += 1
        if work[0] > 5000:
            _fail("root isolation complexity limit")
        while len(coeff) > 1 and coeff[-1] == 0:
            coeff = coeff[:-1]
        if not any(coeff):
            _fail("unexpected zero derivative polynomial")
        endpoints = []
        if coeff[0] == 0:
            endpoints.append((F(0), F(0)))
            while len(coeff) > 1 and coeff[0] == 0:
                coeff = coeff[1:]
        if _rat_value(coeff, F(1)) == 0:
            endpoints.append((F(1), F(1)))
            while len(coeff) > 1 and _rat_value(coeff, F(1)) == 0:
                q = [F(0)] * (len(coeff)-1)
                q[-1] = coeff[-1]
                for i in range(len(q)-2, -1, -1):
                    q[i] = coeff[i+1]+q[i+1]
                if coeff[0]+q[0] != 0:
                    _fail("endpoint deflation lost identity")
                coeff = tuple(q)
        if len(coeff) == 1:
            return sorted(endpoints)
        # A sign-definite Bernstein hull proves there are no interior roots,
        # without demanding well-separated roots of an irrelevant derivative.
        # Zero controls are safe: every Bernstein basis is strictly positive
        # in (0,1), and the polynomial here is not identically zero.
        degree=len(coeff)-1
        controls=[sum((coeff[j]*F(math.comb(i,j),math.comb(degree,j)) for j in range(i+1)),F(0))
                  for i in range(degree+1)]
        if all(v>=0 for v in controls) or all(v<=0 for v in controls) or _bernstein_nonzero(controls):
            return sorted(endpoints)
        derivative = tuple(i*coeff[i] for i in range(1, len(coeff)))
        critical = [(a,b) for a,b in solve(derivative) if b > 0 and a < 1 and not (a == b and a in (0,1))]
        for a,b in critical:
            low,high = _rat_interval(coeff,a,b)
            if low <= 0 <= high:
                reduced=_squarefree(coeff)
                if reduced is None:
                    _fail("ill-conditioned near-multiple root without exact repeated factor")
                # Deflated endpoints remain roots of the original polynomial.
                return sorted(endpoints+solve(reduced))
        spans = []
        left = F(0)
        for a,b in critical:
            if a < left:
                _fail("overlapping critical-root brackets")
            spans.append((left,a)); left = b
        spans.append((left,F(1)))
        roots = list(endpoints)
        for a,b in spans:
            fa,fb = _rat_value(coeff,a),_rat_value(coeff,b)
            if fa == 0 or fb == 0:
                _fail("unaccounted root at a monotonic partition boundary")
            if fa*fb >= 0:
                continue
            for _ in range(192):
                mid = (a+b)/2
                fm = _rat_value(coeff,mid)
                if fm == 0:
                    a=b=mid
                    break
                if fa*fm < 0:
                    b=mid; fb=fm
                else:
                    a=mid; fa=fm
                if b-a <= F(1,2**112):
                    break
            else:
                _fail("root bracket did not converge")
            low,high = _rat_interval(derivative,a,b)
            if low <= 0 <= high:
                _fail("root conditioning cannot be verified")
            roots.append((a,b))
        roots.sort()
        if any(b >= c for (_,b),(c,_) in zip(roots,roots[1:])):
            _fail("root brackets not separated")
        return roots

    return tuple(solve(original)), False


def _roots(poly):
    return _root_brackets(_rcoeff(poly))


def _value(poly, u):
    # Exact rational Horner avoids cancellation silently producing a false zero.
    return _to_float(_rat_value(_rcoeff(poly), F(float(u))))


def _to_float(exact):
    value=float(exact)
    if not math.isfinite(value) or (exact != 0 and value == 0):
        _fail("nonfinite/underflowed polynomial evaluation")
    return value


def _root_rows(poly, lo, length):
    brackets,zero = _roots(poly)
    rows = []
    for a,b in brackets:
        station = lo + length*float((a+b)/2)
        row={"s_m":station,"bracket_s":_station_bracket(a,b,lo,length),
             "endpoint":"start" if a==b==0 else "end" if a==b==1 else None}
        if rows and row["bracket_s"][0] <= rows[-1]["bracket_s"][1]:
            _fail("distinct roots cannot be resolved in global station coordinates")
        rows.append(row)
    return rows,zero


def _station_bracket(a,b,lo,length):
    # Convert once, with a checked direction. Rounding u, multiplying, adding
    # and then expanding one ulp can still miss the exact transformed endpoint.
    first=F(lo)+F(length)*a
    last=F(lo)+F(length)*b
    return [_outward_float(first,False),_outward_float(last,True)]


def _ordered(elements, attr, name):
    starts = [_attr(e,attr) for e in elements]
    if any(b <= a for a,b in zip(starts,starts[1:])):
        _fail(name + " records must be strictly ordered without duplicates")
    return starts


def _active(elements, starts, mid, name):
    i = bisect_right(starts,mid)-1
    if i < 0:
        _fail(name + " has no active record")
    return elements[i], i


def _record_poly(elements, starts, lo, mid, start_attr, origin=0.0):
    if not elements:
        return P([0.])
    global_starts=[origin+s for s in starts]
    record,index = _active(elements,global_starts,mid,"polynomial")
    co = [_attr(record,k) for k in ("a","b","c","d")]
    return P(co)(P([lo-global_starts[index],1.]))


def _primitive(geometry):
    values = {k:_attr(geometry,k) for k in ("s","x","y","hdg","length")}
    if values["length"] <= 0:
        _fail("nonpositive reference primitive length")
    children = [e for e in geometry if e.tag in ("line","arc","spiral","poly3","paramPoly3")]
    if len(children) != 1 or children[0].tag not in ("line","arc","spiral"):
        _fail("only line/arc/spiral reference primitives are supported")
    child=children[0]; values["kind"]=child.tag
    if child.tag == "line":
        values.update(k0=0.,k1=0.)
    elif child.tag == "arc":
        values["k0"]=values["k1"]=_attr(child,"curvature")
    else:
        values.update(k0=_attr(child,"curvStart"),k1=_attr(child,"curvEnd"))
    return values


def _pose(primitive,s):
    x,y,h = [primitive[k] for k in ("x","y","hdg")]
    ds=s-primitive["s"]; k=primitive["k0"]
    sharp=(primitive["k1"]-k)/primitive["length"]
    if ds == 0:
        return x,y,h
    if sharp == 0:
        angle=k*ds/2
        sinc=math.sin(angle)/angle if angle else 1.
        return x+ds*sinc*math.cos(h+angle),y+ds*sinc*math.sin(h+angle),h+2*angle
    try:
        from pyclothoids import Clothoid
        c=Clothoid.StandardParams(x,y,h,k,sharp,ds)
        result=(float(c.XEnd),float(c.YEnd),float(c.ThetaEnd))
    except Exception as exc:
        _fail("spiral pose unavailable: "+str(exc))
    if not all(math.isfinite(v) for v in result):
        _fail("nonfinite spiral pose")
    return result


def _peak(rows):
    if not rows:
        _fail("empty extrema candidates")
    result={"minimum":min(rows,key=lambda r:r["value"]),
            "maximum":max(rows,key=lambda r:r["value"]),
            "max_abs":max(rows,key=lambda r:abs(r["value"]))}
    if all("lower_bound" in row and "upper_bound" in row for row in rows):
        result.update(minimum_lower_bound=min(r["lower_bound"] for r in rows),
                      minimum_upper_bound=min(r["upper_bound"] for r in rows),
                      maximum_lower_bound=max(r["lower_bound"] for r in rows),
                      maximum_upper_bound=max(r["upper_bound"] for r in rows),
                      max_abs_lower_bound=max(0. if r["lower_bound"]<=0<=r["upper_bound"] else min(abs(r["lower_bound"]),abs(r["upper_bound"])) for r in rows),
                      max_abs_upper_bound=max(max(abs(r["lower_bound"]),abs(r["upper_bound"])) for r in rows))
    return result


def _sqrt_bracket(value):
    """Binary64 enclosure whose squared rational endpoints are verified."""
    approximate=math.sqrt(_to_float(value))
    lower=upper=approximate
    for _ in range(8):
        if F(lower)**2<=value<=F(upper)**2:
            return F(lower),F(upper)
        if F(lower)**2>value:lower=math.nextafter(lower,-math.inf)
        if F(upper)**2<value:upper=math.nextafter(upper,math.inf)
    _fail("square root enclosure cannot be verified")


def _outward_float(value,upward):
    approximate=_to_float(value)
    if (upward and F(approximate)<value) or (not upward and F(approximate)>value):
        approximate=math.nextafter(approximate,math.inf if upward else -math.inf)
    if not math.isfinite(approximate):
        _fail("extremum interval outside finite float range")
    return approximate


def _piece(t_s, primitive, lo, hi):
    length=hi-lo
    if not math.isfinite(length) or length <= 0:
        _fail("nonpositive/invalid analysis cell")
    exact_length=F(length)
    # Keep algebraic factors intact. Expanding N, D and their derivatives with
    # binary64 multiplication can split a structural double root into a false
    # close pair, or remove it. Only lateral extraction/reference parameters
    # are float-model inputs; Frenet algebra from this point is rational.
    t=tuple(v*exact_length**i for i,v in enumerate(_rcoeff(t_s)))
    d1=_rscale(_rder(t),1/exact_length)
    d2=_rscale(_rder(d1),1/exact_length)
    d3=_rscale(_rder(d2),1/exact_length)
    sharp=(primitive["k1"]-primitive["k0"])/primitive["length"]
    exact_sharp=F(sharp)
    k=(F(primitive["k0"]+sharp*(lo-primitive["s"])),exact_sharp*exact_length)
    regular=_radd((F(1),),_rscale(_rmul(k,t),-1))
    # A=1-k*t is quartic; all its stationary values and zeros are checked.
    regular_roots,regular_zero=_roots(regular)
    critical,_=_roots(_rder(regular))
    regular_points=[F(0),F(1)]+[(a+b)/2 for a,b in critical]
    regular_rows=[{"s_m":lo+length*float(u),"value":_value(regular,float(u))} for u in regular_points]
    regular_min=min(regular_rows,key=lambda r:r["value"])
    if regular_zero or regular_roots or regular_min["value"] <= 0:
        _fail("nonregular Frenet frame (1-k*t not strictly positive)")
    speed2=_radd(_rmul(regular,regular),_rmul(d1,d1))
    numerator=_radd(_rmul(regular,_radd(_rmul(k,regular),d2)),
                    _rmul(d1,_radd(_rscale(t,exact_sharp),_rscale(_rmul(k,d1),2))))
    b=_radd(_rmul(_rder(numerator),speed2),_rscale(_rmul(numerator,_rder(speed2)),-F(3,2)))
    c=_radd(_rmul(_rder(b),speed2),_rscale(_rmul(b,_rder(speed2)),-3))
    for polynomial in (t,regular,speed2,numerator,b,c):
        _rcoeff(polynomial)
    root_rows,zero=_root_rows(numerator,lo,length)
    curvature_roots,_=_roots(b)
    rate_roots,_=_roots(c)

    def values(u):
        exact_u=F(float(u));q=_rat_value(speed2,exact_u)
        if q <= 0:
            _fail("nonpositive lane speed squared")
        n=_rat_value(numerator,exact_u)
        curvature=_to_float(n/q)/math.sqrt(_to_float(q))
        rate=_to_float(_rat_value(b,exact_u)/(exact_length*q*q*q))
        if not math.isfinite(curvature) or not math.isfinite(rate) or (n!=0 and curvature==0):
            _fail("nonfinite curvature/rate")
        return curvature,rate

    def value_bounds(a,end,index):
        qlow,qhigh=_rat_interval(speed2,a,end)
        if qlow<=0:
            _fail("extremum interval cannot establish positive lane speed")
        nlow,nhigh=_rat_interval(numerator if index==0 else b,a,end)
        if index==0:
            slow,_=_sqrt_bracket(qlow);_,shigh=_sqrt_bracket(qhigh)
            lowden,highden=qlow*slow,qhigh*shigh
        else:
            lowden,highden=exact_length*qlow**3,exact_length*qhigh**3
        if lowden<=0:
            _fail("extremum interval denominator not positive")
        quotients=(nlow/lowden,nlow/highden,nhigh/lowden,nhigh/highden)
        low=_outward_float(min(quotients),False);high=_outward_float(max(quotients),True)
        if high-low>1e-12:
            _fail("extremum uncertainty exceeds numerical availability bound (1e-12)")
        return {"lower_bound":low,"upper_bound":high,"uncertainty":high-low}

    def metric(brackets,index):
        stationary=[{"s_m":lo+length*float((a+b)/2),"value":values(float((a+b)/2))[index],
                     "bracket_s":_station_bracket(a,b,lo,length),**value_bounds(a,b,index)} for a,b in brackets]
        ends=[{"s_m":lo,"value":values(0.)[index],**value_bounds(F(0),F(0),index)},
              {"s_m":hi,"value":values(1.)[index],**value_bounds(F(1),F(1),index)}]
        return {**_peak(ends+stationary),"stationary_points":stationary}

    def jet(u):
        s=lo+length*u; x,y,h=_pose(primitive,s)
        lateral,first,second,third=[_value(p,u) for p in (t,d1,d2,d3)]
        a=_value(regular,u); curvature,rate=values(u)
        result={"s_m":s,"x":x-lateral*math.sin(h),"y":y+lateral*math.cos(h),
                "heading":h+math.atan2(first,a),"kappa":curvature,"rate":rate,
                "t":lateral,"dt_ds":first,"d2t_ds2":second,"d3t_ds3":third,
                "regularity":a,"speed":math.sqrt(a*a+first*first)}
        if not all(math.isfinite(v) for v in result.values()):
            _fail("nonfinite world jet")
        return result

    nroots,_=_roots(numerator)
    positions=[F(0)]+[(a+b)/2 for a,b in nroots if a>0 and b<1]+[F(1)]
    signs=[]
    nco=numerator
    for a,bound in zip(positions[:-1],positions[1:]):
        value=_rat_value(nco,(a+bound)/2)
        signs.append({"domain":[lo+length*float(a),lo+length*float(bound)],
                      "sign":0 if zero else 1 if value>0 else -1 if value<0 else 0})
        if not zero and value == 0:
            _fail("unaccounted zero in curvature sign interval")
    return {"domain":[lo,hi],"kappa":metric(curvature_roots,0),"rate":metric(rate_roots,1),
            "regularity_min":regular_min,"sign_intervals":signs,"roots":root_rows,
            "identically_zero_curvature":zero,"start_jet":jet(0.),"end_jet":jet(1.),
            "polynomials_u01":{"lateral":[_to_float(v) for v in t],"regularity":[_to_float(v) for v in regular],
                               "curvature_numerator":[_to_float(v) for v in numerator],"speed_squared":[_to_float(v) for v in speed2],
                               "rate_numerator":[_to_float(v) for v in b]},
            "root_accounting":"complete rational Frenet algebra and derivative isolation; exact gcd for proven repeated factors"}


def _curve(pieces):
    signs=[]
    for piece in pieces:
        for item in piece["sign_intervals"]:
            if signs and signs[-1]["sign"] == item["sign"]:
                signs[-1]["domain"][1]=item["domain"][1]
            else:
                signs.append({"domain":list(item["domain"]),"sign":item["sign"]})
    nonzero=[r for r in signs if r["sign"] != 0]
    sequence=[]; reversals=[]
    for item in nonzero:
        if not sequence or sequence[-1] != item["sign"]:
            if sequence:
                previous=next(r for r in reversed(nonzero[:nonzero.index(item)]) if r["sign"] == sequence[-1])
                reversals.append({"domain":[previous["domain"][1],item["domain"][0]],"from_sign":sequence[-1],"to_sign":item["sign"]})
            sequence.append(item["sign"])
    roots=[]
    for p in pieces:
        for row in p["roots"]:
            if roots and row["bracket_s"][0] <= roots[-1]["bracket_s"][1]:
                if not (roots[-1]["endpoint"]=="end" and row["endpoint"]=="start"
                        and roots[-1]["s_m"]==row["s_m"]==p["domain"][0]):
                    _fail("nearby cross-record roots cannot be identified uniquely")
                roots[-1]["bracket_s"][1]=max(row["bracket_s"][1],roots[-1]["bracket_s"][1])
                roots[-1]["endpoint"]="record"
            else:
                roots.append({"s_m":row["s_m"],"bracket_s":list(row["bracket_s"]),"endpoint":row["endpoint"]})
    zero_spans=[p["domain"] for p in pieces if p["identically_zero_curvature"]]
    breaks=[{"s_m":pieces[0]["domain"][0],"left_jet":None,"right_jet":pieces[0]["start_jet"]}]
    breaks.extend({"s_m":b["domain"][0],"left_jet":a["end_jet"],"right_jet":b["start_jet"]} for a,b in zip(pieces,pieces[1:]))
    breaks.append({"s_m":pieces[-1]["domain"][1],"left_jet":pieces[-1]["end_jet"],"right_jet":None})
    def aggregate(metric):
        result=_peak([p[metric][k] for p in pieces for k in ("minimum","maximum")])
        # A close extremum can have the larger enclosure while its midpoint
        # estimate loses a tie. Aggregate every piece's complete bounds.
        for name in ("minimum_lower_bound","minimum_upper_bound"):
            result[name]=min(p[metric][name] for p in pieces)
        for name in ("maximum_lower_bound","maximum_upper_bound","max_abs_lower_bound","max_abs_upper_bound"):
            result[name]=max(p[metric][name] for p in pieces)
        return result
    return {"pieces":pieces,"kappa":aggregate("kappa"),
            "rate":aggregate("rate"),
            "sign_intervals":signs,"sign_sequence":sequence,"sign_change_count":len(reversals),"reversals":reversals,
            "roots":roots,"root_count":None if zero_spans else len(roots),"zero_spans":zero_spans,"breakpoints":breaks}


def analyze_road(road, domain, lane_ids=None, extra_cuts=()):
    """Analyse every requested lane, including median by default, without IO.

    `domain=(s0,s1)` is fixed and may touch, but not cross, a laneSection edge.
    `extra_cuts` lets the caller supply a common before/after partition. Every
    active reference/offset/width record contributes its own breakpoint too.
    Jets use increasing road s and planar meters, not lane travel direction.
    """
    try:
        with np.errstate(all="raise"):
            return _analyze(road,domain,lane_ids,extra_cuts)
    except UnavailableError:
        raise
    except (ArithmeticError,TypeError,ValueError,AttributeError,IndexError,KeyError) as exc:
        _fail("invalid/unsupported local shape: "+str(exc))


def _analyze(road,domain,lane_ids,extra_cuts):
    if road.tag != "road" or isinstance(domain,(str,bytes)) or len(domain) != 2:
        _fail("road XML and two-value domain required")
    lo,hi=(_finite(v,"domain") for v in domain)
    road_length=_attr(road,"length")
    if not 0 <= lo < hi <= road_length:
        _fail("domain outside positive road range")
    # A planar arclength is not the physical arclength of a sloped/banked lane.
    # Fail closed even when a profile happens to be outside this local domain:
    # profile inheritance and lane-height semantics are not implemented here.
    for record in road.findall("elevationProfile/*"):
        if record.tag != "elevation":
            _fail("unsupported elevation profile record")
        _attr(record,"s")
        if any(_attr(record,k) != 0. for k in ("a","b","c","d")):
            _fail("nonzero elevation unsupported by planar shape analysis")
    if road.findall("lateralProfile/*"):
        _fail("lateral profile/superelevation/crossfall/shape unsupported")
    if road.findall("lanes/laneSection/*/lane/height") or road.findall("lanes/laneSection/*/lane/shape"):
        _fail("lane height/shape unsupported by planar shape analysis")
    geometries=road.findall("planView/geometry")
    gs=_ordered(geometries,"s","geometry")
    if not gs or gs[0] != 0:
        _fail("reference coverage must start at zero")
    primitives=[_primitive(g) for g in geometries]
    for i,p in enumerate(primitives):
        end=gs[i+1] if i+1<len(gs) else road_length
        numerical_band=128*np.finfo(float).eps*max(1.,abs(end),abs(p["s"]),abs(p["length"]))
        if abs(p["s"]+p["length"]-end)>numerical_band:
            _fail("reference primitive gap/overlap or changed length")
    sections=road.findall("lanes/laneSection"); ss=_ordered(sections,"s","laneSection")
    if not ss or ss[0] != 0 or any(lo < s < hi for s in ss):
        _fail("domain must lie within one complete laneSection")
    sec,si=_active(sections,ss,(lo+hi)/2,"laneSection")
    if hi > (ss[si+1] if si+1<len(ss) else road_length):
        _fail("domain crosses laneSection")
    sec_s=ss[si]
    if sec.get("singleSide") not in (None,"false"):
        _fail("singleSide laneSections unsupported")
    lanes={}
    for side,sign in (("left",1),("right",-1)):
        side_ids=[]
        for lane in sec.findall(side+"/lane"):
            text=lane.get("id"); lid=int(text)
            if str(lid) != text or lid*sign <= 0 or lid in lanes:
                _fail("invalid/duplicate lane identity")
            lanes[lid]=lane; side_ids.append(abs(lid))
        if side_ids and sorted(side_ids) != list(range(1,max(side_ids)+1)):
            _fail("noncontiguous lane ordering")
    zero_lanes=sec.findall("center/lane")
    if len(zero_lanes) != 1 or zero_lanes[0].get("id") != "0":
        _fail("exactly one lane-zero reference required")
    lanes[0]=zero_lanes[0]
    requested=sorted(lanes) if lane_ids is None else list(lane_ids)
    if not requested or any(isinstance(i,bool) or not isinstance(i,int) or i not in lanes for i in requested) or len(set(requested)) != len(requested):
        _fail("unique existing integer lane_ids required")
    offsets=road.findall("lanes/laneOffset"); os=_ordered(offsets,"s","laneOffset")
    if offsets and os[0] != 0:
        _fail("laneOffset coverage must start at zero")
    widths={}; starts={}
    needed={j for i in requested for j in lanes if j*i>0 and abs(j)<=abs(i)}
    breaks={lo,hi}
    breaks.update(s for s in gs+os if lo<s<hi)
    for cut in extra_cuts:
        cut=_finite(cut,"extra_cut")
        if lo<cut<hi: breaks.add(cut)
    for lid in needed:
        lane=lanes[lid]
        if lane.findall("border"):
            _fail("border representation unsupported; width-only required")
        widths[lid]=lane.findall("width"); starts[lid]=_ordered(widths[lid],"sOffset","width")
        if not widths[lid] or starts[lid][0] != 0:
            _fail("complete width coverage from section zero required")
        breaks.update(sec_s+s for s in starts[lid] if lo<sec_s+s<hi)
        for record in widths[lid]:
            for key in ("a","b","c","d"): _attr(record,key)
    for record in offsets:
        for key in ("a","b","c","d"): _attr(record,key)
    partition=sorted(breaks)
    output={lid:({"center":[]} if lid==0 else {"inner":[],"center":[],"outer":[]}) for lid in requested}
    for a,b in zip(partition,partition[1:]):
        # Selecting with a rational midpoint preserves distinct neighbouring
        # binary64 cuts, even when their float midpoint rounds to an endpoint.
        mid=(F(a)+F(b))/2
        _,gi=_active(geometries,gs,mid,"reference")
        offset=_record_poly(offsets,os,a,mid,"s")
        for lid in requested:
            if lid==0:
                output[lid]["center"].append(_piece(offset,primitives[gi],a,b))
                continue
            inner=offset
            for other in sorted((j for j in needed if j*lid>0 and abs(j)<=abs(lid)),key=abs):
                width=_record_poly(widths[other],starts[other],a,mid,"sOffset",sec_s)
                # Verify nonnegative widths over the complete polynomial cell;
                # requested lanes must have strictly positive width throughout.
                scaled=width(P([0.,b-a])); wr,_=_roots(scaled.deriv())
                zeros,identically_zero=_roots(scaled)
                minimum=min(_value(scaled,float(u)) for u in [F(0),F(1)]+[(x+y)/2 for x,y in wr])
                if minimum < 0 or (other==lid and (minimum <= 0 or zeros or identically_zero)):
                    _fail("nonpositive requested lane / negative intermediate width")
                outer=inner+(1 if lid>0 else -1)*width
                if other == lid: break
                inner=outer
            for name,poly in (("inner",inner),("center",(inner+outer)/2),("outer",outer)):
                output[lid][name].append(_piece(poly,primitives[gi],a,b))
    return {"schema":"mapforge/local-shape-math/v1","status":"AVAILABLE","road_id":road.get("id"),
            "domain":[lo,hi],"lane_section_s":sec_s,"partition":partition,
            "lanes":[{"lane_id":lid,"lane_type":lanes[lid].get("type"),
                      "curves":{name:_curve(pieces) for name,pieces in output[lid].items()}} for lid in requested],
            "scope":"planar actual-written float polynomial model; increasing road s; rate is d(kappa)/d(lane arclength)",
            "numerics":{"zero":"only identically zero constructed coefficients are straight; no geometric floor",
                        "roots":"exact rational Frenet algebra, gcd-proven squarefree reduction, Bernstein sign cover and derivative-recursive isolation; uncertain roots raise UNAVAILABLE",
                        "extrema":"rational interval evaluation over root brackets; square root enclosure verified by exact squares",
                        "max_extremum_uncertainty":1e-12,
                        "record_stations":"binary64 global starts (section s + local sOffset), no near-cut merging; rational midpoint selection",
                        "root_bracket_u_max_width":2.**-112,"formal_decimal_xml_certificate":False},
            "quality_assessed":False,"geometry_modified":False}


def analyze(root, road_id, domain, lane_ids=None, extra_cuts=()):
    """Select exactly one road from an XML root, then call analyze_road."""
    roads=[r for r in root.findall("road") if r.get("id")==str(road_id)]
    if len(roads) != 1:
        _fail("road identity missing or ambiguous")
    return analyze_road(roads[0],domain,lane_ids,extra_cuts)
