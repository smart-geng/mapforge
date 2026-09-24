"""Continuous numerical source bounds on explicitly owned long curve domains.

Original join cross-sections, not nearest-point cropping, partition each side.
Ambiguous/repeated/tangent crossings are unresolved, never silently selected.
Adaptive intervals are evaluation only, not output OpenDRIVE segments. Bounds
use floating-point polynomial extrema, not outward-rounded formal arithmetic.
"""
from dataclasses import dataclass
import heapq
import math

import numpy as np
from numpy.polynomial import Polynomial as P

from mapforge.repair_web.coupled_event_sources import ribbon_intervals, polynomial_absmax
from mapforge.repair_web.event_shape_admission import point_to_original


@dataclass
class CurvePiece:
    start: float
    end: float
    ref: object
    offset: float
    co: np.ndarray

    def jet(self,s):
        u=s-self.start; t=P(self.co); q=self.offset+u
        h=self.ref.Theta(q); k=self.ref.KappaStart+self.ref.dk*q
        tangent=np.array([math.cos(h),math.sin(h)]); normal=np.array([-tangent[1],tangent[0]])
        xy=np.array([self.ref.X(q),self.ref.Y(q)])+t(u)*normal
        first=(1-k*t(u))*tangent+t.deriv()(u)*normal
        return xy,first

    def bounds(self,a,b):
        # Rebase ALL polynomials to this subinterval before finding extrema.
        t=P(self.co)(P([a-self.start,1.])); k=P([self.ref.KappaStart+self.ref.dk*(self.offset+a-self.start),self.ref.dk])
        dt=t.deriv(); A=1-k*t; L=b-a
        speed=math.hypot(polynomial_absmax(A.coef,L),polynomial_absmax(dt.coef,L))
        acceleration=math.hypot(polynomial_absmax((-self.ref.dk*t-2*k*dt).coef,L),
                                polynomial_absmax((k*A+t.deriv(2)).coef,L))
        return speed,acceleration


class LongEdge:
    def __init__(self,turn,side):
        if side not in ('left','right'): raise ValueError('Physical side required')
        knots=np.asarray(turn['stations'],float); co=np.asarray(turn['coefficients'][side],float)
        if (knots.ndim!=1 or len(knots)<2 or not np.isfinite(knots).all() or knots[0]!=0
                or np.any(np.diff(knots)<=0) or co.shape!=(len(knots)-1,4) or not np.isfinite(co).all()
                or any(not math.isfinite(r.length) or r.length<=0 for r in turn['refs'])
                or abs(knots[-1]-sum(r.length for r in turn['refs']))>1e-8):
            raise ValueError('Complete finite reference/transverse interval domain required')
        self.pieces=[CurvePiece(a,b,ref,offset,co[side]) for a,b,ref,offset,co in ribbon_intervals(turn)]
        if not self.pieces: raise ValueError('Complete nonempty curve required')
        self.length=self.pieces[-1].end
        for a,b in zip(self.pieces,self.pieces[1:]):
            if abs(a.end-b.start)>1e-8 or np.linalg.norm(a.jet(a.end)[0]-b.jet(b.start)[0])>1e-7:
                raise ValueError('Discontinuous curve: do not insert a hidden chord')

    def cells(self,lo=0.,hi=None):
        hi=self.length if hi is None else hi
        if not 0<=lo<hi<=self.length+1e-8: raise ValueError('Explicit nonempty interval required')
        for p in self.pieces:
            a=max(lo,p.start); b=min(hi,p.end)
            if b>a: yield p,a,b

    def intersections(self,point,normal,*,budget=12000,station_tolerance=1e-8):
        """Isolate all plane crossings; derivative enclosure excludes tangency."""
        point=np.asarray(point,float); normal=np.asarray(normal,float)
        if point.shape!=(2,) or normal.shape!=(2,) or not np.isfinite([*point,*normal]).all() or np.linalg.norm(normal)<1e-12:
            raise ValueError('Finite nonzero original join plane required')
        normal=normal/np.linalg.norm(normal); pending=list(self.cells()); roots=[]; unresolved=[]; count=0
        while pending and count<budget:
            p,a,b=pending.pop(); count+=1; m=(a+b)/2; radius=(b-a)/2
            xy,d1=p.jet(m); fm=float(normal@(xy-point)); derivative=float(normal@d1)
            speed,acc=p.bounds(a,b)
            if abs(fm)>speed*radius+1e-10: continue
            fa=float(normal@(p.jet(a)[0]-point)); fb=float(normal@(p.jet(b)[0]-point))
            sign=1 if derivative-acc*radius>1e-10 else -1 if derivative+acc*radius< -1e-10 else 0
            if sign:
                if fa*fb>0: continue
                left,right=a,b
                if fa==0.: right=left
                elif fb==0.: left=right
                else:
                    while right-left>station_tolerance:
                        mid=(left+right)/2; f=float(normal@(p.jet(mid)[0]-point))
                        if f*fa>0: left=mid
                        else: right=mid
                roots.append(dict(station_m=(left+right)/2,bracket_m=[left,right],direction=sign))
            elif b-a<=station_tolerance:
                unresolved.append([a,b])
            else:
                pending.extend(((p,a,m),(p,m,b)))
        unresolved.extend([a,b] for _,a,b in pending)
        unique=[]
        for r in sorted(roots,key=lambda r:r['station_m']):
            if unique and abs(r['station_m']-unique[-1]['station_m'])<2*station_tolerance:
                if r['direction']!=unique[-1]['direction']: unresolved.append(r['bracket_m'])
            else: unique.append(r)
        return dict(roots=unique,unresolved_intervals=unresolved,evaluations=count,
                    numeric_only=True,nearest_point_selection=False)

    def polyline(self,lo,hi,epsilon=2e-5):
        points=[]; worst=0.
        for p,a,b in self.cells(lo,hi):
            _,acc=p.bounds(a,b); n=max(1,math.ceil((b-a)/min(.25,math.sqrt(8*epsilon/acc) if acc else .25)))
            if n>100000: raise ValueError('Evaluation mesh budget exceeded')
            s=np.linspace(a,b,n+1); points.extend(p.jet(v)[0] for v in s)
            worst=max(worst,acc*((b-a)/n)**2/8)
        mesh=np.asarray(points)
        # Evaluation-only duplicate endpoints, not original data vertices.
        mesh=mesh[np.r_[True,np.linalg.norm(np.diff(mesh,axis=0),axis=1)>0.]]
        return mesh,worst+1e-7  # allowed numerical join discrepancy


def join_plane(row):
    raw=row['oriented_full_xy']; delta=np.diff(raw,axis=0)
    delta=delta[np.linalg.norm(delta,axis=1)>1e-10]
    if not len(delta): raise ValueError('No original ordinary tangent')
    after=row['role']=='successor'
    return raw[0 if after else -1],delta[0 if after else -1]/np.linalg.norm(delta[0 if after else -1])


def owned_domains(edge,rows):
    """Travel-ordered join planes bind roles; never choose one of many roots."""
    cuts=[]; witnesses=[]
    for role in ('predecessor','successor'):
        row=next(r for r in rows if r['role']==role)
        point,normal=join_plane(row); result=edge.intersections(point,normal)
        witnesses.append(dict(role=role,feature=row['feature'],point=point.tolist(),normal=normal.tolist(),**result))
        if result['unresolved_intervals'] or len(result['roots'])!=1 or result['roots'][0]['direction']!=1:
            return dict(status='UNRESOLVED_OWNERSHIP',witnesses=witnesses,domains={},reason='join plane crossing not uniquely forward')
        cuts.append(result['roots'][0]['station_m'])
    if not 0<cuts[0]<cuts[1]<edge.length:
        return dict(status='UNRESOLVED_OWNERSHIP',witnesses=witnesses,domains={},reason='ordered positive-length roles not established')
    return dict(status='NUMERIC_UNIQUE_JOIN_PLANES',witnesses=witnesses,
        domains=dict(predecessor=[0.,cuts[0]],via=cuts,successor=[cuts[1],edge.length]),
        formal_certificate=False,source_ownership_planes_changed=False)


def maximum_enclosure(cells,evaluate,*,tolerance=.001,budget=5000,error=0.):
    """1-Lipschitz distance bound over parametric cells with speed bound.

    cells are (token,a,b); evaluate -> (midpoint distance, speed upper bound).
    Bounds include every interval, even if the subdivision budget is exhausted.
    """
    if not 0<tolerance<.1 or budget<1: raise ValueError('Positive bounded numerical budget required')
    if len(cells)>budget: raise ValueError('Budget cannot cover the initial complete domain')
    heap=[]; lower=0.; evaluations=0; serial=0; witness=None
    def push(token,a,b):
        nonlocal lower,evaluations,serial,witness
        value,speed=evaluate(token,a,b); evaluations+=1
        if not np.isfinite([value,speed]).all() or value<0 or speed<0: raise ValueError('Finite distance and bound required')
        lo=max(0.,value-error); upper=value+speed*(b-a)/2+error
        if lo>=lower: lower=lo; witness=dict(station=(a+b)/2,distance_lower_m=lo)
        serial+=1; heapq.heappush(heap,(-upper,serial,token,a,b))
    for token,a,b in cells: push(token,a,b)
    if not heap: raise ValueError('No intervals: do not report zero error')
    while -heap[0][0]-lower>tolerance and evaluations+2<=budget:
        _,_,token,a,b=heapq.heappop(heap); m=(a+b)/2
        push(token,a,m); push(token,m,b)
    upper=max(lower,-heap[0][0])
    return dict(lower_m=lower,upper_m=upper,gap_m=upper-lower,evaluations=evaluations,
        resolved=upper-lower<=tolerance,witness=witness,numeric_not_formal=True)


def target_to_source(edge,domain,raw,**kw):
    def evaluate(p,a,b):
        point=p.jet((a+b)/2)[0]
        return point_to_original(point,raw)['distance_m'],p.bounds(a,b)[0]
    return maximum_enclosure(list(edge.cells(*domain)),evaluate,**kw)


def source_to_target(edge,domain,fragments,**kw):
    polyline,error=edge.polyline(*domain); cells=[]
    for raw in fragments:
        for a,b in zip(raw,raw[1:]):
            if np.linalg.norm(b-a)>1e-12: cells.append(((a,b),0.,1.))
    def evaluate(pair,a,b):
        p,q=pair; point=p+(q-p)*(a+b)/2
        return point_to_original(point,polyline)['distance_m'],float(np.linalg.norm(q-p))
    result=maximum_enclosure(cells,evaluate,error=error,**kw)
    result['target_chord_bound_m']=error
    return result


def check_turn_domains(turn,rows,*,tolerance=.001,budget=5000):
    result={}
    for side in ('left','right'):
        support=[r for r in rows if r['field']==side]; edge=LongEdge(turn,side)
        ownership=owned_domains(edge,support); checks=[]
        if ownership['domains']:
            for row in support:
                domain=ownership['domains'][row['role']]; limit=.35 if row['role']!='via' else 1.5
                forward=target_to_source(edge,domain,row['oriented_full_xy'],tolerance=tolerance,budget=budget)
                reverse=source_to_target(edge,domain,row['owned_fragments'],tolerance=tolerance,budget=budget) if row['owned_fragments'] else None
                measured=[forward]+([reverse] if reverse is not None else [])
                # Root brackets contribute positional uncertainty, not a
                # silently exact cut. This covers subdomain endpoint motion.
                speed=max(p.bounds(a,b)[0] for p,a,b in edge.cells(*domain))
                margin=2e-8*speed+1e-9
                for bounds in measured:
                    bounds['lower_m']=max(0.,bounds['lower_m']-margin); bounds['upper_m']+=margin
                    bounds['gap_m']=bounds['upper_m']-bounds['lower_m']
                    bounds['resolved']=bounds['gap_m']<=tolerance
                    bounds['join_location_margin_m']=margin
                bad=any(r['lower_m']>limit for r in measured)
                good=all(r['upper_m']<=limit for r in measured)
                checks.append(dict(role=row['role'],feature=row['feature'],domain_m=domain,limit_m=limit,
                    target_to_original=forward,owned_original_to_target=reverse,
                    status='FAIL_BOUND' if bad else 'WITHIN_MAX_BOUND' if good else 'UNRESOLVED_BOUND',
                    empty_owned_source_status=None if reverse is not None else 'EMPTY_ORIGINAL_DOMAIN_NO_REVERSE_VALUE_FULL_PART_RETAINED_IN_ORDINARY_CONTEXT',
                    p95_median_and_movement_accepted=False))
        result[side]=dict(ownership=ownership,checks=checks)
    return dict(status='CONTINUOUS_MAX_DIAGNOSTIC_NOT_MAP_ACCEPTANCE',sides=result,
        original_vertices_removed=0,output_geometry_added=0,map_accepted=False,
        role_assignment='original ordinary-to-via join planes; no nearest-tip crop')
