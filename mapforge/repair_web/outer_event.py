"""Source-owned long-cubic reconstruction of one lane birth/split envelope.

Unlike an additive C2 bump on a broken baseline, all affected boundaries are
solved together. Existing source IDs and lane sections are retained. Polynomial
re-expression at a semantic section cut introduces no extra shape freedom.
This module is deliberately separate from the frozen v1 repair session kernel.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from xml.etree import ElementTree as ET

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import LinearConstraint, minimize, linprog
from pyproj import Transformer

from mapforge.ops.arc_source_chart import ArcChart
from .model import coefficients, digest, intervals, lanes, parse, shifted, extrema


@dataclass(frozen=True)
class EventScope:
    road: str
    knots: tuple[float, ...]
    births: tuple[tuple[int, float], ...]
    source_tolerance_m: float = .75
    minimum_span_m: float = 6.


# The second measured widening is only ~9.7m: allocate two minimum-long
# transition spans after birth, not one cubic constrained at both ends and
# not a knot per source point. Reference geometry remains one 211m Line.
WEST_SPLIT = EventScope('11', (100., 115., 131.0543, 141.3316, 151.6089, 157.6089, 163.6089,
                             173.3754, 187.5, 201.7074107),
                       ((3, 131.0543), (4, 151.6089)))


def cubic_range(c, length):
    return extrema(c, length), -extrema(-np.asarray(c), length)


class OuterEvent:
    """A single shared boundary system in the fixed source reference chart.

    Current admission: a one-Line parent, right driving lanes numbered in
    radial order, explicit source occurrences, and zero-width births already
    approved by source-role decisions. No topology or road endpoints move.
    """
    def __init__(self, data, packet, scope=WEST_SPLIT, *, approved_roles):
        self.data, self.root, self.scope = data, parse(data), scope
        self.packet = packet  # read-only source for independent compiled-XML checks
        self.road = next(r for r in self.root.findall('road') if r.get('id') == scope.road)
        gs = self.road.findall('planView/geometry')
        if len(gs) != 1 or gs[0][0].tag != 'line':
            raise ValueError('Event admission requires one exact Line chart')
        if np.min(np.diff(scope.knots)) < scope.minimum_span_m:
            raise ValueError('Short independently fitted boundary spans forbidden')
        g = gs[0]
        self.axis = ArcChart((float(g.get('x')), float(g.get('y'))), float(g.get('hdg')), 0.)
        self.start, self.end = scope.knots[0], scope.knots[-1]
        self.births = dict(scope.births)
        self.count = max(self.births)
        self.splines, self.slices, self.nvar = {}, {}, 0
        for edge in range(self.count+1):
            lo = self.births.get(edge, self.start)
            knots = np.asarray([lo]*4 + [s for s in scope.knots if lo < s < self.end] + [self.end]*4)
            n = len(knots)-4
            self.splines[edge] = BSpline(knots, np.eye(n), 3, extrapolate=False)
            self.slices[edge] = slice(self.nvar, self.nvar+n)
            self.nvar += n
        self._validate_written_layout()
        for edge,s in self.births.items():
            sec=next(sec for sec,lo,hi in intervals(self.road) if abs(lo-s)<1e-8)
            lane=lanes(sec)[-edge]
            sid=next(e.get('value') for e in lane.findall('userData') if e.get('code')=='mapforge.source_lane')
            obs=packet['observations'][sid]
            if not obs['start_width_known'] or obs['start_width_mm']!=0 or (sid,'start') not in approved_roles:
                raise ValueError('Birth needs the existing exact zero-width source-role approval')
        self.traces, self.inventory, self.paths = self._source_traces(packet)
        self.E, self.e = self._equalities()
        # Well-scaled exact affine elimination, not equality penalty fitting.
        self.origin = np.linalg.lstsq(self.E, self.e, rcond=None)[0]
        if np.max(abs(self.E@self.origin-self.e)) > 1e-7:
            raise ValueError('Inconsistent event endpoint/birth constraints')
        self.Z = null_space(self.E)

    def _validate_written_layout(self):
        for sec, lo, hi in intervals(self.road):
            if hi <= self.start or lo >= self.end:
                continue
            if sec.findall('left/lane'):
                raise ValueError('Unbound opposite lanes would move with laneOffset')
            expected = set(range(1, 3+sum(lo >= s-1e-7 for s in self.births.values())))
            if {-lid for lid in lanes(sec)} != expected:
                raise ValueError('Written lane birth layout differs from registered event')
            if any(l.get('type') != 'driving' or l.findall('border') for l in lanes(sec).values()):
                raise ValueError('Unsupported lane semantics')

    def row(self, edge, s, derivative=0):
        # A newborn outer edge coincides with its inner predecessor before birth.
        while edge in self.births and s < self.births[edge]:
            edge -= 1
        r = np.zeros(self.nvar)
        r[self.slices[edge]] = self.splines[edge](s, nu=derivative)
        return r

    def power(self, edge, s):
        return np.array([self.row(edge, s, j)/math.factorial(j) for j in range(4)])

    def old_power(self, edge, s, left=False):
        from scripts.internal_edge_jets import active
        sec = active(self.road.findall('lanes/laneSection'), s, left, lambda e: float(e.get('s')))
        def shift(items, attr, station):
            if not items:
                return np.zeros(4)
            item = active(items, station, left, lambda e: float(e.get(attr)))
            p = np.polynomial.Polynomial(coefficients(item))
            c = p(np.polynomial.Polynomial([station-float(item.get(attr)), 1])).coef
            return np.pad(c, (0, 4-len(c)))
        c = shift(self.road.findall('lanes/laneOffset'), 's', s)
        for lid in range(1, edge+1):
            if -lid not in lanes(sec):
                continue
            c -= shift(lanes(sec)[-lid].findall('width'), 'sOffset', s-float(sec.get('s')))
        return c

    def _equalities(self):
        rows, rhs = [], []
        for edge in self.splines:
            lo = self.births.get(edge, self.start)
            for s in (lo, self.end):
                for d in range(3):
                    # Scale derivative equations into metre units for conditioning.
                    scale = 20.**d
                    r = self.row(edge, s, d)
                    target = self.old_power(edge, s, left=(s == self.end))[d]*math.factorial(d)
                    if s == lo and edge in self.births:
                        r -= self.row(edge-1, s, d)
                        target = 0.
                    rows.append(r*scale); rhs.append(target*scale)
        return np.asarray(rows), np.asarray(rhs)

    def _source_traces(self, packet):
        occurrences = [o for o in packet['occurrences'] if o['road'] == self.scope.road and o['scope_role'] == 'ordinary']
        owners = {}
        for o in occurrences:
            sid, lid = o['source_lane_id'], o['lane']
            if lid >= 0 or (sid in owners and owners[sid] != -lid):
                raise ValueError('Source identity has ambiguous target ownership')
            owners[sid] = -lid
        ref = self.root.findtext('header/geoReference')
        offset = self.root.find('header/offset')
        if not ref or offset is not None and any(float(offset.get(k, '0')) != 0 for k in ('x','y','z','hdg')):
            raise ValueError('Unregistered coordinate frame')
        proj = Transformer.from_crs(4326, ref, always_xy=True)
        def st(part):
            return self.axis.project(np.asarray([proj.transform(float(p[0]),float(p[1])) for p in part]))
        traces, inventory, paths, seen = [], [], [], {}
        for sid, lid in sorted(owners.items()):
            obs = packet['observations'][sid]
            paths.append({'source_lane_id': sid, 'points': obs['lane_path'],
                          'role': 'unchanged movement observation; not replaced with boundary midpoint'})
            # This IBD event's SIDE=left is its lower-t (outer) boundary.
            # Bind using original relations and occurrences; independently check
            # this ordering instead of selecting a nearby line or changing TOPO.
            pair = {}
            for rel in obs['boundary_relations']:
                key, side = rel['boundary_key'], rel['declared_side']
                edge = lid if side == 'left' else lid-1 if side == 'right' else None
                if edge is None:
                    raise ValueError('Unknown original side')
                if key in seen and seen[key] != edge:
                    raise ValueError('One original boundary mapped to two target edges')
                pair[side] = []
                for ri, record in enumerate(packet['boundaries'][key]['records']):
                    for pi, part in enumerate(record['parts']):
                        xy = st(part)
                        if len(xy) < 2:
                            raise ValueError('Degenerate original boundary')
                        if np.all(np.diff(xy[:,0]) < 0):
                            xy = xy[::-1]
                        if np.any(np.diff(xy[:,0]) <= 0):
                            raise ValueError('Nonmonotone source requires a different chart, not point sorting')
                        pair[side].append(xy)
                        if key in seen:
                            continue
                        traces.append(dict(key=key, record=ri, part=pi, edge=edge, st=xy))
                        inventory.append(dict(key=key, edge=edge, record=ri, part=pi, vertices=len(xy),
                                              source_s=[float(xy[0,0]),float(xy[-1,0])],
                                              edited_s=[max(self.start,float(xy[0,0])),min(self.end,float(xy[-1,0]))]))
                seen[key] = edge
            if set(pair) != {'left', 'right'}:
                raise ValueError('Both explicit original boundaries required')
            for l in pair['left']:
                for r in pair['right']:
                    a, b = max(l[0,0],r[0,0]), min(l[-1,0],r[-1,0])
                    if b > a:
                        ss = np.linspace(a,b,31)
                        if np.min(np.interp(ss,r[:,0],r[:,1])-np.interp(ss,l[:,0],l[:,1])) < -1e-5:
                            raise ValueError('Declared IBD boundary order not valid for this event')
        return traces, inventory, paths

    def source_cells(self):
        for trace in self.traces:
            for a,b in zip(trace['st'][:-1], trace['st'][1:]):
                lo, hi = max(self.start,a[0]), min(self.end,b[0])
                if hi <= lo:
                    continue
                cuts = [lo]+[s for s in self.scope.knots if lo<s<hi]+[hi]
                slope = (b[1]-a[1])/(b[0]-a[0])
                for l,h in zip(cuts,cuts[1:]):
                    yield trace, l,h, np.array([a[1]+(l-a[0])*slope,slope,0.,0.])

    def source_error(self, x):
        rows = []
        for trace,lo,hi,c in self.source_cells():
            # Exact extremum of cubic minus the original straight segment.
            delta=self.power(trace['edge'],lo)@x-c
            roots=np.polynomial.polynomial.polyroots([delta[1],2*delta[2],3*delta[3]])
            query=[0.,hi-lo]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<hi-lo]
            at=max(query,key=lambda u:abs(np.polynomial.polynomial.polyval(u,delta)))
            rows.append(dict(key=trace['key'],edge=trace['edge'],s=[lo,hi],
                             max_m=float(abs(np.polynomial.polynomial.polyval(at,delta))),
                             witness_s=float(lo+at),source_t=float(np.polynomial.polynomial.polyval(at,c))))
        return {'max_m':max(r['max_m'] for r in rows), 'rows':rows,
                'scope':'full original segments intersecting the explicit edit support; outside retained, not certified'}

    def linear_model(self, displacement=0.):
        """One source/width/intent system reused by geometric and dynamic solves."""
        if isinstance(displacement,bool) or not isinstance(displacement, (int,float)) or not math.isfinite(displacement) or abs(displacement)>2:
            raise ValueError('Finite handle displacement within +/-2m required')
        A, y = [], []
        C, low, high = [], [], []
        for trace,lo,hi,c in self.source_cells():
            for s in np.linspace(lo,hi,max(3,math.ceil((hi-lo)/2)+1)):
                row = self.row(trace['edge'],s)
                value = np.polynomial.polynomial.polyval(s-lo,c)
                A.append(row); y.append(value)
                C.append(row); low.append(value-self.scope.source_tolerance_m); high.append(value+self.scope.source_tolerance_m)
        source_row_count=len(C)
        for edge,bs in self.splines.items():
            cuts = sorted(set(bs.t))
            for lo,hi in zip(cuts,cuts[1:]):
                for s in np.linspace(lo,hi,5):
                    A.append(self.row(edge,s,2)*30.); y.append(0.)
                if edge == 0:
                    continue
                # Bernstein nonnegative widths certify each WHOLE cubic span.
                p = self.power(edge-1,lo)-self.power(edge,lo)
                L = hi-lo
                bernstein = np.array([[1,0,0,0],[1,L/3,0,0],[1,2*L/3,L*L/3,0],[1,L,L*L,L**3]])@p
                C.extend(bernstein); low.extend([0.]*4); high.extend([np.inf]*4)
        # One user intent influences the entire coupled event, not a new knot.
        s = 163.
        handle = self.row(self.count,s)
        target = self.old_power(self.count,s)[0]+displacement
        A.append(handle*10.); y.append(target*10.)
        A,y,C,low,high=map(np.asarray,(A,y,C,low,high))
        return A,y,C,low,high,source_row_count,handle,s

    def solve(self, displacement=0.):
        A,y,C,low,high,source_row_count,handle,s=self.linear_model(displacement)
        Z,x0=self.Z,self.origin
        B=A@Z; target=y-A@x0
        hessian=B.T@B+np.eye(Z.shape[1])*1e-10
        linear=-B.T@target
        D=C@Z; lb=low-C@x0; ub=high-C@x0
        norms=np.linalg.norm(D,axis=1)
        live=norms>1e-9
        if np.any(lb[~live]>1e-7) or np.any(ub[~live]<-1e-7):
            raise ValueError('Fixed event constraints contradict source or width bounds')
        D,lb,ub=D[live]/norms[live,None],lb[live]/norms[live],ub[live]/norms[live]
        constraints=LinearConstraint(D,lb,ub)
        U=np.vstack([-D[np.isfinite(lb)],D[np.isfinite(ub)]])
        v=np.r_[-lb[np.isfinite(lb)],ub[np.isfinite(ub)]]
        feasible=linprog(np.zeros(Z.shape[1]),A_ub=U,b_ub=v,bounds=[(None,None)]*Z.shape[1],method='highs')
        diagnosis=None
        if not feasible.success:
            # Diagnostic only: minimum EXTRA source allowance while retaining
            # endpoint/birth and width constraints. Never change the actual gate.
            elastic=np.r_[np.ones(source_row_count),np.zeros(len(C)-source_row_count)][live]/norms[live]
            relax=np.r_[elastic[np.isfinite(lb)],elastic[np.isfinite(ub)]]
            diag=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=np.c_[U,-relax],b_ub=v,
                         bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs')
            diagnosis={'status':str(diag.message),'required_extra_source_m':float(diag.x[-1]) if diag.success else None}
        z0=feasible.x if feasible.success else np.zeros(Z.shape[1])
        fit=minimize(lambda z:.5*z@hessian@z+linear@z,z0,
                     jac=lambda z:hessian@z+linear,method='SLSQP',constraints=constraints,
                     options={'maxiter':150,'ftol':1e-9})
        x=x0+Z@fit.x
        exchange_rounds=0
        # Enforce the continuum at actual polynomial extrema. Extra constraint
        # witnesses are NOT extra curve knots or short exported segments.
        for exchange_rounds in range(9):
            violations=[r for r in self.source_error(x)['rows'] if r['max_m']>self.scope.source_tolerance_m+1e-8]
            if not fit.success or not violations or exchange_rounds==8:
                break
            for r in violations:
                row=self.row(r['edge'],r['witness_s'])
                v=row@Z; norm=np.linalg.norm(v)
                if norm<1e-9:
                    raise ValueError('Locked source endpoint exceeds original envelope')
                D=np.vstack([D,v/norm])
                lb=np.r_[lb,(r['source_t']-self.scope.source_tolerance_m-row@x0)/norm]
                ub=np.r_[ub,(r['source_t']+self.scope.source_tolerance_m-row@x0)/norm]
            fit=minimize(lambda z:.5*z@hessian@z+linear@z,fit.x,
                         jac=lambda z:hessian@z+linear,method='SLSQP',
                         constraints=LinearConstraint(D,lb,ub),options={'maxiter':150,'ftol':1e-9})
            x=x0+Z@fit.x
        violation=float(max(0.,np.max(low-C@x),np.max(C@x-high)))
        source=self.source_error(x)
        report=dict(success=bool(fit.success),message=str(fit.message),iterations=int(fit.nit),
                    linear_feasibility=str(feasible.message),
                    infeasibility_diagnosis=diagnosis,
                    exact_extremum_constraint_rounds=exchange_rounds,
                    nvar=self.nvar,free_variables=Z.shape[1],inequality_violation=violation,
                    equality_residual=float(np.max(abs(self.E@x-self.e))),source=source,
                    handle_s=s,handle_requested_m=displacement,
                    handle_achieved_m=float(handle@x-self.old_power(self.count,s)[0]))
        report['accepted']=bool(fit.success and violation<=1e-7 and source['max_m']<=self.scope.source_tolerance_m+1e-7)
        report.update(status='GEOMETRIC_EVENT_CANDIDATE_NOT_MAP',map_accepted=False,
                      dynamics='NOT_VALIDATED',export_allowed=False)
        return x,report

    def compile(self,x):
        """Write boundary DIFFERENCES, sharing one C2 geometry for adjacent lanes."""
        x=np.asarray(x,float)
        if x.shape!=(self.nvar,) or not np.isfinite(x).all():
            raise ValueError('Invalid event state')
        if np.max(abs(self.E@x-self.e))>1e-7 or self.source_error(x)['max_m']>self.scope.source_tolerance_m+1e-7:
            raise ValueError('Event violates endpoints/births/source envelope')
        root=copy.deepcopy(self.root)
        road=next(r for r in root.findall('road') if r.get('id')==self.scope.road)
        def rewrite(parent,tag,attr,lo,hi,polynomial):
            old=parent.findall(tag)
            # Keep everything outside scope byte-semantically unchanged.
            fragments=[]
            for el in old:
                s=lo+float(el.get(attr))
                end=min([lo+float(e.get(attr)) for e in old if lo+float(e.get(attr))>s]+[hi])
                if end<=self.start or s>=self.end:
                    continue
                for a,b in ((s,min(end,self.start)),(max(s,self.end),end)):
                    if b>a:
                        fragments.append((a,shifted(old,attr,a-lo)))
                parent.remove(el)
            # Only genuine long knots plus mandatory section boundaries, NOT
            # old 10m sample breaks, are written inside this interval.
            cuts=sorted({max(lo,self.start),min(hi,self.end)}|{s for s in self.scope.knots if lo<s<hi})
            if hi<=self.start or lo>=self.end:
                return
            for s,end in zip(cuts,cuts[1:]):
                c=polynomial(s)
                if tag=='width' and extrema(c,end-s)<-1e-7:
                    raise ValueError('Negative interval width')
                ET.SubElement(parent,tag,**{attr:format(s-lo,'.17g'),**{k:format(float(v),'.17g') for k,v in zip('abcd',c)}})
            # Restore exact untouched subintervals of records straddling scope.
            for s,c in fragments:
                if not any(abs(lo+float(e.get(attr))-s)<1e-8 for e in parent.findall(tag)):
                    ET.SubElement(parent,tag,**{attr:format(s-lo,'.17g'),**{k:format(float(v),'.17g') for k,v in zip('abcd',c)}})
            items=sorted(parent.findall(tag),key=lambda e:float(e.get(attr)))
            for e in items:parent.remove(e)
            # OpenDRIVE element ordering: widths before marks/speeds/userData,
            # laneOffset before laneSection, after lane link if present.
            index=1 if tag=='width' and parent.find('link') is not None else 0
            for e in items:parent.insert(index,e);index+=1
        rewrite(road.find('lanes'),'laneOffset','s',0.,float(road.get('length')),lambda s:self.power(0,s)@x)
        for sec,lo,hi in intervals(road):
            for lid,lane in lanes(sec).items():
                edge=-lid
                rewrite(lane,'width','sOffset',lo,hi,lambda s,e=edge:(self.power(e-1,s)-self.power(e,s))@x)
        return ET.tostring(root,encoding='utf-8',xml_declaration=True)
