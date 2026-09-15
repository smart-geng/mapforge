"""Whole-road experiment: one laneOffset and all physical boundaries in one QP.

It preserves existing planView/topology/speeds and junction end states. The
reference-frame fairing target includes reference curvature rate: minimizing
t''' alone is not the same as minimizing world-space road curvature variation.
Unsuccessful candidates never mutate the input road.
"""
import argparse
import copy
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from lxml import etree
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import linprog,OptimizeResult
from scipy.sparse import csc_matrix
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source
from mapforge.ops.lane_family_border import _linked_chains, _materialize_widths
from mapforge.validate.shp_boundary_fidelity import _project, _origin, _densify
from mapforge.validate.smoothness import sample_road_ref, lane_edges_kinematics_at, _ref_kappa_at
from mapforge.validate.g11 import audit_file


@dataclass
class Family:
    key: str
    side: str
    chain: list
    knots: np.ndarray
    basis: BSpline
    columns: slice


def world_kinematics(jets, curvature, sharpness):
    """Exact planar curvature and d(curvature)/d(lane arc length).

    ``jets`` has t,t',t'',t''' in the last dimension. Within a line/arc/spiral
    reference primitive k''=0. Both sides of every reference join are queried
    separately. Complex arithmetic is retained for complex-step Jacobians.
    """
    t, d1, d2, d3 = np.moveaxis(np.asarray(jets), -1, 0)
    k, kp = np.asarray(curvature), np.asarray(sharpness)
    a, b = 1-k*t, d1
    ap, bp = -kp*t-k*d1, d2
    c, d = -kp*t-2*k*d1, k*a+d2
    cp, dp = -3*kp*d1-2*k*d2, kp*a+k*ap+d3
    q = a*a+b*b
    n = a*d-b*c
    np_ = ap*d+a*dp-bp*c-b*cp
    qp = 2*(a*ap+b*bp)
    return np.stack([n/q**1.5, (np_*q-1.5*n*qp)/q**3], axis=-1)


def ordered_source_boundary(candidates, *, select_high):
    """Select physical side from raw transverse ordering, never old target error.

    Both source curves must overlap and keep a consistent order. The source
    list order and DBF left/right enumeration are not assumed to be reliable.
    """
    if len(candidates) != 2:
        raise ValueError('requires two overlapping source boundaries')
    a, b = candidates
    lo, hi = max(a[0,0],b[0,0]), min(a[-1,0],b[-1,0])
    if hi-lo < 1e-4:
        raise ValueError('source boundary domains do not overlap')
    s = np.unique(np.r_[lo, hi, a[(a[:,0]>lo)&(a[:,0]<hi),0], b[(b[:,0]>lo)&(b[:,0]<hi),0]])
    difference = np.interp(s,a[:,0],a[:,1])-np.interp(s,b[:,0],b[:,1])
    if np.min(difference) < -1e-3 and np.max(difference) > 1e-3:
        raise ValueError('source physical boundaries cross')
    high_is_a = np.median(difference) >= 0.
    return a if high_is_a == select_high else b


def phase_certificate(phase, labels):
    """LP dual support, not every constraint tied at the relaxed maximum.

    This only diagnoses the chosen finite-dimensional linear model. It is not
    a global nonlinear infeasibility certificate or permission to relax data.
    """
    if not phase.success or phase.fun <= 1e-7:
        return []
    weights = -np.asarray(phase.ineqlin.marginals)
    return [dict(labels[i], dual_weight=float(weights[i]))
            for i in np.argsort(-weights) if weights[i] > 1e-7]


def linear_feasible(matrix,lower):
    """A fully fixed family has no LP variables but still has hard bounds."""
    if matrix.shape[1]==0:
        return OptimizeResult(success=bool(np.all(lower<=1e-7)),x=np.zeros(0),
                              message='fixed family checked without LP')
    return linprog(np.zeros(matrix.shape[1]),A_ub=-matrix,b_ub=-lower,
                   bounds=[(None,None)]*matrix.shape[1],method='highs')


def _convex_qp(H, g, E, er, C, lo, initial):
    """Solve the convex subproblem; independently check original residuals."""
    import osqp  # Optional spike dependency, Apache-2.0; not a runtime default.
    # Eliminate equality constraints before ADMM. At a zero-width tip the
    # Bernstein inequalities contain rows identical to endpoint equalities;
    # leaving both in the full problem can stall convergence near the tip.
    base=np.linalg.lstsq(E,er,rcond=None)[0] if len(E) else np.zeros(len(g))
    Z=null_space(E) if len(E) else np.eye(len(g))
    reduced=Z.T@H@Z
    column_scale=1/np.sqrt(np.maximum(np.diag(reduced),1e-10))
    Z=Z*column_scale
    HH=Z.T@H@Z; gg=Z.T@(g-H@base)
    DD=C@Z; ll=lo-C@base
    active=np.linalg.norm(DD,axis=1)>1e-10
    if np.any(ll[~active]>1e-6): return None,{'status':'equality-inequality conflict'}
    if not Z.shape[1]:
        return (base,{'status':'equality-fixed'}) if np.max(ll)<=1e-6 else (None,{'status':'fixed conflict'})
    scale=max(np.linalg.norm(HH,2),1.)
    qp=osqp.OSQP()
    qp.setup(P=csc_matrix((HH+HH.T)/(2*scale)),q=-gg/scale,
             A=csc_matrix(DD[active]),l=ll[active],
             u=np.full(int(active.sum()),np.inf),verbose=False,
             eps_abs=1e-8,eps_rel=1e-9,max_iter=200000,polishing=True,
             time_limit=45.)
    qp.warm_start(x=np.linalg.lstsq(Z,initial-base,rcond=None)[0])
    answer=qp.solve(raise_error=False)
    info={'status':answer.info.status,'iterations':answer.info.iter,
          'primal_residual':answer.info.prim_res,'dual_residual':answer.info.dual_res}
    if answer.info.status_val!=1 or answer.x is None: return None,info
    x=base+Z@answer.x
    info['original_inequality_residual']=float(max(0.,np.max(lo-C@x)))
    info['original_equality_residual']=float(np.max(abs(E@x-er))) if len(E) else 0.
    if max(info['original_inequality_residual'],info['original_equality_residual'])>1e-6:
        return None,info
    return x,info


def solve_road(road, src, project, min_span=15., source_tol=.35,
               smooth_weight=1e7, dynamics=True, knot_phase=None,
               source_mode='boundaries', mirror=False, junction_endpoint_mode='preserve',
               restore_dynamics=False, boundary_association='nearest-target',
               physical_graph=False, source_error_budget='legacy-envelope',all_source_vertices=False,
               model_only=False, source_certificate=False,source_widths=None):
    if junction_endpoint_mode not in ('preserve', 'parallel', 'source-parallel', 'free'):
        return {'status':'REJECTED','reason':'unknown junction endpoint construction mode'}
    if junction_endpoint_mode == 'free' and not model_only:
        return {'status':'REJECTED','reason':'free ports require a dependency-closed joint model, not independent writing'}
    if source_mode not in ('boundaries', 'centers'):
        return {'status':'REJECTED','reason':'unknown source geometry semantics'}
    if source_widths is not None and (source_mode!='centers' or not source_certificate):
        return {'status':'REJECTED','reason':'explicit MAP width requires complete center support certificate'}
    if boundary_association not in ('nearest-target', 'source-order'):
        return {'status':'REJECTED','reason':'unknown source boundary association'}
    if source_error_budget not in ('legacy-envelope','absolute'):
        return {'status':'REJECTED','reason':'unknown source error budget'}
    if source_certificate and (source_mode!='centers' or source_error_budget!='absolute'
                               or not all_source_vertices):
        return {'status':'REJECTED','reason':'center certificate requires absolute budget and all source vertices'}
    if source_certificate and (len(road.findall('planView/geometry'))!=1
                              or road.find('planView/geometry/line') is None):
        return {'status':'REJECTED','reason':'exact source certificate currently requires a single Line chart'}
    if not all(np.isfinite(v) and v>0 for v in (min_span,source_tol,smooth_weight)):
        return {'status':'REJECTED','reason':'invalid positive finite fitting parameter'}
    original=copy.deepcopy(road)
    sections=original.findall('lanes/laneSection')
    length=float(original.get('length'))
    starts=np.array([float(s.get('s')) for s in sections]); ends=np.r_[starts[1:],length]
    ids={(side,si):[l.get('id') for l in sorted(sec.findall(f'{side}/lane'),
                          key=lambda l:abs(int(l.get('id'))))]
         for side in ('left','right') for si,sec in enumerate(sections)}
    occurrences={}; owner={}; families=[]; nvar=0; endpoint_ties={}; graph_events=[]
    def family(name,side,chain,a,b):
        nonlocal nvar
        if b-a<min_span-1e-7:
            raise ValueError(f'short physical boundary {name}: {b-a:.6g}m')
        controls=np.linspace(a,b,max(1,int((b-a)/min_span))+1)
        if knot_phase is not None:
            if not 0<=knot_phase<min_span: raise ValueError('invalid knot phase')
            controls=np.r_[a,np.arange(a+min_span+knot_phase,b-min_span+1e-8,min_span),b]
        knots=np.r_[[a]*4,controls[1:-1],[b]*4]
        count=len(knots)-4
        f=Family(name,side,chain,knots,BSpline(knots,np.eye(count),3),slice(nvar,nvar+count))
        families.append(f); nvar+=count
        return len(families)-1
    try:
        family('laneOffset','right',[(si,'0') for si in range(len(sections))],0.,length)
        for side in ('right','left'):
            if physical_graph:
                if source_mode!='boundaries':
                    raise ValueError('physical graph requires original SHP endpoint evidence')
                from spikes.physical_boundary_graph import build_graph,source_tip_evidence
                def evidence(si,lid,role):
                    lane=sections[si].find(f"{side}/lane[@id='{lid}']")
                    ud=lane.find("userData[@code='mapforge.source_lane']")
                    rec=src.lane(ud.get('value')) if ud is not None else None
                    return source_tip_evidence(lane,rec,role)
                occ,_succ,chains,ties,events=build_graph(original,side,evidence)
                endpoint_ties.update({(side,*key):value for key,value in ties.items()})
                graph_events.extend(dict(e,side=side) for e in events)
            else:
                occ,_succ,chains=_linked_chains(sections,side)
            for key,lane in occ.items(): occurrences[(side,*key)]=lane
            for ci,chain in enumerate(chains):
                fi=family(f'{side}-{ci}',side,chain,starts[chain[0][0]],ends[chain[-1][0]])
                for key in chain: owner[(side,*key)]=fi
    except ValueError as exc:
        return {'status':'REJECTED','reason':str(exc)}

    def expr(fi,s,der=0):
        row=np.zeros(nvar); f=families[fi]
        row[f.columns]=f.basis(s,nu=der)
        return row
    def inner_family(side,si,lid):
        index=ids[(side,si)].index(lid)
        return owner[(side,si,ids[(side,si)][index-1])] if index else 0
    def old(side,si,s):
        return lane_edges_kinematics_at(original,float(np.clip(s,starts[si]+1e-7,ends[si]-1e-7)),side)
    xy,ss,hh=sample_road_ref(original,.05); kd=cKDTree(xy); cache={}
    def project_st(raw):
        raw_xy=np.asarray(project(raw));dist=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(raw_xy,axis=0),axis=1))]
        if source_certificate:
            # Exact coordinates for the declared Line chart: no sampled
            # nearest-reference lookup, sorting, densification or source loss.
            g=original.find('planView/geometry');h=float(g.get('hdg'))
            tangent=np.array([np.cos(h),np.sin(h)])
            normal=np.array([-tangent[1],tangent[0]])
            delta=raw_xy-np.array([float(g.get('x')),float(g.get('y'))])
            return np.c_[delta@tangent,delta@normal]
        q=np.unique(np.r_[dist,np.arange(0,dist[-1],.5)])
        pts=(np.c_[np.interp(q,dist,raw_xy[:,0]),np.interp(q,dist,raw_xy[:,1])]
             if all_source_vertices else _densify(raw_xy,.5))
        _,indices=kd.query(pts)
        delta=pts-xy[indices]; h=hh[indices]
        return np.c_[ss[indices]+delta[:,0]*np.cos(h)+delta[:,1]*np.sin(h),
                     -delta[:,0]*np.sin(h)+delta[:,1]*np.cos(h)]
    def source_id(lane):
        ud=lane.find("userData[@code='mapforge.source_lane']")
        return ud.get('value') if ud is not None else None
    def boundary_observations(side,si,lid,base=False):
        lane=occurrences[(side,si,lid)]; sid=source_id(lane)
        if sid not in cache:
            cache[sid]=[project_st(g) for g in src.lane_boundary_geometries(sid)] if sid else []
        index=0 if base else ids[(side,si)].index(lid)+1
        candidates=[]
        for raw in cache[sid]:
            g=raw[(raw[:,0]>=starts[si]-.5)&(raw[:,0]<=ends[si]+.5)]
            if len(g)<2: continue
            # Ambiguous/backtracking projections require a dedicated source
            # chain splitter; sorting a folded source would invent geometry.
            if np.all(np.diff(g[:,0])<=1e-5): g=g[::-1]
            if np.any(np.diff(g[:,0])<-.05): continue
            g=g[np.r_[True,np.diff(g[:,0])>1e-7]]
            if len(g)<2: continue
            score=float(np.median([abs(t-old(side,si,s)[index][0]) for s,t in g]))
            candidates.append((score,g))
        if not candidates: return sid,None
        if boundary_association == 'source-order':
            # Right inner (laneOffset) is the high edge; right outer is low.
            # Left outer is high, irrespective of whether both t are negative.
            return sid, ordered_source_boundary([g for _,g in candidates], select_high=base or side=='left')
        return sid,min(candidates,key=lambda x:x[0])[1]

    objective=[]; target=[]; eqs=[]; eq_values=[]; eq_labels=[]; bounds=[]; minima=[]; labels=[]; source_rows=[]
    dynamic_rows=[]; dynamic_reference=[]; dynamic_limits=[]; dynamic_labels=[]
    certified_intervals=[]
    def obs(row,value,weight=1.):
        objective.append(row*weight); target.append(value*weight)
    def eq(row,value,order=0,label=None):
        scale=20.**order; eqs.append(row*scale); eq_values.append(value*scale)
        eq_labels.append(dict(label or {'kind':'structural'}, derivative=order))
    def lower(row,value,label,unit_scale=1.):
        bounds.append(row/unit_scale); minima.append(value/unit_scale); labels.append(label)
    def tube(row,value,tolerance,label):
        lower(row,value-tolerance,label,tolerance)
        lower(-row,-value-tolerance,label,tolerance)

    for fi,f in enumerate(families):
        for si,lid in f.chain:
            side=f.side; i=0 if fi==0 else ids[(side,si)].index(lid)+1
            a,b=starts[si],ends[si]
            sid,g=None,None
            if source_mode=='centers':
                pass  # MAP observations constrain lane midpoints, never physical edges.
            else:
                try:
                    if fi==0:
                        if ids[('right',si)]: sid,g=boundary_observations('right',si,ids[('right',si)][0],True)
                    else:
                        sid,g=boundary_observations(side,si,lid)
                except ValueError as exc:
                    return {'status':'REJECTED','reason':'source boundary association unresolved',
                            'family':f.key,'section':si,'detail':str(exc)}
            sample_stations=np.linspace(a+1e-6,b-1e-6,max(3,int((b-a)/1.5)+1))
            if all_source_vertices and g is not None:
                sample_stations=np.unique(np.r_[sample_stations,g[(g[:,0]>=a)&(g[:,0]<=b),0]])
            for s in sample_stations:
                raw=old(side,si,s)[i][0]; value=raw; weight=.15
                if g is not None and g[0,0]<=s<=g[-1,0]:
                    value=float(np.interp(s,g[:,0],g[:,1])); weight=1.
                    allowed=(source_tol if source_error_budget=='absolute'
                             else max(source_tol,abs(raw-value)+.005))
                    tube(expr(fi,s),value,allowed,{'kind':'source','family':f.key,'s':float(s),'source':sid})
                    source_rows.append((expr(fi,s),value,raw,sid,float(s),f.key))
                obs(expr(fi,s),value,weight)
        a,b=f.knots[0],f.knots[-1]
        # First-order world curvature-rate proxy, not transverse-frame t'''.
        for s in np.linspace(a,b,max(3,int((b-a)/2)+1)):
            obs(expr(fi,s,3),-_ref_kappa_at(original,float(s))[1],np.sqrt(2*smooth_weight))
        for si,lid,s,role in ((f.chain[0][0],f.chain[0][1],a,'predecessor'),
                             (f.chain[-1][0],f.chain[-1][1],b,'successor')):
            if (s<=1e-6 or s>=length-1e-6) and original.find(f'link/{role}') is None:
                continue
            side=f.side; i=0 if fi==0 else ids[(side,si)].index(lid)+1
            jets=old(side,si,s)
            branch=fi!=0 and 1e-6<s<length-1e-6 and abs(jets[i][0]-jets[i-1][0])<.01
            for der in range(3):
                row=expr(fi,s,der)
                partner=endpoint_ties.get((side,si,lid,role))
                if partner is not None:
                    other=owner[(side,*partner)] if partner[1]!='0' else 0
                    eq(row-expr(other,s,der),0.,der,
                       {'kind':'branch-tie','family':f.key,'other':families[other].key,'s':float(s)})
                elif branch:
                    eq(row-expr(inner_family(side,si,lid),s,der),0.,der,
                       {'kind':'branch-tie','family':f.key,'s':float(s)})
                else:
                    link = original.find(f'link/{role}')
                    mouth=(link is not None and link.get('elementType')=='junction'
                           and (s<=1e-6 or s>=length-1e-6))
                    # Research-only mouth position is bounded by original
                    # source observations, not pinned to the previous fit.
                    # Preserve incoming jets at the other window boundary.
                    if mouth and (junction_endpoint_mode=='free' or junction_endpoint_mode=='source-parallel' and der==0):
                        continue
                    flatten = junction_endpoint_mode in ('parallel','source-parallel') and der>0 and mouth
                    eq(row, 0. if flatten else jets[i][der], der)
        # A G2 reference may have a jump in k'. A globally C2 transverse
        # spline is NOT then world-G2 when t*t' != 0:
        # delta(k_lane) = t*t'*delta(k_ref') / ((1-k*t)^2+t'^2)^1.5.
        # Without inserting extra spline knots, t'=0 at these existing
        # reference joins is a sufficient world-G2 condition for all edges
        # and their midpoint lanes. This is a construction restriction, not
        # an assertion that every source can satisfy it.
        for geom in original.findall('planView/geometry')[1:]:
            s=float(geom.get('s'))
            if a<s<b and abs(_ref_kappa_at(original,s-1e-6)[1]-_ref_kappa_at(original,s+1e-6)[1])>1e-12:
                eq(expr(fi,s,1),0.,1)

    if source_mode=='centers':
        for (side,si,lid),lane in occurrences.items():
            sid=source_id(lane)
            if not sid: continue
            if sid not in cache:
                raw=src.lane_center_geometry(sid)
                cache[sid]=project_st(raw) if raw is not None and len(raw)>=2 else None
            g=cache[sid]
            if g is None: continue
            # Keep full source segments intersecting the section. Filtering
            # vertices first can remove both bracketing endpoints of a sparse
            # segment, leaving an entire interval unconstrained.
            if not source_certificate:
                g=g[(g[:,0]>=starts[si]-.5)&(g[:,0]<=ends[si]+.5)]
            if len(g)<2: continue
            if np.all(np.diff(g[:,0])<=1e-5): g=g[::-1]
            if np.any(np.diff(g[:,0])<-.05):
                return {'status':'REJECTED','reason':'backtracking MAP lane center projection','source':sid}
            if source_certificate:
                if np.any(np.diff(g[:,0])<=1e-7):
                    return {'status':'REJECTED','reason':'repeated/folded MAP source station in exact chart','source':sid}
            else:
                g=g[np.r_[True,np.diff(g[:,0])>1e-7]]
            if len(g)<2: continue
            fi=owner[(side,si,lid)]; inner=inner_family(side,si,lid)
            index=ids[(side,si)].index(lid)
            a=max(starts[si],float(g[0,0])); b=min(ends[si],float(g[-1,0]))
            if b<=a:continue
            grid=np.linspace(a,b,max(3,int((b-a)/1.5)+1))
            if all_source_vertices:
                grid=np.unique(np.r_[grid,g[(g[:,0]>=a)&(g[:,0]<=b),0]])
            for s in grid:
                row=.5*(expr(fi,s)+expr(inner,s))
                jets=old(side,si,s); raw=.5*(jets[index][0]+jets[index+1][0])
                value=float(np.interp(s,g[:,0],g[:,1]))
                allowed=(source_tol if source_error_budget=='absolute'
                         else max(source_tol,abs(raw-value)+.005))
                obs(row,value)
                tube(row,value,allowed,{'kind':'source-center','source':sid,'s':float(s)})
                source_rows.append((row,value,raw,sid,float(s),f'center:{side}:{lid}'))
            if source_certificate:
                for (sa,ta),(sb,tb) in zip(g[:-1],g[1:]):
                    lo,hi=max(a,sa),min(b,sb)
                    if hi<=lo:continue
                    cuts=np.unique(np.concatenate([np.array([lo,hi]),np.arange(lo,hi,2.)]+
                        [f.knots[(f.knots>lo)&(f.knots<hi)] for f in (families[fi],families[inner])]))
                    for l,r in zip(cuts[:-1],cuts[1:]):
                        p0=.5*(expr(fi,l)+expr(inner,l));p1=.5*(expr(fi,r)+expr(inner,r))
                        d0=.5*(expr(fi,l,1)+expr(inner,l,1));d1=.5*(expr(fi,r,1)+expr(inner,r,1))
                        bernstein=(p0,p0+(r-l)*d0/3,p1-(r-l)*d1/3,p1)
                        targets=np.interp(np.linspace(l,r,4),[sa,sb],[ta,tb])
                        for row,value in zip(bernstein,targets):
                            tube(row,value,source_tol,{'kind':'whole-source-center','source':sid,'s':[float(l),float(r)]})
                    certified_intervals.append({'source':sid,'side':side,'lane':lid,
                        'start_s_m':float(lo),'end_s_m':float(hi)})
            if source_widths is not None and source_widths.get(sid) is not None:
                width=float(source_widths[sid]);sign=1 if side=='left' else -1
                if not np.isfinite(width) or width<0:return {'status':'REJECTED','reason':'invalid source width'}
                cuts=np.unique(np.r_[a,b,np.concatenate([f.knots[(f.knots>a)&(f.knots<b)]
                                                        for f in (families[fi],families[inner])])])
                for l,r in zip(cuts[:-1],cuts[1:]):
                    for station in np.linspace(l,r,4):
                        eq(sign*(expr(fi,station)-expr(inner,station)),width,
                           label={'kind':'explicit-source-width','source':sid,'s':float(station)})

    if mirror:
        # Enforce left_i = 2*laneOffset - right_i over complete polynomial spans,
        # not just at observation points. Four collocation values per union
        # interval uniquely determine the cubic equality on that interval.
        for si in range(len(sections)):
            if len(ids[('left',si)]) != len(ids[('right',si)]):
                return {'status':'REJECTED','reason':'mirror lane count mismatch'}
            for left,right in zip(ids[('left',si)],ids[('right',si)]):
                lf,rf=owner[('left',si,left)],owner[('right',si,right)]
                a,b=starts[si],ends[si]
                knots=sorted({float(a),float(b),*[float(v) for k in (0,lf,rf) for v in families[k].knots if a<v<b]})
                for p,q in zip(knots,knots[1:]):
                    for s in np.linspace(p,q,4):
                        eq(expr(lf,s)+expr(rf,s)-2*expr(0,s),0.)

    for (side,si,lid),lane in occurrences.items():
        fi=owner[(side,si,lid)]; inner=inner_family(side,si,lid); sign=1 if side=='left' else -1
        a,b=starts[si],ends[si]
        knots=sorted({float(a),float(b),*[float(v) for k in (fi,inner) for v in families[k].knots if a<v<b]})
        for q,r in zip(knots,knots[1:]):
            v0=sign*(expr(fi,q)-expr(inner,q)); v1=sign*(expr(fi,r)-expr(inner,r))
            d0=sign*(expr(fi,q,1)-expr(inner,q,1)); d1=sign*(expr(fi,r,1)-expr(inner,r,1))
            for row in (v0,v0+(r-q)*d0/3,v1-(r-q)*d1/3,v1):
                lower(row,0.,{'kind':'width','side':side,'lane':lid,'s':[q,r]})
        if dynamics and lane.get('type')=='driving':
            speed=max((float(e.get('max')) for e in lane.findall('speed')),default=60/3.6)
            if speed<=0: return {'status':'REJECTED','reason':'non-positive target speed'}
            grid=list(np.arange(a+1e-5,b,1.))
            cuts=knots+[float(g.get('s')) for g in original.findall('planView/geometry')]
            grid.extend(s+delta for s in cuts for delta in (-1e-5,1e-5) if a+1e-6<s+delta<b-1e-6)
            for s in sorted(set(grid)):
                kref,sharp=_ref_kappa_at(original,float(s))
                dynamic_rows.append([.5*(expr(fi,s,der)+expr(inner,s,der)) for der in range(4)])
                dynamic_reference.append((kref,sharp))
                # Construction margins, not changes to the independent G11
                # acceptance limits (2.5 m/s2 and 1.0 m/s3).
                dynamic_limits.append((2.45/speed**2,.98/speed**3))
                dynamic_labels.append({'kind':'exact-dynamics','lane':lid,'side':side,'s':float(s)})

    A=np.array(objective); y=np.array(target)
    E=np.array(eqs).reshape(-1,nvar); er=np.array(eq_values)
    particular=np.linalg.lstsq(E,er,rcond=None)[0]; Z=null_space(E)
    if len(E) and max(abs(E@particular-er))>1e-5:
        return {'status':'REJECTED','reason':'inconsistent endpoint states'}
    C=np.array(bounds); lo=np.array(minima)
    if not source_rows:
        return {'status':'REJECTED','reason':'no source observations'}
    if model_only:
        from spikes.road_constraint_model import RoadConstraintModel
        return RoadConstraintModel(original, families, owner, ids, starts, ends,
                                   A, y, E, er, C, lo, labels, source_rows,
                                   np.asarray(dynamic_rows), np.asarray(dynamic_reference),
                                   np.asarray(dynamic_limits), dynamic_labels, graph_events, eq_labels)
    D=C@Z; lower_r=lo-C@particular
    feasible=linear_feasible(D,lower_r)
    if not feasible.success:
        phase=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=np.c_[-D,-np.ones(len(D))],
                      b_ub=-lower_r,bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs')
        conflicts=[]
        if phase.success:
            slack=D@phase.x[:-1]-lower_r
            conflicts=[{**labels[i],'normalized_violation':float(-slack[i])}
                       for i in np.argsort(slack)[:12] if slack[i]<-1e-6]
        return {'status':'REJECTED','reason':'infeasible whole-road constraints',
                'normalized_phase_slack':float(phase.fun) if phase.success else None,
                'conflicts':conflicts,'dual_support':phase_certificate(phase,labels),
                'physical_graph_events':graph_events,
                'source_error_budget':source_error_budget,
                'variables':nvar,'free_variables':Z.shape[1]}
    if not source_rows:
        return {'status':'REJECTED','reason':'no source observations'}
    H=A.T@A; g=A.T@y
    solution,qp_info=_convex_qp(H,g,E,er,C,lo,particular+Z@feasible.x)
    iterations=[qp_info]
    if solution is None:
        return {'status':'REJECTED','reason':'source quadratic solve failed','qp':qp_info}
    if dynamic_rows:
        T=np.asarray(dynamic_rows); ref=np.asarray(dynamic_reference); limits=np.asarray(dynamic_limits)
        for iteration in range(10):
            jets=T@solution
            values=world_kinematics(jets,ref[:,0],ref[:,1])
            violation=float(np.max(abs(values)/limits))
            if violation<=1.+1e-6: break
            derivatives=[]
            for der in range(4):
                perturb=jets.astype(complex); perturb[:,der]+=1e-25j
                derivatives.append(world_kinematics(perturb,ref[:,0],ref[:,1]).imag/1e-25)
            jac=np.einsum('dpc,pdn->pcn',np.array(derivatives),T)
            rhs=np.einsum('pcn,n->pc',jac,solution)-values
            J=(jac/limits[:,:,None]).reshape(-1,nvar); rhs=(rhs/limits).ravel()
            CC=np.vstack([C,J,-J]); ll=np.r_[lo,rhs-1,-rhs-1]
            next_solution,qp_info=_convex_qp(H,g,E,er,CC,ll,solution)
            worst = np.unravel_index(np.argmax(abs(values)/limits), values.shape)
            quantity = ('curvature_per_m', 'curvature_rate_per_m2')[worst[1]]
            worst_state = dict(dynamic_labels[worst[0]], quantity=quantity,
                               actual=float(values[worst]), limit=float(limits[worst]),
                               ratio=violation)
            qp_info.update(iteration=iteration,previous_exact_ratio=violation,
                           worst_previous_dynamics=worst_state)
            iterations.append(qp_info)
            if next_solution is None:
                if restore_dynamics:
                    from spikes.nonlinear_feasibility import restore
                    def exact(x):
                        return world_kinematics(T@x,ref[:,0],ref[:,1])/limits
                    def jacobian(x):
                        jets = T@x; derivatives = []
                        for der in range(4):
                            perturbed = jets.astype(complex); perturbed[:,der] += 1e-25j
                            derivatives.append(world_kinematics(perturbed,ref[:,0],ref[:,1]).imag/1e-25)
                        return np.einsum('dpc,pdn->pcn',np.array(derivatives),T)/limits[:,:,None]
                    restored, restoration = restore(solution,E,er,C,lo,exact,jacobian)
                    iterations.append({'feasibility_restoration': restoration})
                    if restored is not None:
                        solution = restored
                        violation = float(np.max(abs(exact(solution))))
                        break
                # The solver status is not proof that source data are wrong.
                # Report the model's linearized conflict without relaxing it.
                dd=CC@Z; rr=ll-CC@particular
                phase=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=np.c_[-dd,-np.ones(len(dd))],
                    b_ub=-rr,bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs')
                conflicts = []
                if phase.success:
                    dynamics_labels = [dict(label, quantity=quantity)
                        for label in dynamic_labels for quantity in ('curvature_per_m', 'curvature_rate_per_m2')]
                    all_labels = labels + [dict(label, bound=direction)
                        for direction in ('lower', 'upper') for label in dynamics_labels]
                    slack = dd@phase.x[:-1]-rr
                    conflicts = [dict(all_labels[i], normalized_violation=float(-slack[i]))
                                 for i in np.argsort(slack)[:12] if slack[i] < -1e-6]
                return {'status':'REJECTED','reason':'exact-dynamics subproblem failed','qp_iterations':iterations,
                        'normalized_phase_slack':float(phase.fun) if phase.success else None,
                        'conflicts':conflicts,
                        'scope':'current linearized subproblem; not proof of global nonlinear infeasibility'}
            solution=next_solution
        else:
            return {'status':'REJECTED','reason':'exact-dynamics iteration did not converge','qp_iterations':iterations}
        # Need regular Frenet coordinates, not a near-singular offset curve.
        if np.min(1-ref[:,0]*(T@solution)[:,0])<.2:
            return {'status':'REJECTED','reason':'non-regular Frenet offset'}
    modified=copy.deepcopy(original); new_sections=modified.findall('lanes/laneSection')
    lanes_element=modified.find('lanes')
    for item in list(lanes_element.findall('laneOffset')): lanes_element.remove(item)
    def coefficients(fi,s):
        f=families[fi]; c=BSpline(f.knots,solution[f.columns],3)
        return {name:f'{float(c(s,nu=k))/[1,1,2,6][k]:.14g}' for k,name in enumerate('abcd')}
    for index,s in enumerate(np.unique(families[0].knots)[:-1]):
        lanes_element.insert(index,etree.Element('laneOffset',s=f'{s:.12g}',**coefficients(0,s)))
    for (side,si,lid),_lane in occurrences.items():
        lane=new_sections[si].find(f"{side}/lane[@id='{lid}']"); fi=owner[(side,si,lid)]
        for item in list(lane.findall('width'))+list(lane.findall('border')): lane.remove(item)
        cuts=sorted({float(starts[si]),*[float(v) for v in families[fi].knots if starts[si]<v<ends[si]]})
        index=1 if lane.find('link') is not None else 0
        for s in cuts:
            lane.insert(index,etree.Element('border',sOffset=f'{s-starts[si]:.12g}',**coefficients(fi,s))); index+=1
        meta=lane.find("userData[@code='mapforge.provenance/v1']")
        if meta is not None:
            prov=json.loads(meta.get('value','{}')); prov.update(status='APPROXIMATED',geometry_adjustment='whole-road-boundary-candidate')
            prov['junction_endpoint_mode'] = junction_endpoint_mode
            meta.set('value',json.dumps(prov,ensure_ascii=False,separators=(',',':')))
    if _materialize_widths(modified,new_sections,starts,ends) is None:
        return {'status':'REJECTED','reason':'materialization failed'}
    # No input mutation until all model constraints and materialization pass.
    road[:]=list(modified)
    old_errors=[abs(raw-v) for row,v,raw,*_ in source_rows]
    new_errors=[abs(row@solution-v) for row,v,*_ in source_rows]
    return {'status':'CANDIDATE','production_promoted':False,'variables':nvar,'free_variables':Z.shape[1],
            'minimum_independent_span_m':float(min(min(np.diff(np.unique(f.knots))) for f in families)),
            'knot_phase_m':knot_phase,
            'source_tolerance_m':source_tol,
            'source_error_budget':source_error_budget,
            'source_geometry_semantics':source_mode,
            'whole_source_certificate_requested':source_certificate,
            'certified_source_intervals':certified_intervals,
            'certificate_scope':'original center segments intersecting written lane support; not longitudinal coverage',
            'boundary_association':boundary_association,
            'physical_graph_events':graph_events,
            'junction_endpoint_mode':junction_endpoint_mode,
            'strict_same_leg_mirror':mirror,
            'world_g2_reference_join_stations_m':[float(g.get('s')) for g in original.findall('planView/geometry')[1:]
                if abs(_ref_kappa_at(original,float(g.get('s'))-1e-6)[1]
                       -_ref_kappa_at(original,float(g.get('s'))+1e-6)[1])>1e-12],
            'source_samples':len(source_rows),'source_p95_m':float(np.percentile(new_errors,95)),
            'source_max_m':float(max(new_errors)),'old_source_p95_m':float(np.percentile(old_errors,95)),
            'old_source_max_m':float(max(old_errors)),'dynamics_constrained':dynamics,
            'qp_iterations':iterations,
            'exact_construction_ratio':violation if dynamic_rows else None}


def main():
    p=argparse.ArgumentParser(); p.add_argument('input',type=Path); p.add_argument('output',type=Path)
    p.add_argument('--road',action='append'); p.add_argument('--span',type=float,default=15.)
    p.add_argument('--source-tolerance',type=float,default=.35); p.add_argument('--diagnostic-no-dynamics',action='store_true')
    p.add_argument('--knot-phase',type=float)
    a=p.parse_args()
    if a.input.resolve()==a.output.resolve(): raise ValueError('candidate must not overwrite input')
    tree=etree.parse(str(a.input)); lat,lon=_origin(tree.getroot()); src=shp_source(); report={}
    for rd in tree.findall('road'):
        if rd.get('junction')!='-1' or rd.get('name')=='junction_paving': continue
        if a.road and rd.get('id') not in a.road: continue
        result=solve_road(rd,src,lambda g:_project(g,lat,lon),a.span,a.source_tolerance,
                          dynamics=not a.diagnostic_no_dynamics,knot_phase=a.knot_phase)
        report[rd.get('id')]=result; print(rd.get('id'),json.dumps(result),flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    tree.write(str(a.output),encoding='utf-8',xml_declaration=True,pretty_print=True)
    g11=audit_file(a.output,ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    rejected=any(r['status']!='CANDIDATE' for r in report.values()) or not report
    report['_metadata']={'candidate_only':True,'production_promoted':False,
        'input_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'g11':g11['status'],
        'solver_rejected':rejected,'note':'A rejected road remains the unchanged input road; never treat its copy as a solved candidate.'}
    a.output.with_suffix('.fit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    a.output.with_suffix('.g11.json').write_text(json.dumps(g11,indent=2),encoding='utf-8')
    print(g11['summary'],flush=True)
    return 2 if rejected else 0 if g11['status']=='PASS' else 1


if __name__=='__main__': raise SystemExit(main())
