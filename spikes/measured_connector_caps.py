"""Measured connector ribbon trial: <=5 reference primitives, fixed C2 sections.

Unlike the MAP endpoint-only trial, both original via edges and center enter
the shape objective and acceptance. IDs and speed fields stay immutable.
Optional structural mouth relocation restricts ordinary geometry exactly and
includes the omitted source pieces in connector support. Failed trials are
diagnostic files, never accepted replacements.
"""
import argparse
import copy
import hashlib
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pyclothoids import Clothoid
from scipy.optimize import minimize,root as solve_root

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source
from spikes.connector_cross_section import endpoint_frame,edge_jet,flat_join_spline
from spikes.road_boundary_family import world_kinematics
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.g11 import load_policy,_audit_d,audit_file
from mapforge.validate.junction_edges import audit as edge_audit
from mapforge.validate.smoothness import junction_lane_interfaces
from mapforge.ops.refline_fit import solve_g2_balanced
from spikes.trial_xml import replace_road,write_trial


def distances(points, polyline):
    """Euclidean point-to-segment distances, retaining endpoints (no cropping)."""
    p=np.asarray(points,float);g=np.asarray(polyline,float)
    a=g[:-1];v=np.diff(g,axis=0);den=np.einsum('ij,ij->i',v,v)
    good=den>1e-14;a=a[good];v=v[good];den=den[good]
    if not len(a):raise ValueError('degenerate source curve')
    d=p[:,None,:]-a
    q=np.clip(np.einsum('nmi,mi->nm',d,v)/den,0,1)
    return np.linalg.norm(d-q[:,:,None]*v,axis=2).min(axis=1)


def frames(root, road):
    roads={r.get('id'):r for r in root.findall('road')}
    lanes=road.findall('lanes/laneSection/right/lane')
    if len(road.findall('lanes/laneSection'))!=1 or len(lanes)!=1 or lanes[0].get('id')!='-1':
        raise ValueError('requires one right lane measured connector')
    out=[]
    for role in ('predecessor','successor'):
        link=road.find('link/'+role);cp=link.get('contactPoint')
        lid=int(lanes[0].find('link/'+role).get('id'))
        out.append(endpoint_frame(roads[link.get('elementId')],lid,cp,
                   cp==('end' if role=='predecessor' else 'start')))
    return out


def chain(parameters, a, b, caps=(True,True)):
    """Three core primitives plus only endpoint caps required by edge jets."""
    n=3+sum(caps)
    lens=np.asarray(parameters[:n],float);k1,k2=parameters[n:]/20.
    if min(lens)<=0:raise ValueError('nonpositive span')
    # Parent line/arc/spiral derivative determines each end cap, not a guessed
    # zero. The last cap is marched forward and exact endpoint closure is a
    # separate constraint, never silently translated after optimization.
    ks=[a['k']]
    if caps[0]:ks.append(a['k']+a['dk']*lens[0])
    ks.extend([k1,k2])
    if caps[1]:ks.append(b['k']-b['dk']*lens[-1])
    ks.append(b['k'])
    x,y,h=a['pose'];cls=[]
    for length,k0,kend in zip(lens,ks[:-1],ks[1:]):
        c=Clothoid.StandardParams(x,y,h,k0,(kend-k0)/length,length);cls.append(c)
        x,y,h=c.XEnd,c.YEnd,c.ThetaEnd
    return cls


def ribbon(cls,a,b,raw=None,world_g2=False):
    if raw is not None:
        from mapforge.ops.source_connector_ribbon import fit_source_ribbon,fit_world_ribbon
        starts={s:edge_jet(a,a['edges'][s],cls[0].KappaStart,cls[0].dk) for s in ('left','right')}
        ends={s:edge_jet(b,b['edges'][s],cls[-1].KappaEnd,cls[-1].dk) for s in ('left','right')}
        return (fit_world_ribbon if world_g2 else fit_source_ribbon)(cls,starts,ends,raw)
    co={};knots=None
    for side in ('left','right'):
        start=edge_jet(a,a['edges'][side],cls[0].KappaStart,cls[0].dk)
        end=edge_jet(b,b['edges'][side],cls[-1].KappaEnd,cls[-1].dk)
        knots,co[side]=flat_join_spline(cls,start,end)
    return knots,co


def sample(cls,knots,co,step=0.5):
    # Dense samples are evaluation only, never independent fitted records.
    ref_st=np.r_[0.,np.cumsum([c.length for c in cls])]
    stations=np.unique(np.r_[np.arange(0,ref_st[-1],step),knots,ref_st])
    xy=[];hh=[];kk=[];dk=[]
    for s in stations:
        i=min(len(cls)-1,max(0,int(np.searchsorted(ref_st,s,side='right')-1)))
        c=cls[i];u=float(np.clip(s-ref_st[i],0,c.length))
        xy.append([c.X(u),c.Y(u)]);hh.append(c.Theta(u));kk.append(c.KappaStart+c.dk*u);dk.append(c.dk)
    xy=np.array(xy);normal=np.c_[-np.sin(hh),np.cos(hh)]
    idx=np.clip(np.searchsorted(knots,stations,side='right')-1,0,len(knots)-2)
    u=stations-knots[idx];jets={};points={}
    for side in ('left','right'):
        c=co[side][idx];a0,b0,c0,d0=c.T
        jets[side]=np.c_[a0+u*(b0+u*(c0+u*d0)),b0+u*(2*c0+u*3*d0),2*c0+6*d0*u,6*d0]
        points[side]=xy+jets[side][:,0,None]*normal
    jets['center']=(jets['left']+jets['right'])/2
    points['center']=(points['left']+points['right'])/2
    dyn=world_kinematics(jets['center'],np.array(kk),np.array(dk))
    return points,jets,dyn


def write_ribbon(road,cls,knots,co):
    result=copy.deepcopy(road);pv=result.find('planView');pv.clear();s=0.
    for c in cls:
        g=ET.SubElement(pv,'geometry',s=str(s),x=str(c.XStart),y=str(c.YStart),hdg=str(c.ThetaStart),length=str(c.length))
        if abs(c.dk)<1e-12:
            if abs(c.KappaStart)<1e-12:ET.SubElement(g,'line')
            else:ET.SubElement(g,'arc',curvature=str(c.KappaStart))
        else:ET.SubElement(g,'spiral',curvStart=str(c.KappaStart),curvEnd=str(c.KappaEnd))
        s+=c.length
    result.set('length',str(s));group=result.find('lanes');lane=result.find('lanes/laneSection/right/lane')
    for e in group.findall('laneOffset'):group.remove(e)
    for e in lane.findall('width')+lane.findall('border'):lane.remove(e)
    for i,so in enumerate(knots[:-1]):
        group.insert(i,ET.Element('laneOffset',s=str(so),**dict(zip('abcd',map(str,co['left'][i])))))
        lane.insert(i+1,ET.Element('width',sOffset=str(so),**dict(zip('abcd',map(str,co['left'][i]-co['right'][i])))))
    ud=lane.find("userData[@code='mapforge.provenance/v1']")
    if ud is not None:
        meta=json.loads(ud.get('value','{}'));meta.update(candidate_only=True,
            cross_section_fit='source-constrained-world-edge-G2-fixed-cubic-ribbon',
            reference_primitive_count=len(cls),width_record_count=len(knots)-1,
            minimum_width_record_span_m=float(min(np.diff(knots))),
            source_acceptance='requires-independent-full-raw-review')
        ud.set('value',json.dumps(meta,separators=(',',':')))
    return result


def raw_curves(src,sid,project):
    rec=src.lane(sid)
    if rec is None:raise ValueError('missing raw via identity')
    center=np.asarray(project(rec.geometry),float)
    edges=[np.asarray(project(g),float) for g in src.lane_boundary_geometries(sid)]
    if len(edges)!=2:raise ValueError('both raw measured via boundaries required')
    # Determine left/right with respect to the original via direction, not
    # proximity to the candidate. Use the center's nearest segment tangent.
    def side(g):
        middle=center[len(center)//2];i=max(0,min(len(center)-2,len(center)//2))
        tangent=center[i+1]-center[i]
        point=g[np.argmin(np.linalg.norm(g-middle,axis=1))]-middle
        return float(tangent[0]*point[1]-tangent[1]*point[0])
    edges.sort(key=side,reverse=True)
    if side(edges[0])<=0 or side(edges[1])>=0:raise ValueError('raw via sides unresolved')
    def dense(v):
        s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(v,axis=0),axis=1))]
        q=np.unique(np.r_[s,np.arange(0,s[-1],.25)])
        return np.c_[np.interp(q,s,v[:,0]),np.interp(q,s,v[:,1])]
    return {k:dense(v) for k,v in zip(('center','left','right'),[center,*edges])}


def crop_poly_records(elements,start_attr,lo,hi):
    """Restrict existing cubics exactly; no resampling/refitting."""
    if not elements:return []
    ordered=sorted(elements,key=lambda e:float(e.get(start_attr)))
    result=[]
    for i,e in enumerate(ordered):
        a=float(e.get(start_attr));b=float(ordered[i+1].get(start_attr)) if i+1<len(ordered) else hi
        start=max(a,lo)
        if min(b,hi)<=start+1e-10:continue
        x=start-a;aa,bb,cc,dd=[float(e.get(k,'0')) for k in 'abcd']
        n=copy.deepcopy(e);n.set(start_attr,str(start-lo))
        for k,v in zip('abcd',[aa+x*(bb+x*(cc+x*dd)),bb+x*(2*cc+3*dd*x),cc+3*dd*x,dd]):n.set(k,str(v))
        result.append(n)
    return result


def trim_ordinary(road,distance,*,interval=None):
    """Move only the structural junction contact upstream, retaining geometry.

    Both trimmed source pieces must be included in connector source composites.
    This is not permission to move original stop lines, signals or coordinates.
    Current trial rejects roads with objects/signals rather than dropping them.
    """
    if not math.isfinite(distance) or distance<0:raise ValueError('invalid mouth relocation distance')
    if distance==0 and interval is None:return copy.deepcopy(road)
    if road.find('signals') is not None or road.find('objects') is not None:
        raise ValueError('mouth relocation requires explicit signal/object station remapping')
    if any(road.find(p) is not None for p in ('elevationProfile/elevation','lateralProfile/superelevation','surface')):
        raise ValueError('mouth relocation requires vertical/surface station remapping')
    if len(road.findall('planView/geometry'))!=1 or road.find('planView/geometry/line') is None:
        raise ValueError('current mouth trial only restricts a known single straight reference')
    L=float(road.get('length'));lo=0.;hi=L
    for role in ('predecessor','successor'):
        link=road.find('link/'+role)
        if link is not None and link.get('elementType')=='junction':
            if role=='predecessor':lo=distance
            else:hi=L-distance
    if interval is not None:
        lo,hi=map(float,interval)
        if not all(math.isfinite(v) for v in (lo,hi)) or lo<0 or hi>L:
            raise ValueError('invalid ordinary restriction interval')
    if hi<=lo:raise ValueError('mouth relocation removes the road')
    r=copy.deepcopy(road);r.set('length',str(hi-lo));g=r.find('planView/geometry');h=float(g.get('hdg'))
    g.set('x',str(float(g.get('x'))+lo*math.cos(h)));g.set('y',str(float(g.get('y'))+lo*math.sin(h)))
    g.set('length',str(hi-lo));group=r.find('lanes')
    new_offsets=crop_poly_records(group.findall('laneOffset'),'s',lo,hi)
    for e in group.findall('laneOffset'):group.remove(e)
    for i,e in enumerate(new_offsets):group.insert(i,e)
    sections=list(group.findall('laneSection'));starts=[float(s.get('s')) for s in sections]
    for i,sec in enumerate(sections):
        a=starts[i];b=starts[i+1] if i+1<len(starts) else L;x=max(a,lo);y=min(b,hi)
        if y<=x+1e-10:group.remove(sec);continue
        sec.set('s',str(x-lo))
        for lane in sec.findall('./left/lane')+sec.findall('./right/lane'):
            for tag in ('width','border'):
                old=lane.findall(tag);new=crop_poly_records(old,'sOffset',x-a,y-a)
                index=list(lane).index(old[0]) if old else 0
                for e in old:lane.remove(e)
                for j,e in enumerate(new):lane.insert(index+j,e)
            # Non-geometric lane marks/speeds are constant in this data path.
            if any(float(e.get('sOffset','0'))!=0 for tag in ('roadMark','speed','access','height') for e in lane.findall(tag)):
                raise ValueError('mouth relocation needs explicit nonconstant lane metadata remapping')
    current=group.findall('laneSection')
    for role,sec in (('predecessor',current[0]),('successor',current[-1])):
        road_link=r.find('link/'+role)
        if road_link is not None and road_link.get('elementType')=='junction':
            for lane in sec.findall('./left/lane')+sec.findall('./right/lane'):
                link=lane.find('link')
                if link is not None:
                    for e in link.findall(role):link.remove(e)
                    if not list(link):lane.remove(link)
    return r


def crop_at_projection(curve,point,keep_after):
    p=np.asarray(point);g=np.asarray(curve);v=np.diff(g,axis=0);den=np.sum(v*v,axis=1)
    f=np.clip(np.sum((p-g[:-1])*v,axis=1)/np.maximum(den,1e-20),0,1)
    q=g[:-1]+f[:,None]*v;i=int(np.argmin(np.linalg.norm(q-p,axis=1)))
    return np.vstack([q[i],g[i+1:]]) if keep_after else np.vstack([g[:i+1],q[i]])


def unique_source_path(topo,start,end,max_hops=6):
    """Bounded original-TOPO path, never a nearest-lane or same-ID guess."""
    pending=[[start]];found=[]
    while pending:
        path=pending.pop(0)
        if path[-1]==end:
            found.append(path)
            if len(found)>1:raise ValueError('ambiguous original source path')
        elif len(path)<=max_hops:
            pending.extend(path+[n] for n in dict.fromkeys(topo.get(path[-1],[])) if n not in path)
    if not found:raise ValueError('no original source path to relocated contact')
    return found[0]


def composite_sources(root,road,src,project):
    """All raw via points plus explicitly linked approach/departure support."""
    rs={r.get('id'):r for r in root.findall('road')};lane=road.find('lanes/laneSection/right/lane')
    vid=lane.find("userData[@code='mapforge.source_lane']").get('value');ids=[]
    for role in ('predecessor','successor'):
        link=road.find('link/'+role);lid=lane.find('link/'+role).get('id');other=rs[link.get('elementId')]
        sec=other.findall('lanes/laneSection')[0 if link.get('contactPoint')=='start' else -1]
        sid=sec.find(('left' if int(lid)>0 else 'right')+"/lane[@id='"+lid+"']/userData[@code='mapforge.source_lane']")
        if sid is None:raise ValueError('missing linked source identity at relocated mouth')
        ids.append(sid.get('value'))
    path=(unique_source_path(src.topo_out,ids[0],vid)
          +unique_source_path(src.topo_out,vid,ids[1])[1:])
    if len(path)<3:raise ValueError('relocated contact does not bound original via')
    sources=[raw_curves(src,s,project) for s in path]
    ends=frames(root,road);result={};gaps={}
    for field in ('center','left','right'):
        pieces=[v[field] for v in sources]
        # Travel ordering must be supported by the original graph AND geometry.
        # Do not silently reverse a center and leave its left/right identities.
        gap=[float(np.linalg.norm(a[-1]-b[0])) for a,b in zip(pieces,pieces[1:])]
        if max(gap)>1e-5:raise ValueError(f'raw {field} chain gap {gap}; no synthetic bridge')
        def point(f):
            state=f['center'] if field=='center' else f['edges'][field]
            return [state['x'],state['y']]
        pieces[0]=crop_at_projection(pieces[0],point(ends[0]),True)
        pieces[-1]=crop_at_projection(pieces[-1],point(ends[1]),False)
        result[field]=np.vstack(pieces);gaps[field]=gap
    return result,{'source_lane_ids':path,'raw_join_gaps_m':gaps,'raw_via_preserved':True}


def seed_chain(a,b,start_cap,end_cap):
    first=Clothoid.StandardParams(*a['pose'],a['k'],a['dk'],start_cap) if start_cap else None
    last=None
    if end_cap:
        reverse=Clothoid.StandardParams(b['pose'][0],b['pose'][1],b['pose'][2]+math.pi,-b['k'],b['dk'],end_cap)
        last=Clothoid.StandardParams(reverse.XEnd,reverse.YEnd,reverse.ThetaEnd-math.pi,-reverse.KappaEnd,b['dk'],end_cap)
    start=(first.XEnd,first.YEnd,first.ThetaEnd) if first is not None else a['pose']
    end=(last.XStart,last.YStart,last.ThetaStart) if last is not None else b['pose']
    middle,_=solve_g2_balanced(start,end,first.KappaEnd if first is not None else a['k'],last.KappaStart if last is not None else b['k'])
    return ([first] if first is not None else [])+list(middle)+([last] if last is not None else [])


def needs_cap(frame):
    """Zero transverse slope makes reference sharpness irrelevant at contact."""
    return any(abs(edge_jet(frame,e,frame['k'],frame['dk'])[1])>1e-9 for e in frame['edges'].values())


def polish_endpoint(z,a,b,caps):
    """Close a nearly feasible chain by parameter solve, never translating it."""
    n=3+sum(caps);indices=[int(caps[0])+1,n,n+1]
    def residual(v):
        q=np.array(z,copy=True);q[indices]=v;c=chain(q,a,b,caps)[-1]
        return [c.XEnd-b['pose'][0],c.YEnd-b['pose'][1],
                20*np.arctan2(np.sin(c.ThetaEnd-b['pose'][2]),np.cos(c.ThetaEnd-b['pose'][2]))]
    result=solve_root(residual,np.asarray(z)[indices],tol=1e-10)
    q=np.array(z,copy=True);q[indices]=result.x
    return q,{'solver_success':bool(result.success),'before':list(map(float,residual(np.asarray(z)[indices]))),
              'after':list(map(float,residual(result.x))),'shape_shift_max':float(max(abs(q-z)))}


def source_slacks(errors,margin=0.):
    """The construction margin tightens, never relaxes, the final source gate."""
    if not 0<=margin<.35:raise ValueError('invalid source construction margin')
    return np.array([v for field in errors.values() for e in field.values()
        for v in (1.5-margin-max(e),.75-margin-np.percentile(e,95),.35-margin-np.median(e))])


def choose_shape(feasible,objective,construction_limits,prefer_margin):
    """Do not discard a margin-safe fit in favor of a cheaper threshold seed."""
    margins=[p for p in feasible if min(construction_limits(p))>=-1e-5]
    eligible=margins if prefer_margin and margins else feasible
    return (min(eligible,key=objective).copy() if eligible else None),len(margins)


def fit(road,a,b,raw,*,optimize=True,min_segment=6.,design_speed_kmh=15.,seed_caps=None,adaptive_caps=False,source_warm_start=False,parameters=None,raw_via=None,require_dynamics=True,source_cross_section=False,world_g2_cross_section=False):
    # Explicit geometry-review mode only. Keep every source/width/end-state
    # constraint and compute dynamics at unchanged speeds; never return the
    # ordinary accepted-candidate status from this mode.
    if not isinstance(require_dynamics,bool):raise ValueError('require_dynamics must be explicit boolean')
    if world_g2_cross_section and not source_cross_section:raise ValueError('world-G2 cross sections require original source observations')
    caps=seed_caps or (min_segment,min_segment)
    if min(caps)<min_segment:raise ValueError('seed cap below reference minimum')
    required=(needs_cap(a),needs_cap(b)) if adaptive_caps else (True,True)
    caps=[v if needed else 0. for v,needed in zip(caps,required)]
    cls=seed_chain(a,b,*caps);n=len(cls);first=int(required[0])
    seed=np.r_[[c.length for c in cls],cls[first].KappaEnd*20,cls[first+1].KappaEnd*20]
    if parameters is not None:
        seed=np.asarray(parameters,float)
        if seed.shape!=(n+2,) or not np.isfinite(seed).all() or min(seed[:n])<=0:
            raise ValueError('invalid saved shape parameters')
    speed=max(design_speed_kmh,max((float(v.get('max'))*3.6 for v in road.findall('.//lane/speed')),default=0.))/3.6
    cache={}
    def evaluate(z):
        key=tuple(z)
        if key not in cache:
            cc=chain(z,a,b,required);kn,coeff=ribbon(cc,a,b,raw if source_cross_section else None,world_g2_cross_section);pts,jets,dyn=sample(cc,kn,coeff)
            # Both directions, all original via observations retained.
            errors={k:{'source_to_target':distances(raw[k],pts[k]),'target_to_source':distances(pts[k],raw[k])} for k in raw}
            if raw_via is not None:
                # Adjacent straight support must not dilute the error statistics
                # of the measured turn itself. Review has always checked this.
                errors.update({'raw_via:'+k:{'source_to_target':distances(v,pts[k])} for k,v in raw_via.items()})
            # Exact cubic extrema prevent a narrow negative-width interval
            # from falling between sampled stations.
            width=[]
            for c,L in zip(coeff['left']-coeff['right'],np.diff(kn)):
                aa,bb,cc0,dd=c;qs=[0.,L]
                qs.extend(float(t.real) for t in np.roots([3*dd,2*cc0,bb]) if abs(t.imag)<1e-9 and 0<t.real<L)
                width.extend(aa+t*(bb+t*(cc0+t*dd)) for t in qs)
            ratio=np.max(np.abs(dyn)*np.array([speed**2/2.5,speed**3]))
            from mapforge.ops.source_connector_ribbon import minimum_forward_factor
            forward=minimum_forward_factor(cc,kn,coeff) if world_g2_cross_section else float('inf')
            end=cc[-1];closure=np.array([end.XEnd-b['pose'][0],end.YEnd-b['pose'][1],
                20*math.atan2(math.sin(end.ThetaEnd-b['pose'][2]),math.cos(end.ThetaEnd-b['pose'][2]))])
            cache.clear();cache[key]=(cc,kn,coeff,errors,width,ratio,forward,closure)
        return cache[key]
    def objective(z):
        _,_,_,errs,_,_,_,_=evaluate(z)
        return sum(np.mean(e**2) for field in errs.values() for e in field.values())+.0002*sum(z[:n])
    def limits(z,construction=False):
        _,_,_,err,width,ratio,forward,_=evaluate(z)
        # Same preferred center fidelity bounds as measured conversion;
        # apply to both measured edges too, not just the center.
        dynamic_limits=[(.98 if construction else 1)-ratio] if require_dynamics else []
        return np.r_[dynamic_limits,min(width)-.1,([forward-.1] if world_g2_cross_section else []),
                     source_slacks(err,.001 if construction else 0.)]
    def closure(z):return evaluate(z)[-1]
    z=seed.copy();optimizer=None;warm=None;polished=None;pool=[z.copy()]
    if optimize:
        z[:n]=np.maximum(z[:n],min_segment)
        if source_warm_start:
            # Phase zero may be outside the final source/dynamics tube. It is
            # only a starting point, NEVER an accepted relaxed conversion.
            rr=minimize(objective,z,method='SLSQP',bounds=[(min_segment,100.)]*n+[(-6.,6.)]*2,
                constraints=[{'type':'eq','fun':closure},{'type':'ineq','fun':lambda q:min(min(evaluate(q)[4]),evaluate(q)[6])-.1}],
                options={'maxiter':100,'ftol':1e-9,**({'eps':1e-5} if source_cross_section else {})})
            warm={'success':bool(rr.success),'message':str(rr.message),'iterations':int(rr.nit),
                  'source_objective':float(objective(rr.x)),'final_gate_slack':float(min(limits(rr.x))),
                  'closure_max':float(max(abs(closure(rr.x))))}
            pool.append(rr.x.copy())
            if warm['closure_max']<1e-5 and objective(rr.x)<objective(z):z=rr.x
        result=minimize(objective,z,method='SLSQP',bounds=[(min_segment,100.)]*n+[(-6.,6.)]*2,
            constraints=[{'type':'eq','fun':closure},{'type':'ineq','fun':lambda q:limits(q,construction=True)}],
            options={'maxiter':100,'ftol':1e-8,**({'eps':1e-5} if source_cross_section else {})})
        z=result.x;optimizer={'success':bool(result.success),'message':str(result.message),'iterations':int(result.nit)}
    # This bound only selects a numerical starting point. Acceptance below
    # still requires exact closure and every original hard constraint.
    if 1e-8<max(abs(closure(z)))<.1 and min(limits(z))>=-1e-5:
        try:
            proposal,polished=polish_endpoint(z,a,b,required)
            accepted=(max(abs(closure(proposal)))<1e-7 and min(proposal[:n])>=min_segment-1e-7
                      and min(limits(proposal))>=-1e-5)
            polished['accepted']=bool(accepted)
            if accepted:z=proposal
        except (ValueError,np.linalg.LinAlgError) as exc:
            polished={'accepted':False,'error':str(exc)}
    pool.append(z.copy())
    feasible=[p for p in pool if min(p[:n])>=min_segment-1e-7
              and max(abs(closure(p)))<=1e-6 and min(limits(p))>=-1e-5]
    chosen,margin_count=choose_shape(feasible,objective,lambda q:limits(q,construction=True),optimize)
    if chosen is not None:z=chosen
    cc,kn,coeff,err,width,ratio,forward,res=evaluate(z)
    candidate=write_ribbon(road,cc,kn,coeff)
    report={'status':('CANDIDATE' if require_dynamics else 'GEOMETRY_REVIEW_CANDIDATE') if max(abs(res))<=1e-6 and min(z[:n])>=min_segment-1e-7 and min(limits(z))>=-1e-5 else 'REJECTED',
        'dynamics_required_for_this_research_selection':require_dynamics,
        'cross_section_model':'source-fitted-structural-at-most-eight-spans' if source_cross_section else 'endpoint-determined',
        'finite_difference_step':1e-5 if source_cross_section else None,
        'world_g2_cross_section':world_g2_cross_section,
        'dynamics_status':'PASS' if ratio<=1 else 'FAIL',
        'production_accepted':False,
        'road':road.get('id'),'optimizer':optimizer,'minimum_requested_reference_span_m':min_segment,
        'source_warm_start':warm,
        'construction_margin':{'source_m':.001,'dynamics_ratio_target':.98,'final_acceptance_unchanged':True},
        'shape_origin':'saved-parameters' if parameters is not None else 'endpoint-seed',
        'endpoint_polish':polished,
        'feasible_shapes_considered':len(feasible),
        'construction_margin_shapes_considered':margin_count,
        'seed_caps_m':list(caps),'shape_parameters':list(map(float,z)),'required_endpoint_caps':list(required),
        'primitive_lengths_m':list(map(float,z[:n])),'width_records':len(kn)-1,
        'minimum_width_record_span_m':float(min(np.diff(kn))),
        'source':{k:{direction:{'median_m':float(np.median(e)),'p95_m':float(np.percentile(e,95)),'max_m':float(max(e))} for direction,e in field.items()} for k,field in err.items()},
        'source_scope':'full raw via and declared adjacent-source support; both directions separately; all raw vertices included',
        'raw_via_separately_constrained':raw_via is not None,
        'source_metric_mode':'separate-directions',
        'endpoint_closure_residual':res.tolist(),'minimum_width_exact_m':float(min(width)),
        'minimum_forward_factor_exact':float(forward) if world_g2_cross_section else None,
        'construction_dynamics_ratio':float(ratio),'evaluation_speed_kmh':speed*3.6,'speed_fields_unchanged':True,
        'production_promoted':False}
    return candidate,report


def run(source,target,road_ids,*,optimize=True,mouth_retreat=0.,adaptive_caps=False,composite_support=False,source_warm_start=False):
    if source.resolve()==target.resolve():raise ValueError('never overwrite source')
    tree=ET.parse(source);root=tree.getroot();src=shp_source();lat,lon=_origin(root)
    if mouth_retreat:
        all_measured={r.get('id') for r in root.findall('road') if r.get('junction')!='-1'
                      and r.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']") is not None}
        if set(road_ids)!=all_measured:raise ValueError('mouth relocation must rebuild all measured connectors together')
        for road in list(root.findall('road')):
            if road.get('junction')=='-1':
                cropped=trim_ordinary(road,mouth_retreat);replace_road(root,road,cropped)
    cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');cfg['dynamics']['sample_step_m']=.02
    rows=[]
    for road in list(root.findall('road')):
        if road.get('id') not in road_ids:continue
        try:
            ud=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']")
            if ud is None:raise ValueError('no measured via source')
            source_info={'source_lane_ids':[ud.get('value')],'raw_via_preserved':True}
            if mouth_retreat or composite_support:raw,source_info=composite_sources(root,road,src,lambda x:_project(x,lat,lon))
            else:raw=raw_curves(src,ud.get('value'),lambda x:_project(x,lat,lon))
            via=raw_curves(src,ud.get('value'),lambda x:_project(x,lat,lon)) if mouth_retreat or composite_support else None
            candidate,row=fit(road,*frames(root,road),raw,optimize=optimize,adaptive_caps=adaptive_caps,source_warm_start=source_warm_start,raw_via=via)
            row['source_composite']=source_info
            sub=ET.Element('OpenDRIVE');sub.append(copy.deepcopy(candidate));row['written_speed_dynamics_2cm']=_audit_d(sub,cfg)
            # Evaluate the existing profile's 15km/h fallback as an additional
            # design check, since old generated connectors can contain lower,
            # geometry-derived speeds. Do NOT alter the file's speed fields.
            for speed in sub.findall('.//lane/speed'):
                speed.set('max',str(max(15/3.6,float(speed.get('max')))))
            row['design_15kmh_dynamics_2cm']=_audit_d(sub,cfg)
            if row['design_15kmh_dynamics_2cm']['status']!='PASS':row['status']='REJECTED'
            # Keep trial geometry for diagnosis, conspicuously marked blocked.
            replace_road(root,road,candidate)
        except (ValueError,KeyError,np.linalg.LinAlgError) as e:
            row={'road':road.get('id'),'status':'REJECTED','reason':str(e)}
        rows.append(row);print('ROAD',road.get('id'),row['status'],row.get('source',row.get('reason')),flush=True)
        target.parent.mkdir(parents=True,exist_ok=True)
        target.with_suffix('.progress.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    xsd=write_trial(tree,target)
    report={'candidate_only':True,'status':'BLOCKED','source':str(source),'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
        'roads':rows,'edges':edge_audit(root),'centers':junction_lane_interfaces(root),
        'ordinary_roads_unchanged':not bool(mouth_retreat),'structural_mouth_retreat_m':mouth_retreat,
        'source_support_mode':'source-linked-composite' if mouth_retreat or composite_support else 'raw-via',
        'ordinary_geometry_restricted_not_refitted':bool(mouth_retreat),'selected_measured_connectors_only':True,
        'source_manifest_not_promoted':True,'xsd':xsd}
    report['g11']=audit_file(target,ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    target.with_suffix('.ribbon.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path)
    p.add_argument('--road',action='append',required=True);p.add_argument('--no-optimize',action='store_true')
    p.add_argument('--mouth-retreat',type=float,default=0.)
    p.add_argument('--adaptive-caps',action='store_true',help='omit caps only at source-constrained parallel mouths')
    p.add_argument('--composite-support',action='store_true',help='read complete source chains when input mouths are already relocated')
    p.add_argument('--source-warm-start',action='store_true',help='source-guided initialization; final hard constraints unchanged')
    a=p.parse_args();r=run(a.source,a.target,a.road,optimize=not a.no_optimize,mouth_retreat=a.mouth_retreat,adaptive_caps=a.adaptive_caps,composite_support=a.composite_support,source_warm_start=a.source_warm_start)
    print('FINAL BLOCKED',r['edges']['status'],r['g11']['status'])
    raise SystemExit(2)
