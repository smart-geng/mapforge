"""Candidate only: few-knot source-boundary families, not independent widths.

No source mutation, speed change, planView change, or new laneSection. Cubic
B-spline coefficients are solved jointly with exact end jets and non-negative
Bernstein width controls. Source errors are reported independently of provenance.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import minimize, linprog
from scipy.spatial import cKDTree
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import shp_source
from mapforge.ops.lane_family_border import _linked_chains, _materialize_widths
from mapforge.validate.smoothness import lane_edges_kinematics_at, sample_road_ref
from mapforge.validate.smoothness import _ref_kappa_at, _offsets
from mapforge.validate.shp_boundary_fidelity import _origin, _project, _densify
from mapforge.validate.g11 import audit_file as g11_audit


def solve_side(road, side, src, project, knot_span=20.0, smooth_weight=1e7,
               boundary_lane=None, source_tolerance=.25, kinematic_constraints=False):
    secs=road.findall('lanes/laneSection')
    starts=np.array([float(s.get('s')) for s in secs])
    length=float(road.get('length'))
    ends=np.r_[starts[1:],length]
    occurrences, successors, chains=_linked_chains(secs,side)
    if not chains: return {'status':'EMPTY'}
    sign=1 if side=='left' else -1
    ids={si:[l.get('id') for l in sorted(sec.findall(f'{side}/lane'),
                                       key=lambda l:abs(int(l.get('id'))))]
         for si,sec in enumerate(secs)}
    xy,ss,hh=sample_road_ref(road,.05)
    tree=cKDTree(xy)
    def st(raw):
        p=_densify(project(raw),.75)
        _,ix=tree.query(p)
        d=p-xy[ix]; h=hh[ix]
        return np.c_[ss[ix]+d[:,0]*np.cos(h)+d[:,1]*np.sin(h),
                     -d[:,0]*np.sin(h)+d[:,1]*np.cos(h)]

    family=[]; owner={}; nvar=0
    for chain in chains:
        a,b=starts[chain[0][0]],ends[chain[-1][0]]
        fixed=boundary_lane is not None and not any(key[1]==boundary_lane for key in chain)
        if b-a < 3*knot_span and not fixed:
            # Short true branches cannot hide extra degrees of freedom in tiny
            # knots. Reject this side for a dedicated topology/transition solve.
            return {'status':'REJECTED','reason':'short-boundary-chain',
                    's':[float(a),float(b)],'chain':chain}
        spans=max(3,int((b-a)/knot_span))
        internal=np.linspace(a,b,spans+1)[1:-1]
        knots=np.r_[np.repeat(a,4),internal,np.repeat(b,4)]
        n=len(knots)-4
        basis=BSpline(knots,np.eye(n),3,extrapolate=False)
        f={'chain':chain,'a':a,'b':b,'knots':knots,'basis':basis,
           'slice':slice(nvar,nvar+n),'fixed':fixed}
        if fixed:
            knots={float(a),float(b)}
            for si,lid in chain:
                knots.add(float(starts[si])); knots.add(float(ends[si]))
                for lane in secs[si].findall(f'{side}/lane'):
                    knots.update(starts[si]+float(w.get('sOffset')) for w in lane.findall('width'))
            knots.update(float(o.get('s')) for o in road.findall('lanes/laneOffset') if a<float(o.get('s'))<b)
            f['knots']=np.array(sorted(knots))
        for key in chain: owner[key]=len(family)
        family.append(f)
        if not fixed: nvar+=n
    nvar+=1   # Homogeneous coordinate for unchanged physical boundaries.

    def expr(fi,s,der=0):
        row=np.zeros(nvar)
        f=family[fi]
        if f['fixed']:
            key=next((k for k in f['chain'] if starts[k[0]]<=s<ends[k[0]]),f['chain'][-1])
            row[-1]=old_edges(key[0],s)[ids[key[0]].index(key[1])+1][der]
        else:
            row[f['slice']]=f['basis'](float(s),nu=der)
        return row
    def old_edges(si,s):
        return lane_edges_kinematics_at(road,min(max(s,starts[si]+1e-7),ends[si]-1e-7),side)
    def inner(key,s,der=0):
        si,lid=key; i=ids[si].index(lid)
        if i:
            return expr(owner[(si,ids[si][i-1])],s,der),0.
        if der==3:
            active=max((o for o in _offsets(road) if o[0]<=s+1e-9),key=lambda o:o[0])
            return np.zeros(nvar),6.*active[4]
        return np.zeros(nvar),old_edges(si,s)[0][der]

    A=[]; y=[]; equality=[]; rhs=[]; inequalities=[]; lower=[]; labels=[]
    source_evidence=[]
    def observe(row,value,weight):
        A.append(row*weight); y.append(value*weight)
    def eq(row,value): equality.append(row); rhs.append(value)
    constant=np.zeros(nvar); constant[-1]=1; eq(constant,1.)
    for fi,f in enumerate(family):
        if f['fixed']:
            # A frozen outer taper must still meet a moving inner boundary
            # at its birth/death. Nonnegative width alone would leave a step.
            for key,s in ((f['chain'][0],f['a']),(f['chain'][-1],f['b'])):
                si,lid=key; i=ids[si].index(lid); jets=old_edges(si,s)
                if 1e-5<s<length-1e-5 and sign*(jets[i+1][0]-jets[i][0])<.01:
                    for der,scale in enumerate((1.,20.,400.)):
                        ir,ic=inner(key,s,der)
                        eq(scale*(expr(fi,s,der)-ir),scale*ic)
            continue
        a,b=f['a'],f['b']
        # Weak prior only in source-free support. Raw SHP controls the shape
        # wherever the same-identity physical boundary exists.
        evidence={}
        for key in f['chain']:
            si,lid=key; i=ids[si].index(lid)
            lo,hi=starts[si],ends[si]
            lane=occurrences[key]
            ud=lane.find("userData[@code='mapforge.source_lane']")
            sid=ud.get('value') if ud is not None else None
            if sid and sid not in evidence:
                evidence[sid]=[st(g) for g in src.lane_boundary_geometries(sid)]
            choices=[]
            for g in evidence.get(sid,[]):
                g=g[(g[:,0]>=lo-1)&(g[:,0]<=hi+1)]
                if len(g)<2: continue
                g=g[np.argsort(g[:,0])]
                score=np.median([abs(t-old_edges(si,float(np.clip(s,lo,hi)))[i+1][0])
                                 for s,t in g])
                choices.append((score,g))
            selected=min(choices,key=lambda pair:pair[0])[1] if choices else None
            for s in np.linspace(lo+1e-6,hi-1e-6,max(3,int((hi-lo)/1.5)+1)):
                raw=old_edges(si,s)[i+1][0]
                value,weight=raw,.25
                if selected is not None and selected[0,0]<=s<=selected[-1,0]:
                    value=float(np.interp(s,selected[:,0],selected[:,1])); weight=1.
                    source_evidence.append((fi,float(s),value,float(raw),sid))
                    # Do not buy fairing with a worse source reconstruction.
                    # Existing source discrepancies remain visible in the
                    # report, never a blanket exemption for an entire lane.
                    tube=max(source_tolerance,abs(raw-value)+.005)
                    row=expr(fi,s)
                    inequalities.extend([row,-row])
                    lower.extend([value-tube,-value-tube])
                    labels.extend([{'kind':'source','source':sid,'s':float(s),'boundary':fi}]*2)
                observe(expr(fi,s),value,weight)
        # Fairing integral is independent of SHP sample density and laneSection
        # lengths. It regularizes curvature variation, not point interpolation.
        for s in np.linspace(a,b,max(3,int((b-a)/2)+1)):
            observe(expr(fi,s,3),0.,np.sqrt(smooth_weight*2.))
        for key,s,end in ((f['chain'][0],a,0),(f['chain'][-1],b,1)):
            si,lid=key; i=ids[si].index(lid)
            jets=old_edges(si,s)
            width=sign*(jets[i+1][0]-jets[i][0])
            # Real birth/death: one physical boundary bifurcates, so it shares
            # position/tangent/curvature with the inner boundary. Ordinary road
            # endpoints retain all three jets used to solve junction curves.
            branch= (s>1e-5 and s<length-1e-5 and width<.01)
            for der,scale in enumerate((1.,20.,400.)):
                # A cropped, unlinked far end has no consumer seam to preserve.
                # Its shape comes from the source tube/prior, not the old
                # converter's displaced position or arbitrary derivatives.
                if s<=1e-5 and road.find('link/predecessor') is None:
                    continue
                row=expr(fi,s,der)
                if branch:
                    ir,ic=inner(key,s,der); eq(scale*(row-ir),scale*ic)
                else:
                    eq(scale*row,scale*jets[i+1][der])

    # Bound every width over its entire cubic domain with Bernstein controls.
    # A non-negative set of controls guarantees no boundary crossing between
    # samples. This adds inequalities, never extra geometric knots.
    for key,lane in occurrences.items():
        si,lid=key; fi=owner[key]
        i=ids[si].index(lid)
        inner_fixed=i==0 or family[owner[(si,ids[si][i-1])]]['fixed']
        if family[fi]['fixed'] and inner_fixed: continue
        knots={float(starts[si]),float(ends[si])}
        knots.update(float(s) for s in family[fi]['knots'] if starts[si]<s<ends[si])
        i=ids[si].index(lid)
        if i:
            fj=owner[(si,ids[si][i-1])]
            knots.update(float(s) for s in family[fj]['knots'] if starts[si]<s<ends[si])
        else:
            knots.update(float(o.get('s')) for o in road.findall('lanes/laneOffset')
                         if starts[si]<float(o.get('s'))<ends[si])
        knots=sorted(knots)
        for a,b in zip(knots,knots[1:]):
            rows=[]; const=[]
            for s in (a,b):
                for der in (0,1):
                    ir,ic=inner(key,s,der)
                    rows.append(sign*(expr(fi,s,der)-ir)); const.append(-sign*ic)
            r0,d0,r1,d1=rows; c0,e0,c1,e1=const; h=(b-a)/3.
            for row,c in ((r0,c0),(r0+h*d0,c0+h*e0),
                          (r1-h*d1,c1-h*e1),(r1,c1)):
                inequalities.append(row); lower.append(-c)
                labels.append({'kind':'width','lane':lid,'s':[float(a),float(b)]})
    if kinematic_constraints:
        if any(f['fixed'] for f in family):
            raise ValueError('kinematic proxy needs a complete solved side')
        for key,lane in occurrences.items():
            if lane.get('type')!='driving': continue
            speeds=lane.findall('speed')
            speed=float(speeds[0].get('max')) if speeds else 60/3.6
            if speeds and speeds[0].get('unit')=='km/h': speed/=3.6
            si,lid=key; fi=owner[key]
            grid=np.arange(starts[si]+1e-5,ends[si],1.)
            knots=[float(s) for f in family for s in f['knots']]
            knots.extend(float(g.get('s')) for g in road.findall('planView/geometry'))
            grid=np.unique(np.r_[grid,[s+e for s in knots for e in (-1e-5,1e-5)
                                      if starts[si]+1e-6<s+e<ends[si]-1e-6]])
            for s in grid:
                for der,reference,budget in ((2,_ref_kappa_at(road,float(s))[0],2./speed**2),
                                              (3,_ref_kappa_at(road,float(s))[1],.8/speed**3)):
                    ir,ic=inner(key,float(s),der)
                    row=.5*(expr(fi,s,der)+ir); constant=.5*ic+reference
                    inequalities.extend([row,-row]); lower.extend([-budget-constant,-budget+constant])
                    labels.extend([{'kind':'kinematic-proxy','lane':lid,'s':float(s),'order':der}]*2)
    A=np.array(A); y=np.array(y); E=np.array(equality); r=np.array(rhs)
    x0=np.linalg.lstsq(E,r,rcond=None)[0]
    eq_error=float(max(abs(E@x0-r)))
    if eq_error>1e-5: return {'status':'REJECTED','reason':'incompatible-end-jets','error':eq_error}
    Z=null_space(E)
    C=np.array(inequalities); lb=np.array(lower)
    AZ=A@Z; yy=y-A@x0; CZ=C@Z; clb=lb-C@x0
    H=AZ.T@AZ; g=AZ.T@yy
    z0=np.linalg.lstsq(AZ,yy,rcond=None)[0]
    if len(z0):
        feasible=linprog(np.zeros(len(z0)),A_ub=-CZ,b_ub=-clb,
                         bounds=[(None,None)]*len(z0),method='highs')
        if not feasible.success:
            phase=linprog(np.r_[np.zeros(len(z0)),1.],
                          A_ub=np.c_[-CZ,-np.ones(len(CZ))],b_ub=-clb,
                          bounds=[(None,None)]*len(z0)+[(0,None)],method='highs')
            return {'status':'REJECTED','reason':'infeasible-source-tube-or-end-jets',
                    'minimum_constraint_slack':float(phase.fun) if phase.success else None,
                    'slack_units':'mixed m, 1/m, 1/m^2' if kinematic_constraints else 'm',
                    'conflicts':([labels[i] for i in np.argsort(CZ@phase.x[:-1]-clb)[:10]]
                                 if phase.success else [])}
        z0=feasible.x
        scale=max(float(np.linalg.norm(H,2)),1.)
        ans=minimize(lambda z:(.5*z@H@z-g@z)/scale,z0,jac=lambda z:(H@z-g)/scale,
                     constraints=[{'type':'ineq','fun':lambda z:CZ@z-clb,
                                   'jac':lambda z:CZ}],method='SLSQP',
                     options={'ftol':1e-10,'maxiter':500})
        solution=x0+Z@ans.x
        success=ans.success; message=ans.message
    else:
        solution=x0; success=True; message='fully constrained'
    min_control=float(np.min(C@solution-lb))
    if not success or min_control < -1e-6:
        return {'status':'REJECTED','reason':'solver-or-width','message':str(message),
                'minimum_width_control':min_control}
    error=np.array([abs(expr(fi,s)@solution-v) for fi,s,v,_,_ in source_evidence])
    old_error=np.array([abs(old-v) for _,_,v,old,_ in source_evidence])
    report={'status':'CANDIDATE','production_promoted':False,
            'kinematic_proxy_constraints':kinematic_constraints,
            'variables':nvar,'free_variables':Z.shape[1],
            'source_tolerance_m':source_tolerance,
            'boundary_chains':sum(not f['fixed'] for f in family),'minimum_control_span_m':float(min(
                min(np.diff(np.unique(f['knots']))) for f in family if not f['fixed'])),
            'minimum_width_control':min_control,'source_sample_count':len(error),
            'source_p95_m':float(np.percentile(error,95)) if len(error) else None,
            'source_max_m':float(max(error)) if len(error) else None,
            'old_source_p95_m':float(np.percentile(old_error,95)) if len(error) else None,
            'old_source_max_m':float(max(old_error)) if len(error) else None}
    from mapforge.ops.lane_family_border import _write_source_shape_side
    if any(f['fixed'] for f in family):
        _write_source_shape_side(road,secs,starts,ends,side)
    for key,lane in occurrences.items():
        si,_lid=key; f=family[owner[key]]
        if f['fixed']: continue
        knots=sorted({float(starts[si]),float(ends[si]),*[float(s) for s in f['knots']
                        if starts[si]<s<ends[si]]})
        coeff=solution[f['slice']]; curve=BSpline(f['knots'],coeff,3)
        for old in list(lane.findall('width'))+list(lane.findall('border')): lane.remove(old)
        index=1 if lane.find('link') is not None else 0
        for s in knots[:-1]:
            values=[float(curve(s,nu=k))/[1,1,2,6][k] for k in range(4)]
            lane.insert(index,etree.Element('border',sOffset=f'{s-starts[si]:.12g}',
                **{k:f'{v:.14g}' for k,v in zip('abcd',values)})); index+=1
        meta=lane.find("userData[@code='mapforge.provenance/v1']")
        if meta is not None:
            provenance=json.loads(meta.get('value','{}'))
            provenance.update(status='APPROXIMATED',geometry_adjustment='source-boundary-joint-candidate')
            meta.set('value',json.dumps(provenance,ensure_ascii=False,separators=(',',':')))
    return report


def main():
    p=argparse.ArgumentParser(); p.add_argument('input',type=Path); p.add_argument('output',type=Path)
    p.add_argument('--road',action='append'); p.add_argument('--side',choices=['left','right'])
    p.add_argument('--boundary-lane',help='Solve this linked outer boundary only; preserve its neighbours exactly')
    p.add_argument('--source-tolerance',type=float,default=.25)
    p.add_argument('--kinematics',action='store_true')
    p.add_argument('--knot-span',type=float,default=20.); p.add_argument('--smooth-weight',type=float,default=1e7)
    args=p.parse_args(); tree=etree.parse(str(args.input)); root=tree.getroot()
    if args.input.resolve()==args.output.resolve():
        raise ValueError('candidate output must not overwrite its input')
    src=shp_source(); lat,lon=_origin(root)
    report={}
    for road in root.findall('road'):
        if road.get('junction')!='-1' or road.get('name')=='junction_paving': continue
        if args.road and road.get('id') not in args.road: continue
        candidate=copy.deepcopy(road); sides={}
        for side in ('left','right'):
            if args.side and args.side!=side:
                from mapforge.ops.lane_family_border import _write_source_shape_side
                secs=candidate.findall('lanes/laneSection'); starts=[float(s.get('s')) for s in secs]
                _write_source_shape_side(candidate,secs,starts,starts[1:]+[float(candidate.get('length'))],side)
                continue
            sides[side]=solve_side(candidate,side,src,lambda g:_project(g,lat,lon),args.knot_span,args.smooth_weight,args.boundary_lane,args.source_tolerance,args.kinematics)
        print(road.get('id'),json.dumps(sides),flush=True)
        report[road.get('id')]=sides
        if all(s['status'] in ('CANDIDATE','EMPTY') for s in sides.values()):
            secs=candidate.findall('lanes/laneSection'); starts=[float(s.get('s')) for s in secs]
            end=starts[1:]+[float(candidate.get('length'))]
            if _materialize_widths(candidate,secs,starts,end) is None: raise ValueError('negative materialized width')
            root.replace(road,candidate)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    tree.write(str(args.output),encoding='utf-8',xml_declaration=True,pretty_print=True)
    audit=g11_audit(args.output,ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    rejected=any(s['status']=='REJECTED' for sides in report.values() for s in sides.values())
    report['_metadata']={'input':str(args.input),'input_sha256':hashlib.sha256(args.input.read_bytes()).hexdigest(),
                         'candidate_only':True,'production_promoted':False,
                         'solver_rejected':rejected,'g11_status':audit['status']}
    args.output.with_suffix('.fit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    args.output.with_suffix('.g11.json').write_text(json.dumps(audit,indent=2),encoding='utf-8')
    print(audit['summary'],flush=True)
    return 2 if rejected else (0 if audit['status']=='PASS' else 1)


if __name__=='__main__': raise SystemExit(main())
