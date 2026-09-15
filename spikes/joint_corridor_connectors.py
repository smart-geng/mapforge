"""One shared state for whole MAP corridors and all connecting clothoids.

Fixed or continuously rotated Line charts and existing long boundary bases.
Source/width/structure are mandatory final constraints. Joint initialization
may search with temporary source residuals, but never returns a violating
state. All ordinary boundaries and every connection share one problem.
"""
import copy
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from lxml import etree
from pyclothoids import Clothoid, SolveG2
from scipy.linalg import block_diag

from spikes.road_boundary_family import solve_road, world_kinematics
from spikes.map_lane_family import ManifestCenters
from spikes.clarabel_joint_candidate import interior_qp
from spikes.connector_shape import turn_branch,heading_values,heading_envelope
from mapforge.validate.smoothness import lane_edges_at, _offsets, _poly_eval


def recover_parameters(model, *, seed_only=False):
    """Exact recovery by default; optional non-authoritative projected seed."""
    x=np.zeros(model.nvar)
    for fi,f in enumerate(model.families):
        rows=[];values=[]
        for si,lid in f.chain:
            a,b=model.starts[si],model.ends[si]
            for s in np.linspace(a+1e-7,b-1e-7,max(8,2*(f.columns.stop-f.columns.start))):
                rows.append(f.basis(s))
                if fi==0:value=_poly_eval(_offsets(model.original),s)
                else:
                    index=model.ids[(f.side,si)].index(lid)+1
                    value=lane_edges_at(model.original,s,f.side)[index]
                values.append(value)
        coef=np.linalg.lstsq(np.asarray(rows),values,rcond=None)[0]
        if not seed_only and np.max(abs(np.asarray(rows)@coef-values))>1e-7:
            raise ValueError('existing road is not representable in the same long basis')
        x[f.columns]=coef
    if seed_only and len(model.E):
        # Initialization only: enforce structural/width equalities, without
        # claiming that this projection preserves source geometry or dynamics.
        x -= np.linalg.lstsq(model.E, model.E@x-model.er, rcond=None)[0]
        if np.max(abs(model.E@x-model.er),initial=0.)>1e-7:
            raise ValueError('seed structural equalities are inconsistent')
    if seed_only:
        # Least-squares projection can make a newborn width negative. Admit
        # only the LINEAR structural polytope here, not fixed-axis source or
        # dynamics feasibility. The complete nonlinear search still follows.
        mask=np.array([l['kind'] not in ('source-center','whole-source-center') for l in model.labels])
        C,lo=model.C[mask],model.lower[mask]
        if np.max(lo-C@x,initial=0.)>1e-7:
            projected,info=interior_qp(np.eye(model.nvar),x,model.E,model.er,C,lo,x)
            if projected is None:raise ValueError('unqualified seed structural projection failed: '+str(info))
            x=projected
    return x


def center_row(model,lane_id,contact):
    si=0 if contact=='start' else len(model.starts)-1
    s=model.starts[si] if contact=='start' else model.ends[si]
    side='left' if lane_id>0 else 'right';lid=str(lane_id)
    rank=model.ids[(side,si)].index(lid)
    outer=model.owner[(side,si,lid)]
    inner=model.owner[(side,si,model.ids[(side,si)][rank-1])] if rank else 0
    row=np.zeros(model.nvar)
    for index in (inner,outer):
        f=model.families[index];row[f.columns]+=.5*f.basis(s)
    return row


def solve(root,manifest,*,target_ratio=.98,max_iterations=20,restore=False,rotating_axes=False,source_widths=None,
          joint_initialization=False):
    if not 0<target_ratio<1:raise ValueError('construction margin must be below unchanged acceptance limits')
    if joint_initialization and not (restore and rotating_axes):
        raise ValueError('joint initialization requires restoration and rotating axes')
    ordinary={r.get('id'):r for r in root.findall('road') if r.get('junction')=='-1'}
    if rotating_axes and any(r.find(tag) is not None for r in ordinary.values()
                             for tag in ('objects/object','signals/signal','signals/signalReference',
                                         'elevationProfile/elevation','lateralProfile/superelevation')):
        raise ValueError('rotating axes do not yet reproject road objects/signals/vertical geometry')
    models={};slices={};pieces=[];cursor=0
    for rid,road in ordinary.items():
        left=road.findall('lanes/laneSection/left/lane')
        prov=[l.find("userData[@code='mapforge.provenance/v1']") for l in left]
        mirror=(bool(left) and all(l.find("userData[@code='mapforge.source_lane']") is None for l in left)
                and all(p is not None and json.loads(p.get('value')).get('support_kind')
                        in ('mirror','lane-transition-ribbon') for p in prov))
        model=solve_road(etree.fromstring(ET.tostring(road)),ManifestCenters(manifest),np.asarray,
            source_mode='centers',source_error_budget='absolute',source_certificate=True,
            all_source_vertices=True,source_tol=.35,mirror=mirror,
            junction_endpoint_mode='source-parallel',model_only=True,source_widths=source_widths)
        if isinstance(model,dict):return None,dict(status='REJECTED',reason='ordinary model unavailable',road=rid,detail=model)
        models[rid]=model;slices[rid]=slice(cursor,cursor+model.nvar);cursor+=model.nvar
        pieces.append(recover_parameters(model,seed_only=joint_initialization))
    nroad=cursor;connections=[]
    ordinary_seed=np.concatenate(pieces)
    axes={rid:nroad+i for i,rid in enumerate(models)} if rotating_axes else {}
    cursor+=len(axes)
    if axes:pieces.append(np.zeros(len(axes)))
    if axes:
        from spikes.rotating_source_tube import RotatingSourceTube
        tubes={rid:RotatingSourceTube(m,ManifestCenters(manifest),source_widths=source_widths) for rid,m in models.items()}
    def port(road,lane,role):
        link=road.find('link/'+role);rid=link.get('elementId');cp=link.get('contactPoint')
        lid=int(lane.find('link/'+role).get('id'));parent=ordinary[rid]
        geoms=parent.findall('planView/geometry')
        if len(geoms)!=1 or geoms[0].find('line') is None:raise ValueError('joint port requires exact Line chart')
        g=geoms[0];h=float(g.get('hdg'));s=0. if cp=='start' else float(parent.get('length'))
        origin=np.array([float(g.get('x'))+s*math.cos(h),float(g.get('y'))+s*math.sin(h)])
        row=np.zeros(nroad);row[slices[rid]]=center_row(models[rid],lid,cp)
        forward=cp==('end' if role=='predecessor' else 'start')
        return dict(origin=origin,normal=np.array([-math.sin(h),math.cos(h)]),row=row,
                    heading=h if forward else h+math.pi,road=rid,lane=lid,contact=cp,
                    axis_heading=h,station=s,length=float(parent.get('length')))
    for road in root.findall('road'):
        if road.get('junction')=='-1' or road.get('name')=='junction_paving':continue
        lanes=road.findall('lanes/laneSection/right/lane')
        if len(lanes)!=1 or lanes[0].get('id')!='-1':raise ValueError('unsupported connecting cross section')
        if lanes[0].find('speed') is not None:raise ValueError('this inferred-MAP solver requires an absent source connector speed')
        a,b=port(road,lanes[0],'predecessor'),port(road,lanes[0],'successor')
        gs=road.findall('planView/geometry')
        if joint_initialization:
            pa=a['origin']+(a['row']@ordinary_seed)*a['normal']
            pb=b['origin']+(b['row']@ordinary_seed)*b['normal']
            cs=SolveG2(*pa,a['heading'],0.,*pb,b['heading'],0.)
            scale=sum(c.length for c in cs)
            if not np.isfinite(scale) or scale<=0:raise ValueError('invalid initial connection scale')
            # The old map can contain microscopic solver seed spans. They
            # are never materialized here: initialize three equal long spans
            # and let the ONE joint restoration close the actual endpoints.
            lengths=np.full(3,scale/3.)
            z=np.r_[lengths/scale,[c.KappaEnd*scale for c in cs[:2]]]
            seed_turn=sum(.5*(c.KappaStart+c.KappaEnd)*c.length for c in cs)
        else:
            if len(gs)!=3 or any(g.find('spiral') is None for g in gs):raise ValueError('exactly three initial clothoids required')
            if max(abs(float(gs[0].find('spiral').get('curvStart'))),abs(float(gs[-1].find('spiral').get('curvEnd'))))>1e-9:
                raise ValueError('joint model requires zero port curvature')
            lengths=np.array([float(g.get('length')) for g in gs]);scale=sum(lengths)
            z=np.r_[lengths/scale,[float(g.find('spiral').get('curvEnd'))*scale for g in gs[:2]]]
            seed_turn=sum(.5*(float(g.find('spiral').get('curvStart'))+
                                float(g.find('spiral').get('curvEnd')))*float(g.get('length')) for g in gs)
        turn=turn_branch(b['heading']-a['heading'],seed_turn)
        connections.append(dict(road=road.get('id'),a=a,b=b,scale=scale,
                                columns=slice(cursor,cursor+5),turn=turn))
        cursor+=5;pieces.append(z)
    state=np.concatenate(pieces);n=cursor
    E=block_diag(*[m.E for m in models.values()]);E=np.pad(E,((0,0),(0,n-nroad)))
    er=np.concatenate([m.er for m in models.values()])
    masks={rid:np.array([not axes or lab['kind'] not in ('source-center','whole-source-center') for lab in m.labels])
           for rid,m in models.items()}
    C=block_diag(*[m.C[masks[rid]] for rid,m in models.items()]);C=np.pad(C,((0,0),(0,n-nroad)))
    lo=np.concatenate([m.lower[masks[rid]] for rid,m in models.items()])
    A=block_diag(*[m.A for m in models.values()]);A=np.pad(A,((0,0),(0,n-nroad)))
    y=np.concatenate([m.target for m in models.values()])
    H=A.T@A;g=A.T@y
    factor=max(1.,float(np.trace(H))/max(1,nroad));H/=factor;g/=factor
    H+=np.eye(n)*1e-5;g+=state*1e-5
    # Connector curvature and sharpness limits are linear in (length,kappa).
    cap=target_ratio*2.5/(15/3.6)**2;rate=target_ratio/(15/3.6)**3
    extra=[];lower=[];elastic=[];elastic_roads=[]
    for c in connections:
        sl=c['columns'];scale=c['scale'];ll=np.zeros((3,n));kk=np.zeros((4,n))
        ll[:,sl.start:sl.start+3]=np.eye(3)*scale
        kk[1:3,sl.start+3:sl.stop]=np.eye(2)/scale
        extra.extend(ll);lower.extend([5.]*3)
        extra.extend(ll-.1*ll.sum(axis=0));lower.extend([0.]*3)
        extra.append(-ll.sum(axis=0));lower.append(-1.1*scale)
        start=len(extra)
        for row in kk[1:3]:extra.extend([row,-row]);lower.extend([-cap,-cap])
        dk=np.diff(kk,axis=0)
        extra.extend(rate*ll-dk);extra.extend(rate*ll+dk);lower.extend([0.]*6)
        elastic.extend(range(start,len(extra)))
        elastic_roads.extend([c['road']]*(len(extra)-start))
        if math.pi/6<abs(c['turn'])<5*math.pi/6:
            extra.extend(np.sign(c['turn'])*kk[1:3]);lower.extend([0.]*2)
    for index in axes.values():
        row=np.zeros(n);row[index]=1.
        extra.extend([row,-row]);lower.extend([-math.radians(3.)]*2)
    elastic=np.asarray(elastic,int)+len(C)
    C=np.vstack([C,np.asarray(extra)]);lo=np.r_[lo,lower]
    hard_mask=np.ones(len(C),bool);hard_mask[elastic]=False

    def port_state(port,x):
        delta=x[axes[port['road']]] if axes else 0.
        h=port['axis_heading']+delta;e=np.array([math.cos(h),math.sin(h)])
        normal=np.array([-e[1],e[0]])
        old_e=np.array([math.cos(port['axis_heading']),math.sin(port['axis_heading'])])
        anchor=port['origin']+(port['length']-port['station'])*old_e
        offset=port['row']@x[:nroad]
        p=anchor+(port['station']-port['length'])*e+offset*normal
        angle_derivative=(port['station']-port['length'])*normal-offset*e
        return p,port['heading']+delta,normal,angle_derivative

    def exact_source(x,with_jac=False):
        if not axes:return np.zeros(0),np.zeros((0,n)) if with_jac else None
        vs=[];js=[]
        for rid,tube in tubes.items():
            if with_jac:
                v,B,a=tube.evaluate(x[slices[rid]],x[axes[rid]],True)
                j=np.zeros((len(v),n));j[:,slices[rid]]=B;j[:,axes[rid]]=a;js.append(j)
            else:v=tube.evaluate(x[slices[rid]],x[axes[rid]])
            vs.extend(v)
        return np.asarray(vs),np.vstack(js) if with_jac else None

    def turn_at(c,x):
        return c['turn']+(x[axes[c['b']['road']]]-x[axes[c['a']['road']]] if axes else 0.)

    def local_end(z,c):
        lens=z[:3]*c['scale'];ks=np.r_[0.,z[3:]/c['scale'],0.]
        x=y=h=0.
        for length,a,b in zip(lens,ks,ks[1:]):
            cl=Clothoid.StandardParams(x,y,h,a,(b-a)/length,length)
            x,y,h=cl.XEnd,cl.YEnd,cl.ThetaEnd
        return np.array([x,y,h])
    def endpoint(x,with_jac=False):
        values=[];jac=[]
        for c in connections:
            sl=c['columns'];a,b=c['a'],c['b'];scale=c['scale']
            p,h,na,da=port_state(a,x);t,_,nb,db=port_state(b,x)
            R=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
            q=local_end(x[sl],c)
            values.extend(np.r_[(p+R@q[:2]-t)/scale,q[2]-turn_at(c,x)])
            if with_jac:
                J=np.zeros((3,n));J[:2,:nroad]=(na[:,None]*a['row']-nb[:,None]*b['row'])/scale
                if axes:
                    rotated=R@q[:2]
                    J[:2,axes[a['road']]]+=(da+np.array([-rotated[1],rotated[0]]))/scale
                    J[:2,axes[b['road']]]-=db/scale
                    J[2,axes[a['road']]]+=1.;J[2,axes[b['road']]]-=1.
                for j in range(5):
                    step=1e-5;u=x[sl].copy();u[j]+=step;v=x[sl].copy();v[j]-=step
                    d=(local_end(u,c)-local_end(v,c))/(2*step)
                    J[:,sl.start+j]=np.r_[R@d[:2]/scale,d[2]]
                jac.append(J)
        return np.asarray(values),np.vstack(jac) if with_jac else None
    def ordinary_dynamics(x,with_jac=False):
        values=[];jac=[]
        for rid,m in models.items():
            T=m.dynamics_rows;jet=T@x[slices[rid]];ref=m.reference;limits=m.dynamics_limits
            values.extend((world_kinematics(jet,ref[:,0],ref[:,1])/limits).ravel())
            if with_jac:
                derivatives=[]
                for d in range(4):
                    z=jet.astype(complex);z[:,d]+=1e-25j
                    derivatives.append(world_kinematics(z,ref[:,0],ref[:,1]).imag/1e-25)
                j=(np.einsum('dpc,pdn->pcn',np.asarray(derivatives),T)/limits[:,:,None]).reshape(-1,m.nvar)
                block=np.zeros((len(j),n));block[:,slices[rid]]=j;jac.append(block)
        return np.asarray(values),np.vstack(jac) if with_jac else None
    def heading_shape(x,with_jac=False):
        def values(z,c,turn):
            lengths=z[:3]*c['scale'];ks=np.r_[0.,z[3:]/c['scale'],0.]
            return heading_values(lengths,ks,turn)
        all_values=[];all_jac=[]
        for c in connections:
            sl=c['columns'];z=x[sl];turn=turn_at(c,x);all_values.extend(values(z,c,turn))
            if with_jac:
                J=np.zeros((6,n))
                for j in range(5):
                    a=z.copy();b=z.copy();a[j]+=1e-5;b[j]-=1e-5
                    J[:,sl.start+j]=(values(a,c,turn)-values(b,c,turn))/2e-5
                if axes:
                    dt=(values(z,c,turn+1e-6)-values(z,c,turn-1e-6))/2e-6
                    J[:,axes[c['a']['road']]]-=dt;J[:,axes[c['b']['road']]]+=dt
                all_jac.append(J)
        return np.asarray(all_values),np.vstack(all_jac) if with_jac else None
    def nonlinear_limits(x,with_jac=False):
        a,A=ordinary_dynamics(x,with_jac);b,B=heading_shape(x,with_jac)
        return np.r_[a,b],np.vstack([A,B]) if with_jac else None
    iterations=[];initial=state.copy();restoration_report=None
    for step in range(max_iterations):
        f,F=endpoint(state,True);v,J=nonlinear_limits(state,True)
        sv,SJ=exact_source(state,True)
        violation=max(float(np.max(abs(f))),float(max(0.,np.max(lo-C@state))),float(max(0.,np.max(abs(v))-1)),
                      float(np.max(sv,initial=0.)))
        print('JOINT',step,'residual',violation,flush=True)
        if step and violation<1e-7:break
        # Same shared coefficients must satisfy full-source tubes, width,
        # ordinary dynamics and every connector endpoint in one QP.
        trust=np.r_[np.full(nroad,.35),np.full(len(axes),.003),np.full(n-nroad-len(axes),.15)]
        CC=np.vstack([C,-SJ,J,-J,np.eye(n),-np.eye(n)])
        lower=np.r_[lo,-SJ@state+sv+(1e-6 if axes else 0.),J@state-v-1,-J@state+v-1,state-trust,-state-trust]
        EE=np.vstack([E,F]);rhs=np.r_[er,F@state-f]
        if joint_initialization and step==0:
            proposed,info=None,{'status':'joint feasibility initialization requested; no independent corridor admission'}
        else:
            proposed,info=interior_qp(H,g,EE,rhs,CC,lower,state)
        iterations.append(dict(iteration=step,previous_violation=violation,qp=info))
        if proposed is None:
            if restore and restoration_report is None:
                from spikes.nonlinear_feasibility import restore_joint
                def relaxed_values(x):
                    f,_=endpoint(x);v,_=nonlinear_limits(x)
                    source=exact_source(x)[0] if joint_initialization else np.zeros(0)
                    return np.r_[f,-f,v-1.,-v-1.,(lo[elastic]-C[elastic]@x)/cap,source]
                def relaxed_jac(x):
                    _,F=endpoint(x,True);_,J=nonlinear_limits(x,True)
                    source=exact_source(x,True)[1] if joint_initialization else np.zeros((0,n))
                    return np.vstack([F,-F,J,-J,-C[elastic]/cap,source])
                extra_args=({'hard_nonlinear':lambda x:exact_source(x)[0],
                             'hard_jacobian':lambda x:exact_source(x,True)[1]} if axes and not joint_initialization else {})
                restored,restoration_report=restore_joint(state,E,er,C[hard_mask],lo[hard_mask],
                    relaxed_values,relaxed_jac,H,g,radii=trust,**extra_args)
                restoration_report.update(joint_initialization=joint_initialization,
                    source_slack_search_only=joint_initialization,source_tolerance_unchanged=True)
                nf=3*len(connections)
                labels=[dict(label,road=rid,kind='ordinary-dynamics',component=component)
                    for rid,m in models.items() for label in m.dynamics_labels for component in ('curvature','sharpness')]
                labels += [dict(kind='heading-envelope',road=c['road'],extremum=i) for c in connections for i in range(6)]
                def describe(i):
                    if i<2*nf:
                        k=i%nf
                        return dict(kind='connector-endpoint',road=connections[k//3]['road'],component=('x/length','y/length','heading')[k%3])
                    i-=2*nf
                    if i<2*len(labels):return labels[i%len(labels)]
                    i-=2*len(labels)
                    if i<len(elastic_roads):return dict(kind='connector-dynamics',road=elastic_roads[i])
                    source_labels=[dict(label,road=rid) for rid,tube in tubes.items() for label in tube.labels]
                    return source_labels[i-len(elastic_roads)]
                restoration_report['worst_constraints']=[dict(describe(r['row']),violation=r['violation'])
                    for r in restoration_report.get('worst_positive_rows',[])]
                if restored is not None:
                    state=restored;continue
            return None,dict(status='REJECTED',reason='shared-state subproblem infeasible or numerical failure',
                             iterations=iterations,restoration=restoration_report,
                             joint_initialization=joint_initialization,
                             scope=('rotating charts' if axes else 'fixed charts')+' and fixed lane-section stations, not global impossibility')
        if axes:
            alpha=1.;direction=proposed-state
            while np.max(exact_source(proposed)[0],initial=0.)>1e-7 and alpha>1e-5:
                alpha*=.5;proposed=state+alpha*direction
            iterations[-1]['source_backtrack_alpha']=alpha
            if np.max(exact_source(proposed)[0],initial=0.)>1e-7:
                return None,dict(status='REJECTED',reason='rotating source hard envelope rejected step',iterations=iterations)
        state=proposed
    else:return None,dict(status='REJECTED',reason='shared-state SQP did not converge',iterations=iterations)
    # Recheck every hard constraint, not merely the QP status.
    if max(np.max(abs(E@state-er),initial=0.),np.max(lo-C@state,initial=0.),
           np.max(abs(endpoint(state)[0])),np.max(abs(nonlinear_limits(state)[0]))-1,
           np.max(exact_source(state)[0],initial=0.))>1e-7:
        return None,dict(status='REJECTED',reason='final exact shared-state check failed',iterations=iterations)
    out=copy.deepcopy(root)
    for rid,m in models.items():
        old=out.find(f"road[@id='{rid}']");new=ET.fromstring(etree.tostring(m.materialize(state[slices[rid]])))
        if axes:
            tube=tubes[rid];angle=state[axes[rid]];e=tube.tangent(angle)
            origin=tube.anchor-tube.length*e;geom=new.find('planView/geometry')
            geom.set('x',str(origin[0]));geom.set('y',str(origin[1]));geom.set('hdg',str(tube.heading+angle))
        for ud in new.findall(".//lane/userData[@code='mapforge.provenance/v1']"):
            prov=json.loads(ud.get('value'));prov.update(geometry_adjustment='simultaneous-corridor-connector-state',candidate_only=True)
            ud.set('value',json.dumps(prov,ensure_ascii=False,separators=(',',':')))
        index=list(out).index(old);out.remove(old);out.insert(index,new)
    curves={};port_changes=[]
    for c in connections:
        z=state[c['columns']];lens=z[:3]*c['scale'];ks=np.r_[0.,z[3:]/c['scale'],0.]
        a=c['a'];p,h,_,_=port_state(a,state);x,y=p;seq=[]
        for length,k0,k1 in zip(lens,ks,ks[1:]):
            cl=Clothoid.StandardParams(x,y,h,k0,(k1-k0)/length,length);seq.append(cl)
            x,y,h=cl.XEnd,cl.YEnd,cl.ThetaEnd
        curves[c['road']]=tuple(seq)
        for port in (c['a'],c['b']):
            port_changes.append(dict(road=port['road'],lane=port['lane'],contact=port['contact'],
                                     lateral_change_m=float(port['row']@(state[:nroad]-initial[:nroad]))))
    def source_bound(rid):
        tube=tubes[rid];v=tube.evaluate(state[slices[rid]],state[axes[rid]])
        keep=np.array([label['kind']!='explicit-source-width' for label in tube.labels])
        return .35*(np.max(v[keep])+1)
    source_errors=({rid:[source_bound(rid)] for rid in models}
                   if axes else {rid:[abs(row@state[slices[rid]]-value) for row,value,*_ in m.source_rows]
                   for rid,m in models.items()})
    return (out,curves),dict(status='CANDIDATE',method='simultaneous whole-corridor coefficients and all three-clothoid connectors',
                            variables=n,ordinary_variables=nroad,connections=len(connections),
                            target_ratio=target_ratio,iterations=iterations,port_changes=port_changes,
                            restoration=restoration_report,
                            joint_initialization=joint_initialization,
                            seed_only_projection=joint_initialization,
                            global_optimality_claimed=False,source_tolerance_m=.35,
                            fixed_reference_axes=not bool(axes),fixed_section_stations=True,
                            axis_angle_changes_deg={rid:float(np.degrees(state[i])) for rid,i in axes.items()},
                            source_metric='rotating whole-segment Bernstein/domain bound' if axes else 'source observations',
                            heading_policy={c['road']:heading_envelope(turn_at(c,state))[2] for c in connections},
                            final_source_observation_max_m={rid:float(max(v)) for rid,v in source_errors.items()},
                            final_scaled_linear_violation=float(max(0.,np.max(lo-C@state))),
                            final_scaled_endpoint_residual=float(np.max(abs(endpoint(state)[0]))),
                            final_ordinary_construction_ratio=float(np.max(abs(ordinary_dynamics(state)[0]))),
                            final_heading_envelope_ratio=float(np.max(abs(heading_shape(state)[0]))))
