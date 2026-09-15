"""Full source-center observation support, without boundary extrapolation.

Ordinary TOPO permits reuse of the SAME physical path across record cuts.
Exterior source tails may map to an existing path endpoint only within the
same Euclidean error budget. Certificate cells never add geometry variables.
"""
import numpy as np

from mapforge.ops.source_transition_domains import midpoint_domains


def frame(model,s):
    if hasattr(model,'axis'):
        return model.axis.frame(s)
    e=np.asarray(model.chart['tangent']);n=np.array([-e[1],e[0]])
    return np.asarray(model.chart['origin'])+float(s)*e,e,n


def source_slice(model,feature,a,b):
    raw=model.raw[feature]
    if a<raw[0,0]-1e-8 or b>raw[-1,0]+1e-8 or b<a:
        raise ValueError('source center slice outside ORIGINAL support')
    ss=np.unique(np.r_[a,raw[(raw[:,0]>a)&(raw[:,0]<b),0],b])
    if hasattr(model,'traces'):
        trace=model.traces[feature];yy=[]
        for s in ss:
            i=min(max(np.searchsorted(raw[:,0],s,side='right')-1,0),len(raw)-2)
            yy.append(float(trace.values(i,s)))
    else:yy=np.interp(ss,raw[:,0],raw[:,1])
    return np.c_[ss,yy]


def source_xy(model,feature,s):
    t=float(source_slice(model,feature,s,s)[0,1]);p,_,n=frame(model,s)
    return p+t*n


def _ordinary_groups(model):
    ids=set(model.source_ids);next_lane={};prev={}
    for e in model.contacts['transition_events']:
        a,b=e['from_source'],e['to_source']
        if a not in ids or b not in ids or e['kind']!='ordinary_continuation':continue
        if e['status']!='SOURCE_C0_CONTACT_PROVEN':raise ValueError('unproven center support continuation')
        if (a in next_lane and next_lane[a]!=b) or (b in prev and prev[b]!=a):
            raise ValueError('ambiguous center path continuation')
        next_lane[a]=b;prev[b]=a
    groups={};visited=set()
    for start in sorted(ids-set(prev)):
        chain=[];sid=start
        while sid is not None:
            if sid in visited:raise ValueError('cyclic center continuation')
            visited.add(sid);chain.append(sid);sid=next_lane.get(sid)
        for sid in chain:groups[sid]=chain
    if visited!=ids:raise ValueError('cyclic or missing source center inventory')
    return groups


def center_support_plan(model):
    # This also checks original side identities/contact records and speed;
    # do not replace it with nearest-road matching or an arbitrary family.
    domains=midpoint_domains(model);groups=_ordinary_groups(model)
    pieces=[];caps=[];unresolved=[];separate=[]
    for sid in model.source_ids:
        feature='lane:'+sid;raw=model.raw[feature]
        if model.roles['feature_roles'][feature]['role']=='movement_path_observation':
            separate.append(sid);continue
        hosts=[d for d in domains if all(v in groups[sid] for v in d['source_lanes'])]
        if not hosts:raise ValueError('source center has no identity-proven path')
        hosts.sort(key=lambda d:(d['a'],d['b'],d['source_lane']))
        first=min(hosts,key=lambda d:d['a']);last=max(hosts,key=lambda d:d['b'])
        a,b=raw[[0,-1],0]
        cuts=np.unique(np.r_[raw[:,0],[s for d in hosts for s in (d['a'],d['b']) if a<s<b]])
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            base=dict(source_lane=sid,feature=feature,a=float(lo),b=float(hi),
                      source_chain=groups[sid],geometry_added=False)
            eligible=[d for d in hosts if d['a']-1e-9<=lo and hi<=d['b']+1e-9]
            if eligible:
                signatures={(model.owner[d['left']],model.owner[d['right']],d['sign']) for d in eligible}
                if len(signatures)>1 and hi-lo>1e-8:
                    raise ValueError('overlapping nonidentical physical paths; no arbitrary center selection')
                d=eligible[0]
                pieces.append(dict(base,left=d['left'],right=d['right'],host_source_lanes=d['source_lanes'],
                                   kind='proven_path_interval',host_domain_kind=d['domain_kind']))
                continue
            contact=None
            if hi<=first['a']+1e-9:d=first;s=d['a'];side='start'
            elif lo>=last['b']-1e-9:d=last;s=d['b'];side='end'
            else:
                before=[d for d in hosts if abs(d['b']-lo)<=1e-9]
                after=[d for d in hosts if abs(d['a']-hi)<=1e-9]
                if hi-lo<=1e-8:
                    for da in before:
                        for db in after:
                            pair=set(da['source_lanes']+db['source_lanes'])
                            for event in model.contacts['transition_events']:
                                if (event['kind']=='ordinary_continuation' and event['status']=='SOURCE_C0_CONTACT_PROVEN'
                                    and pair=={event['from_source'],event['to_source']}):
                                    contact=event;d=da;s=da['b'];side='contact'
                if contact is None:
                    unresolved.append(dict(base,reason='internal path support gap; extrapolation forbidden'))
                    continue
            p,e,n=frame(model,s);witnesses=[];unavailable=False
            # A straight source segment's distance to a FIXED candidate
            # endpoint is convex: its two ends certify the whole segment.
            for station in source_slice(model,feature,lo,hi)[:,0]:
                xy=source_xy(model,feature,station);delta=xy-p
                longitudinal=float(delta@e);target=float(delta@n)
                budget=model.source_tol**2-longitudinal**2
                if budget<0:unavailable=True
                witnesses.append(dict(source_station=float(station),source_xy=xy.tolist(),
                    longitudinal_m=longitudinal,transverse_target_m=target,
                    transverse_allowance_m=float(np.sqrt(max(0.,budget)))))
            cap=dict(base,left=d['left'],right=d['right'],host_source_lanes=d['source_lanes'],
                     kind='numerical_contact_cap' if contact else 'euclidean_endpoint_cap',
                     side=side,host_station=float(s),witnesses=witnesses,
                     original_contact_record=contact['topology_record'] if contact else None,
                     total_tolerance_m=model.source_tol,boundary_extrapolated=False)
            if unavailable:unresolved.append(dict(cap,reason='chosen endpoint cannot cover tail within total source budget'))
            else:caps.append(cap)
    return dict(pieces=pieces,endpoint_caps=caps,unresolved=unresolved,separate_movement_observations=separate,
                source_lane_inventory=list(model.source_ids),raw_vertices_removed=0,geometry_added=False,
                meaning='source-to-physical-center observation coverage; not reverse/full-map acceptance',export_allowed=False)


def audit_center_support(model,x,plan):
    x=np.asarray(x,float)
    if x.shape!=(model.nvar,) or not np.isfinite(x).all():raise ValueError('finite shared coefficients required')
    rows=[]
    for d in plan['pieces']:
        left,right=d['left'],d['right']
        def curve(s,derivative=0):return .5*(model.expression(left,s,derivative)+model.expression(right,s,derivative))@x
        curve.t=np.unique(np.r_[model.families[model.owner[left]].knots,model.families[model.owner[right]].knots])
        curve.k=model.degree
        raw=source_slice(model,d['feature'],d['a'],d['b'])
        maximum=model.source_error(curve,d['feature'],raw)
        if not np.isfinite(maximum):raise ValueError('nonfinite full source center error')
        rows.append(dict(source_lane=d['source_lane'],kind=d['kind'],a=d['a'],b=d['b'],
                         conservative_error_m=float(maximum)))
    for d in plan['endpoint_caps']:
        s=d['host_station'];p,_,n=frame(model,s)
        t=.5*(model.expression(d['left'],s)+model.expression(d['right'],s))@x
        endpoint=p+t*n
        maximum=max(float(np.linalg.norm(np.asarray(w['source_xy'])-endpoint)) for w in d['witnesses'])
        rows.append(dict(source_lane=d['source_lane'],kind=d['kind'],a=d['a'],b=d['b'],
                         conservative_error_m=maximum,host_station=s,endpoint_xy=endpoint.tolist()))
    maximum=max((r['conservative_error_m'] for r in rows),default=None)
    return dict(rows=rows,maximum_conservative_error_m=maximum,unresolved=plan['unresolved'],
        physical_source_to_center_certified=not plan['unresolved'] and maximum is not None and maximum<=model.source_tol+1e-7,
        source_tolerance_m=model.source_tol,separate_movement_observations=plan['separate_movement_observations'],
        reverse_fidelity_certified=False,dynamics_certified=False,export_allowed=False)
