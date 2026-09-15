"""Continuous midpoint-check domains at source-proven ordinary contacts.

Oblique record cuts end the two boundary records at different stations.
Intersecting each record's left/right supports independently leaves a gap.
This module closes ONLY that gap, using original TOPO and physical boundary
continuations. It neither creates geometry nor guesses traffic lane links.
"""
import numpy as np


def source_transition_domains(model):
    intervals=[];events=[];excluded=[]
    endpoints=model.contacts['boundary_endpoints']
    physical={tuple(sorted(r['endpoints'])):r for r in model.physical_graph['relations']}
    for index,event in enumerate(model.contacts['transition_events']):
        fa,fb=event['from_source'],event['to_source']
        if fa not in model.lane_pairs or fb not in model.lane_pairs:continue
        if event['kind']!='ordinary_continuation':
            excluded.append(dict(event_index=index,kind=event['kind'],from_source=fa,to_source=fb,
                                 reason='taper contact is not an ordinary traffic continuation'))
            continue
        if event['status']!='SOURCE_C0_CONTACT_PROVEN' or len(event['contacts'])!=2:
            raise ValueError('ordinary source transition lacks two proven boundary contacts')
        sides={};stations=[]
        for contact in event['contacts']:
            side=contact['from_side']
            if side not in ('left','right') or side!=contact['to_side'] or side in sides:
                raise ValueError('ordinary source transition has ambiguous boundary sides')
            ids=[contact['from_endpoint'],contact['to_endpoint']]
            relation=physical.get(tuple(sorted(ids)))
            if relation is None or not relation['ordinary_lane_link_supported']:
                raise ValueError('ordinary boundary relation is not source-proven')
            keys=[];ss=[]
            for identifier in ids:
                tip=endpoints[identifier];key=tip['feature']
                if key not in model.owner or tip['part_index']!=0:
                    raise ValueError('transition endpoint outside source model')
                order=model.source_vertex_indices[key]
                if tip['vertex_index'] not in order:raise ValueError('source vertex index missing')
                station=float(model.raw[key][order.index(tip['vertex_index']),0])
                if abs(model.contact_station(tip['xy'])-station)>1e-7:
                    raise ValueError('transition station differs from original endpoint')
                keys.append(key);ss.append(station)
            if abs(ss[0]-ss[1])>1e-7:raise ValueError('noncoincident boundary transition requires separate model')
            if keys[0]!=model.lane_pairs[fa][0 if side=='left' else 1] or keys[1]!=model.lane_pairs[fb][0 if side=='left' else 1]:
                raise ValueError('transition boundary identity differs from source lane sides')
            sides[side]=keys;stations.extend(ss)
        a,b=float(min(stations)),float(max(stations))
        # Already-coincident transverse cuts have no omitted positive interval.
        if b-a<=1e-8:
            events.append(dict(event_index=index,from_source=fa,to_source=fb,band_m=[a,b],added_length_m=0.))
            continue
        speeds=[model.scope['observations'][sid].get('source_max_speed_kmh') for sid in (fa,fb)]
        if not all(isinstance(v,(float,int)) and np.isfinite(v) and v>0 for v in speeds):
            raise ValueError('source transition requires explicit source speeds')
        speed_boundary=None
        if abs(speeds[0]-speeds[1])>1e-9:
            from mapforge.ops.source_speed_events import speed_event
            speed_boundary=speed_event(model,event)
        pair_a,pair_b=model.lane_pairs[fa],model.lane_pairs[fb]
        if pair_a[4]!=pair_b[4]:raise ValueError('ordinary source transition reverses physical lane sides')
        # Only uncovered parts of the contact band are added. Source record
        # domains retain their original identities and their unchanged checks.
        cuts=np.unique(np.r_[a,b,stations,[s for p in (pair_a,pair_b) for s in p[2:4] if a<s<b]])
        if speed_boundary is not None and a<speed_boundary['station_m']<b:
            cuts=np.unique(np.r_[cuts,speed_boundary['station_m']])
        added=0.;event_rows=[]
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            if hi-lo<=1e-8:continue
            mid=(lo+hi)/2
            if any(p[2]-1e-9<=mid<=p[3]+1e-9 for p in (pair_a,pair_b)):continue
            chosen=[]
            for side in ('left','right'):
                choices={model.owner[key]:key for key in sides[side]
                         if model.families[model.owner[key]].knots[0]-1e-8<=lo and
                            model.families[model.owner[key]].knots[-1]+1e-8>=hi}
                if len(choices)!=1:
                    raise ValueError('transition requires a unique source-supported physical boundary on each side')
                chosen.append(next(iter(choices.values())))
            speed=(speed_boundary['before_s_kmh' if mid<speed_boundary['station_m'] else 'after_s_kmh']
                   if speed_boundary is not None else float(speeds[0]))
            event_rows.append(dict(source_lane=fa,source_lanes=[fa,fb],left=chosen[0],right=chosen[1],
                                  a=float(lo),b=float(hi),sign=pair_a[4],source_speed_kmh=speed,
                                  event_index=index,topology_record=event['topology_record'],
                                  domain_kind='source_transition_band',geometry_added=False,lane_link_inferred=False))
            added+=hi-lo
        context=[dict(left=p[0],right=p[1],a=p[2],b=p[3]) for p in (pair_a,pair_b)]
        context.extend({k:r[k] for k in ('left','right','a','b')} for r in event_rows)
        context.sort(key=lambda r:r['a'])  # Sort proven source records, never raw vertices.
        if any(abs(a0['b']-b0['a'])>1e-7 for a0,b0 in zip(context[:-1],context[1:])):
            raise ValueError('ordinary transition context has an uncovered/overlapping source interval')
        for row in event_rows:row['context_domains']=context
        if speed_boundary is not None:
            for row in event_rows:row['source_speed_boundary']=speed_boundary
        intervals.extend(event_rows)
        events.append(dict(event_index=index,from_source=fa,to_source=fb,band_m=[a,b],added_length_m=float(added)))
    return dict(intervals=intervals,events=events,excluded_taper_events=excluded,
                scope='ordinary source contact bands only; not exterior ends, movement paths or all junctions',
                geometry_added=False,source_speed_changed=False,export_allowed=False)


def midpoint_domains(model):
    rows=[]
    for sid,(left,right,a,b,sign) in model.lane_pairs.items():
        speed=model.scope['observations'][sid].get('source_max_speed_kmh')
        if speed is None or not np.isfinite(speed) or speed<=0:raise ValueError('explicit positive source speed required')
        rows.append(dict(source_lane=sid,source_lanes=[sid],left=left,right=right,a=a,b=b,sign=sign,
                         source_speed_kmh=speed,domain_kind='source_record_common_support'))
    return rows+source_transition_domains(model)['intervals']


def family_source_values(model, feature, stations):
    """Evaluate original LINE-chart segments across proven same-edge records.

    No extrapolation, no nearest-other-lane matching and no vertex sorting.
    Multiple original records at a shared endpoint must agree.
    """
    if getattr(model,'reference_curvature',0.)!=0:raise ValueError('linear source interpolation requires a Line chart')
    ss=np.asarray(stations,float);values=np.full(ss.shape,np.nan)
    for key in model.families[model.owner[feature]].features:
        raw=model.raw[key];inside=(ss>=raw[0,0]-1e-9)&(ss<=raw[-1,0]+1e-9)
        yy=np.interp(ss[inside],raw[:,0],raw[:,1]);old=values[inside];overlap=np.isfinite(old)
        if np.any(abs(old[overlap]-yy[overlap])>1e-7):raise ValueError('source boundary continuation has conflicting overlap')
        values[inside]=yy
    if not np.isfinite(values).all():raise ValueError('source transition has unsupported boundary interval')
    return values


def transition_source_profile(model, domain, stations):
    """Whole connected source context bounds heading across the small gap.

    Bounding slope from only a 6cm gap and a 35cm tube would be almost useless.
    The proven C2 ordinary path includes BOTH adjacent original records; no
    arbitrary neighboring lane or unsupported extrapolation is used instead.
    """
    ss=np.asarray(stations,float);context=domain['context_domains']
    ss=np.unique(np.r_[ss[(ss>=context[0]['a'])&(ss<=context[-1]['b'])],
                       [d[k] for d in context for k in ('a','b')]])
    values=np.full(ss.shape,np.nan)
    for d in context:
        inside=(ss>=d['a']-1e-9)&(ss<=d['b']+1e-9)
        y=.5*(family_source_values(model,d['left'],ss[inside])+family_source_values(model,d['right'],ss[inside]))
        old=values[inside];overlap=np.isfinite(old)
        if np.any(abs(old[overlap]-y[overlap])>1e-7):raise ValueError('source midpoint context is not C0')
        values[inside]=y
    if not np.isfinite(values).all():raise ValueError('incomplete source midpoint context')
    return np.c_[ss,values]
