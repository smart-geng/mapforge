"""Explicit source-node speed events, independent of geometry knot spacing."""
import numpy as np


def speed_event(model, event):
    if getattr(model,'source_speed_boundary_policy',None) != 'original-lane-node':
        raise ValueError('mixed-speed transition needs a source-supported speed boundary; no guessed envelope')
    if (event['kind']!='ordinary_continuation' or event['status']!='SOURCE_C0_CONTACT_PROVEN'
            or event.get('from_source_contact')!='end' or event.get('to_source_contact')!='start'):
        raise ValueError('source speed event requires original directed end-to-start continuation')
    fa,fb=event['from_source'],event['to_source'];stations=[];nodes=[];speeds=[];points=[]
    for sid,role in ((fa,'end'),(fb,'start')):
        obs=model.scope['observations'][sid]
        rows=obs['raw_records'];layer=obs.get('identity_resolution',{}).get('selected_layer')
        rows=[r for r in rows if layer is None or r['layer']==layer]
        if len(rows)!=1 or len(rows[0]['parts'])!=1:raise ValueError('speed event source identity ambiguous')
        row=rows[0];fields=getattr(model,'source_speed_node_fields',{}).get(row['layer'],{})
        if role not in fields:raise ValueError('Profile source endpoint field mapping required')
        node=row['attributes'].get(fields[role])
        if node is None or not str(node).strip():raise ValueError('source speed event node missing')
        nodes.append(str(node));key='lane:'+sid
        index=len(row['parts'][0])-1 if role=='end' else 0
        point=model.raw[key][model.source_vertex_indices[key].index(index)]
        points.append(point);stations.append(float(point[0]))
        speed=obs.get('source_max_speed_kmh')
        if speed is None or not np.isfinite(speed) or speed<=0:raise ValueError('explicit positive source speed required')
        speeds.append(float(speed))
    if (nodes[0]!=nodes[1] or nodes[0]!=event.get('source_lane_contact_node_id')
            or np.linalg.norm(points[0]-points[1])>1e-7):
        raise ValueError('source speed breakpoint is not one proven original node')
    directions={o['travel_direction'] for o in model.scope['occurrences']
                if o['road']==model.road and o['source_lane_id'] in (fa,fb)}
    if len(directions)!=1 or not directions<={'with_s','against_s'}:raise ValueError('source speed direction ambiguous')
    forward=next(iter(directions))=='with_s'
    return dict(station_m=stations[0],source_node=nodes[0],from_source=fa,to_source=fb,
                before_s_kmh=speeds[0 if forward else 1],after_s_kmh=speeds[1 if forward else 0],
                source_changed=False,geometry_knots_added=0)


def chain_speed_events(model, ids):
    ids=set(ids);events=[]
    for event in model.contacts['transition_events']:
        a,b=event['from_source'],event['to_source']
        if a in ids and b in ids and event['kind']=='ordinary_continuation':
            if model.scope['observations'][a]['source_max_speed_kmh']!=model.scope['observations'][b]['source_max_speed_kmh']:
                events.append(speed_event(model,event))
    events.sort(key=lambda e:e['station_m'])
    if not events or any(abs(a['station_m']-b['station_m'])<1e-8 or a['after_s_kmh']!=b['before_s_kmh']
                         for a,b in zip(events[:-1],events[1:])):
        raise ValueError('source speed chain has missing or contradictory events')
    return events


def speed_records(chain, start, end):
    """Emit lane.speed events; never add laneSections/width/reference records."""
    events=chain.get('speed_events',[])
    current=events[0]['before_s_kmh'] if events else chain['speed_kmh']
    for event in events:
        if event['station_m']<=start+1e-8:current=event['after_s_kmh']
    rows=[(0.,current)]
    rows.extend((e['station_m']-start,e['after_s_kmh']) for e in events if start+1e-8<e['station_m']<end-1e-8)
    return rows
