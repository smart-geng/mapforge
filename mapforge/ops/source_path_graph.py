"""Source-proven ordinary lane paths, retaining every original center vertex.

Traffic direction and chart order are recorded separately. No nearest-lane
matching, no deleted vertices, no zero-width movement link promoted to ordinary.
"""
import numpy as np


def ordinary_source_paths(model):
    if getattr(model,'reference_curvature',0.)!=0:
        raise ValueError('source-only path check requires an orthonormal Line chart')
    ids=set(model.source_ids);next_lane={};previous={};excluded=[]
    for index,event in enumerate(model.contacts['transition_events']):
        a,b=event['from_source'],event['to_source']
        if a not in ids or b not in ids:continue
        if event['kind']!='ordinary_continuation':
            excluded.append(dict(event_index=index,from_source=a,to_source=b,kind=event['kind']))
            continue
        if event['status']!='SOURCE_C0_CONTACT_PROVEN':raise ValueError('unproven ordinary path contact')
        if (a in next_lane and next_lane[a]!=b) or (b in previous and previous[b]!=a):
            raise ValueError('ordinary path branches; no arbitrary chain selection')
        next_lane[a]=b;previous[b]=a
    rows=[];visited=set()
    for start in sorted(ids-set(previous)):
        path=[];sid=start
        while sid is not None:
            if sid in visited:raise ValueError('ordinary source chain contains a cycle or duplicate')
            visited.add(sid);path.append(sid);sid=next_lane.get(sid)
        roles=[model.roles['feature_roles']['lane:'+s]['role'] for s in path]
        if any(r!='physical_lane_center_observation' for r in roles):
            rows.append(dict(source_lanes_in_traffic_order=path,status='SEPARATE_MOVEMENT_CHECK_REQUIRED',roles=roles,
                             source_vertices_removed=0,export_allowed=False))
            continue
        speeds=[model.scope['observations'][s].get('source_max_speed_kmh') for s in path]
        if not all(isinstance(v,(float,int)) and np.isfinite(v) and v>0 for v in speeds):
            raise ValueError('path requires explicit positive source speed')
        if np.ptp(speeds)>1e-9:raise ValueError('mixed source speed path needs piecewise contract')
        order=path if len(path)<2 or model.raw['lane:'+path[0]][0,0]<model.raw['lane:'+path[-1]][0,0] else path[::-1]
        points=[];identities=[];contacts=[]
        for s in order:
            raw=model.raw['lane:'+s];original=model.source_vertex_indices['lane:'+s]
            if np.any(np.diff(raw[:,0])<=0):raise ValueError('non-monotone source center; no sorting')
            if points:
                gap=float(np.linalg.norm(np.asarray(points[-1])-raw[0]))
                if gap>1e-7:raise ValueError('source centers do not share an ordinary contact')
                contacts.append(dict(source_lanes=[identities[-1]['source_lane'],s],gap_m=gap,
                                     original_vertex_pair=[identities[-1]['vertex_index'],original[0]]))
            points.extend(raw.tolist())
            identities.extend(dict(source_lane=s,vertex_index=int(i)) for i in original)
        # Repeated SOURCE endpoints remain above. The necessary test can use
        # one numerical knot for coincident coordinates with explicit aliases.
        anchors=[];aliases=[]
        for point,identity in zip(points,identities):
            if anchors and abs(point[0]-anchors[-1][0])<=1e-7:
                if np.linalg.norm(np.asarray(point)-anchors[-1])>1e-7:raise ValueError('ambiguous coincident center station')
                aliases[-1].append(identity)
            else:
                if anchors and point[0]<=anchors[-1][0]:raise ValueError('source chain reverses chart station')
                anchors.append(point);aliases.append([identity])
        rows.append(dict(status='SOURCE_PATH_PREPARED_NOT_GEOMETRY',source_lanes_in_traffic_order=path,
                         source_lanes_in_chart_order=order,raw_points_st=points,raw_point_identities=identities,
                         necessary_anchors_st=anchors,anchor_source_aliases=aliases,contacts=contacts,
                         source_speed_kmh=float(speeds[0]),source_vertices_removed=0,export_allowed=False))
    if visited!=ids:raise ValueError('source lane inventory contains a closed cycle or missing chain')
    return dict(paths=rows,excluded_nonordinary_links=excluded,source_lane_count=len(ids),
                source_topology_changed=False,geometry_generated=False,export_allowed=False)
