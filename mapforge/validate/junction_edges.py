"""World edge contacts of explicit junction lane links (not just centerlines).

Initial diagnostic consumer: one laneSection connecting roads, contactPoint=start.
Unsupported topology is reported as unresolved, not skipped. Does not replace
an ASAM checker or certify compliance; exposes remaining skew-cap/edge-jet gaps.
"""
import math

from pyclothoids import Clothoid
from mapforge.validate.smoothness import (
    _geoms, _sections, lane_edges_kinematics_at, _edge_world_curvature,
)


def endpoint_edges(road, lane_id, contact, forward):
    if contact not in ('start','end') or not lane_id:
        raise ValueError('invalid edge endpoint')
    geoms=_geoms(road)
    if not geoms or len(geoms)!=len(road.findall('planView/geometry')):
        raise ValueError('unsupported reference geometry')
    _,x,y,h,length,k0,k1=geoms[0 if contact=='start' else -1]
    dk=(k1-k0)/length;k=k0
    if contact=='end':
        c=Clothoid.StandardParams(x,y,h,k0,dk,length)
        x,y,h,k=c.XEnd,c.YEnd,c.ThetaEnd,c.KappaEnd
    s=0. if contact=='start' else float(road.get('length'))
    sec=_sections(road)[0 if contact=='start' else -1]
    ids=[v[0] for v in sec[2 if lane_id>0 else 1]]
    i=ids.index(lane_id)
    edges=lane_edges_kinematics_at(road,s,'left' if lane_id>0 else 'right')
    pair=[edges[i],edges[i+1]]
    if lane_id>0:pair.reverse()  # left/right relative to +s
    if not forward:pair.reverse()
    result={}
    for label,(t,dt,ddt) in zip(('left','right'),pair):
        a=1-k*t
        if a*a+dt*dt<=1e-12:raise ValueError('singular edge')
        result[label]={'x':x-t*math.sin(h),'y':y+t*math.cos(h),
                       'heading':h+math.atan2(dt,a)+(0 if forward else math.pi),
                       'curvature':_edge_world_curvature(t,dt,ddt,k,dk)*(1 if forward else -1)}
    return result


def audit(root):
    roads={r.get('id'):r for r in root.findall('road')}
    rows=[];errors=[]
    for connection in root.findall('junction/connection'):
        rid=connection.get('connectingRoad')
        try:
            cr=roads[rid]
            if connection.get('contactPoint')!='start' or len(cr.findall('lanes/laneSection'))!=1:
                raise ValueError('edge probe supports one-section/start connecting roads only')
            for pair in connection.findall('laneLink'):
                lid=int(pair.get('to'));parent_lid=int(pair.get('from'))
                lane=cr.find(f"lanes/laneSection/{'left' if lid>0 else 'right'}/lane[@id='{lid}']")
                for role,contact in (('predecessor','start'),('successor','end')):
                    link=cr.find('link/'+role)
                    if link is None or link.get('elementType')!='road':raise ValueError('missing endpoint road')
                    other=roads[link.get('elementId')];oc=link.get('contactPoint')
                    other_id=parent_lid if role=='predecessor' else int(lane.find('link/'+role).get('id'))
                    if role=='predecessor' and link.get('elementId')!=connection.get('incomingRoad'):
                        raise ValueError('inconsistent incoming road identity')
                    other_forward=(oc=='end') if role=='predecessor' else (oc=='start')
                    a=endpoint_edges(cr,lid,contact,True)
                    b=endpoint_edges(other,other_id,oc,other_forward)
                    for side in ('left','right'):
                        aa,bb=a[side],b[side]
                        rows.append({'connecting_road':rid,'connecting_lane':lid,'contact':contact,'edge':side,
                                     'linked_road':other.get('id'),'linked_lane':other_id,
                                     'position_m':math.hypot(aa['x']-bb['x'],aa['y']-bb['y']),
                                     'heading_deg':math.degrees(abs((aa['heading']-bb['heading']+math.pi)%(2*math.pi)-math.pi)),
                                     'curvature_per_m':abs(aa['curvature']-bb['curvature'])})
        except (KeyError,ValueError,TypeError,AttributeError,IndexError) as e:
            errors.append({'connecting_road':rid,'reason':str(e)})
    maxima={k:max((v[k] for v in rows),default=0.) for k in ('position_m','heading_deg','curvature_per_m')}
    failures=[r for r in rows if r['position_m']>.01 or r['heading_deg']>.1 or r['curvature_per_m']>1e-7]
    return {'schema':'mapforge/junction-edge-diagnostic/v1','status':'FAIL' if errors or failures else 'PASS',
            'scope':'world inner/outer edges, explicit one-section/start connector links',
            'thresholds':{'position_m':.01,'heading_deg':.1,'curvature_per_m':1e-7},
            'count':len(rows),'failed_count':len(failures),'maxima':maxima,'rows':rows,'unresolved':errors}
