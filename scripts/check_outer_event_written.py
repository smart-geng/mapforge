"""Final-XML independent edge/source readback for the registered L01 event.

Does not import the fitter or consume fitted coefficients. Source geometry is
read again from the frozen packet; raw coordinates are never best-fit aligned.
"""
import copy
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
from pyproj import Transformer

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.repair_web.model import digest
from scripts.internal_edge_jets import states,audit,active
from mapforge.ops.continuous_offset_dynamics import audit_span


def boundary_poly(road, edge, s, left=False):
    sec=active(road.findall('lanes/laneSection'),s,left,lambda e:float(e.get('s')))
    def at(items,attr,station):
        if not items:return np.zeros(4)
        e=active(items,station,left,lambda e:float(e.get(attr)))
        c=np.array([float(e.get(k,'0')) for k in 'abcd']);u=station-float(e.get(attr))
        return np.array([c[0]+c[1]*u+c[2]*u*u+c[3]*u**3,c[1]+2*c[2]*u+3*c[3]*u*u,c[2]+3*c[3]*u,c[3]])
    t=at(road.findall('lanes/laneOffset'),'s',s)
    for lid in range(1,edge+1):
        lane=next((l for l in sec.findall('right/lane') if int(l.get('id'))==-lid),None)
        if lane is None:break
        t-=at(lane.findall('width'),'sOffset',s-float(sec.get('s')))
    return t


def max_abs(c,L):
    roots=np.polynomial.polynomial.polyroots([c[1],2*c[2],3*c[3]])
    xs=[0.,L]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<L]
    return max(abs(float(np.polynomial.polynomial.polyval(v,c))) for v in xs)


def semantic(e):
    return (str(e.tag),sorted(e.attrib.items()),(e.text or '').strip(),tuple(semantic(c) for c in e))


def check(data,baseline,packet,scope,inventory):
    root,old=ET.fromstring(data),ET.fromstring(baseline)
    rid=scope['road'];lo,hi=scope['knots'][0],scope['knots'][-1]
    road=next(r for r in root.findall('road') if r.get('id')==rid)
    before=next(r for r in old.findall('road') if r.get('id')==rid)
    other=all(semantic(a)==semantic(b) for a,b in zip(root.findall('road'),old.findall('road')) if a.get('id')!=rid)
    other=other and [r.get('id') for r in root.findall('road')]==[r.get('id') for r in old.findall('road')]
    other=other and semantic(root.find('header'))==semantic(old.find('header'))
    stripped=[]
    for r in (road,before):
        r=copy.deepcopy(r)
        for parent in r.iter():
            for child in list(parent):
                if child.tag in ('width','laneOffset'):parent.remove(child)
        stripped.append(semantic(r))
    cuts={0.,float(road.get('length')),lo,hi}
    for r in (road,before):
        cuts.update(float(e.get('s')) for e in r.findall('lanes/laneOffset'))
        for sec in r.findall('lanes/laneSection'):
            s=float(sec.get('s'));cuts.add(s)
            cuts.update(s+float(w.get('sOffset')) for w in sec.findall('.//width'))
    cuts=sorted(cuts)
    outside=[]
    for a,b in zip(cuts,cuts[1:]):
        if a>=lo and b<=hi:continue
        for edge in range(5):
            outside.append(max_abs(boundary_poly(road,edge,a)-boundary_poly(before,edge,a),b-a))
    g=road.find('planView/geometry');h=float(g.get('hdg'));origin=np.array([float(g.get('x')),float(g.get('y'))])
    if len(road.findall('planView/geometry'))!=1 or g[0].tag!='line':raise ValueError('Unsupported verification chart')
    basis=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
    proj=Transformer.from_crs(4326,root.findtext('header/geoReference'),always_xy=True)
    rows=[];full_rows=[]
    for item in inventory:
        part=packet['boundaries'][item['key']]['records'][item['record']]['parts'][item['part']]
        xy=np.asarray([proj.transform(float(p[0]),float(p[1])) for p in part])
        st=(xy-origin)@basis
        if np.all(np.diff(st[:,0])<0):st=st[::-1]
        if np.any(np.diff(st[:,0])<=0):raise ValueError('Source not a monotone graph')
        for p,q in zip(st[:-1],st[1:]):
            slope=(q[1]-p[1])/(q[0]-p[0])
            a,b=max(0.,p[0]),min(float(road.get('length')),q[0])
            if b<=a:continue
            points=sorted({a,b}|{v for v in cuts if a<v<b})
            for u,v in zip(points,points[1:]):
                source=np.array([p[1]+slope*(u-p[0]),slope,0.,0.])
                c=boundary_poly(road,item['edge'],u)-source
                error=max_abs(c,v-u)
                row={'key':item['key'],'edge':item['edge'],'s':[float(u),float(v)],'max_m':error}
                full_rows.append(row)
                if u>=lo-1e-8 and v<=hi+1e-8:rows.append(row)
    internal=audit(root)
    event_joints=[r for r in internal['rows'] if r['road']==rid and lo<=r['s_m']<=hi]
    # Explicit newborn-edge closure is not covered by audit's linked-lane loop.
    births=[]
    for edge,s in scope['births']:
        a=states(road,-(edge-1),s,True)[1];b=states(road,-edge,s,False)[1]
        births.append({'edge':edge,'s':s,'position_m':math.hypot(a[0]-b[0],a[1]-b[1]),
                       'heading_rad':abs(a[2]-b[2]),'curvature_per_m':abs(a[3]-b[3])})
    max_source=max(r['max_m'] for r in rows)
    max_outside=max(outside,default=0.)
    good=(other and stripped[0]==stripped[1] and max_outside<1e-8 and
          max_source<=scope['source_tolerance_m']+1e-7 and
          not any(r['road']==rid and lo<=r['s_m']<=hi for r in internal['failures']) and
          all(r['position_m']<1e-8 and r['heading_rad']<1e-8 and r['curvature_per_m']<1e-8 for r in births))
    return {'status':'PASS_LOCAL_EVENT_NOT_MAP' if good else 'FAIL', 'xodr_sha256':digest(data),
            'map_accepted':False,'other_roads_unchanged':other,'ids_links_speeds_marks_unchanged':stripped[0]==stripped[1],
            'outside_event_max_change_m':max_outside,'source_event_max_m':max_source,
            'full_same_road_source_max_m':max(r['max_m'] for r in full_rows),
            'full_same_road_source_scope':'original boundary segment portions within target longitudinal extent; not endpoint/tail admission',
            'source_event_rows':rows,'worst_full_same_road_source':sorted(full_rows,key=lambda r:r['max_m'],reverse=True)[:10],
            'event_joints':event_joints,'birth_jets':births,'whole_map_internal_failures':internal['failures'],
            'surface':'NOT_VALIDATED','dynamics':'NOT_VALIDATED','absolute_crs':'NOT_VALIDATED'}


def check_dynamics(data,packet,scope):
    root=ET.fromstring(data);rid=scope['road']
    road=next(r for r in root.findall('road') if r.get('id')==rid)
    rows=[]
    for sec in road.findall('lanes/laneSection'):
        lo=float(sec.get('s'));following=[float(s.get('s')) for s in road.findall('lanes/laneSection') if float(s.get('s'))>lo]
        hi=min(following+[float(road.get('length'))])
        a,b=max(lo,scope['knots'][0]),min(hi,scope['knots'][-1])
        if b<=a:continue
        cuts={a,b}
        cuts.update(float(o.get('s')) for o in road.findall('lanes/laneOffset') if a<float(o.get('s'))<b)
        cuts.update(lo+float(w.get('sOffset')) for w in sec.findall('.//width')+sec.findall('.//speed') if a<lo+float(w.get('sOffset'))<b)
        cuts=sorted(cuts)
        for lane in sec.findall('right/lane'):
            lid=int(lane.get('id'))
            sid=next(e.get('value') for e in lane.findall('userData') if e.get('code')=='mapforge.source_lane')
            speed=packet['observations'][sid]['source_max_speed_kmh']
            if not speed or speed<=0:raise ValueError('Unknown source speed')
            for start,end in zip(cuts,cuts[1:]):
                w=active(lane.findall('speed'),start-lo,False,lambda e:float(e.get('sOffset')))
                if w.get('unit','m/s')!='m/s' or abs(float(w.get('max'))*3.6-speed)>1e-5:
                    raise ValueError('Written speed differs from original observation')
                co=(boundary_poly(road,-lid-1,start)+boundary_poly(road,-lid,start))*.5
                rows.append({'lane':lid,'source_lane':sid,'s':[start,end],
                             'check':audit_span(co,end-start,0.,0.,speed)})
    return {'status':'FAIL' if any(r['check']['status']=='FAIL' for r in rows) else
            'UNKNOWN' if any(r['check']['status']=='UNKNOWN' for r in rows) or not rows else 'BOUNDED',
            'xodr_sha256':digest(data),'scope':'written driving-lane centers in L01 support; all lanes and polynomial intervals; not whole map',
            'limits':[2.5,1.],'source_speed_changed':False,'rows':rows,
            'counts':{k:sum(r['check']['status']==k for r in rows) for k in ('FAIL','UNKNOWN','BOUNDED')},
            'max_observed':[max(r['check']['observed_maxima'][i]['value'] for r in rows) for i in range(2)],
            'map_accepted':False}
