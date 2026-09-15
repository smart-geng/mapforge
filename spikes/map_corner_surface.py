"""Isolated boundary-native MAP surface candidate (simulation auxiliary only).

Each inferred curb return is kept as three analytic spirals, not resampled into
many width segments. Constant/linear inward strips overlap a small interior
polygon. The latter is triangulated exactly; no triangle may leave the inferred
envelope plus the existing ordinary roads. Never changes driving geometry.
"""
import argparse
import copy
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from pyclothoids import SolveG2
from shapely.geometry import Polygon
from shapely.ops import unary_union, triangulate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.opendrive import writer as W
from mapforge.ops.map_to_xodr import _shift
from mapforge.validate.smoothness import (road_surface_polygon, surface_continuity,
                                        sample_road_ref, lane_edges_at)
from scripts.map_paving_audit import xml_mouths


def corner_chains(mouths):
    center = np.mean([m['pose'][:2] for m in mouths],axis=0)
    entries = []
    for i,m in enumerate(mouths):
        h=m['pose'][2]
        sign=1 if np.dot([math.cos(h),math.sin(h)],center-np.asarray(m['pose'][:2]))>=0 else -1
        for side in ('right','left'):
            point=np.asarray(_shift(m['pose'],m[side+'_t'])[:2])
            heading=m[side+'_heading']+(math.pi if sign<0 else 0.)
            entries.append({'mouth':i,'side':side,'point':point,'heading':heading,
                            'curvature':sign*m[side+'_curvature'],
                            'half_width':abs(m['left_t']-m['right_t'])/2})
    entries.sort(key=lambda p:math.atan2(*(p['point']-center)[::-1]))
    chains=[]
    for a,b in zip(entries,entries[1:]+entries[:1]):
        if a['mouth']==b['mouth']:continue
        cls=SolveG2(*a['point'],a['heading'],a['curvature'],
                    *b['point'],b['heading']+math.pi,-b['curvature'])
        length=sum(c.length for c in cls)
        if length>4*np.linalg.norm(b['point']-a['point'])+2 or any(
            max(abs(c.KappaStart),abs(c.KappaEnd))>.5 for c in cls):
            raise ValueError('no bounded G2 curb solution; no silent Bezier/hull fallback')
        chains.append((a,b,cls))
    if len(chains)!=len(mouths):raise ValueError('interleaved mouth corner ordering')
    return chains


def provenance(kind):
    return {'eligibility':'excluded','role':'paving','status':'INFERRED',
            'support_kind':kind,'travel_direction':'against_s',
            'exclusion_code':'inferred-paving','candidate_only':True,
            'consumer_scope':'simulation auxiliary; not driving topology'}


def analytic_envelope(mouths, ds=.025):
    """Dense *verification* samples of the exact curb primitives, not XODR pieces."""
    ring=[]
    for _,_,curves in corner_chains(mouths):
        for i,c in enumerate(curves):
            xs,ys=c.SampleXY(max(3,int(math.ceil(c.length/ds))+1))
            ring.extend(np.column_stack([xs,ys])[1 if i else 0:])
    polygon=Polygon(ring)
    if not polygon.is_valid or polygon.area<10 or polygon.interiors:
        raise ValueError('invalid analytic curb outline; no repair by hull/buffer')
    return polygon


def triangle_road(triangle,rid,jid):
    p=np.asarray(triangle.exterior.coords)[:3]
    i,j=max(((i,j) for i in range(3) for j in range(i+1,3)),key=lambda q:np.linalg.norm(p[q[1]]-p[q[0]]))
    other=3-i-j
    a,b=p[i].copy(),p[j].copy()
    v=b-a;length=float(np.linalg.norm(v));u=v/length;n=np.array([-u[1],u[0]])
    if np.dot(p[other]-a,n)<0:
        a,b=b,a;u=-u;n=-n
    station=float(np.dot(p[other]-a,u));height=float(np.dot(p[other]-a,n))
    if length<4 or height<1e-6 or not 1e-8<station<length-1e-8:
        raise ValueError('degenerate/small interior triangle')
    road=W.Road(rid,'junction_paving',jid).add_geometry('line',*a,math.atan2(u[1],u[0]),length)
    lane=W.Lane(1,'restricted',provenance=provenance('interior-core-triangle'))
    lane.add_width(0,height/station)
    lane.add_width(height,-height/(length-station),s_offset=station)
    section=W.LaneSection(0);section.left.append(lane);road.sections.append(section)
    return road


def build(root,*,width_factor=1.05):
    if not math.isfinite(width_factor) or width_factor<=0:
        raise ValueError('invalid inward strip width factor')
    mouths=xml_mouths(root)
    envelope=analytic_envelope(mouths)
    legs=unary_union([road_surface_polygon(r,.05) for r in root.findall('road') if r.get('junction')=='-1'])
    target=envelope.union(legs)
    old=[r for r in root.findall('road') if r.get('name')=='junction_paving']
    if not old:raise ValueError('missing original simulation auxiliary surface')
    jid=int(old[0].get('junction'))
    reserved={int(r.get('id')) for r in root.findall('road') if r not in old}
    rid=90
    doc=W.XodrDoc('boundary-native-surface')
    for a,b,cls in corner_chains(mouths):
        while rid in reserved:rid+=1
        road=W.Road(rid,'junction_paving',jid);reserved.add(rid);rid+=1
        for c in cls:road.add_geometry('spiral',c.XStart,c.YStart,c.ThetaStart,c.length,c.KappaStart,c.KappaEnd)
        wa,wb=a['half_width']*width_factor,b['half_width']*width_factor
        if any(1-max(c.KappaStart,c.KappaEnd)*max(wa,wb)<.1 for c in cls):
            raise ValueError('singular inward strip offset')
        section=W.LaneSection(0)
        lane=W.Lane(1,'restricted',provenance=provenance('analytic-G2-curb-strip'))
        lane.add_width(wa,(wb-wa)/road.length);section.left.append(lane);road.sections.append(section)
        doc.add_road(road)
    # Each fan starts 0.5m inside ordinary-road coverage. Its base uses only the
    # inner 80% of that section; corner strips cover the rest. No global hull.
    midpoints=[]
    for m in mouths:
        h=m['pose'][2];p=np.asarray(_shift(m['pose'],(m['left_t']+m['right_t'])/2)[:2])
        midpoints.append(p-.5*np.array([math.cos(h),math.sin(h)]))
    center=np.mean(midpoints,axis=0)
    midpoints.sort(key=lambda p:math.atan2(*(p-center)[::-1]))
    core=None
    for core_scale in (1.,.95,.9,.85,.8,.75,.7,.65,.6,.55,.5):
        q=Polygon([center+core_scale*(point-center) for point in midpoints])
        if q.is_valid and q.difference(target.buffer(.002)).area<=1e-4:
            core=q;break
    if core is None:raise ValueError('no admissible interior core')
    for tri in triangulate(core):
        if tri.difference(core).area>1e-6:raise ValueError('nonconvex interior core needs explicit decomposition')
        while rid in reserved:rid+=1
        doc.add_road(triangle_road(tri,rid,jid));reserved.add(rid);rid+=1
    fans=[]
    for m in mouths:
        rd=root.find(f"road[@id='{m['road_id']}']")
        pts,ss,hh=sample_road_ref(rd,.025)
        s=float(rd.get('length'))-.5
        position=np.array([np.interp(s,ss,pts[:,i]) for i in range(2)])
        h=float(np.interp(s,ss,hh));normal=np.array([-math.sin(h),math.cos(h)])
        lo=lane_edges_at(rd,s,'right')[-1];hi=lane_edges_at(rd,s,'left')[-1]
        tri=Polygon([position+(lo+.1*(hi-lo))*normal,
                     position+(hi-.1*(hi-lo))*normal,center])
        if not tri.is_valid or tri.difference(target.buffer(.002)).area>1e-4:
            raise ValueError('mouth fan exits admissible envelope')
        fans.append(tri)
        while rid in reserved:rid+=1
        doc.add_road(triangle_road(tri,rid,jid));reserved.add(rid);rid+=1
    candidate=ET.Element('OpenDRIVE')
    for road in doc.roads:doc._road_el(candidate,road)
    polygons=[road_surface_polygon(r,.025) for r in candidate.findall('road')]
    union=unary_union(polygons)
    excess=union.difference(target.buffer(.002)).area
    missing=envelope.difference(union.union(legs).buffer(.002)).area
    # Candidate rejection is explicit, not auto-clipping away underlying geometry.
    stats={'auxiliary_roads':len(polygons),'excess_m2':excess,'missing_m2':missing,
           'width_factor':width_factor,'core_scale':core_scale,
           'core_area_m2':unary_union([core,*fans]).area,
           'paving_reference_segments':[len(r.geoms) for r in doc.roads],
           'paving_width_records':[len(r.sections[0].left[0].widths) for r in doc.roads]}
    preview=copy.deepcopy(root)
    for rd in list(preview.findall("road[@name='junction_paving']")):preview.remove(rd)
    index=list(preview).index(preview.find('junction'))
    for rd in candidate.findall('road'):preview.insert(index,copy.deepcopy(rd));index+=1
    stats['surface']=surface_continuity(preview)
    # Keep rejected geometry out of deliverable files. No clipping, hole filling,
    # or declaring a disconnected/overshooting surface successful.
    surface=stats['surface']
    if (excess>.01 or missing>.5 or surface['paving_components']!=1 or
        surface['paving_holes_gt1cm2'] or surface['paving_leg_overlap_min']<.2):
        raise ValueError('surface candidate rejected: '+json.dumps(stats))
    for rd in old:root.remove(rd)
    index=list(root).index(root.find('junction'))
    for rd in candidate.findall('road'):root.insert(index,rd);index+=1
    stats['status']='CANDIDATE (not promoted)'
    return stats


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path)
    p.add_argument('--width-factor',type=float,default=1.05);a=p.parse_args()
    root=ET.parse(a.source).getroot();st=build(root,width_factor=a.width_factor)
    a.target.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root);ET.ElementTree(root).write(a.target,encoding='utf-8',xml_declaration=True)
    a.target.with_suffix('.surface-candidate.json').write_text(json.dumps(st,indent=2),encoding='utf-8')
    print(json.dumps(st))
