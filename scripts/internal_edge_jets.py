"""Exact one-sided world edge jets, without epsilon finite-difference jumps.

Diagnostic for line/arc/spiral roads. Both edges of driving lanes are checked
at every written geometry/offset/width/section boundary, following explicit
lane links across sections. Born/terminated lanes are recorded separately.
Auxiliary paving is explicitly outside this driving-edge diagnostic scope.
"""
import bisect
import math
from mapforge.validate.g11 import _poly_entries, _poly_value, _primitive, _advance
from mapforge.validate.smoothness import _edge_world_curvature


def active(items, station, left, key):
    items = sorted(items, key=key)
    starts = [key(i) for i in items]
    index = (bisect.bisect_left if left else bisect.bisect_right)(starts, station)-1
    return items[max(0, index)]


def section(road, s, left):
    return active(road.findall('lanes/laneSection'),s,left,lambda e:float(e.get('s')))


def states(road, lid, s, left):
    sec = section(road,s,left)
    def poly(elements, attr, station):
        entries = _poly_entries(elements, attr)
        if not entries: return [0.,0.,0.]
        e = active(entries,station,left,lambda v:v[0])
        return [_poly_value(e,station-e[0],d) for d in range(3)]
    inner = poly(road.findall('lanes/laneOffset'),'s',s)
    side = 'left' if lid>0 else 'right'; sign = 1 if lid>0 else -1
    for lane in sorted(sec.findall(side+'/lane'),key=lambda e:abs(int(e.get('id')))):
        borders = lane.findall('border')
        if borders:
            outer = poly(borders,'sOffset',s-float(sec.get('s')))
        else:
            widths = poly(lane.findall('width'),'sOffset',s-float(sec.get('s')))
            outer = [a+sign*b for a,b in zip(inner,widths)]
        if int(lane.get('id')) == lid:
            break
        inner = outer
    else:
        raise ValueError('missing linked edge lane')
    g = active(road.findall('planView/geometry'),s,left,lambda e:float(e.get('s')))
    p = _primitive(g)
    if p['kind'] not in ('line','arc','spiral'):
        raise ValueError('unsupported reference primitive')
    dk = (p['k1']-p['k0'])/p['length']
    distance = s-float(g.get('s'))
    k = p['k0']+dk*distance
    short = dict(p,length=distance,k1=k)
    x,y,h,_ = _advance(short)
    result = []
    for t,dt,ddt in (inner,outer):
        a = 1-k*t
        if a*a+dt*dt<=1e-12:raise ValueError('singular lane edge')
        result.append((x-t*math.sin(h),y+t*math.cos(h),h+math.atan2(dt,a),
                       _edge_world_curvature(t,dt,ddt,k,dk)))
    return result


def audit(root):
    rows=[]; errors=[]; births_deaths=[]
    for road in root.findall('road'):
        if road.get('name')=='junction_paving':continue
        length=float(road.get('length'))
        cuts={float(g.get('s')) for g in road.findall('planView/geometry')}
        cuts.update(float(o.get('s')) for o in road.findall('lanes/laneOffset'))
        for sec in road.findall('lanes/laneSection'):
            s0=float(sec.get('s')); cuts.add(s0)
            cuts.update(s0+float(p.get('sOffset')) for p in sec.findall('.//width')+sec.findall('.//border'))
        for s in sorted(c for c in cuts if 0<c<length):
            before,after=section(road,s,True),section(road,s,False)
            for side in ('left','right'):
                for lane in before.findall(side+'/lane'):
                    if lane.get('type')!='driving':continue
                    lid=int(lane.get('id')); following=lane.find('link/successor')
                    if before is not after and following is None:
                        births_deaths.append({'road':road.get('id'),'s':s,'lane':lid,'reason':'no-successor'})
                        continue
                    newlid=lid if before is after else int(following.get('id'))
                    try:
                        a,b=states(road,lid,s,True),states(road,newlid,s,False)
                        for edge,(aa,bb) in enumerate(zip(a,b)):
                            rows.append({'road':road.get('id'),'s_m':s,'lane':lid,'next_lane':newlid,
                                         'edge':'inner' if edge==0 else 'outer',
                                         'position_m':math.hypot(aa[0]-bb[0],aa[1]-bb[1]),
                                         'heading_deg':math.degrees(abs((aa[2]-bb[2]+math.pi)%(2*math.pi)-math.pi)),
                                         'curvature_per_m':abs(aa[3]-bb[3])})
                    except (ValueError,TypeError,KeyError) as exc:
                        errors.append({'road':road.get('id'),'s_m':s,'lane':lid,'reason':str(exc)})
    maxima={k:max((r[k] for r in rows),default=0.) for k in ('position_m','heading_deg','curvature_per_m')}
    failures=[r for r in rows if r['position_m']>.01 or r['heading_deg']>.1 or r['curvature_per_m']>1e-7]
    return {'schema':'mapforge/internal-driving-edges/v1','status':'FAIL' if failures or errors else 'PASS',
            'scope':'driving lane internal edge joints; explicit section links; excludes auxiliary paving',
            'maxima':maxima,'rows':rows,'failures':failures,'errors':errors,'lane_terminations':births_deaths}
