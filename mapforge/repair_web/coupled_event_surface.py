"""Read-only whole-road-band surface check, without buffer(0) hole repairs.

A source junction footprint is compared to driving roads and visible paving
separately. This is sampled numerical evidence, never route or map acceptance.
"""
import math

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.validation import explain_validity
from pyclothoids import Clothoid

from mapforge.repair_web.coupled_event_sources import sample_ribbon
from mapforge.validate.shp_boundary_fidelity import _project
from mapforge.validate.smoothness import _geoms


def poly_values(items, attr, stations):
    if not items: return np.zeros_like(stations)
    starts=np.array([float(e.get(attr)) for e in items]); rows=np.array([[float(e.get(k)) for k in 'abcd'] for e in items])
    i=np.clip(np.searchsorted(starts,stations,side='right')-1,0,len(items)-1)
    u=stations-starts[i]; c=rows[i]
    return c[:,0]+u*(c[:,1]+u*(c[:,2]+u*c[:,3]))


def xml_bands(road, step=.1):
    """Respect lane types, median/island gaps, section cuts and actual cubics."""
    geoms=_geoms(road); elements=road.findall('planView/geometry')
    if len(elements)!=len(geoms): raise ValueError('Unsupported whole-surface reference')
    gs=np.array([float(e.get('s')) for e in elements]); refs=[Clothoid.StandardParams(x,y,h,k0,(k1-k0)/L,L) for _,x,y,h,L,k0,k1 in geoms]
    offsets=road.findall('lanes/laneOffset'); sections=road.findall('lanes/laneSection'); rows=[]
    for j,section in enumerate(sections):
        lo=float(section.get('s')); hi=float(sections[j+1].get('s')) if j+1<len(sections) else float(road.get('length'))
        ss=set(np.linspace(lo,hi,max(2,math.ceil((hi-lo)/step)+1)))
        ss.update(float(e.get('s')) for e in offsets if lo<float(e.get('s'))<hi)
        ss.update(s for s in gs if lo<s<hi)
        ss.update(lo+float(w.get('sOffset')) for w in section.findall('.//width') if lo<lo+float(w.get('sOffset'))<hi)
        ss=np.array(sorted(ss)); idx=np.clip(np.searchsorted(gs,ss,side='right')-1,0,len(refs)-1)
        xy=np.array([[refs[i].X(s-gs[i]),refs[i].Y(s-gs[i])] for s,i in zip(ss,idx)])
        hh=np.array([refs[i].Theta(s-gs[i]) for s,i in zip(ss,idx)]); normal=np.c_[-np.sin(hh),np.cos(hh)]
        off=poly_values(offsets,'s',ss)
        for side,sign in (('left',1),('right',-1)):
            edge=off.copy()
            for lane in sorted(section.findall(side+'/lane'),key=lambda e:abs(int(e.get('id')))):
                if lane.findall('border'): raise ValueError('Explicit border surface evaluator required')
                width=poly_values(lane.findall('width'),'sOffset',ss-lo)
                # Collapse ONLY floating-point zero-width end caps, recording
                # the numerical correction. No interior clipping/buffering or
                # actual XML/source change. Substantial negative widths remain.
                caps=[i for i in (0,len(width)-1) if 0<abs(width[i])<1e-12]
                cap_roundoff=max([0.,*[abs(width[i]) for i in caps]])
                width[caps]=0.; outer=edge+sign*width
                pg=Polygon(np.vstack([xy+edge[:,None]*normal,(xy+outer[:,None]*normal)[::-1]]))
                rows.append(dict(road=road.get('id'),section=j,lane=int(lane.get('id')),lane_type=lane.get('type'),
                    auxiliary=road.get('name')=='junction_paving',polygon=pg,minimum_sampled_width_m=float(min(width)),
                    numeric_zero_cap_correction_m=float(cap_roundoff)))
                edge=outer
    return rows


def parent_bands(contract, coefficients, step=.1):
    p=contract.parent; result=[]
    # Frozen upstream is read from its actual XML, then clipped at an existing
    # section boundary START. No partial polygon is extended across that cut.
    result.extend(r for r in xml_bands(p.road,step) if float(p.road.findall('lanes/laneSection')[r['section']].get('s'))<p.start)
    for edge in range(1,5):
        start=p.births.get(edge,p.start)
        cuts=sorted({start,p.end}|{s for s in p.scope.knots if start<s<p.end})
        ss=np.unique(np.concatenate([np.linspace(a,b,max(2,math.ceil((b-a)/step)+1)) for a,b in zip(cuts,cuts[1:])]))
        xy=np.array([p.axis.frame(s)[0] for s in ss]); normal=p.axis.frame(start)[2]
        left=np.array([p.row(edge-1,s)@coefficients for s in ss]); right=np.array([p.row(edge,s)@coefficients for s in ss])
        pg=Polygon(np.vstack([xy+left[:,None]*normal,(xy+right[:,None]*normal)[::-1]]))
        result.append(dict(road='11',section='event',lane=-edge,lane_type='driving',auxiliary=False,polygon=pg,
                           minimum_sampled_width_m=float(min(left-right))))
    return result


def surface_summary(rows, source_polygon):
    excluded_types={'median','border','none'}; invalid=[]; physical=[]; driving=[]
    for row in rows:
        pg=row['polygon']
        if row['lane_type'] in excluded_types: continue
        if not pg.is_valid or row['minimum_sampled_width_m']< -1e-7:
            invalid.append(dict(road=row['road'],section=row['section'],lane=row['lane'],reason=explain_validity(pg),
                minimum_sampled_width_m=row['minimum_sampled_width_m']))
        physical.append(pg)
        if row['lane_type']=='driving' and not row['auxiliary']: driving.append(pg)
    report=dict(status='INVALID_BANDS_NO_REPAIR' if invalid else 'SAMPLED_SURFACE_ONLY',invalid_bands=invalid,
        whole_map_accepted=False,buffer_or_hull_repair_used=False,band_count=len(rows),
        auxiliary_is_not_driving=True,source_islands_preserved=len(source_polygon.interiors),
        numeric_zero_cap_correction_max_m=max([0.,*[r.get('numeric_zero_cap_correction_m',0.) for r in rows]]))
    if invalid: return report
    def measure(polygons):
        shape=unary_union(polygons); components=list(getattr(shape,'geoms',[shape]))
        holes=[Polygon(r).area for p in components for r in p.interiors]
        missing=source_polygon.difference(shape); extra=shape.difference(source_polygon)
        return dict(components=len(components),holes_m2=holes,source_junction_missing_m2=float(missing.area),
            outside_junction_area_m2=float(extra.area),source_missing_geometry=missing,
            union_geometry=shape,not_a_route_connectivity_verdict=True)
    report['visible']=measure(physical); report['driving']=measure(driving)
    return report


class EventSurface:
    def __init__(self, contract, step=.1):
        self.contract=contract; self.step=step
        record=contract.domain['inventory']['junction_record']; projection=contract.domain['partition']['projection']
        if len(record['parts'])!=1: raise ValueError('Source multipart/island hierarchy needs explicit binding')
        self.original=Polygon(_project(np.array(record['parts'][0]),projection['lat_0'],projection['lon_0']))
        if not self.original.is_valid: raise ValueError('Invalid original source junction polygon')
        movable={'11',*contract.decision['connectors']}
        self.fixed=[r for rid,road in contract.graph.roads.items() if rid not in movable for r in xml_bands(road,step)]

    def evaluate(self, snapshot=None):
        c=self.contract; rows=list(self.fixed)
        if snapshot is None:
            rows.extend(r for rid in ('11',*c.decision['connectors']) for r in xml_bands(c.graph.roads[rid],self.step))
        else:
            rows.extend(parent_bands(c,snapshot['parent_coefficients'],self.step))
            for cid,turn in snapshot['turns'].items():
                mesh=sample_ribbon(turn,step=self.step); left,right=mesh['points']['left'],mesh['points']['right']
                rows.append(dict(road=cid,section=0,lane=-1,lane_type='driving',auxiliary=False,
                    polygon=Polygon(np.vstack([left,right[::-1]])),minimum_sampled_width_m=turn['minimum_width_m']))
        return surface_summary(rows,self.original)
