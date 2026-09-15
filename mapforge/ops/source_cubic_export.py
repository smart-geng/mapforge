"""Source-native cubic road compilation for explicit geometry review.

The reference is one Line/Arc; all lane edges share a source-event knot layout.
Widths are exact differences of cubic boundaries, not sampled quintics.
This module does not approve dynamics, source roles, or production delivery.
"""
import copy
import json
from math import factorial, atan2

import numpy as np
from lxml import etree as ET


def source_end_axis(original):
    """Fit ONE chart's two end normals to original boundary cut endpoints.

    This changes coordinates, not source points or tolerance. It adds no
    reference primitives, and cannot by itself certify exported geometry.
    """
    from scipy.optimize import minimize
    from mapforge.ops.arc_source_chart import ArcChart
    origin=np.asarray(original.chart['origin']);e=np.asarray(original.chart['tangent'])
    n=np.array([-e[1],e[0]]);heading=atan2(e[1],e[0])
    ends=[]
    for index,ext in ((0,min(f.knots[0] for f in original.families)),
                      (-1,max(f.knots[-1] for f in original.families))):
        st=np.array([r[index] for key,r in original.raw.items()
                     if key in original.owner and abs(r[index,0]-ext)<2.])
        if len(st)<3:raise ValueError('insufficient original transverse endpoints')
        ends.append(origin+st@np.stack([e,n]))
    def spreads(z):
        axis=ArcChart(tuple(origin),heading+z[0],z[1]/original.chart['length'])
        return [float(np.ptp(axis.project(xy)[:,0])) for xy in ends]
    # Minimax spread: not an unconstrained fit to a prior generated road.
    fit=minimize(lambda z:max(spreads(z)),[-.05,.03],method='Powell',
                 bounds=[(-.1,.1),(-.1,.1)],options={'xtol':1e-10,'ftol':1e-10,'maxiter':100})
    if not np.isfinite(fit.x).all():raise ValueError('nonfinite source cut axis')
    return dict(heading_delta=float(fit.x[0]),curvature=float(fit.x[1]/original.chart['length']),
                source_endpoint_spreads_m=spreads(fit.x),source_points_changed=False)


def cubic_model(original, minimum_span=6.0, end_axis=False, structural_stations=()):
    from spikes.arc_source_boundary import ArcSourceBoundaryBlock
    if original.degree != 3 or not np.isfinite(minimum_span) or minimum_span < 5.5:
        raise ValueError('cubic source model and explicit >=5.5m width spans required; reference remains one primitive')
    model = copy.deepcopy(original)
    model.min_span = float(minimum_span)
    axis=source_end_axis(original) if end_axis else dict(heading_delta=0.,curvature=0.)
    result=ArcSourceBoundaryBlock(model,axis['heading_delta'],axis['curvature'],structural_stations)
    result.export_axis_selection=axis
    return result


def power(model, x, feature, station):
    if model.degree != 3:
        raise ValueError('quintic boundaries cannot be emitted as cubic widths')
    return np.array([model.expression(feature, station, j) @ x / factorial(j) for j in range(4)])


def coefficients_minimum(c, length):
    c = np.asarray(c, float)
    roots = np.polynomial.polynomial.polyroots(np.arange(1, len(c)) * c[1:])
    query = [0., length] + [float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0 < r.real < length]
    return float(min(np.polynomial.polynomial.polyval(query, c)))


def _poly(parent, tag, station, c, station_name='sOffset'):
    return ET.SubElement(parent, tag, **{station_name:format(station,'.17g'),
        **{k:format(float(v),'.17g') for k,v in zip('abcd', c)}})


def source_chains(model):
    """Identity chains use original TOPO, never nearest-neighbour matches."""
    from mapforge.ops.source_center_support import _ordinary_groups
    from mapforge.ops.source_transition_domains import midpoint_domains
    groups=_ordinary_groups(model)
    domains=midpoint_domains(model)
    result=[]
    for ids in sorted({tuple(ids) for ids in groups.values()}):
        ids=list(ids)
        directions={o['travel_direction'] for o in model.scope['occurrences']
                    if o['road']==model.road and o['source_lane_id'] in ids}
        if len(directions)!=1: raise ValueError('ambiguous original traffic direction')
        pairs=[model.lane_pairs[sid] for sid in ids]
        # Families extend across original cuts without new curve variables.
        l,r=pairs[0][:2]
        selected=[d for d in domains if set(d['source_lanes'])<=set(ids)]
        a=min(d['a'] for d in selected);b=max(d['b'] for d in selected)
        speeds={float(model.scope['observations'][sid]['source_max_speed_kmh']) for sid in ids}
        if not all(np.isfinite(v) and v>0 for v in speeds):raise ValueError('explicit finite positive source speed required')
        speed_events=[]
        if len(speeds)!=1:
            from mapforge.ops.source_speed_events import chain_speed_events
            speed_events=chain_speed_events(model,ids)
        status=('SEPARATE_MOVEMENT_CHECK_REQUIRED' if any(model.roles['feature_roles']['lane:'+sid]['role']!='physical_lane_center_observation'
                for sid in ids) else 'SOURCE_PATH_PREPARED_NOT_GEOMETRY')
        result.append(dict(key=len(result),source_ids=ids,left=l,right=r,a=float(a),b=float(b),domains=selected,
                           direction=next(iter(directions)),speed_kmh=next(iter(speeds)) if len(speeds)==1 else None,
                           **({'speed_events':speed_events} if speed_events else {}),source_path_status=status))
    return result


def transverse_domain(model, chains):
    start=max(c['a'] for c in chains if c['a']<min(f.knots[0] for f in model.families)+2.)
    end=min(c['b'] for c in chains if c['b']>max(f.knots[-1] for f in model.families)-2.)
    return start,end


def source_at_port(chain, station):
    """Use the ORIGINAL record supporting the NEW port, not the old endpoint.

    A record cut crossed by an extended shared road does not change TOPO.
    Keeping the old ID would incorrectly charge already reconstructed parent
    geometry to the incident connector's source corridor.
    """
    domains=[d for d in chain['domains'] if d['a']-1e-8<=station<=d['b']+1e-8]
    source_ids={sid for d in domains for sid in d['source_lanes']}
    if len(source_ids)!=1:raise ValueError('new port has no unique original source record')
    return source_ids.pop()


def constrain_written_endpoints(model, *, transverse_stations=None):
    """Charge lost transverse tails to the SAME Euclidean source budget.

    Witnesses constrain the actual compiled endpoint, including full original
    boundary/physical-center vertices. No source extrapolation or new knots.
    """
    chains=source_chains(model)
    if transverse_stations is None:
        start,end=transverse_domain(model,chains)
    else:
        stations=np.asarray(transverse_stations,float)
        if stations.shape!=(2,) or not np.isfinite(stations).all() or stations[1]<=stations[0]:
            raise ValueError('finite ordered shared transverse endpoint stations required')
        start,end=stations
    rows=[];lower=[];labels=[]
    for side,station in (('start',start),('end',end)):
        point,tangent,normal=model.axis.frame(station)
        features={k:model.expression(k,station) for k in model.owner
                  if model.families[model.owner[k]].knots[0]<=station<=model.families[model.owner[k]].knots[-1]}
        for chain in chains:
            domains=[d for d in chain['domains'] if d['a']-1e-9<=station<=d['b']+1e-9]
            if not domains:continue
            d=domains[0];row=.5*(model.expression(d['left'],station)+model.expression(d['right'],station))
            for sid in chain['source_ids']:
                key='lane:'+sid
                if model.roles['feature_roles'][key]['role']=='physical_lane_center_observation':features[key]=row
        for feature,row in features.items():
            st=model.raw[feature];mask=st[:,0]<station if side=='start' else st[:,0]>station
            for index in np.flatnonzero(mask):
                delta=model.source_xy[feature][index]-point
                longitudinal=float(delta@tangent);target=float(delta@normal)
                budget=model.source_tol**2-longitudinal**2
                if budget < -1e-10:raise ValueError(f'full written endpoint budget impossible: {feature}/{side}/{longitudinal:.6f}m')
                allowance=np.sqrt(max(0.,budget))
                rows.extend([row,-row]);lower.extend([target-allowance,-target-allowance])
                labels.extend([dict(kind='written-endpoint-budget',feature=feature,side=side,source_vertex_index=int(index))]*2)
    if rows:
        model.C=np.vstack([model.C,rows]);model.lower=np.r_[model.lower,lower];model.labels.extend(labels)
    model.written_endpoint_constraints=len(rows)
    return model


def constrain_flat_connector_port(model):
    """One shared endpoint candidate for ALL incident movements.

    Test zero lateral boundary slopes at the junction-side transverse cut.
    This is a model hypothesis under unchanged full-source constraints, not
    a modification of measured headings or an independently glued-on cap.
    If infeasible the entire candidate must be rejected.
    """
    _,station=transverse_domain(model,source_chains(model))
    families=sorted({model.owner[d['left']] for c in source_chains(model) for d in c['domains']
                     if d['a']-1e-8<=station<=d['b']+1e-8}|
                    {model.owner[d['right']] for c in source_chains(model) for d in c['domains']
                     if d['a']-1e-8<=station<=d['b']+1e-8})
    for fi in families:
        row=model.expression(model.families[fi].features[0],station,1)
        model.E=np.vstack([model.E,row])
        model.equality_labels.append(dict(kind='shared-flat-connector-port',family=fi,station=float(station)))
    model.connector_port_hypothesis=dict(kind='zero-lateral-slope-shared-by-all-movements',
                                        station=float(station),families=families,source_changed=False)
    return model


def compile_road(model, x, original, *, source_domain=None, source_direction=None):
    """Build a new road on the source-supported transverse domain.

    Source IDs survive in an explicit chain/span ledger; source record cuts
    are not geometry knots. Birth/death events retain zero-width semantics.
    No external junction references are changed here: return the port map.
    """
    if model.degree!=3 or (model.reference_curvature!=0 and not hasattr(model,'axis')):
        raise ValueError('writer requires direct cubic boundaries on one exact Line/Arc')
    x=np.asarray(x,float)
    if x.shape!=(model.nvar,) or not np.isfinite(x).all():raise ValueError('invalid shared state')
    audit=model.audit(x)
    if (audit['inequality_violation']>1e-7 or audit['scaled_C2_residual']>1e-7 or
        audit['boundary_same_chart_max_m']>model.source_tol+1e-7 or audit['exact_width_min_m'] < -1e-7 or
        not audit['full_source_center_support']['physical_source_to_center_certified']):
        raise ValueError('source/width/continuity geometry state failed')
    chains=source_chains(model)
    if source_direction is not None:
        if (source_direction not in ('with_s','against_s') or source_domain is not None
                or {c['direction'] for c in chains}!={source_direction}):
            raise ValueError('explicit one-direction compilation requires exactly that original traffic direction')
    if source_domain is not None:
        from mapforge.ops.source_export_domain import plan_source_domain
        if source_domain!=plan_source_domain(model,original):
            raise ValueError('source domain differs from current original source/road plan')
    # Exterior ends are a common physical transverse cut. Their full raw
    # source tails must subsequently pass world-distance readback; no crop PASS.
    start,end=(transverse_domain(model,chains) if source_domain is None else
               (source_domain['start_m'],source_domain['end_m']))
    cuts=np.unique(np.r_[start,model.global_breaks[(model.global_breaks>start+1e-7)&(model.global_breaks<end-1e-7)],end])
    if min(np.diff(cuts))<model.min_span-1e-7:raise ValueError('end cut creates a short output interval')
    road=copy.deepcopy(original);road.set('length',format(end-start,'.17g'))
    for tag in ('planView','lanes','elevationProfile','lateralProfile','objects','signals'):
        if tag in ('objects','signals') and road.find(tag) is not None and len(road.find(tag)):
            raise ValueError('attached object reprojection not implemented; refusing stale objects')
        for old in road.findall(tag):road.remove(old)
    if hasattr(model,'axis'):xy,tangent,_=model.axis.frame(start)
    else:
        tangent=np.asarray(model.chart['tangent']);xy=np.asarray(model.chart['origin'])+start*tangent
    pv=ET.SubElement(road,'planView');g=ET.SubElement(pv,'geometry',s='0',x=format(xy[0],'.17g'),y=format(xy[1],'.17g'),
                       hdg=format(atan2(tangent[1],tangent[0]),'.17g'),length=road.get('length'))
    if model.reference_curvature==0:ET.SubElement(g,'line')
    else:ET.SubElement(g,'arc',curvature=format(model.reference_curvature,'.17g'))
    lanes=ET.SubElement(road,'lanes');sections=[];ledger=[]
    for lo,hi in zip(cuts[:-1],cuts[1:]):
        mid=(lo+hi)/2;active=[c for c in chains if c['a']<=mid<c['b']]
        active=copy.deepcopy(active)
        for c in active:
            candidates=[d for d in c['domains'] if d['a']<=mid<=d['b']]
            pairs={(model.owner[d['left']],model.owner[d['right']]) for d in candidates}
            if len(pairs)!=1:raise ValueError('uncovered or ambiguous physical source chain interval')
            c['left'],c['right']=candidates[0]['left'],candidates[0]['right']
        groups={direction:sorted([c for c in active if c['direction']==direction],
             key=lambda c: .5*(model.expression(c['left'],mid)+model.expression(c['right'],mid))@x,
             reverse=direction=='with_s') for direction in ('with_s','against_s')}
        if source_domain is not None:
            groups={d:cs if mid>=source_domain['direction_starts_m'][d] else [] for d,cs in groups.items()}
            anchor=source_domain['anchor_direction']
            if not groups[anchor]:raise ValueError('source-backed continuous offset anchor missing')
        elif source_direction is not None:
            anchor=source_direction
            if not groups[anchor]:raise ValueError('one-direction original support has an internal gap')
        elif not all(groups.values()):raise ValueError('bidirectional source domain incomplete')
        else:anchor='with_s'
        def edges(c):
            l,r=power(model,x,c['left'],lo),power(model,x,c['right'],lo)
            sign=(model.expression(c['left'],mid)-model.expression(c['right'],mid))@x
            return (l,r) if sign>=0 else (r,l)
        rin=edges(groups['with_s'][0])[0] if groups['with_s'] else None
        lin=edges(groups['against_s'][0])[1] if groups['against_s'] else None
        # Put reference-lane zero on the innermost right physical edge;
        # positive non-driving lane 1 represents the source median gap.
        offset=lin if anchor=='against_s' else rin
        _poly(lanes,'laneOffset',lo-start,offset,'s')
        sections.append((lo,hi,groups,rin,lin))
    emitted=[]
    for si,(lo,hi,groups,rin,lin) in enumerate(sections):
        anchor=source_domain['anchor_direction'] if source_domain is not None else source_direction or 'with_s'
        sec=ET.SubElement(lanes,'laneSection',s=format(lo-start,'.17g'))
        left=ET.SubElement(sec,'left') if groups['against_s'] else None
        center=ET.SubElement(sec,'center');ET.SubElement(center,'lane',id='0',type='none',level='false')
        right=ET.SubElement(sec,'right') if groups['with_s'] else None;mapping={}
        both=bool(groups['with_s'] and groups['against_s'])
        if both:
            median=lin-rin
            if coefficients_minimum(median,hi-lo)<-1e-7:raise ValueError('source median edges cross')
            ml=ET.SubElement(left if anchor=='with_s' else right,'lane',
                id='1' if anchor=='with_s' else '-1',type='median',level='false');_poly(ml,'width',0.,median)
        for direction,side,sign in (('with_s',right,-1),('against_s',left,1)):
            previous=rin if sign<0 else lin
            for rank,c in enumerate(groups[direction]):
                l,r=power(model,x,c['left'],lo),power(model,x,c['right'],lo)
                sign_at_mid=(model.expression(c['left'],(lo+hi)/2)-model.expression(c['right'],(lo+hi)/2))@x
                high,low=(l,r) if sign_at_mid>=0 else (r,l)
                inner,outer=(high,low) if sign<0 else (low,high)
                if max(abs(inner-previous))>1e-7:raise ValueError('unshared adjacent boundary; no gap filling')
                width=sign*(outer-inner)
                if coefficients_minimum(width,hi-lo)<-1e-7:raise ValueError('negative exact output width')
                lid=sign*(rank+1+(both and direction!=anchor));ln=ET.SubElement(side,'lane',id=str(lid),type='driving',level='false')
                ET.SubElement(ln,'link');_poly(ln,'width',0.,width)
                ET.SubElement(ln,'roadMark',sOffset='0',type='broken',weight='standard',color='standard',width='0.12',laneChange='both')
                from mapforge.ops.source_speed_events import speed_records
                for offset,kmh in speed_records(c,lo,hi):
                    ET.SubElement(ln,'speed',sOffset=format(offset,'.17g'),max=format(kmh/3.6,'.17g'),unit='m/s')
                ET.SubElement(ln,'userData',code='mapforge.source_chain/v1',value=json.dumps(c['source_ids']))
                ET.SubElement(ln,'userData',code='mapforge.provenance/v1',value=json.dumps({
                    'travel_direction':c['direction'], 'source_identity':'source_chain/v1',
                    'status':'TRANSFORMED', 'geometry_acceptance':'NOT_DELIVERY'}))
                mapping[c['key']]=(ln,lid,c)
                ledger.append(dict(section=si,lane=lid,source_chain=c['key'],source_ids=c['source_ids'],
                                  chart_s=[float(lo),float(hi)],left=c['left'],right=c['right']))
                previous=outer
        emitted.append(mapping)
    for before,after in zip(emitted[:-1],emitted[1:]):
        for key in before.keys()&after.keys():
            a,aid,_=before[key];b,bid,_=after[key]
            ET.SubElement(a.find('link'),'successor',id=str(bid));ET.SubElement(b.find('link'),'predecessor',id=str(aid))
    # External IDs derived from original source identity at each port, not rank guesses.
    port_map={};port_sources=[]
    oldsecs=original.findall('lanes/laneSection')
    for contact,oldsec,new in (('start',oldsecs[0],emitted[0]),('end',oldsecs[-1],emitted[-1])):
        for ln in oldsec.findall('*/lane'):
            sid=ln.find("userData[@code='mapforge.source_lane']")
            if sid is None:continue
            matches=[(l,lid,c) for l,lid,c in new.values() if sid.get('value') in c['source_ids']]
            if not matches and source_domain is not None and contact=='start':
                if original.find("link/predecessor") is not None:
                    raise ValueError('asymmetric external linked port requires upstream dependency rebuild')
                port_sources.append(dict(contact=contact,old_lane=int(ln.get('id')),old_source=sid.get('value'),
                    status='SOURCE_START_IS_INTERIOR_SECTION_NOT_ROAD_ENDPOINT',original_topology_changed=False))
                continue
            if len(matches)!=1:raise ValueError('original endpoint has no unique rebuilt source chain')
            port_map[(contact,int(ln.get('id')))]=matches[0][1]
            # The endpoint-specific original identity is useful to existing
            # via-TOPO readers; the full interval identity remains in the ledger.
            current=source_at_port(matches[0][2],start if contact=='start' else end)
            ET.SubElement(matches[0][0],'userData',code='mapforge.source_lane',value=current)
            port_sources.append(dict(contact=contact,old_lane=int(ln.get('id')),new_lane=matches[0][1],
                                     old_source=sid.get('value'),current_source=current,original_topology_changed=False))
    order={'link':0,'type':1,'planView':2,'elevationProfile':3,'lateralProfile':4,'lanes':5,'objects':6,'signals':7,
           'surface':8,'railroad':9,'userData':10,'include':11}
    road[:]=sorted(road,key=lambda e:order.get(e.tag,12))
    return road,dict(chart_start_m=float(start),chart_end_m=float(end),reference_primitives=1,
        independent_knots_m=cuts.tolist(),minimum_width_span_m=float(min(np.diff(cuts))),
        lane_ledger=ledger,chains=chains,port_sources=port_sources,source_domain=source_domain,source_direction=source_direction,
        dynamics_accepted=False,production_accepted=False),port_map
