"""Explicit ordinary-road/source ownership, without a fictitious common start.

Mouth tails remain ORIGINAL observations owned by a pending junction rebuild.
Planning/compiling this component never certifies their aggregate coverage.
"""
import numpy as np

from mapforge.ops.source_cubic_export import source_chains


def direction_starts(model):
    chains=source_chains(model);starts={}
    for direction in ('with_s','against_s'):
        rows=[c for c in chains if c['direction']==direction]
        if not rows:raise ValueError('this asymmetric compiler expects two original direction groups')
        first=min(c['a'] for c in rows)
        external=[c for c in rows if c['a']-first<=2*model.source_tol]
        starts[direction]=max(c['a'] for c in external)
    return starts


def plan_source_domain(model, original):
    """Select source-supported cuts; no optimizer may hide source beyond them."""
    if original.find("link/successor[@elementType='junction']") is None:
        raise ValueError('tail ownership requires an original junction at road end')
    starts=direction_starts(model);anchor=min(starts,key=starts.get)
    if abs(starts['with_s']-starts['against_s'])<model.min_span:
        raise ValueError('use common-cut writer when direction separation is not structural')
    chains=source_chains(model)
    source_end=max(c['b'] for c in chains)
    mouth=min(c['b'] for c in chains if source_end-c['b']<=2.)
    # Retain a long ordinary interval, transfer the entire residual source
    # to dependent connecting roads. This is a coverage debt, NOT crop PASS.
    eligible=model.global_breaks[model.global_breaks<mouth-model.source_tol]
    if not len(eligible):raise ValueError('no long shared knot before original mouth')
    end=float(eligible[-1]);start=float(starts[anchor])
    if end-start<model.min_span:raise ValueError('ordinary source domain too short')
    split=max(starts.values())
    if np.min(abs(model.global_breaks-split))>1e-7:
        raise ValueError('rebuild shared cubic basis with source direction start as structural station')
    tails=[]
    for c in chains:
        if c['b']<=end:continue
        ids=sorted({sid for d in c['domains'] if d['b']>end for sid in d['source_lanes']})
        tails.append(dict(source_chain=c['key'],source_ids=ids,chart_s=[end,c['b']],
            owner='ORIGINAL_TOPO_INCIDENT_CONNECTORS_PENDING',coverage_accepted=False))
    return dict(schema='mapforge.source-export-domain/v1',start_m=start,end_m=end,
        direction_starts_m=starts,anchor_direction=anchor,structural_stations_m=[split],
        retained_junction_tails=tails,source_vertices_removed=0,source_extrapolated=False,
        aggregate_source_coverage_accepted=False,production_accepted=False)


def constrain_external_cuts(model, domain):
    """Full original external end caps spend the same 0.35m Euclidean budget.

Only the mouth is delegated. A zero-width tip cannot be moved to create an
artificial birth/death; a source beyond the budget rejects before solving.
"""
    chains=source_chains(model);rows=[];lower=[];labels=[]
    for direction,station in domain['direction_starts_m'].items():
        external=[c for c in chains if c['direction']==direction and c['a']<=station+1e-8]
        features={}
        for c in external:
            d=next((d for d in c['domains'] if d['a']-1e-8<=station<=d['b']+1e-8),None)
            if d is None:raise ValueError('external source cut has no physical support')
            for key in (d['left'],d['right']):
                for source_key in model.families[model.owner[key]].features:
                    features[source_key]=model.expression(key,station)
            center=.5*(model.expression(d['left'],station)+model.expression(d['right'],station))
            for sid in c['source_ids']:
                key='lane:'+sid
                if model.roles['feature_roles'][key]['role']=='physical_lane_center_observation':features[key]=center
            # Every approved or unapproved original zero-width physical tip
            # remains fixed; role approval never authorizes shifting it.
            for event in model.contacts.get('source_endpoint_inventory',[]):
                if event['source_lane_id'] not in c['source_ids'] or event.get('width_mm')!=0:continue
                tips=[model.contacts['boundary_endpoints'][v] for v in event.get('boundary_endpoints',{}).values()]
                if not tips:continue
                s=model.contact_station(tips[0]['xy'])
                if c['a']-1e-7<=s<station-1e-7:
                    raise ValueError('common side start would move an original zero-width tip')
        point,tangent,normal=model.axis.frame(station)
        for key,row in features.items():
            for i in np.flatnonzero(model.raw[key][:,0]<station):
                delta=model.source_xy[key][i]-point
                longitudinal=float(delta@tangent);target=float(delta@normal)
                remaining=model.source_tol**2-longitudinal**2
                if remaining < -1e-10:raise ValueError(f'external full source endpoint budget impossible: {key}')
                r=np.sqrt(max(0.,remaining));rows.extend([row,-row]);lower.extend([target-r,-target-r])
                labels.extend([dict(kind='asymmetric-external-source-budget',feature=key,
                    source_vertex_index=int(i),direction=direction,station_m=station)]*2)
    if rows:
        model.C=np.vstack([model.C,rows]);model.lower=np.r_[model.lower,lower];model.labels.extend(labels)
    model.written_endpoint_constraints=len(rows)
    return model
