"""Compare written cubic-road geometry to full original observations.

This deliberately does not reuse the solver's source-error verdict. In
particular, transverse export cuts can lose oblique raw endpoint tails.
"""
import numpy as np


def audit_written_source(road, compiled, model, step=.1):
    from scripts.basemap_overlay import _lane_lines
    from scripts.shp_xodr_overlay import _densify
    from spikes.measured_connector_caps import distances
    edges,centers=_lane_lines(road,step)
    edge_map={(side,section,rank):xy for xy,side,section,rank in edges}
    center_map={(side,section,rank):xy for xy,side,section,rank in centers}
    families={};chains={}
    for row in compiled['lane_ledger']:
        side='left' if row['lane']>0 else 'right';rank=abs(row['lane']);si=row['section']
        # IBD declared SIDE is not inferred from the output lane ID. Use the
        # original physical sides' chart order, just as the compiler ledger
        # does; no nearest-target boundary association is allowed.
        mid=sum(row['chart_s'])/2
        lv=np.interp(mid,model.raw[row['left']][:,0],model.raw[row['left']][:,1])
        rv=np.interp(mid,model.raw[row['right']][:,0],model.raw[row['right']][:,1])
        high,low=(row['left'],row['right']) if lv>=rv else (row['right'],row['left'])
        inner,outer=(low,high) if row['lane']>0 else (high,low)
        for feature,i in ((inner,rank-1),(outer,rank)):
            families.setdefault(model.owner[feature],[]).append(edge_map[side,si,i])
        chains.setdefault(row['source_chain'],[]).append(center_map[side,si,rank])
    def minimum(pts,lines):return np.min([distances(pts,g) for g in lines],axis=0)
    rows=[]
    for feature,fi in model.owner.items():
        raw=model.source_xy[feature];query=np.unique(np.vstack([raw,_densify(raw,step)]),axis=0)
        values=minimum(query,families[fi]);original=minimum(raw,families[fi])
        chart=(model.axis.project(query)[:,0] if hasattr(model,'axis') else
               (query-np.asarray(model.chart['origin']))@np.asarray(model.chart['tangent']))
        interior=(chart>=compiled['chart_start_m'])&(chart<=compiled['chart_end_m'])
        owned=chart<=compiled['chart_end_m'] if compiled.get('source_domain') else np.ones(len(query),bool)
        rows.append(dict(feature=feature,kind='boundary',source_to_written_max_m=float(max(values)),
                         raw_vertex_max_m=float(max(original)),raw_vertices=len(raw),sampled_source_points=len(query),
                         inside_transverse_cuts_max_m=float(max(values[interior])) if any(interior) else None,
                         outside_transverse_cuts_points=int(sum(~interior)),
                         ordinary_owned_max_m=float(max(values[owned])) if any(owned) else None,
                         pending_junction_source_points=int(sum(~owned))))
    for chain in compiled['chains']:
        for sid in chain['source_ids']:
            feature='lane:'+sid;raw=model.source_xy[feature];query=np.unique(np.vstack([raw,_densify(raw,step)]),axis=0)
            values=minimum(query,chains[chain['key']]);role=model.roles['feature_roles'][feature]['role']
            chart=(model.axis.project(query)[:,0] if hasattr(model,'axis') else
                   (query-np.asarray(model.chart['origin']))@np.asarray(model.chart['tangent']))
            owned=chart<=compiled['chart_end_m'] if compiled.get('source_domain') else np.ones(len(query),bool)
            rows.append(dict(feature=feature,kind='center',source_role=role,source_to_written_max_m=float(max(values)),
                             raw_vertex_max_m=float(max(minimum(raw,chains[chain['key']]))),raw_vertices=len(raw),
                             sampled_source_points=len(query),ordinary_owned_max_m=float(max(values[owned])) if any(owned) else None,
                             pending_junction_source_points=int(sum(~owned))))
    # Reverse comparison against the same original physical boundary family,
    # not an arbitrary nearby lane or a prior fitted map.
    reverse=[]
    for fi,lines in families.items():
        original=[model.source_xy[k] for k in model.families[fi].features]
        reverse.append(dict(family=fi,written_to_source_max_m=float(max(max(minimum(xy,original)) for xy in lines))))
    physical=[r for r in rows if r['kind']=='boundary' or r.get('source_role')=='physical_lane_center_observation']
    worst=max(r['source_to_written_max_m'] for r in physical)
    back=max(r['written_to_source_max_m'] for r in reverse)
    return dict(status='SAMPLED_PASS_NOT_CERTIFICATE' if max(worst,back)<=model.source_tol+1e-5 else 'FAIL',
                source_to_written_max_m=worst,written_to_source_max_m=back,rows=rows,reverse=reverse,
                tolerance_m=model.source_tol,source_sample_step_m=step,source_vertices_removed=0,
                ordinary_owned_source_to_written_max_m=max(r['ordinary_owned_max_m'] for r in physical if r['ordinary_owned_max_m'] is not None),
                pending_junction_source_points=sum(r['pending_junction_source_points'] for r in physical),
                source_endpoints_included=True,complete_continuous_certificate=False,production_accepted=False)
