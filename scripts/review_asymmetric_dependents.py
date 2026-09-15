"""Read written parent+incident turns against FULL unchanged source records."""
import argparse
import copy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from lxml import etree as LE
from scripts.build_source_geometry_candidate import dump
from scripts.gen_all import _sha256,shp_source
from scripts.fit_source_boundary_block import unchanged
from scripts.build_ordinary_source_road import load_road
from mapforge.ops.source_cubic_export import cubic_model,compile_road
from mapforge.ops.source_export_domain import plan_source_domain,constrain_external_cuts
from mapforge.ops.reconstruction_scope import digest
from scripts.basemap_overlay import _lane_lines
from scripts.shp_xodr_overlay import _densify
from spikes.measured_connector_caps import distances,raw_curves,composite_sources,crop_at_projection
from scripts.review_measured_ribbon import target_curves,stats,fidelity,minimum_width,sampled_strip_validity,independent_design_check
from scripts.review_source_connectors import written_forward_minimum,source_ok
from scripts.internal_edge_jets import audit as internal_audit
from scripts.esmini_lane_interfaces import check as consumer_check
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def current_source_state(directory):
    state=json.loads((directory/'shared-state.json').read_text(encoding='utf8'))
    original,roles,hashes,root=load_road(state['source_directory'],state['decision_file'],state['road'])
    model=cubic_model(original,minimum_span=state['minimum_width_span'],end_axis=state['end_axis'],
        structural_stations=state['source_export_domain']['structural_stations_m'])
    domain=plan_source_domain(model,root.find(f"road[@id='{state['road']}']"))
    if domain!=state['source_export_domain']:raise ValueError('source ownership plan changed')
    model=constrain_external_cuts(model,domain)
    if digest(model.describe())!=state['model_sha']:raise ValueError('current shared model differs from saved state')
    x=np.asarray(state['coefficients'])
    _,compiled,_=compile_road(model,x,root.find(f"road[@id='{state['road']}']"),source_domain=domain)
    return model,compiled,hashes


def ordinary_targets(road,compiled,model,step):
    edges,centers=_lane_lines(road,step)
    edges={(side,section,rank):xy for xy,side,section,rank in edges}
    centers={(side,section,rank):xy for xy,side,section,rank in centers}
    families={};chains={}
    for row in compiled['lane_ledger']:
        si=row['section'];lid=row['lane'];side='left' if lid>0 else 'right';rank=abs(lid)
        mid=sum(row['chart_s'])/2;l,r=row['left'],row['right']
        lv=np.interp(mid,*model.raw[l].T);rv=np.interp(mid,*model.raw[r].T)
        high,low=(l,r) if lv>=rv else (r,l)
        inner,outer=(low,high) if lid>0 else (high,low)
        for key,index in ((inner,rank-1),(outer,rank)):
            families.setdefault(model.owner[key],[]).append(edges[side,si,index])
        chains.setdefault(row['source_chain'],[]).append(centers[side,si,rank])
    return families,chains


def aggregate_original(root,model,compiled,records,src,project,step=.1):
    """No nearest other lane: membership comes from the rebuilt port ledger.

Original tails are compared without dropping ANY original vertices. A turn
tail is only its prefix/suffix up to the unchanged original mouth endpoint;
the entire via is separately audited, not charged to the ordinary boundary.
"""
    road=root.find(f"road[@id='{model.road}']")
    families,chains=ordinary_targets(road,compiled,model,step);receipts=[];tail_lines=[]
    ports={int(r['new_lane']):r for r in compiled['port_sources'] if r['contact']=='end'}
    for row in records:
        connector=root.find(f"road[@id='{row['road']}']")
        actual=target_curves(connector,step)
        for role in ('predecessor','successor'):
            link=connector.find('link/'+role)
            if link.get('elementId')!=model.road:continue
            if link.get('contactPoint')!='end':raise ValueError('unexpected original junction contact')
            lid=int(connector.find('lanes/laneSection/right/lane/link/'+role).get('id'))
            sid=ports[lid]['current_source']
            matches=[c for c in compiled['chains'] if sid in c['source_ids']]
            if len(matches)!=1:raise ValueError('ambiguous original tail source chain')
            c=matches[0];l,r,*_=model.lane_pairs[sid]
            mid=compiled['chart_end_m'];lv=np.interp(mid,*model.raw[l].T);rv=np.interp(mid,*model.raw[r].T)
            high,low=(l,r) if lv>=rv else (r,l)
            left,right=(high,low) if c['direction']=='with_s' else (low,high)
            source=raw_curves(src,sid,project)
            for field,key in (('left',left),('right',right),('center',None)):
                original_tip=source[field][-1 if role=='predecessor' else 0]
                curve=crop_at_projection(actual[field],original_tip,role=='successor')
                if len(curve)<2:raise ValueError('written tail is degenerate')
                if key is None:chains[c['key']].append(curve)
                else:families[model.owner[key]].append(curve);tail_lines.append(curve)
            receipts.append(dict(source_chain=c['key'],source_lane_id=sid,connector=row['road'],role=role,
                source_topology_path=row['source_identity']['source_lane_ids'],original_endpoint_retained=True))
    covered={r['source_chain'] for r in receipts}
    pending=[r for r in compiled['source_domain']['retained_junction_tails'] if r['source_chain'] not in covered]
    def minimum(points,lines):
        # Bound memory without changing sampled support or correspondence.
        return np.concatenate([np.min([distances(q,g) for g in lines],axis=0) for q in np.array_split(points,max(1,len(points)//500))])
    rows=[]
    for key,fi in model.owner.items():
        raw=model.source_xy[key];q=np.vstack([raw,_densify(raw,step)])
        rows.append(dict(feature=key,role='physical_boundary',raw_vertices=len(raw),
            full_source_to_written=stats(minimum(q,families[fi]))))
    for c in compiled['chains']:
        for sid in c['source_ids']:
            key='lane:'+sid;raw=model.source_xy[key];q=np.vstack([raw,_densify(raw,step)])
            rows.append(dict(feature=key,role=model.roles['feature_roles'][key]['role'],raw_vertices=len(raw),
                full_source_to_written=stats(minimum(q,chains[c['key']]))))
    reverse=[]
    for fi,lines in families.items():
        sources=[model.source_xy[k] for k in model.families[fi].features]
        reverse.append(dict(family=fi,full_written_to_source_max_m=max(float(max(minimum(g,sources))) for g in lines)))
    physical=[r for r in rows if r['role'] in ('physical_boundary','physical_lane_center_observation')]
    front=max(r['full_source_to_written']['max_m'] for r in physical)
    back=max(r['full_written_to_source_max_m'] for r in reverse)
    return dict(status='SAMPLED_PASS_NOT_CERTIFICATE' if max(front,back)<=model.source_tol+1e-5 and not pending else 'FAIL',
        full_source_to_written_max_m=front,full_written_to_source_max_m=back,
        rows=rows,reverse=reverse,tail_ownership_receipts=receipts,unassigned_tails=pending,
        complete_original_vertices_retained=True,source_vertices_removed=0,sample_step_m=step,
        full_continuous_certificate=False,movement_paths_separately_validated=False,production_accepted=False),tail_lines


def review_slots(connectors,axes):
    """A plotting grid must never silently truncate an audit inventory."""
    rows=list(connectors);slots=list(axes)
    ids=[r['road'] for r in rows]
    if not ids or len(set(ids))!=len(ids):raise ValueError('nonempty unique turn inventory required')
    if len(slots)<len(rows):raise ValueError('plot grid would omit source-linked turns')
    return [(slots[i],row) for i,row in enumerate(rows)]


def run(folder):
    folder=Path(folder).resolve();trial=json.loads((folder/'report.json').read_text(encoding='utf8'))
    output=folder/'independent-review';output.mkdir(exist_ok=False)
    path=Path(trial['artifact']);unchanged(trial['source_hashes'])
    if _sha256(path)!=trial['sha256']:raise ValueError('written map changed')
    root=ET.parse(path).getroot();model,compiled,hashes=current_source_state(Path(trial['component']))
    hashes.update({str(p):_sha256(p) for p in [Path(__file__),path,folder/'report.json']})
    from mapforge.validate.source_cubic_readback import audit_written_source
    ordinary=audit_written_source(root.find(f"road[@id='{model.road}']"),compiled,model)
    dump(output/'ordinary-written-source.json',ordinary)
    interfaces=consumer_check(path,edges=True)
    src=shp_source();lat,lon=_origin(root);project=lambda p:_project(p,lat,lon)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    count=len(trial['connectors']);cols=min(5,count);nrows=int(np.ceil(count/max(1,cols)))
    if not count:raise ValueError('no source-linked turn inventory')
    fig,axes=plt.subplots(nrows,cols,figsize=(4.2*cols,4*nrows),squeeze=False,layout='constrained');records=[]
    for ax,rr in review_slots(trial['connectors'],axes.flat):
        rid=rr['road'];road=root.find(f"road[@id='{rid}']")
        raw,identity=composite_sources(root,road,src,project)
        via=raw_curves(src,road.find(".//userData[@code='mapforge.source_lane']").get('value'),project)
        actual=target_curves(road,.05);errors=fidelity(raw,actual)
        via_errors={k:stats(distances(via[k],actual[k])) for k in via}
        sub=ET.Element('OpenDRIVE');sub.append(copy.deepcopy(road));internal=internal_audit(sub)
        width=minimum_width(road);forward=written_forward_minimum(road);strip=sampled_strip_validity(actual)
        ends=[r for r in interfaces['rows'] if r['road']==rid]
        ports_ok=len(ends)==6 and all(r['status']=='PASS' for r in ends)
        ok=(source_ok([v for f in errors.values() for v in f.values()]+list(via_errors.values()))
            and width>=.1 and forward>=.1-1e-8 and strip['status']=='PASS' and internal['status']=='PASS' and ports_ok)
        records.append(dict(road=rid,geometry_status='PASS' if ok else 'FAIL',source=errors,raw_via=via_errors,
            source_identity=identity,internal_edges=internal,minimum_width_m=width,minimum_forward_factor=forward,
            strip=strip,interfaces_pass=ports_ok,source_speed_dynamics=independent_design_check(road)))
        for field in raw:
            ax.plot(*raw[field].T,c='#e88c20',lw=1.4)
            ax.plot(*actual[field].T,c='#176bbb',lw=.85,ls='--' if field=='center' else '-')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_title(f'{rid}: '+records[-1]['geometry_status'])
        print('READBACK TURN',rid,records[-1]['geometry_status'],flush=True)
    for ax in list(axes.flat)[count:]:ax.set_visible(False)
    if len(records)!=count:raise ValueError('incomplete actual source readback')
    fig.suptitle(f'ALL {count} declared turns | orange=original TOPO support; blue=actual written XODR\n'
        'Each direction median/P95/max <= 0.35/0.75/1.5m; not a whole-map acceptance.')
    fig.savefig(output/'all-turns.png',dpi=140);plt.close(fig)
    aggregate,tails=aggregate_original(root,model,compiled,records,src,project)
    dump(output/'east-full-source-aggregate.json',aggregate)
    fig,axes=plt.subplots(2,2,figsize=(17,10),layout='constrained')
    road=root.find(f"road[@id='{model.road}']");edges,_=_lane_lines(road,.05)
    for ax in axes.flat:
        for key in model.owner:ax.plot(*model.source_xy[key].T,c='#e88c20',lw=1.5)
        for xy,*_ in edges:ax.plot(*xy.T,c='#176bbb',lw=.9)
        for xy in tails:ax.plot(*xy.T,c='#9649a9',lw=.9)
        ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0,0].set_title('Complete original east scope; no source vertices removed')
    for ax,box,title in [(axes[0,1],(20,40,-15,15),'Original mouth tails / actual incident turns'),
                         (axes[1,0],(175,222,-10,14),'Real asymmetric outside end'),
                         (axes[1,1],(125,165,-15,8),'Original long tapers')]:
        ax.set_xlim(*box[:2]);ax.set_ylim(*box[2:]);ax.set_title(title)
    fig.suptitle('Orange=raw SHP | Blue=written ordinary road | Purple=original-TOPO linked written tails\n'
        f'FULL original aggregate {aggregate["status"]}: source->written {aggregate["full_source_to_written_max_m"]:.4f}m; '
        f'written->source {aggregate["full_written_to_source_max_m"]:.4f}m. NOT delivery.')
    fig.savefig(output/'east-aggregate.png',dpi=150);plt.close(fig)
    schema=LE.XMLSchema(LE.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')));xsd=schema.validate(LE.parse(str(path)))
    unchanged(hashes);unchanged(trial['source_hashes'])
    report=dict(status='REJECTED_REVIEW_NOT_DELIVERY',artifact=str(path),sha256=_sha256(path),source_hashes=hashes,
        xsd_pass=xsd,xsd_errors=str(schema.error_log),all_map_internal_edges=internal_audit(root),interfaces=interfaces,
        geometry_pass_roads=[r['road'] for r in records if r['geometry_status']=='PASS'],
        geometry_failed_roads=[r['road'] for r in records if r['geometry_status']!='PASS'],rows=records,
        east_full_source_aggregate={k:v for k,v in aggregate.items() if k not in ('rows','reverse','tail_ownership_receipts')},
        ordinary_owned_source_to_written_max_m=ordinary['ordinary_owned_source_to_written_max_m'],
        production_accepted=False,source_changed=False,whole_surface_accepted=False,MAP_retested=False)
    dump(output/'report.json',report)
    print('FULL SOURCE AGGREGATE',report['east_full_source_aggregate'],flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');run(p.parse_args().folder)
