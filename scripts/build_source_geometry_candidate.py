"""Compile source-native ordinary-road geometry to an independently readable XODR.

Explicit geometry-first review only: source limits remain unchanged and dynamic
failures are reported, not reclassified. Existing delivered files are read-only.
"""
import argparse
import copy
import json
import sys
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from lxml import etree as ET
from mapforge.ops.source_cubic_export import cubic_model,compile_road,constrain_written_endpoints,constrain_flat_connector_port
from mapforge.ops.reconstruction_scope import digest
from scripts.fit_source_boundary_block import load,unchanged
from spikes.clarabel_joint_candidate import interior_qp
from scripts.gen_all import _sha256


def dump(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')


def restore_source_speeds(roads,src):
    """A prior generated limit is NOT the source limit; read original SHP."""
    rows=[]
    import xml.etree.ElementTree as stdET
    for road in roads:
        for lane in road.findall('lanes/laneSection/right/lane')+road.findall('lanes/laneSection/left/lane'):
            if lane.get('type')!='driving':continue
            source=lane.find("userData[@code='mapforge.source_lane']")
            if source is None:raise ValueError('connector source speed identity missing')
            sid=source.get('value');record=src.lane(sid)
            value=float(record.max_speed_kmh) if record is not None else float('nan')
            if not np.isfinite(value) or value<=0:raise ValueError('explicit original connector speed required')
            old=[dict(e.attrib) for e in lane.findall('speed')]
            if any(float(e.get('sOffset','0'))!=0 for e in lane.findall('speed')):
                raise ValueError('nonconstant generated speed requires explicit source event mapping')
            for e in lane.findall('speed'):lane.remove(e)
            speed=stdET.Element('speed',sOffset='0',max=format(value/3.6,'.17g'),unit='m/s')
            # XSD: speed after roadMark/material/visibility/friction and before
            # access/height/rule/userData.
            index=next((i for i,e in enumerate(lane) if e.tag in ('access','height','rule','userData','include')),len(lane))
            lane.insert(index,speed)
            rows.append(dict(road=road.get('id'),source_lane=sid,original_source_kmh=value,previous_generated=old,
                             source_changed=False,generated_limit_restored=True))
    return rows


def verify_review(output):
    """Independently check the actual rejected/accepted geometry review copy.

    Restore source attributes on a NEW artifact, never a historical source.
    This does not turn rejected shapes into delivery candidates.
    """
    import xml.etree.ElementTree as stdET
    from scripts.gen_all import shp_source
    from scripts.esmini_lane_interfaces import check
    from scripts.internal_edge_jets import audit as internal_edges
    from scripts.recheck_speed_contract import check_limits
    from mapforge.validate.g11 import load_policy,_audit_d
    from spikes.trial_xml import write_trial
    output=Path(output).resolve();checkpoint=json.loads((output/'shared-state.json').read_text(encoding='utf8'))
    unchanged(checkpoint['source_hashes'])
    review=json.loads((output/'junction-review.json').read_text(encoding='utf8'))
    source=output/'node4-review.xodr'
    if _sha256(source)!=review['sha256']:raise ValueError('review file changed since connector solve')
    tree=stdET.parse(source);root=tree.getroot();ids={r['road'] for r in review['connectors']}
    selected=[r for r in root.findall('road') if r.get('id') in ids]
    speed_rows=restore_source_speeds(selected,shp_source())
    target=output/'node4-source-speed-review.xodr';write_trial(tree,target)
    actual=stdET.parse(target).getroot()
    scope=stdET.Element('OpenDRIVE')
    for r in actual.findall('road'):
        if r.get('id') in ids:scope.append(copy.deepcopy(r))
    limits=check_limits(scope,{r['source_lane']:r['original_source_kmh'] for r in speed_rows})
    # Include rebuilt ordinary road for source-speed dynamics. The other
    # unchanged ordinary roads are not misrepresented as reconstructed.
    scope.append(copy.deepcopy(next(r for r in actual.findall('road') if r.get('id')=='10')))
    cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');cfg['dynamics']['sample_step_m']=.02
    dynamics=_audit_d(scope,cfg)
    schema=ET.XMLSchema(ET.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    xsd=schema.validate(ET.parse(target))
    try:interfaces=check(target,edges=True)
    except (ValueError,RuntimeError) as exc:interfaces=dict(status='FAIL',error=str(exc))
    report=dict(status='REJECTED_REVIEW_NOT_DELIVERY',artifact=str(target),sha256=_sha256(target),
                source_speed_restoration=speed_rows,connector_speed_integrity=limits,
                source_speed_dynamics=dynamics,dynamics_scope=['10',*sorted(ids)],
                old_connector_selection_dynamics_not_authoritative=True,
                internal_edges=internal_edges(actual),esmini_interfaces=interfaces,xsd_pass=xsd,
                rejected_connectors=[r['road'] for r in review['connectors'] if r['status']=='REJECTED'],
                source_hashes=checkpoint['source_hashes'],source_changed=False,production_accepted=False,
                reviewed_code_sha256={str(p):_sha256(p) for p in [Path(__file__).resolve(),
                    ROOT/'mapforge/ops/source_cubic_export.py',ROOT/'mapforge/validate/source_cubic_readback.py',
                    ROOT/'spikes/measured_connector_caps.py',ROOT/'spikes/arc_source_boundary.py',
                    ROOT/'spikes/connector_cross_section.py',
                    ROOT/'scripts/esmini_lane_interfaces.py',ROOT/'scripts/internal_edge_jets.py']})
    unchanged(checkpoint['source_hashes']);dump(output/'review-validation.json',report)
    print('INDEPENDENT REVIEW',target,'XSD',xsd,'SOURCE SPEED',limits['status'],
          'INTERFACES',interfaces.get('status'),'DYNAMICS',dynamics['status'],flush=True)
    return report


def retry_connectors(output, road_ids):
    """Retry explicitly rejected incident roads; keep the parent state fixed."""
    import xml.etree.ElementTree as stdET
    from scripts.gen_all import shp_source
    from spikes.measured_connector_caps import frames,composite_sources,raw_curves,fit
    from mapforge.validate.shp_boundary_fidelity import _origin,_project
    from spikes.trial_xml import write_trial,replace_road
    from mapforge.validate.smoothness import junction_lane_interfaces
    output=Path(output).resolve();report=json.loads((output/'junction-review.json').read_text(encoding='utf8'))
    source=output/'node4-review.xodr'
    if _sha256(source)!=report['sha256']:raise ValueError('changed input review')
    rows={r['road']:r for r in report['connectors']}
    if any(rid not in rows or rows[rid]['status']!='REJECTED' for rid in road_ids):
        raise ValueError('only explicitly rejected connectors may be retried')
    root=stdET.parse(source).getroot();src=shp_source();lat,lon=_origin(root)
    for rid in road_ids:
        road=next(r for r in root.findall('road') if r.get('id')==rid)
        restored=restore_source_speeds([road],src)
        sid=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']").get('value')
        via=raw_curves(src,sid,lambda p:_project(p,lat,lon))
        raw,support=composite_sources(root,road,src,lambda p:_project(p,lat,lon));a,b=frames(root,road)
        new,row=fit(road,a,b,raw,adaptive_caps=True,source_warm_start=True,raw_via=via,require_dynamics=False)
        row.update(source_support=support,source_speed_restoration=restored,previous_rejection=rows[rid])
        rows[rid]=row;replace_road(root,road,new);print('RETRIED',rid,row['status'],flush=True)
    write_trial(stdET.ElementTree(root),source)
    report.update(connectors=list(rows.values()),sha256=_sha256(source),interfaces=junction_lane_interfaces(root))
    dump(output/'junction-review.json',report);dump(output/'connectors-progress.json',list(rows.values()))
    return verify_review(output)


def run(output,resume=False,junction=False,end_axis=False,minimum_width_span=6.,flat_port=False):
    output=Path(output).resolve()
    if output.exists() and not resume:raise FileExistsError('use new output directory, or explicit resume')
    model,roles,files=load(ROOT/'out/source-role-input-v151',ROOT/'profiles/repair/node4-zero-width-source-roles-v1.yaml',3)
    model=cubic_model(model,minimum_span=minimum_width_span,end_axis=end_axis)
    if end_axis:model=constrain_written_endpoints(model)
    if flat_port:model=constrain_flat_connector_port(model)
    print('AXIS',model.export_axis_selection,flush=True)
    output.mkdir(parents=True,exist_ok=True)
    checkpoint=output/'shared-state.json'
    if resume and checkpoint.exists():
        state=json.loads(checkpoint.read_text(encoding='utf8'))
        if state['model_sha']!=digest(model.describe()):raise ValueError('saved source/model changed')
        x=np.array(state['coefficients']);phase=state['phase'];unchanged(state['source_hashes'])
    else:
        with patch('spikes.source_contact_fit._convex_qp',interior_qp):x,phase=model.solve()
        if x is None:
            dump(output/'rejection.json',dict(status='REJECTED_NOT_DELIVERY',phase=phase,model=model.describe(),
                 axis=model.export_axis_selection,source_hashes=files,production_accepted=False))
            raise ValueError('direct cubic source geometry rejected: '+str(phase))
        dump(checkpoint,dict(model_sha=digest(model.describe()),coefficients=x.tolist(),phase=phase,source_hashes=files))
    print('SHARED CUBIC SOLVED',model.nvar,flush=True)
    source=ROOT/'out/shp-shared-section-final-v145/node4.xodr'
    root=ET.parse(str(source)).getroot();original=next(r for r in root.findall('road') if r.get('id')==model.road)
    road,compiled,ports=compile_road(model,x,original)
    only=ET.Element('OpenDRIVE');only.append(copy.deepcopy(root.find('header')));only.append(copy.deepcopy(road))
    # Component is explicitly isolated, not an apparently connected full map.
    for e in only.find('road').findall('link'):only.find('road').remove(e)
    path=output/'north-road.xodr';ET.ElementTree(only).write(str(path),encoding='utf-8',xml_declaration=True,pretty_print=True)
    schema=ET.XMLSchema(ET.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    valid=schema.validate(ET.parse(str(path)))
    if not valid:raise ValueError(str(schema.error_log))
    from mapforge.validate.g11 import load_policy,_audit_d
    import xml.etree.ElementTree as stdET
    cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');cfg['dynamics']['sample_step_m']=.02
    dynamics=_audit_d(stdET.parse(path).getroot(),cfg)
    source_audit=model.audit(x)
    from mapforge.validate.source_cubic_readback import audit_written_source
    readback=audit_written_source(stdET.parse(path).getroot().find('road'),compiled,model)
    dump(output/'written-source-audit.json',readback)
    report=dict(status='GEOMETRY_COMPONENT_REVIEW_NOT_COMPLETE_MAP',compiled=compiled,
        source_model_audit=source_audit,written_source_audit_status=readback['status'],
        axis_selection=model.export_axis_selection,written_endpoint_constraints=getattr(model,'written_endpoint_constraints',0),
        connector_port_hypothesis=getattr(model,'connector_port_hypothesis',None),
        dynamics_readback=dynamics,xsd_pass=valid,
        source_speed_changed=False,source_changed=False,whole_junction_accepted=False,production_accepted=False,
        external_port_map=[dict(contact=c,old_lane=o,new_lane=n) for (c,o),n in ports.items()],
        original_sha=_sha256(source),component_sha=_sha256(path))
    dump(output/'report.json',report)
    from scripts.basemap_overlay import _lane_lines
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,8))
    actual=stdET.parse(path).getroot().find('road');edges,centers=_lane_lines(actual,.05)
    raw=model.source_xy
    for ax in axes:
        for k,xy in raw.items():
            if k.startswith('boundary:'):ax.plot(*xy.T,color='#ed8c22',lw=1.2)
        for xy,*_ in edges:ax.plot(*xy.T,color='#176eae',lw=.9)
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('local x / m');ax.set_ylabel('local y / m')
    axes[0].set_title('Written XODR readback / original SHP')
    for ax,event in zip(axes[1:],model.contacts['role_conflicts']):
        px,py=event['collapsed_boundary_xy'];ax.set_xlim(px-6,px+6);ax.set_ylim(py-14,py+14);ax.set_title('Source split/merge close-up')
    fig.suptitle('Orange: original boundaries / Blue: WRITTEN lane edges\nGeometry component only; full junction and source-speed dynamics NOT accepted')
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(output/'north-readback.png',dpi=150);plt.close(fig)
    unchanged(files)
    print('WRITTEN',path,'XSD',valid,'SOURCE',readback['status'],'DYNAMICS',dynamics.get('status'),flush=True)
    if junction:
        # Replace the road and ALL its dependent connectors. Failed connector
        # states remain explicitly rejected; never retain the old connector
        # and claim it attaches to the newly compiled parent.
        from spikes.measured_connector_caps import frames,composite_sources,raw_curves,fit
        from scripts.gen_all import shp_source
        from mapforge.validate.shp_boundary_fidelity import _origin,_project
        from spikes.trial_xml import replace_road,write_trial
        from mapforge.validate.smoothness import junction_lane_interfaces
        staged=stdET.fromstring(ET.tostring(root));parent=next(r for r in staged.findall('road') if r.get('id')==model.road)
        replace_road(staged,parent,stdET.fromstring(ET.tostring(road)))
        affected=[r for r in staged.findall('road') if r.get('junction')!='-1' and any(
            e.get('elementType')=='road' and e.get('elementId')==model.road for e in r.findall('link/*'))]
        src=shp_source();lat,lon=_origin(staged);rows=[]
        for connector in affected:
            rid=connector.get('id');ln=connector.find('lanes/laneSection/right/lane')
            for role in ('predecessor','successor'):
                link=connector.find('link/'+role)
                if link.get('elementId')==model.road:
                    ll=ln.find('link/'+role);ll.set('id',str(ports[(link.get('contactPoint'),int(ll.get('id')))]))
            for jc in staged.findall('junction/connection'):
                if jc.get('connectingRoad')==rid and jc.get('incomingRoad')==model.road:
                    cp=connector.find('link/predecessor').get('contactPoint')
                    for ll in jc.findall('laneLink'):ll.set('from',str(ports[(cp,int(ll.get('from')))]))
            try:
                sid=ln.find("userData[@code='mapforge.source_lane']").get('value')
                restored=restore_source_speeds([connector],src)
                via=raw_curves(src,sid,lambda p:_project(p,lat,lon))
                raw,support=composite_sources(staged,connector,src,lambda p:_project(p,lat,lon))
                a,b=frames(staged,connector)
                new,row=fit(connector,a,b,raw,adaptive_caps=True,source_warm_start=True,raw_via=via,require_dynamics=False)
                row['source_support']=support;row['source_speed_restoration']=restored;replace_road(staged,connector,new)
            except (ValueError,AttributeError,IndexError) as exc:
                row=dict(road=rid,status='REJECTED',reason=str(exc),geometry_rebuilt=False)
            rows.append(row);dump(output/'connectors-progress.json',rows)
            print('CONNECTOR',rid,row['status'],row.get('reason',''),flush=True)
        # An isolated review copy is not a promoted mixed pass/fail map.
        target=output/'node4-review.xodr';write_trial(stdET.ElementTree(staged),target)
        interfaces=junction_lane_interfaces(staged)
        dump(output/'junction-review.json',dict(status='REJECTED_REVIEW_NOT_DELIVERY',connectors=rows,
             affected_connectors=len(affected),interfaces=interfaces,sha256=_sha256(target),
             unchanged_roads_are_not_new_solutions=True,production_accepted=False))
        unchanged(files)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--resume',action='store_true');p.add_argument('--junction',action='store_true');p.add_argument('--source-end-axis',action='store_true');p.add_argument('--minimum-width-span',type=float,default=6.);p.add_argument('--flat-junction-port',action='store_true');p.add_argument('--verify-only',action='store_true');p.add_argument('--retry-connector',action='append',default=[]);a=p.parse_args()
    if a.retry_connector:retry_connectors(a.output,a.retry_connector)
    elif a.verify_only:verify_review(a.output)
    else:run(a.output,a.resume,a.junction,a.source_end_axis,a.minimum_width_span,a.flat_junction_port)
