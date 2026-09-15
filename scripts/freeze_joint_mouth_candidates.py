"""Reproducible three-case joint-mouth evidence, isolated from formal output."""
import hashlib
import json
import sys
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from lxml import etree
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from spikes.joint_mouth_candidate import run
from spikes.map_corner_surface import build as surface
from scripts.internal_edge_jets import audit as internal_edges
from scripts.map_paving_audit import inspect as outline
from scripts.esmini_lane_interfaces import check as consumer
from scripts.freeze_map_surface_candidates import signature
from scripts.visual_sweep import capture
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.g11 import load_policy,_audit_d


def source_plot(path, baseline, output, *, baseline_title='v1.37 baseline',
                candidate_title='joint mouth candidate'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from scripts.basemap_overlay import _lane_lines, _map_road_center_layer, _EqcInverse
    from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all
    from mapforge.validate.shp_boundary_fidelity import _point_polyline_distance
    from mapforge.validate.smoothness import _sections
    root=ET.parse(path).getroot(); old=ET.parse(baseline).getroot()
    manifest=json.loads(path.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
    inverse=_EqcInverse(root.findtext('header/geoReference'))
    raw_lanes={}
    for xml in (ROOT/'v2x_map_xml').glob('map*.xml'):
        for node in parse_map_xml_all(str(xml)):
            for link in node.links:
                for lane in link.lanes:
                    key=(node.region,node.node_id,tuple(link.upstream),link.name,lane.lane_id)
                    if key in raw_lanes:raise ValueError('ambiguous raw MAP lane identity')
                    raw_lanes[key]=np.array(lane.points)
    original_points=[]
    for ln in manifest['lanes']:
        owner=ln['owner']
        key=(owner['region'],owner['node'],tuple(owner['upstream']),owner['link'],owner['lane'])
        p=raw_lanes[key]
        if len(p)==0:continue
        xy=np.column_stack([np.radians(p[:,0]-inverse.lon0)*inverse.radius*np.cos(np.radians(inverse.lat_ts)),
                            np.radians(p[:,1]-inverse.lat0)*inverse.radius])
        original_points.append((ln['source_lane_id'],xy))
    target_lines={}
    for road in root.findall('road'):
        sections=_sections(road)
        _,centers=_lane_lines(road,.1)
        for pts,side,index,ordinal in centers:
            lid=sections[index][2 if side=='left' else 1][ordinal-1][0]
            lane=road.findall('lanes/laneSection')[index].find(f"{side}/lane[@id='{lid}']")
            identity=lane.find("userData[@code='mapforge.source_lane']")
            if identity is not None:
                target_lines.setdefault(identity.get('value'),[]).append(pts)
    coverage=[]
    for sid,pts in original_points:
        if sid not in target_lines:raise ValueError('missing raw lane target identity: '+sid)
        distances=np.min([_point_polyline_distance(pts,p)[0] for p in target_lines[sid]],axis=0)
        coverage.append({'source_lane_id':sid,'raw_vertex_count':len(pts),
                         'max_m':float(max(distances)),'p95_m':float(np.percentile(distances,95)),
                         'vertices_beyond_1m':int(sum(distances>1.)),
                         'distances_m':distances.tolist()})
    fig,axs=plt.subplots(1,3,figsize=(19,7),dpi=160)
    for ax,model,title in zip(axs,(old,root,root),(baseline_title,candidate_title,'junction close-up')):
        for r in model.findall('road'):
            if r.get('name')=='junction_paving':continue
            edges,centers=_lane_lines(r,.15)
            for pts,*_ in edges:ax.plot(*pts.T,color='#a5b5bf',lw=.45)
            for pts,*_ in centers:ax.plot(*pts.T,color='#1675be',lw=.8)
        for sid,pts in original_points:
            ax.plot(*pts.T,color='#dc8f05',lw=1.1,marker='o',ms=2)
        for feature in _map_road_center_layer(manifest)['features']:
            p=np.array(feature['geometry']['coordinates'])
            xy=np.column_stack([np.radians(p[:,0]-inverse.lon0)*inverse.radius*np.cos(np.radians(inverse.lat_ts)),
                                np.radians(p[:,1]-inverse.lat0)*inverse.radius])
            ax.plot(*xy.T,color='#269636',lw=1.,marker='.',ms=3)
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set(title=title,xlabel='local x (m)',ylabel='local y (m)')
    axs[-1].set(xlim=(-45,45),ylim=(-45,45))
    fig.suptitle(path.stem+' | orange = raw Lane.points; green = raw Link.points; blue = final lane centers\n'
                 'Local source comparison, not absolute CRS/surveyed curb certification')
    fig.tight_layout(rect=(0,0,1,.94));output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output);plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps({'source':'original XML Lane.points, not manifest cropped support',
        'raw_vertex_fidelity':coverage,
        'lanes':[{'source_lane_id':sid,'raw_vertex_count':len(pts),'raw_local_xy':pts.tolist()}
                 for sid,pts in original_points]},ensure_ascii=False,indent=2),encoding='utf-8')


def screenshots(path, output):
    tiles=[]
    with tempfile.TemporaryDirectory(prefix='mapforge_joint_view_') as tmp:
        for label,camera in (('whole','0,-50,600,1.5708,1.45'),('mouths','0,-20,85,1.5708,1.55'),
                              ('oblique','40,-60,50,2.2,0.6')):
            frame=capture(path.resolve(),['--density','0','--ground_plane','off','--camera_mode','custom_fixed',
                                         '--custom_fixed_camera',camera],Path(tmp))
            if frame is None:raise RuntimeError('no esmini screenshot: '+label)
            with Image.open(frame) as image:
                tile=image.convert('RGB').resize((960,540))
            ImageDraw.Draw(tile).text((12,12),path.stem+' / '+label,fill=(255,50,50))
            tiles.append(tile)
        sheet=Image.new('RGB',(2880,540))
        for i,tile in enumerate(tiles):sheet.paste(tile,(960*i,0))
        sheet.save(output)


def main():
    directory=ROOT/'out/map-joint-review-v138';directory.mkdir(parents=True,exist_ok=True)
    images=ROOT/'out/preview/map-joint-review-v138';images.mkdir(parents=True,exist_ok=True)
    policy=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m']=.02
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    summary=[]
    for name in ('node3','NODE5','node13'):
        source=ROOT/'out/map-surface-review-v137'/(name+'.xodr');target=directory/(name+'.xodr')
        if not run(source,target):
            raise RuntimeError(f'{name}: current fit rejected; do not validate an older target')
        root=ET.parse(target).getroot();old=ET.parse(source).getroot()
        paving=surface(root)
        for r in old.findall('road'):
            if r.get('name')=='junction_paving':continue
            new=root.find(f"road[@id='{r.get('id')}']")
            if r.get('junction')=='-1':assert signature(r.find('planView'))==signature(new.find('planView'))
            assert signature(r.find('link'))==signature(new.find('link'))
            for selector in ('.//lane/link','.//lane/speed'):
                assert [signature(e) for e in r.findall(selector)]==[signature(e) for e in new.findall(selector)]
        assert signature(old.find('junction'))==signature(root.find('junction'))
        ET.indent(root);ET.ElementTree(root).write(target,encoding='utf-8',xml_declaration=True)
        manifest=json.loads(target.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
        from scripts.gen_all import CASES
        final=finalize_opendrive_g8(target,manifest,ROOT/'profiles/validation/g8-opendrive-jinfeng-v1.yaml',
                                   raw_map_paths=[ROOT/'v2x_map_xml'/file for _,file in CASES])
        dense=_audit_d(root,policy); inside=internal_edges(root); engine=consumer(target,edges=True)
        envelope=outline(target,images/(name+'-outline.png'))
        source_plot(target,source,images/(name+'-source.png'))
        screenshots(target,images/(name+'-esmini.png'))
        report={'case':name,'candidate_only':True,'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
                'G8':final['gate']['status'],'G11':final['g11']['status'],
                'raw_source_integrity':final['quality']['gates']['G8-source-integrity']['status'],
                'edge_contacts':final['edge_contacts']['status'],'internal_edges':inside['status'],
                'dense_2cm':dense['status'],'esmini_center_and_edges':engine['status'],
                'esmini_contacts':len(engine['rows']),'xsd':schema.validate(etree.parse(str(target))),
                'max_jerk_mps3':max(r['lateral_jerk_mps3'] for r in dense['roads']),
                'edge_contact_maxima':final['edge_contacts']['maxima'],
                'internal_edge_maxima':inside['maxima'],
                'surface':paving,'envelope':{k:v for k,v in envelope.items() if k!='mouths'},
                'delivery':'BLOCKED','unchanged_speeds_topology_and_ordinary_references':True}
        decision=final['decision'];decision['candidate_only']=True;decision['status']='BLOCKED'
        decision['blocked_reasons'] += [{'code':'candidate_not_promoted'}, {'code':'sim_auxiliary_only_not_ad_strict'}]
        for k in ('internal_edges','dense_2cm','esmini_center_and_edges'):
            if report[k]!='PASS':decision['blocked_reasons'].append({'code':k+'_failed'})
        if not report['xsd']:decision['blocked_reasons'].append({'code':'xsd_failed'})
        final['quality']['delivery_decision']=decision
        for suffix,data in (('.dense-all.json',dense),('.internal-edges.json',inside),('.verification.json',report),
                            ('.delivery-decision.json',decision),('.quality-report.json',final['quality'])):
            target.with_suffix(suffix).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        summary.append(report);print('FROZEN',name,report,flush=True)
    (directory/'review-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    refresh_semantics(directory)


def refresh_semantics(directory):
    """Attach the raw-reference review finding without altering any XODR."""
    from scripts.map_connection_semantics import inspect
    raw = [inspect(p) for p in sorted((ROOT/'v2x_map_xml').glob('map*.xml'))]
    summaries=[]
    for name in ('node3','NODE5','node13'):
        path=directory/(name+'.xodr')
        before=hashlib.sha256(path.read_bytes()).hexdigest()
        manifest=json.loads(path.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
        owners={(x['region'],x['node_id']) for x in manifest['source_contexts']}
        concerns=[dict(row,source=part['source'],source_sha256=part['sha256'])
                  for part in raw for row in part['rows']
                  if tuple(row['owner']) in owners and row['same_upstream_ref']]
        finding={'status':'REVIEW_REQUIRED' if concerns else 'NO_SAME_UPSTREAM_REFERENCE_FOUND',
                 'scope':'raw remote/upstream equality, not full topology validation','rows':concerns}
        report=json.loads(path.with_suffix('.verification.json').read_text(encoding='utf-8'))
        report['source_semantics_review']=finding
        source_evidence=ROOT/'out/preview/map-joint-review-v138'/(name+'-source.json')
        vertices=json.loads(source_evidence.read_text(encoding='utf-8'))['raw_vertex_fidelity']
        raw_review={'scope':'original XML vertices to same-identity written lane; not dense all-source certification',
                    'status':'REVIEW_REQUIRED' if any(r['vertices_beyond_1m'] for r in vertices) else 'NO_VERTEX_OVER_1M',
                    'rows':vertices}
        report['raw_source_vertex_review']=raw_review
        decision=json.loads(path.with_suffix('.delivery-decision.json').read_text(encoding='utf-8'))
        if concerns and not any(x.get('code')=='source_same_upstream_reference_review' for x in decision['blocked_reasons']):
            decision['blocked_reasons'].append({'code':'source_same_upstream_reference_review','count':len(concerns)})
        if raw_review['status']=='REVIEW_REQUIRED' and not any(x.get('code')=='raw_source_vertices_outside_written_lane' for x in decision['blocked_reasons']):
            decision['blocked_reasons'].append({'code':'raw_source_vertices_outside_written_lane',
                'count':sum(r['vertices_beyond_1m'] for r in vertices),'maximum_m':max(r['max_m'] for r in vertices)})
        quality=json.loads(path.with_suffix('.quality-report.json').read_text(encoding='utf-8'))
        quality['delivery_decision']=decision;quality['source_semantics_review']=finding
        quality['raw_source_vertex_review']=raw_review
        for suffix,data in (('.verification.json',report),('.delivery-decision.json',decision),('.quality-report.json',quality)):
            path.with_suffix(suffix).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
        assert hashlib.sha256(path.read_bytes()).hexdigest()==before
        summaries.append(report)
    (directory/'review-summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    if sys.argv[1:]==['--semantics-only']:refresh_semantics(ROOT/'out/map-joint-review-v138')
    elif sys.argv[1:]:raise SystemExit('usage: freeze_joint_mouth_candidates.py [--semantics-only]')
    else:main()
