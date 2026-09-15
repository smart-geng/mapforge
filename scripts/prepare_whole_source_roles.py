"""Bind prior explicit source-role reviews to freshly read whole-junction data.

Never fit geometry, modify raw sources, or grant missing role permissions.
Historical compiler output is NOT rerun as if it were today's compiler:
its saved manifest/decision is checked, then each selected raw fact is compared
to the complete freshly rebuilt target. Missing decisions remain a hard gate.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import shapefile
import yaml
from lxml import etree as ET
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.ops.source_roles import compose_reviewed_source_roles,replay_source_role_packet
from mapforge.ops.reconstruction_scope import model_scope_report
from mapforge.validate.shp_boundary_fidelity import _project
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.prepare_joint_reconstruction import verify_preparation
from scripts.review_source_role_conflicts import inspect_tips


def read(path):return json.loads(Path(path).read_text(encoding='utf8'))


def dump(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')


def load_review(directory,decision_path,hashes):
    directory,decision_path=Path(directory).resolve(),Path(decision_path).resolve()
    manifest=read(directory/'run.json'); old=read(directory/'input/run.json')
    if _sha256(directory/'contact-model.json')!=manifest['model_file_sha256']:
        raise ValueError('historical reviewed contact file changed')
    if set(manifest['preparation_files_sha256'])!={'run.json','reconstruction-input.json','source-domain.json'}:
        raise ValueError('incomplete historical preparation manifest')
    for name,sha in manifest['preparation_files_sha256'].items():
        if _sha256(directory/'input'/name)!=sha:raise ValueError('historical reviewed input changed')
    for name,sha in old['input_files_sha256'].items():
        if name in hashes and hashes[name]!=sha:raise ValueError('conflicting historical/current source hash')
        hashes[name]=sha
    paths=[decision_path,directory/'run.json',directory/'contact-model.json']
    paths += [directory/'input'/name for name in manifest['preparation_files_sha256']]
    hashes.update({str(p):_sha256(p) for p in paths});unchanged(hashes)
    return dict(scope=read(directory/'input/reconstruction-input.json'),
                domain=read(directory/'input/source-domain.json'),contacts=read(directory/'contact-model.json'),
                decision=yaml.safe_load(decision_path.read_text(encoding='utf8')))


def independent_pending(scope,domain,contacts,roles,source_dir):
    """Direct SHP/DBF endpoint check, independent of generated map geometry."""
    readers={};rows=[]
    def record(expected):
        layer=expected['layer'];index=expected['record_index']
        if layer not in readers:readers[layer]=shapefile.Reader(str(Path(source_dir)/layer),encoding='gbk')
        r=readers[layer]; shape=r.shape(index)
        if len(shape.parts)!=1:raise ValueError('pending review requires explicit multipart interpretation')
        attrs=dict(zip([f[0] for f in r.fields[1:]],r.record(index)))
        points=[list(p[:2]) for p in shape.points]
        if attrs!=expected['attributes'] or points!=expected['parts'][0]:
            raise ValueError('independent raw source record changed')
        return dict(layer=layer,record_index=index,attributes=attrs,points=points)
    try:
        for conflict in roles['unresolved']:
            sid=conflict['source_lane_id'];obs=scope['observations'][sid]
            primary=[r for r in obs['raw_records'] if r['layer']==obs['identity_resolution']['selected_layer']]
            if len(primary)!=1:raise ValueError('ambiguous original lane record')
            lane=record(primary[0]);boundaries={}
            for rel in obs['boundary_relations']:
                records=scope['boundaries'][rel['boundary_key']]['records']
                if len(records)!=1 or rel['declared_side'] in boundaries:raise ValueError('ambiguous original side relation')
                boundaries[rel['declared_side']]=record(records[0])
            check=inspect_tips(lane,boundaries,conflict['contact'],domain['partition']['projection'])
            if abs(check['gap_m']-conflict['gap_m'])>1e-10:raise ValueError('independent source tip evidence disagrees')
            rows.append(dict(check,logical_consumers=contacts['source_bindings'][sid],
                raw_lane=lane,raw_boundaries=boundaries,proposed_decision=None))
    finally:
        for reader in readers.values():reader.close()
    return rows


def draw_pending(rows,projection,output):
    if not rows:return
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,len(rows),figsize=(6*len(rows),8),squeeze=False,layout='constrained')
    for col,row in enumerate(rows):
        tip=np.array(row['collapsed_point_xy'])
        series=[('original lane path',row['raw_lane'],'#1565b0','--'),
                ('original left edge',row['raw_boundaries']['left'],'#d87812','-'),
                ('original right edge',row['raw_boundaries']['right'],'#188050','-')]
        for ax in axes[:,col]:
            for label,record,color,style in series:
                xy=_project(np.asarray(record['points']),projection['lat_0'],projection['lon_0'])-tip
                ax.plot(*xy.T,color=color,linestyle=style,marker='o',ms=3,label=label)
            gap=np.array([row['lane_point_xy'],row['collapsed_point_xy']])-tip
            ax.plot(*gap.T,'r-',lw=2,label='point-role separation')
            ax.scatter(*gap[0],s=45,color='#1565b0',zorder=5)
            ax.scatter(0,0,s=50,color='black',marker='x',zorder=6)
            ax.set_aspect('equal');ax.grid(alpha=.2)
            ax.set_xlabel('x relative to original boundary tip / m')
            ax.set_ylabel('y relative to original boundary tip / m')
        axes[0,col].set_title('/'.join(row['logical_consumers'])+'\n'+row['source_lane_id'],fontsize=11)
        lo=gap.min(axis=0)-2;hi=gap.max(axis=0)+2
        axes[1,col].set_xlim(lo[0],hi[0]);axes[1,col].set_ylim(lo[1],hi[1])
        axes[1,col].set_title(f"Original {row['raw_zero_width_field']}=0; gap={row['gap_m']:.3f}m",fontsize=11)
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Pending original point roles: full records above, endpoint detail below. NO edits / NO fitted XODR.')
    fig.savefig(output/'pending-originals.png',dpi=130);plt.close(fig)


def run(preparation,output,review_paths):
    preparation,output=Path(preparation).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    config=read(preparation/'run.json');hashes=dict(config['input_files_sha256'])
    code=['scripts/prepare_whole_source_roles.py','scripts/prepare_joint_reconstruction.py',
          'scripts/review_source_role_conflicts.py','mapforge/ops/source_roles.py',
          'mapforge/ops/source_contacts.py','mapforge/ops/reconstruction_scope.py','mapforge/ops/source_domain.py']
    files=[ROOT/p for p in code]+[preparation/p for p in ('run.json','reconstruction-input.json','source-domain.json')]
    hashes.update({str(p):_sha256(p) for p in files});unchanged(hashes)
    dump(output/'check-contract.json',dict(status='WHOLE_SOURCE_ROLE_REPLAY_ONLY',
        fitting_budget=0,geometry_export=False,new_authorizations=0,
        review_paths=[[str(Path(p).resolve()) for p in pair] for pair in review_paths],
        stop_conditions=['changed original source/role facts','missing existing approval evidence','unresolved point-role binding']))
    verify_preparation(preparation)
    if 'whole_junction' not in config:raise ValueError('explicit complete junction preparation required')
    scope=read(preparation/'reconstruction-input.json');domain=read(preparation/'source-domain.json')
    root=ET.parse(config['input']).getroot();source=ProfileSource(config['source_dir'],config['profile'])
    contacts=compile_source_contacts(root,source,scope,domain,chart_mode='exact-line-arc-v1')
    bundles=[load_review(directory,decision,hashes) for directory,decision in review_paths]
    roles=compose_reviewed_source_roles(scope,domain,contacts,bundles)
    replay_source_role_packet(scope,domain,contacts,roles)
    rows=independent_pending(scope,domain,contacts,roles,config['source_dir'])
    jid=config['whole_junction']['xodr_junction_id']
    parents=[]
    for rid in scope['mutable_roads']:
        pending=[r for r in rows if 'road:'+rid in r['logical_consumers']]
        parents.append(dict(road=rid,source_ids=sorted(sid for sid,owners in contacts['source_bindings'].items() if 'road:'+rid in owners),
            source_role_status='SOURCE_ROLE_DECISION_REQUIRED' if pending else 'SOURCE_ROLES_REPLAYED_OR_ORIGINAL',
            unresolved=[r['source_lane_id'] for r in pending],geometry_model_built=False))
    packet=dict(status='SOURCE_ROLE_DECISION_REQUIRED' if rows else 'SOURCE_ROLES_READY_NOT_GEOMETRY',
        scope=model_scope_report(root,jid,scope['mutable_roads'],scope['connectors']),
        parents=parents,source_features=len(contacts['full_source_support']),original_endpoint_count=len(contacts['source_endpoint_inventory']),
        prior_decisions_replayed=[dict(source_lane_id=r['source_lane_id'],contact=r['contact']) for r in roles['resolved']],
        pending=rows,source_role_binding_complete=roles['role_binding_complete'],
        new_source_role_authorizations=0,source_modified=False,geometry_solver_ran=False,xodr_generated=False,production_accepted=False,
        pending_independently_read=True,input_files_sha256=hashes)
    draw_pending(rows,domain['partition']['projection'],output)
    unchanged(hashes)
    dump(output/'whole-contact-model.json',contacts);dump(output/'source-roles.json',roles);dump(output/'report.json',packet)
    print(packet['status'],'replayed',len(roles['resolved']),'pending',len(rows),'hashes',len(hashes))
    print([(r['logical_consumers'],r['source_lane_id'],r['gap_m']) for r in rows])
    return packet


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('preparation');p.add_argument('output');p.add_argument('--review',nargs=2,action='append',required=True,
        metavar=('ORIGINAL_SOURCE_DIRECTORY','DECISION_YAML'))
    a=p.parse_args();run(a.preparation,a.output,a.review)
    raise SystemExit(2)
