"""Read disputed SHP records directly, without fitting or approving roles.

The contact preparation is an index, not evidence by itself. DBF rows, original
vertices, SIDE relations and endpoint node IDs are re-read with pyshp here.
Research local coordinates do not certify the absolute CRS.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import shapefile
import yaml
from mapforge.ops.source_roles import require_body
from mapforge.validate.shp_boundary_fidelity import _project
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged


def inspect_tips(lane, boundaries, contact, projection, tolerance=.35):
    """Preserve recorded point roles; no snapping, sorting or role inference."""
    if not np.isfinite(tolerance) or not 0 < tolerance <= .35:
        raise ValueError('do not relax the original source budget')
    if contact not in ('start', 'end') or set(boundaries) != {'left', 'right'}:
        raise ValueError('explicit contact and both original sides required')
    width_key = 'S_WIDTH' if contact == 'start' else 'E_WIDTH'
    value = lane['attributes'].get(width_key)
    if value is None or str(value).strip() == '' or float(value) != 0:
        raise ValueError('requires an explicit raw zero width, not a missing/default width')
    lat, lon = projection['lat_0'], projection['lon_0']
    path = _project(np.asarray(lane['points']), lat, lon)
    if len(path)<2 or not np.isfinite(path).all():raise ValueError('invalid original lane path')
    point = path[0 if contact == 'start' else -1]
    chord = path[-1]-path[0]
    tips = []
    for side, boundary in boundaries.items():
        xy = _project(np.asarray(boundary['points']), lat, lon)
        if len(xy)<2 or not np.isfinite(xy).all():raise ValueError('invalid original boundary')
        orientation = float((xy[-1]-xy[0]) @ chord)
        if abs(orientation) < 1e-8:
            raise ValueError('ambiguous original boundary direction')
        index = 0 if (contact == 'start') == (orientation > 0) else len(xy)-1
        node = boundary['attributes'].get('START_PID' if index == 0 else 'END_PID')
        if node is None or str(node).strip() == '':
            raise ValueError('missing original boundary node identity')
        tips.append(dict(side=side,xy=xy[index].tolist(),vertex_index=index,node=str(node),
                         boundary_id=str(boundary['attributes']['BORDER_PID'])))
    separation = float(np.linalg.norm(np.subtract(tips[0]['xy'], tips[1]['xy'])))
    if tips[0]['node'] != tips[1]['node'] or separation > 1e-7:
        raise ValueError('independent raw tips do not prove this exact collapsed contact')
    collapsed = np.mean([t['xy'] for t in tips], axis=0)
    gap = float(np.linalg.norm(point-collapsed))
    return dict(source_lane_id=str(lane['attributes']['LANE_PID']),contact=contact,
        raw_zero_width_field=width_key,raw_zero_width_value=value,
        raw_lane_node=str(lane['attributes']['S_LANE_PID' if contact == 'start' else 'E_LANE_PID']),
        original_boundary_tips=tips,boundary_tip_separation_m=separation,
        lane_point_xy=point.tolist(),collapsed_point_xy=collapsed.tolist(),gap_m=gap,
        source_budget_m=tolerance,conditional_minimum_common_radius_m=gap/2,
        conditional_excess_over_two_budgets_m=gap-2*tolerance,
        assumption='same zero-width endpoint is required to represent both original point roles',
        general_geometry_impossibility_proven=False,role_override_applied=False)


def original_endpoint_inventory(scope, read, compare):
    """Independent IBD raw-row census, including ends absent from TOPO events."""
    inventory, zero_ends = [], []
    for sid, obs in sorted(scope['observations'].items()):
        selected=[r for r in obs['raw_records'] if r['layer']==obs['identity_resolution']['selected_layer']]
        if len(selected)!=1:raise ValueError('ambiguous primary source lane')
        expected=selected[0];lane=read(expected['layer'],expected['record_index']);compare(lane,expected)
        for contact, field in (('start','S_WIDTH'), ('end','E_WIDTH')):
            value=lane['attributes'].get(field)
            try: width=float(value) if value is not None and str(value).strip() else None
            except (ValueError,TypeError): width=None
            known=width is not None and np.isfinite(width) and width>=0
            if known!=bool(obs[contact+'_width_known']):
                raise ValueError('raw width presence differs from prepared endpoint inventory')
            if known and abs(width-obs[contact+'_width_mm'])>1e-8:
                raise ValueError('raw endpoint width differs from prepared width')
            item=dict(source_lane_id=sid,contact=contact,raw_field=field,raw_value=value,
                      explicit_zero=known and width==0,width_known=known,
                      lane_layer=lane['layer'],record_index=lane['record_index'])
            inventory.append(item)
            if item['explicit_zero']:zero_ends.append((sid,contact,lane))
    return inventory,zero_ends


def run(directory, output=None, decision_files=()):
    directory = Path(directory).resolve()
    output = Path(output).resolve() if output else directory/'independent-role-review'
    output.mkdir(exist_ok=False)
    contacts = json.loads((directory/'contact-model.json').read_text(encoding='utf8'))
    scope = json.loads((directory/'input/reconstruction-input.json').read_text(encoding='utf8'))
    domain = json.loads((directory/'input/source-domain.json').read_text(encoding='utf8'))
    run = json.loads((directory/'input/run.json').read_text(encoding='utf8'))
    for packet in (contacts,scope,domain): require_body(packet)
    if contacts['scope_sha256'] != scope['content_sha256'] or contacts['source_domain_sha256'] != domain['content_sha256']:
        raise ValueError('source packet revision mismatch')
    hashes = dict(run['input_files_sha256'])
    decision_files=[Path(p).resolve() for p in decision_files]
    for p in [directory/'contact-model.json',directory/'input/reconstruction-input.json',
              directory/'input/source-domain.json',directory/'input/run.json',Path(__file__).resolve(),
              ROOT/'mapforge/validate/shp_boundary_fidelity.py',ROOT/'mapforge/ops/source_roles.py',
              ROOT/'profiles/repair/node4-zero-width-source-roles-v1.yaml']:
        hashes[str(p)] = _sha256(p)
    for p in decision_files:hashes[str(p)]=_sha256(p)
    unchanged(hashes)
    readers = {}
    def read(layer, index, geometry=True):
        if layer not in readers:
            readers[layer] = shapefile.Reader(str(Path(run['source_dir'])/layer), encoding='gbk')
        reader = readers[layer]
        rec = reader.record(index)
        shape = reader.shape(index) if geometry else None
        if geometry and len(shape.parts) != 1:
            raise ValueError('this reviewer does not reinterpret multipart geometry')
        return dict(attributes=dict(zip([f[0] for f in reader.fields[1:]], rec)),
                    points=[list(p[:2]) for p in shape.points] if geometry else None,record_index=index,layer=layer)
    def compare(actual, expected):
        if actual['attributes'] != expected['attributes'] or not np.array_equal(actual['points'],expected['parts'][0]):
            raise ValueError('raw DBF/SHP differs from prepared original record')
    plots=[];rows=[]
    try:
        inventory,zero_ends=original_endpoint_inventory(scope,read,compare)
        conflicts={(c['source_lane_id'],c['contact']):c for c in contacts['role_conflicts']}
        compiled={(i['source_lane_id'],i['contact']):i for i in contacts.get('source_endpoint_inventory',[])}
        for sid,contact,lane in zero_ends:
            obs=scope['observations'][sid]
            bounds={};relations=[]
            for rel in obs['boundary_relations']:
                rawrel=read(rel['relation_layer'],rel['relation_index'],geometry=False)
                if rawrel['attributes']!=rel['attributes']:raise ValueError('original SIDE relation changed')
                side={1:'left',2:'right'}.get(int(rawrel['attributes']['SIDE']))
                if side!=rel['declared_side'] or side in bounds:raise ValueError('ambiguous original SIDE')
                original=scope['boundaries'][rel['boundary_key']]['records']
                if len(original)!=1:raise ValueError('ambiguous source boundary')
                original=original[0];boundary=read(original['layer'],original['record_index']);compare(boundary,original)
                if str(rawrel['attributes']['BORDER_PID'])!=str(boundary['attributes']['BORDER_PID']):
                    raise ValueError('original boundary identity mismatch')
                bounds[side]=boundary;relations.append(rawrel)
            row=inspect_tips(lane,bounds,contact,domain['partition']['projection'])
            conflict=conflicts.get((sid,contact))
            if conflict and abs(row['gap_m']-conflict['gap_m'])>1e-8:raise ValueError('raw-tip gap disagrees with contact compiler')
            row.update(raw_lane=lane,raw_boundaries=bounds,raw_relations=relations,independent_raw_read_match=True)
            row.update(listed_in_contact_conflicts=conflict is not None,
                       listed_in_compiled_endpoint_inventory=(sid,contact) in compiled,
                       requires_source_role_decision=row['gap_m']>row['source_budget_m'])
            rows.append(row);plots.append((lane,bounds,row))
    finally:
        for reader in readers.values():reader.close()
    decision=yaml.safe_load((ROOT/'profiles/repair/node4-zero-width-source-roles-v1.yaml').read_text(encoding='utf8'))
    approved={(d['source_lane_id'],d['contact']) for d in decision['decisions']}
    for row in rows:row['listed_in_existing_north_decision']=(row['source_lane_id'],row['contact']) in approved
    for row in rows:
        row['listed_decisions']=[dict(path=str(p),source_contact_sha256=d['source_contact_sha256'],
                                     matches_current_contact_revision=d['source_contact_sha256']==contacts['content_sha256'])
            for p in decision_files for d in [yaml.safe_load(p.read_text(encoding='utf8'))]
            if any(e['source_lane_id']==row['source_lane_id'] and e['contact']==row['contact'] for e in d['decisions'])]
    report=dict(status='ORIGINAL_ENDPOINTS_REVIEWED_NOT_GEOMETRY',roads=scope['mutable_roads'],rows=rows,
        raw_endpoint_inventory=inventory,raw_lane_count=len(scope['observations']),
        raw_endpoint_count=len(inventory),explicit_zero_endpoint_count=len(rows),
        missing_compiled_conflicts=[dict(source_lane_id=r['source_lane_id'],contact=r['contact'])
            for r in rows if r['requires_source_role_decision'] and not r['listed_in_contact_conflicts']],
        decision_listing_is_not_rebinding_or_new_authorization=True,
        source_contact_sha256=contacts['content_sha256'],original_vertices_preserved=True,
        selected_roles_changed=False,geometry_solver_ran=False,xodr_generated=False,
        whole_map_accepted=False,absolute_crs_verified=False,hashes=hashes)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if not plots:raise ValueError('no original zero-width endpoints to visualize')
    fig,axes=plt.subplots(2,len(plots),figsize=(5*len(plots),8),squeeze=False,layout='constrained')
    pr=domain['partition']['projection']
    for col,(lane,bounds,row) in enumerate(plots):
        for ax in axes[:,col]:
            p=_project(np.asarray(lane['points']),pr['lat_0'],pr['lon_0'])
            ax.plot(*p.T,'o--',c='#176bbb',ms=3,label='Original lane path')
            for side,boundary in bounds.items():
                p=_project(np.asarray(boundary['points']),pr['lat_0'],pr['lon_0'])
                ax.plot(*p.T,'o-',c='#e78c1a' if side=='left' else '#228b5b',ms=3,label='Original '+side+' boundary')
            p=np.array([row['lane_point_xy'],row['collapsed_point_xy']]);ax.plot(*p.T,'r-',lw=2)
            ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.25)
            ax.set_xlabel('local x / m');ax.set_ylabel('local y / m')
        axes[0,col].set_title(row['source_lane_id']+' / '+row['raw_zero_width_field']+'=0')
        p=np.array([row['lane_point_xy'],row['collapsed_point_xy']]);lo=p.min(axis=0)-2;hi=p.max(axis=0)+2
        axes[1,col].set_xlim(lo[0],hi[0]);axes[1,col].set_ylim(lo[1],hi[1])
        axes[1,col].set_title(f"Endpoint gap {row['gap_m']:.3f}m; common radius >= {row['gap_m']/2:.3f}m")
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Roads '+','.join(scope['mutable_roads'])+': raw SHP only, no fitted curves or point edits\n'
        'Top: complete conflicting records. Bottom: endpoint close-ups. No role overrides applied; local CRS only.')
    fig.savefig(output/'raw-source-conflicts.png',dpi=150);plt.close(fig)
    unchanged(hashes)
    report['image_sha256']=_sha256(output/'raw-source-conflicts.png')
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps({k:report[k] for k in ('status','roads','source_contact_sha256')}),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source_directory');p.add_argument('--output')
    p.add_argument('--decision',action='append',default=[])
    a=p.parse_args();run(a.source_directory,a.output,a.decision)
