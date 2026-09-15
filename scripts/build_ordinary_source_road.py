"""Compile a declared ordinary source block; do not promote or mix port states.

Unlike the historical north-specific driver, road and source-role decisions
are explicit. The baseline XODR and original materials remain read-only.
"""
import argparse
import copy
import json
import sys
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import yaml
from lxml import etree as ET
from scripts.prepare_source_contacts import verify_contacts
from scripts.fit_source_boundary_block import unchanged
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from mapforge.ops.source_roles import resolve_source_roles,resolve_original_source_roles,rebind_unchanged_source_decision
from mapforge.ops.reconstruction_scope import digest
from spikes.source_contact_fit import SourceBoundaryBlock
from spikes.clarabel_joint_candidate import interior_qp
from mapforge.ops.source_cubic_export import (cubic_model,constrain_written_endpoints,
    constrain_flat_connector_port,compile_road)


class SourceSpeedBoundaryBlock(SourceBoundaryBlock):
    """Source-node speed events explicitly enabled only for this new driver."""
    source_speed_boundary_policy='original-lane-node'

    def __init__(self,*args,speed_node_fields=None,**kwargs):
        # Arc reconstruction copies the instance state, not class attributes.
        self.source_speed_boundary_policy='original-lane-node'
        self.source_speed_node_fields=speed_node_fields or {}
        super().__init__(*args,**kwargs)


class SourceAdmissionBlocked(ValueError):
    def __init__(self,roles):
        super().__init__('unresolved source roles; no geometry solver or export permitted')
        self.roles=roles


def load_road(directory,decision_file,road,previous_source_directory=None):
    directory=Path(directory).resolve()
    decision_file=Path(decision_file).resolve() if decision_file is not None else None
    verify_contacts(directory)
    scope=json.loads((directory/'input/reconstruction-input.json').read_text(encoding='utf8'))
    domain=json.loads((directory/'input/source-domain.json').read_text(encoding='utf8'))
    contacts=json.loads((directory/'contact-model.json').read_text(encoding='utf8'))
    run=json.loads((directory/'input/run.json').read_text(encoding='utf8'))
    decision=yaml.safe_load(decision_file.read_text(encoding='utf8')) if decision_file else None
    if previous_source_directory is not None:
        if decision is None:raise ValueError('role migration requires an explicit prior decision')
        old=Path(previous_source_directory).resolve()
        previous=json.loads((old/'contact-model.json').read_text(encoding='utf8'))
        decision=rebind_unchanged_source_decision(scope,domain,previous,contacts,decision)
    roles=(resolve_source_roles(scope,domain,contacts,decision) if decision is not None else
           resolve_original_source_roles(scope,domain,contacts))
    if not roles['role_binding_complete']:
        raise SourceAdmissionBlocked(roles)
    source_root=ET.parse(run['input']).getroot()
    profile=yaml.safe_load(Path(run['profile']).read_text(encoding='utf8'))
    node_fields={spec['file']:{role:spec['fields'][role+'_node'] for role in ('start','end')}
                 for name,spec in profile['layers'].items() if name in ('lane','lane_merge')
                 and all(role+'_node' in spec.get('fields',{}) for role in ('start','end'))}
    model=SourceSpeedBoundaryBlock(source_root,scope,domain,contacts,roles,road=road,degree=3,speed_node_fields=node_fields)
    files=dict(run['input_files_sha256'])
    paths=[directory/'contact-model.json',directory/'input/reconstruction-input.json',
           directory/'input/source-domain.json',directory/'input/run.json',directory/'run.json',
           Path(__file__).resolve()]
    if decision_file:paths.append(decision_file)
    paths += [ROOT/name for name in ('mapforge/ops/source_cubic_export.py','spikes/source_contact_fit.py',
        'spikes/arc_source_boundary.py','mapforge/ops/source_roles.py','mapforge/ops/source_center_support.py',
        'mapforge/ops/source_contacts.py','mapforge/ops/physical_continuations.py','mapforge/ops/arc_source_chart.py',
        'mapforge/ops/source_transition_domains.py','mapforge/ops/source_speed_events.py','spikes/clarabel_joint_candidate.py')]
    files.update({str(p):_sha256(p) for p in paths});unchanged(files)
    return model,roles,files,source_root


def run(directory,decision_file,road,output,end_axis=True,flat_port=False,minimum_width_span=5.5,previous_source_directory=None,asymmetric_domain=False,source_direction=None):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    print('VERIFY ORIGINAL SOURCE',road,flush=True)
    try:original,roles,files,root=load_road(directory,decision_file,road,previous_source_directory)
    except SourceAdmissionBlocked as exc:
        dump(output/'roles.json',exc.roles)
        report=dict(status='SOURCE_ROLE_DECISION_REQUIRED',road=road,
                    unresolved=exc.roles['unresolved'],resolved_count=len(exc.roles['resolved']),
                    source_contact_sha256=exc.roles['source_contact_sha256'],
                    geometry_solver_ran=False,xodr_generated=False,production_accepted=False)
        dump(output/'report.json',report);print(json.dumps(report,ensure_ascii=False),flush=True)
        return report
    print('SOURCE READY',road,len(original.source_ids),original.nvar,flush=True)
    dump(output/'roles.json',roles)
    model=cubic_model(original,minimum_span=minimum_width_span,end_axis=end_axis)
    domain=None
    if asymmetric_domain:
        from mapforge.ops.source_export_domain import direction_starts,plan_source_domain,constrain_external_cuts
        structural=[max(direction_starts(model).values())]
        model=cubic_model(original,minimum_span=minimum_width_span,end_axis=end_axis,structural_stations=structural)
        domain=plan_source_domain(model,root.find(f"road[@id='{road}']"))
        model=constrain_external_cuts(model,domain)
        dump(output/'source-export-domain.json',domain)
        files[str(ROOT/'mapforge/ops/source_export_domain.py')]=_sha256(ROOT/'mapforge/ops/source_export_domain.py')
    # A single transverse start/end is not a source fact. If it cannot cover
    # the original oblique/asymmetric supports, keep a full source state for
    # the dependency/domain compiler. Never crop observations to emit a PASS.
    export_domain_error=None
    try:
        if domain is None:model=constrain_written_endpoints(model)
    except ValueError as exc:
        if 'full written endpoint budget impossible' not in str(exc):raise
        export_domain_error=str(exc)
    if flat_port:model=constrain_flat_connector_port(model)
    print('SOURCE MODEL',road,model.nvar,model.export_axis_selection,flush=True)
    with patch('spikes.source_contact_fit._convex_qp',interior_qp):x,phase=model.solve()
    dump(output/'model.json',model.describe())
    if x is None:
        dump(output/'rejection.json',dict(status='REJECTED_NOT_DELIVERY',phase=phase,
             source_hashes=files,production_accepted=False))
        unchanged(files);return None
    print('SHARED CUBIC SOLVED',road,flush=True)
    dump(output/'shared-state.json',dict(model_sha=digest(model.describe()),coefficients=x.tolist(),phase=phase,
         source_hashes=files,road=road,source_directory=str(Path(directory).resolve()),
         decision_file=str(Path(decision_file).resolve()) if decision_file else None,end_axis=end_axis,flat_port=flat_port,
         previous_source_directory=str(Path(previous_source_directory).resolve()) if previous_source_directory else None,
         minimum_width_span=minimum_width_span,source_export_domain=domain,source_direction=source_direction))
    if export_domain_error:
        from mapforge.ops.source_cubic_export import source_chains
        chains=source_chains(model)
        domains=[dict(direction=direction,start_m=min(c['a'] for c in chains if c['direction']==direction),
                      end_m=max(c['b'] for c in chains if c['direction']==direction))
                 for direction in sorted({c['direction'] for c in chains})]
        source_audit=model.audit(x)
        report=dict(status='SOURCE_STATE_NOT_WRITABLE_AS_ONE_COMMON_CUT',road=road,
            reason=export_domain_error,axis=model.export_axis_selection,source_audit=source_audit,
            direction_domains=domains,source_chains=chains,source_hashes=files,source_changed=False,
            xodr_generated=False,production_accepted=False,
            required_next='compile source-proven asymmetric extents and junction support jointly; never drop original tails')
        dump(output/'report.json',report)
        draw_source_state(model,x,output/'full-source-state.png')
        unchanged(files);print('FULL SOURCE SOLVED; EXPORT DOMAIN REJECTED',export_domain_error,flush=True)
        return report
    new,compiled,ports=compile_road(model,x,root.find(f"road[@id='{road}']"),source_domain=domain,source_direction=source_direction)
    isolated=ET.Element('OpenDRIVE');isolated.append(copy.deepcopy(root.find('header')))
    isolated.append(new)
    for e in new.findall('link'):new.remove(e)
    path=output/f'road{road}.xodr';ET.ElementTree(isolated).write(str(path),encoding='utf-8',xml_declaration=True,pretty_print=True)
    from mapforge.validate.source_cubic_readback import audit_written_source
    import xml.etree.ElementTree as stdET
    readback=audit_written_source(stdET.parse(path).getroot().find('road'),compiled,model)
    schema=ET.XMLSchema(ET.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    xsd=schema.validate(ET.parse(path))
    from scripts.internal_edge_jets import audit as internal_audit
    internal=internal_audit(stdET.parse(path).getroot())
    from mapforge.validate.g11 import load_policy,_audit_d
    cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');cfg['dynamics']['sample_step_m']=.02
    dynamics=_audit_d(stdET.parse(path).getroot(),cfg)
    report=dict(status='COMPONENT_REVIEW_NOT_DELIVERY',road=road,artifact=str(path),sha256=_sha256(path),compiled=compiled,
        source_hashes=files,written_source=readback,internal=internal,xsd_pass=xsd,xsd_errors=str(schema.error_log),
        source_speed_dynamics=dynamics,source_changed=False,production_accepted=False,
        ports=[dict(contact=cp,old_lane=old,new_lane=new) for (cp,old),new in ports.items()])
    dump(output/'report.json',report)
    from scripts.basemap_overlay import _lane_lines
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(18,9),layout='constrained')
    edges,centers=_lane_lines(stdET.parse(path).getroot().find('road'),.05)
    for ax in axes.flat:
        for key,xy in model.source_xy.items():
            if key.startswith('boundary:'):ax.plot(*xy.T,c='#e78c1a',lw=1.3)
        for xy,*_ in edges:ax.plot(*xy.T,c='#176bbb',lw=.9)
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    axes[0,0].set_title('Full original boundaries / actual written road')
    for ax,event in zip(list(axes.flat)[1:],model.contacts['role_conflicts']):
        px,py=event['collapsed_boundary_xy'];ax.set_xlim(px-12,px+12);ax.set_ylim(py-5,py+5)
        ax.set_title(event['source_lane_id'])
    fig.suptitle(f'road{road}: orange=raw SHP boundaries, blue=WRITTEN XODR\n'
        'Isolated component; incident turns, separate movement paths and entire map NOT accepted.')
    fig.savefig(output/'source-readback.png',dpi=150);plt.close(fig)
    unchanged(files)
    print('WRITTEN',road,readback['status'],'XSD',xsd,'INTERNAL',internal['status'],flush=True)
    return report


def draw_source_state(model,x,path):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from scipy.interpolate import BSpline
    fig,axes=plt.subplots(2,3,figsize=(18,9),layout='constrained')
    for ax in axes.flat:
        for key,xy in model.source_xy.items():
            if key.startswith('boundary:'):ax.plot(*xy.T,c='#e78c1a',lw=1.3)
            else:ax.plot(*xy.T,c='#888888',lw=.6,ls='--')
        for family in model.families:
            ss=np.linspace(family.knots[0],family.knots[-1],800)
            yy=BSpline(family.knots,x[family.columns],3)(ss)
            xy=model.axis.world(np.c_[ss,yy]);ax.plot(*xy.T,c='#176bbb',lw=.9)
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    axes[0,0].set_title('Full original extent, including asymmetric outer ends')
    for ax,event in zip(list(axes.flat)[1:],model.contacts['role_conflicts']):
        px,py=event['collapsed_boundary_xy'];ax.set_xlim(px-12,px+12);ax.set_ylim(py-5,py+5)
        ax.set_title(event['source_lane_id'])
    end_xy=np.array([xy[-1] if xy[-1,0]<xy[0,0] else xy[0]
                     for key,xy in model.source_xy.items() if key.startswith('boundary:')])
    # Last panel is the complete junction-side source end area, not a hidden
    # subset around only the previously known three source-role exceptions.
    tip=end_xy[end_xy[:,0]<np.min(end_xy[:,0])+3]
    ax=axes.flat[-1]
    ax.set_xlim(tip[:,0].min()-2,tip[:,0].max()+3)
    ax.set_ylim(tip[:,1].min()-2,tip[:,1].max()+2)
    ax.set_title('Original staggered mouth ends (source state, not XODR)')
    fig.suptitle('ORANGE=raw boundaries; GRAY=all raw paths; BLUE=shared cubic SOURCE STATE\n'
        'NOT a written XODR. Asymmetric domain/compiler and all incident connectors are NOT accepted.')
    fig.savefig(path,dpi=150);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('decision_file');p.add_argument('road');p.add_argument('output')
    p.add_argument('--line-axis',action='store_true');p.add_argument('--flat-port',action='store_true')
    p.add_argument('--previous-source-directory');p.add_argument('--asymmetric-domain',action='store_true')
    p.add_argument('--source-direction',choices=['with_s','against_s'])
    a=p.parse_args();run(a.directory,None if a.decision_file=='-' else a.decision_file,a.road,a.output,not a.line_axis,a.flat_port,
                       previous_source_directory=a.previous_source_directory,asymmetric_domain=a.asymmetric_domain,
                       source_direction=a.source_direction)
    raise SystemExit(2)  # All outcomes of this research driver are non-delivery.
