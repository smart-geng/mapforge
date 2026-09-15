"""Read original reviewed SHP boundary graph; necessary dynamics test only."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.fit_source_boundary_block import load,unchanged,runtime_versions
from scripts.recheck_speed_contract import sha,dump
from spikes.source_jet_relaxation import SourceJetRelaxation


def run(source,decision,output,step=2.,fit_events=False):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('new output directory required')
    m,roles,files=load(source,decision,degree=5)
    for folder in ('mapforge','spikes','scripts'):
        for p in (ROOT/folder).rglob('*.py'):files[str(p)]=sha(p)
    print('PREPARED source graph',len(m.owner),'boundaries',flush=True)
    check=SourceJetRelaxation(m,step)
    print('RELAXATION',check.nvar,'variables',len(check.rhs),'inequalities',flush=True)
    x,report=check.solve()
    print('FREE JETS',report['status'],report.get('minimum_normalized_slack'),flush=True)
    coefficients,basis_report=check.solve(restrict_basis=True)
    print('EXISTING LONG BASIS',basis_report['status'],basis_report.get('minimum_normalized_slack'),flush=True)
    event_result=None;event_model=None;event_x=None
    if fit_events:
        from spikes.source_event_basis import event_aligned_model
        from spikes.shared_boundary_dynamics import refine_dynamics
        from spikes import source_contact_fit
        from spikes.clarabel_joint_candidate import interior_qp
        from unittest.mock import patch
        event_model,event_layout=event_aligned_model(m)
        print('EVENT BASIS',event_layout['variables'],event_layout['min_span_m'],flush=True)
        with patch.object(source_contact_fit,'_convex_qp',interior_qp):
            event_x,phase=event_model.solve()
        fairness=None if event_x is None else event_x.copy();dynamic=None
        if event_x is not None:event_x,dynamic=refine_dynamics(event_model,event_x)
        _,event_necessary=SourceJetRelaxation(event_model,step).solve(restrict_basis=True)
        event_result=dict(layout=event_layout,phase=phase,dynamics=dynamic,necessary_basis=event_necessary,
            initial_coefficients=None if fairness is None else fairness.tolist(),
            coefficients=None if event_x is None else event_x.tolist(),
            audit=event_model.audit(event_x) if event_x is not None else None,
            model=event_model.describe(),export_allowed=False)
    report.update(original_scope_sha=m.scope['content_sha256'],roles_sha=roles['content_sha256'],
        no_source_removed=True,source_contact_stations_fixed=True,
        source_family_features=[f.features for f in m.families],
        missing_full_center_support=m.uncovered_centers,incident_connectors_checked=False,
        search_jets_are_not_candidate_geometry=True)
    unchanged(files);output.mkdir(parents=True)
    dump(output/'report.json',report)
    dump(output/'relaxed-jets.json',dict(status='NOT_A_CURVE',variables=check.variables,
        scaled_jet_values=None if x is None else x.tolist(),export_allowed=False))
    dump(output/'bounds.json',dict(derivative_bounds=check.dynamic_intervals))
    dump(output/'basis-relaxation.json',dict(report=basis_report,
        coefficients=None if coefficients is None else coefficients.tolist(),
        polynomial_degree=m.degree,export_allowed=False))
    if event_result is not None:
        from scripts.fit_source_boundary_block import draw
        dump(output/'event-basis.json',event_result)
        draw(event_model,event_x,output/'event-basis.png')
    dump(output/'run.json',dict(source_directory=str(Path(source).resolve()),decision_file=str(Path(decision).resolve()),
        step=step,fit_events=fit_events,source_and_code_sha256=files,runtime_versions=runtime_versions(),
        output_sha256={p.name:sha(p) for p in output.iterdir() if p.is_file()}))
    print({k:v for k,v in report.items() if k in ['status','minimum_normalized_slack','primal_residual','dual_residual','duality_gap']},flush=True)
    return report


def verify(directory):
    """Fresh source/model rebuild and numerical readback, not a status replay."""
    import numpy as np
    from mapforge.ops.reconstruction_scope import digest
    from spikes.source_event_basis import event_aligned_model
    from spikes.shared_boundary_dynamics import verify_refinement
    directory=Path(directory);read=lambda name:json.loads((directory/name).read_text(encoding='utf-8'))
    run_record=read('run.json');unchanged(run_record['source_and_code_sha256'])
    if run_record['runtime_versions']!=runtime_versions():raise ValueError('runtime changed')
    names={'report.json','relaxed-jets.json','bounds.json','basis-relaxation.json'}
    if run_record.get('fit_events'):names|={'event-basis.json','event-basis.png'}
    if set(run_record['output_sha256'])!=names:raise ValueError('missing or unexpected output inventory')
    for name,h in run_record['output_sha256'].items():
        if sha(directory/name)!=h:raise ValueError('changed output '+name)
    model,roles,files=load(run_record['source_directory'],run_record['decision_file'],degree=5)
    for folder in ('mapforge','spikes','scripts'):
        for p in (ROOT/folder).rglob('*.py'):files[str(p)]=sha(p)
    if files!=run_record['source_and_code_sha256']:raise ValueError('source/code inventory differs')
    check=SourceJetRelaxation(model,run_record['step']);saved=read('report.json')
    claims=dict(original_scope_sha=model.scope['content_sha256'],roles_sha=roles['content_sha256'],
        no_source_removed=True,source_contact_stations_fixed=True,
        source_family_features=[f.features for f in model.families],
        missing_full_center_support=model.uncovered_centers,incident_connectors_checked=False,
        search_jets_are_not_candidate_geometry=True)
    if any(digest(v)!=digest(saved.get(k)) for k,v in claims.items()):
        raise ValueError('saved source scope or qualification differs')
    _,fresh=check.solve()
    if any(digest(value)!=digest(saved.get(key)) for key,value in fresh.items()):
        raise ValueError('necessary relaxation differs from fresh solve')
    jets=read('relaxed-jets.json');x=np.asarray(jets['scaled_jet_values'])
    if jets['status']!='NOT_A_CURVE' or jets['export_allowed'] is not False:
        raise ValueError('relaxed jets cannot be promoted to geometry')
    if digest(check.variables)!=digest(jets['variables']):raise ValueError('wrong witness indexing')
    if fresh['status']=='RELAXATION_FEASIBLE_ONLY':
        violation=max(float(np.max(check.matrix(check.inequalities)@x-np.asarray(check.rhs),initial=0)),
                      float(np.max(abs(check.matrix(check.equalities)@x),initial=0)))
        if violation>1e-6:raise ValueError('saved relaxed jets fail original inequalities')
    if digest(check.dynamic_intervals)!=digest(read('bounds.json')['derivative_bounds']):
        raise ValueError('saved derivative bounds differ from original source')
    _,basis=check.solve(restrict_basis=True)
    saved_basis=read('basis-relaxation.json')
    if digest(basis)!=digest(saved_basis['report']):raise ValueError('basis diagnosis differs')
    if saved_basis['export_allowed'] is not False or saved_basis['polynomial_degree']!=model.degree:
        raise ValueError('basis scope/qualification differs')
    result=dict(status='DIAGNOSTIC_VERIFIED_NOT_MAP',original_source_rebuilt=True,
                basis_status=basis['status'],jet_status=fresh['status'],export_allowed=False)
    if run_record.get('fit_events'):
        m,layout=event_aligned_model(model);event=read('event-basis.json')
        if digest(layout)!=digest(event['layout']) or digest(m.describe())!=digest(event['model']):
            raise ValueError('event basis model differs')
        if event['coefficients'] is not None:
            verify_refinement(m,event['initial_coefficients'],event['coefficients'],event['dynamics'])
            audit=m.audit(np.array(event['coefficients']))
            if digest(audit)!=digest(event['audit']):raise ValueError('saved dense dynamics/geometry differs')
            result['event_dense_dynamics']=audit['physical_midpoint_dynamics_worst']
        _,necessary=SourceJetRelaxation(m,run_record['step']).solve(restrict_basis=True)
        if digest(necessary)!=digest(event['necessary_basis']):raise ValueError('event necessary diagnosis differs')
        if event['export_allowed'] is not False:raise ValueError('event model not compiled or approved')
    unchanged(files)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('decision');p.add_argument('output')
    p.add_argument('--step',type=float,default=2.);p.add_argument('--fit-events',action='store_true');a=p.parse_args()
    run(a.source,a.decision,a.output,a.step,a.fit_events)
    raise SystemExit(2)  # A necessary relaxation never qualifies an XODR.
