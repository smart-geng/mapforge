"""Versioned role decision + source-native boundary solve/replay; no XODR."""
import argparse
import json
from importlib.metadata import version
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import yaml
from lxml import etree

from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_roles import resolve_source_roles
from scripts.prepare_source_contacts import verify_contacts
from scripts.gen_all import _sha256
from spikes.source_contact_fit import SourceBoundaryBlock
from spikes.shared_boundary_dynamics import refine_dynamics,verify_refinement


def load(source_directory, decision_file, degree=3):
    directory=Path(source_directory).resolve()
    verify_contacts(directory)  # fresh source reader, original input hashes
    scope=json.loads((directory/'input/reconstruction-input.json').read_text(encoding='utf8'))
    domain=json.loads((directory/'input/source-domain.json').read_text(encoding='utf8'))
    contacts=json.loads((directory/'contact-model.json').read_text(encoding='utf8'))
    run=json.loads((directory/'input/run.json').read_text(encoding='utf8'))
    decision=yaml.safe_load(Path(decision_file).read_text(encoding='utf8'))
    roles=resolve_source_roles(scope,domain,contacts,decision)
    model=SourceBoundaryBlock(etree.parse(run['input']).getroot(),scope,domain,contacts,roles,degree=degree)
    files={p:sha for p,sha in run['input_files_sha256'].items()}
    paths=[directory/'contact-model.json',directory/'run.json',directory/'input/run.json',
           directory/'input/reconstruction-input.json',directory/'input/source-domain.json',Path(decision_file).resolve()]
    paths += [ROOT/name for name in ('mapforge/ops/source_roles.py','mapforge/ops/source_contacts.py','mapforge/ops/physical_continuations.py',
              'spikes/source_contact_fit.py','spikes/shared_boundary_dynamics.py','spikes/road_boundary_family.py',
              'spikes/clarabel_joint_candidate.py','scripts/fit_source_boundary_block.py')]
    files.update({str(p):_sha256(p) for p in paths})
    return model,roles,files


def unchanged(files):
    if any(_sha256(Path(p))!=h for p,h in files.items()):raise ValueError('source/config/code changed during run')


def runtime_versions():
    return {name:version(name) for name in ('numpy','scipy','osqp','clarabel','lxml')}


def draw(model, x, output, diagnostic=False, state_label=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from scipy.interpolate import BSpline
    fig,axes=plt.subplots(1,3,figsize=(14,7))
    def world(st):
        tangent=np.array(model.chart['tangent']);normal=np.array([-tangent[1],tangent[0]])
        return np.asarray(model.chart['origin'])+np.asarray(st)@np.stack([tangent,normal])
    for ax in axes:
        for key,raw in model.raw.items():
            xy=world(raw);is_boundary=key.startswith('boundary:')
            ax.plot(xy[:,0],xy[:,1],color='#e2861c' if is_boundary else '#92969b',
                    lw=1.4 if is_boundary else .7,ls='-' if is_boundary else '--',alpha=.8)
        if x is not None:
            for f in model.families:
                ss=np.linspace(f.knots[0],f.knots[-1],1000)
                curve=BSpline(f.knots,np.asarray(x)[f.columns],model.degree)
                xy=world(np.c_[ss,curve(ss)]);ax.plot(xy[:,0],xy[:,1],color='#bd2753' if diagnostic else '#1466b8',lw=1)
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('local x (m)');ax.set_ylabel('local y (m)')
    axes[0].set_title('Full source boundary families')
    for ax,c in zip(axes[1:],model.contacts['role_conflicts']):
        p=c['collapsed_boundary_xy'];ax.set_xlim(p[0]-5,p[0]+5);ax.set_ylim(p[1]-12,p[1]+12)
        ax.set_title('Zero-width source '+c['source_lane_id'][-7:])
    fig.suptitle('Shared source-boundary block / NOT an accepted XODR\n'
                 'Orange: original boundaries; gray: original lane paths; '+
                 (state_label or ('RED: REJECTED minimum-slack diagnosis (not exportable)' if diagnostic else 'blue: coupled candidate')),fontsize=12)
    fig.tight_layout(rect=(0,0,1,.91));fig.savefig(output,dpi=140);plt.close(fig)


def run(source_directory,decision_file,output,degree=3,dynamics=False):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('use a new output directory')
    model,roles,files=load(source_directory,decision_file,degree)
    x,phase=model.solve();audit=model.audit(x) if x is not None else None
    fairness_x=x.copy() if x is not None else None
    dynamic_report=None
    if dynamics and x is not None:
        x,dynamic_report=refine_dynamics(model,x)
        audit=model.audit(x)
    diagnostic_x,necessary=model.necessary_source_preflight()
    result={'status':'BLOCKED','geometry_solver_ran':True,'fairing_optimization_ran':'optimization' in phase,
            'xodr_generated':False,'export_allowed':False,
            'model':model.describe(),'phase':phase,'audit':audit,'necessary_source_phase':necessary,
            'physical_continuations':model.physical_graph,
            'fairness_coefficients':fairness_x.tolist() if fairness_x is not None else None,
            'dynamics_refinement':dynamic_report,
            'rejected_diagnostic_coefficients':diagnostic_x.tolist() if x is None else None,
            'candidate_coefficients':x.tolist() if x is not None else None,
            'remaining':['incident connector coupling','source role movement-path checks','domain endpoints',
                         'full-road dynamics','XODR write/readback','MAP and full junction regression']}
    unchanged(files);output.mkdir(parents=True)
    def write(name,value):(output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    write('source-roles.json',roles);write('decision.json',roles['decision']);write('report.json',result)
    draw(model,x if x is not None else diagnostic_x,output/'boundary-block.png',diagnostic=x is None)
    manifest={'source_directory':str(Path(source_directory).resolve()),'decision_file':str(Path(decision_file).resolve()),
              'runtime_versions':runtime_versions(),
              'polynomial_degree':degree,
              'dynamics_refinement_requested':dynamics,
              'input_sha256':files,'outputs_sha256':{name:_sha256(output/name) for name in
                 ('source-roles.json','decision.json','report.json','boundary-block.png')}}
    write('run.json',manifest)
    print(json.dumps({'status':'BLOCKED','phase':phase['status'],'slack_m':phase['minimum_uniform_constraint_slack_m'],
                      'source_max_m':audit['boundary_same_chart_max_m'] if audit else None,
                      'dense_dynamics':audit['physical_midpoint_dynamics_worst'] if audit else None,
                      'dynamics_refinement_status':dynamic_report['status'] if dynamic_report else None,
                      'variables':model.nvar,'families':len(model.families),'output':str(output)},ensure_ascii=False,indent=2))
    return result


def verify(directory):
    directory=Path(directory);run=json.loads((directory/'run.json').read_text(encoding='utf8'))
    unchanged(run['input_sha256'])
    if run.get('runtime_versions')!=runtime_versions():raise ValueError('solver runtime changed; rebuild research evidence')
    if set(run['outputs_sha256'])!={'source-roles.json','decision.json','report.json','boundary-block.png'}:
        raise ValueError('incomplete output manifest')
    unchanged({str(directory/n):h for n,h in run['outputs_sha256'].items()})
    model,roles,files=load(run['source_directory'],run['decision_file'],run.get('polynomial_degree',3))
    if files!=run['input_sha256']:raise ValueError('source/code inventory mismatch')
    saved_roles=json.loads((directory/'source-roles.json').read_text(encoding='utf8'))
    saved_decision=json.loads((directory/'decision.json').read_text(encoding='utf8'))
    report=json.loads((directory/'report.json').read_text(encoding='utf8'))
    if digest(roles)!=digest(saved_roles) or digest(roles['decision'])!=digest(saved_decision):
        raise ValueError('saved source decisions differ from reviewed input')
    if digest(model.describe())!=digest(report['model']):raise ValueError('saved model differs from fresh source')
    if digest(model.physical_graph)!=digest(report['physical_continuations']):raise ValueError('physical graph differs from fresh source')
    dx,necessary=model.necessary_source_preflight()
    if digest(necessary)!=digest(report['necessary_source_phase']):raise ValueError('necessary diagnosis differs from replay')
    if report['candidate_coefficients'] is not None:
        fairness,phase=model.solve()
        if fairness is None or not np.allclose(fairness,report['fairness_coefficients'],rtol=0,atol=1e-8):
            raise ValueError('saved initial fairing coefficients differ from fresh solve')
        if digest(phase)!=digest(report['phase']):raise ValueError('saved fairing phase differs from replay')
        if run.get('dynamics_refinement_requested'):
            verify_refinement(model,fairness,report['candidate_coefficients'],report['dynamics_refinement'])
        elif report['dynamics_refinement'] is not None or not np.array_equal(fairness,report['candidate_coefficients']):
            raise ValueError('unrequested dynamics refinement')
        audit=model.audit(np.array(report['candidate_coefficients']))
        if digest(audit)!=digest(report['audit']):raise ValueError('saved candidate audit differs from recomputation')
    else:
        _,phase=model.solve()
        if digest(phase)!=digest(report['phase']):raise ValueError('saved phase diagnosis differs from recomputation')
        if not np.array_equal(dx,np.asarray(report['rejected_diagnostic_coefficients'])):
            raise ValueError('rejected diagnostic state differs from recomputation')
    if report['status']!='BLOCKED' or report['export_allowed'] or report['xodr_generated']:
        raise ValueError('boundary block cannot claim completed map acceptance')
    unchanged(files)
    return {'fresh_source_roles_match':True,'parameter_readback_match':True,'status':'BLOCKED','new_xodr':False}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source_directory');p.add_argument('decision_file');p.add_argument('output')
    p.add_argument('--degree',type=int,choices=(3,5),default=3,help='research target only; quintic cannot be exported as XODR width')
    p.add_argument('--dynamics',action='store_true',help='jointly minimize actual midpoint dynamics without changing source/speed')
    p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.verify:print(json.dumps(verify(a.output),indent=2))
    else:run(a.source_directory,a.decision_file,a.output,a.degree,a.dynamics)
    raise SystemExit(2)
