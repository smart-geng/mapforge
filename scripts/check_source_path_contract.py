"""Source-only necessary conditions, separate from existing fitted maps.

Rebuild full original ordinary center chains. No generated graph, polynomial
basis, branch equalities or lane-width fitting are used by the single-path test.
"""
import argparse
import copy
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from mapforge.ops.source_path_graph import ordinary_source_paths
from mapforge.ops.reconstruction_scope import digest
from scripts.fit_source_boundary_block import load,unchanged,runtime_versions
from scripts.recheck_speed_contract import sha,dump
from spikes.source_path_jerk_bound import SourcePathJerkBound
from spikes.source_jet_relaxation import SourceJetRelaxation


def ablations(model):
    original=SourceJetRelaxation(model,2.);out=[]
    for name,drop,equalities in [('full',set(),True),('without_centers',{'source-center'},True),
        ('without_fork_C2',set(),False),('without_centers_or_fork_C2',{'source-center'},False)]:
        c=copy.copy(original);keep=[i for i,r in enumerate(original.labels) if r['kind'] not in drop]
        c.inequalities=[original.inequalities[i] for i in keep];c.rhs=[original.rhs[i] for i in keep]
        c.labels=[original.labels[i] for i in keep];c.equalities=original.equalities if equalities else []
        _,r=c.solve();out.append(dict(case=name,diagnostic_constraint_removal_only=True,report=r,export_allowed=False))
        print('ABLATION',name,r['status'],r.get('minimum_normalized_slack'),flush=True)
    return out


def calculate(model):
    graph=ordinary_source_paths(model);checks=[]
    for index,path in enumerate(graph['paths']):
        if path['status']!='SOURCE_PATH_PREPARED_NOT_GEOMETRY':continue
        for step in (2.,1.):
            c=SourcePathJerkBound(path['necessary_anchors_st'],path['source_speed_kmh'],step=step)
            _,r=c.solve();checks.append(dict(path_index=index,source_lanes=path['source_lanes_in_traffic_order'],report=r))
            print('PATH',index,'step',step,r['status'],r.get('dual_lower_bound'),flush=True)
    return dict(status='SOURCE_CONTRACT_RESEARCH_NOT_MAP',source_scope_sha256=model.scope['content_sha256'],
        source_roles_sha256=model.roles['content_sha256'],path_graph=graph,checks=checks,
        ablations=ablations(model),source_tolerance_m=.35,source_speed_changed=False,
        new_geometry_generated=False,xodr_generated=False,export_allowed=False,
        significance='single-path conflict can preclude a contract before expensive shared-map search',
        limitations=['monotone planar path graph and whole source polyline tube assumed',
            'necessary numerical lower bound is not an attainable optimized curve',
            'passing the necessary test cannot approve any map or path',
            'two movement-observation chains require separate validation',
            'no default profile, source interpretation or speed changed'])


def draw_sources(report,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[p for p in report['path_graph']['paths'] if p['status']=='SOURCE_PATH_PREPARED_NOT_GEOMETRY']
    fig,axes=plt.subplots(len(rows),1,figsize=(13,12),squeeze=False)
    for ax,p in zip(axes[:,0],rows):
        index=report['path_graph']['paths'].index(p);raw=np.asarray(p['necessary_anchors_st'])
        r=next(v['report'] for v in report['checks'] if v['path_index']==index and v['report']['witness_step_m']==1.)
        delta=r['necessary_vertical_superset_m'];ss=raw[:,0];yy=raw[:,1]-raw[0,1]
        ax.fill_between(ss,yy-delta,yy+delta,color='#e6bc7b',alpha=.5,label='necessary superset of 0.35 m tube')
        ax.plot(ss,yy,'o-',ms=4,color='#3a546c',label='original source centers (no candidate)')
        active=[v for v in r.get('dual_support',[]) if v['kind']=='source']
        st=sorted({v['station_m'] for v in active})
        if st and r.get('valid_numerical_certificate'):
            ax.scatter(st,np.interp(st,ss,yy),s=35,color='#c02b4c',zorder=5,label='active source witnesses')
        value=r.get('dual_lower_bound');valid=r.get('valid_numerical_certificate')
        description=('necessary jerk >= '+format(value,'.3f')+' m/s^3' if value and value>1.
                     else 'necessary test did NOT exclude a path; no curve proved') if valid else 'numerical bound unavailable'
        ax.set_title('Source chain '+str(index)+' / original '+format(p['source_speed_kmh'],'g')+' km/h / '+description,fontsize=11)
        ax.set_ylabel('lateral shift (m)');ax.set_xlabel('local longitudinal coordinate (m)')
        ax.grid(alpha=.2);ax.legend(fontsize=8,loc='best')
    fig.suptitle('ONLY ORIGINAL SOURCE / longitudinal and transverse scales differ\nNo spline, no fitted XODR, no speed change',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(path,dpi=140);plt.close(fig)


def code_files(files):
    for folder in ('mapforge','spikes','scripts'):
        for p in (ROOT/folder).rglob('*.py'):files[str(p)]=sha(p)
    return files


def run(source,decision,output):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('new output directory required')
    model,roles,files=load(source,decision,degree=5);code_files(files)
    result=calculate(model);unchanged(files);output.mkdir(parents=True)
    dump(output/'report.json',result);draw_sources(result,output/'source-paths.png');unchanged(files)
    dump(output/'run.json',dict(source=str(Path(source).resolve()),decision=str(Path(decision).resolve()),
        inputs_sha256=files,runtime_versions=runtime_versions(),
        outputs_sha256={p.name:sha(p) for p in output.iterdir() if p.is_file()}))
    return result


def verify(directory):
    directory=Path(directory);read=lambda n:json.loads((directory/n).read_text(encoding='utf-8'))
    record=read('run.json');saved=read('report.json');unchanged(record['inputs_sha256'])
    if record['runtime_versions']!=runtime_versions():raise ValueError('runtime mismatch')
    if set(record['outputs_sha256'])!={'report.json','source-paths.png'}:raise ValueError('output inventory differs')
    unchanged({str(directory/n):h for n,h in record['outputs_sha256'].items()})
    model,roles,files=load(record['source'],record['decision'],degree=5);code_files(files)
    if files!=record['inputs_sha256']:raise ValueError('source/code inventory mismatch')
    graph=ordinary_source_paths(model)
    if digest(graph)!=digest(saved['path_graph']):raise ValueError('source path graph differs')
    # Independently multiply every saved dual/primal by fresh constraints
    # before running the optimizer again. No solver status is taken on faith.
    for row in saved['checks']:
        p=graph['paths'][row['path_index']];r=row['report']
        if 'certificate' not in r:continue
        c=SourcePathJerkBound(p['necessary_anchors_st'],p['source_speed_kmh'],step=r['witness_step_m'])
        actual=c.verify_certificate(r['certificate'])
        for k,v in actual.items():
            if digest(v)!=digest(r[k]):raise ValueError('saved necessary certificate differs: '+k)
        if r['status']!='UNAVAILABLE' and not actual['valid_numerical_certificate']:
            raise ValueError('invalid certificate promoted to diagnosis')
    fresh=calculate(model)
    if digest(saved)!=digest(fresh):raise ValueError('fresh source-bound diagnosis differs')
    unchanged(files)
    return dict(status='VERIFIED_SOURCE_CONTRACT_NOT_MAP',inputs=len(files),
        conflict_paths=sorted({r['path_index'] for r in fresh['checks'] if r['report']['status']=='NECESSARY_JERK_CONFLICT'}),
        unavailable_checks=sum(r['report']['status']=='UNAVAILABLE' for r in fresh['checks']),export_allowed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('decision');p.add_argument('output')
    p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.verify:print(verify(a.output))
    else:run(a.source,a.decision,a.output)
    raise SystemExit(2)
