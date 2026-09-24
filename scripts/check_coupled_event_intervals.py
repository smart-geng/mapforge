"""One bounded real-data diagnostic replay; no candidate/solver/Web writes."""
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.prepare_coupled_event_contract import load_contract
from scripts.check_coupled_event_guards import verify,serial,WEB
from mapforge.repair_web.model import atomic,json_bytes,digest
from mapforge.repair_web.coupled_event_sources import EventSources
from mapforge.repair_web.coupled_event_guards import reference_turn
from mapforge.repair_web.coupled_event_intervals import check_turn_domains,LongEdge
from mapforge.repair_web.coupled_event_compiler import SevenRoadCompiler

PRIOR=ROOT/'out/node4-coupled-event-guards-20260917'
DEST=ROOT/'out/node4-coupled-event-intervals-20260917'
BUDGET=1500


def figures(contract,sources,results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
    for ax,cid,role in ((axes[0],'111','predecessor'),(axes[1],'106','successor')):
        edge=LongEdge(reference_turn(contract.graph.roads[cid]),'right')
        result=results[cid]['sides']['right']; row=next(r for r in result['checks'] if r['role']==role)
        original=next(r for r in sources.by_turn[cid] if r['field']=='right' and r['role']==role)
        a,b=row['domain_m']; poly,_=edge.polyline(a,b)
        full,_=edge.polyline(max(0.,a-1.5),min(edge.length,b+1.5))
        raw=original['oriented_full_xy']
        ax.plot(*raw.T,color='#df8b21',lw=2,label='Complete same-identity original part')
        ax.plot(*full.T,color='#678ea7',lw=1,label='Unchanged XML edge')
        ax.plot(*poly.T,color='#ba251e',lw=2,label='Explicit ordinary-tail target domain')
        plane=next(r for r in result['ownership']['witnesses'] if r['role']==role)
        p=np.array(plane['point']); n=np.array(plane['normal']); t=np.array([-n[1],n[0]])
        cross=np.array([p-3*t,p+3*t]); ax.plot(*cross.T,ls='--',lw=1,color='#6a5599',label='Original ordinary/via join plane')
        xy=np.vstack([full,p]); lo=xy.min(axis=0); hi=xy.max(axis=0); extra=.15 if cid=='106' else .5
        ax.set(xlim=(lo[0]-extra,hi[0]+extra),ylim=(lo[1]-extra,hi[1]+extra),aspect='equal',xlabel='Local x (m)',ylabel='Local y (m)')
        r=row['target_to_original']
        ax.set_title('Turn %s right %s | maximum deviation in [%.6f, %.6f] m; limit 0.350 m\nTarget interval %.6f-%.6f m; original source is NOT shortened'%(cid,role,r['lower_m'],r['upper_m'],a,b),fontsize=10)
        ax.grid(alpha=.2); ax.legend(fontsize=8,loc='best')
    fig.suptitle('Unchanged r6 XML: continuous numerical tail evidence, NOT a repair result',fontsize=13)
    fig.savefig(DEST/'tail-interval-witnesses.png',dpi=145); plt.close(fig)


def main():
    if DEST.exists(): raise ValueError('Do not overwrite previous evidence or reset budgets')
    bindings=json.loads((PRIOR/'binding.json').read_bytes()); verify(bindings)
    extra=[*PRIOR.glob('*.json'),Path(__file__).resolve(),ROOT/'tests/test_coupled_event_intervals.py',ROOT/'OpenDRIVE_1.5M.xsd']
    for module in list(sys.modules.values()):
        path=getattr(module,'__file__',None); name=getattr(module,'__name__','')
        if path and name.split('.')[0] in ('scripts','mapforge','spikes'): extra.append(Path(path).resolve())
    for path in extra:
        sha=digest(path.read_bytes())
        if str(path) in bindings and bindings[str(path)]!=sha: raise ValueError('Prior binding drift')
        bindings[str(path)]=sha
    verify(bindings)
    web={str(p):json.loads(p.read_bytes())['binding'] for p in WEB}
    for b in web.values(): verify(b)
    contract=load_contract(); sources=EventSources(contract); results={}
    for cid,rows in sources.by_turn.items():
        results[cid]=check_turn_domains(reference_turn(contract.graph.roads[cid]),rows,budget=BUDGET)
        print('Read-only continuous domains complete: '+cid,flush=True)
    compiler=SevenRoadCompiler(contract); compiled=compiler.diagnose(compiler.model.diagnostic)
    failures=[]; unresolved=[]; checks=0
    for cid,result in results.items():
        for side,value in result['sides'].items():
            if not value['ownership']['domains']: unresolved.append(dict(connector=cid,side=side,reason='ownership'))
            for row in value['checks']:
                checks+=1
                if row['status']=='FAIL_BOUND': failures.append(dict(connector=cid,side=side,**row))
                elif row['status']=='UNRESOLVED_BOUND': unresolved.append(dict(connector=cid,side=side,reason='maximum',**row))
    verify(bindings); DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json',json_bytes(bindings))
    atomic(DEST/'continuous-source.json',json_bytes(serial(results)))
    atomic(DEST/'in-memory-compiler.json',json_bytes(serial(compiled)))
    figures(contract,sources,results)
    for b in web.values(): verify(b)
    verify(bindings)
    status=dict(status='CONTINUOUS_SOURCE_AND_XML_READBACK_NOT_TRIAL_ADMISSION',
        reference_sha256=digest(contract.reference),source_parts_removed=0,source_speed_or_role_changes=0,
        ordinary_via_checks=checks,maximum_limit_failures=failures,unresolved_maximum_decisions=unresolved,
        budget=dict(maximum_per_direction_evaluations=BUDGET,maximum_gap_target_m=.001,
            plane_isolation_per_join_budget=12000,budgets_do_not_authorize_optimization=True),
        complete_xml_in_memory=True,new_xodr_files=0,nonlinear_solver_calls=0,export_allowed=False,
        web_changed=False,map_accepted=False,trial_admitted=False,
        remaining=['full-event shape intent beyond previous guard domain; surface category acceptance',
                   'source quantiles/movement validation and derived station/provenance updates',
                   'candidate receipt and persistent atomic application/independent consumer gate',
                   'one explicitly registered finite joint optimization budget'],
        input_bindings=len(bindings),input_drift=0,web_binding_counts=[len(b) for b in web.values()],
        artifact_sha256={p.name:digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})
    atomic(DEST/'status.json',json_bytes(serial(status)))
    print(json.dumps(dict(status=status['status'],input_bindings=len(bindings),checks=checks,
        failures=len(failures),unresolved=len(unresolved),xsd_valid=compiled['xsd_1_5M_valid'],
        model_xml_jet_difference=compiled['maximum_world_jet_readback_error'],new_xodr_files=0)))


if __name__=='__main__': main()
