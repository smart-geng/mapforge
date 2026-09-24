"""Read-only representation/rank review. No fitting, optimiser or new XODR.

Recover the existing XML in its registered basis, impose exact polynomial
zero-change restrictions, and measure the remaining linear edit space. This
is necessary-control-capacity evidence, NOT geometric feasibility or a map.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.check_outer_event_control import inputs, SOURCE, REFERENCE, PACKET, DECISION
from mapforge.repair_web.model import atomic, json_bytes, digest


def frozen_rows(event, editable_edges, lo, hi):
    """Exact polynomial preservation outside selection, not sampled pinning."""
    if (not editable_edges or not set(editable_edges)<=set(event.splines)
            or not event.start<=lo<hi<=event.end):
        raise ValueError('Explicit existing edges and contained interval required')
    rows=[]
    for edge,bs in event.splines.items():
        cuts=sorted(set(bs.t)|{s for s in (lo,hi) if min(bs.t)<s<max(bs.t)})
        for a,b in zip(cuts,cuts[1:]):
            if edge in editable_edges and lo<=a and b<=hi:continue
            rows.extend(event.power(edge,a)*np.array([1.,b-a,(b-a)**2,(b-a)**3])[:,None])
    return np.array(rows).reshape(-1,event.nvar)


def capacity(event,handle,editable_edges,lo,hi):
    F=frozen_rows(event,editable_edges,lo,hi)
    A=F@event.Z
    # Bounded scaling avoids inflating near-zero rows due to elimination noise.
    norms=np.linalg.norm(A,axis=1);A=A/np.maximum(norms,1e-6)[:,None]
    _,singular,Vh=np.linalg.svd(A,full_matrices=True)
    threshold=max(A.shape)*np.finfo(float).eps*max(1.,float(singular[0]))
    ranks=[int(np.sum(singular>threshold*scale)) for scale in (1.,10.,100.)]
    rank=ranks[1];space=event.Z@Vh[rank:].T
    influence=float(np.linalg.norm(handle@space)) if space.shape[1] else 0.
    return dict(editable_edges=list(editable_edges),interval_m=[lo,hi],
                shared_free_before=event.Z.shape[1],free_after_preservation=space.shape[1],
                rank_by_tolerance=ranks,rank_stable=len(set(ranks))==1,
                singular_values=singular.tolist(),threshold_base=threshold,
                original_constraint_residual=float(np.max(abs(event.E@space))) if space.size else 0.,
                frozen_polynomial_residual=float(np.max(abs(F@space))) if space.size else 0.,
                target_response_norm=influence,target_has_linear_freedom=influence>1e-10,
                target_feasibility='NOT_EVALUATED',nonlinear_guards='NOT_EVALUATED',map_accepted=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();dest=args.out.resolve()
    if dest==ROOT/'out' or not dest.is_relative_to(ROOT/'out'):p.error('new workspace out child required')
    dest.mkdir(exist_ok=False)
    binding=dict(json.loads(PACKET.with_name('run.json').read_text(encoding='utf8'))['input_files_sha256'])
    paths=[SOURCE,REFERENCE,PACKET,DECISION,Path(__file__).resolve(),
           *[ROOT/('mapforge/repair_web/'+n+'.py') for n in ('model','outer_event','outer_event_control','outer_event_shape')],
           *[ROOT/('scripts/'+n+'.py') for n in ('check_outer_event_control','check_outer_event_written','check_outer_event_shape','internal_edge_jets')]]
    for path in paths:binding[str(path)]=digest(path.read_bytes())
    def verify():
        for path,sha in binding.items():
            if digest(Path(path).read_bytes())!=sha:raise ValueError('Original/code drift: '+path)
    verify();event,control=inputs()
    # Diagnostic domains, not new edit permissions or fitting candidates.
    domains=[('whole-event-edge3-only',[3],event.start,event.end),
             ('tail-three-spans-edge3',[3],event.scope.knots[6],event.end),
             ('tail-four-spans-edge3',[3],event.scope.knots[5],event.end),
             ('tail-four-spans-edges3-4',[3,4],event.scope.knots[5],event.end)]
    rows=[dict(id=name,**capacity(event,control.handle,edges,lo,hi)) for name,edges,lo,hi in domains]
    result=dict(schema='mapforge/edit-structure-review/v1',reference_sha256=control.reference_sha256,
                nvar=event.nvar,common_free=event.Z.shape[1],control_station_m=control.point.station,
                original_knots=list(event.scope.knots),domains=rows,optimizer_calls=0,new_xodr=False,
                map_accepted=False,scope='fixed Line chart and current exact C2 cubic basis only; not general impossibility')
    verify();atomic(dest/'binding.json',json_bytes(binding));atomic(dest/'rank-review.json',json_bytes(result))
    print(json.dumps(dict(domains=[{k:r[k] for k in ('id','free_after_preservation','rank_stable','target_response_norm')} for r in rows],
                          input_bindings=len(binding),optimizer_calls=0,new_xodr=False)))


if __name__=='__main__':main()
