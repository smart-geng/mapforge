"""Freeze independent geometry + shape + rendered evidence, fail closed."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump


def verify_full_source_budget(shape, budget, shape_path):
    """No N/A exemption or stale hand-copied failure list at final sealing."""
    from scripts.review_source_turn_budget import classify
    if (budget['sha256']!=shape['sha256'] or Path(budget['input']).resolve()!=shape_path.resolve()
        or budget['input_sha256']!=_sha256(shape_path)):
        raise ValueError('turn budget is not bound to this exact independent shape review')
    unchanged(budget['hashes'])
    original={r['road']:r for r in shape['records']};actual={r['road']:r for r in budget['records']}
    if (not original or len(original)!=len(shape['records']) or len(actual)!=len(budget['records'])
        or original.keys()!=actual.keys()):raise ValueError('whole declared turn inventory required')
    failed=[]
    for rid,row in original.items():
        if set(row['fields'])!={'left','right','center'}:raise ValueError('missing original field')
        expected={k:classify(v) for k,v in row['fields'].items()}
        if actual[rid]['fields']!=expected:raise ValueError('full-source turn verdict was altered')
        if any(v['status']=='EXCESS_SOURCE_TURN_BUDGET' for v in expected.values()):failed.append(rid)
    if set(failed)!=set(budget['failed_roads']):raise ValueError('failure summary omits a field/road')
    return failed


def run(directory):
    directory=Path(directory).resolve()
    files=[directory/p for p in ('report.json','independent-review/report.json',
        'independent-review/written-events.json','shape-review-sealed/report.json','esmini-review/esmini-captures.json',
        'shape-review-sealed/full-source-turn-budget.json')]
    trial,geometry,events,shape,render,budget=[json.loads(p.read_text(encoding='utf-8')) for p in files]
    path=Path(trial['artifact']);sha=_sha256(path)
    if any(r['sha256']!=sha for r in (trial,geometry,events,shape,render,budget)):
        raise ValueError('review stages belong to different XML files')
    unchanged(trial['source_hashes']);unchanged(shape['hashes'])
    all_shape_failed=verify_full_source_budget(shape,budget,files[3])
    result=dict(status='REJECTED_NOT_DELIVERY',artifact=str(path),sha256=sha,
        fidelity_interface_subset_pass_roads=geometry['geometry_pass_roads'],
        artificial_reverse_turn_roads=all_shape_failed,
        historical_single_turn_subset_failed_roads=shape['failed_roads'],
        full_source_turn_budget_required=True,
        ordinary_internal_failed_roads=sorted({r['road'] for r in geometry['all_map_internal_edges']['failures']}),
        source_events_status=events['status'],render_automatically_accepted=False,
        source_speed_dynamics_accepted=False,whole_surface_accepted=False,MAP_retested=False,
        full_shape_or_controller_certificate=False,production_accepted=False,
        reason='Point error and G2 alone do not forbid artificial waves; source roles and whole surface remain unresolved.',
        evidence_hashes={str(p):_sha256(p) for p in files+[Path(__file__)]})
    target=directory/'FINAL-REVIEW-STATUS.json'
    if target.exists():raise FileExistsError('preserve previous review seal')
    dump(target,result);print(result['status'],result['artificial_reverse_turn_roads'])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();run(a.directory)
