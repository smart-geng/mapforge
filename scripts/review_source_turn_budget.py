"""Supplemental whole-source turning-variation screen, including source S-bends.

Source turning budget is retained, not replaced by zero curvature reversals.
This is a numerical source-relative diagnostic, never a standards certificate.
"""
import argparse
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged


def classify(field):
    source=float(field['source_reverse_turn_deg']);actual=float(field['written_reverse_turn_deg'])
    if not math.isfinite(source) or not math.isfinite(actual) or min(source,actual)<0:
        raise ValueError('finite nonnegative original/written turn evidence required')
    return dict(status='EXCESS_SOURCE_TURN_BUDGET' if actual>source+1. else 'WITHIN_SOURCE_TURN_BUDGET',
        source_reverse_turn_deg=source,written_reverse_turn_deg=actual,added_reverse_turn_deg=actual-source,
        allowance_deg=1.,source_single_turn_observed=bool(field['source_single_turn_observed']),
        source_s_bend_forced_monotone=False,continuous_certificate=False,production_accepted=False)


def run(source,output):
    source=Path(source).resolve();output=Path(output).resolve()
    data=json.loads(source.read_text(encoding='utf-8'));unchanged(data['hashes'])
    if not data['records'] or any(set(row['fields'])!={'left','right','center'} for row in data['records']):
        raise ValueError('complete three-field source-linked turn inventory required')
    if _sha256(Path(data['artifact']))!=data['sha256']:raise ValueError('actual XML changed')
    records=[dict(road=row['road'],fields={k:classify(v) for k,v in row['fields'].items()}) for row in data['records']]
    failed=[r['road'] for r in records if any(v['status']=='EXCESS_SOURCE_TURN_BUDGET' for v in r['fields'].values())]
    result=dict(status='REJECTED_ADDITIONAL_SHAPE_DIAGNOSTIC' if failed else 'LIMITED_DIAGNOSTIC_NOT_ACCEPTANCE',
        artifact=data['artifact'],sha256=data['sha256'],records=records,failed_roads=failed,
        input=str(source),input_sha256=_sha256(source),
        input_method=data['method'],method='compare cumulative reverse turn to FULL original budget, including S/compound curves',
        scope='all fields of declared source-linked turns; no ordinary-road/surface or dynamic certificate',
        production_accepted=False,source_changed=False,
        hashes={str(p):_sha256(p) for p in (source,Path(__file__))})
    if output.exists():raise FileExistsError('preserve previous diagnosis')
    dump(output,result);print(result['status'],failed,flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');a=p.parse_args();run(a.source,a.output)
