import copy
import json
from pathlib import Path
import pytest
from scripts.gen_all import _sha256
from scripts.review_source_turn_budget import classify
from scripts.seal_geometry_review import verify_full_source_budget


def case(tmp_path):
    fields={k:dict(source_reverse_turn_deg=2.,written_reverse_turn_deg=v,source_single_turn_observed=False)
            for k,v in zip(('left','right','center'),(2.1,7.5,2.4))}
    shape=dict(sha256='same actual XML',records=[dict(road='116',fields=fields)])
    path=tmp_path/'shape.json';path.write_text(json.dumps(shape),encoding='utf-8')
    budget=dict(sha256=shape['sha256'],input=str(path),input_sha256=_sha256(path),hashes={},
        records=[dict(road='116',fields={k:classify(v) for k,v in fields.items()})],failed_roads=['116'])
    return shape,budget,path


def test_compound_field_blocks_the_seal_even_without_single_turn_flag(tmp_path):
    s,b,p=case(tmp_path)
    assert verify_full_source_budget(s,b,p)==['116']


@pytest.mark.parametrize('change',['summary','verdict','inventory','sha','source'])
def test_missing_or_altered_source_budget_fails_closed(tmp_path,change):
    s,b,p=case(tmp_path)
    if change=='summary':b['failed_roads']=[]
    if change=='verdict':b['records'][0]['fields']['right']['status']='WITHIN_SOURCE_TURN_BUDGET'
    if change=='inventory':b['records']=[]
    if change=='sha':b['sha256']='other map'
    if change=='source':p.write_text('changed original review',encoding='utf-8')
    with pytest.raises(ValueError):verify_full_source_budget(s,b,p)
