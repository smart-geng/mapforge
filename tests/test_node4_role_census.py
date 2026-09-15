import copy
import pytest
from scripts.summarize_node4_role_census import remaining


def fixture():
    return dict(roads=['10','11','12','13'],raw_lane_count=2,
        raw_endpoint_inventory=[dict(source_lane_id=s,contact=c) for s in ('a','b') for c in ('start','end')],
        missing_compiled_conflicts=[],rows=[dict(source_lane_id=s,requires_source_role_decision=True,
            listed_decisions=[{'path':'existing exact-ID decision'}] if s=='a' else [],
            listed_in_existing_north_decision=False) for s in ('a','b')])


def test_only_new_roles_listed_and_original_packet_not_mutated():
    report=fixture(); before=copy.deepcopy(report)
    assert [r['source_lane_id'] for r in remaining(report)]==['b']
    assert report==before


@pytest.mark.parametrize('fault',['missing_arm','missing_end','duplicate','omitted_conflict'])
def test_no_full_scope_claim_from_partial_or_inconsistent_census(fault):
    report=fixture()
    if fault=='missing_arm':report['roads'].pop()
    elif fault=='missing_end':report['raw_endpoint_inventory'].pop()
    elif fault=='duplicate':report['raw_endpoint_inventory'][0]=report['raw_endpoint_inventory'][1]
    else:report['missing_compiled_conflicts']=['b']
    with pytest.raises(ValueError):remaining(report)
