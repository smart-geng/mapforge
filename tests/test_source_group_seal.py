import copy
from types import SimpleNamespace
import pytest
import scripts.seal_source_group_review as seal


def case(monkeypatch):
    # Deliberately include one unchanged movement outside both mutable parents.
    graph=SimpleNamespace(connections={
        '100':(SimpleNamespace(road='10'),SimpleNamespace(road='30')),
        '101':(SimpleNamespace(road='10'),SimpleNamespace(road='13')),
        '110':(SimpleNamespace(road='11'),SimpleNamespace(road='12'))},
        validate_junction_table=lambda:True)
    monkeypatch.setattr(seal,'PortDependencies',lambda root:graph)
    trial=dict(parent_components={'10':'north','30':'west-exit'},affected_roads=['100','101'],
        connectors=[dict(road='100'),dict(road='101')])
    packet=dict(connectors=[dict(road=r) for r in graph.connections])
    geometry=dict(rows=[dict(road=r,geometry_status='PASS') for r in graph.connections],geometry_failed_roads=[])
    shape=dict(records=copy.deepcopy(packet['connectors']))
    parents=[dict(road='10'),dict(road='30')]
    return trial,packet,geometry,shape,parents,None


def test_all_parents_and_even_unchanged_movements_required(monkeypatch):
    args=list(case(monkeypatch));seal.require_group_inventory(*args)
    args[4]=args[4][:1]
    with pytest.raises(ValueError,match='every source parent'):seal.require_group_inventory(*args)
    args=list(case(monkeypatch))
    for obj,key in zip(args[1:4],('connectors','rows','records')):obj[key]=obj[key][:2]
    with pytest.raises(ValueError,match='all explicit movements'):seal.require_group_inventory(*args)


@pytest.mark.parametrize('kind',['turn_duplicate','parent_duplicate','missing_dependency'])
def test_group_cannot_relabel_partial_shared_state_as_full(monkeypatch,kind):
    args=list(case(monkeypatch))
    if kind=='turn_duplicate':args[0]['connectors']=[dict(road='100')]*2
    if kind=='parent_duplicate':args[4]=[dict(road='10')]*2
    if kind=='missing_dependency':args[0]['affected_roads']=['100']
    with pytest.raises(ValueError):seal.require_group_inventory(*args)
