import copy
import pytest

from scripts.review_source_role_conflicts import inspect_tips,original_endpoint_inventory


def fixture():
    lane=dict(attributes=dict(LANE_PID='branch',S_WIDTH=0,E_WIDTH=3000,
                              S_LANE_PID='lane-start',E_LANE_PID='lane-end'),
              points=[[106.,29.],[106.0001,29.]])
    boundaries={side:dict(attributes=dict(BORDER_PID=side,START_PID='collapse',END_PID=side+'-end'),
                         points=[[106.,29.00002],[106.0001,29.00002+offset]])
                for side,offset in (('left',.00001),('right',-.00001))}
    return lane,boundaries,dict(lat_0=29.,lon_0=106.)


def test_original_tips_replay_with_reversed_boundary_and_without_role_override():
    lane,bounds,projection=fixture();original=copy.deepcopy((lane,bounds))
    row=inspect_tips(lane,bounds,'start',projection)
    assert row['gap_m']>2 and row['boundary_tip_separation_m']==0
    assert not row['role_override_applied'] and not row['general_geometry_impossibility_proven']
    assert row['conditional_minimum_common_radius_m']==pytest.approx(row['gap_m']/2)
    assert (lane,bounds)==original
    bounds['right']['points'].reverse()
    b=bounds['right']['attributes'];b['START_PID'],b['END_PID']=b['END_PID'],b['START_PID']
    reversed_row=inspect_tips(lane,bounds,'start',projection)
    assert reversed_row['gap_m']==row['gap_m']
    assert reversed_row['original_boundary_tips'][1]['vertex_index']==1


@pytest.mark.parametrize('fault',['missing-width','nonzero-width','missing-side','node-mismatch','not-collapsed'])
def test_no_default_width_or_geometric_pairing_can_manufacture_conflict(fault):
    lane,bounds,projection=fixture()
    if fault=='missing-width':lane['attributes'].pop('S_WIDTH')
    elif fault=='nonzero-width':lane['attributes']['S_WIDTH']=3000
    elif fault=='missing-side':bounds.pop('left')
    elif fault=='node-mismatch':bounds['left']['attributes']['START_PID']='different-node'
    else:bounds['left']['points'][0][1]+=.00001
    with pytest.raises(ValueError):inspect_tips(lane,bounds,'start',projection)


def test_raw_census_does_not_use_a_compilers_conflict_list():
    lane,_,_=fixture();lane.update(layer='L',record_index=7)
    scope={'observations':{'branch':{'raw_records':[lane],
        'identity_resolution':{'selected_layer':'L'},'start_width_known':True,'end_width_known':True,
        'start_width_mm':0,'end_width_mm':3000}}}
    seen=[]
    def read(layer,index):seen.append((layer,index));return copy.deepcopy(lane)
    def compare(a,b):assert a==b
    inventory,zeros=original_endpoint_inventory(scope,read,compare)
    assert seen==[('L',7)] and len(inventory)==2
    assert [(sid,role) for sid,role,_ in zeros]==[('branch','start')]
    scope['observations']['branch']['start_width_known']=False
    with pytest.raises(ValueError,match='presence'):original_endpoint_inventory(scope,read,compare)
