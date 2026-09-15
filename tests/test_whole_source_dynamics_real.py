"""Replay stored float curves, not final XML/esmini or geometry acceptance."""
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code
from spikes.road_boundary_family import world_kinematics

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'out/node4-whole-dynamics-r1-20260914'


def read(name):return json.loads((OUTPUT/name).read_text(encoding='utf8'))


@pytest.fixture(scope='module')
def packet():
    if not (OUTPUT/'report.json').exists():pytest.skip('local real node4 dynamics check not installed')
    report=read('report.json')
    current,changes=bind_current_code(report['input_files_sha256']);unchanged(current)
    return report,read('parent-spans.json'),read('turn-spans.json')


def values(row,u):
    c=row['cubic_coefficients'];s=np.asarray(u)*(row['end']-row['start'])
    jets=np.stack([np.polynomial.polynomial.polyval(s,np.polynomial.polynomial.polyder(c,j)) for j in range(4)],axis=-1)
    k=row['curvature']+row['sharpness']*s
    return abs(world_kinematics(jets,k,row['sharpness']))*[(row['speed_kmh']/3.6)**2,(row['speed_kmh']/3.6)**3]


def test_real_all29_same_registered_geometry_and_every_check_span_is_retained(packet):
    report,parents,turns=packet
    assert set(parents)=={'10','11','12','13','30'}
    assert set(turns)==set(map(str,range(100,124)))
    assert report['parent_reference_primitives']==5 and report['turn_reference_primitives']==120
    assert report['minimum_reference_primitive_m']>=6
    assert not report['geometry_segments_added'] and not report['geometry_optimized']
    old=json.loads((ROOT/'out/node4-whole-source-model-r4-20260914/whole-coordinate-state.json').read_text(encoding='utf8'))
    np.testing.assert_array_equal(read('coordinate-state.json')['vector'],old['initial'])
    for rows in [*parents.values(),*turns.values()]:
        for r in rows:
            leaves=sorted(r['check']['interval_leaves'],key=lambda a:a['u'][0])
            assert leaves[0]['u'][0]==0 and leaves[-1]['u'][1]==1
            assert all(a['u'][1]==b['u'][0] for a,b in zip(leaves[:-1],leaves[1:]))
    assert not report['export_allowed'] and not report['xodr_generated']


def test_actual_violation_witnesses_replay_and_bounded_leaves_hold_dense_readback(packet):
    _,parents,turns=packet;witnesses=0;bounded=0
    for rows in [*parents.values(),*turns.values()]:
        for r in rows:
            for leaf in r['check']['interval_leaves']:
                if leaf['status']=='FAIL':
                    assert leaf['reason']=='actual-point-dynamic-exceedance'
                    j=('ay_mps2','lateral_rate_mps3').index(leaf['metric'])
                    exact=float(values(r,leaf['witness_u'])[j])
                    assert exact==pytest.approx(leaf['witness_value'],rel=1e-10,abs=1e-9)
                    assert exact>leaf['limit']
                    witnesses+=1
                elif leaf['status']=='BOUNDED':
                    lo,hi=leaf['u'];n=max(5,int(np.ceil((hi-lo)*(r['end']-r['start'])/.1))+1)
                    val=values(r,np.linspace(lo,hi,n))
                    assert np.all(val<=np.array(leaf['upper'])+1e-9)
                    assert np.all(val<=[2.5,1.])
                    bounded+=1
    assert witnesses>0 and bounded>0


def test_mixed_speed_envelope_is_not_actual_turn_speed_or_closed_domain(packet):
    report,_,_=packet
    bindings=[r['speed_binding'] for r in report['turns'].values()]
    assert any(not b['source_interval_mapping_complete'] for b in bindings)
    for b in bindings:
        speeds={r['source_speed_kmh'] for r in b['rows']}
        assert b['evaluation_speed_kmh']==max(speeds)
        assert b['source_interval_mapping_complete']==(len(speeds)==1)
        assert not b['source_speed_changed'] and not b['approved_movement_design_speed']
    assert not report['source_interval_speed_mapping_complete']
    assert not report['full_original_written_domain_accepted']
    assert not report['physical_surface_accepted'] and not report['full_map_dynamics_accepted']
