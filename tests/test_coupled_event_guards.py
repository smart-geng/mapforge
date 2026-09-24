import copy
import math
import numpy as np
import pytest
from shapely.geometry import Polygon, box
from pyclothoids import Clothoid

from scripts.prepare_coupled_event_contract import load_contract
from mapforge.repair_web.coupled_event_guards import CoupledEventGuards, reference_turn
from mapforge.repair_web.coupled_event_sources import EventSources, sample_ribbon
from mapforge.repair_web.coupled_event_surface import EventSurface, surface_summary, xml_bands
from mapforge.repair_web.coupled_event_layout import check_parent_records
from mapforge.repair_web.coupled_event_state import world_state
from mapforge.repair_web.event_shape_admission import point_to_original
from scripts.internal_edge_jets import states


@pytest.fixture(scope='module')
def engine(): return CoupledEventGuards(load_contract())


@pytest.fixture(scope='module')
def reference(engine): return engine.evaluate(reference=True)


def test_complete_source_ownership_without_path_midpoint_equations(engine):
    inventory=engine.sources.inventory()
    assert inventory['source_consumer_parts']==54
    assert inventory['physical_consumers']==36 and inventory['independent_movement_consumers']==18
    assert len(inventory['empty_owned_parts'])==1
    assert inventory['empty_owned_parts'][0]['connector']=='106'
    assert inventory['source_vertices_removed']==inventory['synthetic_joins']==0
    for r in engine.sources.rows:
        assert len(r['full_original_xy'])==len(r['original_arclength_m'])
        if r['role']=='via':
            assert r['owned_intervals_m']==[[0.,r['original_arclength_m'][-1]]]
            assert len(r['owned_fragments'][0])==len(r['full_original_xy'])


def test_wrong_source_port_or_side_does_not_choose_nearest(engine):
    c=load_contract(); c.domain=copy.deepcopy(c.domain)
    r=next(r for r in c.domain['comparison']['actual'] if r['connector']=='106')
    r['incoming_port']['lane']=-4
    with pytest.raises(ValueError,match='physical ports'): EventSources(c)


def test_changed_cut_cannot_discard_failed_tail(engine):
    c=load_contract(); c.domain=copy.deepcopy(c.domain)
    for f in c.domain['partition']['features'].values():
        for p in f['parts']:
            for cut in p['consumer_cuts']:
                if cut['owner']=='connector:111' and cut['source_lane_id']=='2023041111104141678': cut['intervals_m']=[]
    with pytest.raises(ValueError,match='frozen intervals'): EventSources(c)


def test_parent_full_original_segments_and_old_failures_preserved(reference):
    p=reference['parent']
    assert p['source_max_m']==pytest.approx(.74999770075,abs=1e-8)
    assert p['source_local_research_failures']==0 and p['source_ordinary_final_failures']>0
    assert abs(p['target_s178']['error_m'])==pytest.approx(.1253632057,abs=1e-8)
    assert p['old_guard_failures']==[]  # old XML vs its EXACT old guard baseline
    assert p['old_guard_domain_m']==[100.,201.7074107]
    assert p['independent_parent_movement_paths']==19 and not p['movement_center_equality_applied']
    assert len(p['finite_physical_endpoint_checks'])==2


def test_reference_tail_failures_not_averaged_with_via(reference):
    failed={(cid,r['field'],r['role']) for cid,t in reference['turns'].items() for r in t['ordinary_tail_rows'] if r['sampled_violation']}
    assert failed=={('106','left','predecessor'),('107','left','predecessor'),('107','right','predecessor'),
                    ('110','right','predecessor'),('111','right','predecessor')}
    for t in reference['turns'].values():
        assert len(t['movement_observations'])==3
        assert not t['source_fidelity_continuous_pass']
        assert t['chord_error_bound_m']<=.0001+1e-12
        assert t['output_primitives_added']==0
        assert t['center_fairing_acceptance'].startswith('NOT_EVALUATED')


def test_actual_xml_reference_intervals_match_independent_readback(engine):
    road=engine.contract.graph.roads['111']; t=reference_turn(road)
    mesh=sample_ribbon(t); pos=np.argmin(abs(mesh['stations']-6.))
    # s=6 is a real width/reference break, not just a nearest draw sample.
    assert mesh['stations'][pos]==pytest.approx(6.,abs=1e-12)
    actual=states(road,-1,6.,False)
    # For lane -1, the independent reader returns [inner/left, outer/right].
    assert mesh['points']['left'][pos]==pytest.approx(actual[0][:2],abs=1e-9)
    assert mesh['points']['right'][pos]==pytest.approx(actual[1][:2],abs=1e-9)


def test_chord_error_bound_on_analytic_long_curve():
    ref=Clothoid.StandardParams(0,0,.2,.04,.001,30.)
    co=dict(left=np.array([[2.,.03,.0002,-.00001]]),right=np.array([[-2.,-.01,.0001,.00001]]))
    turn=dict(refs=[ref],stations=np.array([0.,30.]),coefficients=co)
    mesh=sample_ribbon(turn,step=.25)
    for side in co:
        for i in range(len(mesh['stations'])-1):
            s=(mesh['stations'][i]+mesh['stations'][i+1])/2
            midpoint=world_state(ref,co[side][0],s)[:2]
            chord=(mesh['points'][side][i]+mesh['points'][side][i+1])/2
            assert np.linalg.norm(midpoint-chord)<=mesh['chord_error_bound_m']+1e-10
    assert mesh['output_primitives_added']==0 and len(turn['refs'])==1


def row(polygon,kind='driving',aux=False):
    return dict(road='test',section=0,lane=1,lane_type=kind,auxiliary=aux,polygon=polygon,minimum_sampled_width_m=1.)


def test_auxiliary_fill_does_not_become_driving_coverage():
    source=box(0,0,10,10)
    r=surface_summary([row(source,'restricted',True),row(box(0,0,4,10))],source)
    assert r['visible']['source_junction_missing_m2']==0
    assert r['driving']['source_junction_missing_m2']==pytest.approx(60.)
    assert r['auxiliary_is_not_driving'] and not r['whole_map_accepted']


def test_self_intersection_is_reported_not_fixed_with_buffer():
    p=Polygon([(0,0),(2,2),(0,2),(2,0)])
    r=surface_summary([row(p)],box(0,0,2,2))
    assert r['status']=='INVALID_BANDS_NO_REPAIR' and r['invalid_bands']
    assert 'visible' not in r and not r['buffer_or_hull_repair_used']


def test_original_island_is_not_treated_as_missing_pavement():
    source=Polygon(box(0,0,10,10).exterior,[box(3,3,7,7).exterior])
    r=surface_summary([row(source)],source)
    assert r['source_islands_preserved']==1
    assert r['visible']['source_junction_missing_m2']==0
    assert r['visible']['holes_m2']==[16.]


def test_record_level_width_stack_readback_retains_long_polynomials(engine):
    p=engine.parent; snapshot=engine.model.evaluate(engine.model.diagnostic)
    r=check_parent_records(engine.contract,snapshot['parent_coefficients'])
    assert r['width_records']==35 and r['lane_offset_records']==9
    assert r['maximum_stack_readback_error_m']<1e-10
    assert len(r['short_semantic_reexpressions'])==5
    assert max(i['same_long_polynomial_error_m'] for i in r['short_semantic_reexpressions'])<1e-10
    assert not r['actual_xodr_written'] and not r['full_writer_admitted']


def test_common_state_still_fails_and_cannot_be_exported(engine):
    result=engine.evaluate(engine.model.diagnostic)
    assert result['parent']['source_max_m']>5. and result['parent']['old_guard_failures']
    assert not result['trial_admitted'] and not result['map_accepted']
    assert result['optimizer_calls']==0 and result['unresolved']
    with pytest.raises(ValueError): engine.model.compile()


def test_common_state_guards_do_not_launch_optimizers(engine,monkeypatch):
    import scipy.optimize as opt
    def forbidden(*a,**k): raise AssertionError('No registered trial')
    for name in ('minimize','least_squares','root','linprog'): monkeypatch.setattr(opt,name,forbidden)
    result=engine.evaluate(engine.model.diagnostic)
    assert result['optimizer_calls']==0


def test_full_actual_surface_distinguishes_footprints_and_numerical_caps(engine):
    result=EventSurface(engine.contract).evaluate()
    assert not result['invalid_bands']
    assert 0<result['numeric_zero_cap_correction_max_m']<1e-12
    assert result['driving']['source_junction_missing_m2']>result['visible']['source_junction_missing_m2']
    assert not result['whole_map_accepted']
