import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
import xml.etree.ElementTree as ET

from mapforge.ops.arc_source_chart import ArcChart
from mapforge.ops.movable_source_support import clip_original_plane
from mapforge.ops.movable_source_support import working_side_order, original_cut_heading
from mapforge.ops.port_dependencies import LanePort, revision
from mapforge.ops.source_roles import resolve_original_source_roles
from mapforge.ops.source_port_domain import SourcePortDomain
from mapforge.ops.coupled_source_junction import WholeSourceGeometryState
from spikes.source_contact_fit import SourceBoundaryBlock
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from spikes.whole_source_parent import OriginalParentBoundaryBlock, original_greville_seed
from tests.test_source_contact_fit import straight_fixture
from tests.test_source_role_composition import reseal_target


def test_tangent_source_coordinates_do_not_replace_baseline_arc_or_original_vertices():
    root, scope, domain, contacts, _ = straight_fixture()
    g = root.find('road/planView/geometry')
    g.remove(g.find('line')); ET.SubElement(g,'arc',curvature='.0001')
    scope['base_revision'] = revision(root); reseal_target(scope, domain, contacts)
    roles = resolve_original_source_roles(scope, domain, contacts)
    before = copy.deepcopy((scope,domain,contacts,roles)); base = revision(root)
    with pytest.raises(ValueError,match='exact Line'):
        SourceBoundaryBlock(root,scope,domain,contacts,roles)
    original = OriginalParentBoundaryBlock(root,scope,domain,contacts,roles)
    model = ArcSourceBoundaryBlock(original,curvature=.0001)
    x = original_greville_seed(model)
    assert x.shape == (model.nvar,) and np.isfinite(x).all()
    assert revision(root) == base and root.find('road/planView/geometry/arc') is not None
    assert (scope,domain,contacts,roles) == before
    assert original.original_parent_chart['curvature'] == .0001
    for key in model.source_xy:
        # Raw world points, not points sampled from the old Arc.
        assert model.source_xy[key][:,0] == pytest.approx(original.raw[key][:,0],abs=1e-12)


@pytest.mark.parametrize('k', [0., .002, -.002])
@pytest.mark.parametrize('reverse', [False, True])
def test_original_tail_split_uses_exact_current_normal_plane_without_source_mutation(k,reverse):
    axis = ArcChart((2.,3.),.15,k)
    xy = axis.world(np.c_[[0.,10.,30.],[1.,2.,4.]])
    if reverse: xy=xy[::-1].copy()
    before=xy.copy(); s=15.
    a=clip_original_plane(xy,axis,s,True); b=clip_original_plane(xy,axis,s,False)
    assert axis.project(a[0])[0] == pytest.approx(s,abs=1e-10)
    assert a[0] == pytest.approx(b[-1],abs=1e-12)
    assert a[-1] == pytest.approx(xy[-1]) and b[0] == pytest.approx(xy[0])
    # Both complementary slices retain all original vertices. Their split is
    # one point on an ORIGINAL straight segment, not a fitted geometry knot.
    for p in xy: assert any(np.array_equal(p,q) for q in np.vstack([a,b]))
    assert np.array_equal(xy,before)


def test_outside_or_folded_original_tail_is_not_silently_extended_or_sorted():
    axis=ArcChart((0.,0.),0.,0.)
    with pytest.raises(ValueError,match='outside'):
        clip_original_plane([[0,0],[10,0]],axis,11,True)
    with pytest.raises(ValueError,match='nonmonotone'):
        clip_original_plane([[0,0],[10,0],[5,0]],axis,6,True)


def test_zero_length_source_tail_keeps_original_tangent_without_extra_geometry():
    from mapforge.ops.world_curve_fairness import source_turn_evidence
    axis=ArcChart((0,0),0,0);before=np.array([[0.,0.],[10.,0.]])
    after=np.array([[30.,0.],[40.,0.]]);via=np.array([[10.,0.],[20.,1.],[30.,0.]])
    results=[];legacy=[]
    for gap in (0.,1e-5,1e-9):
        raw=np.vstack([clip_original_plane(before,axis,10-gap,True),via,
                       clip_original_plane(after,axis,30,False)])
        ends={'center':(original_cut_heading(before,axis,10-gap,True),
                        original_cut_heading(after,axis,30,False))}
        assert ends['center']==(0.,0.)
        results.append(source_turn_evidence({'center':raw},endpoint_headings=ends)['center'])
        legacy.append(source_turn_evidence({'center':raw})['center'])
    assert legacy[0]['reverse_turn_deg']!=legacy[1]['reverse_turn_deg']
    for r in results[1:]:assert r==pytest.approx(results[0])
    assert np.array_equal(before,[[0,0],[10,0]]) and np.array_equal(after,[[30,0],[40,0]])


def source_port_fixture(monkeypatch):
    from tests.test_junction_edges import network
    from mapforge.ops import source_port_domain
    root=network();lane=root.find("road[@id='10']/lanes/laneSection/right/lane")
    import xml.etree.ElementTree as stdET
    stdET.SubElement(lane,'userData',code='mapforge.source_lane',value='a')
    chains=[dict(source_ids=['a'],a=0.,b=60.,direction='with_s',domains=[dict(a=0.,b=60.,left='l',right='r')]),
            dict(source_ids=['other'],a=20.,b=50.,direction='against_s',domains=[])]
    monkeypatch.setattr(source_port_domain,'source_chains',lambda _:chains)
    model=SimpleNamespace(scope={'base_revision':revision(root)},road='10',owner={'l':0,'r':1},
        raw={'l':np.array([[0.,2.],[60.,2.]]),'r':np.array([[0.,-2.],[60.,-2.]])})
    return root,model,chains


def test_source_cut_never_forces_asymmetric_exterior_into_one_common_endpoint(monkeypatch):
    root,model,chains=source_port_fixture(monkeypatch);p=SourcePortDomain(model,root)
    state=p.evaluate(model,[0.,-.5]);port=LanePort('10','end',-1)
    assert state['stations'][port] == 59.5
    assert state['cuts'][0] == 0.  # Not max(0,20).
    assert state['original_chain_domains'][1]['a'] == 20.
    assert state['tails'][0]['full_chain_s'] == [0.,60.]
    assert state['tails'][0]['junction_tail_s'] == [59.5,60.]
    assert not state['external_common_cut_imposed'] and not state['aggregate_ownership_accepted']
    with pytest.raises(ValueError,match='unused exterior'):
        p.evaluate(model,[.1,0.])
    with pytest.raises(ValueError,match='left its original'):
        p.evaluate(model,[0.,1.])
    with pytest.raises(ValueError,match='unique original'):
        chains.append(copy.deepcopy(chains[0]));SourcePortDomain(model,root)


def test_only_the_three_explicitly_confirmed_new_endpoints_are_in_new_decision():
    p=Path(__file__).resolve().parents[1]/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml'
    d=yaml.safe_load(p.read_text(encoding='utf8'))
    assert d['authorization']['response']=='继续'
    assert {(r['source_lane_id'],r['contact']) for r in d['decisions']} == {
        ('2023041111104060474','start'),('2023041111104128071','start'),('2023041810470164021','start')}
    assert d['source_contact_sha256']=='88dab542381e54b7b5e89c274b164561e925b0efe9758bd15feda4164cff5c34'


@pytest.mark.parametrize('reversed_labels',[False,True])
def test_working_sides_follow_same_boundary_identities_at_both_ports_not_profile_names(reversed_labels):
    ports=(LanePort('10','end',-1),LanePort('11','start',-1))
    parents={};scope={'observations':{}}
    for i,p in enumerate(ports):
        hi,lo=f'boundary:B:{i}:hi',f'boundary:B:{i}:lo'
        parents[p.road]=SimpleNamespace(original=SimpleNamespace(owner={hi:0,lo:1}),port_features={p:(hi,lo)})
        keys=(lo,hi) if reversed_labels else (hi,lo)
        scope['observations'][str(i)]={'boundary_relations':[dict(declared_side=s,boundary_key=k.removeprefix('boundary:'))
            for s,k in zip(('left','right'),keys)]}
    before=copy.deepcopy(scope)
    assert working_side_order(parents,ports,[['0'],['1']],scope)==(('right','left') if reversed_labels else ('left','right'))
    assert scope==before
    # One endpoint contradicts the other; never guess a cross-lane connection.
    pairs=scope['observations']['1']['boundary_relations']
    pairs[0]['boundary_key'],pairs[1]['boundary_key']=pairs[1]['boundary_key'],pairs[0]['boundary_key']
    with pytest.raises(ValueError,match='disagrees'):
        working_side_order(parents,ports,[['0'],['1']],scope)


def test_full_geometry_binding_refuses_any_missing_shape_or_source_support():
    ports=SimpleNamespace(connectors=['100','101'])
    with pytest.raises(ValueError,match='all actual'):
        WholeSourceGeometryState(ports,{'100':{}},{'100':{}})
    with pytest.raises(ValueError,match='all actual'):
        WholeSourceGeometryState(ports,{'100':{},'101':{}},{'100':{}})


def test_dynamic_source_data_cannot_reuse_a_cached_old_geometry_fidelity():
    from tests.test_coupled_source_junction import real_kernel_fixture
    _,_,kernel=real_kernel_fixture();v=kernel['unpack'](kernel['initial'])
    old=kernel['evaluate'](v)
    curves={k:old[3][k] for k in ('left','right','center')}
    # Evaluate a deliberately distant new source set without changing curve
    # coefficients or frames. The old cached source result must not be reused.
    raw={k:np.array([[1e3,1e3],[1e3+10,1e3],[1e3+20,1e3+1.]]) for k in curves}
    result=kernel['evaluate'](v,source_data=dict(raw=raw,via=raw,tails=[]))
    assert min(result[5]) < -100
    assert kernel['evaluate'](v) is old


def test_approved_navigation_path_cannot_become_physical_center_constraint_again():
    from tests.test_coupled_source_junction import real_kernel_fixture
    from spikes.measured_connector_caps import sample
    _,_,kernel=real_kernel_fixture();v=kernel['unpack'](kernel['initial'])
    r=kernel['evaluate'](v);points,_,_=sample(r[0],r[1],r[2],step=.2)
    data=dict(raw=copy.deepcopy(points),via=copy.deepcopy(points),tails=[],
        separate_movement_ids=['synthetic-reviewed-path'],physical_center_parts=[points['center']],
        movement_path_observations=[dict(source_lane_id='synthetic-reviewed-path',
            points=np.array([[0.,1000.],[10.,1000.],[20.,1000.]]),validation_required=True)])
    a=kernel['evaluate'](v,source_data=data)
    changed=copy.deepcopy(data);changed['raw']['center']+=1000.
    b=kernel['evaluate'](v,source_data=changed)
    # Boundary-derived physical geometry sees the same physical evidence;
    # the retained navigation observation still needs a SEPARATE path check.
    assert b[4] == pytest.approx(a[4],abs=1e-12)
    assert b[5] == pytest.approx(a[5],abs=1e-12)
    assert b[11] == pytest.approx(a[11],abs=1e-12)
    assert len(changed['movement_path_observations'][0]['points'])==3
    changed['physical_center_parts']=[]
    with pytest.raises(ValueError,match='physical via'):
        kernel['evaluate'](v,source_data=changed)
