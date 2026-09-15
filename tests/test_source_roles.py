import copy

import pytest

from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.ops.source_roles import resolve_source_roles,resolve_original_source_roles
from tests.test_source_contacts import source_fixture, seal


def decision_for(contacts):
    return {'schema':'mapforge.source-role-decision/v1','id':'test-source-roles','version':1,
            'authorization':{'basis':'user-confirmed-in-current-thread','date':'2026-09-11','scope':'test fixture only'},
            'source_contact_sha256':contacts['content_sha256'],
            'decisions':[{'source_lane_id':c['source_lane_id'],'contact':c['contact'],
                          'physical_authority':'original_left_right_boundaries','lane_path_role':'movement_path_observation'}
                         for c in contacts['role_conflicts']]}


def fixture():
    r,s,p,d=source_fixture();m=compile_source_contacts(r,s,p,d)
    return r,s,p,d,m,decision_for(m)


def test_explicit_interpretation_preserves_every_original_and_only_scoped_path_roles():
    *_,p,d,m,decision=fixture();before=copy.deepcopy((p,d,m,decision))
    roles=resolve_source_roles(p,d,m,decision)
    assert roles['role_binding_complete'] and not roles['unresolved']
    assert roles['feature_roles']['lane:c']['role']=='movement_path_observation'
    assert roles['feature_roles']['lane:a']['physical_geometry_constraint']
    assert roles['feature_roles']['boundary:B:cl']['physical_geometry_constraint']
    assert roles['feature_roles']['lane:c']['movement_path_check_required']
    assert not roles['export_allowed'] and not roles['movement_paths_validated']
    assert (p,d,m,decision)==before
    for key,feature in roles['feature_roles'].items():assert feature['parts']==m['full_source_support'][key]['parts']


@pytest.mark.parametrize('fault',['stale','other_lane','duplicate','discard','authority','version','body','source_issue'])
def test_role_override_rejects_broadening_staleness_and_corruption(fault):
    *_,p,d,m,decision=fixture()
    if fault=='stale':decision['source_contact_sha256']='another-source'
    elif fault=='other_lane':decision['decisions'][0]['source_lane_id']='b'
    elif fault=='duplicate':decision['decisions']*=2
    elif fault=='discard':decision['decisions'][0]['lane_path_role']='ignore'
    elif fault=='authority':decision['authorization']['basis']='auto-inferred'
    elif fault=='version':decision['version']=2
    elif fault=='body':p['observations'].pop('c')
    else:m['issues']=[{'code':'missing'}];seal(m);decision['source_contact_sha256']=m['content_sha256']
    with pytest.raises(ValueError):resolve_source_roles(p,d,m,decision)


def test_missing_decision_stays_unresolved_not_automatic():
    *_,p,d,m,decision=fixture();decision['decisions']=[]
    roles=resolve_source_roles(p,d,m,decision)
    assert not roles['role_binding_complete'] and len(roles['unresolved'])==1
    assert roles['feature_roles']['lane:c']['physical_geometry_constraint']


def test_no_conflict_admission_preserves_all_originals_without_user_approval():
    from tests.test_source_contacts import reseal
    r,s,p,d=source_fixture()
    # Fixture source path starts at the actual collapsed physical boundaries.
    xy=d['partition']['features']['boundary:B:cl']['parts'][0]['raw_vertices'][0]
    p['observations']['c']['raw_records'][0]['parts'][0][0]=list(xy)
    d['partition']['features']['lane:c']['parts'][0]['raw_vertices'][0]=list(xy)
    import numpy as np
    from mapforge.validate.shp_boundary_fidelity import _project
    part=d['partition']['features']['lane:c']['parts'][0]
    length=float(np.linalg.norm(np.diff(_project(np.asarray(part['raw_vertices']),0.,0.),axis=0)))
    part['source_vertex_s_m']=[0.,length];part['length_m']=length
    part['atoms'][0]['source_s_m']=[0.,length]
    # An unpaired physical birth: no invented source center-to-center TOPO.
    s.rows=[row for row in s.rows if row['attributes']['T']!='c']
    reseal(p,d);m=compile_source_contacts(r,s,p,d)
    assert not m['role_conflicts']
    before=copy.deepcopy((p,d,m))
    result=resolve_original_source_roles(p,d,m)
    assert result['role_binding_complete'] and result['resolved']==[]
    assert result['decision']['authorization']['basis']=='no-conflicts-original-observations'
    assert all(f['physical_geometry_constraint'] and not f['movement_path_check_required']
               for f in result['feature_roles'].values())
    assert (p,d,m)==before and not result['export_allowed']
    m['source_endpoint_inventory'].pop();seal(m)
    with pytest.raises(ValueError,match='complete endpoint inventory'):resolve_original_source_roles(p,d,m)


def test_no_conflict_admission_never_bypasses_real_role_decisions():
    *_,p,d,m,decision=fixture()
    with pytest.raises(ValueError,match='cannot resolve any'):resolve_original_source_roles(p,d,m)
    decision['authorization']={'basis':'no-conflicts-original-observations'}
    with pytest.raises(ValueError,match='cannot resolve any'):resolve_source_roles(p,d,m,decision)
