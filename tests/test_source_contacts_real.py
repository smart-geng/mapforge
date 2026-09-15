"""Real readonly data; these tests do NOT generate or validate new geometry."""
import copy
import json
from pathlib import Path

import pytest
import numpy as np
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.ops.reconstruction_scope import prepare_scope, digest
from mapforge.ops.source_domain import prepare_source_domain
from scripts import prepare_source_contacts as job
from tests.test_source_domain_real import original, SOURCE_JID


@pytest.fixture(scope='module')
def contacts(original):
    root,source,_=original
    p=prepare_scope(root,source,['10'],{'source_junction_id':SOURCE_JID,'xodr_junction_id':'1'})
    d=prepare_source_domain(root,source,p,SOURCE_JID,'1')
    return root,source,p,d,compile_source_contacts(root,source,p,d)


def test_original_contact_evidence_preserves_all_vertices_and_unassigned_source_ids(contacts):
    root,source,p,d,m=contacts
    assert not m['issues'] and len(m['transition_events'])==20
    assert sum(len(e['contacts']) for e in m['transition_events'])==40
    assert max(v['gap_m'] for e in m['transition_events'] for v in e['contacts'])<1e-7
    assert sum(len(part['vertex_indices']) for f in m['full_source_support'].values() for part in f['parts'])==3009
    assert len(m['full_source_support'])==118
    assert len(m['previous_unassigned_intervals'])==54
    assert m['previous_unassigned_by_class_m']['SOURCE_CONTACT_BEYOND_LEGACY_CUT']==pytest.approx(14.6670046650,abs=1e-7)
    assert m['previous_unassigned_by_class_m']['PACKAGE_EXTERNAL_SOURCE_CONTACT']==pytest.approx(1.11766824375,abs=1e-7)
    assert d['partition']['status']=='UNRESOLVED'  # Never rewrite old coverage as repaired.
    assert not m['export_allowed'] and not m['geometry_solver_ran']
    assert all(not ev['scope_expansion_authorized'] for g in m['previous_unassigned_intervals'] for ev in g['external_contact_evidence'])


def test_two_zero_width_path_role_conflicts_are_preserved_not_snapped_or_dropped(contacts):
    root,source,p,d,m=contacts
    gaps={r['source_lane_id']:r['gap_m'] for r in m['role_conflicts']}
    assert gaps==pytest.approx({'2023081117223317273':1.74823205325,'2023081117251327605':2.17639076842})
    for c in m['role_conflicts']:
        assert c['source_point_retained'] and not c['automatic_override_allowed']
        assert c['conditional_infeasibility']['necessary_common_tube_radius_m']>.35
        original_source=source.lane(c['source_lane_id']).geometry.tolist()
        assert np.array_equal(d['partition']['features']['lane:'+c['source_lane_id']]['parts'][0]['raw_vertices'],original_source)


def test_replay_must_compare_actual_saved_model_not_only_its_self_hash(tmp_path,monkeypatch,contacts):
    *_,model=contacts
    inp=tmp_path/'input'; inp.mkdir()
    for f in ('run.json','reconstruction-input.json','source-domain.json'):
        (inp/f).write_text('original test artifact',encoding='utf-8')
    modelpath=tmp_path/'contact-model.json'
    modelpath.write_text(json.dumps(model),encoding='utf-8')
    manifest={'model_file_sha256':job._sha256(modelpath), 'preparation_files_sha256':{
        f:job._sha256(inp/f) for f in ('run.json','reconstruction-input.json','source-domain.json')}}
    (tmp_path/'run.json').write_text(json.dumps(manifest),encoding='utf-8')
    monkeypatch.setattr(job,'build_from_preparation',lambda directory:copy.deepcopy(model))
    assert job.verify_contacts(tmp_path)['fresh_read_match']
    modified=copy.deepcopy(model); modified['role_conflicts']=[]
    modified['content_sha256']=digest({k:v for k,v in modified.items() if k!='content_sha256'})
    modelpath.write_text(json.dumps(modified),encoding='utf-8')
    manifest['model_file_sha256']=job._sha256(modelpath)
    (tmp_path/'run.json').write_text(json.dumps(manifest),encoding='utf-8')
    with pytest.raises(ValueError,match='fresh original inputs'): job.verify_contacts(tmp_path)
