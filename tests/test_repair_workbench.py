"""Scope tests: candidate transactions are not whole-map acceptance."""
import copy
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from fastapi.testclient import TestClient

from mapforge.repair_web.model import RepairModel, Project, Conflict, parse, digest
from mapforge.repair_web.app import create_app, selection_geometry
from scripts.internal_edge_jets import states

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr'
PACKET = ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
RUN = PACKET.with_name('run.json')


@pytest.fixture(scope='module')
def model():
    return RepairModel(SOURCE.read_bytes())


@pytest.fixture
def project(tmp_path):
    return Project(SOURCE,PACKET,RUN,tmp_path/'project')


def test_zero_edit_byte_identity(model):
    assert model.compile({})[0] == SOURCE.read_bytes()
    assert model.compile({'11:-1':0})[0] == SOURCE.read_bytes()


def test_real_shared_boundary_and_other_roads_unchanged(model):
    data,guard = model.compile({'11:-1':.25})
    r = parse(data); base = model.root
    h = model.handles['11:-1']
    old = next(x for x in base.findall('road') if x.get('id') == '11')
    new = next(x for x in r.findall('road') if x.get('id') == '11')
    assert guard['changed_roads']==['11']
    assert guard['new_geometry_records']==guard['new_width_records']==0
    for s in np.linspace(0,float(old.get('length')),151):
        left = s==float(old.get('length'))
        a,b = states(old,-1,s,left),states(new,-1,s,left)
        aa,bb = states(old,-2,s,left),states(new,-2,s,left)
        assert np.allclose(a[0],b[0],atol=1e-9)  # inner road boundary locked
        assert np.allclose(aa[1],bb[1],atol=1e-9)  # outer edge of pair locked
        assert np.allclose(b[1],bb[0],atol=1e-9)  # shared boundary one identity
        if s < h['knots'][0] or s > h['knots'][-1]:
            assert np.allclose(a,b,atol=1e-9)
    at=h['s']; a=states(old,-1,at,False)[1]; b=states(new,-1,at,False)[1]
    assert math.hypot(a[0]-b[0],a[1]-b[1]) == pytest.approx(.25,abs=1e-9)
    for a,b in zip(base.findall('road'),r.findall('road')):
        if a.get('id')!='11': assert ET.tostring(a)==ET.tostring(b)


def test_end_jets_and_c2_basis_are_locked(model):
    root=parse(model.compile({'11:-1':.3})[0]); road=next(r for r in root.findall('road') if r.get('id')=='11')
    old=next(r for r in model.root.findall('road') if r.get('id')=='11')
    for s in model.handles['11:-1']['knots']:
        # Preserve inherited one-sided jet jumps, not claim old map is smooth.
        for lid in (-1,-2):
            delta=[]
            for left in (True,False):
                delta.append(np.array(states(road,lid,s,left))-np.array(states(old,lid,s,left)))
            assert np.allclose(delta[0],delta[1],atol=2e-7)
    for s in (0.,float(road.get('length'))):
        for lid in (-1,-2):
            assert np.allclose(states(old,lid,s,s>0),states(road,lid,s,s>0),atol=1e-12)


@pytest.mark.parametrize('vals',[{'unknown':.1},{'11:-1':float('nan')},{'11:-1':2.1},{'11:-1':None}])
def test_bad_or_locked_intents_rejected(model,vals):
    with pytest.raises(ValueError): model.compile(vals)


def test_negative_width_rejected_without_mutating_model(model):
    # Copy a real model, reduce the active lane's width for a controlled conflict.
    m=copy.deepcopy(model)
    road=next(r for r in m.root.findall('road') if r.get('id')=='11')
    for w in road.findall('lanes/laneSection/right/lane[@id="-1"]/width'):
        w.set('a','.01');w.set('b','0');w.set('c','0');w.set('d','0')
    original=ET.tostring(m.root)
    with pytest.raises(ValueError,match='负宽'): m.compile({'11:-1':1.})
    assert ET.tostring(m.root)==original


def test_transactions_undo_save_reopen_export(project):
    initial=project.revision
    p=project.preview(initial,'11:-1',.25)
    assert project.values()=={}
    s=project.apply(initial,p['preview_id']); applied_sha=s['sha256']
    with pytest.raises(Conflict):project.apply(initial,p['preview_id'])
    old=project.navigate(project.revision,-1)
    assert old['sha256']==project.model.base_hash
    again=project.navigate(project.revision,1)
    assert again['sha256']==applied_sha
    project.save(project.revision)
    reopened=Project(SOURCE,PACKET,RUN,project.directory)
    assert reopened.snapshot()['sha256']==applied_sha
    exported=reopened.export(reopened.revision)
    assert digest(Path(exported['path']).read_bytes())==applied_sha
    assert exported['report']['status']=='BLOCKED_RESEARCH_CANDIDATE'
    assert exported['report']['esmini']=='PASS'
    assert exported['report']['consumer']['xsd']=='PASS'
    assert exported['report']['consumer']['checked_positions']>100
    assert exported['report']['consumer']['map_accepted'] is False


def test_source_drift_and_stale_preview_rejected(project,tmp_path):
    watched=tmp_path/'source';watched.write_bytes(b'original')
    project.binding[str(watched)]=digest(b'original')
    a=project.preview(project.revision,'11:-1',.1)
    project.preview(project.revision,'11:-1',.2)
    with pytest.raises(Conflict):project.apply(project.revision,a['preview_id'])
    watched.write_bytes(b'changed')
    with pytest.raises(Conflict,match='Source drift'):project.save(project.revision)


def test_loopback_token_origin_and_api_contract(project):
    with TestClient(create_app(project,'test-token',8765),base_url='http://127.0.0.1:8765') as client:
        auth={'Authorization':'Bearer test-token','X-Mapforge-CSRF':'test-token'}
        assert client.get('/api/project').status_code==403
        assert client.get('/api/project',headers={**auth,'Host':'evil.example'}).status_code==403
        assert client.get('/api/project',headers={**auth,'Origin':'https://evil.example'}).status_code==403
        revision=client.get('/api/project',headers=auth).json()['state']['revision']
        payload={'revision':revision,'handle':'11:-1','value':.25}
        assert client.post('/api/action/preview',headers={'Authorization':'Bearer test-token'},json=payload).status_code==403
        p=client.post('/api/action/preview',headers=auth,json=payload)
        assert p.status_code==200
        result=client.post('/api/action/apply',headers=auth,json={'revision':revision,'preview_id':p.json()['preview_id']})
        assert result.status_code==200
        assert client.post('/api/action/save',headers=auth,json={'revision':revision}).status_code==409
        assert client.get('/assets/model.py').status_code==404
        assert client.get('/api/exports/bad/candidate.xodr',headers=auth).status_code==404


def test_selection_is_owned_shared_edge_from_actual_xml(project):
    before = selection_geometry(project)
    assert set(before) == set(project.model.handles)
    data, _ = project.model.compile({'11:-1': .25})
    after = selection_geometry(project, data)
    assert after['11:-1'] != before['11:-1']
    for key in before:
        if key != '11:-1':
            assert after[key] == before[key]
    h = project.model.handles['11:-1']
    road = next(r for r in parse(data).findall('road') if r.get('id') == '11')
    assert np.allclose(after[h['id']]['ends'][0], states(road,h['lane'],h['knots'][0],False)[1][:2])
    assert np.allclose(after[h['id']]['ends'][1], states(road,h['lane'],h['knots'][-1],True)[1][:2])
    assert project.values() == {}  # presentation does not alter the working version


def test_preview_and_apply_selection_match_without_mutating_on_cancel(project):
    with TestClient(create_app(project,'test',8766),base_url='http://127.0.0.1:8766') as client:
        auth={'Authorization':'Bearer test','X-Mapforge-CSRF':'test'}
        initial=client.get('/api/project',headers=auth).json()['state']
        preview=client.post('/api/action/preview',headers=auth,json={
            'revision':initial['revision'],'handle':'11:-1','value':.25}).json()
        current=client.get('/api/project',headers=auth).json()['state']
        assert current['sha256']==initial['sha256']
        assert current['selection']==initial['selection']
        applied=client.post('/api/action/apply',headers=auth,json={
            'revision':initial['revision'],'preview_id':preview['preview_id']}).json()
        assert applied['selection']==preview['selection']
        assert applied['sha256']==preview['sha256']
