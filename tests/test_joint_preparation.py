import json
import xml.etree.ElementTree as ET

import pytest

from scripts import prepare_joint_reconstruction as job
from tests.test_reconstruction_scope import fixture
from mapforge.ops.reconstruction_scope import digest
from tests.test_source_domain import domain_fixture


def prepared(tmp_path, monkeypatch):
    root, src, _ = fixture()
    profile = tmp_path/'profile.yaml'
    profile.write_text('test only', encoding='utf-8')
    boundary = tmp_path/'B.shp'
    boundary.write_bytes(b'synthetic fixture, not an actual shapefile')
    src.dir = tmp_path
    src.p = {'_path': str(profile), 'crs': {'verified': False}}
    monkeypatch.setattr(job, 'ProfileSource', lambda *a: src)
    source = tmp_path/'input.xodr'
    ET.ElementTree(root).write(source, encoding='utf-8')
    output = tmp_path/'prepared'
    result = job.prepare(source, output)
    return output, source, boundary, result


def test_prepare_reopen_reads_sources_and_never_claims_geometry_pass(tmp_path, monkeypatch):
    output, source, _, result = prepared(tmp_path, monkeypatch)
    before = source.read_bytes()
    check = job.verify_preparation(output)
    assert check['fresh_read_match'] and check['status'] == 'BLOCKED'
    assert not check['geometry_solver_ran'] and not check['export_allowed']
    assert not list(output.glob('*.xodr'))
    with pytest.raises(FileExistsError):
        job.prepare(source, output)
    assert source.read_bytes() == before


@pytest.mark.parametrize('target', ['source', 'packet'])
def test_changed_source_or_packet_is_rejected_on_reopen(tmp_path, monkeypatch, target):
    output, _, boundary, _ = prepared(tmp_path, monkeypatch)
    path = boundary if target == 'source' else output/'reconstruction-input.json'
    path.write_bytes(path.read_bytes() + b'\nchanged')
    with pytest.raises(ValueError, match='changed|hash mismatch'):
        job.verify_preparation(output)


@pytest.mark.parametrize('rehash_content', [False, True])
def test_forged_self_hash_cannot_hide_omitted_observation(tmp_path, monkeypatch, rehash_content):
    output, _, _, _ = prepared(tmp_path, monkeypatch)
    path = output/'reconstruction-input.json'
    packet = json.loads(path.read_text(encoding='utf-8'))
    packet['observations'].pop(next(iter(packet['observations'])))
    if rehash_content:
        packet['content_sha256'] = digest({k: v for k, v in packet.items() if k != 'content_sha256'})
    path.write_text(json.dumps(packet), encoding='utf-8')
    run_path = output/'run.json'
    run = json.loads(run_path.read_text(encoding='utf-8'))
    run['packet_file_sha256'] = job._sha256(path)
    run_path.write_text(json.dumps(run), encoding='utf-8')
    with pytest.raises(ValueError, match='fresh original inputs|digest mismatch'):
        job.verify_preparation(output)


@pytest.mark.parametrize('tamper', [False, True, 'drop_run_marker', 'drop_packet_binding'])
def test_source_domain_file_replays_originals_and_rejects_rehashed_missing_movement(tmp_path, monkeypatch, tamper):
    root, src, _ = domain_fixture()
    header = ET.SubElement(root, 'header')
    ET.SubElement(header, 'geoReference').text = '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m'
    profile = tmp_path/'profile.yaml'; profile.write_text('fixture', encoding='utf-8')
    src.dir = tmp_path; src.p['_path'] = str(profile)
    monkeypatch.setattr(job, 'ProfileSource', lambda *a: src)
    xodr = tmp_path/'input.xodr'; ET.ElementTree(root).write(xodr, encoding='utf-8')
    output = tmp_path/'domain'
    job.prepare(xodr, output, source_junction='original-j', xodr_junction=root.find('junction').get('id'))
    if not tamper:
        r = job.verify_preparation(output)
        assert r['source_domain']['fresh_read_match'] and r['source_domain']['movement_status'] == 'MATCH'
        assert r['source_domain']['partition_status'] == 'UNRESOLVED'
        assert r['status'] == 'BLOCKED' and not list(output.glob('*.xodr'))
    elif tamper is True:
        path = output/'source-domain.json'
        packet = json.loads(path.read_text(encoding='utf-8'))
        packet['inventory']['paths'].pop()
        packet['content_sha256'] = digest({k: v for k, v in packet.items() if k != 'content_sha256'})
        path.write_text(json.dumps(packet), encoding='utf-8')
        run_path = output/'run.json'; run = json.loads(run_path.read_text(encoding='utf-8'))
        run['source_domain']['file_sha256'] = job._sha256(path)
        run_path.write_text(json.dumps(run), encoding='utf-8')
        with pytest.raises(ValueError, match='fresh original inputs'): job.verify_preparation(output)
    else:
        run_path = output/'run.json'; run = json.loads(run_path.read_text(encoding='utf-8'))
        if tamper == 'drop_run_marker': del run['source_domain']
        else:
            path = output/'reconstruction-input.json'
            packet = json.loads(path.read_text(encoding='utf-8'))
            del packet['source_domain_binding']
            packet['schema'] = 'mapforge.reconstruction-input/v1'
            packet['content_sha256'] = digest({k: v for k, v in packet.items() if k != 'content_sha256'})
            path.write_text(json.dumps(packet), encoding='utf-8')
            run['packet_file_sha256'] = job._sha256(path)
        run_path.write_text(json.dumps(run), encoding='utf-8')
        with pytest.raises(ValueError, match='source domain required'): job.verify_preparation(output)
