import copy
import hashlib
import json
import xml.etree.ElementTree as ET
import pytest
from tests.test_junction_edges import network
from scripts.assemble_fixed_parent_trials import run


def packet(directory,root,**fields):
    directory.mkdir();path=directory/'node4.xodr';ET.ElementTree(root).write(path)
    report=dict(artifact=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        source_hashes={},code_sha256={},connectors=[dict(road='100')],**fields)
    (directory/'report.json').write_text(json.dumps(report),encoding='utf-8')
    return report


def setup(tmp_path,mutate=False):
    root=network();parent=packet(tmp_path/'parent',root)
    changed=copy.deepcopy(root);geo=changed.find("road[@id='100']/planView/geometry")
    geo.set('length',str(float(geo.get('length'))+1))
    if mutate:changed.find("road[@id='10']").set('name','changed parent')
    packet(tmp_path/'trial',changed,input_sha256=parent['sha256'])
    return parent


def test_combination_is_never_delivery(tmp_path):
    setup(tmp_path)
    result=run(tmp_path/'parent',[tmp_path/'trial'],tmp_path/'combined')
    assert not result['production_accepted']
    assert result['status']=='REVIEW_NOT_DELIVERY'
    assert result['unchanged_other_roads']


def test_no_different_parent_geometry(tmp_path):
    setup(tmp_path,True)
    with pytest.raises(ValueError,match='fixed parent'):
        run(tmp_path/'parent',[tmp_path/'trial'],tmp_path/'combined')
    assert not (tmp_path/'combined').exists()


def test_no_duplicate_turns(tmp_path):
    setup(tmp_path)
    with pytest.raises(ValueError,match='duplicate'):
        run(tmp_path/'parent',[tmp_path/'trial',tmp_path/'trial'],tmp_path/'combined')


def test_different_baseline_sha_rejected(tmp_path):
    setup(tmp_path)
    path=tmp_path/'trial'/'report.json';trial=json.loads(path.read_text());trial['input_sha256']='another-parent'
    path.write_text(json.dumps(trial))
    with pytest.raises(ValueError,match='another parent'):
        run(tmp_path/'parent',[tmp_path/'trial'],tmp_path/'combined')


def test_exact_refinement_lineage_keeps_same_parent(tmp_path):
    setup(tmp_path)
    seed=json.loads((tmp_path/'trial'/'report.json').read_text())
    root=ET.parse(seed['artifact']).getroot()
    root.find("road[@id='100']/planView/geometry").set('length','18')
    packet(tmp_path/'refined',root,input=seed['artifact'],input_sha256=seed['sha256'])
    result=run(tmp_path/'parent',[tmp_path/'refined'],tmp_path/'combined')
    assert result['combined_trials'][0]['refinement_ancestry']
    assert result['input_sha256']!=seed['sha256']


def test_refinement_cannot_hide_different_parent(tmp_path):
    setup(tmp_path,True)
    seed=json.loads((tmp_path/'trial'/'report.json').read_text())
    root=network()  # Pretend to repair parents in the final artifact.
    packet(tmp_path/'refined',root,input=seed['artifact'],input_sha256=seed['sha256'])
    with pytest.raises(ValueError,match='fixed parent'):
        run(tmp_path/'parent',[tmp_path/'refined'],tmp_path/'combined')
