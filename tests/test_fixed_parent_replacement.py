import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from scripts.review_fixed_parent_replacement import require_fixed_parent
from scripts import research_code_revision
from tests.test_junction_edges import network


def test_only_declared_turn_geometry_may_change():
    before = network(end_width=3.)
    after = copy.deepcopy(before)
    after.find("road[@id='100']/planView/geometry").set('length', '33')
    require_fixed_parent(before, after, {'100'})


@pytest.mark.parametrize('fault', ['parent', 'speed', 'topology', 'identity', 'junction', 'empty', 'ordinary'])
def test_same_parent_review_rejects_silent_changes(fault):
    before = network(end_width=3.); after = copy.deepcopy(before)
    selected = {'100'}
    if fault == 'parent':
        after.find("road[@id='10']/planView/geometry").set('x', '2')
    elif fault == 'speed':
        ET.SubElement(after.find("road[@id='100']/lanes/laneSection/right/lane"), 'speed', max='4', unit='m/s')
    elif fault == 'topology':
        after.find("road[@id='100']/link/predecessor").set('elementId', '11')
    elif fault == 'identity':
        after.find("road[@id='100']").set('id', '101')
    elif fault == 'junction':
        ET.SubElement(after, 'junction', id='new', name='not-approved')
    elif fault == 'empty':
        selected = set()
    else:
        selected = {'10'}
    with pytest.raises(ValueError):
        require_fixed_parent(before, after, selected)


def test_research_migration_logs_code_but_never_raw_or_configuration(tmp_path, monkeypatch):
    monkeypatch.setattr(research_code_revision, 'ROOT', tmp_path)
    code = tmp_path/'mapforge'/'ops'/'example.py'
    code.parent.mkdir(parents=True); code.write_text('new implementation')
    hashes, changes = research_code_revision.bind_current_code({str(code): 'old'})
    assert changes == [dict(file=str(code), previous_sha256='old', current_sha256=hashes[str(code)])]
    for name in ('source.shp', 'decision.yaml', 'external.py'):
        source = tmp_path/name; source.write_text('changed')
        with pytest.raises(ValueError, match='immutable'):
            research_code_revision.bind_current_code({str(source): 'old'})
