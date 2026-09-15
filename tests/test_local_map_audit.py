from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from scripts.local_geometry_audit import _map_link_records


def test_reference_audit_uses_manifest_identity_not_filename(monkeypatch):
    node = SimpleNamespace(ref_lat=30., ref_lon=106., links=[
        SimpleNamespace(name='east', points=[[106.,30.],[106.0001,30.]])])
    monkeypatch.setattr('scripts.local_geometry_audit._all_map_nodes', lambda: {(21901,21901):node})
    root = ET.fromstring('''<OpenDRIVE><road id="10" name="east" junction="-1" length="10">
      <planView><geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry></planView>
      </road></OpenDRIVE>''')
    manifest = {'source_contexts':[{'role':'main','region':21901,'node_id':21901}]}
    rows = _map_link_records('node13', root, manifest)
    assert len(rows) == 1 and len(rows[0]['source']) == 2


def test_reference_audit_does_not_turn_missing_source_into_zero_error():
    with pytest.raises(ValueError, match='main MAP identity'):
        _map_link_records('node13', ET.Element('OpenDRIVE'), {})
