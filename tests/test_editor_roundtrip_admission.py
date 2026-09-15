import json
from pathlib import Path
import pytest
from scripts.check_editor_roundtrip import compare, load, speed_at
from lxml import etree as ET


XML = '''<OpenDRIVE><header revMajor="1" revMinor="5"><geoReference>test-local</geoReference></header>
<road id="10" length="20" junction="-1"><planView>
<geometry s="0" x="0" y="0" hdg="0" length="20"><line/></geometry></planView>
<lanes><laneSection s="0"><center><lane id="0" type="none"/></center><right>
<lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
<speed sOffset="0" max="60" unit="km/h"/></lane></right></laneSection></lanes></road></OpenDRIVE>'''


def trial(tmp_path, changed):
    src, dst, report = (tmp_path/n for n in ('source.xodr','target.xodr','report.json'))
    src.write_text(XML)
    dst.write_text(changed)
    compare(src,dst,report)
    return json.loads(report.read_text())


def test_identical_geometry_in_namespace(tmp_path):
    r=trial(tmp_path,XML.replace('<OpenDRIVE>', '<OpenDRIVE xmlns="urn:test">'))
    assert r['status']=='SAMPLED_NO_EDIT_CHECK_ONLY'
    assert r['rows'][0]['edge_max_m']==0
    assert not r['map_accepted']


def test_speed_rounding_not_deleted_limit(tmp_path):
    r=trial(tmp_path,XML.replace('max="60" unit="km/h"','max="16.6667" unit="m/s"'))
    assert r['rows'][0]['speed_state_mismatches']==0
    assert r['rows'][0]['speed_rounding_samples']>0
    assert r['rows'][0]['speed_rounding_max_ms']==pytest.approx(1/30000)


def test_missing_speed_rejected(tmp_path):
    r=trial(tmp_path,XML.replace('<speed sOffset="0" max="60" unit="km/h"/>',''))
    assert r['status']=='REJECTED_FOR_LOSSLESS_EDITING'
    assert r['rows'][0]['speed_state_mismatches']>0


def test_edge_drift_rejected(tmp_path):
    r=trial(tmp_path,XML.replace('a="3.5"','a="6.5"'))
    assert r['rows'][0]['edge_max_m']==pytest.approx(3.)
    assert r['status']=='REJECTED_FOR_LOSSLESS_EDITING'


def test_source_crs_drift_cannot_be_aligned_away(tmp_path):
    with pytest.raises(ValueError,match='Different coordinate frames'):
        trial(tmp_path,XML.replace('test-local','some-other-frame'))


def test_speed_event_one_sided():
    lane=ET.fromstring('<lane><speed sOffset="0" max="10"/><speed sOffset="5" max="20"/></lane>')
    assert speed_at(lane,5.,True)==10.
    assert speed_at(lane,5.,False)==20.
