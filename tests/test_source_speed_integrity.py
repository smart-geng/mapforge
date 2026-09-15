import xml.etree.ElementTree as ET

import pytest

from scripts.recheck_speed_contract import check_limits


def road(speed='60', source='source-lane'):
    root=ET.fromstring('''<OpenDRIVE><road id="10" junction="-1"><lanes><laneSection s="0">
        <right><lane id="-1" type="driving"/></right></laneSection></lanes></road></OpenDRIVE>''')
    lane=root.find('.//lane')
    if source: ET.SubElement(lane,'userData',code='mapforge.source_lane',value=source)
    if speed is not None: ET.SubElement(lane,'speed',sOffset='0',max=speed,unit='km/h')
    return root


@pytest.mark.parametrize('speed,expected', [('60','PASS'),('12','FAIL'),(None,'FAIL')])
def test_lower_or_missing_written_limit_is_failure(speed,expected):
    assert check_limits(road(speed),{'source-lane':60.})['status']==expected


def test_source_absence_is_not_license_to_invent_speed():
    assert check_limits(road('15',None),{})['status']=='FAIL'
    assert check_limits(road(None,None),{})['status']=='PASS'
    assert check_limits(road(None),{})['status']=='FAIL'


def test_empty_coverage_does_not_pass():
    assert check_limits(ET.Element('OpenDRIVE'),{})['status']=='FAIL'


def test_whole_section_limit_is_preserved_not_only_its_minimum():
    root=road();ET.SubElement(root.find('.//lane'),'speed',sOffset='10',max='15',unit='km/h')
    result=check_limits(root,{'source-lane':60.})
    assert result['status']=='FAIL'
    assert result['failures'][0]['reasons']==['source-limit-not-preserved']
