"""A copied rejected road is a diagnostic, never a successful conversion."""
import json
import pytest
from lxml import etree
from spikes import physical_graph_candidate as candidate


@pytest.mark.parametrize('mouth_mode',[None,'parallel'])
def test_rejected_candidate_retains_failure_status_and_false_result(tmp_path,monkeypatch,mouth_mode):
    source=tmp_path/'input.xodr';target=tmp_path/'output.xodr'
    root=etree.Element('OpenDRIVE');etree.SubElement(root,'road',id='1',junction='-1')
    etree.ElementTree(root).write(str(source))
    source.with_suffix('.source-lanes.json').write_text('{}',encoding='utf-8')
    before=source.read_bytes()
    monkeypatch.setattr(candidate,'shp_source',lambda:object())
    monkeypatch.setattr(candidate,'_origin',lambda _: (0.,0.))
    monkeypatch.setattr(candidate,'load_policy',lambda _: {'dynamics':{}})
    calls=[]
    def rejected(*a,**kwargs):
        calls.append(kwargs)
        return {'status':'REJECTED','reason':'test conflict'}
    monkeypatch.setattr(candidate.family,'solve_road',rejected)
    monkeypatch.setattr(candidate,'finalize_opendrive_g8',lambda *a:{
        'decision':{'blocked_reasons':[]},'quality':{'gates':{'G11':{'status':'FAIL'}}}})
    kwargs={} if mouth_mode is None else {'mouth_mode':mouth_mode}
    assert candidate.run(source,target,**kwargs) is False
    assert all(c['junction_endpoint_mode']==(mouth_mode or 'preserve') for c in calls)
    assert all(c['source_error_budget']=='absolute' for c in calls)
    assert source.read_bytes()==before
    report=json.loads(target.with_suffix('.fit.json').read_text())
    assert report['unmodified_rejected_roads_are_not_solutions']
    assert report['roads'][0]['status']=='REJECTED'
    assert report['junction_endpoint_mode']==(mouth_mode or 'preserve')
    assert report['measured_connectors_rebuilt'] is False
    assert json.loads(target.with_suffix('.delivery-decision.json').read_text())['status']=='BLOCKED'
