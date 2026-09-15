import copy
import math
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_contacts import compile_source_contacts, contact_groups
from mapforge.ops.port_dependencies import revision


def seal(value):
    value['content_sha256'] = digest({k: v for k, v in value.items() if k != 'content_sha256'})
    return value


def source_fixture():
    """Parent, through lane, zero-width left birth; arbitrary field aliases."""
    root = ET.fromstring('<OpenDRIVE><road id="10" length="20" junction="-1">'
        '<planView><geometry s="0" x="0" y="0" hdg="0" length="20"><line/></geometry></planView>'
        '<lanes><laneSection s="0"/><laneSection s="10"/></lanes></road></OpenDRIVE>')
    def coord(points): return (np.asarray(points)/6378137.*180./math.pi).tolist()
    lines = {'a': [[0, 0], [10, 0]], 'b': [[10, 0], [20, 0]], 'c': [[10, 0], [20, 3]]}
    bnd = {'al': ([[0,2],[10,2]], 'bl0', 'bl10'), 'ar': ([[0,-2],[10,-2]], 'br0', 'br10'),
           'bl': ([[10,2],[20,2]], 'bl10', 'bl20'), 'br': ([[10,-2],[20,-2]], 'br10', 'br20'),
           'cl': ([[10,2],[20,4]], 'bl10', 'cl20')}
    relations = {'a': [('al','left'),('ar','right')], 'b': [('bl','left'),('br','right')],
                 'c': [('cl','left'),('bl','right')]}
    bndrows, features, observations = {}, {}, {}
    def part(points):
        xy=np.asarray(points); length=float(np.linalg.norm(xy[-1]-xy[0]))
        return {'raw_vertices': coord(points), 'source_vertex_s_m': [0., length], 'length_m': length,
                'atoms': [{'status': 'ASSIGNED', 'source_s_m': [0., length], 'consumers': ['old']} ]}
    for key, (points, start, end) in bnd.items():
        row={'layer':'B', 'record_index':len(bndrows), 'attributes':{'B':key,'SN':start,'EN':end},
             'parts':[coord(points)]}
        bndrows['B:'+key] = {'records':[row]}
        features['boundary:B:'+key] = {'kind':'physical_boundary', 'parts':[part(points)],
            'source_lane_ids':[sid for sid, rs in relations.items() if any(k==key for k,_ in rs)]}
    for sid, points in lines.items():
        rr={'layer':'L', 'record_index':len(observations), 'attributes':{
            'I':sid,'R':'original-road','SN':'n0' if sid=='a' else 'n10','EN':'n10' if sid=='a' else 'n20'+sid},
            'parts':[coord(points)], 'geometry_origin':'field'}
        observations[sid]={'raw_records':[rr], 'start_width_known':True, 'end_width_known':True,
            'start_width_mm':0 if sid=='c' else 4000, 'end_width_mm':2000 if sid=='c' else 4000,
            'boundary_relations':[{'boundary_key':'B:'+k,'declared_side':side} for k,side in relations[sid]]}
        features['lane:'+sid]={'kind':'lane_path','parts':[part(points)], 'source_lane_ids':[sid]}
    source=SimpleNamespace(L={'lane':{'file':'L','fields':{'id':'I','road':'R','start_node':'SN','end_node':'EN'}},
        'boundary':{'file':'B','fields':{'id':'B','start_node':'SN','end_node':'EN'}},
        'topo':{'file':'T','fields':{'from':'F','to':'T'}}})
    source.rows=[{'layer':'T','record_index':i,'attributes':{'F':'a','T':s}} for i,s in enumerate(('b','c'))]
    source.layer_raw_records=lambda name: copy.deepcopy(source.rows)
    source.lane=lambda sid: SimpleNamespace(link_pid='original-road',lane_pid=sid,geometry=np.array(coord(lines[sid])))
    source.lanes_of=lambda link: [source.lane(sid) for sid in lines]
    scope=seal({'base_revision':revision(root), 'observations':observations, 'boundaries':bndrows, 'mutable_roads':['10'], 'connectors':[],
        'occurrences':[{'scope_role':'ordinary','road':'10','section':0 if s=='a' else 1,'source_lane_id':s} for s in lines],
        'source_links':[{'kind':'ordinary_section','road':'10','section':0,'from_source':'a','to_source':'b'}]})
    domain=seal({'scope_sha256':scope['content_sha256'], 'comparison':{'status':'MATCH','actual':[]},
        'partition':{'projection':{'lat_0':0.,'lon_0':0.},'features':features}})
    return root, source, scope, domain


def reseal(scope, domain):
    seal(scope); domain['scope_sha256']=scope['content_sha256']; seal(domain)


def test_zero_width_birth_contacts_do_not_force_path_tip_to_boundary_midpoint():
    root, source, scope, domain = source_fixture()
    before = copy.deepcopy((scope,domain)); result=compile_source_contacts(root,source,scope,domain)
    assert result['source_support_compiled'] and not result['issues']
    assert len(result['transition_events']) == 2 and sum(len(t['contacts']) for t in result['transition_events']) == 4
    birth = next(t for t in result['transition_events'] if t['kind']=='zero_width_birth')
    assert {(e['from_side'],e['to_side']) for e in birth['contacts']}=={('left','left'),('left','right')}
    assert result['role_conflicts'][0]['gap_m']==pytest.approx(2.)
    proof=result['role_conflicts'][0]['conditional_infeasibility']
    assert proof['necessary_common_tube_radius_m']==pytest.approx(1.)
    assert not result['export_allowed'] and not result['geometry_solver_ran']
    assert (scope,domain)==before
    assert all(p['vertex_indices']==[0,1] for f in result['full_source_support'].values() for p in f['parts'])


@pytest.mark.parametrize('reverse', [False, True])
def test_unpaired_external_zero_end_is_qualified_without_inventing_topology(reverse):
    root, source, scope, domain = source_fixture()
    source.rows = [r for r in source.rows if r['attributes']['T'] != 'c']
    if reverse:
        o = scope['observations']['c']; row = o['raw_records'][0]
        row['parts'][0].reverse()
        a = row['attributes']; a['SN'], a['EN'] = a['EN'], a['SN']
        o['start_width_mm'], o['end_width_mm'] = o['end_width_mm'], o['start_width_mm']
        domain['partition']['features']['lane:c']['parts'][0]['raw_vertices'].reverse()
        reseal(scope, domain)
    before = copy.deepcopy((scope, domain, source.rows))
    result = compile_source_contacts(root, source, scope, domain)
    inventory = result['source_endpoint_inventory']
    assert len(inventory) == 2 * len(scope['observations'])
    assert len(result['transition_events']) == 1 and not result['issues']
    conflict = result['role_conflicts'][0]
    assert conflict['source_lane_id'] == 'c'
    assert conflict['contact'] == ('end' if reverse else 'start')
    assert conflict['gap_m'] == pytest.approx(2.)
    row = next(i for i in inventory if i['source_lane_id']=='c' and i['width_mm']==0)
    assert not row['has_compiled_transition'] and row['status']=='SOURCE_ROLE_DECISION_REQUIRED'
    assert (scope, domain, source.rows) == before
    assert not result['geometry_solver_ran'] and not result['export_allowed']


def test_external_zero_needs_source_node_identity_not_just_coincident_coordinates():
    root, source, scope, domain = source_fixture()
    source.rows = [r for r in source.rows if r['attributes']['T'] != 'c']
    scope['boundaries']['B:cl']['records'][0]['attributes']['SN'] = 'other'
    reseal(scope, domain)
    result = compile_source_contacts(root, source, scope, domain)
    assert any(i['code']=='SOURCE_ZERO_WIDTH_ENDPOINT_UNRESOLVED' for i in result['issues'])
    assert not result['source_support_compiled'] and not result['role_conflicts']


def test_unknown_external_width_is_not_promoted_to_a_zero_width_exception():
    root, source, scope, domain = source_fixture()
    source.rows = [r for r in source.rows if r['attributes']['T'] != 'c']
    scope['observations']['c']['start_width_known'] = False
    reseal(scope, domain)
    result = compile_source_contacts(root, source, scope, domain)
    row = next(i for i in result['source_endpoint_inventory'] if i['source_lane_id']=='c' and i['contact']=='start')
    assert row['status']=='WIDTH_UNKNOWN_NOT_ZERO' and not result['role_conflicts']


def test_reversed_boundary_preserves_original_endpoint_identity():
    root, source, scope, domain=source_fixture()
    for key in ('bl','cl'):
        row=scope['boundaries']['B:'+key]['records'][0]
        row['parts'][0].reverse(); a=row['attributes']; a['SN'],a['EN']=a['EN'],a['SN']
        domain['partition']['features']['boundary:B:'+key]['parts'][0]['raw_vertices'].reverse()
    reseal(scope,domain); r=compile_source_contacts(root,source,scope,domain)
    assert not r['issues'] and len(r['transition_events'])==2
    assert any(role['boundary_reversed_from_lane'] for p in r['boundary_endpoints'].values() for role in p['source_roles'])


def test_reversed_lane_digitization_uses_original_E_endpoint_without_changing_TOPO():
    root,source,scope,domain=source_fixture()
    row=scope['observations']['b']['raw_records'][0]
    row['parts'][0].reverse(); a=row['attributes']; a['SN'],a['EN']=a['EN'],a['SN']
    domain['partition']['features']['lane:b']['parts'][0]['raw_vertices'].reverse()
    reseal(scope,domain); r=compile_source_contacts(root,source,scope,domain)
    assert not r['issues']
    event=next(e for e in r['transition_events'] if e['to_source']=='b')
    assert event['from_source_contact']=='end' and event['to_source_contact']=='end'
    assert event['topology_record']['attributes']=={'F':'a','T':'b'}


@pytest.mark.parametrize('fault',['lane_node','boundary_node','missing_mapping','zero_not_known','missing_sibling','multipart'])
def test_same_coordinates_or_favorable_counts_cannot_hide_source_faults(fault):
    root, source, scope, domain=source_fixture()
    if fault=='lane_node': scope['observations']['c']['raw_records'][0]['attributes']['SN']='wrong'
    elif fault=='boundary_node': scope['boundaries']['B:cl']['records'][0]['attributes']['SN']='different-node'
    elif fault=='missing_mapping': del source.L['boundary']['fields']['start_node']
    elif fault=='zero_not_known': scope['observations']['c']['start_width_known']=False
    elif fault=='missing_sibling': source.lanes_of=lambda link:[SimpleNamespace(lane_pid='absent')]
    elif fault=='multipart':
        f=domain['partition']['features']['boundary:B:cl']; f['parts'].append(copy.deepcopy(f['parts'][0]))
        scope['boundaries']['B:cl']['records'][0]['parts'].append(copy.deepcopy(scope['boundaries']['B:cl']['records'][0]['parts'][0]))
    reseal(scope,domain); r=compile_source_contacts(root,source,scope,domain)
    assert r['issues'] and not r['source_support_compiled'] and not r['export_allowed']


def test_contact_group_rejects_identity_and_transitive_distance_conflicts():
    nodes={str(i):{'xy':[x,0],'original_node_id':'one'} for i,x in enumerate((0,.015,.03))}
    with pytest.raises(ValueError,match='transitive'): contact_groups(nodes,[('0','1'),('1','2')],.02)
    nodes['1']['original_node_id']='other'
    with pytest.raises(ValueError,match='identities'): contact_groups(nodes,[('0','1')],.02)


def test_research_tolerances_cannot_be_silently_relaxed():
    r,s,p,d=source_fixture()
    with pytest.raises(ValueError): compile_source_contacts(r,s,p,d,contact_tolerance_m=.1)
    with pytest.raises(ValueError): compile_source_contacts(r,s,p,d,source_error_budget_m=2.)


def test_digest_is_not_just_a_display_string():
    r,s,p,d=source_fixture(); p['observations'].pop('c')
    with pytest.raises(ValueError,match='content mismatch'): compile_source_contacts(r,s,p,d)


def test_stale_baseline_is_rejected_before_computing_event_stations():
    r,s,p,d=source_fixture(); r.find('road').set('length','21')
    with pytest.raises(ValueError,match='stale'): compile_source_contacts(r,s,p,d)


@pytest.mark.parametrize('fault',['feature','part','vertex','owner','arclength','upstream'])
def test_rehashed_partial_inventory_cannot_claim_complete_source_support(fault):
    r,s,p,d=source_fixture();features=d['partition']['features']
    if fault=='feature':features.pop('lane:b')
    elif fault=='part':features['lane:b']['parts']=[]
    elif fault=='vertex':features['lane:b']['parts'][0]['raw_vertices'].pop()
    elif fault=='owner':features['lane:b']['source_lane_ids']=['c']
    elif fault=='arclength':features['lane:b']['parts'][0]['length_m']=9.
    else:d['partition']['issues']=[{'code':'SOURCE_INCOMPLETE'}]
    reseal(p,d)
    with pytest.raises(ValueError,match='inventory|mismatch|upstream'):
        compile_source_contacts(r,s,p,d)
