"""Type proven source contacts without turning movement links into lane links.

This is a constraint graph, not a geometry repair. A zero-width taper keeps
the project's G2 requirement; typing it does NOT authorize a relaxed join.
"""
from collections import defaultdict
import math

from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_roles import require_body


def compile_physical_continuations(contacts, roles):
    for packet in (contacts, roles):
        require_body(packet)
    if roles['decision']['source_contact_sha256'] != contacts['content_sha256']:
        raise ValueError('physical graph/source role revision mismatch')
    if contacts['issues'] or not roles['role_binding_complete']:
        raise ValueError('source contacts/roles must be resolved before typing')
    endpoints = contacts['boundary_endpoints']
    group_of = {key: i for i, g in enumerate(contacts['contact_groups']) for key in g['members']}
    if set(group_of) != set(endpoints):
        raise ValueError('physical contact group inventory incomplete')
    relations = defaultdict(list)
    for event in contacts['transition_events']:
        if event['status'] != 'SOURCE_C0_CONTACT_PROVEN':
            raise ValueError('unproven transition cannot define physical continuity')
        if event['kind'] not in ('ordinary_continuation', 'zero_width_birth', 'zero_width_death'):
            raise ValueError('unknown source transition semantics')
        for pair in event['contacts']:
            a, b = pair['from_endpoint'], pair['to_endpoint']
            if a not in endpoints or b not in endpoints or group_of[a] != group_of[b]:
                raise ValueError('transition pair lacks a proven physical contact')
            if a == b:
                raise ValueError('self endpoint link is not a source continuation')
            if event['kind'] == 'ordinary_continuation' and pair['from_side'] != pair['to_side']:
                raise ValueError('ordinary continuation cannot silently exchange lane sides')
            relations[tuple(sorted((a,b)))].append({
                'kind': event['kind'], 'road': event['road'],
                'from_source': event['from_source'], 'to_source': event['to_source'],
                'from_side': pair['from_side'], 'to_side': pair['to_side'],
                'topology_record': event['topology_record'],
            })
    rows = []
    for (a,b), witnesses in sorted(relations.items()):
        ordinary = any(w['kind'] == 'ordinary_continuation' for w in witnesses)
        rows.append({'endpoints': [a,b], 'source_group_index': group_of[a],
                     'role': 'continuing_lane_edge' if ordinary else 'zero_width_taper_contact',
                     'required_geometric_continuity': 'G2',
                     'requirement_basis': 'project_continuing_lane_smoothness' if ordinary else
                                          'project_smooth_taper_policy_not_ordinary_lane_link',
                     'ordinary_lane_link_supported': ordinary,
                     'witnesses': witnesses})
    # A package-external zero-width lane can have no in-scope TOPO event.
    # Its declared sides + identical original boundary node still prove a
    # physical taper. Do not merge overlapping boundary families or fabricate
    # an ordinary lane predecessor: couple only their endpoint jets.
    external_tapers=[]
    for item in contacts.get('source_endpoint_inventory',[]):
        if not item['width_known'] or item['width_mm']!=0:continue
        if item['status'] not in ('SOURCE_ROLE_DECISION_REQUIRED','ZERO_WIDTH_POINT_ROLES_WITHIN_BUDGET'):
            raise ValueError('unproven zero-width endpoint cannot define physical taper')
        pair=item.get('boundary_endpoints',{})
        if set(pair)!={'left','right'}:raise ValueError('zero endpoint needs both original sides')
        a,b=pair['left'],pair['right']
        if a==b or a not in endpoints or b not in endpoints:
            raise ValueError('zero endpoint must identify two distinct physical boundaries')
        if (not endpoints[a]['original_node_id']
                or endpoints[a]['original_node_id']!=endpoints[b]['original_node_id']
                or endpoints[a]['original_node_id']!=item.get('original_boundary_node_id')):
            raise ValueError('zero endpoint original node identity mismatch')
        distance=math.dist(endpoints[a]['xy'],endpoints[b]['xy'])
        if not math.isfinite(distance) or distance>1e-7:
            raise ValueError('noncoincident zero endpoint requires a movable station model')
        if group_of[a]==group_of[b]:continue  # Already coupled by the proven fork graph.
        row={'endpoints':[a,b], 'source_group_index':None,
             'role':'zero_width_taper_contact','required_geometric_continuity':'G2',
             'requirement_basis':'project_smooth_taper_policy_not_ordinary_lane_link',
             'ordinary_lane_link_supported':False,
             'witnesses':[{'kind':'original_zero_width_endpoint','source_lane_id':item['source_lane_id'],
                           'contact':item['contact'],'original_boundary_node_id':item['original_boundary_node_id'],
                           'source_point_retained':True,'topology_record':None}]}
        rows.append(row);external_tapers.append(row)
    forks = []
    for i, g in enumerate(contacts['contact_groups']):
        if len(g['members']) < 3:
            continue
        edges = [r for r in rows if r['source_group_index'] == i]
        covered = {key for r in edges for key in r['endpoints']}
        if covered != set(g['members']):
            raise ValueError('fork has an untyped physical endpoint')
        forks.append({'source_group_index': i, 'members': g['members'],
                      'continuing_edge_pairs': sum(r['ordinary_lane_link_supported'] for r in edges),
                      'taper_only_pairs': sum(not r['ordinary_lane_link_supported'] for r in edges),
                      'derivative_relaxation_authorized': False})
    result = {'schema': 'mapforge.physical-continuation-graph/v1',
              'source_contact_sha256': contacts['content_sha256'],
              'source_role_sha256': roles['content_sha256'],
              'relations': rows, 'forks': forks,
              'external_zero_width_tapers':external_tapers,
              'source_topology_modified': False, 'geometry_fitted': False,
              'xodr_lane_links_generated': False, 'export_allowed': False}
    result['content_sha256'] = digest(result)
    return result
