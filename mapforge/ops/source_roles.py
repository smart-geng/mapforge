"""Explicit, source-revision-bound role decisions; never rewrite raw geometry.

An accepted interpretation is NOT a geometry or dynamics acceptance. Movement
observations remain in the problem inventory and require a separate path check.
"""
import copy

from mapforge.ops.reconstruction_scope import digest


def require_body(packet):
    if digest({k: v for k, v in packet.items() if k != 'content_sha256'}) != packet['content_sha256']:
        raise ValueError('packet body digest mismatch')


def rebind_unchanged_source_decision(scope, domain, previous, current, decision):
    """Carry ONLY prior explicit selections across an endpoint-inventory fix.

    Raw scope/domain and every selected conflict must be byte-content identical.
    New conflicts stay unresolved. The historical decision is never modified.
    This is an explicit compiler migration, not a broader source-role policy.
    """
    resolve_source_roles(scope, domain, previous, decision)
    require_body(current)
    for key in ('scope_sha256', 'source_domain_sha256', 'policy', 'source_bindings',
                'full_source_support', 'transition_events'):
        # Raw readers use tuples for NULL-shape parts; JSON packets use lists.
        # Compare the exact canonical packet content, not Python container type.
        if digest(previous[key]) != digest(current[key]):
            raise ValueError('role migration requires unchanged original supports and contacts')
    if current['issues'] or not current['source_support_compiled']:
        raise ValueError('role migration cannot resolve source structure issues')
    inventory=current.get('source_endpoint_inventory',[])
    expected={(sid,role) for sid in scope['observations'] for role in ('start','end')}
    if (current.get('source_endpoint_inventory_policy')!='all-retained-original-lanes-both-ends/v1'
            or len(inventory)!=len(expected)
            or {(r['source_lane_id'],r['contact']) for r in inventory}!=expected):
        raise ValueError('complete new endpoint inventory required for role migration')
    old={(c['source_lane_id'],c['contact']):c for c in previous['role_conflicts']}
    new={(c['source_lane_id'],c['contact']):c for c in current['role_conflicts']}
    if len(new)!=len(current['role_conflicts']) or any(new.get(k)!=v for k,v in old.items()):
        raise ValueError('role migration cannot remove or change reviewed conflict evidence')
    result=copy.deepcopy(decision)
    result['source_contact_sha256']=current['content_sha256']
    result['binding_migration']={
        'kind':'unchanged-originals-endpoint-inventory-fix',
        'previous_source_contact_sha256':previous['content_sha256'],
        'previous_decision_sha256':digest(decision),
        'scope_sha256':scope['content_sha256'], 'source_domain_sha256':domain['content_sha256'],
        'new_source_role_authorizations':0}
    # Validate rather than treating a changed hash as a new authorization.
    resolve_source_roles(scope,domain,current,result)
    return result


def resolve_source_roles(scope, domain, contacts, decision):
    for packet in (scope, domain, contacts):
        require_body(packet)
    if (contacts['scope_sha256'] != scope['content_sha256']
            or contacts['source_domain_sha256'] != domain['content_sha256']
            or domain['scope_sha256'] != scope['content_sha256']):
        raise ValueError('source role input binding mismatch')
    if contacts['issues'] or not contacts['source_support_compiled']:
        raise ValueError('source structural issues cannot be resolved by role decisions')
    if decision['schema'] != 'mapforge.source-role-decision/v1' or decision['version'] != 1:
        raise ValueError('unsupported explicit role decision version')
    if decision['source_contact_sha256'] != contacts['content_sha256']:
        raise ValueError('source role decision requires its exact reviewed source revision')
    authority = decision['authorization']
    original_only = authority.get('basis') == 'no-conflicts-original-observations'
    if original_only:
        if contacts['role_conflicts'] or decision['decisions']:
            raise ValueError('original-only admission cannot resolve any role conflict')
        expected={(sid,cp) for sid in scope['observations'] for cp in ('start','end')}
        inventory=contacts.get('source_endpoint_inventory',[])
        if (contacts.get('source_endpoint_inventory_policy')!='all-retained-original-lanes-both-ends/v1'
                or len(inventory)!=len(expected)
                or {(r['source_lane_id'],r['contact']) for r in inventory}!=expected):
            raise ValueError('original-only admission requires complete endpoint inventory')
    elif authority.get('basis') != 'user-confirmed-in-current-thread' or not authority.get('scope') or not authority.get('date'):
        raise ValueError('explicit user decision record required')
    return _role_result(scope, domain, contacts, decision)


def _role_result(scope, domain, contacts, decision):
    """Build role rows only AFTER the caller has verified their authority."""
    conflicts = {(c['source_lane_id'], c['contact']): c for c in contacts['role_conflicts']}
    if len(conflicts) != len(contacts['role_conflicts']):
        raise ValueError('ambiguous source role conflicts')
    resolved, selected = [], set()
    for entry in decision['decisions']:
        key = (entry['source_lane_id'], entry['contact'])
        if key not in conflicts or key in selected:
            raise ValueError('role decision selects an unknown or duplicate source conflict')
        if (entry['physical_authority'] != 'original_left_right_boundaries'
                or entry['lane_path_role'] != 'movement_path_observation'):
            raise ValueError('role decision cannot discard or silently reclassify other data')
        obs = scope['observations'][key[0]]
        if not obs[key[1]+'_width_known'] or obs[key[1]+'_width_mm'] != 0:
            raise ValueError('explicit zero-width source evidence required')
        selected.add(key)
        resolved.append({'source_lane_id': key[0], 'contact': key[1],
                         'original_conflict': copy.deepcopy(conflicts[key]),
                         'physical_authority': entry['physical_authority'],
                         'lane_path_role': entry['lane_path_role'],
                         'raw_geometry_changed': False, 'source_point_retained': True,
                         'interpretation_state': 'USER_CONFIRMED', 'conversion_state': 'TRANSFORMED'})
    role_map = {}
    selected_ids = {sid for sid, _ in selected}
    for feature, support in contacts['full_source_support'].items():
        path = feature.startswith('lane:')
        separate = path and feature[5:] in selected_ids
        role_map[feature] = {'role': 'movement_path_observation' if separate else
                            'physical_lane_center_observation' if path else 'physical_boundary_observation',
                            'physical_geometry_constraint': not separate,
                            'movement_path_check_required': separate,
                            'parts': copy.deepcopy(support['parts']),
                            'logical_consumers': list(support['logical_consumers'])}
    pending = [copy.deepcopy(c) for key, c in conflicts.items() if key not in selected]
    result = {'schema': 'mapforge.resolved-source-roles/v1', 'status': 'BLOCKED',
              'scope_sha256': scope['content_sha256'], 'source_domain_sha256': domain['content_sha256'],
              'source_contact_sha256': contacts['content_sha256'], 'decision_sha256': digest(decision),
              'decision': copy.deepcopy(decision), 'resolved': resolved, 'unresolved': pending,
              'feature_roles': role_map, 'role_binding_complete': not pending,
              'movement_paths_validated': False, 'geometry_solver_ran': False, 'export_allowed': False,
              'domain_endpoint_policy_resolved': False}
    result['content_sha256'] = digest(result)
    return result


def _complete_endpoint_inventory(scope, contacts):
    expected={(sid,cp) for sid in scope['observations'] for cp in ('start','end')}
    actual=contacts.get('source_endpoint_inventory',[])
    if (contacts.get('source_endpoint_inventory_policy')!='all-retained-original-lanes-both-ends/v1'
            or len(actual)!=len(expected)
            or {(r['source_lane_id'],r['contact']) for r in actual}!=expected):
        raise ValueError('complete target original endpoint inventory required')


def _same_original_role_facts(old_scope, old_domain, old_contacts, scope, domain, contacts, entry):
    """Compare reviewed source facts, not generated chart/cut/ownership output.

    Expanding the evaluation scope changes its packet hash. It does NOT grant
    any other source a role exception. Every selected raw lane, both boundary
    records, side relation, original support, conflict and logical road binding
    must still be identical; changed source/CRS/budget requires a new review.
    """
    sid, cp = entry['source_lane_id'],entry['contact']
    if sid not in scope['observations']:
        raise ValueError('reviewed source is absent from the target; no silent dropped approval')
    if digest(old_scope['observations'][sid])!=digest(scope['observations'][sid]):
        raise ValueError('reviewed original lane/side/speed/width facts changed')
    if digest(old_domain['partition']['projection'])!=digest(domain['partition']['projection']):
        raise ValueError('reviewed source projection changed')
    if digest(old_contacts['policy'])!=digest(contacts['policy']):
        raise ValueError('reviewed source contact/budget policy changed')
    if old_contacts['source_bindings'].get(sid)!=contacts['source_bindings'].get(sid):
        raise ValueError('reviewed source logical road binding changed')
    old={(c['source_lane_id'],c['contact']):c for c in old_contacts['role_conflicts']}
    new={(c['source_lane_id'],c['contact']):c for c in contacts['role_conflicts']}
    if (sid,cp) not in old or (sid,cp) not in new or digest(old[sid,cp])!=digest(new[sid,cp]):
        raise ValueError('reviewed source role conflict evidence changed')
    features=['lane:'+sid]
    for rel in old_scope['observations'][sid]['boundary_relations']:
        bid=rel['boundary_key'];features.append('boundary:'+bid)
        if bid not in scope['boundaries'] or digest(old_scope['boundaries'][bid])!=digest(scope['boundaries'][bid]):
            raise ValueError('reviewed original boundary record changed')
    for key in features:
        a=old_domain['partition']['features'].get(key);b=domain['partition']['features'].get(key)
        if a is None or b is None:
            raise ValueError('reviewed original feature support is missing')
        raw=lambda f:[p['raw_vertices'] for p in f['parts']]
        if digest(raw(a))!=digest(raw(b)):
            raise ValueError('reviewed original full parts/vertices changed')
        old_support=old_contacts['full_source_support'].get(key)
        support=contacts['full_source_support'].get(key)
        if not old_support or not support or digest(old_support['parts'])!=digest(support['parts']):
            raise ValueError('reviewed full original support inventory changed')
    return dict(source_lane_id=sid,contact=cp,unchanged_features=features,
                previous_scope_sha256=old_scope['content_sha256'],
                previous_contact_sha256=old_contacts['content_sha256'],
                raw_geometry_changed=False,new_source_role_authorizations=0)


def compose_reviewed_source_roles(scope, domain, contacts, reviews):
    """Replay a finite union of prior decisions onto identical source facts.

    Research packets are self-contained so a new session can recheck the OLD
    exact revision and decision. This never edits/signs a user decision file.
    Any unselected target conflict stays unresolved and prohibits full fitting.
    """
    reviews=list(reviews)
    for packet in (scope,domain,contacts):require_body(packet)
    if (contacts['scope_sha256']!=scope['content_sha256']
            or contacts['source_domain_sha256']!=domain['content_sha256']
            or domain['scope_sha256']!=scope['content_sha256']):
        raise ValueError('whole source role input binding mismatch')
    if contacts['issues'] or not contacts['source_support_compiled']:
        raise ValueError('whole source contact issues cannot be resolved by role replay')
    _complete_endpoint_inventory(scope,contacts)
    selected=[]; evidence=[]; identities=set()
    for bundle in reviews:
        if set(bundle)!={'scope','domain','contacts','decision'}:
            raise ValueError('complete original review revision and decision required')
        previous=resolve_source_roles(bundle['scope'],bundle['domain'],bundle['contacts'],bundle['decision'])
        if bundle['decision']['authorization']['basis']!='user-confirmed-in-current-thread':
            raise ValueError('only explicit existing user decisions may be replayed')
        if not previous['resolved']:
            raise ValueError('empty review is not role authorization')
        for entry in bundle['decision']['decisions']:
            key=entry['source_lane_id'],entry['contact']
            if key in identities:raise ValueError('overlapping review decisions; supply each selected endpoint once')
            identities.add(key)
            witness=_same_original_role_facts(bundle['scope'],bundle['domain'],bundle['contacts'],
                                               scope,domain,contacts,entry)
            witness['previous_decision_sha256']=digest(bundle['decision'])
            evidence.append(witness);selected.append(copy.deepcopy(entry))
    if not selected:raise ValueError('explicit nonempty prior review set required')
    decision=dict(schema='mapforge.source-role-replay/v1',
                  authorization=dict(basis='replay-of-identical-reviewed-source-facts',new_authorizations=0),
                  source_contact_sha256=contacts['content_sha256'],decisions=selected,
                  review_bundles=copy.deepcopy(list(reviews)))
    result=_role_result(scope,domain,contacts,decision)
    result['schema']='mapforge.resolved-source-roles/v2'
    result['replay_evidence']=evidence
    result['new_source_role_authorizations']=0
    for row in result['resolved']:row['interpretation_state']='USER_CONFIRMED_PRIOR_REVISION_REPLAYED'
    result['content_sha256']=digest({k:v for k,v in result.items() if k!='content_sha256'})
    return result


def replay_source_role_packet(scope, domain, contacts, roles):
    """Revalidate v1 direct decisions or v2 exact-source approval composition."""
    require_body(roles)
    if roles['schema']=='mapforge.resolved-source-roles/v1':
        fresh=resolve_source_roles(scope,domain,contacts,roles['decision'])
    elif roles['schema']=='mapforge.resolved-source-roles/v2':
        fresh=compose_reviewed_source_roles(scope,domain,contacts,roles['decision']['review_bundles'])
    else:raise ValueError('unsupported resolved source role packet')
    if fresh['content_sha256']!=roles['content_sha256']:
        raise ValueError('resolved roles differ from fresh verified original decisions')
    return fresh


def resolve_original_source_roles(scope, domain, contacts):
    """Keep every original physical observation; grant NO role overrides.

    A conflict-free packet needs no invented user-approval record. Any actual
    conflict still requires the existing explicit-decision workflow.
    """
    decision={'schema':'mapforge.source-role-decision/v1','version':1,
              'authorization':{'basis':'no-conflicts-original-observations'},
              'source_contact_sha256':contacts['content_sha256'],'decisions':[]}
    return resolve_source_roles(scope,domain,contacts,decision)
