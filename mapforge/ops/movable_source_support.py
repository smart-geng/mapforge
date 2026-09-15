"""Repartition immutable original movement support at CURRENT source cuts.

No source point is moved. Clipped ordinary tails remain in the complete
parent source model; via observations remain entire. Union coverage still
requires final written-map verification, not this preparation calculation.
"""
import copy
import numpy as np
from mapforge.ops.reconstruction_scope import _source_id
from mapforge.validate.shp_boundary_fidelity import _project
from spikes.measured_connector_caps import unique_source_path


def working_side_order(parents, ports, chains, scope):
    """Bind declared source fields to travel-left/right via shared BOUNDARY IDs.

    Profile SIDE labels are preserved source attributes, not necessarily the
    travel-side ordering of the current XODR port. Never infer a new neighbour
    or topology. Both endpoints must prove the SAME identity permutation.
    """
    orders=[]
    for end,(port,ids) in enumerate(zip(ports,chains)):
        parent=parents[port.road];owner=parent.original.owner
        keys=parent.port_features[port]
        forward=port.contact==('end' if end==0 else 'start')
        expected=tuple(owner[k] for k in (keys if forward else keys[::-1]))
        matches=set()
        for sid in ids:
            relations=scope['observations'][sid]['boundary_relations']
            byside={r['declared_side']:owner['boundary:'+r['boundary_key']] for r in relations}
            if set(byside)!={'left','right'}:raise ValueError('complete declared source boundary pair required')
            if (byside['left'],byside['right'])==expected:matches.add(('left','right'))
            if (byside['right'],byside['left'])==expected:matches.add(('right','left'))
        if len(matches)!=1:
            raise ValueError('source sides cannot bind uniquely to both physical port boundary identities')
        orders.append(matches.pop())
    if len(orders)!=2 or orders[0]!=orders[1]:
        raise ValueError('source side permutation disagrees between incoming/outgoing boundary identities')
    return orders[0]


def clip_original_plane(points, axis, station, keep_travel_after):
    """Split an original monotone polyline at an exact Line/Arc normal plane."""
    xy = np.asarray(points, float)
    st = axis.project(xy)[:,0]
    delta = np.diff(st)
    if np.all(delta >= -1e-8) and st[-1]-st[0] > 1e-7:
        direction = 1
    elif np.all(delta <= 1e-8) and st[0]-st[-1] > 1e-7:
        direction = -1
    else:
        raise ValueError('original movement tail is nonmonotone in its parent chart')
    if station < min(st)-1e-7 or station > max(st)+1e-7:
        raise ValueError('current source cut lies outside the complete original movement tail')
    u = direction*st; cut = direction*station
    i = min(max(np.searchsorted(u, cut, side='right')-1, 0), len(xy)-2)
    point, tangent, _ = axis.frame(station)
    denom = float((xy[i+1]-xy[i]) @ tangent)
    if abs(denom) < 1e-10:
        raise ValueError('original segment does not have a unique normal-plane crossing')
    fraction = float((point-xy[i]) @ tangent / denom)
    if not -1e-7 <= fraction <= 1+1e-7:
        raise ValueError('source-plane intersection escaped the original segment')
    cut_xy = xy[i]+np.clip(fraction, 0., 1.)*(xy[i+1]-xy[i])
    result = np.vstack([cut_xy, xy[i+1:]]) if keep_travel_after else np.vstack([xy[:i+1], cut_xy])
    return result


def original_cut_heading(points, axis, station, keep_travel_after):
    """One-sided original tangent, including a ZERO-length clipped tail.

    Never estimate this heading from the subtraction of two nearly identical
    clipped points. It comes from a complete original segment, even when the
    current cut coincides with the source endpoint. No geometric point is added.
    """
    xy=np.asarray(points,float);ss=axis.project(xy)[:,0];delta=np.diff(xy,axis=0)
    nonzero=np.linalg.norm(delta,axis=1)>1e-8
    lo=np.minimum(ss[:-1],ss[1:]);hi=np.maximum(ss[:-1],ss[1:])
    ids=np.flatnonzero(nonzero & (lo-1e-8<=station) & (station<=hi+1e-8))
    if not len(ids):raise ValueError('source cut has no original tangent support')
    i=int(ids[-1] if keep_travel_after else ids[0])
    return float(np.arctan2(delta[i,1],delta[i,0]))


class MovableMovementSupport:
    def __init__(self, graph, cid, parents, source, scope, domain):
        self.cid = cid; self.ports = graph.connections[cid]
        self.parent_chains = [parents[p.road].source_port_domain.bindings[p] for p in self.ports]
        lane = graph.roads[cid].find('lanes/laneSection/right/lane')
        self.via_id = _source_id(lane)
        a, b = self.parent_chains
        middle = unique_source_path(source.topo_out, a[-1], self.via_id) + unique_source_path(source.topo_out, self.via_id, b[0])[1:]
        if len(middle) < 3:
            raise ValueError('full original via is not between the registered source parents')
        self.middle_ids = tuple(middle[1:-1])
        self.source_ids = tuple(a)+self.middle_ids+tuple(b)
        if len(set(self.source_ids)) != len(self.source_ids):
            raise ValueError('cyclic or duplicated original movement support')
        if any(s not in scope['observations'] for s in self.source_ids):
            raise ValueError('movement exceeds admitted original source inventory')
        pr = domain['partition']['projection']; features = domain['partition']['features']
        self.original = {}; self.orientation = {}
        def xy(key):
            parts = features[key]['parts']
            if len(parts) != 1:
                raise ValueError('explicit multipart movement support is required')
            return _project(np.asarray(parts[0]['raw_vertices']), pr['lat_0'], pr['lon_0'])
        for sid in self.source_ids:
            center = xy('lane:'+sid); fields = {'center':center}; orientations = {}
            chord = center[-1]-center[0]
            for rel in scope['observations'][sid]['boundary_relations']:
                side = rel['declared_side']; curve = xy('boundary:'+rel['boundary_key'])
                if side not in ('left','right') or side in fields:
                    raise ValueError('explicit unique original movement sides required')
                dot = float((curve[-1]-curve[0]) @ chord)
                if abs(dot) < 1e-9:
                    raise ValueError('ambiguous original side orientation')
                fields[side] = curve.copy() if dot > 0 else curve[::-1].copy()
                orientations[side] = dict(boundary_key=rel['boundary_key'], travel_reversed=dot<0)
            if set(fields) != {'center','left','right'}:
                raise ValueError('missing original side relation; no inferred boundary')
            for v in fields.values(): v.setflags(write=False)
            self.original[sid] = fields; self.orientation[sid] = orientations
        # Match the SAME physical features used by the parent constraints.
        # Do not equate a DBF/Profile label to the target's travel-left side:
        # doing so can make a 3.5m lane's left target fit its right source.
        order=working_side_order(parents,self.ports,self.parent_chains,scope)
        self.declared_original=self.original
        self.source_side_mapping=dict(zip(('left','right'),order))
        self.original={sid:dict(center=fields['center'],left=fields[order[0]],right=fields[order[1]])
                       for sid,fields in self.declared_original.items()}
        self.before = self._join(a); self.middle = self._join(self.middle_ids); self.after = self._join(b)
        # The original joins are verified, never closed by a synthetic bridge.
        self._join(self.source_ids)
        self.separate_movement_ids = [sid for sid in self.source_ids if
            parents[self.ports[0].road].original.roles['feature_roles']['lane:'+sid]['movement_path_check_required']]

    def _join(self, ids):
        result = {}
        for side in ('left','right','center'):
            parts = [self.original[s][side] for s in ids]
            if not parts or any(np.linalg.norm(a[-1]-b[0]) > 1e-5 for a,b in zip(parts[:-1],parts[1:])):
                raise ValueError('complete original movement chain gap; no artificial join')
            result[side] = np.vstack(parts)
            result[side].setflags(write=False)
        return result

    def evaluate(self, parent_states):
        a,b = self.ports
        incoming,outgoing = parent_states[a.road],parent_states[b.road]
        raw = {}; tails = []; physical_center_parts = [self.middle['center']]; endpoint_headings={}
        movement_observations = []
        for side in ('left','right','center'):
            before = clip_original_plane(self.before[side], incoming['model'].axis, incoming['stations'][a], True)
            after = clip_original_plane(self.after[side], outgoing['model'].axis, outgoing['stations'][b], False)
            raw[side] = np.vstack([before,self.middle[side],after])
            endpoint_headings[side]=(
                original_cut_heading(self.before[side],incoming['model'].axis,incoming['stations'][a],True),
                original_cut_heading(self.after[side],outgoing['model'].axis,outgoing['stations'][b],False))
        for role, ids, curves in (('predecessor',self.parent_chains[0],self.before),
                                  ('successor',self.parent_chains[1],self.after)):
            tails.append(dict(role=role, source_lane_id=ids[-1] if role=='predecessor' else ids[0],
                              source_ids=list(ids), curves=curves))
        for state, port, ids, after in ((incoming,a,self.parent_chains[0],True),
                                       (outgoing,b,self.parent_chains[1],False)):
            axis=state['model'].axis; cut=state['stations'][port]
            for sid in ids:
                points=self.original[sid]['center']
                if sid in self.separate_movement_ids:
                    # Retain the WHOLE original path, not only its current
                    # connector tail. It is not a physical-center constraint.
                    movement_observations.append(dict(source_lane_id=sid,points=points,
                        role='movement_path_observation',validation_required=True))
                    continue
                ss=axis.project(points)[:,0];direction=1 if ss[-1]>ss[0] else -1
                lo,hi=direction*ss[[0,-1]];target=direction*cut
                if (after and target>=hi-1e-8) or (not after and target<=lo+1e-8):continue
                entire=(after and target<=lo+1e-8) or (not after and target>=hi-1e-8)
                physical_center_parts.append(points if entire else clip_original_plane(points,axis,cut,after))
        return dict(raw=raw, via=self.middle, tails=tails,
            source_ids=list(self.source_ids), entire_via_preserved=True,
            travel_side_to_declared_side=self.source_side_mapping,
            side_binding_policy='same-source-boundary-identities-as-both-parent-ports/v1',
            original_endpoint_headings=endpoint_headings,
            separate_movement_ids=list(self.separate_movement_ids),
            physical_center_parts=physical_center_parts, movement_path_observations=movement_observations,
            center_role_policy='original-physical-fragments-and-separate-approved-paths/v1',
            source_vertices_removed=0, aggregate_written_coverage_accepted=False)
