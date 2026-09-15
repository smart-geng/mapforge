"""Explicit lane-port incidence for dependency-closed geometry transactions.

This does not infer missing traffic topology. A physical shared boundary
affects BOTH adjacent lanes, regardless of which turn originally failed.
"""
from dataclasses import dataclass
import copy
import hashlib
import math


def revision(root):
    """Snapshot content, independent of the stdlib/lxml serializer chosen."""
    def tree(e):
        return (str(e.tag), sorted(e.attrib.items()), e.text, e.tail, [tree(c) for c in e])
    return hashlib.sha256(repr(tree(root)).encode('utf-8')).hexdigest()


@dataclass(frozen=True, order=True)
class LanePort:
    road: str
    contact: str
    lane: int


class PortDependencies:
    def __init__(self, root):
        self.base_revision = revision(root)
        root = copy.deepcopy(root)
        self.junctions = root.findall('junction')
        self.roads = {r.get('id'): r for r in root.findall('road')}
        if len(self.roads) != len(root.findall('road')):
            raise ValueError('duplicate road identity')
        self.connections = {}
        self.incident = {}
        for road in self.roads.values():
            if road.get('junction', '-1') == '-1' or road.get('name') == 'junction_paving':
                continue
            sections = road.findall('lanes/laneSection')
            lanes = road.findall('lanes/laneSection/right/lane')
            if len(sections) != 1 or len(lanes) != 1 or sections[0].findall('left/lane'):
                raise ValueError('dependency prototype requires one-lane connecting roads')
            ports = []
            for role in ('predecessor', 'successor'):
                link, ll = road.find('link/'+role), lanes[0].find('link/'+role)
                if link is None or ll is None or link.get('elementType') != 'road':
                    raise ValueError('missing explicit road/lane contact')
                parent = self.roads.get(link.get('elementId'))
                cp = link.get('contactPoint')
                if parent is None or cp not in ('start', 'end'):
                    raise ValueError('unresolved explicit parent contact')
                if parent.get('junction','-1') != '-1' or not parent.findall('lanes/laneSection'):
                    raise ValueError('prototype requires an ordinary parent with lane sections')
                sec = parent.findall('lanes/laneSection')[0 if cp == 'start' else -1]
                lid = int(ll.get('id'))
                if not any(int(v.get('id')) == lid for v in sec.findall('left/lane')+sec.findall('right/lane')):
                    raise ValueError('unresolved explicit parent lane')
                key = LanePort(parent.get('id'), cp, lid)
                ports.append(key)
                self.incident.setdefault(key, set()).add(road.get('id'))
            self.connections[road.get('id')] = tuple(ports)

    def validate_junction_table(self):
        """Require the junction table and road/lane contact graph to agree."""
        expected = set()
        for cid, ports in self.connections.items():
            lane = self.roads[cid].find('lanes/laneSection/right/lane')
            expected.add((self.roads[cid].get('junction'), cid, ports[0].road,
                          ports[0].lane, int(lane.get('id')), 'start'))
        actual = []
        jids = [j.get('id') for j in self.junctions]
        if any(v is None for v in jids) or len(set(jids)) != len(jids):
            raise ValueError('duplicate or missing junction identity')
        for junction in self.junctions:
            cids = [c.get('id') for c in junction.findall('connection')]
            if any(v is None for v in cids) or len(set(cids)) != len(cids):
                raise ValueError('duplicate or missing junction connection identity')
            for c in junction.findall('connection'):
                if not c.findall('laneLink'):
                    raise ValueError('junction connection has no laneLink')

                for link in c.findall('laneLink'):
                    actual.append((junction.get('id'), c.get('connectingRoad'), c.get('incomingRoad'),
                                   int(link.get('from')), int(link.get('to')), c.get('contactPoint')))
        if len(actual) != len(set(actual)) or set(actual) != expected:
            raise ValueError('junction table disagrees with explicit road/lane contacts')
        return True

    @staticmethod
    def semantic_signature(road):
        """Allowed edits are planar geometry, not speed, IDs, source or lane links.

        Section station and road length may change with the geometry model.
        Source support propagation still requires independent validation.
        """
        excluded = {'planView', 'laneOffset', 'width', 'border'}
        def tree(e):
            attrs = dict(e.attrib)
            if e.tag == 'road':
                attrs.pop('length', None)
                attrs.pop('name', None)
            if e.tag == 'laneSection':
                attrs.pop('s', None)
            return (e.tag, sorted(attrs.items()), (e.text or '').strip(),
                    [tree(c) for c in e if c.tag not in excluded])
        return tree(road)

    def require_fixed_ports_unchanged(self, candidate, mutable_ports):
        """Recompute undeclared center/edge end states; caller PASS cannot bypass it."""
        from mapforge.validate.junction_edges import endpoint_edges
        from mapforge.validate.smoothness import lane_endpoint_state
        mutable_ports = frozenset(mutable_ports)
        for rid in {p.road for p in mutable_ports}:
            old, new = self.roads[rid], candidate.roads[rid]
            for cp in ('start', 'end'):
                before = old.findall('lanes/laneSection')[0 if cp == 'start' else -1]
                after = new.findall('lanes/laneSection')[0 if cp == 'start' else -1]
                old_ids = {int(l.get('id')) for l in before.findall('left/lane') + before.findall('right/lane')}
                new_ids = {int(l.get('id')) for l in after.findall('left/lane') + after.findall('right/lane')}
                if old_ids != new_ids:
                    raise ValueError('geometry transaction cannot change endpoint lane identities')
                for lid in old_ids:
                    if LanePort(rid, cp, lid) in mutable_ports:
                        continue
                    aa = endpoint_edges(old, lid, cp, True)
                    bb = endpoint_edges(new, lid, cp, True)
                    aa['center'] = lane_endpoint_state(old, lid, cp, forward=True)
                    bb['center'] = lane_endpoint_state(new, lid, cp, forward=True)
                    for side in aa:
                        for key in ('x', 'y', 'heading', 'curvature'):
                            a, b = aa[side][key], bb[side][key]
                            delta = math.atan2(math.sin(b-a), math.cos(b-a)) if key == 'heading' else b-a
                            if not math.isfinite(delta) or abs(delta) > 1e-8:
                                raise ValueError(f'undeclared port changed: {rid}/{cp}/{lid}/{side}/{key}')
        return True
    def boundary_ports(self, road_id, contact, side, boundary_index):
        """Index 0=laneOffset, index i=outer edge of i-th sorted lane.

        The center shared edge touches the first lane of BOTH sides. A
        missing/born lane is not invented to complete a convenient graph.
        """
        if side not in ('right', 'left') or contact not in ('start', 'end'):
            raise ValueError('invalid physical boundary address')
        road = self.roads[road_id]
        sec = road.findall('lanes/laneSection')[0 if contact == 'start' else -1]
        ids = sorted((int(l.get('id')) for l in sec.findall(side+'/lane')), key=abs)
        if not 0 <= boundary_index <= len(ids) or not ids:
            raise ValueError('missing physical boundary')
        touched = ids[max(0, boundary_index-1):min(len(ids), boundary_index+1)]
        if boundary_index == 0:
            other = 'left' if side == 'right' else 'right'
            opposite = sorted((int(l.get('id')) for l in sec.findall(other+'/lane')), key=abs)
            touched += opposite[:1]
        return frozenset(LanePort(road_id, contact, lid) for lid in touched)

    def closure(self, mutable_ports):
        changed = frozenset(mutable_ports)
        if any(p.road not in self.roads for p in changed):
            raise ValueError('unknown mutable parent')
        for p in changed:
            sections = self.roads[p.road].findall('lanes/laneSection')
            if p.contact not in ('start','end') or not sections:
                raise ValueError('invalid mutable port')
            sec = sections[0 if p.contact=='start' else -1]
            if p.lane not in {int(v.get('id')) for v in sec.findall('left/lane')+sec.findall('right/lane')}:
                raise ValueError('unknown mutable lane')
        connectors = frozenset(c for p in changed for c in self.incident.get(p, ()))
        # Fixed opposite ends are dependencies, NOT newly movable ports.
        read_ports = frozenset(p for c in connectors for p in self.connections[c])
        return {'mutable_ports': changed, 'fixed_ports': read_ports-changed,
                'connectors': connectors}

    def require_complete(self, mutable_ports, rebuilt):
        needed = self.closure(mutable_ports)['connectors']
        if needed != frozenset(rebuilt):
            raise ValueError('incomplete dependency transaction: missing='+
                             ','.join(sorted(needed-frozenset(rebuilt)))+
                             '; unexpected='+','.join(sorted(frozenset(rebuilt)-needed)))

    def commit(self, original, replacements, mutable_ports, validation):
        """Atomic in-memory acceptance: failed/unvalidated proposals never leak.

        Validation is supplied by the independent caller for every changed
        parent and connector. This local transaction is not delivery approval.
        """
        if revision(original) != self.base_revision:
            raise ValueError('stale base revision; rebuild the dependency model')
        mutable_ports = frozenset(mutable_ports)
        rebuilt = set(replacements) & self.connections.keys()
        self.require_complete(mutable_ports, rebuilt)
        expected = {p.road for p in mutable_ports} | set(rebuilt)
        if set(replacements) != expected or any(r.get('id') != k for k, r in replacements.items()):
            raise ValueError('replacement identity/scope mismatch')
        if any(self.semantic_signature(self.roads[k]) != self.semantic_signature(r)
               for k, r in replacements.items()):
            raise ValueError('geometry transaction cannot change traffic contacts, source identities or speeds')
        if set(validation) != expected or any(v != 'PASS' for v in validation.values()):
            return copy.deepcopy(original), False
        out = copy.deepcopy(original)
        for old in list(out.findall('road')):
            if old.get('id') in replacements:
                index = list(out).index(old)
                out.remove(old)
                out.insert(index, copy.deepcopy(replacements[old.get('id')]))
        candidate_graph = PortDependencies(out)
        if candidate_graph.connections != self.connections:
            raise ValueError('geometry transaction cannot silently change traffic contacts')
        self.validate_junction_table()
        candidate_graph.validate_junction_table()
        self.require_fixed_ports_unchanged(candidate_graph, mutable_ports)
        return out, True
