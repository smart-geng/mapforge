"""Reviewed seven-road event and shared incoming jets, without inner fits.

Positions/widths stay fixed at the mouth. Edge slopes/curvatures are SINGLE
parent variables consumed by every dependent turn. Other parents stay fixed.
Registering this affine interface is not solving, compiling or accepting a map.
"""
from dataclasses import asdict
import copy
import math

import numpy as np

from mapforge.ops.coupled_source_junction import source_contact_frame
from mapforge.ops.port_dependencies import LanePort, PortDependencies
from mapforge.ops.reconstruction_scope import digest as object_digest
from mapforge.repair_web.model import digest, parse
from mapforge.repair_web.outer_event import OuterEvent, EventScope
from mapforge.repair_web.split_event_admission import START, END, BIRTHS, CONNECTORS, REFERENCE_SHA, checked_payload
from mapforge.repair_web.event_shape_admission import point_to_original
from mapforge.validate.shp_boundary_fidelity import _project
from spikes.connector_cross_section import endpoint_frame

PACKET_SHA = '0e681854d9fa3be890455411249687b0eb862ae30b6381bb678811d1c3f3e8d0'
DOMAIN_SHA = 'b640f0efc0f8430cd0a0486a2997740f4a45351e750827b5d9db54a77a4138f5'
SOURCE_BINDING_SHA = '952ecea00e1e8f0e1126d9ca0782f487ff434810adb7e610da89f8236671d20b'
ROLE_DECISION_SHA = 'b1c7aa7924b0fda596576214bd0c44aef94573840866f0e035c968df53142bb3'


class SharedMouthEvent(OuterEvent):
    """C2 long boundary basis with fixed positions, shared variable end jets."""
    def _equalities(self):
        rows, rhs = [], []
        for edge in self.splines:
            lo = self.births.get(edge, self.start)
            for derivative in range(3):
                scale = 20.**derivative
                row = self.row(edge, lo, derivative)
                value = self.old_power(edge, lo, left=True)[derivative]*math.factorial(derivative)
                if edge in self.births:
                    row -= self.row(edge-1, lo, derivative); value = 0.
                rows.append(row*scale); rhs.append(value*scale)
            # No hidden freezing of old internal slopes/curvature at the mouth.
            rows.append(self.row(edge, self.end)); rhs.append(self.old_power(edge, self.end, left=True)[0])
        return np.asarray(rows), np.asarray(rhs)

    def source_cells(self):
        # Prebirth original fragments are finite endpoint observations in the
        # separate source contract, NOT a fictitious branch on its mother edge.
        for trace in self.traces:
            floor = max(self.start, self.births.get(trace['edge'], self.start))
            for a, b in zip(trace['st'][:-1], trace['st'][1:]):
                lo, hi = max(floor, a[0]), min(self.end, b[0])
                if hi <= lo: continue
                cuts = [lo]+[s for s in self.scope.knots if lo < s < hi]+[hi]
                slope = (b[1]-a[1])/(b[0]-a[0])
                for l, h in zip(cuts, cuts[1:]):
                    yield trace, l, h, np.array([a[1]+(l-a[0])*slope, slope, 0., 0.])

    def solve(self, *args, **kwargs):
        raise ValueError('Contract only: old parent-only solve must not bypass the six-turn event')

    def compile(self, *args, **kwargs):
        raise ValueError('Contract only: require joint actual-XML source/shape/transaction admission')


class CoupledEventContract:
    def __init__(self, reference, packet, domain, source_binding, scope_decision, role_decision):
        if digest(reference) != REFERENCE_SHA or scope_decision['reference_sha256'] != REFERENCE_SHA:
            raise ValueError('Unreviewed event revision')
        for payload in (packet, domain, source_binding): checked_payload(payload)
        if ((packet['content_sha256'], domain['content_sha256'], source_binding['content_sha256'])
                != (PACKET_SHA, DOMAIN_SHA, SOURCE_BINDING_SHA)
                or object_digest(role_decision) != ROLE_DECISION_SHA
                or packet['identity_mode'] != 'source-chain-v1'):
            raise ValueError('Unreviewed source identity or source-role decision')
        if (source_binding['packet_sha256'] != packet['content_sha256']
                or source_binding['domain_sha256'] != domain['content_sha256']
                or source_binding['reference_sha256'] != REFERENCE_SHA
                or source_binding['source_constraint_status'] != 'READY_FOR_SHAPE_DESIGN'):
            raise ValueError('Source contract does not match the event')
        if (scope_decision['parent'] != '11' or scope_decision['parent_edit_interval_m'] != [START, END]
                or scope_decision['connectors'] != list(CONNECTORS)
                or scope_decision['physical_edges'] != list(range(5))
                or scope_decision['source_role_changes'] != 0 or scope_decision['source_speed_changes'] != 0
                or scope_decision['frozen_births'] != [[k, v[0]] for k, v in BIRTHS.items()]
                or scope_decision['outgoing_mouths'] != 'fixed_full_world_jets_to_unchanged_other_parents'
                or scope_decision['incoming_mouth'] != dict(positions_and_cross_section='fixed_to_reference',
                    edge_tangents_and_curvatures='shared_parent_and_all_incident_turns', independent_connector_port_adjustment='forbidden')):
            raise ValueError('Event scope differs from the confirmed seven-road decision')
        layout = scope_decision['parent_layout_proposal']
        if scope_decision['connector_layout_proposal'] != dict(reference_primitives=5, core_spirals=3,
                endpoint_caps=[True, True], minimum_independent_span_m=6.,
                transverse_intervals='reference_primitives_only_no_half_caps',
                internal_continuity='world_edges_and_center_G2_explicit_constraints',
                status='STRUCTURE_ONLY_NOT_SHAPE_FEASIBILITY'):
            raise ValueError('Unreviewed connector layout')
        if not scope_decision['frozen_parent_axis'] or scope_decision['frozen_upstream_interval_m'] != [0., START]:
            raise ValueError('Reference axis/upstream must remain frozen')
        if scope_decision['other_road_changes'] != 'forbidden' or layout['minimum_independent_span_m'] != 6.:
            raise ValueError('Unreviewed scope or short independent layout')
        if (scope_decision['execution'] != dict(optimizer_authorized_by_this_file=False,
                new_xodr_authorized_by_this_file=False, old_S2_R2_restarts='forbidden',
                required_before_trial='full_source_shape_surface_and_actual_written_layout_admission',
                web_changes='forbidden', commit_push='forbidden')):
            raise ValueError('Interface contract cannot authorize solve/write or restart a consumed trial')
        knots = tuple(layout['knots_m'])
        root = parse(reference); self.graph = PortDependencies(root); self.graph.validate_junction_table()
        parent = self.graph.roads['11']; semantic = {float(s.get('s')) for s in parent.findall('lanes/laneSection')} | {END}
        if (knots[0] != START or knots[-1] != END or any(k not in semantic for k in knots)
                or min(np.diff(knots)) < 6. or any(v[0] not in knots for v in BIRTHS.values())):
            raise ValueError('Layout must keep births and use reviewed long semantic intervals')
        closure = self.graph.closure({LanePort('11', 'end', lid) for lid in (-1, -2, -3, -4)})
        if closure['connectors'] != frozenset(CONNECTORS): raise ValueError('Incomplete dependent closure')
        roles = {(d['source_lane_id'], d['contact']) for d in role_decision['decisions']
                 if d['physical_authority'] == 'original_left_right_boundaries'
                 and d['lane_path_role'] == 'movement_path_observation'}
        self.reference = reference; self.packet = copy.deepcopy(packet); self.domain = copy.deepcopy(domain)
        self.source_binding = copy.deepcopy(source_binding); self.decision = copy.deepcopy(scope_decision)
        self.parent = SharedMouthEvent(reference, packet, EventScope('11', knots, tuple((k,v[0]) for k,v in BIRTHS.items())), approved_roles=roles)
        self.fixed_frames = {}; self.port_sources = []
        for cid in CONNECTORS:
            for end, port in enumerate(self.graph.connections[cid]):
                road = self.graph.roads[port.road]
                forward = port.contact == ('end' if end == 0 else 'start')
                frame = endpoint_frame(road, port.lane, port.contact, forward)
                if end == 1: self.fixed_frames[cid] = frame
                self.port_sources.extend(self._source_port(cid, end, port, frame))
        self.jet_matrix = np.array([self.parent.row(e, END, d) for e in range(5) for d in (1,2)])

    def _source_port(self, cid, end, port, frame):
        road = self.graph.roads[port.road]
        section = road.findall('lanes/laneSection')[0 if port.contact == 'start' else -1]
        lane = next(l for l in section.findall('left/lane')+section.findall('right/lane') if int(l.get('id')) == port.lane)
        sid = lane.find("userData[@code='mapforge.source_lane']").get('value')
        occ = [o for o in self.packet['occurrences'] if o['road'] == port.road and o['lane'] == port.lane
               and o['section'] == road.findall('lanes/laneSection').index(section) and o['source_lane_id'] == sid]
        if len(occ) != 1: raise ValueError('Missing original port lane identity')
        forward = port.contact == ('end' if end == 0 else 'start')
        if (occ[0]['travel_direction'] == 'with_s') != forward:
            raise ValueError('Original travel direction is inconsistent with connector port')
        # This admitted IBD profile defines declared-right as the inner edge.
        # Coordinate handedness + explicit travel direction determine the map;
        # proximity is NOT allowed to swap a source side to make a port pass.
        inner_is_left = (port.lane < 0) == forward
        declared = {'left': 'right' if inner_is_left else 'left', 'right': 'left' if inner_is_left else 'right'}
        pr = self.domain['partition']['projection']; rows = []
        for side, original_side in declared.items():
            rels = [r for r in self.packet['observations'][sid]['boundary_relations'] if r['declared_side'] == original_side]
            if len(rels) != 1: raise ValueError('Ambiguous original physical port edge')
            key = 'boundary:'+rels[0]['boundary_key']; feature = self.domain['partition']['features'][key]
            if len(feature['parts']) != 1: raise ValueError('Explicit multipart port support required')
            raw = _project(np.asarray(feature['parts'][0]['raw_vertices']), pr['lat_0'], pr['lon_0'])
            point = [frame['edges'][side][k] for k in ('x','y')]
            check = point_to_original(point, raw)
            rows.append(dict(connector=cid, end='incoming' if end == 0 else 'outgoing', port=asdict(port),
                source_lane_id=sid, field=side, source_feature=key, source_part=0,
                world_point=point, complete_original_part_xy=raw.tolist(), distance_m=check['distance_m'],
                position_budget_m=.35, position_within_budget=check['distance_m'] <= .35+1e-7,
                source_side_mapping=declared, independent_movement_tested=False))
        return rows

    def frames(self, coefficients):
        x = np.asarray(coefficients, float)
        if x.shape != (self.parent.nvar,) or not np.isfinite(x).all(): raise ValueError('Invalid shared parent state')
        if np.max(abs(self.parent.E@x-self.parent.e)) > 1e-7: raise ValueError('Frozen positions/birth/upstream changed')
        frames = {}
        for cid in CONNECTORS:
            port = self.graph.connections[cid][0]; edge = -port.lane
            jets = np.array([self.parent.row(e, END, d)@x for e in (edge-1, edge) for d in range(3)])
            frames[cid] = (source_contact_frame(self.parent.axis, END, jets, True), copy.deepcopy(self.fixed_frames[cid]))
        return frames

    def written_layout(self):
        """Predict semantic coefficient re-expression, never claim it is a write.

        A section cut can create a short XML width record with no additional
        independent shape freedom. Report BOTH lengths; do not exempt it from
        the final written-layout gate by calling it a long spline.
        """
        sections = self.parent.road.findall('lanes/laneSection')
        records = []; previous_short = []
        for i, sec in enumerate(sections):
            lo = float(sec.get('s')); hi = float(sections[i+1].get('s')) if i+1 < len(sections) else END
            if hi <= START: continue
            lo = max(lo, START)
            cuts = [lo]+[s for s in self.parent.scope.knots if lo < s < hi]+[hi]
            for lane in sec.findall('right/lane'):
                old = lane.findall('width')
                for j, width in enumerate(old):
                    a = float(sec.get('s'))+float(width.get('sOffset'))
                    b = float(sec.get('s'))+float(old[j+1].get('sOffset')) if j+1 < len(old) else hi
                    if b > START and b-a < 6.-1e-8:
                        previous_short.append(dict(section=i, lane=int(lane.get('id')), start_m=a, end_m=b, length_m=b-a))
                for a, b in zip(cuts, cuts[1:]):
                    records.append(dict(section=i, lane=int(lane.get('id')), start_m=a, end_m=b,
                        length_m=b-a, boundary_span_start_m=max(s for s in self.parent.scope.knots if s <= a)))
        short = [r for r in records if r['length_m'] < 6.-1e-8]
        old_pairs = {(r['lane'], round(r['start_m'], 7), round(r['end_m'], 7)) for r in previous_short}
        return dict(status='PREDICTED_NOT_COMPILED_OR_ADMITTED', width_records=len(records),
            minimum_width_record_span_m=min(r['length_m'] for r in records),
            short_semantic_records=short, previous_short_records=previous_short,
            short_records_not_identical_to_existing=[r for r in short if (r['lane'],round(r['start_m'],7),round(r['end_m'],7)) not in old_pairs],
            new_semantic_sections=0, new_reference_primitives=0,
            new_independent_sub_6m_spans=0, records=records)

    def describe(self):
        freedom = self.jet_matrix@self.parent.Z
        incident = {str(e): [cid for cid in CONNECTORS if e in (-self.graph.connections[cid][0].lane-1,
                                                                   -self.graph.connections[cid][0].lane)] for e in range(5)}
        return dict(schema='mapforge/coupled-event-contract/v1', status='SHARED_PORT_INTERFACE_NOT_SHAPE_ADMISSION',
            reference_sha256=digest(self.reference), scope_decision_sha256=object_digest(self.decision),
            source_binding_sha256=self.source_binding['content_sha256'], replacement_roads=['11', *CONNECTORS],
            parent_variables=self.parent.nvar, parent_affine_freedom=self.parent.Z.shape[1],
            shared_mouth_jet_rank=int(np.linalg.matrix_rank(freedom)), edge_dependents=incident,
            five_mouth_positions_fixed=True, incoming_jets_shared_not_independently_adjustable=True,
            outgoing_all_world_jets_fixed=True, port_source_positions=self.port_sources,
            parent_knots=list(self.parent.scope.knots), minimum_independent_span_m=float(min(np.diff(self.parent.scope.knots))),
            source_inventory=dict(physical_boundary_parts=len(self.parent.traces), original_movement_paths=len(self.parent.paths),
                                  source_interval_bindings=self.source_binding['binding_counts']),
            predicted_written_layout=self.written_layout(),
            initial_affine_origin_is_map=False, old_xml_exactly_representable_not_claimed=True,
            whole_shape_or_fidelity_feasibility=False, joint_connector_trial_admitted=False,
            next_required='complete six long-turn variable blocks and source/shape/layout inequalities before a single registered trial',
            source_role_changes=0, solver_calls=0, new_xodr=False, web_changed=False, map_accepted=False)

    def unchanged_xml(self):
        return self.reference
