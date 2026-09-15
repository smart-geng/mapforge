"""One variable state for source-native parents and every incident connector.

Research kernel only. Fixed long models, unchanged original-source inequalities,
analytic parent contact maps and block-local numerical connector Jacobians.
No inner parent/connector fit is silently frozen as an independent optimum.
"""
import numpy as np
from scipy.linalg import null_space
from scipy.sparse import csr_matrix
from mapforge.ops.port_dependencies import LanePort, PortDependencies
from mapforge.validate.smoothness import _edge_world_curvature
from spikes.connector_cross_section import endpoint_frame


def source_contact_frame(axis, station, jets, forward):
    """World jets from the CURRENT chart, not a cached exported pose."""
    xy, tangent, normal = axis.frame(station)
    h = float(np.arctan2(tangent[1], tangent[0])); k = axis.curvature
    high, low = np.asarray(jets, float).reshape(2, 3)
    direction = 1 if forward else -1

    def edge(values):
        t, d1, d2 = values
        A = 1-k*t
        if A <= .1:
            raise ValueError('nonregular shared parent contact')
        p = xy+t*normal
        return dict(x=float(p[0]), y=float(p[1]),
            heading=h+float(np.arctan2(d1, A))+(0 if forward else np.pi),
            curvature=direction*_edge_world_curvature(t, d1, d2, k, 0.))

    center = (high+low)/2; tc = center[0]; a = 1-k*tc
    if a <= .1:
        raise ValueError('nonregular shared parent center')
    p = xy+tc*normal
    return dict(pose=(float(p[0]), float(p[1]), h+(0 if forward else np.pi)),
        k=direction*k/a, dk=0.,
        edges=dict(left=edge(high if forward else low), right=edge(low if forward else high)),
        center=edge(center))


class MovableSourceParentState:
    """Continuous chart, shared cubic knots, cut stations and coefficients.

    Unlike SourceParentState, E/C and the source interval witnesses change
    with the chart: no fixed null-space or CZ is exported to the old QP.
    This is an evaluator, not a nonlinear solver or an acceptance gate.
    """
    def __init__(self, original, model, coefficients, compiled=None, *, source_root=None):
        import copy
        from mapforge.ops.source_knot_layout import SourceKnotLayout
        from mapforge.ops.reconstruction_scope import digest
        if (original.road != model.road or original.owner != model.owner
                or digest(original.roles) != digest(model.roles)
                or original.source_tol != model.source_tol):
            raise ValueError('mutable layout must use the same original source roles and budget')
        if model.degree != 3 or not np.isfinite(model.min_span) or model.min_span < 5.5:
            raise ValueError('source layout requires cubic widths and unchanged >=5.5m research span floor')
        self.original = copy.deepcopy(original)
        self.original.min_span = model.min_span
        self.layout = SourceKnotLayout.capture(model)
        self.road = model.road
        self.coefficient_count = model.nvar
        self.knot_count = len(self.layout.free_nodes)
        self.coefficient_slice = slice(6+self.knot_count, 6+self.knot_count+model.nvar)
        from mapforge.ops.source_cubic_export import source_chains, transverse_domain
        if (compiled is None) == (source_root is None):
            raise ValueError('choose compiled component OR explicit whole-source port binding')
        self.source_port_domain = None
        if source_root is not None:
            from mapforge.ops.source_port_domain import SourcePortDomain
            self.source_port_domain = SourcePortDomain(model, source_root)
            self.nominal_stations = self.source_port_domain.evaluate(model, [0.,0.])['cuts']
            offsets = np.zeros(2)
        else:
            self.nominal_stations = np.array(transverse_domain(model,source_chains(model)))
            offsets = np.array([compiled['chart_start_m'], compiled['chart_end_m']])-self.nominal_stations
        self.initial = np.r_[model.origin_delta, model.heading_delta, model.reference_curvature,
                             offsets, np.zeros(self.knot_count), coefficients]
        self.nvar = len(self.initial)
        if np.shape(coefficients) != (model.nvar,) or not np.isfinite(self.initial).all():
            raise ValueError('complete finite mutable source state required')
        self.port_features = {}
        if self.source_port_domain is not None:
            self.port_features = dict(self.source_port_domain.features)
            self.evaluate(self.initial)
            return
        last = max(row['section'] for row in compiled['lane_ledger'])
        for cp, section, station in (('start', 0, compiled['chart_start_m']),
                                      ('end', last, compiled['chart_end_m'])):
            for row in compiled['lane_ledger']:
                if row['section'] != section:
                    continue
                keys = [row['left'], row['right']]
                # Coordinate-side ordering fixed at registration, not sorted
                # after a bad trial has crossed its physical boundaries.
                keys.sort(key=lambda k: model.expression(k, station) @ coefficients, reverse=True)
                port = LanePort(model.road, cp, row['lane'])
                if port in self.port_features:
                    raise ValueError('duplicate mutable source contact')
                self.port_features[port] = tuple(keys)
        self.evaluate(self.initial)  # Shape/layout sanity, not source PASS.

    def evaluate(self, vector):
        from spikes.arc_source_boundary import ArcSourceBoundaryBlock
        v = np.asarray(vector, float)
        if v.shape != (self.nvar,) or not np.isfinite(v).all():
            raise ValueError('finite whole mutable parent vector required')
        model = ArcSourceBoundaryBlock(self.original, heading_delta=v[2], curvature=v[3],
            origin_delta=v[:2], fixed_layout=self.layout, knot_displacements=v[6:self.coefficient_slice.start])
        from mapforge.ops.source_cubic_export import (constrain_written_endpoints,
                                                       source_chains, transverse_domain)
        # Cuts follow the newly projected ORIGINAL support plus free offsets.
        # Keeping their old absolute s silently fixes the mouth and makes
        # infinitesimal axis derivatives fail at an active source cap.
        domain = None
        if self.source_port_domain is not None:
            domain = self.source_port_domain.evaluate(model, v[4:6])
            cuts = domain['cuts']
            # ALL original features stay in model.C/E. Junction tails are
            # explicit dependencies, not collapsed onto a fictitious common
            # exterior endpoint. A final domain compiler must still certify
            # full ownership and emitted minimum width spans before writing.
        else:
            cuts = np.asarray(transverse_domain(model,source_chains(model)))+v[4:6]
            if cuts[1]-cuts[0] < self.layout.minimum_span:
                raise ValueError('parent transverse domain reversed or sub-budget')
            internal=model.global_breaks[(model.global_breaks>cuts[0]+1e-7)&(model.global_breaks<cuts[1]-1e-7)]
            if np.min(np.diff(np.r_[cuts[0],internal,cuts[1]])) < self.layout.minimum_span-1e-7:
                raise ValueError('moving cut creates a short width interval; no split or merge fallback')
            constrain_written_endpoints(model, transverse_stations=cuts)
        if model.nvar != self.coefficient_count:
            raise ValueError('continuous evaluation changed the discrete dimension')
        return self._coefficient_state(model, domain, cuts, v, constraints_rebuilt=True)

    def _coefficient_state(self, model, domain, cuts, v, *, constraints_rebuilt):
        x = v[self.coefficient_slice].copy()
        jets = {}; stations = {}
        for port, keys in self.port_features.items():
            station = float(cuts[0 if port.contact == 'start' else 1])
            # The source-supported family must own the new cut; no extension
            # of a measured boundary, point deletion, or new source identity.
            jets[port] = np.array([model.expression(k, station, j) @ x for k in keys for j in range(3)])
            if jets[port][0] < jets[port][3]-1e-8:
                raise ValueError('physical source edges reversed at a shared port')
            stations[port] = station
        return dict(model=model, coefficients=x, jets=jets, stations=stations,
                    source_inequality_slack=model.C@x-model.lower,
                    source_equalities=model.E@x, constraints_rebuilt=constraints_rebuilt,
                    source_role_overrides_added=0, source_domain=domain,
                    coordinate_vector=v.copy(), cuts=np.asarray(cuts).copy(),
                    geometry_solver_ran=False, export_allowed=False)

    def evaluate_from_snapshot(self, vector, snapshot):
        """Reuse source matrices ONLY if every non-coefficient coordinate agrees.

        Chart, cuts or knots changing always rebuild the ORIGINAL constraints.
        Coefficient stencils still recompute all jets and C/E products. This
        is not the old fixed-C/CZ optimization and does not mutate the snapshot.
        """
        v = np.asarray(vector, float)
        if v.shape != (self.nvar,) or not np.isfinite(v).all():
            raise ValueError('finite whole mutable parent vector required')
        old = snapshot['coordinate_vector']
        if not np.array_equal(v[:self.coefficient_slice.start], old[:self.coefficient_slice.start]):
            return self.evaluate(v)
        if snapshot['model'].road != self.road:
            raise ValueError('coefficient snapshot belongs to another parent')
        return self._coefficient_state(snapshot['model'], snapshot['source_domain'],
            snapshot['cuts'], v, constraints_rebuilt=False)

    @staticmethod
    def frame(state, port, forward):
        return source_contact_frame(state['model'].axis, state['stations'][port], state['jets'][port], forward)


class WholeSourcePortState:
    """All parent variables feed every turn through ONE evaluation snapshot.

    Contains no connector fit. Incomplete parent/turn sets are rejected, and
    the result cannot be submitted as a solved/connected map. Cross-parent
    ownership of one physical source boundary needs a common world model;
    it must not become two independently adjustable copies here.
    """
    def __init__(self, root, junction_id, parents, connectors):
        from mapforge.ops.reconstruction_scope import model_scope_report
        self.report = model_scope_report(root, junction_id, list(parents), list(connectors))
        if self.report['status'] != 'SCOPE_COMPLETE_NOT_MODEL_FEASIBILITY':
            raise ValueError('complete whole-junction parents and turns required')
        self.graph = PortDependencies(root)
        self.parents = dict(parents); self.connectors = tuple(connectors)
        self.slices = {}; initial = []; owners = {}
        for rid in sorted(parents):
            parent = parents[rid]
            if parent.road != rid:
                raise ValueError('source parent identity mismatch')
            for family in parent.layout.features:
                for feature in family:
                    if feature in owners and owners[feature] != rid:
                        raise ValueError('shared boundary crosses parent charts; common world binding required')
                    owners[feature] = rid
            self.slices[rid] = slice(len(initial), len(initial)+parent.nvar)
            initial.extend(parent.initial)
        self.initial = np.asarray(initial)
        for cid in self.connectors:
            for port in self.graph.connections[cid]:
                if port not in self.parents[port.road].port_features:
                    raise ValueError('unbound original source port in complete junction')

    def evaluate(self, vector):
        v = np.asarray(vector, float)
        if v.shape != self.initial.shape or not np.isfinite(v).all():
            raise ValueError('complete finite whole-junction source state required')
        # Build locally, return only after ALL parents succeed. No parent or
        # caller vector is mutated if a later block rejects the trial.
        states = {rid: p.evaluate(v[self.slices[rid]]) for rid, p in self.parents.items()}
        frames = {cid: tuple(self.parents[port.road].frame(states[port.road], port,
                              port.contact == ('end' if end == 0 else 'start'))
                        for end, port in enumerate(self.graph.connections[cid])) for cid in self.connectors}
        return dict(parents=states, connector_frames=frames,
                    status='SOURCE_PORT_EVALUATION_NOT_CONNECTED_GEOMETRY',
                    geometry_solver_ran=False, export_allowed=False)


class WholeSourceGeometryState:
    """All movable parents and all long turn kernels in a single vector.

    Source constraints/partition and turn frames are recomputed at that same
    state. No fixed-parent solve, cached source C, or partial-success output.
    This evaluator is NOT yet a domain/dynamics-complete optimization driver.
    """
    def __init__(self, ports, kernels, source_support):
        if set(kernels) != set(ports.connectors) or set(source_support) != set(kernels):
            raise ValueError('all actual turn shapes and moving original support required')
        self.ports = ports; self.kernels = dict(kernels); self.source_support = dict(source_support)
        self.parent_size = len(ports.initial); self.turn_slices = {}; initial = list(ports.initial)
        for cid in sorted(kernels, key=int):
            kernel = kernels[cid]
            if not all(kernel['caps']):
                raise ValueError('movable parent slopes require both registered long endpoint caps')
            self.turn_slices[cid] = slice(len(initial), len(initial)+len(kernel['initial']))
            initial.extend(kernel['initial'])
        self.initial = np.asarray(initial, float)

    def evaluate(self, vector):
        vector = np.asarray(vector, float)
        if vector.shape != self.initial.shape or not np.isfinite(vector).all():
            raise ValueError('complete finite source-and-turn vector required')
        snapshot = self.ports.evaluate(vector[:self.parent_size])
        equalities = [p['source_equalities'] for p in snapshot['parents'].values()]
        inequalities = [p['source_inequality_slack'] for p in snapshot['parents'].values()]
        turns = {}; objectives = []
        for cid, sl in self.turn_slices.items():
            row = self.evaluate_turn(cid, vector[sl], snapshot['parents'])
            result = row['result']; eq = row['equalities']
            equalities.append(eq); inequalities.append(result[5]); objectives.append(float(result[11]))
            turns[cid] = row
        return dict(status='WHOLE_SOURCE_AND_SHAPE_EVALUATED_NOT_ACCEPTED', parents=snapshot['parents'],
            turns=turns, equalities=np.concatenate(equalities), inequalities=np.concatenate(inequalities),
            source_objective=sum(objectives), simultaneous_parent_connector_variables=True,
            dynamic_source_partition=True, optimizer_ran=False, export_allowed=False,
            final_domain_dynamics_and_surface_constraints_complete=False)

    def evaluate_turn(self, cid, local_vector, parents):
        """One dependent block using CURRENT parents, including source tails."""
        ps = self.ports.graph.connections[cid]
        frames = tuple(self.ports.parents[p.road].frame(parents[p.road], p,
            p.contact == ('end' if end == 0 else 'start')) for end, p in enumerate(ps))
        kernel = self.kernels[cid]
        support = self.source_support[cid].evaluate(parents)
        full = kernel['unpack'](local_vector, frames)
        result = kernel['evaluate'](full, frames, support)
        eq = np.r_[result[4][12:], center_endpoint_residual(result, frames)]
        return dict(result=result, source_support=support, frames=frames, equalities=eq,
            maximum_scaled_equality=float(np.max(abs(eq), initial=0.)),
            minimum_inequality_slack=float(min(result[5])),
            reference_primitives=len(result[0]), minimum_reference_span_m=min(c.length for c in result[0]))


class SourceParentState:
    def __init__(self, model, coefficients, compiled):
        self.model = model
        self.x0 = np.array(coefficients, float, copy=True)
        if self.x0.shape != (model.nvar,) or not np.isfinite(self.x0).all():
            raise ValueError('finite complete parent source state required')
        if np.max(abs(model.E @ self.x0), initial=0.) > 1e-7:
            raise ValueError('parent source equality state is not valid')
        self.Z = null_space(model.E) if len(model.E) else np.eye(model.nvar)
        self.CZ = model.C @ self.Z
        self.slack0 = model.C @ self.x0 - model.lower
        if min(self.slack0) < -1e-7:
            raise ValueError('parent source inequalities fail at initialization')
        self.nvar = self.Z.shape[1]
        self.maps = {}
        last = max(r['section'] for r in compiled['lane_ledger'])
        for cp, index, station in [('start', 0, compiled['chart_start_m']),
                                    ('end', last, compiled['chart_end_m'])]:
            for row in compiled['lane_ledger']:
                if row['section'] != index:
                    continue
                keys = [row['left'], row['right']]
                # Freeze source family order, never re-sort moving lane states.
                mid = sum(row['chart_s']) / 2
                keys.sort(key=lambda k: model.expression(k, mid) @ self.x0, reverse=True)
                P = np.array([model.expression(k, station, d) for k in keys for d in range(3)])
                port = LanePort(model.road, cp, row['lane'])
                if port in self.maps:
                    raise ValueError('duplicate source endpoint identity')
                self.maps[port] = (float(station), P, P @ self.Z)

    def coefficients(self, z):
        z = np.asarray(z, float)
        if z.shape != (self.nvar,) or not np.isfinite(z).all():
            raise ValueError('invalid shared parent coordinates')
        return self.x0 + self.Z @ z

    def jets(self, port, z):
        _, P, PZ = self.maps[port]
        return P @ self.x0 + PZ @ z

    def contact_slope_can_vary(self, port):
        # Test the full permitted linear source space, not a flat initial value.
        return bool(np.any(abs(self.maps[port][2][[1,4]])>1e-12))

    def frame(self, port, jets, forward):
        station = self.maps[port][0]
        return source_contact_frame(self.model.axis, station, jets, forward)


def center_endpoint_residual(result, frames):
    cls, knots, co = result[:3]
    out = []
    for endpoint, frame in enumerate(frames):
        i = 0 if endpoint == 0 else -1
        c = cls[i]
        u = 0. if endpoint == 0 else knots[-1]-knots[-2]
        values = (co['left'][i]+co['right'][i])/2
        t = np.polynomial.polynomial.polyval(u, values)
        d1 = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(values))
        d2 = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(values, 2))
        k, h = (c.KappaStart, c.ThetaStart) if endpoint == 0 else (c.KappaEnd, c.ThetaEnd)
        target = frame['center']
        dh = h+np.arctan2(d1, 1-k*t)-target['heading']
        out.extend([20*np.arctan2(np.sin(dh), np.cos(dh)),
            100*(_edge_world_curvature(t, d1, d2, k, c.dk)-target['curvature'])])
    return np.asarray(out)


class CoupledSourceJunction:
    def __init__(self, graph, parents, kernels):
        self.graph, self.parents, self.kernels = graph, dict(parents), dict(kernels)
        required = {rid for rid, ports in graph.connections.items() if any(p.road in parents for p in ports)}
        if not required or required != set(kernels):
            raise ValueError('every dependent turn must share the same optimization state')
        self.parent_slices, self.turn_slices = {}, {}
        n = 0; initial = []; bounds = []
        for rid, parent in sorted(self.parents.items()):
            self.parent_slices[rid] = slice(n, n+parent.nvar); n += parent.nvar
            initial.extend(np.zeros(parent.nvar))
            bounds.extend([(-.5, .5)]*parent.nvar)  # Declared local search, not a global feasibility claim.
        for rid, kernel in sorted(self.kernels.items(), key=lambda kv: int(kv[0])):
            self.turn_slices[rid] = slice(n, n+len(kernel['initial'])); n += len(kernel['initial'])
            initial.extend(kernel['initial']); bounds.extend(kernel['bounds'])
        self.initial = np.asarray(initial)
        self.bounds = np.asarray(bounds)
        self.fixed, self.forward = {}, {}
        for rid in kernels:
            for end, port in enumerate(graph.connections[rid]):
                fwd = port.contact == ('end' if end == 0 else 'start')
                self.forward[rid, end] = fwd
                if port.road not in parents:
                    self.fixed[rid, end] = endpoint_frame(graph.roads[port.road], port.lane, port.contact, fwd)
                elif port not in parents[port.road].maps:
                    raise ValueError('dependent port has no source coefficient map')
        self.evaluations = 0

    def contact_jets(self, vector, rid):
        return {end: self.parents[p.road].jets(p, vector[self.parent_slices[p.road]])
            for end, p in enumerate(self.graph.connections[rid]) if p.road in self.parents}

    def fixed_source_obstructions(self):
        """An exact endpoint cannot also lie outside its required source tube.

        This is a conditional obstruction to keeping THAT parent immutable,
        not a general impossibility result or permission to alter source roles.
        """
        from spikes.measured_connector_caps import distances
        conflicts = []
        for rid, kernel in self.kernels.items():
            for end, port in enumerate(self.graph.connections[rid]):
                if port.road in self.parents:
                    continue
                role = 'predecessor' if end == 0 else 'successor'
                tails = [t for t in kernel['source_tails'] if t['role'] == role]
                if len(tails) != 1:
                    raise ValueError('original fixed-port source support required before joint solve')
                tail = tails[0]
                for side in ('left', 'right'):
                    edge = self.fixed[rid, end]['edges'][side]
                    distance = float(distances([[edge['x'], edge['y']]], tail['curves'][side])[0])
                    if distance > .35+1e-7:
                        conflicts.append(dict(road=rid, parent=port.road, lane=port.lane,
                            contact=port.contact, field=side, source_lane_id=tail['source_lane_id'],
                            fixed_endpoint_source_distance_m=distance, final_source_budget_m=.35,
                            reason='fixed endpoint already exceeds the corresponding original tail budget',
                            global_impossibility_proven=False, source_change_authorized=False))
        return conflicts

    def frames(self, rid, contacts):
        return [self.parents[p.road].frame(p, contacts[end], self.forward[rid, end])
                if p.road in self.parents else self.fixed[rid, end]
                for end, p in enumerate(self.graph.connections[rid])]

    def turn(self, rid, local, contacts):
        kernel = self.kernels[rid]; frames = self.frames(rid, contacts)
        full = kernel['unpack'](local, frames)
        result = kernel['evaluate'](full, frames)
        # End-edge jets are eliminated exactly; retain center as a separate gate.
        equality = np.r_[result[4][12:], center_endpoint_residual(result, frames)]
        return equality, result[5], result, full

    def residual(self, vector):
        self.evaluations += 1
        out = [np.minimum(p.slack0+p.CZ@vector[self.parent_slices[rid]], 0.)
            for rid, p in sorted(self.parents.items())]
        for rid, sl in self.turn_slices.items():
            eq, iq, _, _ = self.turn(rid, vector[sl], self.contact_jets(vector, rid))
            out.append(np.r_[10*eq, np.minimum(iq, 0.)])
        return np.concatenate(out)

    def regular_geometry(self, vector):
        """Keep a valid offset chart and positive ribbon during restoration.

        Lower endpoint/shape residual does not justify crossing a Frenet
        singularity. Both quantities are polynomial-extremum checks already
        supplied by the same full geometry kernel, not a coarse new grid.
        """
        rows=[]
        for rid,sl in self.turn_slices.items():
            _,_,result,_=self.turn(rid,vector[sl],self.contact_jets(vector,rid))
            width,forward=float(result[7]),float(result[8])
            rows.append(dict(road=rid,minimum_width_m=width,minimum_forward_factor=forward,
                regular=bool(np.isfinite([width,forward]).all() and min(width,forward)>=.1-1e-8)))
        return dict(regular=all(r['regular'] for r in rows),rows=rows)

    def jacobian(self, vector):
        """Differentiate six source contact jets, not hundreds of parent coefficients.

        The parent map is exact and linear; only each connector's own geometric
        function is finite-differenced. No independent optimal solution is used.
        """
        rows = []
        n = len(vector)
        for rid, p in sorted(self.parents.items()):
            sl = self.parent_slices[rid]; slack = p.slack0+p.CZ@vector[sl]
            block = np.zeros((len(slack), n)); block[:, sl] = p.CZ*(slack < 0)[:, None]
            rows.append(block)
        for rid, sl in self.turn_slices.items():
            local = vector[sl]; contacts = self.contact_jets(vector, rid)
            def f(y, jets):
                eq, iq, _, _ = self.turn(rid, y, jets)
                return np.r_[10*eq, np.minimum(iq, 0.)]
            base = f(local, contacts); block = np.zeros((len(base), n))
            for j in range(len(local)):
                step = 1e-6*max(1., abs(local[j]))
                if local[j]+step > self.bounds[sl.start+j, 1]: step = -step
                y = local.copy(); y[j] += step
                block[:, sl.start+j] = (f(y, contacts)-base)/step
            for end, jets in contacts.items():
                port = self.graph.connections[rid][end]; parent = self.parents[port.road]
                d = np.zeros((len(base), 6))
                for j in range(6):
                    step = 1e-6*max(1., abs(jets[j])); changed = dict(contacts)
                    changed[end] = jets.copy(); changed[end][j] += step
                    d[:, j] = (f(local, changed)-base)/step
                block[:, self.parent_slices[port.road]] += d @ parent.maps[port][2]
            rows.append(block)
        return csr_matrix(np.vstack(rows))

    def summary(self, vector):
        parents = {rid: dict(maximum_source_inequality_violation=float(max(0., -min(p.slack0+p.CZ@vector[self.parent_slices[rid]]))),
            coefficient_change_max_m=float(max(abs(p.coefficients(vector[self.parent_slices[rid]])-p.x0))))
            for rid, p in self.parents.items()}
        turns = []
        for rid, sl in self.turn_slices.items():
            eq, iq, r, _ = self.turn(rid, vector[sl], self.contact_jets(vector, rid))
            turns.append(dict(road=rid,maximum_scaled_equality=float(max(abs(eq))),
                minimum_construction_slack=float(min(iq)),source_objective=float(r[11]),
                status='CONSTRUCTION_CANDIDATE' if max(abs(eq)) <= 1e-6 and min(iq) >= -1e-5 else 'REJECTED'))
        feasible = all(v['maximum_source_inequality_violation'] <= 1e-7 for v in parents.values()) and all(
            t['status'] == 'CONSTRUCTION_CANDIDATE' for t in turns)
        return dict(status='REQUIRES_FILE_READBACK' if feasible else 'REJECTED', parents=parents, turns=turns,
            simultaneous_parent_connector_variables=True, source_prioritized_optimization_completed=False,
            production_accepted=False)
