import copy

import numpy as np
import pytest

from mapforge.ops.source_knot_layout import SourceKnotLayout
from mapforge.ops.coupled_source_junction import (MovableSourceParentState,
    WholeSourcePortState, source_contact_frame)
from mapforge.ops.port_dependencies import LanePort, PortDependencies
from spikes.source_contact_fit import SourceBoundaryBlock
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from tests.test_source_contact_fit import straight_fixture


def block():
    raw = SourceBoundaryBlock(*straight_fixture(), min_span=17.)
    return raw, ArcSourceBoundaryBlock(raw)


def constant_coefficients(model):
    x = np.zeros(model.nvar)
    for family in model.families:
        x[family.columns] = model.raw[family.features[0]][0, 1]
    return x


@pytest.mark.parametrize('k', [0., 1e-12, -.0001, .0001])
def test_axes_move_without_variable_count_or_original_vertex_changes(k):
    raw, base = block(); before = copy.deepcopy(raw.raw)
    layout = SourceKnotLayout.capture(base)
    moved = ArcSourceBoundaryBlock(raw, heading_delta=.0001, curvature=k,
                                  origin_delta=(.01, -.01), fixed_layout=layout)
    assert moved.nvar == base.nvar
    assert layout.features == tuple(tuple(f.features) for f in moved.families)
    for key in before:
        assert np.array_equal(raw.raw[key], before[key])
        assert np.array_equal(moved.source_xy[key], base.source_xy[key])
    for f, old in zip(moved.families, base.families):
        assert f.columns == old.columns
        assert min(np.diff(np.unique(f.knots))) >= 17-1e-7
    # New source rows, not the old chart's feasible linear region.
    assert not np.array_equal(moved.raw['boundary:B:ar'], base.raw['boundary:B:ar'])
    assert not hasattr(moved, 'CZ')


def test_shared_free_knot_moves_both_sides_without_new_spans():
    raw, base = block(); layout = SourceKnotLayout.capture(base)
    delta = np.zeros(len(layout.free_nodes)); delta[0] = .2
    moved = ArcSourceBoundaryBlock(raw, fixed_layout=layout, knot_displacements=delta)
    assert moved.nvar == base.nvar
    assert moved.global_breaks[layout.free_nodes[0]] == pytest.approx(base.global_breaks[layout.free_nodes[0]]+.2)
    for f in moved.families:
        assert np.unique(f.knots)[1] == pytest.approx(base.global_breaks[1]+.2)
    # Two source records of the same physical edge still own one vector.
    assert moved.owner['boundary:B:al'] == moved.owner['boundary:B:bl']


@pytest.mark.parametrize('fault', ['short', 'nonfinite', 'dimension', 'new_anchor', 'new_origin'])
def test_invalid_trial_rejects_instead_of_repairing_layout_or_mutating_source(fault):
    raw, base = block(); before = copy.deepcopy(raw.raw); layout = SourceKnotLayout.capture(base)
    kwargs = dict(fixed_layout=layout)
    if fault == 'short': kwargs['knot_displacements'] = [-10., 0.]
    if fault == 'nonfinite': kwargs['knot_displacements'] = [np.nan, 0.]
    if fault == 'dimension': kwargs['knot_displacements'] = [0.]
    if fault == 'new_anchor': kwargs['structural_stations'] = [30.]
    if fault == 'new_origin': kwargs['origin_delta'] = [3., 0.]
    with pytest.raises(ValueError): ArcSourceBoundaryBlock(raw, **kwargs)
    for key in before: assert np.array_equal(raw.raw[key], before[key])


def parent(monkeypatch):
    raw, model = block(); x = constant_coefficients(model)
    # This synthetic fixture has two raw source records and no speed ledger.
    # Supply that fixture-only source chain, not a production role override.
    from mapforge.ops import source_cubic_export
    monkeypatch.setattr(source_cubic_export, 'source_chains', lambda m: [dict(
        a=0., b=60., source_ids=['a','b'], domains=[dict(a=0.,b=60.,
            source_lanes=['a','b'],left='boundary:B:al',right='boundary:B:ar')])])
    compiled = dict(chart_start_m=0., chart_end_m=60., lane_ledger=[dict(section=0,
        lane=-1, left='boundary:B:al', right='boundary:B:ar', chart_s=[0.,60.])])
    return MovableSourceParentState(raw, model, x, compiled)


def test_moving_cut_and_axis_rebuilds_actual_port_world_jets_and_source_constraints(monkeypatch):
    p = parent(monkeypatch); original = p.initial.copy(); v = original.copy()
    v[1] = .01; v[2] = .0001; v[3] = .00001
    v[4] = .1; v[5] = -.1
    a = p.evaluate(original); b = p.evaluate(v); port = LanePort('10', 'end', -1)
    assert not hasattr(p, 'CZ')
    assert b['stations'][port] == 59.9
    assert not np.array_equal(a['model'].raw['boundary:B:al'], b['model'].raw['boundary:B:al'])
    old = p.frame(a, port, True); new = p.frame(b, port, True)
    assert abs(new['center']['x']-old['center']['x']) > .09
    assert abs(new['center']['heading']-old['center']['heading']) > .0001
    assert abs(new['center']['curvature']-old['center']['curvature']) > 1e-6
    assert b['model'].written_endpoint_constraints > 0
    assert not b['export_allowed'] and b['constraints_rebuilt']
    assert np.array_equal(p.initial, original)
    # Deliberately bad coefficients must remain visible as negative source
    # slack, not labelled PASS just because the endpoint state exists.
    bad = v.copy(); bad[p.coefficient_slice] += 1.
    assert p.evaluate(bad)['source_inequality_slack'].min() < -.5


@pytest.mark.parametrize('forward', [True, False])
def test_moved_world_derivatives_match_independent_finite_difference(monkeypatch, forward):
    p = parent(monkeypatch); v = p.initial.copy(); v[3] = .00001; v[4:6] = [.1, -.1]
    state = p.evaluate(v); axis = state['model'].axis; s = 59.9
    jets = np.array([2., .03, .0002, -2., .01, .0001])
    frame = source_contact_frame(axis, s, jets, forward)
    t, d1, d2 = .5*(jets[:3]+jets[3:])
    eps = .001
    def point(ds): return axis.world([s+ds, t+ds*d1+.5*ds*ds*d2])
    tangent = (point(eps)-point(-eps))/(2*eps)
    accel = (point(eps)-2*point(0)+point(-eps))/eps**2
    k = float(np.linalg.det(np.stack([tangent, accel]))/np.linalg.norm(tangent)**3)
    assert frame['center']['curvature'] == pytest.approx(k*(1 if forward else -1), abs=1e-8)


def test_whole_parent_state_updates_all_incident_turns_and_rejects_partial_scope(monkeypatch):
    from tests.test_coupled_source_junction import double_graph
    from tests.test_junction_edges import network
    root = network(); graph = double_graph(root)
    p = parent(monkeypatch); q = copy.deepcopy(p); q.road = '11'
    q.original.road = '11'
    # Independent synthetic source identities for the second parent.
    # Use a delegated fixture with real evaluation and remapped port identity.
    class Parent11:
        road = '11'; initial = q.initial; nvar = q.nvar
        from types import SimpleNamespace
        layout = SimpleNamespace(features=(('other-left','other-right'),))
        port_features = {LanePort('11','start',-1): ('other-left','other-right')}
        def evaluate(self, v): return p.evaluate(v)
        def frame(self, state, port, forward):
            return p.frame(state, LanePort('10',port.contact,port.lane), forward)
    with pytest.raises(ValueError, match='complete whole-junction'):
        WholeSourcePortState(root, '1', {'10':p}, graph.connections)
    whole = WholeSourcePortState(root, '1', {'10':p,'11':Parent11()}, graph.connections)
    a = whole.evaluate(whole.initial); v = whole.initial.copy(); v[whole.slices['10'].start+1] = .01
    b = whole.evaluate(v)
    assert set(b['connector_frames']) == {'100','101'}
    for cid in graph.connections:
        assert b['connector_frames'][cid][0]['center']['y']-a['connector_frames'][cid][0]['center']['y'] == pytest.approx(.01)
        assert b['connector_frames'][cid][1] == a['connector_frames'][cid][1]
    before = whole.initial.copy(); v[whole.slices['11'].start+5] = -61.
    with pytest.raises(ValueError): whole.evaluate(v)
    assert np.array_equal(before, whole.initial)
    assert not b['export_allowed']
