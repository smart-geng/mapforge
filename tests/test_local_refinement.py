"""R1 probes validate serialization, not satisfaction of the failed S2 target."""
from dataclasses import FrozenInstanceError, replace
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from lxml import etree

from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle, ScopeError
from mapforge.repair_web.local_refinement import CUTS, EDGE, KNOT, LO, HI, prepare_local_refinement
from mapforge.repair_web.model import parse, digest
from scripts.check_outer_event_control import inputs, POINT, ROOT
from scripts.check_outer_event_written import boundary_poly


@pytest.fixture(scope='module')
def context():
    event, control = inputs()
    request = EditScopeRequest(control.reference_sha256, '11', (EDGE,), (LO, HI), (ScopeHandle(POINT),))
    model = prepare_local_refinement(event, control.reference, request)
    return event, control, request, model


def compile_probe(model, state):
    return model.compile_probe(state, contract_sha256=model.contract_sha256, purpose='R1_COMPILER_PROBE')


def test_noop_identity_and_no_extra_records(context):
    model = context[-1]
    data, report = compile_probe(model, [0., 0.])
    assert data is model.reference
    assert report['noop_original_bytes']
    assert not any(report['structure_change'].values())
    assert not report['map_accepted'] and not report['export_allowed']
    assert not report['user_target_checked']


@pytest.mark.parametrize('state', [[1e-6, 0.], [0., 1e-6], [1e-6, -1e-6], [-1e-6, 1e-6]])
def test_two_directions_actual_xml_xsd_source_and_frozen_curves(context, state):
    model = context[-1]
    data, report = compile_probe(model, state)
    schema = etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    schema.assertValid(etree.fromstring(data))
    assert report['structure_change'] == dict(geometry=0, width=2, laneSection=0, laneOffset=0)
    assert report['frozen_curve_change_m'] < 1e-10
    assert report['whole_polynomial_error_m'] < 1e-10
    assert report['source_readback']['source_event_max_m'] <= .7500001
    assert report['status'] == 'R1_WRITER_CONSISTENT_NOT_SHAPE_ACCEPTANCE'
    root = parse(data); old = parse(model.reference)
    road, before = (r.find("road[@id='11']") for r in (root, old))
    delta = model.delta_power(178., state)
    change = boundary_poly(road, EDGE, 178.)-boundary_poly(before, EDGE, 178.)
    np.testing.assert_allclose(change, delta, atol=1e-12)
    assert np.linalg.norm(change) > 0
    for edge in (0, 1, 2, 4):
        np.testing.assert_allclose(boundary_poly(road, edge, 178.), boundary_poly(before, edge, 178.), atol=1e-12)


def test_zero_jets_finite_support_and_c2_internal_knots(context):
    model = context[-1]
    for state in ([1., 0.], [0., 1.]):
        for s in (0., LO-.01, HI+.01, 211.):
            np.testing.assert_array_equal(model.delta_power(s, state), np.zeros(4))
        np.testing.assert_allclose(model.delta_power(LO, state)[:3], 0., atol=1e-12)
        np.testing.assert_allclose(model.delta_power(HI, state, left=True)[:3], 0., atol=1e-12)
        for s in CUTS[1:-1]:
            np.testing.assert_allclose(model.delta_power(s, state)[:3], model.delta_power(s, state, left=True)[:3], atol=1e-12)


@pytest.mark.parametrize('state', [[True, 0.], [0., float('nan')], [0., float('inf')], [0.], [[0., 0.]], ['0', 0.]])
def test_invalid_state_not_clipped(context, state):
    with pytest.raises(ScopeError): compile_probe(context[-1], state)


def test_scope_reference_and_source_identity_cannot_change(context):
    event, control, request, model = context
    for changed in (replace(request, edges=(3, 4)), replace(request, interval=(LO-1., HI)),
                    replace(request, handles=(ScopeHandle(replace(POINT, source_key='wrong-source')),))):
        with pytest.raises(ScopeError): prepare_local_refinement(event, control.reference, changed)
    with pytest.raises(ScopeError): prepare_local_refinement(event, control.reference+b' ', request)


@pytest.mark.parametrize('bad_handles', [None, (), (object(),), [ScopeHandle(POINT)]])
def test_malformed_nested_request_rejected(context, bad_handles):
    event, control, request, _ = context
    with pytest.raises(ScopeError):
        prepare_local_refinement(event, control.reference, replace(request, handles=bad_handles))


def test_immutable_contract_and_stale_state_rejection(context):
    model = context[-1]
    with pytest.raises(FrozenInstanceError): model.reference = b'changed'
    for attr in ('reference', '_powers', '_packet', '_reference_cells'):
        changed = replace(model, **{attr: getattr(model, attr)+b' '})
        with pytest.raises(ScopeError): compile_probe(changed, [0., 0.])
    report = model.contract; report['free_directions'] = 100
    assert model.contract['free_directions'] == 2
    with pytest.raises(ScopeError):
        model.compile_probe([0., 0.], contract_sha256='stale', purpose='R1_COMPILER_PROBE')
    with pytest.raises(ScopeError):
        model.compile_probe([0., 0.], contract_sha256=model.contract_sha256, purpose='export')


@pytest.mark.parametrize('mutation', ['neighbor', 'outside', 'speed', 'junction', 'header', 'extra-width', 'nan', 'extension'])
def test_corrupted_actual_xml_refused(context, mutation):
    model = context[-1]; state = [1e-6, -1e-6]
    data, _ = compile_probe(model, state)
    root = parse(data); road = root.find("road[@id='11']")
    sec = next(s for s in road.findall('lanes/laneSection') if float(s.get('s')) == 181.7074107)
    lane = sec.find("right/lane[@id='-3']")
    if mutation == 'neighbor':
        w = sec.find("right/lane[@id='-1']/width"); w.set('a', str(float(w.get('a'))+.1))
    elif mutation == 'outside':
        w = road.find('lanes/laneSection/right/lane/width'); w.set('a', str(float(w.get('a'))+.1))
    elif mutation == 'speed':
        road.find('.//speed').set('max', '1')
    elif mutation == 'junction':
        root.find('junction').set('name', 'changed')
    elif mutation == 'header':
        root.find('header').set('name', 'changed')
    elif mutation == 'extra-width':
        ET.SubElement(lane, 'width', sOffset='0.001', a='3', b='0', c='0', d='0')
    elif mutation == 'nan':
        lane.find('width').set('a', 'nan')
    else:
        lane.find('width').set('unregistered', 'value')
    with pytest.raises(ScopeError): model.audit_compiled(ET.tostring(root), state)


def test_source_violation_or_negative_width_not_returned(context):
    with pytest.raises(ScopeError): compile_probe(context[-1], [100., -100.])


def test_no_solver_old_global_compiler_or_s2_called(context, monkeypatch):
    event, _, _, model = context
    def forbidden(*args, **kwargs): raise AssertionError('R1 is not a target solve')
    import mapforge.repair_web.outer_event as module
    import mapforge.repair_web.unique_edit as s2
    monkeypatch.setattr(module, 'minimize', forbidden)
    monkeypatch.setattr(module, 'linprog', forbidden)
    monkeypatch.setattr(event, 'compile', forbidden)
    monkeypatch.setattr(s2, 'evaluate_unique_edit', forbidden)
    data, report = compile_probe(model, [1e-6, -1e-6])
    assert report['xodr_sha256'] == digest(data)
    assert report['shape_acceptance'] == 'NOT_EVALUATED'
