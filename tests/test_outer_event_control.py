"""Hard target, source ownership, bounded range and no-mutation contracts."""
from dataclasses import replace
import json
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from mapforge.repair_web.model import digest, complexity
from mapforge.repair_web.outer_event_control import EventControl, recover_state
from scripts.check_outer_event_control import inputs, POINT, REFERENCE


@pytest.fixture(scope='module')
def prepared():
    event, control = inputs()
    report = control.prepare_range()
    assert report['verified_delta_m'][0] < 0
    return event, control, report


def test_recover_actual_xml_in_common_basis(prepared):
    event, control, _ = prepared
    assert np.max(abs(event.E@control.current-event.e)) < 1e-7
    assert digest(control.reference) == control.reference_sha256
    assert event.source_error(control.current)['max_m'] < .7500001


@pytest.mark.parametrize('fraction', [0., .1, .5, 1.])
def test_exact_target_in_final_xml_and_all_shared_guards(prepared, fraction):
    event, control, report = prepared
    requested = report['verified_delta_m'][0]*fraction
    before = control.reference
    x = control.current.copy()
    data, guard = control.preview(requested)
    assert guard['requested_m'] == requested
    assert guard['achieved_m'] == pytest.approx(requested, abs=1e-8)
    assert guard['readback']['status'] == 'PASS_LOCAL_EVENT_NOT_MAP'
    assert guard['outside_max_change_m'] == 0.
    assert guard['readback']['other_roads_unchanged']
    assert guard['readback']['ids_links_speeds_marks_unchanged']
    assert np.all(np.array(list(guard['reversal_by_edge_m'].values())) <= control.budgets+1e-7)
    assert complexity(ET.fromstring(data))['geometry'] == complexity(ET.fromstring(before))['geometry']
    assert not guard['map_accepted'] and not guard['static_shape_pass']
    assert control.preview(requested)[0] == data  # byte-identical replay, no solve
    assert control.reference == before and np.array_equal(control.current, x)
    if fraction == 0.: assert data == before


def test_rejection_is_not_clipping_or_a_new_geometry(prepared):
    _, control, report = prepared
    previous, _ = control.preview(report['verified_delta_m'][0]*.5)
    with pytest.raises(ValueError, match='超出已验证区间'):
        control.preview(report['source_point_delta_m'])
    assert control.preview(report['verified_delta_m'][0]*.5)[0] == previous
    assert control.reference == REFERENCE.read_bytes()


@pytest.mark.parametrize('delta', [True, None, '0.1', float('nan'), float('inf')])
def test_malformed_target_is_not_a_noop(prepared, delta):
    with pytest.raises(ValueError, match='Finite absolute'):
        prepared[1].preview(delta)


@pytest.mark.parametrize('kwargs', [dict(edge=2), dict(edge=True), dict(station=99.),
                                  dict(station=float('nan')), dict(source_key='unknown'), dict(record=-1)])
def test_handle_identity_not_guessed_from_nearest_line(prepared, kwargs):
    event, control, _ = prepared
    with pytest.raises(ValueError): EventControl(event, control.reference, replace(POINT, **kwargs))


def test_hidden_between_samples_record_is_rejected(prepared):
    event, control, _ = prepared
    root = ET.fromstring(control.reference)
    road = next(r for r in root.findall('road') if r.get('id') == '11')
    sec = max((s for s in road.findall('lanes/laneSection') if float(s.get('s')) <= 177.123),
              key=lambda s: float(s.get('s')))
    lane = sec.find('right/lane[@id="-3"]')
    ET.SubElement(lane, 'width', sOffset=str(177.123-float(sec.get('s'))), a='2', b='0', c='0', d='0')
    with pytest.raises(ValueError): recover_state(event, ET.tostring(root))


def test_timeout_is_unknown_not_impossible_and_only_zero_is_admitted(prepared, monkeypatch):
    import mapforge.repair_web.outer_event_control as module
    event, reference, _ = prepared
    control = EventControl(event, reference.reference, POINT)
    monkeypatch.setattr(module, '_convex', lambda *a: (None, {'status': 'MaxTime'}))
    report = control.prepare_range()
    assert report['verified_delta_m'] == [0., 0.]
    assert all(r['termination'] == 'UNDETERMINED_KEEP_VERIFIED_REFERENCE' for r in report['history'])
    assert report['beyond_interval'] == 'UNVERIFIED_NOT_PROVEN_IMPOSSIBLE'
    assert control.preview(0.)[0] == reference.reference
    with pytest.raises(ValueError): control.preview(.01)


def test_target_exclusion_identifies_conservative_guard_not_universal_impossibility(prepared):
    _, control, report = prepared
    diagnosis = control.diagnose_target(report['source_point_delta_m'])
    rows = diagnosis['rows']
    assert rows[0]['target_status'] == 'NUMERICALLY_EXCLUDED'
    assert rows[0]['stationarity_residual'] < 1e-7
    assert 'derivative-nonregression' in rows[0]['active_families']
    assert rows[1]['omitted_diagnostic_only'] == 'derivative-nonregression'
    assert rows[1]['target_status'] == 'INCONCLUSIVE'
    assert not diagnosis['formal_certificate'] and not diagnosis['candidate_written']
    assert not diagnosis['omitted_guards_accepted']


def test_corrupt_witness_cannot_change_written_point(prepared):
    event, original, report = prepared
    control = EventControl(event, original.reference, POINT)
    control.report = json.loads(json.dumps(report))
    control.witnesses[-1] = original.witnesses[-1].copy()
    control.witnesses[-1][0] += .1
    with pytest.raises(ValueError): control.preview(report['verified_delta_m'][0]*.5)


@pytest.mark.parametrize('budget', [0, 25, True])
def test_unbounded_or_invalid_range_search_forbidden(prepared, budget):
    with pytest.raises(ValueError): prepared[1].prepare_range(max_rounds=budget)
