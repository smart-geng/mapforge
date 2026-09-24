"""Source/scope metadata only; explicitly prohibit nonlinear solving/writing."""
import numpy as np
import pytest
from scripts.review_split_event_scope import source_at, review, START, END


def test_source_at_preserves_identity_and_does_not_extrapolate():
    trace = dict(key='measured', edge=3, record=2, part=1, st=np.array([[10., 1.], [20., 3.]]))
    assert source_at([trace], 3, 9.) == []
    assert source_at([trace], 2, 15.) == []
    assert source_at([trace], 3, 15.) == [dict(key='measured', record=2, part=1,
        segment_s=[10., 20.], t=2., slope=.2)]


@pytest.fixture(scope='module')
def report():
    from scripts.check_outer_event_control import inputs
    e, c = inputs()
    return review(e, c.reference)


def test_full_review_does_not_authorize_geometry_or_new_trial(report):
    assert not report['new_xodr'] and not report['new_trial_registered']
    assert not report['connector_geometry_edit_allowed'] and not report['map_accepted']
    assert report['optimizer_calls'] == 0
    assert report['proposed_edit_domain'] == [START, END]


def test_linked_births_and_six_connectors_are_not_hidden(report):
    assert report['births'] == {3: 131.0543, 4: 151.6089}
    assert {r['connectingRoad'] for r in report['connections']} == {str(r) for r in range(106, 112)}
    assert [p['edge'] for p in report['outer_envelope']] == [2, 3, 4]


def test_raw_support_not_trimmed_to_parent(report):
    assert report['source_traces'] == 25 and report['movement_paths'] == 19
    assert len([r for r in report['source_inventory'] if r['outside_parent_support']]) == 6
    assert any(r['source_s'][0] < 0 for r in report['source_inventory'])
    assert max(r['source_s'][1] for r in report['source_inventory']) > END+6


def test_bad_old_anchor_not_silently_relabelled_source_truth(report):
    rows = report['old_cut_anchors']
    assert rows[0]['source_direction_disagreement']
    assert rows[2]['source_direction_disagreement']
    assert all(not r['final_anchor_admitted'] for r in report['proposed_anchors'])
