from scripts.review_source_turn_budget import classify
import pytest


def test_source_s_bend_keeps_its_turning_budget():
    source=dict(source_reverse_turn_deg=55.,written_reverse_turn_deg=55.3,source_single_turn_observed=False)
    row=classify(source)
    assert row['status']=='WITHIN_SOURCE_TURN_BUDGET'
    assert not row['source_s_bend_forced_monotone'] and not row['production_accepted']


def test_non_single_turn_is_not_silently_unchecked():
    source=dict(source_reverse_turn_deg=2.268961,written_reverse_turn_deg=7.558796,source_single_turn_observed=False)
    row=classify(source)
    assert row['status']=='EXCESS_SOURCE_TURN_BUDGET'
    assert row['added_reverse_turn_deg']>5.


def test_simple_turn_rule_unchanged():
    row=classify(dict(source_reverse_turn_deg=0.,written_reverse_turn_deg=1.1,source_single_turn_observed=True))
    assert row['status']=='EXCESS_SOURCE_TURN_BUDGET'


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-1.])
def test_nonfinite_or_invalid_evidence_fails_closed(value):
    with pytest.raises(ValueError):
        classify(dict(source_reverse_turn_deg=0.,written_reverse_turn_deg=value,source_single_turn_observed=True))
