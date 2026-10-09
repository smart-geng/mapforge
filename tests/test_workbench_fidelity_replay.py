from copy import deepcopy
import pytest

from mapforge.workbench.fidelity_replay import matches_fidelity_replay


def report():
    return {"status": "FAIL", "policy": {"tolerance_m": .1}, "count": 1,
            "per_lane": [{"source_id": "original", "status": "FAIL",
                          "coverage": {"target_length_m": 29.37297012321616},
                          "source_to_target": {"median_m": .020222782544949833, "p95_m": .09726489743646086},
                          "target_to_source": {"median_m": .020222782544949833, "p95_m": .09726489743646086},
                          "endpoint": {"travel_end_m": 0.0}}]}


def test_native_roundoff_preserves_original_report_and_failure():
    saved=report();before=deepcopy(saved);native=deepcopy(saved)
    native["per_lane"][0]["coverage"]["target_length_m"]=29.372970123216156
    native["per_lane"][0]["source_to_target"]["median_m"]=.020222782544951693
    native["per_lane"][0]["endpoint"]["travel_end_m"]=3.3350058825654116e-15
    assert matches_fidelity_replay(saved,native)
    assert saved==before and saved["status"]=="FAIL"


@pytest.mark.parametrize("fault",["measurement","policy","decision","lane_decision","identity","count","missing","extra","nonfinite","type"])
def test_roundoff_cannot_hide_changes_to_quality_or_binding(fault):
    saved=report();changed=deepcopy(saved);lane=changed["per_lane"][0]
    if fault=="measurement":lane["source_to_target"]["p95_m"]+=1e-10
    if fault=="policy":changed["policy"]["tolerance_m"]+=1e-15
    if fault=="decision":changed["status"]="PASS"
    if fault=="lane_decision":lane["status"]="PASS"
    if fault=="identity":lane["source_id"]="another"
    if fault=="count":changed["count"]=True
    if fault=="missing":del lane["endpoint"]
    if fault=="extra":lane["invented"]=0
    if fault=="nonfinite":lane["endpoint"]["travel_end_m"]=float("nan")
    if fault=="type":lane["endpoint"]["travel_end_m"]=0
    assert not matches_fidelity_replay(saved,changed)


def test_unlisted_metric_remains_exact_even_with_small_noise():
    saved=report();saved["per_lane"][0]["source_to_target"]["max_m"]=1.0
    changed=deepcopy(saved);changed["per_lane"][0]["source_to_target"]["max_m"]+=1e-15
    assert not matches_fidelity_replay(saved,changed)
