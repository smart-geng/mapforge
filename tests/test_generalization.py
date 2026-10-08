from mapforge.validate import generalization as G


def test_cause_comes_from_the_last_exception_line():
    log = """│   93 │   │   except RF.ReflineFitError as exc:                                   │
└─────────────────────────────────────────────────────────────────────────────┘
ValueError: source-envelope mouth would remove the seed road: station=2.167,
min_keep=6.000"""
    assert G.cause_of(log) == "mouth-would-remove-seed-road"
    assert G.cause_of("ReflineFitError: SHP junction=1 enter_link=2 points=9: 参考线拟合无合格候选") == "leg-reference-fit"
    assert G.cause_of("ValueError: envelope surface not generated: FAIL") == "junction-paving"
    assert G.cause_of("KeyError: 'x'") == "other:KeyError"


def test_summary_counts_causes_and_tiers():
    rows = [{"generated": False, "cause": "junction-paving"},
            {"generated": True, "tiers": {"T1": {"status": "PASS", "failed": []},
                                          "T2": {"status": "FAIL", "failed": [{"metric": "m"}]}}}]
    s = G.summarize(rows)
    assert s["generated"] == 1 and s["not_generated_by_cause"] == {"junction-paving": 1}
    assert s["tier_pass"] == {"T1": 1, "T2": 0} and s["failed_metrics"] == {"T2:m": 1}
