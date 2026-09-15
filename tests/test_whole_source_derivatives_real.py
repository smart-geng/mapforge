"""Local node4 proof of coordinate/source checks, explicitly NOT map PASS."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.sparse import load_npz
from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'out/node4-whole-derivatives-r5-20260914'


@pytest.fixture(scope='module')
def report():
    path=OUTPUT/'report.json'
    if not path.exists():pytest.skip('local whole-model derivative evidence not installed')
    r=json.loads(path.read_text(encoding='utf8'))
    # This is an immutable r5 measurement, not a rerun of its old compiler.
    # Only explicit Python-code migration is allowed; original data,
    # budgets, decisions and Profile must still match the recorded hashes.
    current,changes=bind_current_code(r['input_files_sha256']);unchanged(current)
    r['_current_input_binding']=current;r['_code_revision_changes']=changes
    return r


def test_complete_original_rows_and_partial_derivative_extent_are_explicit(report):
    assert report['variables']==1681 and report['free_variables']==1676
    assert report['equality_count']==507 and report['inequality_count']==1217
    assert len(report['fixed_columns'])==5 and len(report['probe_columns'])==9
    assert set(report['parent_source_certificates'])=={'10','11','12','13','30'}
    assert sum(x['original_total'] for x in report['parent_source_certificates'].values())==51084
    assert all(x['all_original_rows_accounted'] for x in report['parent_source_certificates'].values())
    for scale in ('1.0','0.5'):
        matrix=load_npz(OUTPUT/f'stencil-{scale}.npz')
        assert matrix.shape==(1724,9) and np.isfinite(matrix.data).all()
    assert all(r['maximum_residual_difference']==0 for r in report['whole_local_equivalence'])
    assert report['max_scaled_equality']>1 and report['original_kernel_minimum_slack']< -1
    assert not report['full_jacobian_checked'] and not report['smoothness_certified']
    assert not report['optimization_ran'] and not report['production_accepted']
    assert not report['domain_dynamics_surface_complete'] and not list(OUTPUT.glob('*.xodr'))


def test_all_24_working_sides_use_same_source_boundaries_as_both_parent_ports(report):
    rows=report['physical_port_source_binding']
    assert {r['road'] for r in rows}==set(map(str,range(100,124)))
    assert sum(len(r['endpoint_source_gaps']) for r in rows)==96
    assert all(r['side_mapping']=={'left':'right','right':'left'} for r in rows)
    # Registration-source match only. NOT target connector closure or G2.
    assert max(r['maximum_m'] for r in rows)<.03


def test_original_tangent_budget_no_longer_jumps_when_tail_length_reaches_zero(report):
    traces=json.loads((OUTPUT/'source-correspondence-trace.json').read_text(encoding='utf8'))
    budgets=[t['fairness']['left']['source']['reverse_turn_deg'] for r in traces
             for t in r['turns'] if t['road']=='106']
    assert len(budgets)==3 and max(budgets)-min(budgets)<1e-8
    assert budgets[0]==pytest.approx(.402291151,abs=1e-8)
    assert not any('added-reverse-turn' in str(r['identity']) for r in report['step_halving_disagreements'])
    unchanged(report['_current_input_binding'])
