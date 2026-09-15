"""Real node4 whole-state numerical evidence; never a map acceptance test."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.sparse import load_npz

from scripts.fit_source_boundary_block import unchanged

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'out/node4-whole-joint-step-r2-20260914'


def read(name):return json.loads((OUTPUT/name).read_text(encoding='utf8'))


@pytest.fixture(scope='module')
def report():
    if not (OUTPUT/'report.json').exists():pytest.skip('complete local node4 joint-step evidence not installed')
    r=read('report.json');unchanged(r['input_files_sha256'])
    return r


def test_every_real_free_column_has_explicit_sides_and_no_unchecked_padding(report):
    contract=read('residual-contract.json');valid=read('sided-validity.json')
    columns=np.array(contract['columns']);diag=read('stencil-diagnostics.json')
    assert len(columns)==len(diag)==1676 and report['stencil_shape']==[1724,1676]
    assert set(columns)|set(report['fixed_columns'])==set(range(1681))
    assert not set(columns)&set(report['fixed_columns'])
    assert list(columns)==[d['column'] for d in diag]
    assert all(d['jacobian_smoothness_certified'] is False for d in diag)
    assert all(a or b for a,b in zip(valid['right'],valid['left']))
    for name in ('central','right','left'):
        a=load_npz(OUTPUT/f'stencil-{name}.npz')
        assert a.shape==(1724,1676) and np.isfinite(a.data).all()
    assert report['complete_free_columns'] and not report['smooth_jacobian_certified']
    assert report['one_sided_columns']  # Long primitive/domain limits remain explicit.


def test_all_original_certificates_survive_affine_acceleration_and_actual_joint_rebuild(report):
    summary=read('residual-contract.json')['summary']
    assert sum(r['original_total'] for r in summary['parent_source_certificates'].values())==51084
    assert all(r['all_original_rows_accounted'] for r in summary['parent_source_certificates'].values())
    checks=report['coefficient_equivalence']
    assert len(checks)==10 and {r['parent'] for r in checks}=={'10','11','12','13','30'}
    assert max(r['max_residual_difference'] for r in checks)<1e-10
    assert any(len(r['recomputed_blocks'])==1 for r in checks)
    assert any(len(r['recomputed_blocks'])>1 for r in checks)
    assert report['original_rows_accounted'] and report['all_source_rows_retained']
    assert report['independent_full_max_difference']<1e-10


def test_one_atomic_all29_block_trial_is_not_optimization_or_a_written_map(report):
    trial=read('joint-trial-registration.json');delta=np.array(trial['delta'])
    assert np.count_nonzero(delta)==len(report['moved_columns'])==29
    assert np.array_equal(np.flatnonzero(delta),report['moved_columns'])
    assert np.all(delta[delta!=0]==1e-5)
    blocks={tuple(b) for b in report['recomputed_blocks']}
    assert blocks==({('parent',r) for r in ('10','11','12','13','30')}|
                    {('turn',str(r)) for r in range(100,124)})
    saved=json.loads((ROOT/'out/node4-whole-source-model-r4-20260914/whole-coordinate-state.json').read_text(encoding='utf8'))
    np.testing.assert_array_equal(trial['base'],saved['initial'])
    actual=np.load(OUTPUT/'joint-trial-residuals.npz',allow_pickle=False)
    assert actual['actual'].shape==actual['before'].shape==(1724,)
    assert not np.array_equal(actual['actual'],actual['before'])
    assert report['actual_violated_inequalities']>0 and report['actual_max_scaled_equality']>1
    assert not report['optimization_ran'] and not report['xodr_generated']
    assert not report['export_allowed'] and not report['domain_dynamics_surface_complete']
    assert not report['equality_feasibility_certified'] and not list(OUTPUT.glob('*.xodr'))
