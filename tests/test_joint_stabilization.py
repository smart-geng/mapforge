from types import SimpleNamespace
import numpy as np
import pytest
from pyclothoids import Clothoid
from mapforge.ops.world_curve_fairness import source_turn_slacks
from mapforge.ops.fixed_ribbon_sampling import sample_fixed
from mapforge.ops.step_limited_sqp import solve


def test_compound_source_is_budgeted_not_exempt_or_forced_monotone():
    rows={k:dict(additional_reverse_turn_deg=v,source=dict(single_turn=False,reverse_turn_deg=20.))
          for k,v in zip(('left','right','center'),(0.,5.,-.1))}
    slack=source_turn_slacks(rows)
    assert len(slack)==3 and slack[0]>0 and slack[1]<0 and slack[2]>0
    with pytest.raises(ValueError):source_turn_slacks({'center':rows['center']})


def test_fixed_sites_have_constant_count_and_include_structural_midpoints():
    co={'left':np.array([[2.,0.,0.,0.]]*2),'right':np.array([[-2.,0.,0.,0.]]*2)}
    arrays=[]
    for L in (6.,6.+1e-7,8.):
        cls=[Clothoid.StandardParams(0,0,0,0,0,L)]
        p=sample_fixed(cls,np.array([0.,L/2,L]),co,[8.],.1)['center'];arrays.append(p)
        assert np.max(np.diff(p[:,0]))<=.1+1e-12
        assert np.min(abs(p[:,0]-L/2))<1e-12
        assert p[0,0]==0 and p[-1,0]==pytest.approx(L)
    assert len({len(a) for a in arrays})==1
    assert np.max(abs(arrays[1]-arrays[0]))<1.1e-7
    with pytest.raises(ValueError):sample_fixed(cls,[0.,4.,8.],co,[7.],.1)


def test_failed_inner_solver_cannot_replace_feasible_anchor():
    def evaluate(x):return np.array([x[0]]),np.array([1.]),-x[0]
    def bad(fun,x,**kwargs):
        trial=np.array([.25]);kwargs['callback'](trial)
        return SimpleNamespace(x=trial,success=True,message='false success',nit=1)
    x,r=solve(evaluate,[0.],[[-10,10]],[1.],max_iterations=2,optimizer=bad)
    assert x[0]==0 and not any(h['accepted'] for h in r['stages'])


def test_local_steps_can_accumulate_without_relaxing_equations():
    def evaluate(x):return np.array([x[1]]),np.array([x[0]+1]),(x[0]-3.)**2
    x,r=solve(evaluate,[0.,0.],[[-10,10],[-10,10]],[.5,.2],max_iterations=70)
    assert x==pytest.approx([3.,0.],abs=1e-5)
    assert r['success'] and not r['global_optimality_claimed']


def test_optimizer_escape_is_rejected():
    def bad(fun,x,**kwargs):return SimpleNamespace(x=np.array([20.]),success=True,message='bad',nit=1)
    with pytest.raises(ValueError,match='escaped'):
        solve(lambda x:(np.array([0.]),np.array([1.]),0.),[0.],[[-1,1]],[1.],max_iterations=1,optimizer=bad)


def test_loose_via_gate_cannot_hide_stricter_original_tail_failure(monkeypatch):
    from tests.test_joint_connector_restore import truth
    import mapforge.ops.joint_connector_fit as joint
    road,a,b,raw,seed=truth()
    curves={k:v+np.array([0.,1.]) for k,v in raw.items()}
    def unchanged_ls(fun,x,**kwargs):
        return SimpleNamespace(x=x,success=False,message='test retained failed seed',nfev=1)
    def unchanged_sqp(fun,x,**kwargs):
        assert min(kwargs['constraints'][1]['fun'](x))<0
        return SimpleNamespace(x=x,success=False,message='test no mutation',nit=1)
    monkeypatch.setattr(joint,'least_squares',unchanged_ls)
    monkeypatch.setattr(joint,'minimize',unchanged_sqp)
    _,r=joint.fit_joint(road,a,b,raw,raw,seed['shape_parameters'],
        initial_coefficients=seed['joint_coefficients'],fair_world=True,max_iterations=1,
        source_tails=[dict(role='predecessor',source_lane_id='fixture',curves=curves)])
    assert r['status']=='REJECTED'
    assert max(r['original_tail_reverse_errors_m'].values())>.35
    assert r['original_tail_final_max_m']==.35
    assert max(v['max_m'] for f in r['source'].values() for v in f.values())<.01
