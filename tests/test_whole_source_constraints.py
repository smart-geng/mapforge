from types import SimpleNamespace

import numpy as np
import pytest

from mapforge.ops.port_dependencies import LanePort
from mapforge.ops.whole_source_constraints import WholeSourceConstraintSystem
from mapforge.ops.whole_source_constraints import squared_distance_slack


def synthetic_system(monkeypatch):
    """Dependency/FD fixture; real source certificates tested separately."""
    from mapforge.ops import whole_source_constraints as module
    class Envelope:
        keys=('source-A','source-B');eq_keys=('contact',)
        def __init__(self,model,coefficients):pass
        def evaluate(self,state):
            v=state['coordinate_vector']
            return dict(equalities=np.array([v[1]**2+v[3]*20+v[6]-v[7]]),
                inequalities=np.array([1-v[2]**2,min(v[6],v[7])]),accounting={})
    monkeypatch.setattr(module,'SourceConstraintEnvelope',Envelope)
    monkeypatch.setattr(module,'turn_inequality_keys',lambda t:('synthetic-shape','synthetic-source'))
    class Parent:
        def __init__(self,rid,cp):
            self.road=rid;self.nvar=8
            self.initial=np.array([.1,.2,.03,.001,0,0,1,2.])
            self.port_features={LanePort(rid,cp,-1):('l','r')}
            self.calls=0
        def evaluate(self,v):
            self.calls+=1
            return dict(coordinate_vector=v.copy(),model=SimpleNamespace(road=self.road),coefficients=v[6:].copy())
        def evaluate_from_snapshot(self,v,state):return self.evaluate(v)
    ps={'10':Parent('10','end'),'11':Parent('11','start')}
    pairs={'100':(LanePort('10','end',-1),LanePort('11','start',-1)),
           '101':(LanePort('10','end',-1),LanePort('10','end',-2)),
           '102':(LanePort('11','start',-1),LanePort('11','start',-2))}
    ports=SimpleNamespace(parents=ps,slices={'10':slice(0,8),'11':slice(8,16)},graph=SimpleNamespace(connections=pairs))
    class Geometry:
        def __init__(self):
            self.ports=ports;self.initial=np.r_[ps['10'].initial,ps['11'].initial,.5,.5,.5]
            self.turn_slices={cid:slice(16+i,17+i) for i,cid in enumerate(pairs)}
            self.kernels={cid:dict(bounds=np.array([[0.,1.]])) for cid in pairs};self.calls=[]
        def evaluate_turn(self,cid,local,parents):
            self.calls.append(cid)
            u,w=[parents[p.road]['coordinate_vector'] for p in pairs[cid]]
            # Source-partition dependency is NOT only the contact position:
            # chart/cut terms and edge coefficients are independently present.
            f=np.sin(u.sum())+w[2]*w[6]+local[0]**2
            eq=np.array([f]);iq=np.array([3-f,min(u[6],w[6])+local[0]])
            result=([0]*5,None,None,{'center':{'source_to_target':[]}},eq,iq,0,0,0,{}, {},0)
            return dict(result=result,equalities=eq,source_support={'whole_source':True})
        def evaluate(self,v):
            parents={rid:p.evaluate(v[ports.slices[rid]]) for rid,p in ps.items()}
            turns={cid:self.evaluate_turn(cid,v[sl],parents) for cid,sl in self.turn_slices.items()}
            return dict(parents=parents,turns=turns)
    return WholeSourceConstraintSystem(Geometry())


def test_every_local_column_equals_independent_full_evaluation_and_leaves_snapshot_unchanged(monkeypatch):
    s=synthetic_system(monkeypatch);before=s.base.values.copy();vector=s.base.vector.copy()
    for col in s.free_columns:
        s.geometry.calls.clear()
        local=s.trial_column(s.base,int(col),s.steps[col])
        kind,rid=s.column_owner[col]
        expected=set(s.dependents[rid]) if kind=='parent' else {rid}
        assert set(s.geometry.calls)==expected
        full=s.evaluate(local.vector)
        np.testing.assert_array_equal(local.values,full.values)
    assert np.array_equal(before,s.base.values) and np.array_equal(vector,s.base.vector)


def test_full_sparse_stencil_matches_whole_direction_and_keeps_fixed_slots_out(monkeypatch):
    s=synthetic_system(monkeypatch);r=s.linearize();matrix=r['matrix']
    assert r['all_free_columns_evaluated'] and matrix.shape==(15,17)
    assert s.fixed_columns==[4,13]
    direction=np.zeros(19);direction[[1,10,16]]=[.2,.3,.4]
    h=1e-7;numeric=(s.evaluate(s.initial+h*direction).values-s.base.values)/h
    np.testing.assert_allclose(matrix@direction[r['columns']],numeric,atol=1e-6,rtol=1e-5)
    assert not r['smooth_jacobian_certified'] and not r['export_allowed']


def test_partial_jacobian_has_no_fabricated_unchecked_zero_columns(monkeypatch):
    s=synthetic_system(monkeypatch);r=s.linearize(columns=[1,16])
    assert r['matrix'].shape==(len(s.base.values),2)
    assert not r['all_free_columns_evaluated'] and list(r['columns'])==[1,16]
    with pytest.raises(ValueError,match='unique free'):s.linearize(columns=[4])
    with pytest.raises(ValueError,match='unique free'):s.linearize(columns=[1,1])
    with pytest.raises(ValueError,match='fixed'):s.trial_column(s.base,4,1e-5)


def test_boundary_one_sided_and_nonsmooth_ties_are_reported_not_silently_smooth(monkeypatch):
    s=synthetic_system(monkeypatch);v=s.initial.copy();v[16]=0;v[7]=v[6]
    base=s.evaluate(v);r=s.linearize(base,columns=[6,16])
    tie,bound=r['diagnostics']
    assert tie['left_right_disagreement_rows'] and tie['nonsmooth_or_numerically_unresolved']
    assert bound['method']=='one-sided-domain-stencil' and 'minus' in bound['domain_rejections']
    assert not r['smooth_jacobian_certified']


def test_incomplete_whole_state_and_nonfinite_inputs_are_rejected(monkeypatch):
    s=synthetic_system(monkeypatch)
    with pytest.raises(ValueError):s.evaluate(s.initial[:-1])
    bad=s.initial.copy();bad[6]=np.nan
    with pytest.raises(ValueError):s.evaluate(bad)
    bad=s.initial.copy();bad[16]=-1
    with pytest.raises(ValueError,match='bounds'):s.evaluate(bad)


def test_equivalent_squared_distance_does_not_change_gate_or_active_derivative():
    b=.349;errors=np.r_[0.,1e-9,.1,b-.001,b,b+.001,1.,100.]
    old=b-errors;new=squared_distance_slack(old,b)
    assert np.array_equal(old>=0,new>=0)
    np.testing.assert_allclose(new,(b*b-errors*errors)/(2*b))
    h=1e-7
    derivative=(squared_distance_slack(-h,b)-squared_distance_slack(h,b))/(2*h)
    assert derivative==pytest.approx(-1.,abs=1e-9)
    # A zero-distance norm used to have opposite left/right derivatives;
    # its equivalent squared constraint is flat at that inactive point.
    right=(squared_distance_slack(b-h,b)-squared_distance_slack(b,b))/h
    left=(squared_distance_slack(b,b)-squared_distance_slack(b-h,b))/h
    assert abs(right-left)<1e-6
    with pytest.raises(ValueError):squared_distance_slack(.4,.349)
