from types import SimpleNamespace

import numpy as np
import pytest

from mapforge.ops.source_coefficient_stencil import SourceCoefficientStencil, ParentCoefficientStencils
from mapforge.ops.source_constraint_envelope import SourceConstraintEnvelope
from mapforge.ops.whole_source_constraints import WholeSourceConstraintSystem
from mapforge.ops.port_dependencies import LanePort
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from tests.test_movable_source_layout import block,constant_coefficients,parent


@pytest.mark.parametrize('k',[0.,.0001,-.0001])
def test_every_affine_witness_column_matches_fresh_envelope_across_active_switches(k):
    raw,_=block();m=ArcSourceBoundaryBlock(raw,curvature=k)
    x=constant_coefficients(m);layout=SourceConstraintEnvelope(m,x)
    fast=SourceCoefficientStencil(layout,dict(model=m,coefficients=x))
    assert fast.accounting['all_original_rows_accounted']
    for j in range(m.nvar):
        for delta in (-.5,-1e-5,1e-5,.5):
            v=x.copy();v[j]+=delta
            full=layout.evaluate(dict(model=m,coefficients=v))
            np.testing.assert_allclose(fast.values(j,delta),
                np.r_[full['equalities'],full['inequalities']],atol=2e-12,rtol=0)
    np.testing.assert_array_equal(x,constant_coefficients(m))


def actual_parent_system(monkeypatch):
    p=parent(monkeypatch)
    end=LanePort('10','end',-1)
    ports=SimpleNamespace(parents={'10':p},slices={'10':slice(0,p.nvar)},
        graph=SimpleNamespace(connections={'100':(end,end)}))
    class Geometry:
        def __init__(self):
            self.ports=ports;self.initial=np.r_[p.initial,.5];self.calls=0
            self.turn_slices={'100':slice(p.nvar,p.nvar+1)}
            self.kernels={'100':dict(bounds=np.array([[0.,1.]]))}
        def evaluate_turn(self,cid,local,parents):
            self.calls+=1
            frame=p.frame(parents['10'],end,True)
            v=frame['center']['y']+frame['center']['heading']+frame['center']['curvature']
            eq=np.array([v+local[0]**2]);iq=np.array([4-v,3+v])
            return dict(equalities=eq,result=([0]*5,None,None,{},eq,iq,0,0,0,{}, {},0))
        def evaluate(self,v):
            parents={'10':p.evaluate(v[:p.nvar])}
            return dict(parents=parents,turns={'100':self.evaluate_turn('100',v[p.nvar:],parents)})
    from mapforge.ops import whole_source_constraints as module
    monkeypatch.setattr(module,'turn_inequality_keys',lambda t:('shape-a','shape-b'))
    return WholeSourceConstraintSystem(Geometry())


def test_coefficient_fast_path_uses_exact_basis_dependencies_and_full_unchanged_chart(monkeypatch):
    s=actual_parent_system(monkeypatch);fast=ParentCoefficientStencils(s,s.base)
    p=s.geometry.ports.parents['10'];called=[];omitted=[]
    assert fast.evaluate(1,1e-5) is None  # Axes/cuts/knots must rebuild originals.
    for j in range(p.coefficient_slice.start,p.nvar):
        s.geometry.calls=0
        a=fast.evaluate(j,1e-5)
        (called if s.geometry.calls else omitted).append(j)
        b=s.trial_column(s.base,j,1e-5)
        np.testing.assert_allclose(a['values'],b.values,rtol=0,atol=2e-12)
    assert called and omitted
    # These are genuinely zero turn dependencies, not unchecked zero columns:
    # every source envelope row is still evaluated on each coefficient change.
    for j in omitted:
        a=fast.evaluate(j,.1)
        np.testing.assert_allclose(a['values'],s.trial_column(s.base,j,.1).values,rtol=0,atol=2e-12)
    columns=np.arange(p.coefficient_slice.start,p.nvar)
    a=s.linearize(columns=columns,coefficient_stencils=True)
    b=s.linearize(columns=columns)
    for side in ('right','left'):
        np.testing.assert_allclose(a['sided_matrices'][side].toarray(),
            b['sided_matrices'][side].toarray(),atol=3e-7,rtol=1e-7)
    assert all(d.get('original_certificate_accounting',{}).get('all_original_rows_accounted')
               for d in a['diagnostics'])
